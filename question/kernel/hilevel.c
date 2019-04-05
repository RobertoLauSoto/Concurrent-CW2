/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* We assume there will be 2 user processes, stemming from the 2 user programs, 
 * and so can 
 * 
 * - allocate a fixed-size process table (of PCBs), and then maintain an index 
 *   into it to keep track of the currently executing process, and
 * - employ a fixed-case of round-robin scheduling: no more processes can be 
 *   created, and neither can be terminated, so assume both are always ready
 *   to execute.
 */

#define numProcesses 16

uint32_t currentNumProcesses = 0;

pcb_t pcb[ numProcesses ]; pcb_t* current = NULL; int current_index = 0;; extern uint32_t tos_P;



extern void     main_P3();
extern void     main_P4();
extern void     main_P5();
extern void     main_console();

void (*main_ptr[numProcesses])= {&main_P3, &main_P4, &main_P5};
int priorities[] = {5, 3, 1};

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  prev->status = STATUS_READY;             // update   execution status  of previous P_ 
  next->status = STATUS_EXECUTING;         // update   execution status  of next     P_

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    current = next;                             // update   executing index   to P_{next}

  return;
}



int getNextAvailablePid() {
  int nextAvailableProcessPid;
  for ( int i=1; i < numProcesses; i++){
    if (pcb[i].alive == 0){
      nextAvailableProcessPid = i;
      break;
    }
  }
  return nextAvailableProcessPid;
}

uint32_t getNextProcessRR() {
  return (current_index +1) % currentNumProcesses;
}

void schedule_RR( ctx_t* ctx ) {
  uint32_t next_index = getNextProcessRR();
  for (int i = 0; i < numProcesses; i++) {
    if ((current_index != next_index) && (pcb[next_index].status != STATUS_TERMINATED) && (pcb[next_index].alive == 1) ) {
      dispatch(ctx, &pcb[current_index], &pcb[next_index]);
      current_index=next_index;
      break;
    }

    else {
      next_index = (next_index + 1) % (currentNumProcesses + 1);
    }
  }
}

int getPriority(int index) {
  return pcb[index].curr_prty + pcb[index].init_prty;
}

int getNextProcessPriority(){
  int highest_prty_index = 0;
  for (int i = 0; i < currentNumProcesses; i++){
    if (pcb[i].alive){
      if(getPriority(i)> getPriority(highest_prty_index)){
        highest_prty_index = i;
      }
    }
  }
  return highest_prty_index;
}

void increasePrioritiesExceptAt(int index) {
  for(int i = 0; i < currentNumProcesses; i++){
    if(i!=index){
      pcb[i].curr_prty++;
    }
  }
}

void schedule_priority( ctx_t* ctx ) {
  int next_process_index = getNextProcessPriority();
  if    (current_index != next_process_index) {
    dispatch(ctx, &pcb[current_index], &pcb[next_process_index]);
    
    
    pcb[ next_process_index ].curr_prty  = 0;   // update   priority  value   of P_next to initial priority value

    pcb[ current_index].status = STATUS_READY;
    pcb[ next_process_index ].status = STATUS_EXECUTING;

    current_index=next_process_index;
  }
  increasePrioritiesExceptAt(current_index);   // increase priorities of all processes except the one with highest priority (i.e the P_next)
}

void schedule( ctx_t* ctx) {
  schedule_priority(ctx);
}

void hilevel_handler_rst( ctx_t* ctx              ) { 
  /* Initialise two PCBs, representing user processes stemming from execution 
   * of two user programs.  Note in each case that
   *    
   * - the CPSR value of 0x50 means the processor is switched into USR mode, 
   *   with IRQ interrupts enabled, and
   * - the PC and SP values matche the entry point and top of stack. 
   */
  
  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();

  // for (uint32_t i; i<numProcesses; i++) {
  //   memset( &pcb[ i ], 0, sizeof(pcb_t) );
  //   pcb[ i ].pid = i + 3;
  //   pcb[ i ].status = STATUS_CREATED;
  //   pcb[ i ].ctx.cpsr = 0x50;
  //   pcb[ i ].ctx.pc = (uint32_t)(main_ptr[i]);
  //   pcb[ i ].ctx.sp = (uint32_t)( &tos_P  )+0x00001000*i;
  //   pcb[ i ].init_prty = (uint32_t) (priorities[i]);
  //   pcb[ i ].curr_prty = (uint32_t) (priorities[i]);
  //   pcb[ i ].tos = (uint32_t)( &tos_P  )+0x00001000*i;
  // }

  memset( &pcb[ 0 ], 0, sizeof(pcb_t) );
  pcb[ 0 ].pid = 0;
  pcb[ 0 ].status = STATUS_CREATED;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc = (uint32_t)(&main_console);
  pcb[ 0 ].ctx.sp = (uint32_t)( &tos_P  )+0x00001000;
  pcb[ 0 ].tos = (uint32_t)( &tos_P  )+0x00001000;
  pcb[ 0 ].init_prty = 1;
  pcb[ 0 ].curr_prty = 0;
  pcb[ 0 ].alive = 1;
  currentNumProcesses++;

  /* Once the PCBs are initialised, we arbitrarily select the one in the 0-th 
   * PCB to be executed: there is no need to preserve the execution context, 
   * since it is is invalid on reset (i.e., no process will previously have
   * been executing).
   */

  dispatch( ctx, NULL, &pcb[ 0 ] );

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) { 
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction, 
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      schedule( ctx );

      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );  
      char*  x = ( char* )( ctx->gpr[ 1 ] );  
      int    n = ( int   )( ctx->gpr[ 2 ] ); 

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }
      
      ctx->gpr[ 0 ] = n;

      break;
    }

    case 0x03 : { // 0x03 => fork
      uint32_t newId=getNextAvailablePid();
      currentNumProcesses++;

      memset( &pcb[ newId ], 0, sizeof(pcb_t) );
      memcpy(&pcb[ newId ].ctx, ctx, sizeof(ctx_t));

      // current_index = newId;
      pcb[ newId ].pid = newId;
      pcb[ newId ].status = STATUS_CREATED;
      pcb[ newId ].init_prty = priorities[newId % 3];
      pcb[ newId ].curr_prty = 0;
      pcb[ newId ].tos = (uint32_t)( &tos_P - 0x00001000*newId);
      pcb[ newId ].alive = 1;
      //pcb[ newId ].ctx.pc = ctx->pc;
      
      uint32_t offset = pcb[newId-1].tos-ctx->sp;

      pcb[ newId ].ctx.sp = (uint32_t) pcb[ newId ].tos-offset;

      // memcpy((void *) &pcb[ newId ].tos-offset, (const void *) &pcb[ newId-1 ].tos-offset, offset);
      
      pcb[ newId ].ctx.gpr[0] = 0;
      ctx->gpr[0] = newId;
      
      break;
    }

    case 0x04 : { // 0x04 => exit
      int x = (int) (ctx->gpr[0]);
      current->status = x;
      current->alive = 0;
      current->init_prty = 0;
      current->curr_prty = 0;
      // pcb[ current_index ].status = STATUS_TERMINATED;
      // pcb[ current_index ].alive = 0; 
      // pcb[ current_index ].init_prty = 0;
      // pcb[ current_index ].curr_prty = 0;
      currentNumProcesses--;
      schedule( ctx );
      break;
    }

    case 0x05 : { // 0x05 => exec
      const void* x = (const void* )( ctx->gpr[0]);
      ctx->pc = (uint32_t) ( x );
      ctx->sp = pcb[ current -> pid ].tos;
      ctx->cpsr=0x50;
      break;
    }

    case 0x06 : { //0x06 => kill (int pid, int x)
      int pid = (int)(ctx->gpr[0]);
      int x = (int) (ctx->gpr[1]);

      pcb[ pid ].status = STATUS_TERMINATED;
      if (pcb[pid].alive){
        pcb[ pid ].alive = 0;
        currentNumProcesses--;
      }
      pcb[ pid ].ctx.gpr[ 0 ] = x;
      
      schedule( ctx );
      break;
    }

    case 0x07 : {// 0x07 => nice
      int pid = (int)(ctx->gpr[0]);
      int x = (int) (ctx->gpr[1]);

      pcb[ pid ].init_prty = x;
      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;
  PL011_putc( UART0, '\n', 2 );


  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    schedule( ctx );
    PL011_putc( UART0, 'T', true );
    TIMER0->Timer1IntClr=0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

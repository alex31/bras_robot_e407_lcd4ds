/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

.text
.syntax unified
.thumb

/*
 * Imports the Cortex-Mx configuration headers.
 */
#define _FROM_ASM_
#include "chconf.h"
#include "chcore.h"

#define CONTEXT_OFFSET  12
#define SCB_ICSR        0xE000ED04
#define ICSR_PENDSVSET  0x10000000

#ifndef CORTEX_USE_FPU
#error  assembler needs to know CORTEX_USE_FPU
#endif

#ifndef CH_DBG_SYSTEM_STATE_CHECK
#error  assembler needs to know CH_DBG_SYSTEM_STATE_CHECK
#endif

#ifndef CORTEX_SIMPLIFIED_PRIORITY
#error  assembler needs to know CORTEX_SIMPLIFIED_PRIORITY
#endif

#ifndef CRT0_INIT_STACKS
#error assembler needs to know CRT0_INIT_STACKS
#endif

/*
 * Control special register initialization value.
 * The system is setup to run in privileged mode using the PSP
 * stack (dual stack mode).
 */
#if !defined(CRT0_CONTROL_INIT)
#define CRT0_CONTROL_INIT           0x00000002
#endif

/*
 * Performs a context switch between two threads.
 */
.global  _port_switch
.type _port_switch, %function
.thumb_func
_port_switch:
                push    {r4-r11,lr}
#if CORTEX_USE_FPU
                vpush   {s16-s31}
#endif
                str     sp, [r1, #CONTEXT_OFFSET]                          
                ldr     sp, [r0, #CONTEXT_OFFSET]
#if CORTEX_USE_FPU
                vpop    {s16-s31}
#endif
                pop     {r4-r11,pc}

/*
 * Start a thread by invoking its work function.
 * If the work function returns @p chThdExit() is automatically invoked.
 */
.global  _port_thread_start
.type _port_thread_start, %function
.thumb_func
_port_thread_start:
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
#if CORTEX_SIMPLIFIED_PRIORITY
                cpsie   i
#else
                mov     r3, #CORTEX_BASEPRI_DISABLED
                msr     BASEPRI, r3
#endif
                mov     r0, r5
                blx     r4
                b       chThdExit

/*
 * Post-IRQ switch code.
 * Exception handlers return here for context switching.
 */
.global  _port_switch_from_isr
.type _port_switch_from_isr, %function
.thumb_func
_port_switch_from_isr:
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_lock
#endif
                bl      chSchDoReschedule
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
.global  _port_exit_from_isr
.type _port_exit_from_isr, %function
.thumb_func
_port_exit_from_isr:
#if CORTEX_SIMPLIFIED_PRIORITY
                movw    r3, #:lower16:SCB_ICSR
                movt    r3, #:upper16:SCB_ICSR
                mov     r2, #ICSR_PENDSVSET
                str     r2, [r3, #0]
                cpsie   i
1:              b       1b
#else
                svc     #0
#endif

/*
 * Reset handler
 */
.global ResetHandler
.type ResetHandler, %function
.thumb_func
ResetHandler:
               /* The processor enters Thread mode when it comes out of reset.
                  Out of reset, all code uses the main stack.*/
               cpsid   i
               
               /* Process Stack initialization, it is allocated starting from the
                  symbol __process_stack_end__ and its lower limit is the symbol
                  __process_stack_base__.*/
#if CRT0_INIT_STACKS
               bl _init_process_stack
#endif
               movw    r0, #:lower16:__process_stack_end__
               movt    r0, #:upper16:__process_stack_end__
               msr     PSP, r0
               
               mov     r1, #CRT0_CONTROL_INIT
               msr     CONTROL, r1
               isb
               
               /* Running on process stack.*/
               b _start

/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author       Notes
 * 2010-01-25   Bernard      first version
 * 2012-06-01   aozima       set pendsv priority to 0xFF.
 * 2012-08-17   aozima       fixed bug: store r8 - r11.
 * 2013-02-20   aozima       port to gcc.
 * 2013-06-18   aozima       add restore MSP feature.
 * 2013-11-04   bright       fixed hardfault bug for gcc.
 * 2019-03-31   xuzhuoyi     port to Cortex-M23.
 */

/*************************************************
File name: pendsv.S
Description: PendSV interrupt handler
Others: take RT-Thread v4.0.2/libcpu/arm/cortex-m23/context_gcc.S for references
        https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2022-02-28
Author: AIIT XUOS Lab
*************************************************/

#include <xsconfig.h>

.cpu cortex-m4
.syntax unified
.thumb
.text

.equ    SCB_VTOR,           0xE000ED08
.equ    NVIC_INT_CTRL,      0xE000ED04
.equ    NVIC_SYSPRI2,       0xE000ED20
.equ    NVIC_PENDSV_PRI,    0xFFFF0000
.equ    NVIC_PENDSVSET,     0x10000000

/* R0 --> switch from thread stack
 * R1 --> switch to thread stack
 * psr, pc, LR, R12, R3, R2, R1, R0 are pushed into [from] stack
 */
    .global PendSV_Handler_NS
    .type PendSV_Handler_NS, %function
PendSV_Handler_NS:
    /* disable interrupt to protect context switch */
    MRS r2, PRIMASK
    CPSID   I

    /* get KtaskSwitchInterruptFlag */
    LDR     r0, =KtaskSwitchInterruptFlag           /* r0 = &KtaskSwitchInterruptFlag */
    LDR     r1, [r0]                                /* r1 = *r1 */
    CMP     r1, #0x00                               /* compare r1 == 0x00 */
    BNE     schedule                                

    MSR     PRIMASK, r2                             /* if r1 == 0x00, do msr PRIMASK, r2 */
    BX      lr                                      /* if r1 == 0x00, do bx lr */

schedule:
    PUSH    {r2}                                    /* store interrupt state */

    /* clear KtaskSwitchInterruptFlag to 0 */
    MOV     r1, #0x00                               /* r1 = 0x00 */
    STR     r1, [r0]                                /* *r0 = r1 */

    /* skip register save at the first time */
    LDR     r0, =InterruptFromKtask                 /* r0 = &InterruptFromKtask */
    LDR     r1, [r0]                                /* r1 = *r0 */
    CBZ     r1, switch_to_thread                    /* if r1 == 0, goto switch_to_thread */

    /* Whether TrustZone thread stack exists */
    LDR     r1,  =xz_trustzone_current_context      /* r1 = &xz_trustzone_current_context */
    LDR     r1, [r1]                                /* r1 = *r1 */
    CBZ     r1, contex_ns_store                     /* if r1 == 0, goto contex_ns_store */

    /*call TrustZone fun, Save TrustZone stack */
    STMFD   sp!, {r0-r1, lr}                        /* push register */
    MOV     r0, r1                                  /* r0 = rt_secure_current_context */
    BL xz_trustzone_context_store                   /* call TrustZone store fun */
    LDMFD   sp!, {r0-r1, lr}                        /* pop register */

    /* check break from TrustZone */
    MOV     r2, lr                                  /* r2 = lr */
    TST     r2, #0x40                               /* if EXC_RETURN[6] is 1, TrustZone stack was used */
    BEQ     contex_ns_store                         /* if r2 & 0x40 == 0, goto contex_ns_store */

    /* push PSPLIM CONTROL PSP LR current_context to stack */
    MRS     r3, psplim                              /* r3 = psplim */
    MRS     r4, control                             /* r4 = control */
    MRS     r5, psp                                 /* r5 = psp */
    STMFD   r5!, {r1-r4}                            /* push to thread stack */

    /* update from thread stack pointer */
    LDR     r0, [r0]                                /* r0 = rt_thread_switch_interrupt_flag */
    STR     r5, [r0]                                /* *r0 = r5 */
    b switch_to_thread                              /* goto switch_to_thread */

contex_ns_store:

    MRS     r1, psp                                 /* get from thread stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    TST     lr, #0x10                               /* if(!EXC_RETURN[4]) */
    IT      EQ
    VSTMDBEQ  r1!, {d8 - d15}                       /* push FPU register s16~s31 */
#endif

    STMFD   r1!, {r4 - r11}                         /* push r4 - r11 register */

    LDR     r2,  =xz_trustzone_current_context      /* r2 = &xz_trustzone_current_context */
    LDR     r2, [r2]                                /* r2 = *r2 */
    MOV     r3, lr                                  /* r3 = lr */
    MRS     r4, psplim                              /* r4 = psplim */
    MRS     r5, control                             /* r5 = control */
    STMFD   r1!, {r2-r5}                            /* push to thread stack */

    LDR     r0, [r0]
    STR     r1, [r0]                                /* update from thread stack pointer */

switch_to_thread:
    LDR     r1, =InterruptToKtask
    LDR     r1, [r1]
    LDR     r1, [r1]                                /* load thread stack pointer */

    /* update current TrustZone context */
    LDMFD   r1!, {r2-r5}                            /* pop thread stack */
    MSR     psplim, r4                              /* psplim = r4 */
    MSR     control, r5                             /* control = r5 */
    MOV     lr, r3                                  /* lr = r3 */
    LDR     r6,  =xz_trustzone_current_context      /* r6 = &xz_trustzone_current_context */
    STR     r2, [r6]                                /* *r6 = r2 */
    MOV     r0, r2                                  /* r0 = r2 */

    /* Whether TrustZone thread stack exists */
    CBZ     r0, contex_ns_load                      /* if r0 == 0, goto contex_ns_load */
    PUSH    {r1, r3}                                /* push lr, thread_stack */
    BL xz_trustzone_context_load                    /* call TrustZone load fun */
    POP     {r1, r3}                                /* pop lr, thread_stack */
    MOV     lr, r3                                  /* lr = r1 */
    TST     r3, #0x40                               /* if EXC_RETURN[6] is 1, TrustZone stack was used */
    BEQ     contex_ns_load                          /* if r1 & 0x40 == 0, goto contex_ns_load */
    B pendsv_exit

contex_ns_load:
    LDMFD   r1!, {r4 - r11}                         /* pop r4 - r11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    TST     lr, #0x10                               /* if(!EXC_RETURN[4]) */
    IT      EQ
    VLDMIAEQ  r1!, {d8 - d15}                       /* pop FPU register s16~s31 */
#endif

pendsv_exit:
    MSR     psp, r1                                 /* update stack pointer */
    /* restore interrupt */
    POP    {r2}
    MSR     PRIMASK, r2

    BX      lr
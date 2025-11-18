/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */

#include <xs_base.h>
#include <xs_ktask.h>
#include <hal_data.h>
#include "arch_interrupt.h"

#define SCB_VTOR            "0xE000ED08"        /* Vector Table Offset Register */
#define NVIC_INT_CTRL       "0xE000ED04"        /* interrupt control state register */
#define NVIC_SYSPRI2        "0xE000ED20"        /* system priority register (2) */
#define NVIC_PENDSV_PRI     "0xFFFF0000"        /* PendSV and SysTick priority value (lowest) */
#define NVIC_PENDSVSET      "0x10000000"        /* value to trigger PendSV exception */

void __attribute__((naked)) HwInterruptcontextSwitch(x_ubase from, x_ubase to, struct TaskDescriptor *to_task, void *context)
{
    __asm__ volatile("LDR r3, =InterruptFromKtask");
    __asm__ volatile("STR r0, [r3]");
    __asm__ volatile("LDR r3, =InterruptToKtask");
    __asm__ volatile("STR r1, [r3]");
    __asm__ volatile("LDR r3, =InterruptToKtaskDescriptor");
    __asm__ volatile("STR r2, [r3]");

    __asm__ volatile("LDR r2, =KtaskSwitchInterruptFlag");
    __asm__ volatile("LDR r3, [r2]");
    __asm__ volatile("CMP r3, #1");

    __asm__ volatile("BEQ Arm32SwitchReswitch");

    __asm__ volatile("MOVS r3, #1");
    __asm__ volatile("STR r3, [r2]");

    __asm__ volatile("B Arm32SwitchReswitch");
}

void __attribute__((naked)) Arm32SwitchReswitch()
{
    __asm__ volatile("LDR r2, =InterruptToKtask");
    __asm__ volatile("STR r1, [r2]");

    __asm__ volatile("LDR r0, =" NVIC_INT_CTRL); /* trigger the PendSV exception (causes context switch) */
    __asm__ volatile("LDR r1, =" NVIC_PENDSVSET);
    __asm__ volatile("STR r1, [r0]");
    __asm__ volatile("BX  lr");
}

void __attribute__((naked)) SwitchKtaskContext(x_ubase from, x_ubase to, struct TaskDescriptor *to_task)
{
    __asm__ volatile("B HwInterruptcontextSwitch");
}

void SwitchKtaskContextTo(x_ubase to, struct TaskDescriptor *to_task)
{
    __asm__ volatile("LDR r2, =InterruptToKtask");
    __asm__ volatile("STR r0, [r2]");

    __asm__ volatile("LDR r2, =InterruptToKtaskDescriptor");
    __asm__ volatile("STR r1, [r2]");

    __asm__ volatile("LDR r1, =InterruptFromKtask");
    __asm__ volatile("MOV r0, #0x0");
    __asm__ volatile("STR r0, [r1]");

    __asm__ volatile("LDR r1, =KtaskSwitchInterruptFlag");
    __asm__ volatile("MOV r0, #1");
    __asm__ volatile("STR r0, [r1]");

    __asm__ volatile("LDR r0, =" NVIC_SYSPRI2);
    __asm__ volatile("LDR r1, =" NVIC_PENDSV_PRI);

    __asm__ volatile("LDR r2, [r0,#0x00]");
    __asm__ volatile("ORR r1,r1,r2");
    __asm__ volatile("STR r1, [r0]");

    __asm__ volatile("LDR r0, =" NVIC_INT_CTRL);
    __asm__ volatile("LDR r1, =" NVIC_PENDSVSET);
    __asm__ volatile("STR r1, [r0]");

    __asm__ volatile("LDR r0, =" SCB_VTOR);
    __asm__ volatile("LDR r0, [r0]");
    __asm__ volatile("LDR r0, [r0]");
    __asm__ volatile("NOP");
    __asm__ volatile("MSR msp, r0");

    __asm__ volatile("CPSIE   F");
    __asm__ volatile("CPSIE   I");

    __asm__ volatile("DSB");
    __asm__ volatile("ISB");
}

void __attribute__((naked)) Arm32SwitchGetSpDone()
{
    __asm__ volatile("MRS     r3, primask");

    __asm__ volatile("SUB   r0, r0, #0x24");
    __asm__ volatile("STMIA   r0!, {r3 - r7}");
    __asm__ volatile("MOV r3, r8");
    __asm__ volatile("MOV r4, r9");
    __asm__ volatile("MOV r5, r10");
    __asm__ volatile("MOV r6, r11");
    __asm__ volatile("STMIA   r0!, {r3 - r6}");
    __asm__ volatile("SUB   r0, r0, #0x24");

    __asm__ volatile("SUB   r0, r0, #0x4");
    __asm__ volatile("MOV   r0, lr");

    __asm__ volatile("MOV     r1, lr");
    __asm__ volatile("MOV     r2, #0x04");
    __asm__ volatile("TST     r1, r2");

    __asm__ volatile("BEQ     Arm32SwitchUpdateMsp");
    __asm__ volatile("MSR     psp, r0");
    __asm__ volatile("B       Arm32SwitchUpdateDone");
    __asm__ volatile("B       Arm32SwitchUpdateMsp");
}

void __attribute__((naked)) Arm32SwitchUpdateMsp()
{
    __asm__ volatile("MSR     msp, r0");
    __asm__ volatile("B       Arm32SwitchUpdateDone");
}

void __attribute__((naked)) Arm32SwitchUpdateDone()
{
    __asm__ volatile("PUSH    {LR}");
    __asm__ volatile("BL      HwHardFaultException");

    __asm__ volatile("POP     {R1}");
    __asm__ volatile("MOV     lr, r1");

    __asm__ volatile("MOV     r1, lr");
    __asm__ volatile("MOV     r2, #0x04");
    __asm__ volatile("ORR     r1, r2");
    __asm__ volatile("MOV     lr, r1");

    __asm__ volatile("BX      lr");
}

void __attribute__((naked)) MemFaultHandler()
{
    __asm__ volatile("MRS     r0, msp");

    __asm__ volatile("TST     lr, #0x04");
    __asm__ volatile("MOV     r1, lr");
    __asm__ volatile("MOV     r2, #0x04");
    __asm__ volatile("TST     r1, r2");

    __asm__ volatile("BEQ     Arm32Switch1");
    __asm__ volatile("MRS     r0, psp");
    __asm__ volatile("B       Arm32Switch1");
}

void __attribute__((naked)) Arm32Switch1()
{
    __asm__ volatile("MRS     r3, primask");

    __asm__ volatile("SUB   r0, r0, #0x24");
    __asm__ volatile("STMIA   r0!, {r3 - r7}");
    __asm__ volatile("MOV r3, r8");
    __asm__ volatile("MOV r4, r9");
    __asm__ volatile("MOV r5, r10");
    __asm__ volatile("MOV r6, r11");
    __asm__ volatile("STMIA   r0!, {r3 - r6}");
    __asm__ volatile("SUB   r0, r0, #0x24");

    __asm__ volatile("SUB   r0, r0, #0x4");
    __asm__ volatile("MOV   r0, lr");

    __asm__ volatile("PUSH    {LR}");
    __asm__ volatile("BL      MemFaultHandle");

    __asm__ volatile("POP     {R5}");
    __asm__ volatile("MOV     lr, r5");

    __asm__ volatile("MOV     r5, lr");
    __asm__ volatile("MOV     r6, #0x04");
    __asm__ volatile("ORR     r5, r6");
    __asm__ volatile("MOV     lr, r5");

    __asm__ volatile("BX      lr");
}
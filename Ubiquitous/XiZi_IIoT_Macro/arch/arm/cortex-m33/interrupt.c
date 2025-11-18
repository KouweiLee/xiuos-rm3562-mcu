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

/**
* @file interrupt.c
* @brief support arm cortex-m23 interrupt function
* @version 1.1
* @author AIIT XUOS Lab
* @date 2022-02-24
*/

#include <xizi.h>
#include "arch_interrupt.h"

x_base __attribute__((naked)) DisableLocalInterrupt()
{
    __asm__ volatile ("MRS     r0, PRIMASK");
    __asm__ volatile ("CPSID   I");
    __asm__ volatile ("DSB");
    __asm__ volatile ("ISB");
    __asm__ volatile ("BX      LR ");
}

void __attribute__((naked)) EnableLocalInterrupt(x_base level)
{
    __asm__ volatile ("MSR     PRIMASK, r0");
    __asm__ volatile ("DSB");
    __asm__ volatile ("ISB");
    __asm__ volatile ("BX      LR");
}

int32 ArchEnableHwIrq(uint32 irq_num)
{
    R_BSP_IrqEnable(irq_num);

    return EOK;
}

int32 ArchDisableHwIrq(uint32 irq_num)
{
    R_BSP_IrqDisable(irq_num);
    
    return EOK;
}

extern void KTaskOsAssignAfterIrq(void *context);

void IsrEntry()
{
    uint32 ipsr;

    __asm__ volatile("MRS %0, IPSR" : "=r"(ipsr));

    isrManager.done->incCounter();
    isrManager.done->handleIrq(ipsr);
    KTaskOsAssignAfterIrq(NONE);
    isrManager.done->decCounter();

}

void UsageFault_Handler()
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

void BusFault_Handler()
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

void NMI_Handler()
{
    while (1)
    {
    }
}

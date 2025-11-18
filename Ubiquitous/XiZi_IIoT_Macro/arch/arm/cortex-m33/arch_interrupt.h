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

#ifndef ARCH_INTERRUPT_H__
#define ARCH_INTERRUPT_H__

#include <xs_base.h>
#include "bsp_api.h"

#define ARCH_MAX_IRQ_NUM (480)

#define ARCH_IRQ_NUM_OFFSET 16

#ifndef LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
#define LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY ((1))
#endif

#ifndef MAX_SYSCALL_INTERRUPT_PRIORITY
#define MAX_SYSCALL_INTERRUPT_PRIORITY (LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - __NVIC_PRIO_BITS))
#endif

#define NVIC_SHPR2_REG            (*((volatile uint32_t *) 0xe000ed1c))
#define NVIC_SHPR3_REG            (*((volatile uint32_t *) 0xe000ed20))
#define MAX_INTERRUPT_PRIORITY    (0UL)
#define MIN_INTERRUPT_PRIORITY    (255UL)
#define NVIC_SVC_PRI              (MAX_INTERRUPT_PRIORITY << 24UL)
#define NVIC_PENDSV_PRI           (MIN_INTERRUPT_PRIORITY << 16UL)
#define NVIC_SYSTICK_PRI          (MIN_INTERRUPT_PRIORITY << 24UL)
#define NVIC_SVC_MASK             (unsigned long) (~(0xffUL << 24UL))
#define NVIC_PENDSV_MASK          (unsigned long) (~(0xffUL << 16UL))
#define NVIC_SYSTICK_MASK         (unsigned long) (~(0xffUL << 24UL))

#define SVC_START_FIRST_TASK 1

int32 ArchEnableHwIrq(uint32 irq_num);
int32 ArchDisableHwIrq(uint32 irq_num);

int32 ArchHwIrqState(void);

#endif

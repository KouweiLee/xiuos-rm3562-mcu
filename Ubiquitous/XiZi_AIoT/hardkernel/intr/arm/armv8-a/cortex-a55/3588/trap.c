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
 * @file trap.c
 * @brief trap interface of hardkernel
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2023.05.06
 */

/*************************************************
File name: trap.c
Description: trap interface of hardkernel
Others:
History:
Author: AIIT XUOS Lab
Modification:
1. first version
*************************************************/
#include <stdint.h>

#include "exception_registers.h"

#include "assert.h"
#include "core.h"
#include "multicores.h"
#include "syscall.h"
#include "task.h"

#include "mmu.h"

extern void dabort_handler(struct trapframe* r);
extern void iabort_handler(struct trapframe* r);

void kernel_abort_handler(struct trapframe* tf)
{
    uint64_t esr = r_esr_el1();
    switch ((esr >> 0x1A) & 0x3F) {
    case 0b100100:
    case 0b100101:
        dabort_handler(tf);
        break;
    case 0b100000:
    case 0b100001:
        iabort_handler(tf);
        break;
    default: {
        uint64_t ec = (esr >> 26) & 0x3f;
        uint64_t iss = esr & 0x1ffffff;
        ERROR("esr:  %016lx %016lx %016lx\n", esr, ec, iss);
        ERROR("elr = %016lx far = %016lx\n", r_elr_el1(), r_far_el1());
        ERROR("Current Task: %s.\n", cur_cpu()->task->name);
        panic("Unimplemented Error Occured.\n");
    }
    }
    panic("Return from abort handler.\n");
}

void kernel_intr_handler(struct trapframe* tf)
{
    panic("Intr at kernel mode should never happen by design.\n");
}

extern void context_switch(struct context**, struct context*);
void syscall_arch_handler(struct trapframe* tf)
{

    uint64_t esr = r_esr_el1();
    uint64_t ec = (esr >> 0x1A) & 0x3F;
    switch (ec) {
    case 0B010101:
        software_irq_dispatch(tf);
        break;
    case 0b100100:
    case 0b100101:
        dabort_handler(tf);
        break;
    case 0b100000:
    case 0b100001:
        iabort_handler(tf);
        break;
    default: {
        ERROR("USYSCALL: unexpected\n");
        ERROR("          esr: %016lx\n", esr);
        ERROR("          elr = %016lx far = %016lx\n", r_elr_el1(), r_far_el1());
        w_esr_el1(0);
        extern void dump_tf(struct trapframe * tf);
        dump_tf(tf);

        uint32_t sctlr = 0;
        SCTLR_R(sctlr);
        DEBUG("SCTLR: %x\n", sctlr);
        uint32_t spsr = 0;
        __asm__ volatile("mrs %0, spsr_el1" : "=r"(spsr)::"memory");
        DEBUG("SPSR: %x\n", spsr);
        uint64_t tcr = 0;
        __asm__ volatile("mrs %0, tcr_el1" : "=r"(tcr)::"memory");
        DEBUG("TCR: %x\n", tcr);
        uint64_t mair = 0;
        __asm__ volatile("mrs %0, mair_el1" : "=r"(mair)::"memory");
        DEBUG("MAIR: %x\n", mair);

        // kill error task
        xizi_enter_kernel();
        assert(cur_cpu()->task != NULL);
        ERROR("Error Task: %s\n", cur_cpu()->task->name);
        sys_exit(cur_cpu()->task);
        context_switch(&cur_cpu()->task->thread_context.context, cur_cpu()->scheduler);
        panic("dabort end should never be reashed.\n");
    }
    }
}
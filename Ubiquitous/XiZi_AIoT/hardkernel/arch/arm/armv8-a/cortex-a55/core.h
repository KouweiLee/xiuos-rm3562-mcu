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
 * @file core.h
 * @brief cortex-a55 core function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024.04.11
 */

/*************************************************
File name: core.h
Description: cortex-a55 core function
Others:
History:
Author: AIIT XUOS Lab
Modification:
*************************************************/

#pragma once

// Interrupt control bits
#define NO_INT 0x80 // disable IRQ.
#define DIS_INT 0xc0 // disable both IRQ and FIQ.

#define MODE_STACK_SIZE 0x4000

//! @name SPSR fields
//@{
#define SPSR_EL1_N (1 << 31) //!< Negative
#define SPSR_EL1_Z (1 << 30) //!< Zero
#define SPSR_EL1_C (1 << 29) //!< Carry
#define SPSR_EL1_V (1 << 28) //!< Overflow
#define SPSR_EL1_SS (1 << 21) //!< Software Step
#define SPSR_EL1_IL (1 << 20) //!< Illegal Exception
#define SPSR_EL1_D (1 << 9) //!< Debug mask
#define SPSR_EL1_A (1 << 8) //!< SError mask
#define SPSR_EL1_I (1 << 7) //!< IRQ mask
#define SPSR_EL1_F (1 << 6) //!< FIQ mask
#define SPSR_EL1_M (1 << 4) //!< Execution state    0=64-bit  1=32-bit
#define SPSR_EL1_MODE (0x7) //!< Current processor mode
//@}

//! @name Interrupt enable bits in SPSR
//@{
#define I_BIT 0x80 //!< When I bit is set, IRQ is disabled
#define F_BIT 0x40 //!< When F bit is set, FIQ is disabled
//@}

// ARM Modes         t indicates selecting  sp_el0 pointer, h indicates selecting  sp_eln pointer
#define SPSR_MODE_MASK 0x0f
#define ARM_MODE_EL0_t 0x00
#define ARM_MODE_EL1_t 0x04
#define ARM_MODE_EL1_h 0x05
#define ARM_MODE_EL2_t 0x08
#define ARM_MODE_EL2_h 0x09
#define ARM_MODE_EL3_t 0x0c
#define ARM_MODE_EL3_h 0x0d

#ifndef __ASSEMBLER__
#include <stdint.h>
#include <string.h>

#include "cortex_a55.h"

#define NR_CPU 8 // maximum number of CPUs

static inline uintptr_t arch_curr_tick()
{
    uint64_t x;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(x));
    return x;
}

__attribute__((always_inline)) static inline uint64_t EL0_mode() // Set ARM mode to EL0
{
    uint64_t val = 0;

    return val;
}

__attribute__((always_inline, optimize("O0"))) static inline void cpu_into_low_power()
{
    WFE();
}

__attribute__((always_inline, optimize("O0"))) static inline void cpu_leave_low_power()
{
    SEV();
}

struct context {
    uint64_t sp;

    /* callee register */
    uint64_t x18;
    uint64_t x19;
    uint64_t x20;
    uint64_t x21;
    uint64_t x22;
    uint64_t x23;
    uint64_t x24;
    uint64_t x25;
    uint64_t x26;
    uint64_t x27;
    uint64_t x28;
    uint64_t x29;
    uint64_t x30;
};

/// @brief init task context, set return address to trap return
/// @param  ctx
extern void task_prepare_enter(void);
__attribute__((__always_inline__)) static inline void arch_init_context(struct context* ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->x30 = (uintptr_t)(task_prepare_enter + 4);
}

struct trapframe {
    uint64_t x0;
    uint64_t x1;
    uint64_t x2;
    uint64_t x3;
    uint64_t x4;
    uint64_t x5;
    uint64_t x6;
    uint64_t x7;
    uint64_t x8;
    uint64_t x9;
    uint64_t x10;
    uint64_t x11;
    uint64_t x12;
    uint64_t x13;
    uint64_t x14;
    uint64_t x15;
    uint64_t x16;
    uint64_t x17;
    uint64_t x18;
    uint64_t x19;
    uint64_t x20;
    uint64_t x21;
    uint64_t x22;
    uint64_t x23;
    uint64_t x24;
    uint64_t x25;
    uint64_t x26;
    uint64_t x27;
    uint64_t x28;
    uint64_t x29;
    uint64_t x30;
    uint64_t pc;
    uint64_t spsr;
    uint64_t sp;
};

/// @brief init task trapframe
/// @param tf
/// @param sp
/// @param pc
__attribute__((__always_inline__)) static inline void arch_init_trapframe(struct trapframe* tf, uintptr_t sp, uintptr_t pc)
{
    memset(tf, 0, sizeof(*tf));
    tf->sp = sp;
    tf->spsr = EL0_mode();
    tf->pc = pc;
}

/// @brief set pc and sp to trapframe
/// @param tf
/// @param sp
/// @param pc
__attribute__((__always_inline__)) static inline void arch_trapframe_set_sp_pc(struct trapframe* tf, uintptr_t sp, uintptr_t pc)
{
    tf->sp = sp;
    tf->pc = pc;
}

/// @brief set params of main(int argc, char** argv) to trapframe (argc, argv)
/// @param tf
/// @param argc
/// @param argv
__attribute__((__always_inline__)) static inline void arch_set_main_params(struct trapframe* tf, int argc, uintptr_t argv)
{
    tf->x0 = (uint64_t)argc;
    tf->x1 = (uint64_t)argv;
}

/// @brief retrieve params to trapframe (up to max number of 6) and pass it to syscall()
/// @param sys_num
/// @param param1
/// @param param2
/// @param param3
/// @param param4
/// @param param5
/// @return
extern int syscall(int sys_num, uintptr_t param1, uintptr_t param2, uintptr_t param3, uintptr_t param4);
__attribute__((__always_inline__)) static inline int arch_syscall(struct trapframe* tf, int* syscall_num)
{
    // call syscall
    *syscall_num = tf->x0;
    return syscall(*syscall_num, tf->x1, tf->x2, tf->x3, tf->x4);
}

/// @brief set return reg to trapframe
/// @param tf
/// @param ret
__attribute__((__always_inline__)) static inline void arch_set_return(struct trapframe* tf, int ret)
{
    tf->x0 = (uint64_t)ret;
}

void cpu_start_secondary(uint8_t cpu_id);
void start_smp_cache_broadcast(int cpu_id);
#endif

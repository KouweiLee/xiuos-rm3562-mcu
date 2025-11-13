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
 * @file exception_registers.h
 * @brief exception registers
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024.05.09
 */

static inline void w_vbar_el1(uint64_t x)
{
    __asm__ volatile("msr vbar_el1, %0" : : "r"(x));
}

static inline uint64_t r_esr_el1()
{
    uint64_t x;
    __asm__ volatile("mrs %0, esr_el1" : "=r"(x));
    return x;
}

static inline void w_esr_el1(uint64_t x)
{
    __asm__ volatile("msr esr_el1, %0" : : "r"(x));
}

static inline uint64_t r_elr_el1()
{
    uint64_t x;
    __asm__ volatile("mrs %0, elr_el1" : "=r"(x));
    return x;
}

static inline uint64_t r_far_el1()
{
    uint64_t x;
    __asm__ volatile("mrs %0, far_el1" : "=r"(x));
    return x;
}

static inline uint64_t daif()
{
    uint64_t x;
    __asm__ volatile("mrs %0, daif" : "=r"(x));
    return x;
}

// enable interrupts(irq)
static inline void intr_on()
{
    __asm__ volatile("msr daifclr, #0xf" ::: "memory");
}

// disable interrupts(irq)
static inline void intr_off()
{
    __asm__ volatile("msr daifset, #0xf" ::: "memory");
}
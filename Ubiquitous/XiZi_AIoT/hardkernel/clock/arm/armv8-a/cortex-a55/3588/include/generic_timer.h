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
#pragma once

#include <stddef.h>
#include <stdint.h>

// armv8 generic timer
static inline uint32_t r_cntp_ctl_el0()
{
    uint32_t x;
    __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(x));
    return x;
}

static inline void w_cntp_ctl_el0(uint32_t x)
{
    __asm__ volatile("msr cntp_ctl_el0, %0" : : "r"(x));
}

static inline uint32_t r_cntp_tval_el0()
{
    uint32_t x;
    __asm__ volatile("mrs %0, cntp_tval_el0" : "=r"(x));
    return x;
}

static inline void w_cntp_tval_el0(uint32_t x)
{
    __asm__ volatile("msr cntp_tval_el0, %0" : : "r"(x));
}

static inline uint64_t r_cntvct_el0()
{
    uint64_t x;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(x));
    return x;
}

static inline uint32_t r_cntfrq_el0()
{
    uint32_t x;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(x));
    return x;
}
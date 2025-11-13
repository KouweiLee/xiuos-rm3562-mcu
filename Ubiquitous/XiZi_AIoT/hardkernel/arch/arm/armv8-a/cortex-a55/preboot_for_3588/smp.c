/*
 * Copyright (c) 2010-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file smp.c
 * @brief start multicore
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024.04.10
 */

/*************************************************
File name: smp.c
Description:
Others:
History:
Author: AIIT XUOS Lab
Modification:
1. No modifications
*************************************************/
#include <stdint.h>

#define PSCI_CPUON 0xc4000003
struct xizi_smccc_res {
    unsigned long a0;
    unsigned long a1;
    unsigned long a2;
    unsigned long a3;
};

extern void _boot_start();
extern void __print();

extern void __xizi_smccc_smc(unsigned long a0, unsigned long a1, unsigned long a2,
    unsigned long a3, unsigned long a4, unsigned long a5,
    unsigned long a6, unsigned long a7, struct xizi_smccc_res* res);

static struct xizi_smccc_res __invoke_sip_fn_smc(unsigned long function_id,
    unsigned long arg0,
    unsigned long arg1,
    unsigned long arg2)
{
    struct xizi_smccc_res res;

    __xizi_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
    return res;
}

void cpu_start_secondary(uint8_t cpu_id)
{
    __invoke_sip_fn_smc(PSCI_CPUON, cpu_id, (uintptr_t)0x200000, 0);
}

void start_smp_cache_broadcast(int cpu_id)
{
    return;
}
/*
 *
 *  Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/**
 * @file gicv3_registers.h
 * @brief gicv3 registers
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024.05.09
 */

/*************************************************
File name: gicv3_registers.c
Description: gicv3 registers
Others:
History:
Author: AIIT XUOS Lab
Modification:
1. Rename the file
*************************************************/

#pragma once

#include "memlayout.h"

// clang-format off
// interrupt controller GICv3
#define GICV3               MMIO_P2V_WO(0xfe600000ULL)
#define GICV3_REDIST        MMIO_P2V_WO(0xfe680000ULL)

#define D_CTLR 0x0
#define D_TYPER 0x4
#define D_IGROUPR(n)        (0x80 + (uint64_t)(n) * 4)
#define D_ISENABLER(n)      (0x100 + (uint64_t)(n) * 4)
#define D_ICENABLER(n)      (0x180 + (uint64_t)(n) * 4)
#define D_ISPENDR(n)        (0x200 + (uint64_t)(n) * 4)
#define D_ICPENDR(n)        (0x280 + (uint64_t)(n) * 4)
#define D_IPRIORITYR(n)     (0x400 + (uint64_t)(n) * 4)
#define D_ITARGETSR(n)      (0x800 + (uint64_t)(n) * 4)
#define D_ICFGR(n)          (0xc00 + (uint64_t)(n) * 4)

#define R_CTLR              0x0
#define R_WAKER             0x14

#define SGI_BASE            0x10000
#define R_IGROUPR0          (SGI_BASE + 0x80)
#define R_ISENABLER0        (SGI_BASE + 0x100)
#define R_ICENABLER0        (SGI_BASE + 0x180)
#define R_ICPENDR0          (SGI_BASE + 0x280)
#define R_IPRIORITYR(n)     (SGI_BASE + 0x400 + (n) * 4)
#define R_ICFGR0            (SGI_BASE + 0xc00)
#define R_ICFGR1            (SGI_BASE + 0xc04)
#define R_IGRPMODR0         (SGI_BASE + 0xd00)
// clang-format on

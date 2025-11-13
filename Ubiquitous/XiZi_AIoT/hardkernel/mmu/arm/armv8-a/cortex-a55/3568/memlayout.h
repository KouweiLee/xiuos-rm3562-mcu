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
 * @file memlayout.h
 * @brief virtual memory and physical memory layout
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-04-25
 */
/*************************************************
File name: memlayout.h
Description: virtual memory and physical memory layout
Others:
History:
Author: AIIT XUOS Lab
Modification:
1. first version
*************************************************/
#pragma once

// Memory layout
// clang-format off

#define ARCH_BIT        64

/* A55 physical memory layout */
#define PHY_MEM_BASE            (0x0000000010000000ULL)
#define PHY_USER_FREEMEM_BASE   (0x0000000040000000ULL)
#define PHY_USER_FREEMEM_TOP    (0x00000000E0000000ULL)
#define PHY_MEM_STOP            (0x00000000E0000000ULL)
#define PHY_KERN_STOP           (0x00000000e0000000ULL)
/* PTE-PAGE_SIZE */
#define LEVEL4_PTE_SHIFT    12
#define LEVEL4_PTE_SIZE     (1 << LEVEL4_PTE_SHIFT)

/* PDE-SECTION_SIZE */
#define LEVEL3_PDE_SHIFT    21
#define LEVEL3_PDE_SIZE     (1 << LEVEL3_PDE_SHIFT)

#define LEVEL2_PDE_SHIFT    30
#define LEVEL2_PDE_SIZE     (1 << LEVEL2_PDE_SHIFT)

#define LEVEL1_PTE_SHIFT    39

#define NUM_LEVEL2_PDE      (1 << (LEVEL1_PTE_SHIFT - LEVEL2_PDE_SHIFT))
#define NUM_LEVEL3_PDE      (1 << (LEVEL2_PDE_SHIFT - LEVEL3_PDE_SHIFT)) // how many PDE in a PT
#define NUM_LEVEL4_PTE      (1 << (LEVEL3_PDE_SHIFT - LEVEL4_PTE_SHIFT)) // how many PTE in a PT
#define NUM_TOPLEVEL_PDE    NUM_LEVEL2_PDE 

#define PAGE_SIZE           LEVEL4_PTE_SIZE
#define MAX_NR_FREE_PAGES   ((PHY_USER_FREEMEM_BASE - PHY_MEM_BASE) >> LEVEL4_PTE_SHIFT)

/* Deivce memory layout */
#define DEV_PHYMEM_BASE     (0x00000000F0000000ULL)
#define DEV_VRTMEM_BASE     (0x00000040F0000000ULL)
#define DEV_MEM_SIZE        (0x0000000010000000ULL)

/* User memory layout */
#define USER_STACK_SIZE     PAGE_SIZE
#define USER_MEM_BASE       (0x0000000000000000ULL)
#define USER_MEM_TOP        (0x0000004000000000ULL)
#define USER_IPC_SPACE_BASE (0x0000003000000000ULL)
#define USER_IPC_USE_ALLOCATOR_WATERMARK    (0x0000003000010000ULL)
#define USER_IPC_SPACE_TOP  (USER_IPC_SPACE_BASE + 0x10000000ULL) 

/* Kernel memory layout */
#define KERN_MEM_BASE       (0x0000006010000000ULL) // First kernel virtual address
#define KERN_OFFSET         (KERN_MEM_BASE - PHY_MEM_BASE)

#define V2P(a) (((uint64_t)(a)) - KERN_OFFSET)
#define P2V(a) ((void *)(((char *)(a)) + KERN_OFFSET))

#define V2P_WO(x) ((x) - KERN_OFFSET)    // same as V2P, but without casts
#define P2V_WO(x) ((x) + KERN_OFFSET)    // same as P2V, but without casts
// clang-format on
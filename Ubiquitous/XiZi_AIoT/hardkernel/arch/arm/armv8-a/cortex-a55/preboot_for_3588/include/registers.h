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
#ifndef INC_SYSREGS_H_
#define INC_SYSREGS_H_

/* SCTLR_EL1, System Control Register (EL1). */
#define SCTLR_RESERVED \
    ((3 << 28) | (3 << 22) | (1 << 20) | (1 << 11) | (1 << 8) | (1 << 7))
#define SCTLR_EE_LITTLE_ENDIAN (0 << 25)
#define SCTLR_E0E_LITTLE_ENDIAN (0 << 24)
#define SCTLR_I_CACHE (1 << 12)
#define SCTLR_D_CACHE (1 << 2)
#define SCTLR_MMU_DISABLED (0 << 0)
#define SCTLR_MMU_ENABLED (1 << 0)

#define SCTLR_VALUE_MMU_DISABLED                                       \
    (SCTLR_RESERVED | SCTLR_EE_LITTLE_ENDIAN | SCTLR_E0E_LITTLE_ENDIAN \
        | SCTLR_I_CACHE | SCTLR_D_CACHE | SCTLR_MMU_DISABLED)

/* HCR_EL2, Hypervisor Configuration Register (EL2). */
#define HCR_RW (1 << 31)
#define HCR_VALUE HCR_RW

/* CPACR_EL1, Architectural Feature Access Control Register. */
#define CPACR_FP_EN (3 << 20)
#define CPACR_TRACE_EN (0 << 28)
#define CPACR_VALUE (CPACR_FP_EN | CPACR_TRACE_EN)

/* SCR_EL3, Secure Configuration Register (EL3). */
#define SCR_RESERVED (3 << 4)
#define SCR_RW (1 << 10)
#define SCR_HCE (1 << 8)
#define SCR_SMD (1 << 7)
#define SCR_NS (1 << 0)
#define SCR_VALUE (SCR_RESERVED | SCR_RW | SCR_HCE | SCR_SMD | SCR_NS)

/* SPSR_EL1/2/3, Saved Program Status Register. */
#define SPSR_MASK_ALL (7 << 6)
#define SPSR_EL1h (5 << 0)
#define SPSR_EL2h (9 << 0)
#define SPSR_EL3_VALUE (SPSR_MASK_ALL | SPSR_EL2h)
#define SPSR_EL2_VALUE (SPSR_MASK_ALL | SPSR_EL1h)

/* Exception Class in ESR_EL1. */
#define EC_SHIFT 26
#define EC_UNKNOWN 0x00
#define EC_SVC64 0x15
#define EC_DABORT 0x24
#define EC_IABORT 0x20

#define PTE_VALID 1 // level 0,1,2 descriptor: valid
#define PTE_TABLE 2 // level 0,1,2 descriptor: table
#define PTE_V 3 // level 3 descriptor: valid
// PTE_AF(Access Flag)
//
// 0 -- this block entry has not yet.
// 1 -- this block entry has been used.
#define PTE_AF (1 << 10)
// PTE_AP(Access Permission) is 2bit field.
//        EL0   EL1
// 00 --   x    RW
// 01 --  RW    RW
// 10 --   x    RO
// 11 --  RO    RO
#define PTE_AP(ap) (((ap) & 3) << 6)
#define PTE_U PTE_AP(1)
#define PTE_RO PTE_AP(2)
#define PTE_URO PTE_AP(3)
#define PTE_PXN (1UL << 53) // Privileged eXecute Never
#define PTE_UXN (1UL << 54) // Unprivileged(user) eXecute Never
#define PTE_XN (PTE_PXN | PTE_UXN) // eXecute Never

// attribute index
// index is set by mair_el1
#define AI_DEVICE_nGnRnE_IDX 0x0
#define AI_NORMAL_NC_IDX 0x1

// memory type
#define MT_DEVICE_nGnRnE 0x0
#define MT_NORMAL_NC 0x44

#define PTE_INDX(i) (((i) & 7) << 2)
#define PTE_DEVICE PTE_INDX(AI_DEVICE_nGnRnE_IDX)
#define PTE_NORMAL PTE_INDX(AI_NORMAL_NC_IDX)

// shift a physical address to the right place for a PTE.
#define PA2PTE(pa) ((uint64_t)(pa) & 0xfffffffff000)
#define PTE2PA(pte) ((uint64_t)(pte) & 0xfffffffff000)

#define PTE_FLAGS(pte) ((pte) & (0x600000000003ff))

// translation control register
// #define TCR_T0SZ(n) ((n) & 0x3f)
// #define TCR_TG0(n) (((n) & 0x3) << 14)
// #define TCR_T1SZ(n) (((n) & 0x3f) << 16)
// #define TCR_TG1(n) (((n) & 0x3) << 30)
// #define TCR_IPS(n) (((n) & 0x7) << 32)

#define ISS_MASK 0xFFFFFF

#endif // INC_SYSREGS_H_

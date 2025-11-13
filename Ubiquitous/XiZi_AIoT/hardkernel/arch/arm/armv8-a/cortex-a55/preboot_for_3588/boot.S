/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *    http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */
#include "core.h"

#define HCR_VALUE (1 << 31)
#define SPSR_EL2_VALUE (7 << 6) | (5 << 0)
#define SCTLR_EL1_VALUE (0x30D00800)

.section ".text", "ax"
.global _boot_start
.global primary_cpu_init
.global el2_setup

_boot_start:
    bl el2_setup

    /* set NSACR, both Secure and Non-secure access are allowed to NEON */
    mov     x0,         #(3 << 20)
    msr     cpacr_el1,  x0
    isb

    // clear some registers
    msr     elr_el1,    XZR

    ldr     x0, =stacks_top
    mov     x1, #MODE_STACK_SIZE

    // get cpu id, and subtract the offset from the stacks base address
    mrs     x2, mpidr_el1
    and     x2, x2, #0xFFF
    lsr     x2, x2, #8
    mov     x5, x2
    mul     x3, x2, x1
    sub     x0, x0, x3
    mov     sp, x0

    mov     x2,         #ARM_MODE_EL1_h | DIS_INT  
    msr     spsr_el1,   x2

    // check cpu id - cpu0 is primary cpu
    mrs     x2, mpidr_el1
    and     x2, x2, #0xFFF
    lsr     x2, x2, #8
    mov     x5, x2
    cmp     x5, #0
    beq     primary_cpu_init
    bl      bootmain     // for secondary cpus, jump to argument function pointer passed in by ROM

    bl      .

primary_cpu_init:
   	/* init .bss */
     /* clear the .bss section (zero init) */
    ldr x1,	=boot_start_addr
    ldr x2,	=boot_end_addr
    mov x3,	#0
1:  
    cmp x1,	x2
    stp x3, x3, [x1], #16
    b.lt    1b  

	bl bootmain

    .func el2_setup
el2_setup:
    mrs x0, CurrentEL
    lsr x0, x0, #2
    and x0, x0, #3
    cmp x0, #2
    beq 1f
    ret

    /* Hyp configuration. */
1:
    mov x0, #(1 << 31)
    msr hcr_el2, x0

    /* Generic timers. */ 
    mrs  x0, cnthctl_el2
    orr  x0, x0, #3       // Enable EL1 physicaltimers
    msr  cnthctl_el2, x0

    /* Populate ID registers. */
    mrs  x0,    midr_el1
    mrs  x1,    mpidr_el1
    msr  vpidr_el2,  x0
    msr  vmpidr_el2, x1

    /* Disable Coprocessor traps. */
    mov x0, #0x33ff
    msr  cptr_el2, x0 // Disable copro. traps to EL2
    
    msr  hstr_el2, xzr // Disable CP15 traps to EL2

    mov x0, sp
    msr sp_el1, x0

    mrs x0, sctlr_el1
    orr x0, x0, #(1 << 0)
    bic x0, x0, #(1 << 1)
    orr x0, x0, #(1 << 2)
    msr sctlr_el1, x0

    /* spsr */
    mov x0, #SPSR_EL2_VALUE
    msr spsr_el2, x0
    msr elr_el2, lr
    eret
    .endfunc

    .end  
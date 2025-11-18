/* ------------------------------------------------------------------------- */
/*  @file:    startup_MIMXRT1052.s                                           */
/*  @purpose: CMSIS Cortex-M7 Core Device Startup File                       */
/*            MIMXRT1052                                                     */
/*  @version: 1.0                                                            */
/*  @date:    2018-9-21                                                      */
/*  @build:   b180921                                                        */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* Copyright 1997-2016 Freescale Semiconductor, Inc.                         */
/* Copyright 2016-2018 NXP                                                   */
/* All rights reserved.                                                      */
/*                                                                           */
/* SPDX-License-Identifier: BSD-3-Clause                                     */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/


/**
* @file boot.S
* @brief Cortex-M7 start function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-05-28
*/

/*************************************************
File name: boot.S
Description: Contex-M7 start function function
Others: take startup_MIMXRT1052.s for references
History: 
1. Date: 2021-05-28
Author: AIIT XUOS Lab
Modification: 
1. add OS entry function
*************************************************/

    .syntax unified
    .arch armv7-m
    .text
    .thumb

.word  _sidata
.word  __data_start__
.word  __data_end__
.word  __bss_start__
.word  __bss_end__

  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:
  ldr   sp, =__StackTop
  movs  r1, #0

/* Copy the data segment initializers from flash to SRAM */
DataInit:
  ldr  r0, =__data_start__
  ldr  r3, =__data_end__
  adds r2, r0, r1
  cmp  r2, r3
  bcs  DataInitEnd
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds r1, r1, #4
  b  DataInit

DataInitEnd:
  ldr  r2, =__bss_start__

/* Zero fill the bss segment. */
BSSInit:
  ldr  r3, = __bss_end__
  cmp  r2, r3
  bcs  BSSInitEnd
  movs r3, #0
  str  r3, [r2], #4
  b  BSSInit

BSSInitEnd:
  bl  SystemInit

  bl  entry
  bx  lr
.size  Reset_Handler, .-Reset_Handler
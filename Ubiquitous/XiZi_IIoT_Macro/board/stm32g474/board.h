/*
* Copyright (c) 2025 AIIT XUOS Lab
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
 * @file board.h
 * @brief Define STM32G474 board initialization and memory layout
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-07-03
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include <xsconfig.h>
#include <stdint.h>
#include "stm32g4xx_hal_pwr_ex.h"

/* ------------ SRAM & Stack Configuration ------------ */
#define STM32_SRAM_SIZE       96            // KB
#define SRAM_BASE_ADDR        0x20000000
#define SRAM_END_ADDR         (SRAM_BASE_ADDR + STM32_SRAM_SIZE * 1024)

/* ------------ Linker Symbols ------------ */
extern int __stack_end__;
extern int __bss_end;

/* ------------ Dynamic Heap Settings ------------ */
#define MEMORY_START_ADDRESS  ((void *)&__stack_end__)
#define MEMORY_END_ADDRESS    ((void *)SRAM_END_ADDR)

/* ------------ For Separate Compilation ------------ */
#ifdef SEPARATE_COMPILE

typedef int (*main_t)(int argc, char *argv[]);
typedef void (*exit_t)(void);

struct userspace_s {
    main_t    us_entrypoint;
    exit_t    us_taskquit;
    uintptr_t us_textstart;
    uintptr_t us_textend;
    uintptr_t us_datasource;
    uintptr_t us_datastart;
    uintptr_t us_dataend;
    uintptr_t us_bssstart;
    uintptr_t us_bssend;
    uintptr_t us_heapend;
};

/* 用户空间结构体地址示例：放在 Flash 特定位置 */
#define USERSPACE ((struct userspace_s *)0x08040000)

/* 服务表地址：位于 SRAM 起始 */
#ifndef SERVICE_TABLE_ADDRESS
#define SERVICE_TABLE_ADDRESS  (0x20000000)
#endif

#define USER_SRAM_SIZE              32
#define USER_MEMORY_START_ADDRESS (USERSPACE->us_bssend)
#define USER_MEMORY_END_ADDRESS   (0x10000000 + USER_SRAM_SIZE * 1024)

#endif /* SEPARATE_COMPILE */


/* ------------ Hardware Initialization ------------ */
void SystemClock_Config(void);
void InitBoardHardware(void);
// void stm32g474_start(void);
void Error_Handler(void);
#endif /* __BOARD_H__ */

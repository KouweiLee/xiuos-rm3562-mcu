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
 * @file board.h
 * @brief define stm32l476rgt6 nucleo board init configure and start-up function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-03-04
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stm32l4xx.h>

extern void *__bss_end;
extern void *_heap_end;
extern int __stack_end__;

#define MEMORY_START_ADDRESS (&__stack_end__)
#define STM32_SRAM_SIZE 95
#define MEMORY_END_ADDRESS (0x20000000 + STM32_SRAM_SIZE * 1024)

#define HEAP_START ((void *)&__bss_end)
#define HEAP_END ((void *)&_heap_end)

void SystemClock_Config(void);

#endif

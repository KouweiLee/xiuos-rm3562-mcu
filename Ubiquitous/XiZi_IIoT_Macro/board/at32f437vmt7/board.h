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
 * @brief define at32f437 board init configure and start-up function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-09-05
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stdio.h"


/**************** define system ******************/
extern unsigned int __stack_end__;
extern unsigned int __ram_end__;

#define MEMORY_START_ADDRESS    (&__stack_end__)
#define MEMORY_END_ADDRESS      (&__ram_end__)

/******************** functions ********************/
void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);

void InitBoardHardware(void);

#endif

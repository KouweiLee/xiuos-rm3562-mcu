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
* @brief define ch32v208 init configure and start-up function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2024-04-29
*/

/*************************************************
File name: board.h
Description: define ch32v208 rbt6 init configure and start-up function
Others: 
History: 
1. Date: 2024-04-29
Author: AIIT XUOS Lab
Modification: 
1. define ch32v208 rbt6 InitBoardHardware
*************************************************/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "ch32v20x.h"
#include <xsconfig.h>

#define ch32v20x_PIN_NUMBERS   48
/* board configuration */
#define SRAM_SIZE 64
#define EUSR_STACK_SIZE 2048

// #define SRAM_END (0x20000000 + SRAM_SIZE * 0x400)
// #define SRAM_END (0x20008000)
#define SRAM_END (0x20010000)
extern int _ebss;
extern int __stack_size;
#define MEMORY_START_ADDRESS    ((void *)&_ebss)
#define MEMORY_END_ADDRESS      (SRAM_END)

void InitBoardHardware(void);

#endif /* __BOARD_H__ */

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
* @brief define ch569 init configure and start-up function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2025-04-07
*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>
#include "CH56xSFR.h"


extern uint32_t _ebss;
extern uint32_t __stack_size;

#define MEMORY_START_ADDRESS    ((void *)&_ebss)
//#define MEMORY_END_ADDRESS      ((void *)(BA_RAM) + (SZ_RAM))
#define MEMORY_END_ADDRESS      ((void *)(BA_RAMX) + (SZ_RAMX))
#define MEMORY_STACK_SIZE    (&__stack_size)

void InitBoardHardware(void);

#endif /* __BOARD_H__ */

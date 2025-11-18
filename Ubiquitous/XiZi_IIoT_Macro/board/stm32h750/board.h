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
* @brief define stm32h750 init configure and start-up function
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2025-03-13
*/

#ifndef __BOARD_H__
#define __BOARD_H__

extern int __StackTop;

void InitBoardHardware();


#define HEAP_START   (void *)(&__StackTop)
#define HEAP_END    0x24080000



#endif

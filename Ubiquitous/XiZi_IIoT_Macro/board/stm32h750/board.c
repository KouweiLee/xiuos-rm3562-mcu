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
* @file board.c
* @brief support stm32f103-nano-board init configure and start-up
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-11-25
*/

#include <board.h>
#include <xizi.h>
#include <device.h>
#include <arch_interrupt.h>
#include <connect_uart.h>
#include <sys.h>
#include <delay.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_rcc.h>
#include <stm32h7xx_hal_rcc_ex.h>



void InitBoardHardware()
{
  sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(192, 5, 2, 4);
 
    //delay_init(480); 
    InitBoardMemory((void*)HEAP_START, (void*)HEAP_END);
  InitHwUart();

	
	InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);

  

}

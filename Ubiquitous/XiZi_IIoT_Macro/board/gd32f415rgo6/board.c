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
* @brief support stm32f407-st-discovery-board init configure and start-up
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

/*************************************************
File name: board.c
Description: support stm32f407-st-discovery-board init configure and driver/task/... init
Others: 
History: 
1. Date: 2021-04-25
Author: AIIT XUOS Lab
Modification: 
1. support stm32f407-st-discovery-board InitBoardHardware
*************************************************/

#include <xizi.h>
#include <board.h>
#include <gd32f4xx.h>
#include "connect_uart.h"

void SysTickConfiguration(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while(1) {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}


void SysTick_Handler(int irqn, void *arg)
{

    TickAndTaskTimesliceUpdate();

}

void InitBoardHardware()
{
    // system_clock_config() is called from SystemInit (void)
    SysTickConfiguration();

#ifdef BSP_USING_UART
    Gd32HwUsartInit();
#endif

    InitBoardMemory((void *)MEMORY_START_ADDRESS, (void *)MEMORY_END_ADDRESS);

#ifdef KERNEL_CONSOLE
    InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);

    KPrintf("\nconsole init completed.\n");
    KPrintf("board initialization......\n");
#endif

    KPrintf("board init done.\n");
    KPrintf("start kernel...\n");

}

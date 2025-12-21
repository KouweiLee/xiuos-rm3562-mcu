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
* @date 2025-04-25
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
#include "hal_base.h"
#include "hal_bsp.h"
#include "connect_uart.h"
#include "xs_base.h"
extern void RPMsgTask_Entry(void *param);

void SysTickConfiguration(void)
{
    /* tick init */
    HAL_SYSTICK_Disable();
    HAL_SetTickFreq(1000 / TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(HAL_SYSTICK_CLKSRC_CORE); // for ok562-j board, HAL_SYSTICK_CLKSRC_EXT doesn't work
    HAL_SYSTICK_Config((PLL_INPUT_OSC_RATE / TICK_PER_SECOND) - 1);
    HAL_SYSTICK_Enable();
    
}


extern void INTMUX_Dispatch(uint32_t irqOut);
void NVIC_INTMUX_Handler(int irqn, void *arg)
{
    uint32_t irqOut = irqn - INTMUX_OUT0_IRQn - NVIC_USER_IRQ_OFFSET;
    INTMUX_Dispatch(irqOut);
}


void NVIC_INTMUX_Configuration(void)
{
#ifdef INTMUX_IRQ_OUT0
    HAL_NVIC_EnableIRQ(INTMUX_OUT0_IRQn);
    isrManager.done->registerIrq(INTMUX_OUT0_IRQn+NVIC_USER_IRQ_OFFSET, NVIC_INTMUX_Handler, NULL);
#endif
#ifdef INTMUX_IRQ_OUT1
    HAL_NVIC_EnableIRQ(INTMUX_OUT1_IRQn);
    isrManager.done->registerIrq(INTMUX_OUT1_IRQn+NVIC_USER_IRQ_OFFSET, NVIC_INTMUX_Handler, NULL);
#endif
#ifdef INTMUX_IRQ_OUT2  
    HAL_NVIC_EnableIRQ(INTMUX_OUT2_IRQn);
    isrManager.done->registerIrq(INTMUX_OUT2_IRQn+NVIC_USER_IRQ_OFFSET, NVIC_INTMUX_Handler, NULL);
#endif
#ifdef INTMUX_IRQ_OUT3  
    HAL_NVIC_EnableIRQ(INTMUX_OUT3_IRQn);
    isrManager.done->registerIrq(INTMUX_OUT3_IRQn+NVIC_USER_IRQ_OFFSET, NVIC_INTMUX_Handler, NULL);
#endif
}


void SysTick_Handler(int irqn, void *arg)
{
    TickAndTaskTimesliceUpdate();

}
DECLARE_HW_IRQ(SYSTICK_IRQN, SysTick_Handler, NONE);


static struct UART_REG *pUart = UART9;
static void HAL_IOMUX_Uart9M1Config(void)
{
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_C2,
                         PIN_CONFIG_MUX_FUNC3);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_C3,
                         PIN_CONFIG_MUX_FUNC3);
}

void InitBoardHardware()
{
    /* HAL_Init */
    HAL_Init();

    /* hal bsp init */
    BSP_Init();

    NVIC_INTMUX_Configuration();

    SysTickConfiguration();

#ifdef BSP_USING_UART
    Rk3562HwUsartInit();
#endif

    InitBoardMemory((void *)MEMORY_START_ADDRESS, (void *)MEMORY_END_ADDRESS);


#ifdef KERNEL_CONSOLE
    InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);
    
    KPrintf("memory address range: [0x%08x - 0x%08x]\n", (x_ubase)MEMORY_START_ADDRESS,
            (x_ubase)MEMORY_END_ADDRESS);
    KPrintf("\nconsole init completed.\n");
    KPrintf("board initialization......\n");
#endif

    KPrintf("board init done.\n");
    KPrintf("start kernel...\n");

}

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
 * @brief support ch569 init configure and start-up
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-04-07
 */

#include <device.h>
#include <stdint.h>
#include <xizi.h>

#include "CH56x_common.h"
#include "connect_uart.h"
#include "board.h"


void InitBoardHardware()
{
	SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

    SysTick_Config(FREQ_SYS / 8 / TICK_PER_SECOND);
    PFIC_EnableIRQ(SWI_IRQn);

    /* initialize memory system */
    InitBoardMemory(MEMORY_START_ADDRESS, (void *)MEMORY_END_ADDRESS);

#ifdef BSP_USING_UART
    InitHwUart();
    InstallConsole("uart1", SERIAL_DRV_NAME_1, SERIAL_1_DEVICE_NAME_0);
    KPrintf("\nconsole init completed.\n");
#endif

#ifdef BSP_USING_SPI
    int InitHwSpi(void);
    InitHwSpi();
#endif

    KPrintf("memory address range: [0x%08x - 0x%08x] ssize: %x\n", (x_ubase)MEMORY_START_ADDRESS,
            (x_ubase)MEMORY_END_ADDRESS, MEMORY_STACK_SIZE);

    KPrintf("board init done.\n");
    KPrintf("start kernel...\n");
}

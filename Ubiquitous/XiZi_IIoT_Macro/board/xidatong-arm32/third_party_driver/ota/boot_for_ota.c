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
* @file boot_for_ota.c
* @brief support bootloader function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2023-11-20
*/

#include <stdint.h>
#include <xs_base.h>
#include "common.h"
#include "boot_for_ota.h"
#include "flash_for_ota.h"

#ifdef TOOL_USING_OTA

void mcuboot_bord_init(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    UartConfig();
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
}

void mcuboot_reset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

void mcuboot_jump(void)
{
    uint32_t addr = XIUOS_FLAH_ADDRESS;

    SCB->VTOR = addr;
    asm volatile("LDR R0, %0" : : "m"(addr));
    asm volatile("LDR   R0, [R0]");
    asm volatile("MOV   SP, R0");
    
    addr += 4;
    asm volatile("LDR R0, %0" : : "m"(addr));
    asm volatile("LDR   R0, [R0]");
    asm volatile("BX  R0");
}

extern void ImxrtMsDelay(uint32 ms);

void mcuboot_delay(uint32_t ms)
{
    ImxrtMsDelay(ms);
}
#endif
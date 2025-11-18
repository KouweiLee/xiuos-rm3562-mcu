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
 * @brief support at32f437 board init configure and start-up
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-09-05
 */

#include <board.h>
#include <xizi.h>
#include <device.h>
#include "at32f435_437_clock.h"
#include "at32f435_437_conf.h"
#include "board.h"
#ifdef BSP_USING_UART
#include "connect_usart.h"
#endif
#ifdef BSP_USING_ETH
#include "connect_ethernet.h"
#endif


/** @addtogroup AT32F435_437_board
  * @{
  */

#define MS_TICK                          (system_core_clock / 1000U)

/* delay macros */
#define STEP_DELAY_MS                    50

/* delay variable */
static __IO uint32_t fac_us;
static __IO uint32_t fac_ms;

/**
  * @brief  config systick and enable interrupt.
  * @param  none
  * @retval none
  */
static uint32_t systick_interrupt_config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  SysTick->VAL   = 0UL;
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |
                   SysTick_CTRL_ENABLE_Msk;
  return (0UL);
}


/**
  * @brief  initialize delay function
  * @param  none
  * @retval none
  */
void delay_init()
{
  /* configure systick */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  fac_us = system_core_clock / (1000000U);
  fac_ms = fac_us * (1000U);
}

/**
  * @brief  inserts a delay time.
  * @param  nus: specifies the delay time length, in microsecond.
  * @retval none
  */
void delay_us(uint32_t nus)
{
  uint32_t temp = 0;
  SysTick->LOAD = (uint32_t)(nus * fac_us);
  SysTick->VAL = 0x00;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
  do
  {
    temp = SysTick->CTRL;
  }while((temp & 0x01) && !(temp & (1 << 16)));

  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  SysTick->VAL = 0x00;
}

/**
  * @brief  inserts a delay time.
  * @param  nms: specifies the delay time length, in milliseconds.
  * @retval none
  */
void delay_ms(uint16_t nms)
{
  uint32_t temp = 0;
  while(nms)
  {
    if(nms > STEP_DELAY_MS)
    {
      SysTick->LOAD = (uint32_t)(STEP_DELAY_MS * fac_ms);
      nms -= STEP_DELAY_MS;
    }
    else
    {
      SysTick->LOAD = (uint32_t)(nms * fac_ms);
      nms = 0;
    }
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do
    {
      temp = SysTick->CTRL;
    }while((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
  }
}

/**
  * @brief  inserts a delay time.
  * @param  sec: specifies the delay time, in seconds.
  * @retval none
  */
void delay_sec(uint16_t sec)
{
  uint16_t index;
  for(index = 0; index < sec; index++)
  {
    delay_ms(500);
    delay_ms(500);
  }
}


/**
  * @brief  This function will initial your board.
  * @param  none
  * @retval none
 */
void InitBoardHardware(void)
{
  system_clock_config();

  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  systick_interrupt_config(MS_TICK);

  delay_init();

  InitBoardMemory((void *)MEMORY_START_ADDRESS, (void *)MEMORY_END_ADDRESS);

#ifdef BSP_USING_UART
  HwUsartInit();
#endif

#ifdef KERNEL_CONSOLE
  InstallConsole(KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME, KERNEL_CONSOLE_DEVICE_NAME);
  KPrintf("\nAT32F437 board console\n");
#endif
  KPrintf("RAM start=%08x end=%08x\n", MEMORY_START_ADDRESS, MEMORY_END_ADDRESS);

#ifdef BSP_USING_ETH
  HwEthInit();
#endif
}

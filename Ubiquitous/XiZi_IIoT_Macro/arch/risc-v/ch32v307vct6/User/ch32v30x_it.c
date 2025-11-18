/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v10x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main Interrupt Service Routines.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
/*************************************************
File name: ch32v30x_it.c
Description: include peripheral supports for ch32v30x
History:
1. Date: 2022-08-09
Author: AIIT XUOS Lab
Modification:
1. add HardFault interrupt implementation.
*************************************************/
#include "ch32v30x_it.h"
#include "board.h"
#include "ch32v30x_exti.h"
#include "ch32v30x_tim.h"
#include "eth_driver.h"
#include <xs_isr.h>

void NMI_Handler(void) __attribute__((interrupt()));
void HardFault_Handler(void) __attribute__((interrupt()));
void ETH_IRQHandler(void) __attribute__((interrupt()));
void TIM2_IRQHandler(void) __attribute__((interrupt()));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt()));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
    GET_INT_SP();
    isrManager.done->incCounter();
    KPrintf("NMI_Handler.\n");
    isrManager.done->decCounter();
    FREE_INT_SP();
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    GET_INT_SP();
    isrManager.done->incCounter();
    KPrintf("HardFault_Handler.\n");

    KPrintf("mepc  :%08x\r\n", __get_MEPC());
    KPrintf("mcause:%08x\r\n", __get_MCAUSE());
    KPrintf("mtval :%08x\r\n", __get_MTVAL());
    while (1)
        ;

    isrManager.done->decCounter();
    FREE_INT_SP();
}

/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   This function handles ETH exception.
 *
 * @return  none
 */
void ETH_IRQHandler(void)
{
    WCHNET_ETHIsr();
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void TIM2_IRQHandler(void)
{
    WCHNET_TimeIsr(WCHNETTIMERPERIOD);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

/*********************************************************************
 * @fn      EXTI9_5_IRQHandler
 *
 * @brief   This function handles GPIO exception.
 *
 * @return  none
 */
void EXTI9_5_IRQHandler(void)
{
    ETH_PHYLink();
    EXTI_ClearITPendingBit(EXTI_Line7); /* Clear Flag */
}
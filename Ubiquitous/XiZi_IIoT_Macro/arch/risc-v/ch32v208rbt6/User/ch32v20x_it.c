/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"
#include "board.h"
#include "ch32v20x_exti.h"
#include "ch32v20x_tim.h"
#include "eth_driver.h"
#include <xs_isr.h>
#include "wchble.h"

void NMI_Handler(void) __attribute__((interrupt()));
void HardFault_Handler(void) __attribute__((interrupt()));
void ETH_IRQHandler(void) __attribute__((interrupt()));
void TIM2_IRQHandler(void) __attribute__((interrupt()));
void BB_IRQHandler(void) __attribute__((interrupt()));
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
#ifdef TOOL_SHELL
	extern void ShowTask(void);
	extern void ShowMemory(void);
	ShowTask();
	ShowMemory();
#endif
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
#ifdef BSP_USING_ETH
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
u32 timeCnt;
void TIM2_IRQHandler(void)
{
    timeCnt++;
    WCHNET_TimeIsr(WCHNETTIMERPERIOD);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
#endif

/*********************************************************************
 * @fn      BB_IRQHandler
 *
 * @brief   BB Interrupt for BLE.
 *
 * @return  None
 */
#ifdef BSP_USING_BLE
void BB_IRQHandler(void)
{
    BB_IRQLibHandler();
}
#endif
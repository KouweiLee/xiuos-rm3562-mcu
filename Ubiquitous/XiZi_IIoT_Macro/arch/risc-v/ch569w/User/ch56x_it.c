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
#include "core_riscv.h"
#include "ch56x_it.h"
#include "board.h"
#include <xs_isr.h>


void NMI_Handler(void) __attribute__((interrupt()));
void HardFault_Handler(void) __attribute__((interrupt()));

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
	extern void ShowTask(void);
	extern void ShowMemory(void);
	ShowTask();
	ShowMemory();
    while (1)
        ;

    isrManager.done->decCounter();
    FREE_INT_SP();
}

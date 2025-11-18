/**
 * @file connect_uart.h
 * @brief define stm32l476 usart function and struct
 * @version 2.0
 * @author AIIT XUOS Lab
 * @date 2025-02-27
 */

#ifndef CONNECT_UART_H
#define CONNECT_UART_H

#include <device.h>
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

    struct UsartHwCfg
    {
        UART_HandleTypeDef *uart_device;
        IRQn_Type irq;
        uint32 usart_clock;
    };

#define UART_BAUDRATE_ERR_MAX (0.025F)
#define FCG_USART_CLK FCG_Fcg3PeriphClockCmd
#define FCG_TMR0_CLK FCG_Fcg2PeriphClockCmd
#define FCG_DMA_CLK FCG_Fcg0PeriphClockCmd

#define KERNEL_CONSOLE_BUS_NAME SERIAL_BUS_NAME_3
#define KERNEL_CONSOLE_DRV_NAME SERIAL_DRV_NAME_3
#define KERNEL_CONSOLE_DEVICE_NAME SERIAL_3_DEVICE_NAME_0

    int HwUsartInit(void);
#ifdef __cplusplus
}
#endif

#endif

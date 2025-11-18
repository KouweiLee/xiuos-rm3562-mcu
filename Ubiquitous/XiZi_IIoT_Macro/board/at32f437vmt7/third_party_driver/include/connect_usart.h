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
#include <at32f435_437_usart.h>

#ifdef __cplusplus
extern "C"
{
#endif


struct UsartHwCfg
{
    usart_type *uart_device;
    IRQn_Type irq;
};


int HwUsartInit(void);

#ifdef __cplusplus
}
#endif

#endif

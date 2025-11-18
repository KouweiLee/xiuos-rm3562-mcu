/**
 * @file connect_uart.h
 * @brief define rt1062xb usart function and struct
 * @version 2.0
 * @author AIIT XUOS Lab
 * @date 2025-05-23
 */

#ifndef CONNECT_UART_H
#define CONNECT_UART_H

#include <device.h>
#include "fsl_common.h"
#include "MIMXRT1062_COMMON.h"
#include "fsl_lpuart.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define KERNEL_CONSOLE_BUS_NAME SERIAL_BUS_NAME_1
#define KERNEL_CONSOLE_DRV_NAME SERIAL_DRV_NAME_1
#define KERNEL_CONSOLE_DEVICE_NAME SERIAL_1_DEVICE_NAME_0

    int HwUsartInit(void);
#ifdef __cplusplus
}
#endif

#endif

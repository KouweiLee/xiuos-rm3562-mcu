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
* @file connect_usart.h
* @brief define stm32f407zgt6-board usart function and struct
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

#ifndef CONNECT_USART_H
#define CONNECT_USART_H

#include <device.h>
#include "gd32f4xx_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KERNEL_CONSOLE_BUS_NAME        SERIAL_BUS_NAME_5
#define KERNEL_CONSOLE_DRV_NAME        SERIAL_DRV_NAME_5
#define KERNEL_CONSOLE_DEVICE_NAME SERIAL_5_DEVICE_NAME_0

struct UsartHwCfg
{
    uint32_t uart_device;
};

struct Gd32Usart
{
    struct SerialBus serial_bus;
};

int Gd32HwUsartInit(void);

#ifdef __cplusplus
}
#endif

#endif

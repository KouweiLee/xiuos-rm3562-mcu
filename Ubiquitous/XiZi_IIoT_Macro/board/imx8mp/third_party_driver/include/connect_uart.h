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

#ifndef CONNECT_UART_H
#define CONNECT_UART_H

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KERNEL_CONSOLE_BUS_NAME       "uart4"
#define KERNEL_CONSOLE_DRV_NAME       "uart4_drv"
#define KERNEL_CONSOLE_DEVICE_NAME    "uart4_dev"

int HwUartInit(void);
#ifdef __cplusplus
}
#endif

#endif

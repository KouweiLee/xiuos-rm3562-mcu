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
* @file connect_usb.h
* @brief define stm32l476 usb function and struct
* @version 2.0
* @author AIIT XUOS Lab
* @date 2025-03-12
*/

#ifndef CONNECT_USB_H
#define CONNECT_USB_H

#include <device.h>
#include "stm32l476xx.h"


#ifdef __cplusplus
extern "C" {
#endif

#define USB_HOST_STACK_SIZE 4096

#define USB_SINGLE_BLOCK_SIZE 512

int HwUsbDeviceInit(void);

#ifdef __cplusplus
}
#endif

#endif

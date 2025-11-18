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
* @brief define aiit-arm32-board usb function and struct
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

#ifndef CONNECT_USB_H
#define CONNECT_USB_H


#include <bus_usb.h>
#include <dev_usb.h>
#ifdef RESOURCES_USB_HOST
#ifdef BSP_USING_USBH
#include <ch32v30x_usbhs_host.h>
#endif
#endif




#ifdef __cplusplus
extern "C" {
#endif

int InitHwUsb(void);

#ifdef __cplusplus
}
#endif

#endif

/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *      http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */

/**
 * @file connect_usb.h
 * @brief Declarations for USB device connection and registration to XiUOS bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-07-01
 */

#ifndef CONNECT_USB_H__
#define CONNECT_USB_H__

#include <bus_usb.h>
#include <dev_usb.h>
#include <dev_serial.h>
#include <usbd_cdc_if.h>
#include <usbd_desc.h>
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usb_device.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GPIO_AF10_USB
#define GPIO_AF10_USB  ((uint8_t)0x0A)
#endif

#define USB_RX_BUFFER_SIZE      128 
#define USB_TIME_OUT            WAITING_FOREVER 


/**
 * @brief 初始化 USB 设备（注册总线、驱动、设备，挂载，总线初始化）
 * @return 初始化成功返回 EOK，否则返回错误码
 */
int InitUSB(void);
void UsbSetIsr(struct UsbHardwareDevice *usb_dev, int event);
void USB_HP_IRQHandler(int irqn, void *arg);
void USB_LP_IRQHandler(int irqn, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* CONNECT_USB_H__ */

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
* @file connect_usb.c
* @brief support stm32l476 usb function and register to bus framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2025-03-12
*/

/*************************************************
File name: connect_usb.c
Description: support stm32l476 usb function and register to bus framework
Others: 
History: 
1. Date: 2025-03-12
Author: AIIT XUOS Lab
Modification: 
1. support stm32l476 usb configure, write and read
2. support stm32l476 usb bus device and driver register
*************************************************/

#include <connect_usb.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBRingBuffer received_ring_buff;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void USB_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(OTG_FS_IRQn, USB_IRQHandler, NONE);

static uint32 UsbDeviceOpen(void *dev)
{
    return EOK;
}

static uint32 UsbDeviceClose(void *dev)
{
    return EOK;
}

static uint32 UsbDeviceRead(void *dev, struct BusBlockReadParam *read_param)
{
    uint16 read_lenth = 0;
    received_ring_buff.clear(&received_ring_buff);
    while (read_lenth < read_param->size)
    {
        uint16 need_to_read = read_param->size - read_lenth;
        uint16 read_count = MIN(need_to_read, received_ring_buff.count);
        received_ring_buff.pop(&received_ring_buff, read_param->buffer + read_lenth, read_count);
        read_lenth += read_count;
    }

    received_ring_buff.clear(&received_ring_buff);

    return 0;
}

static uint32 UsbDeviceWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    CDC_Transmit_FS((uint8 *)(write_param->buffer), write_param->size);
    return 0;
}

static int UsbControl(struct HardwareDev *dev, struct HalDevBlockParam *block_param)
{
    NULL_PARAM_CHECK(dev);
    return EOK;
}

/*manage the usb device operations*/
static const struct UsbDevDone dev_done =
{
    .open = UsbDeviceOpen,
    .close = UsbDeviceClose,
    .write = UsbDeviceWrite,
    .read = UsbDeviceRead,
};

/*Init usb host bus、driver*/
static int BoardUsbBusInit(struct UsbBus *usb_bus, struct UsbDriver *usb_driver)
{
    x_err_t ret = EOK;

    /*Init the usb bus */
    ret = UsbBusInit(usb_bus, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("board_usb_init UsbBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the usb driver*/
    ret = UsbDriverInit(usb_driver, USB_DRIVER_NAME);
    if (EOK != ret){
        KPrintf("board_usb_init UsbDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the usb driver to the usb bus*/
    ret = UsbDriverAttachToBus(USB_DRIVER_NAME, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("board_usb_init USEDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the usb device to the usb bus*/
static int BoardUsbDevBend(void)
{
    x_err_t ret = EOK;
    static struct UsbHardwareDevice usb_device;
    memset(&usb_device, 0, sizeof(struct UsbHardwareDevice));

    usb_device.dev_done = &dev_done;
    usb_device.haldev.dev_block_control = UsbControl;

    ret = USBDeviceRegister(&usb_device, NONE, USB_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("USBDeviceRegister device %s error %d\n", USB_DEVICE_NAME, ret);
        return ERROR;
    }

    ret = USBDeviceAttachToBus(USB_DEVICE_NAME, USB_BUS_NAME);
    if (EOK != ret) {
        KPrintf("USBDeviceAttachToBus device %s error %d\n", USB_DEVICE_NAME, ret);
        return ERROR;
    }

    return ret;
}

int HwUsbDeviceInit(void)
{
    x_err_t ret = EOK;

    static struct UsbBus usb_bus;
    memset(&usb_bus, 0, sizeof(struct UsbBus));

    static struct UsbDriver usb_driver;
    memset(&usb_driver, 0, sizeof(struct UsbDriver));

    ret = BoardUsbBusInit(&usb_bus, &usb_driver);
    if (EOK != ret) {
        KPrintf("BoardUsbBusInit error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardUsbDevBend();
    if (EOK != ret) {
        KPrintf("BoardUsbDevBend error ret %u\n", ret);
        return ERROR;
    }

    MX_USB_DEVICE_Init();
    return ret;
}


#ifdef TEST_USB
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

void USBTest(void)
{
    x_err_t ret = EOK;

    bus = BusFind(USB_BUS_NAME);
    dev = BusFindDevice(bus, USB_DEVICE_NAME);
    drv = BusFindDriver(bus, USB_DRIVER_NAME);

    struct BusBlockWriteParam write_param;
    struct BusBlockReadParam read_param;

    uint8 write_data[] = "Please Input:";
    uint8 read_data[11] = {0};

    write_param.buffer = (void *)write_data;
    write_param.size = sizeof(write_data);
    BusDevWriteData(dev, &write_param);

    read_param.buffer = (void *)read_data;
    read_param.size = 10;
    BusDevReadData(dev, &read_param);

    KPrintf("USB received: %s\n", read_data);

}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                    USBTest, USBTest, open usb device and write_read parameters);
#endif

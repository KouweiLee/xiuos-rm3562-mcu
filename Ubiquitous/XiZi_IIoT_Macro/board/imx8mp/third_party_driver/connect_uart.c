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

/*************************************************
File name: connect_uart.c
Description: support imx8mp-board uart configure and uart bus register function
History: 
1. Date: 2025-02-26
Author: AIIT XUOS Lab
Modification: 
1. support imx8mp-board uart configure, write and read
2. support imx8mp-board uart bus device and driver register
*************************************************/

#include <board.h>
#include <connect_uart.h>
#include <stdint.h>

#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "xs_base.h"
struct SerialBus serial_bus_1;
struct SerialDriver serial_driver_1;
struct SerialHardwareDevice serial_device_1;


static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);
    
    // struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
	struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
	struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;
	
    dev_param->serial_timeout = WAITING_FOREVER;


    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
    
    return EOK;
}

// Now for imx8mp, shell uses polling for reading, so each time before `SerialDevRead`
// function is executed, we need to set the sem value to a nonzero value so that
// shell can enter `SerialDevPollingRead` function
static void SerialSemInit() {
    int ret = KSemaphoreSetValue(serial_device_1.haldev.dev_sem, 100);
    if (ret != EOK) {
        KPrintf("SerialSemInit error %d\n", ret);
    }
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    // This function is used to change the interrupt status of serial device, now we don't support interrupt for uart4.
    return EINVALED;
}

static UART_Type *const s_UartAdapterBase[] = UART_BASE_PTRS;

static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char c)
{
    SerialSemInit();
    (void)UART_WriteBlocking(s_UartAdapterBase[BOARD_DEBUG_UART_INSTANCE], (uint8_t *)(&c), 1);
    return 1;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    int c = -1;
    SerialSemInit();
    uint8_t buf;
    (void)UART_ReadBlocking(s_UartAdapterBase[BOARD_DEBUG_UART_INSTANCE], &buf, 1);
    c = (int)buf;
    return c;
}

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = SerialInit,
    .configure = SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

static uint32 SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = SerialInit(serial_drv, configure_info);
            break;
        case OPE_CFG:
            serial_operation_cmd = *(int *)configure_info->private_data;
            ret = SerialConfigure(serial_drv, serial_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret) {
        KPrintf("HwUartInit SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret) {
        KPrintf("HwUartInit SerialDriverInit error %d\n", ret);
        return ERROR;
    }

        /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("HwUartInit SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/*Attach the serial device to the serial bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *serial_device, void *serial_param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(serial_device, serial_param, dev_name);
    if (EOK != ret) {
        KPrintf("HwUartInit SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("HwUartInit SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}

int HwUartInit(void)
{
    x_err_t ret = EOK;

    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(struct SerialDevParam));

    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = &SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_driver_1.private_data = (void *)&serial_cfg_1; 

    serial_dev_param_1.serial_work_mode = 0;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DRV_NAME);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit uart error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, KERNEL_CONSOLE_BUS_NAME, KERNEL_CONSOLE_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit uart error ret %u\n", ret);
        return ERROR;
    }    

    return ret;
}
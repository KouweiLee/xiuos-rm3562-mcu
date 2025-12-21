/*
 * Copyright (c) 2020 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

/**
* @file connect_usart.c
* @brief support stm32f407zgt6-board usart function and register to bus framework
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-25
*/

/*************************************************
File name: connect_uart.c
Description: support stm32f407zgt6-board usart configure and uart bus register function
Others: take RT-Thread v4.0.2/bsp/stm32/libraries/HAL_Drivers/drv_usart.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2021-04-25
Author: AIIT XUOS Lab
Modification: 
1. support stm32f407zgt6-board usart configure, write and read
2. support stm32f407zgt6-board usart bus device and driver register
*************************************************/

#include "board.h"
#include "connect_uart.h"
#include "hal_bsp.h"
#include "hal_base.h"


static void RCCConfiguration(void)
{
#ifdef BSP_USING_UART9

#endif
}

static void GPIOConfiguration(void)
{
#ifdef BSP_USING_UART9
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_C2,
                         PIN_CONFIG_MUX_FUNC3);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_C3,
                         PIN_CONFIG_MUX_FUNC3);

    struct HAL_UART_CONFIG hal_uart_config = {
        .baudRate = UART_BR_1500000,
        .dataBit = UART_DATA_8B,
        .stopBit = UART_ONE_STOPBIT,
        .parity = UART_PARITY_DISABLE,
    };
    HAL_UART_Init(&g_uart9Dev, &hal_uart_config);
#endif
}

static void NVIC_Configuration(IRQn_Type irq)
{
    // TODO: add interrupt handler function here
}

static void SerialCfgParamCheck(struct SerialCfgParam *serial_cfg_default, struct SerialCfgParam *serial_cfg_new)
{
    struct SerialDataCfg *data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &serial_cfg_new->data_cfg;

    if ((data_cfg_default->serial_baud_rate != data_cfg_new->serial_baud_rate) && (data_cfg_new->serial_baud_rate)) {
        data_cfg_default->serial_baud_rate = data_cfg_new->serial_baud_rate;
    }

    if ((data_cfg_default->serial_bit_order != data_cfg_new->serial_bit_order) && (data_cfg_new->serial_bit_order)) {
        data_cfg_default->serial_bit_order = data_cfg_new->serial_bit_order;
    }

    if ((data_cfg_default->serial_buffer_size != data_cfg_new->serial_buffer_size) && (data_cfg_new->serial_buffer_size)) {
        data_cfg_default->serial_buffer_size = data_cfg_new->serial_buffer_size;
    }

    if ((data_cfg_default->serial_data_bits != data_cfg_new->serial_data_bits) && (data_cfg_new->serial_data_bits)) {
        data_cfg_default->serial_data_bits = data_cfg_new->serial_data_bits;
    }

    if ((data_cfg_default->serial_invert_mode != data_cfg_new->serial_invert_mode) && (data_cfg_new->serial_invert_mode)) {
        data_cfg_default->serial_invert_mode = data_cfg_new->serial_invert_mode;
    }

    if ((data_cfg_default->serial_parity_mode != data_cfg_new->serial_parity_mode) && (data_cfg_new->serial_parity_mode)) {
        data_cfg_default->serial_parity_mode = data_cfg_new->serial_parity_mode;
    }

    if ((data_cfg_default->serial_stop_bits != data_cfg_new->serial_stop_bits) && (data_cfg_new->serial_stop_bits)) {
        data_cfg_default->serial_stop_bits = data_cfg_new->serial_stop_bits;
    }

    if ((data_cfg_default->serial_timeout != data_cfg_new->serial_timeout) && (data_cfg_new->serial_timeout)) {
        data_cfg_default->serial_timeout = data_cfg_new->serial_timeout;
    }
}

static uint32 Rk3562SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

	struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
	struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);

        if (serial_cfg_new->data_cfg.dev_recv_callback) {
            BusDevRecvCallback(&(serial_dev->haldev), serial_cfg_new->data_cfg.dev_recv_callback);
        }   
    }

	// config serial receive sem timeout
	dev_param->serial_timeout = serial_cfg->data_cfg.serial_timeout;

    struct UART_REG * uart_device = serial_hw_cfg->uart_device;

    struct HAL_UART_CONFIG hal_uart_config = {
        .baudRate = UART_BR_1500000,
        .dataBit = UART_DATA_8B,
        .stopBit = UART_ONE_STOPBIT,
        .parity = UART_PARITY_DISABLE,
    };
    HAL_UART_Init(&g_uart9Dev, &hal_uart_config);

    return EOK;
}

static uint32 Rk3562SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

    switch (serial_operation_cmd)
    {
        default:
            break;
    }

    return EOK;
}

static int Rk3562SerialPutchar(struct SerialHardwareDevice *serial_dev, char c)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    HAL_UART_SerialOutChar(serial_hw_cfg->uart_device, c);

    return EOK;
}

static int Rk3562SerialGetchar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    char ch = -1;
    
    // TODO: add getchar function here
    int ret = HAL_UART_SerialIn(serial_hw_cfg->uart_device, &ch, 1);

    return ret>0?ch:-1;
}

static uint32 Rk3562SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = Rk3562SerialInit(serial_drv, configure_info);
            break;
        default:
            break;
    }

    return ret;
}

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = Rk3562SerialInit,
    .configure = Rk3562SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = Rk3562SerialPutchar,
    .get_char = Rk3562SerialGetchar,
};

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    NULL_PARAM_CHECK(serial_bus);
    NULL_PARAM_CHECK(serial_driver);
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(drv_name);

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret){
        KPrintf("hw_serial_init SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret){
        KPrintf("hw_serial_init SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret){
        KPrintf("hw_serial_init SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/*Attach the serial device to the serial bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *serial_device, void *serial_param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(serial_device, serial_param, dev_name);
    if (EOK != ret){
        KPrintf("hw_serial_init SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("hw_serial_init SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}

#ifdef BSP_USING_UART9
struct Rk3562Usart serial_9;
struct SerialDriver serial_driver_9;
struct SerialHardwareDevice serial_device_9;
#endif

#define BAUD_RATE_1500000      1500000

static const struct SerialDataCfg data_cfg_init = {
    .serial_baud_rate = BAUD_RATE_1500000,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
    .serial_timeout = WAITING_FOREVER,
};

HAL_Status UART9_IRQHandler(uint32_t irq, void *args)
{
    SerialSetIsr(&serial_device_9, SERIAL_EVENT_RX_IND);
}

int Rk3562HwUsartInit(void)
{
    x_err_t ret = EOK;

    RCCConfiguration();
    GPIOConfiguration();

#ifdef BSP_USING_UART9
    static struct SerialCfgParam serial_cfg_9;
    memset(&serial_cfg_9, 0, sizeof(struct SerialCfgParam));

    static struct UsartHwCfg serial_hw_cfg_9;
    memset(&serial_hw_cfg_9, 0, sizeof(struct UsartHwCfg));

    static struct SerialDevParam serial_dev_param_9;
    memset(&serial_dev_param_9, 0, sizeof(struct SerialDevParam));
    
    serial_driver_9.drv_done = &drv_done;
    serial_driver_9.configure = &Rk3562SerialDrvConfigure;
    serial_device_9.hwdev_done = &hwdev_done;

    serial_cfg_9.data_cfg = data_cfg_init;

    serial_hw_cfg_9.uart_device = UART9;
    serial_cfg_9.hw_cfg.private_data = (void *)&serial_hw_cfg_9;
    serial_driver_9.private_data = (void *)&serial_cfg_9;

    // use interrupt mode to receive data
    serial_dev_param_9.serial_work_mode = SIGN_OPER_INT_RX;
    HAL_INTMUX_SetIRQHandler(UART9_IRQn, UART9_IRQHandler, NULL);
    HAL_INTMUX_EnableIRQ(UART9_IRQn);
    HAL_UART_EnableIrq(UART9, UART_IER_RDI);

    serial_device_9.haldev.private_data = (void *)&serial_dev_param_9;

    ret = BoardSerialBusInit(&serial_9.serial_bus, &serial_driver_9, SERIAL_BUS_NAME_9, SERIAL_DRV_NAME_9);
    if (EOK != ret) {
        KPrintf("Rk3562HwUsartInit usart9 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_9, (void *)&serial_cfg_9, SERIAL_BUS_NAME_9, SERIAL_9_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("Rk3562HwUsartInit usart9 error ret %u\n", ret);
        return ERROR;
    }    
#endif

    return ret;
}

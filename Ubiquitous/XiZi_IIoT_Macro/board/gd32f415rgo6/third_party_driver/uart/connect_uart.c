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
#include "gd32f4xx_usart.h"
#include "gd32f4xx_gpio.h"


static void RCCConfiguration(void)
{
#ifdef BSP_USING_UART5
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART5);
#endif
}

static void GPIOConfiguration(void)
{
#ifdef BSP_USING_UART5
    gpio_af_set(GPIOA, GPIO_AF_8, GPIO_PIN_11 | GPIO_PIN_12);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11 | GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11 | GPIO_PIN_12);
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

}

static uint32 Gd32SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    uint32_t uart_device = serial_hw_cfg->uart_device;

    // Reset USART5
    usart_deinit(uart_device);

    // Set baud rate to 115200
    usart_baudrate_set(uart_device, 115200);

    // Configure other parameters, such as no parity, 8 data bits, 1 stop bit
    usart_parity_config(uart_device, USART_PM_NONE);
    usart_word_length_set(uart_device, USART_WL_8BIT);
    usart_stop_bit_set(uart_device, USART_STB_1BIT);

    // Enable transmission and reception
    usart_transmit_config(uart_device, USART_TRANSMIT_ENABLE);
    usart_receive_config(uart_device, USART_RECEIVE_ENABLE);

    // Enable USART5
    usart_enable(uart_device);

    return EOK;
}

static uint32 Gd32SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
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

static int Gd32SerialPutchar(struct SerialHardwareDevice *serial_dev, char c)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    while(RESET == usart_flag_get(USART5, USART_FLAG_TBE));
    usart_data_transmit(USART5, (uint8_t)c);

    return EOK;
}

static int Gd32SerialGetchar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    int ch = -1;

    if (usart_flag_get(serial_hw_cfg->uart_device, USART_FLAG_RBNE))
    {
        ch = usart_data_receive(serial_hw_cfg->uart_device);
    }

    return ch;
}

static uint32 Gd32SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = Gd32SerialInit(serial_drv, configure_info);
            break;
        default:
            break;
    }

    return ret;
}

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = Gd32SerialInit,
    .configure = Gd32SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = Gd32SerialPutchar,
    .get_char = Gd32SerialGetchar,
};

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

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

#ifdef BSP_USING_UART5
struct Gd32Usart serial_5;
struct SerialDriver serial_driver_5;
struct SerialHardwareDevice serial_device_5;

#endif

int Gd32HwUsartInit(void)
{
    x_err_t ret = EOK;

    RCCConfiguration();
    GPIOConfiguration();

#ifdef BSP_USING_UART5
    static struct SerialCfgParam serial_cfg_5;
    memset(&serial_cfg_5, 0, sizeof(struct SerialCfgParam));

    static struct UsartHwCfg serial_hw_cfg_5;
    memset(&serial_hw_cfg_5, 0, sizeof(struct UsartHwCfg));

    static struct SerialDevParam serial_dev_param_5;
    memset(&serial_dev_param_5, 0, sizeof(struct SerialDevParam));
    
    serial_driver_5.drv_done = &drv_done;
    serial_driver_5.configure = &Gd32SerialDrvConfigure;
    serial_device_5.hwdev_done = &hwdev_done;

    serial_hw_cfg_5.uart_device = USART5;
    serial_cfg_5.hw_cfg.private_data = (void *)&serial_hw_cfg_5;
    serial_driver_5.private_data = (void *)&serial_cfg_5;

    serial_device_5.haldev.private_data = (void *)&serial_dev_param_5;

    ret = BoardSerialBusInit(&serial_5.serial_bus, &serial_driver_5, SERIAL_BUS_NAME_5, SERIAL_DRV_NAME_5);
    if (EOK != ret) {
        KPrintf("Gd32HwUsartInit usart5 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_5, (void *)&serial_cfg_5, SERIAL_BUS_NAME_5, SERIAL_5_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("Gd32HwUsartInit usart5 error ret %u\n", ret);
        return ERROR;
    }    
#endif

    return ret;
}

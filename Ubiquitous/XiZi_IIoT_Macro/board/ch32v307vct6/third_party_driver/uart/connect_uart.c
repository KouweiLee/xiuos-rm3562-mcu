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
 * @file connect_usart.c
 * @brief support ch32v307 vct6 uart function and register to bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022-08-01
 */

#include "connect_uart.h"
#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "xsconfig.h"
#include <xizi.h>

/*  uart driver */

static void SerialCfgParamCheck(struct SerialCfgParam* serial_cfg_default, struct SerialCfgParam* serial_cfg_new)
{
    struct SerialDataCfg* data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg* data_cfg_new = &serial_cfg_new->data_cfg;

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

static void UartIsr(struct SerialDriver* serial_drv, struct SerialHardwareDevice* serial_dev)
{
    struct SerialCfgParam* serial_cfg = (struct SerialCfgParam*)serial_drv->private_data;

    if (RESET != USART_GetITStatus((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_IT_RXNE)) {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
        USART_ClearITPendingBit((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_IT_RXNE);
    }
}

static uint32 SerialInit(struct SerialDriver* serial_drv, struct BusConfigureInfo* configure_info)
{
    KPrintf("Serial init\n");
    NULL_PARAM_CHECK(serial_drv);
    struct SerialCfgParam* serial_cfg = (struct SerialCfgParam*)serial_drv->private_data;
    // struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    struct SerialHardwareDevice* serial_dev = (struct SerialHardwareDevice*)serial_drv->driver.owner_bus->owner_haldev;
    struct SerialDevParam* dev_param = (struct SerialDevParam*)serial_dev->haldev.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam* serial_cfg_new = (struct SerialCfgParam*)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);

        if (serial_cfg_new->data_cfg.dev_recv_callback) {
            BusDevRecvCallback(&(serial_dev->haldev), serial_cfg_new->data_cfg.dev_recv_callback);
        }
    }

    // config serial receive sem timeout
    dev_param->serial_timeout = serial_cfg->data_cfg.serial_timeout;

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver* serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam* serial_cfg = (struct SerialCfgParam*)serial_drv->private_data;

    switch (serial_operation_cmd) {
    case OPER_CLR_INT:
        NVIC_DisableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
        USART_ITConfig((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, DISABLE);
        break;
    case OPER_SET_INT:
        NVIC_EnableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
        /* enable USART0 receive interrupt */
        USART_ITConfig((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, ENABLE);
        break;
    }

    return EOK;
}

static uint32 SerialDrvConfigure(void* drv, struct BusConfigureInfo* configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver* serial_drv = (struct SerialDriver*)drv;

    switch (configure_info->configure_cmd) {
    case OPE_INT:
        ret = SerialInit(serial_drv, configure_info);
        break;
    case OPE_CFG:
        serial_operation_cmd = *(int*)configure_info->private_data;
        ret = SerialConfigure(serial_drv, serial_operation_cmd);
        break;
    default:
        break;
    }

    return ret;
}

static int SerialPutChar(struct SerialHardwareDevice* serial_dev, char c)
{
    struct SerialCfgParam* serial_cfg = (struct SerialCfgParam*)serial_dev->private_data;

    while (USART_GetFlagStatus((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, (uint8_t)c);
    return 0;
}

static int SerialGetChar(struct SerialHardwareDevice* serial_dev)
{
    int ch = -1;
    struct SerialCfgParam* serial_cfg = (struct SerialCfgParam*)serial_dev->private_data;

    if (RESET != USART_GetFlagStatus((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base, USART_FLAG_RXNE)) {
        ch = USART_ReceiveData((USART_TypeDef*)serial_cfg->hw_cfg.serial_register_base) & 0xff;
    }

    return ch;
}

static const struct SerialDataCfg data_cfg_init = {
    .serial_baud_rate = BAUD_RATE_9600,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
    .serial_timeout = WAITING_FOREVER,
};

static const struct SerialDataCfg data_cfg_init_uart3 = {
    .serial_baud_rate = BAUD_RATE_115200,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
    .serial_timeout = WAITING_FOREVER,
};

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done = {
    .init = SerialInit,
    .configure = SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done = {
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

static int BoardSerialBusInit(struct SerialBus* serial_bus, struct SerialDriver* serial_driver, const char* bus_name, const char* drv_name)
{
    x_err_t ret = EOK;

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the serial device to the serial bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice* serial_device, void* serial_param, const char* bus_name, const char* dev_name)
{
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(serial_device, serial_param, dev_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }

    return ret;
}

#ifdef BSP_USING_UART1
struct SerialDriver serial_driver_1;
struct SerialHardwareDevice serial_device_1;

void USART1_IRQHandler(void) __attribute__((interrupt()));
void USART1_IRQHandler(void)
{
    GET_INT_SP();
    x_base level;
    level = DisableLocalInterrupt();
    isrManager.done->incCounter();
    EnableLocalInterrupt(level);
    UartIsr(&serial_driver_1, &serial_device_1);
    level = DisableLocalInterrupt();
    isrManager.done->decCounter();
    EnableLocalInterrupt(level);
    FREE_INT_SP();
}
#endif

uint16_t UART_ReceiveData(USART_TypeDef* USARTx)
{
    return (uint16_t)(USARTx->DATAR & (uint16_t)0x01FF);
}

void UART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
    USARTx->DATAR = (Data & (uint16_t)0x01FF);
}

#ifdef BSP_USING_UART2
struct SerialDriver serial_driver_2;
struct SerialHardwareDevice serial_device_2;

#define TxSize2 100
uint16_t RxBuffer2[TxSize2] = { 0 };
uint16_t TxCnt2 = 0, RxCnt2 = 0;

void USART3_IRQHandler(void) __attribute__((interrupt()));

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        RxBuffer2[RxCnt2++] = UART_ReceiveData(USART3);

        if (RxCnt2 == 100) {
            USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
        }
    }
}
#endif

#ifdef BSP_USING_UART4
struct SerialDriver serial_driver_4;
struct SerialHardwareDevice serial_device_4;

#define TxSize2 100
uint16_t RxBuffer4[TxSize2] = { 0 };
uint16_t TxCnt4 = 0, RxCnt4 = 0;

void UART4_IRQHandler(void) __attribute__((interrupt()));

void UART4_IRQHandler(void)
{
    GET_INT_SP();
    x_base level;
    level = DisableLocalInterrupt();
    isrManager.done->incCounter();
    EnableLocalInterrupt(level);
    UartIsr(&serial_driver_4, &serial_device_4);
    level = DisableLocalInterrupt();
    isrManager.done->decCounter();
    EnableLocalInterrupt(level);
    FREE_INT_SP();
}
#endif

#ifdef BSP_USING_UART5
struct SerialDriver serial_driver_5;
struct SerialHardwareDevice serial_device_5;

#define TxSize5 100
uint16_t RxBuffer5[TxSize5] = { 0 };
uint16_t TxCnt5 = 0, RxCnt5 = 0;

void USART5_IRQHandler(void) __attribute__((interrupt()));

void USART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        RxBuffer5[RxCnt5++] = UART_ReceiveData(UART5);
        if (RxCnt5 == 100) {
            USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
        }
    }
}
#endif


int InitHwUart(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_UART1
    static struct SerialBus serial_bus;
    memset(&serial_bus, 0, sizeof(struct SerialBus));

    memset(&serial_driver_1, 0, sizeof(struct SerialDriver));

    memset(&serial_device_1, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param;
    memset(&serial_dev_param, 0, sizeof(struct SerialDevParam));

    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = &SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_cfg.data_cfg = data_cfg_init;

    serial_cfg.hw_cfg.serial_register_base = (uint32)USART1;
    serial_cfg.hw_cfg.serial_irq_interrupt = USART1_IRQn;

    serial_driver_1.private_data = (void*)&serial_cfg;

    serial_dev_param.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void*)&serial_dev_param;

    ret = BoardSerialBusInit(&serial_bus, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void*)&serial_cfg, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    GPIO_InitTypeDef gpio_init_struct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    gpio_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);
    gpio_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_init_struct);

    USART_InitTypeDef usart_init_struct;
    usart_init_struct.USART_BaudRate = 9600;
    usart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_struct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_struct.USART_WordLength = USART_WordLength_8b;
    usart_init_struct.USART_StopBits = USART_StopBits_1;
    usart_init_struct.USART_Parity = USART_Parity_No;
    USART_Init((USART_TypeDef*)serial_cfg.hw_cfg.serial_register_base, &usart_init_struct);
    USART_Cmd((USART_TypeDef*)serial_cfg.hw_cfg.serial_register_base, ENABLE);
#endif

#ifdef BSP_USING_UART2
    static struct SerialBus serial_bus_2;
    memset(&serial_bus_2, 0, sizeof(struct SerialBus));

    memset(&serial_driver_2, 0, sizeof(struct SerialDriver));

    memset(&serial_device_2, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_2;
    memset(&serial_cfg_2, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_2;
    memset(&serial_dev_param_2, 0, sizeof(struct SerialDevParam));

    serial_driver_2.drv_done = &drv_done;
    serial_driver_2.configure = &SerialDrvConfigure;
    serial_device_2.hwdev_done = &hwdev_done;

    serial_cfg_2.data_cfg = data_cfg_init;

    serial_cfg_2.hw_cfg.serial_register_base = (uint32)USART3;
    serial_cfg_2.hw_cfg.serial_irq_interrupt = USART3_IRQn;

    serial_driver_2.private_data = (void*)&serial_cfg_2;

    serial_dev_param_2.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_2.haldev.private_data = (void*)&serial_dev_param_2;

    ret = BoardSerialBusInit(&serial_bus_2, &serial_driver_2, SERIAL_BUS_NAME_2, SERIAL_DRV_NAME_2);
    if (EOK != ret) {
        KPrintf("InitHwUart 2 uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_2, (void*)&serial_cfg_2, SERIAL_BUS_NAME_2, SERIAL_2_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart 2 uarths error ret %u\n", ret);
        return ERROR;
    }

    GPIO_InitTypeDef gpio_init_struct_2;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);
    gpio_init_struct_2.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct_2.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_2.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_init_struct_2);
    gpio_init_struct_2.GPIO_Pin = GPIO_Pin_11;
    gpio_init_struct_2.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio_init_struct_2);

    USART_InitTypeDef usart_init_struct_2;
    usart_init_struct_2.USART_BaudRate = 9600;
    usart_init_struct_2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_struct_2.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_struct_2.USART_WordLength = USART_WordLength_8b;
    usart_init_struct_2.USART_StopBits = USART_StopBits_1;
    usart_init_struct_2.USART_Parity = USART_Parity_No;
    USART_Init((USART_TypeDef*)serial_cfg_2.hw_cfg.serial_register_base, &usart_init_struct_2);
    USART_Cmd((USART_TypeDef*)serial_cfg_2.hw_cfg.serial_register_base, ENABLE);
#endif

#ifdef BSP_USING_UART4
    static struct SerialBus serial_bus_4;
    memset(&serial_bus_4, 0, sizeof(struct SerialBus));

    memset(&serial_driver_4, 0, sizeof(struct SerialDriver));

    memset(&serial_device_4, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_4;
    memset(&serial_cfg_4, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_4;
    memset(&serial_dev_param_4, 0, sizeof(struct SerialDevParam));

    serial_driver_4.drv_done = &drv_done;
    serial_driver_4.configure = &SerialDrvConfigure;
    serial_device_4.hwdev_done = &hwdev_done;

    serial_cfg_4.data_cfg = data_cfg_init;

    serial_cfg_4.hw_cfg.serial_register_base = (uint32)UART4;
    serial_cfg_4.hw_cfg.serial_irq_interrupt = UART4_IRQn;

    serial_driver_4.private_data = (void*)&serial_cfg_4;

    serial_dev_param_4.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_4.haldev.private_data = (void*)&serial_dev_param_4;

    ret = BoardSerialBusInit(&serial_bus_4, &serial_driver_4, SERIAL_BUS_NAME_4, SERIAL_DRV_NAME_4);
    if (EOK != ret) {
        KPrintf("InitHwUart 4 uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_4, (void*)&serial_cfg_4, SERIAL_BUS_NAME_4, SERIAL_4_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart 4 uarths error ret %u\n", ret);
        return ERROR;
    }

    GPIO_InitTypeDef gpio_init_struct_4;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio_init_struct_4.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct_4.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_4.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio_init_struct_4);
    gpio_init_struct_4.GPIO_Pin = GPIO_Pin_11;
    gpio_init_struct_4.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_4.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio_init_struct_4);

    USART_InitTypeDef usart_init_struct_4;
    usart_init_struct_4.USART_BaudRate = 9600;
    usart_init_struct_4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_struct_4.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_struct_4.USART_WordLength = USART_WordLength_8b;
    usart_init_struct_4.USART_StopBits = USART_StopBits_1;
    usart_init_struct_4.USART_Parity = USART_Parity_No;
    USART_Init((USART_TypeDef*)serial_cfg_4.hw_cfg.serial_register_base, &usart_init_struct_4);
    USART_Cmd((USART_TypeDef*)serial_cfg_4.hw_cfg.serial_register_base, ENABLE);
#endif

#ifdef BSP_USING_UART5
    static struct SerialBus serial_bus_5;
    memset(&serial_bus_5, 0, sizeof(struct SerialBus));

    memset(&serial_driver_5, 0, sizeof(struct SerialDriver));

    memset(&serial_device_5, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_5;
    memset(&serial_cfg_5, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_5;
    memset(&serial_dev_param_5, 0, sizeof(struct SerialDevParam));

    serial_driver_5.drv_done = &drv_done;
    serial_driver_5.configure = &SerialDrvConfigure;
    serial_device_5.hwdev_done = &hwdev_done;

    serial_cfg_5.data_cfg = data_cfg_init;

    serial_cfg_5.hw_cfg.serial_register_base = (uint32)UART5;
    serial_cfg_5.hw_cfg.serial_irq_interrupt = UART5_IRQn;

    serial_driver_5.private_data = (void*)&serial_cfg_5;

    serial_dev_param_5.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_5.haldev.private_data = (void*)&serial_dev_param_5;

    ret = BoardSerialBusInit(&serial_bus_5, &serial_driver_5, SERIAL_BUS_NAME_5, SERIAL_DRV_NAME_5);
    if (EOK != ret) {
        KPrintf("InitHwUart 5 uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_5, (void*)&serial_cfg_5, SERIAL_BUS_NAME_5, SERIAL_5_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart 5 uarths error ret %u\n", ret);
        return ERROR;
    }

    GPIO_InitTypeDef gpio_init_struct_5;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    gpio_init_struct_5.GPIO_Pin = GPIO_Pin_12;
    gpio_init_struct_5.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_5.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio_init_struct_5);
    gpio_init_struct_5.GPIO_Pin = GPIO_Pin_2;
    gpio_init_struct_5.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct_5.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio_init_struct_5);

    USART_InitTypeDef usart_init_struct_5;
    usart_init_struct_5.USART_BaudRate = 9600;
    usart_init_struct_5.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_struct_5.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_struct_5.USART_WordLength = USART_WordLength_8b;
    usart_init_struct_5.USART_StopBits = USART_StopBits_1;
    usart_init_struct_5.USART_Parity = USART_Parity_No;
    USART_Init((USART_TypeDef*)serial_cfg_5.hw_cfg.serial_register_base, &usart_init_struct_5);
    USART_Cmd((USART_TypeDef*)serial_cfg_5.hw_cfg.serial_register_base, ENABLE);
#endif

    return ret;
}

/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 * PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the
 * Mulan PSL v2 for more details.
 */

/**
 * @file connect_rs485.c
 * @brief  support ch32v208rbt6 rs485 function and register to bus framework
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-07-04
 */

#include <ModuleConfig.h>
#include <connect_rs485.h>
#include <dev_serial.h>
#include <xizi.h>

#include "ch32v20x.h"
#include "ch32v20x_usart.h"

static void Rs485CfgParamCheck(struct SerialCfgParam *rs485_cfg_default, struct SerialCfgParam *rs485_cfg_new) {
    struct SerialDataCfg *data_cfg_default = &rs485_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &rs485_cfg_new->data_cfg;

    if ((data_cfg_default->serial_baud_rate != data_cfg_new->serial_baud_rate) && (data_cfg_new->serial_baud_rate)) {
        data_cfg_default->serial_baud_rate = data_cfg_new->serial_baud_rate;
    }

    if ((data_cfg_default->serial_bit_order != data_cfg_new->serial_bit_order) && (data_cfg_new->serial_bit_order)) {
        data_cfg_default->serial_bit_order = data_cfg_new->serial_bit_order;
    }

    if ((data_cfg_default->serial_buffer_size != data_cfg_new->serial_buffer_size) &&
        (data_cfg_new->serial_buffer_size)) {
        data_cfg_default->serial_buffer_size = data_cfg_new->serial_buffer_size;
    }

    if ((data_cfg_default->serial_data_bits != data_cfg_new->serial_data_bits) && (data_cfg_new->serial_data_bits)) {
        data_cfg_default->serial_data_bits = data_cfg_new->serial_data_bits;
    }

    if ((data_cfg_default->serial_invert_mode != data_cfg_new->serial_invert_mode) &&
        (data_cfg_new->serial_invert_mode)) {
        data_cfg_default->serial_invert_mode = data_cfg_new->serial_invert_mode;
    }

    if ((data_cfg_default->serial_parity_mode != data_cfg_new->serial_parity_mode) &&
        (data_cfg_new->serial_parity_mode)) {
        data_cfg_default->serial_parity_mode = data_cfg_new->serial_parity_mode;
    }

    if ((data_cfg_default->serial_stop_bits != data_cfg_new->serial_stop_bits) && (data_cfg_new->serial_stop_bits)) {
        data_cfg_default->serial_stop_bits = data_cfg_new->serial_stop_bits;
    }

    if ((data_cfg_default->serial_timeout != data_cfg_new->serial_timeout) && (data_cfg_new->serial_timeout)) {
        data_cfg_default->serial_timeout = data_cfg_new->serial_timeout;
    }
}

static void UartIsr(struct SerialDriver *rs485_drv, struct SerialHardwareDevice *rs485_dev) {
    struct SerialCfgParam *rs485_cfg = (struct SerialCfgParam *)rs485_drv->private_data;

    if (RESET != USART_GetITStatus((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_IT_RXNE)) {
        SerialSetIsr(rs485_dev, SERIAL_EVENT_RX_IND);
        USART_ClearITPendingBit((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_IT_RXNE);
    }
}

static uint32 Rs485Init(struct SerialDriver *rs485_drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(rs485_drv);
    struct SerialCfgParam *rs485_cfg = (struct SerialCfgParam *)rs485_drv->private_data;

    struct SerialHardwareDevice *rs485_dev = (struct SerialHardwareDevice *)rs485_drv->driver.owner_bus->owner_haldev;
    struct SerialDevParam *dev_param = (struct SerialDevParam *)rs485_dev->haldev.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *rs485_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        Rs485CfgParamCheck(rs485_cfg, rs485_cfg_new);

        if (rs485_cfg_new->data_cfg.dev_recv_callback) {
            BusDevRecvCallback(&(rs485_dev->haldev), rs485_cfg_new->data_cfg.dev_recv_callback);
        }
    }

    // config serial receive sem timeout
    dev_param->serial_timeout = rs485_cfg->data_cfg.serial_timeout;

    USART_InitTypeDef USART_InitStructure_Rs485;
    USART_InitStructure_Rs485.USART_BaudRate = rs485_cfg->data_cfg.serial_baud_rate;

    switch (rs485_cfg->data_cfg.serial_data_bits) {
        case DATA_BITS_8:
            if (rs485_cfg->data_cfg.serial_parity_mode == PARITY_ODD || rs485_cfg->data_cfg.serial_parity_mode == PARITY_EVEN)
                USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_9b;
            else
                USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_8b;
            break;
        case DATA_BITS_9:
            USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_9b;
            break;
        default:
            USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_8b;
            break;
    }

    switch (rs485_cfg->data_cfg.serial_stop_bits) {
        case STOP_BITS_1:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_1;
            break;
        case STOP_BITS_2:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_2;
            break;
        default:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_1;
            break;
    }

    switch (rs485_cfg->data_cfg.serial_parity_mode) {
        case PARITY_NONE:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_No;
            break;
        case PARITY_ODD:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_Odd;
            break;
        case PARITY_EVEN:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_Even;
            break;
        default:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_No;
            break;
    }

    USART_InitStructure_Rs485.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure_Rs485.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART3, &USART_InitStructure_Rs485);
    USART_Cmd(USART3, ENABLE);

    return EOK;
}

static uint32 Rs485Configure(struct SerialDriver *rs485_drv, int rs485_operation_cmd) {
    NULL_PARAM_CHECK(rs485_drv);

    struct SerialCfgParam *rs485_cfg = (struct SerialCfgParam *)rs485_drv->private_data;

    switch (rs485_operation_cmd) {
        case OPER_CLR_INT:
            NVIC_DisableIRQ(rs485_cfg->hw_cfg.serial_irq_interrupt);
            USART_ITConfig((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, DISABLE);
            break;
        case OPER_SET_INT:
            NVIC_EnableIRQ(rs485_cfg->hw_cfg.serial_irq_interrupt);
            /* enable USART0 receive interrupt */
            USART_ITConfig((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, ENABLE);
            break;
    }

    return EOK;
}

static uint32 Rs485DrvConfigure(void *drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int rs485_operation_cmd;
    struct SerialDriver *rs485_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd) {
        case OPE_INT:
            ret = Rs485Init(rs485_drv, configure_info);
            break;
        case OPE_CFG:
            rs485_operation_cmd = *(int *)configure_info->private_data;
            ret = Rs485Configure(rs485_drv, rs485_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static int Rs485PutChar(struct SerialHardwareDevice *rs485_dev, char c) {
    struct SerialCfgParam *rs485_cfg = (struct SerialCfgParam *)rs485_dev->private_data;

    while (USART_GetFlagStatus((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_FLAG_TXE) == RESET);
    USART_SendData((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, (uint8_t)c);
    return 0;
}

static int Rs485GetChar(struct SerialHardwareDevice *rs485_dev) {
    int ch = -1;
    struct SerialCfgParam *rs485_cfg = (struct SerialCfgParam *)rs485_dev->private_data;
    if (RESET != USART_GetFlagStatus((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base, USART_FLAG_RXNE)) {
        ch = USART_ReceiveData((USART_TypeDef *)rs485_cfg->hw_cfg.serial_register_base) & 0xff;
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

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done = {
    .init = Rs485Init,
    .configure = Rs485Configure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done = {
    .put_char = Rs485PutChar,
    .get_char = Rs485GetChar,
};

static int BoardSerialBusInit(struct SerialBus *rs485_bus, struct SerialDriver *rs485_driver, const char *bus_name,
                              const char *drv_name) {
    x_err_t ret = EOK;

    /*Init the rs485 bus */
    ret = SerialBusInit(rs485_bus, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the rs485 driver*/
    ret = SerialDriverInit(rs485_driver, drv_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the rs485 driver to the rs485 bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the rs485 device to the rs485 bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *rs485_device, void *rs485_param, const char *bus_name,
                              const char *dev_name) {
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(rs485_device, rs485_param, dev_name);
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

#ifdef BSP_USING_RS485
struct SerialDriver serial_driver_rs485;
struct SerialHardwareDevice serial_device_rs485;

void USART3_IRQHandler(void) __attribute__((interrupt()));

void USART3_IRQHandler(void) {
    GET_INT_SP();
    x_base level;
    level = DisableLocalInterrupt();
    isrManager.done->incCounter();
    EnableLocalInterrupt(level);
    UartIsr(&serial_driver_rs485, &serial_device_rs485);
    level = DisableLocalInterrupt();
    isrManager.done->decCounter();
    EnableLocalInterrupt(level);
    FREE_INT_SP();
}
#endif

int InitHwRs485(void) {
    x_err_t ret = EOK;

    static struct SerialBus serial_bus_rs485;
    memset(&serial_bus_rs485, 0, sizeof(struct SerialBus));

    memset(&serial_driver_rs485, 0, sizeof(struct SerialDriver));

    memset(&serial_device_rs485, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_rs485;
    memset(&serial_cfg_rs485, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_rs485;
    memset(&serial_dev_param_rs485, 0, sizeof(struct SerialDevParam));

    serial_driver_rs485.drv_done = &drv_done;
    serial_driver_rs485.configure = &Rs485DrvConfigure;
    serial_device_rs485.hwdev_done = &hwdev_done;

    serial_cfg_rs485.data_cfg = data_cfg_init;

    serial_cfg_rs485.hw_cfg.serial_register_base = (uint32)USART3;
    serial_cfg_rs485.hw_cfg.serial_irq_interrupt = USART3_IRQn;

    serial_driver_rs485.private_data = (void *)&serial_cfg_rs485;

    serial_dev_param_rs485.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_rs485.haldev.private_data = (void *)&serial_dev_param_rs485;

    ret = BoardSerialBusInit(&serial_bus_rs485, &serial_driver_rs485, RS485_BUS_NAME, RS485_DRV_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart rs485 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_rs485, (void *)&serial_cfg_rs485, RS485_BUS_NAME, RS485_DEVICE_NAME_1);
    if (EOK != ret) {
        KPrintf("InitHwUart rs485 error ret %u\n", ret);
        return ERROR;
    }

    // uart2 init
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* USART3 TX-->B.10  RX-->B.11 */
    GPIO_InitTypeDef GPIO_InitStructure_Rs485 = {0};
    GPIO_InitStructure_Rs485.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure_Rs485.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_Rs485.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure_Rs485);
    GPIO_InitStructure_Rs485.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure_Rs485.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure_Rs485);

    extern pmodule_cfg CFG;
    USART_InitTypeDef USART_InitStructure_Rs485 = {0};
    switch (CFG->baudRate_Rs485) {
        case 1:
            USART_InitStructure_Rs485.USART_BaudRate = 2400;
            break;
        case 2:
            USART_InitStructure_Rs485.USART_BaudRate = 4800;
            break;
        case 3:
            USART_InitStructure_Rs485.USART_BaudRate = 9600;
            break;
        case 4:
            USART_InitStructure_Rs485.USART_BaudRate = 19200;
            break;
        case 5:
            USART_InitStructure_Rs485.USART_BaudRate = 38400;
            break;
        case 6:
            USART_InitStructure_Rs485.USART_BaudRate = 57600;
            break;
        case 7:
            USART_InitStructure_Rs485.USART_BaudRate = 115200;
            break;
        case 8:
            USART_InitStructure_Rs485.USART_BaudRate = 230400;
            break;
        default:
            USART_InitStructure_Rs485.USART_BaudRate = 9600;
            break;
    }
    switch (CFG->dataBits_Rs485) {
        case 1:
            USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_8b;
            break;
        case 2:
            USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_9b;
            break;
        default:
            USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_8b;
            break;
    }
    switch (CFG->stopBits_Rs485) {
        case 1:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_1;
            break;
        case 2:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_2;
            break;
        default:
            USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_1;
            break;
    }
    switch (CFG->parity_Rs485) {
        case 1:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_No;
            break;
        case 2:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_Odd;
            break;
        case 3:
            USART_InitStructure_Rs485.USART_Parity = USART_Parity_Even;
            break;
    }
    // USART_InitStructure_Rs485.USART_BaudRate = 9600;
    // USART_InitStructure_Rs485.USART_WordLength = USART_WordLength_8b;
    // USART_InitStructure_Rs485.USART_StopBits = USART_StopBits_1;
    // USART_InitStructure_Rs485.USART_Parity = USART_Parity_No;
    USART_InitStructure_Rs485.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure_Rs485.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure_Rs485);
    USART_Cmd(USART3, ENABLE);

    return ret;
}

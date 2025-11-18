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
 * @file connect_lte.c
 * @brief  support ch32v208rbt6 lte function and register to bus framework
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-07-04
 */

#include <xizi.h>
#include <connect_lte.h>

#include "ch32v20x.h"
#include "ch32v20x_usart.h"


static void LteCfgParamCheck(struct SerialCfgParam *lte_cfg_default, struct SerialCfgParam *lte_cfg_new) {
    struct SerialDataCfg *data_cfg_default = &lte_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &lte_cfg_new->data_cfg;

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

static void UartIsr(struct SerialDriver *lte_drv, struct SerialHardwareDevice *lte_dev) {
    struct SerialCfgParam *lte_cfg = (struct SerialCfgParam *)lte_drv->private_data;

    if (RESET != USART_GetITStatus((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_IT_RXNE)) {
        SerialSetIsr(lte_dev, SERIAL_EVENT_RX_IND);
        USART_ClearITPendingBit((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_IT_RXNE);
    }
}

static uint32 LteInit(struct SerialDriver *lte_drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(lte_drv);
    struct SerialCfgParam *lte_cfg = (struct SerialCfgParam *)lte_drv->private_data;

    struct SerialHardwareDevice *lte_dev = (struct SerialHardwareDevice *)lte_drv->driver.owner_bus->owner_haldev;
    struct SerialDevParam *dev_param = (struct SerialDevParam *)lte_dev->haldev.private_data;

    if (configure_info->private_data) {
        struct SerialCfgParam *lte_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        LteCfgParamCheck(lte_cfg, lte_cfg_new);

        if (lte_cfg_new->data_cfg.dev_recv_callback) {
            BusDevRecvCallback(&(lte_dev->haldev), lte_cfg_new->data_cfg.dev_recv_callback);
        }
    }

    // config serial receive sem timeout
    dev_param->serial_timeout = lte_cfg->data_cfg.serial_timeout;

    return EOK;
}

static uint32 LteConfigure(struct SerialDriver *lte_drv, int lte_operation_cmd) {
    NULL_PARAM_CHECK(lte_drv);

    struct SerialCfgParam *lte_cfg = (struct SerialCfgParam *)lte_drv->private_data;

    switch (lte_operation_cmd) {
        case OPER_CLR_INT:
            NVIC_DisableIRQ(lte_cfg->hw_cfg.serial_irq_interrupt);
            USART_ITConfig((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, DISABLE);
            break;
        case OPER_SET_INT:
            NVIC_EnableIRQ(lte_cfg->hw_cfg.serial_irq_interrupt);
            /* enable USART0 receive interrupt */
            USART_ITConfig((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_IT_RXNE, ENABLE);
            break;
    }

    return EOK;
}

static uint32 LteDrvConfigure(void *drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int lte_operation_cmd;
    struct SerialDriver *lte_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd) {
        case OPE_INT:
            ret = LteInit(lte_drv, configure_info);
            break;
        case OPE_CFG:
            lte_operation_cmd = *(int *)configure_info->private_data;
            ret = LteConfigure(lte_drv, lte_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static int LtePutChar(struct SerialHardwareDevice *lte_dev, char c) {
    struct SerialCfgParam *lte_cfg = (struct SerialCfgParam *)lte_dev->private_data;

    while (USART_GetFlagStatus((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_FLAG_TXE) == RESET);
    USART_SendData((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, (uint8_t)c);
    return 0;
}

static int LteGetChar(struct SerialHardwareDevice *lte_dev) {
    int ch = -1;
    struct SerialCfgParam *lte_cfg = (struct SerialCfgParam *)lte_dev->private_data;
    if (RESET != USART_GetFlagStatus((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base, USART_FLAG_RXNE)) {
        ch = USART_ReceiveData((USART_TypeDef *)lte_cfg->hw_cfg.serial_register_base) & 0xff;
    }
    return ch;
}

static const struct SerialDataCfg data_cfg_init = {
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
    .init = LteInit,
    .configure = LteConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done = {
    .put_char = LtePutChar,
    .get_char = LteGetChar,
};

static int BoardSerialBusInit(struct SerialBus *lte_bus, struct SerialDriver *lte_driver, const char *bus_name,
                              const char *drv_name) {
    x_err_t ret = EOK;

    /*Init the lte bus */
    ret = SerialBusInit(lte_bus, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the lte driver*/
    ret = SerialDriverInit(lte_driver, drv_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the lte driver to the lte bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("InitHwUart SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

/*Attach the lte device to the lte bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *lte_device, void *lte_param, const char *bus_name,
                              const char *dev_name) {
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(lte_device, lte_param, dev_name);
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

#ifdef BSP_USING_LTE
struct SerialDriver serial_driver_lte;
struct SerialHardwareDevice serial_device_lte;

void USART2_IRQHandler(void) __attribute__((interrupt()));

void USART2_IRQHandler(void) {
    GET_INT_SP();
    x_base level;
    level = DisableLocalInterrupt();
    isrManager.done->incCounter();
    EnableLocalInterrupt(level);
    UartIsr(&serial_driver_lte, &serial_device_lte);
    level = DisableLocalInterrupt();
    isrManager.done->decCounter();
    EnableLocalInterrupt(level);
    FREE_INT_SP();
}
#endif

int InitHwLte(void) {
    x_err_t ret = EOK;

    static struct SerialBus serial_bus_lte;
    memset(&serial_bus_lte, 0, sizeof(struct SerialBus));

    memset(&serial_driver_lte, 0, sizeof(struct SerialDriver));

    memset(&serial_device_lte, 0, sizeof(struct SerialHardwareDevice));

    static struct SerialCfgParam serial_cfg_lte;
    memset(&serial_cfg_lte, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_lte;
    memset(&serial_dev_param_lte, 0, sizeof(struct SerialDevParam));

    serial_driver_lte.drv_done = &drv_done;
    serial_driver_lte.configure = &LteDrvConfigure;
    serial_device_lte.hwdev_done = &hwdev_done;

    serial_cfg_lte.data_cfg = data_cfg_init;

    serial_cfg_lte.hw_cfg.serial_register_base = (uint32)USART2;
    serial_cfg_lte.hw_cfg.serial_irq_interrupt = USART2_IRQn;

    serial_driver_lte.private_data = (void *)&serial_cfg_lte;

    serial_dev_param_lte.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_lte.haldev.private_data = (void *)&serial_dev_param_lte;

    ret = BoardSerialBusInit(&serial_bus_lte, &serial_driver_lte, LTE_BUS_NAME, LTE_DRV_NAME);
    if (EOK != ret) {
        KPrintf("InitHwUart lte error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_lte, (void *)&serial_cfg_lte, LTE_BUS_NAME, LTE_DEVICE_NAME_1);
    if (EOK != ret) {
        KPrintf("InitHwUart lte error ret %u\n", ret);
        return ERROR;
    }

    // uart2 init
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 TX-->B.10  RX-->B.11 */
    GPIO_InitTypeDef GPIO_InitStructure_Lte = {0};
    GPIO_InitStructure_Lte.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure_Lte.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_Lte.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure_Lte);
    GPIO_InitStructure_Lte.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure_Lte.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure_Lte);

    USART_InitTypeDef USART_InitStructure_Lte = {0};
    USART_InitStructure_Lte.USART_BaudRate = 115200;
    USART_InitStructure_Lte.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure_Lte.USART_StopBits = USART_StopBits_1;
    USART_InitStructure_Lte.USART_Parity = USART_Parity_No;
    USART_InitStructure_Lte.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure_Lte.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure_Lte);
    USART_Cmd(USART2, ENABLE);

    /* Enable module pin */
    GPIO_InitStructure_Lte.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure_Lte.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_Lte.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure_Lte);
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
    Delay_Ms(500);    // >= 300ms
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    GPIO_InitStructure_Lte.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure_Lte.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure_Lte.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure_Lte);
    GPIO_SetBits(GPIOA, GPIO_Pin_6);
    Delay_Ms(1000);    // >= 500ms
    GPIO_ResetBits(GPIOA, GPIO_Pin_6);

    return ret;
}

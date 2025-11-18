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
 * @file connect_usart.c
 * @brief support ch569 uart function and register to bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-04-07
 */

#include <xizi.h>
#include "xsconfig.h"
#include "CH56x_common.h"
#include "ch56x_it.h"
#include "board.h"
#include "connect_uart.h"


/*  uart driver */


static void SerialCfgParamCheck(struct SerialCfgParam *serial_cfg_default, struct SerialCfgParam *serial_cfg_new) {
    struct SerialDataCfg *data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &serial_cfg_new->data_cfg;

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

static void UartIsr(struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev) {
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct uart_registers *uxreg = (struct uart_registers *)serial_cfg->hw_cfg.serial_register_base;

    if (UART_II_RECV_RDY == uxreg->IIR.int_mask) {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
        PFIC_ClearPendingIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
    }
}

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(serial_drv);
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;

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

    // config serial reg
    struct uart_registers *uxreg = (struct uart_registers *)serial_cfg->hw_cfg.serial_register_base;
    union _uart_fcr fcr;
    union _uart_lcr lcr;
    uint32_t x;
    uint32_t t = FREQ_SYS;

    // x = 10 * sys_hclk_get() / 8 / serial_cfg->data_cfg.serial_baud_rate;
    x = 10 * t / 8 / serial_cfg->data_cfg.serial_baud_rate;
    x = (x + 5) / 10;
    uxreg->DL = x;
    uxreg->DIV = 1;

    lcr.reg = 0;
    switch (serial_cfg->data_cfg.serial_data_bits)
    {
    case DATA_BITS_5:
        lcr.word_sz = LCR_DATA_BITS_5;
        break;
    case DATA_BITS_6:
        lcr.word_sz = LCR_DATA_BITS_6;
        break;
    case DATA_BITS_7:
        lcr.word_sz = LCR_DATA_BITS_7;
        break;
    case DATA_BITS_8:
    default:
        lcr.word_sz = LCR_DATA_BITS_8;
        break;
    }

    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
    case STOP_BITS_2:
        lcr.stop_bit = LCR_STOP_BITS_2;
        break;
    case STOP_BITS_1:
    default:
        lcr.stop_bit = LCR_STOP_BITS_1;
        break;
    }

    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
    case PARITY_ODD:
        lcr.par_mod = LCR_PARITY_ODD;
        lcr.par_en = 1;
        break;
    case PARITY_EVEN:
        lcr.par_mod = LCR_PARITY_EVEN;
        lcr.par_en = 1;
        break;
    case PARITY_NONE:
    default:
        lcr.par_en = 0;
        break;
    }
    uxreg->LCR.reg = lcr.reg;

    fcr.reg = RB_FCR_FIFO_EN | RB_FCR_RX_FIFO_CLR | RB_FCR_TX_FIFO_CLR;
    fcr.fifo_trig = UART_1BYTE_TRIG;
    uxreg->FCR.reg = fcr.reg;

    /* TXD pin output enable */
    uxreg->IER.txd_en = 1;

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd) {
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct uart_registers *uxreg = (struct uart_registers *)serial_cfg->hw_cfg.serial_register_base;

    switch (serial_operation_cmd) {
        case OPER_CLR_INT:
            uxreg->IER.recv_rdy = 0;
            uxreg->IER.line_stat = 0;
            uxreg->IER.thr_empty = 0;
            PFIC_DisableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
            break;
        case OPER_SET_INT:
            uxreg->FCR.fifo_trig = UART_1BYTE_TRIG;
            uxreg->MCR.int_oe = 1;
            uxreg->IER.recv_rdy = 1;
            uxreg->IER.line_stat = 1;
            uxreg->IER.thr_empty = 1;
            PFIC_EnableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
            break;
    }

    return EOK;
}

static uint32 SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info) {
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd) {
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

static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char ch) {
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    volatile struct uart_registers *uxreg = (struct uart_registers *)serial_cfg->hw_cfg.serial_register_base;

    if (uxreg->TFC >= UART_FIFO_SIZE) {
        return -1;
    }
    while (1 != uxreg->LSR.tx_fifo_emp);
    uxreg->THR = ch;

    return 1;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev) {
    int ch = -1;
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    volatile struct uart_registers *uxreg = (struct uart_registers *)serial_cfg->hw_cfg.serial_register_base;

    if (1 == uxreg->LSR.data_rdy) {
        /* UART_II_RECV_RDY is cleared by reading RBR */
        ch = (uxreg->RFC > 0) ? uxreg->RBR : -1;
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
    .init = SerialInit,
    .configure = SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done = {
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name,
                              const char *drv_name) {
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
static int BoardSerialDevBend(struct SerialHardwareDevice *serial_device, void *serial_param, const char *bus_name,
                              const char *dev_name) {
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

void UART1_IRQHandler(void) __attribute__((interrupt()));
void UART1_IRQHandler(void) {
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

int InitHwUart(void) {
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

    serial_cfg.hw_cfg.serial_register_base = (uint32)BA_UART1;
    serial_cfg.hw_cfg.serial_irq_interrupt = UART1_IRQn;

    serial_driver_1.private_data = (void *)&serial_cfg;

    serial_dev_param.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param;

    ret = BoardSerialBusInit(&serial_bus, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("InitHwUart uarths error ret %u\n", ret);
        return ERROR;
    }

    /* Configure the serial port */
	GPIOA_SetBits(GPIO_Pin_8);
	GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PU_NSMT);			// RXD-pull-up input
	GPIOA_ModeCfg(GPIO_Pin_8, GPIO_Slowascent_PP_8mA);		// TXD-push-pull output
	UART1_DefInit();
#endif

    return ret;
}

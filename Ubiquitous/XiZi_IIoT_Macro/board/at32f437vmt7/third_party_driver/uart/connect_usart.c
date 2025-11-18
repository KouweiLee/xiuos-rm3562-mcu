/**
 * @file connect_usart.c
 * @brief support stm32l476 usart function and register to bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-02-27
 */

/*************************************************
File name: connect_usart.c
Description: support stm32l476 usart configure and usart bus register function
History:
1. Date: 2025-03-04
Author: AIIT XUOS Lab
Modification:
1. support stm32l476 usart configure, write and read
2. support stm32l476 usart bus device and driver register
*************************************************/

#include <board.h>
#include <connect_usart.h>
#include <dev_serial.h>
#include <at32f435_437_usart.h>
#include <at32f435_437_misc.h>


static void UsartClockInit(void)
{
#ifdef BSP_USING_UART1
    /* enable the usart1 and gpio clock */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
#endif

#ifdef BSP_USING_UART2
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
#endif
}

static void UsartGpioInit(void)
{
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

#ifdef BSP_USING_UART1
    gpio_init_struct.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_7);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_7);
#endif

#ifdef BSP_USING_UART2
    gpio_init_struct.gpio_pins = GPIO_PINS_5 | GPIO_PINS_6;
    gpio_init(GPIOD, &gpio_init_struct);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE5, GPIO_MUX_7);
    gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE6, GPIO_MUX_7);
#endif
}

static void UartIsr(struct SerialBus *serial, struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    if(usart_interrupt_flag_get(serial_hw_cfg->uart_device, USART_RDBF_FLAG) != RESET) {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
    }
}

#ifdef BSP_USING_UART1
struct SerialBus serial_bus_1;
struct SerialDriver serial_driver_1;
struct SerialHardwareDevice serial_device_1;

void USART1_IRQHandler(void)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_1, &serial_driver_1, &serial_device_1);
    ENABLE_INTERRUPT(lock);
}
#endif

#ifdef BSP_USING_UART2
struct SerialBus serial_bus_2;
struct SerialDriver serial_driver_2;
struct SerialHardwareDevice serial_device_2;

void USART2_IRQHandler(void)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_2, &serial_driver_2, &serial_device_2);
    ENABLE_INTERRUPT(lock);
}
#endif

static void SerialCfgParamCheck(struct SerialCfgParam *serial_cfg_default, struct SerialCfgParam *serial_cfg_new)
{
    struct SerialDataCfg *data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &serial_cfg_new->data_cfg;

    if ((data_cfg_default->serial_baud_rate != data_cfg_new->serial_baud_rate) && (data_cfg_new->serial_baud_rate))
    {
        data_cfg_default->serial_baud_rate = data_cfg_new->serial_baud_rate;
    }

    if ((data_cfg_default->serial_bit_order != data_cfg_new->serial_bit_order) && (data_cfg_new->serial_bit_order))
    {
        data_cfg_default->serial_bit_order = data_cfg_new->serial_bit_order;
    }

    if ((data_cfg_default->serial_buffer_size != data_cfg_new->serial_buffer_size) && (data_cfg_new->serial_buffer_size))
    {
        data_cfg_default->serial_buffer_size = data_cfg_new->serial_buffer_size;
    }

    if ((data_cfg_default->serial_data_bits != data_cfg_new->serial_data_bits) && (data_cfg_new->serial_data_bits))
    {
        data_cfg_default->serial_data_bits = data_cfg_new->serial_data_bits;
    }

    if ((data_cfg_default->serial_invert_mode != data_cfg_new->serial_invert_mode) && (data_cfg_new->serial_invert_mode))
    {
        data_cfg_default->serial_invert_mode = data_cfg_new->serial_invert_mode;
    }

    if ((data_cfg_default->serial_parity_mode != data_cfg_new->serial_parity_mode) && (data_cfg_new->serial_parity_mode))
    {
        data_cfg_default->serial_parity_mode = data_cfg_new->serial_parity_mode;
    }

    if ((data_cfg_default->serial_stop_bits != data_cfg_new->serial_stop_bits) && (data_cfg_new->serial_stop_bits))
    {
        data_cfg_default->serial_stop_bits = data_cfg_new->serial_stop_bits;
    }

    if ((data_cfg_default->serial_timeout != data_cfg_new->serial_timeout) && (data_cfg_new->serial_timeout))
    {
        data_cfg_default->serial_timeout = data_cfg_new->serial_timeout;
    }
}

static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
    struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

    if (configure_info->private_data)
    {
        struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
        SerialCfgParamCheck(serial_cfg, serial_cfg_new);

        if (serial_cfg_new->data_cfg.dev_recv_callback)
        {
            BusDevRecvCallback(&(serial_dev->haldev), serial_cfg_new->data_cfg.dev_recv_callback);
        }
    }

    // config serial receive sem timeout
    dev_param->serial_timeout = serial_cfg->data_cfg.serial_timeout;

    /* configure usart param */
    usart_type* usart_x = serial_hw_cfg->uart_device;
    uint32_t baud_rate = serial_cfg->data_cfg.serial_baud_rate;
    usart_data_bit_num_type data_bit;
    usart_stop_bit_num_type stop_bit;
    usart_parity_selection_type parity;

    switch (serial_cfg->data_cfg.serial_data_bits)
    {
        case DATA_BITS_8:
            data_bit = USART_DATA_8BITS;
            break;
        case DATA_BITS_9:
            data_bit = USART_DATA_9BITS;
            break;
        default:
            data_bit = USART_DATA_8BITS;
            break;
    }

    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
        case STOP_BITS_1:
            stop_bit = USART_STOP_1_BIT;
            break;
        case STOP_BITS_2:
            stop_bit = USART_STOP_2_BIT;
            break;
        default:
            stop_bit = USART_STOP_1_BIT;
            break;
    }

    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
        case PARITY_NONE:
            parity = USART_PARITY_NONE;
            break;
        case PARITY_EVEN:
            parity = USART_PARITY_EVEN;
            break;
        case PARITY_ODD:
            parity = USART_PARITY_ODD;
            break;
        default:
            parity = USART_PARITY_NONE;
            break;
    }

    usart_init(usart_x, baud_rate, data_bit, stop_bit);
    usart_parity_selection_config(usart_x, parity);
    usart_transmitter_enable(usart_x, TRUE);
    usart_receiver_enable(usart_x, TRUE);

    usart_flag_clear(usart_x, USART_RDBF_FLAG);
    usart_interrupt_enable(usart_x, USART_RDBF_INT, TRUE);

    usart_enable(usart_x, TRUE);

    nvic_irq_enable(serial_hw_cfg->irq, 1, 0);

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    switch (serial_operation_cmd)
    {
    case OPER_CLR_INT:
        nvic_irq_disable(serial_hw_cfg->irq);
        break;

    case OPER_SET_INT:
        nvic_irq_enable(serial_hw_cfg->irq, 1, 0);
        break;
    }

    return EOK;
}

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

static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char c)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    while(usart_flag_get(serial_hw_cfg->uart_device, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(serial_hw_cfg->uart_device, (uint16_t)c);

    return 1;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;
    int ch = -1;

    if(usart_flag_get(serial_hw_cfg->uart_device, USART_RDBF_FLAG) != RESET)
    {
        ch = usart_data_receive(serial_hw_cfg->uart_device);
    }

    return ch;
}


static const struct SerialDataCfg data_cfg_init =
    {
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

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret)
    {
        KPrintf("HwUartInit SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret)
    {
        KPrintf("HwUartInit SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret)
    {
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
    if (EOK != ret)
    {
        KPrintf("HwUartInit SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret)
    {
        KPrintf("HwUartInit SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }

    return ret;
}

#ifdef BSP_USING_UART1
static int HwUsart1Init(void)
{
    x_err_t ret = EOK;

    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(struct SerialDevParam));

    static struct UsartHwCfg serial_hw_cfg_1;
    memset(&serial_hw_cfg_1, 0, sizeof(struct UsartHwCfg));

    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_cfg_1.data_cfg = data_cfg_init;

    serial_hw_cfg_1.uart_device = USART1;
    serial_hw_cfg_1.irq = USART1_IRQn;

    serial_cfg_1.hw_cfg.private_data = (void *)&serial_hw_cfg_1;
    serial_driver_1.private_data = (void *)&serial_cfg_1;

    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart1 BoardSerialBusInit error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart1 BoardSerialDevBend error ret %u\n", ret);
        return ERROR;
    }

    return ret;
}
#endif


#ifdef BSP_USING_UART2
static int HwUsart2Init(void)
{
    x_err_t ret = EOK;

    static struct SerialCfgParam serial_cfg_2;
    memset(&serial_cfg_2, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_2;
    memset(&serial_dev_param_2, 0, sizeof(struct SerialDevParam));

    static struct UsartHwCfg serial_hw_cfg_2;
    memset(&serial_hw_cfg_2, 0, sizeof(struct UsartHwCfg));

    serial_driver_2.drv_done = &drv_done;
    serial_driver_2.configure = SerialDrvConfigure;
    serial_device_2.hwdev_done = &hwdev_done;

    serial_cfg_2.data_cfg = data_cfg_init;

    serial_hw_cfg_2.uart_device = USART2;
    serial_hw_cfg_2.irq = USART2_IRQn;

    serial_cfg_2.hw_cfg.private_data = (void *)&serial_hw_cfg_2;
    serial_driver_2.private_data = (void *)&serial_cfg_2;

    serial_dev_param_2.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_2.haldev.private_data = (void *)&serial_dev_param_2;

    ret = BoardSerialBusInit(&serial_bus_2, &serial_driver_2, SERIAL_BUS_NAME_2, SERIAL_DRV_NAME_2);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart2 BoardSerialBusInit error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_2, (void *)&serial_cfg_2, SERIAL_BUS_NAME_2, SERIAL_2_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart2 BoardSerialDevBend error ret %u\n", ret);
        return ERROR;
    }

    return ret;
}
#endif

int HwUsartInit(void)
{
    x_err_t ret = EOK;

    UsartClockInit();
    UsartGpioInit();

#ifdef BSP_USING_UART1
    ret = HwUsart1Init();
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart1 error ret %u\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_UART2
    ret = HwUsart2Init();
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart2 error ret %u\n", ret);
        return ERROR;
    }
#endif

    return ret;
}


#if defined(BSP_USING_RS485) || defined(BSP_USING_RS232)
void Rs485Rs232Test(void)
{
    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    x_err_t ret = EOK;
    uint8 write_data[] = "Hello";

    bus = BusFind(SERIAL_BUS_NAME_2);
    dev = BusFindDevice(bus, SERIAL_2_DEVICE_NAME_0);
    drv = BusFindDriver(bus, SERIAL_DRV_NAME_2);

    /*step 1: init bus_driver, change struct SerialCfgParam if necessary*/
    struct BusConfigureInfo configure_info;
    struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));
    configure_info.configure_cmd = OPE_INT;
    configure_info.private_data = &serial_cfg;
    ret = BusDrvConfigure(drv, &configure_info);

    /*step 2: match serial bus_driver with bus_device*/
    ret = bus->match(drv, dev);
    if (EOK != ret) {
        KPrintf("BusMatch failed error code %d\n", ret);
    }

    /*step 3: open bus_device, configure struct SerialDevParam if necessary*/
    ret = BusDevOpen(dev);
    if (EOK != ret) {
        KPrintf("BusDevOpen failed error code %d\n", ret);
    }


    KPrintf("Rs485 or Rs232 send: %s\n", write_data);
    struct BusBlockWriteParam write_param;
    memset(&write_param, 0, sizeof(struct BusBlockWriteParam));
    write_param.buffer = (void *)write_data;
    write_param.size = sizeof(write_data);
    BusDevWriteData(dev, &write_param);

    struct BusBlockReadParam readParam;
    char buffer[256];
    readParam.buffer = (void *)buffer;
    readParam.size = 256;
    readParam.read_length = 0;
    KPrintf("Rs485 or Rs232 recv: ");
    while (1) {
        int res = BusDevReadData(dev, &readParam);
        int i = 0;
        for (i = 0; i < readParam.read_length; i++) {
            KPrintf("0x%02X ", buffer[i]);
            if ('#' == buffer[i]) {
                break;
            }
        }
        if (i != readParam.read_length) {
            break;
        }
    }
    KPrintf("\n");

    BusDevClose(dev);

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                    Rs485Rs232Test, Rs485Rs232Test, rs485 or rs232 write read test);
#endif

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

#if defined(BSP_USING_UART1)
UART_HandleTypeDef huart1;
#endif
#if defined(BSP_USING_UART2)
UART_HandleTypeDef huart2;
#endif
#if defined(BSP_USING_UART3)
UART_HandleTypeDef huart3;
#endif
#if defined(BSP_USING_UART4)
UART_HandleTypeDef huart4;
#endif
#if defined(BSP_USING_UART5)
UART_HandleTypeDef huart5;
#endif

x_err_t Uart_Init(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    x_err_t result = EOK;

    switch ((uint32)huart->Instance)
    {
#ifdef BSP_USING_UART1
    case (uint32)USART1:
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
#endif
#ifdef BSP_USING_UART2
    case (uint32)USART2:
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
#endif
#ifdef BSP_USING_UART3
    case (uint32)USART3:
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PC4     ------> USART3_TX
        PC5     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        break;
#endif
#ifdef BSP_USING_UART4
    case (uint32)UART4:
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PC4     ------> USART3_TX
        PC5     ------> USART3_RX
        */
        /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
#endif
#ifdef BSP_USING_UART5
    case (uint32)UART5:
        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**UART5 GPIO Configuration
        PC12     ------> UART5_TX
        PD2     ------> UART5_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
#endif
    default:
        result = -1;
        break;
    }

    return result;
}

static void UartIsr(struct SerialBus *serial, struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev)
{
    // struct Stm32UsartDma *dma = &serial->dma; 
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    if (__HAL_UART_GET_FLAG(serial_hw_cfg->uart_device, UART_FLAG_RXNE) != RESET) {
        SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
    }
    HAL_UART_IRQHandler(serial_hw_cfg->uart_device);
}

#ifdef BSP_USING_UART1
struct SerialBus serial_bus_1;
struct SerialDriver serial_driver_1;
struct SerialHardwareDevice serial_device_1;

void USART1_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_1, &serial_driver_1, &serial_device_1);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(USART1_IRQn, USART1_IRQHandler, NONE);
#endif

#ifdef BSP_USING_UART2
struct SerialBus serial_bus_2;
struct SerialDriver serial_driver_2;
struct SerialHardwareDevice serial_device_2;
void USART2_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_2, &serial_driver_2, &serial_device_2);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(USART2_IRQn, USART2_IRQHandler, NONE);
#endif

#ifdef BSP_USING_UART3
struct SerialBus serial_bus_3;
struct SerialDriver serial_driver_3;
struct SerialHardwareDevice serial_device_3;

void UART3_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_3, &serial_driver_3, &serial_device_3);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(USART3_IRQn, UART3_IRQHandler, NONE);
#endif

#ifdef BSP_USING_UART4
struct SerialBus serial_bus_4;
struct SerialDriver serial_driver_4;
struct SerialHardwareDevice serial_device_4;

void UART4_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_4, &serial_driver_4, &serial_device_4);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(UART4_IRQn, UART4_IRQHandler, NONE);
#endif

#ifdef BSP_USING_UART5
struct SerialBus serial_bus_5;
struct SerialDriver serial_driver_5;
struct SerialHardwareDevice serial_device_5;

void UART5_IRQHandler(int irq_num, void *arg)
{
    x_base lock = 0;
    lock = DISABLE_INTERRUPT();
    UartIsr(&serial_bus_5, &serial_driver_5, &serial_device_5);
    ENABLE_INTERRUPT(lock);
}
DECLARE_HW_IRQ(UART5_IRQn, UART5_IRQHandler, NONE);
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

    UART_HandleTypeDef *huart = serial_hw_cfg->uart_device;
    UART_InitTypeDef *uart_init = &huart->Init;

    uart_init->OverSampling = UART_OVERSAMPLING_16;
    uart_init->BaudRate = serial_cfg->data_cfg.serial_baud_rate;
    uart_init->Mode = UART_MODE_TX_RX;
    switch (serial_cfg->data_cfg.serial_data_bits)
    {
    case DATA_BITS_8:
        uart_init->WordLength = UART_WORDLENGTH_8B;
        break;
    case DATA_BITS_9:
        uart_init->WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        uart_init->WordLength = UART_WORDLENGTH_8B;
        break;
    }

    switch (serial_cfg->data_cfg.serial_stop_bits)
    {
    case STOP_BITS_1:
        uart_init->StopBits = UART_STOPBITS_1;
        break;
    case STOP_BITS_2:
        uart_init->StopBits = UART_STOPBITS_2;
        break;
    default:
        uart_init->StopBits = UART_STOPBITS_1;
        break;
    }

    switch (serial_cfg->data_cfg.serial_parity_mode)
    {
    case PARITY_NONE:
        uart_init->Parity = UART_PARITY_NONE;
        break;
    case PARITY_EVEN:
        uart_init->Parity = UART_PARITY_EVEN;
        break;
    case PARITY_ODD:
        uart_init->Parity = UART_PARITY_ODD;
        break;
    default:
        uart_init->Parity = UART_PARITY_NONE;
        break;
    }

    if (BIT_ORDER_LSB == serial_cfg->data_cfg.serial_bit_order)
    {
        huart->AdvancedInit.MSBFirst = UART_ADVFEATURE_MSBFIRST_DISABLE;
    }
    else
    {
        huart->AdvancedInit.MSBFirst = UART_ADVFEATURE_MSBFIRST_ENABLE;
    }

    /* Enable USART clock */
    // FCG_USART_CLK(serial_hw_cfg->usart_clock, ENABLE);
    Uart_Init(serial_hw_cfg->uart_device);
    if (EOK != HAL_UART_Init(serial_hw_cfg->uart_device))
    {
        return ERROR;
    }

    /* Enable error interrupt */
    __HAL_UART_ENABLE_IT(serial_hw_cfg->uart_device, UART_IT_RXNE);
    HAL_NVIC_SetPriority(serial_hw_cfg->irq, 1, 2);
    HAL_NVIC_EnableIRQ(serial_hw_cfg->irq);

    // HAL_UART_Transmit(huart, (uint8_t *)"Hello World\n", 12, 1000);

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
        HAL_NVIC_DisableIRQ(serial_hw_cfg->irq);
        break;

    case OPER_SET_INT:
        HAL_NVIC_EnableIRQ(serial_hw_cfg->irq);
        break;
    }

    return EOK;
}

static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char c)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    /* Polling mode. */
    HAL_UART_Transmit(serial_hw_cfg->uart_device, (uint8_t *)&c, 1, 100);

    return 1;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    struct UsartHwCfg *serial_hw_cfg = (struct UsartHwCfg *)serial_cfg->hw_cfg.private_data;

    int ch = -1;
    if (serial_hw_cfg->uart_device->Instance->ISR & UART_FLAG_RXNE) {
        ch = serial_hw_cfg->uart_device->Instance->RDR & 0xff;
    }
    return ch;
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

int HwUsartInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_UART1
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

    // default irq configure
    huart1.Instance = USART1;
    serial_hw_cfg_1.uart_device = &huart1;
    serial_hw_cfg_1.irq = USART1_IRQn;

    serial_cfg_1.hw_cfg.private_data = (void *)&serial_hw_cfg_1;
    serial_driver_1.private_data = (void *)&serial_cfg_1;

    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart1 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart1 error ret %u\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_UART2
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

    // default irq configure
    huart2.Instance = USART2;
    serial_hw_cfg_2.uart_device = &huart2;
    serial_hw_cfg_2.irq = USART2_IRQn;

    serial_cfg_2.hw_cfg.private_data = (void *)&serial_hw_cfg_2;
    serial_driver_2.private_data = (void *)&serial_cfg_2;

    serial_dev_param_2.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_2.haldev.private_data = (void *)&serial_dev_param_2;

    ret = BoardSerialBusInit(&serial_bus_2, &serial_driver_2, SERIAL_BUS_NAME_2, SERIAL_DRV_NAME_2);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart2 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_2, (void *)&serial_cfg_2, SERIAL_BUS_NAME_2, SERIAL_2_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart2 error ret %u\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_UART3
    static struct SerialCfgParam serial_cfg_3;
    memset(&serial_cfg_3, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_3;
    memset(&serial_dev_param_3, 0, sizeof(struct SerialDevParam));

    static struct UsartHwCfg serial_hw_cfg_3;
    memset(&serial_hw_cfg_3, 0, sizeof(struct UsartHwCfg));

    serial_driver_3.drv_done = &drv_done;
    serial_driver_3.configure = SerialDrvConfigure;
    serial_device_3.hwdev_done = &hwdev_done;

    serial_cfg_3.data_cfg = data_cfg_init;

    // default irq configure
    huart3.Instance = USART3;
    serial_hw_cfg_3.uart_device = &huart3;
    serial_hw_cfg_3.irq = USART3_IRQn;

    serial_cfg_3.hw_cfg.private_data = (void *)&serial_hw_cfg_3;
    serial_driver_3.private_data = (void *)&serial_cfg_3;

    serial_dev_param_3.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_3.haldev.private_data = (void *)&serial_dev_param_3;

    ret = BoardSerialBusInit(&serial_bus_3, &serial_driver_3, SERIAL_BUS_NAME_3, SERIAL_DRV_NAME_3);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart3 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_3, (void *)&serial_cfg_3, SERIAL_BUS_NAME_3, SERIAL_3_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart3 error ret %u\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_UART4
    static struct SerialCfgParam serial_cfg_4;
    memset(&serial_cfg_4, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_4;
    memset(&serial_dev_param_4, 0, sizeof(struct SerialDevParam));

    static struct UsartHwCfg serial_hw_cfg_4;
    memset(&serial_hw_cfg_4, 0, sizeof(struct UsartHwCfg));

    serial_driver_4.drv_done = &drv_done;
    serial_driver_4.configure = SerialDrvConfigure;
    serial_device_4.hwdev_done = &hwdev_done;

    serial_cfg_4.data_cfg = data_cfg_init;

    // default irq configure
    huart4.Instance = UART4;
    serial_hw_cfg_4.uart_device = &huart4;
    serial_hw_cfg_4.irq = UART4_IRQn;

    serial_cfg_4.hw_cfg.private_data = (void *)&serial_hw_cfg_4;
    serial_driver_4.private_data = (void *)&serial_cfg_4;

    serial_dev_param_4.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_4.haldev.private_data = (void *)&serial_dev_param_4;

    ret = BoardSerialBusInit(&serial_bus_4, &serial_driver_4, SERIAL_BUS_NAME_4, SERIAL_DRV_NAME_4);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart3 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_4, (void *)&serial_cfg_4, SERIAL_BUS_NAME_4, SERIAL_4_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart3 error ret %u\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_UART5
    static struct SerialCfgParam serial_cfg_5;
    memset(&serial_cfg_5, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_5;
    memset(&serial_dev_param_5, 0, sizeof(struct SerialDevParam));

    static struct UsartHwCfg serial_hw_cfg_5;
    memset(&serial_hw_cfg_5, 0, sizeof(struct UsartHwCfg));

    serial_driver_5.drv_done = &drv_done;
    serial_driver_5.configure = SerialDrvConfigure;
    serial_device_5.hwdev_done = &hwdev_done;

    serial_cfg_5.data_cfg = data_cfg_init;

    // default irq configure
    huart5.Instance = UART5;
    serial_hw_cfg_5.uart_device = &huart5;
    serial_hw_cfg_5.irq = UART5_IRQn;

    serial_cfg_5.hw_cfg.private_data = (void *)&serial_hw_cfg_5;
    serial_driver_5.private_data = (void *)&serial_cfg_5;

    serial_dev_param_5.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_5.haldev.private_data = (void *)&serial_dev_param_5;

    ret = BoardSerialBusInit(&serial_bus_5, &serial_driver_5, SERIAL_BUS_NAME_5, SERIAL_DRV_NAME_5);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart5 error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_5, (void *)&serial_cfg_5, SERIAL_BUS_NAME_5, SERIAL_5_DEVICE_NAME_0);
    if (EOK != ret)
    {
        KPrintf("HwUartInit uart5 error ret %u\n", ret);
        return ERROR;
    }
#endif

    return ret;
}


#ifdef TEST_RS485
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

void RS485Test(void)
{
    x_err_t ret = EOK;

    bus = BusFind(SERIAL_BUS_NAME_4);
    dev = BusFindDevice(bus, SERIAL_4_DEVICE_NAME_0);
    drv = BusFindDriver(bus, SERIAL_DRV_NAME_4);

    struct BusConfigureInfo configure_info;
    configure_info.configure_cmd = OPE_INT;
    struct SerialCfgParam serial_cfg;
    memset(&serial_cfg, 0, sizeof(struct SerialCfgParam));
    configure_info.private_data = &serial_cfg;
    ret = BusDrvConfigure(drv, &configure_info);

    struct BusBlockWriteParam write_param;
    memset(&write_param, 0, sizeof(struct BusBlockWriteParam));

    uint8 write_data[] = "Message from stm32L476 by RS485(UART5)";

    write_param.buffer = (void *)write_data;
    write_param.size = sizeof(write_data);
    BusDevWriteData(dev, &write_param);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                    RS485Test, RS485Test, open uart device and use rs485 write_read parameters);
#endif

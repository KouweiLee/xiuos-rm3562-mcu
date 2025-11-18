#include "board.h"
#include "bus_serial.h"
#include "dev_serial.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal.h"
#include "connect_uart.h"


static UART_HandleTypeDef huart1;
static struct SerialBus serial_bus_1;
static struct SerialDriver serial_driver_1;
static struct SerialHardwareDevice serial_device_1;

static uint8_t rx_ch;

// === putchar / getchar 接口 ===
static int HalSerialPutchar(struct SerialHardwareDevice *serial_dev, char c)
{
    uint8_t ret = -ERROR;
    uint8_t ch = (uint8_t)c;
    if(serial_dev == &serial_device_1){
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
        ret = (status == HAL_OK) ? EOK : ERROR;
    }

    return ret;
}
static int SerialGetchar(struct SerialHardwareDevice *serial_dev)
{
    if (USART1->ISR & USART_ISR_RXNE)
    {
        return (int)(USART1->RDR & 0xFF);
    }
    return -ERROR;
}

// === HAL 串口初始化 ===
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart1);
}


// === GPIO + RCC + NVIC 初始化 ===
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1){
        __HAL_RCC_USART1_CLK_ENABLE();
        // __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        __HAL_RCC_USART1_CONFIG(RCC_USART1CLKSOURCE_PCLK2);

        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* USART1 RX = PC5 */
        GPIO_InitStruct.Pin = GPIO_USART1_RX | GPIO_USART1_TX;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIO_USART1_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

// === XiUOS 驱动接口 ===
static uint32 HalSerialInit(struct SerialDriver *drv, struct BusConfigureInfo *info)
{
    return EOK; // 初始化已在 InitUart 中完成
}

static uint32 HalSerialConfigure(struct SerialDriver *drv, int cmd)
{
    if (cmd == OPER_SET_INT) {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    } else if (cmd == OPER_CLR_INT) {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    }
    return EOK;
}


// === 固定参数表 ===
static const struct SerialDataCfg data_cfg_init = {
    .serial_baud_rate = 115200,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
    .serial_timeout = WAITING_FOREVER,
};

static const struct SerialDrvDone drv_done = {
    .init = HalSerialInit,
    .configure = HalSerialConfigure,
};

static struct SerialHwDevDone hwdev_done = {
    .put_char = HalSerialPutchar,
    .get_char = SerialGetchar,
};

int InitUART(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_UART1
    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1,       0, sizeof(serial_cfg_1));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(serial_dev_param_1));

    MX_USART1_UART_Init();

    serial_cfg_1.data_cfg = data_cfg_init;
    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.private_data = &serial_cfg_1;

    serial_device_1.private_data = &huart1;
    serial_device_1.hwdev_done = &hwdev_done;
    serial_device_1.haldev.private_data = &serial_dev_param_1;
    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;

    ret = SerialBusInit(&serial_bus_1, SERIAL1_BUS_NAME);
    if (ret != EOK) return ret;

    ret = SerialDriverInit(&serial_driver_1, SERIAL1_DRV_NAME);
    if (ret != EOK) return ret;

    ret = SerialDriverAttachToBus(SERIAL1_DRV_NAME, SERIAL1_BUS_NAME);
    if (ret != EOK) return ret;

    ret = SerialDeviceRegister(&serial_device_1, &serial_cfg_1, SERIAL1_DEVICE_NAME);
    if (ret != EOK) return ret;

    ret = SerialDeviceAttachToBus(SERIAL1_DEVICE_NAME, SERIAL1_BUS_NAME);
    if (ret != EOK) return ret;

#endif
    return ret;
}



// === 中断服务函数 ===
void USART1_IRQHandler(int irq_num, void *arg)
{
    // SerialSetIsr(&serial_device_1, SERIAL_EVENT_RX_IND);

}
DECLARE_HW_IRQ(USART1_IRQn, USART1_IRQHandler, NONE);

#include "board.h"
#include <device.h>
#include "connect_spi.h"
#include "stm32g4xx_hal_spi.h"
#include "bus_spi.h"
#include "dev_spi.h"

/* ===================== SPI1 ===================== */
SPI_HandleTypeDef hspi1;

/**SPI1 GPIO Configuration
* PB3     ------> SPI1_SCK
* PA6     ------> SPI1_MISO
* PA7     ------> SPI1_MOSI
* PD2     ------> SPI1_CS

*/

#define ESC_RD          0x02
#define ESC_WR          0x04

typedef union {
    uint16_t Word;
    uint8_t  Byte[2];
} UBYTETOWORD;

static uint8_t tx_buf[4] = {0x30, 0x82, 0x00, 0x00};
static uint8_t rx_buf[4] = {0};

static struct SpiDataStandard spi1_data_std = 
{
    .spi_chip_select = 0,
    .spi_cs_release = 1,

    .tx_buff = tx_buf,
    .tx_len = 4,

    .rx_buff = rx_buf,
    .rx_len = 4,

    .length = 4,
    .next = NULL
};



/* ===================== SPI 外设时钟选择（PLLQ） ===================== */
void SPIClock_Config(void)
{
// 

}

/* ===================== XiUOS 设备 I/O 实现 ===================== */

static uint32 SpiOpen(struct SpiHardwareDevice *dev)
{
    KPrintf("SPI open\n");
    return EOK;
}

static uint32 SpiClose(struct SpiHardwareDevice *dev)
{
    KPrintf("SPI close\n");
    return EOK;
}

static void Spi_Select(uint8 spi_chip_select)
{
    HAL_GPIO_WritePin(GPIO_SPI1_CS_PORT, GPIO_SPI1_CS_PIN, GPIO_PIN_RESET);
}

static void Spi_Release(uint8 spi_chip_select)
{
    HAL_GPIO_WritePin(GPIO_SPI1_CS_PORT, GPIO_SPI1_CS_PIN, GPIO_PIN_SET);
}

static uint32 SpiTransmitMsg(struct SpiHardwareDevice *dev, struct SpiDataStandard *msg)
{
    if (!dev || !msg || !msg->tx_buff || msg->tx_len == 0)
        return ERROR;

    Spi_Select(msg->spi_chip_select);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit((SPI_HandleTypeDef *)dev->private_data,
                                             (uint8_t *)msg->tx_buff,
                                             msg->tx_len,
                                             HAL_MAX_DELAY);

    if (msg->spi_cs_release)
        Spi_Release(msg->spi_chip_select);

    return (ret == HAL_OK) ? EOK : ERROR;
}

static uint32 SpiRecvMsg(struct SpiHardwareDevice *dev, struct SpiDataStandard *msg)
{
    if (!dev || !msg || !msg->rx_buff || msg->rx_len == 0)
        return ERROR;

    Spi_Select(msg->spi_chip_select);

    HAL_StatusTypeDef ret = HAL_SPI_Receive((SPI_HandleTypeDef *)dev->private_data,
                                            msg->rx_buff,
                                            msg->rx_len,
                                            HAL_MAX_DELAY);

    if (msg->spi_cs_release)
        Spi_Release(msg->spi_chip_select);

    return (ret == HAL_OK) ? EOK : ERROR;
}

/* 一次完整的 SPI 事务：发送 tx，接收 rx */
uint32 SpiTransmitReceive(struct SpiHardwareDevice *dev, struct SpiDataStandard *msg)
{
    if (!dev || !msg || !msg->tx_buff || !msg->rx_buff || msg->length == 0)
        return ERROR;

    Spi_Select(msg->spi_chip_select);
    for(volatile int i=0; i<500; i++);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(
        (SPI_HandleTypeDef *)dev->private_data,     // SPI 外设句柄
        (uint8_t *)msg->tx_buff,                    // 发送缓冲区
        (uint8_t *)msg->rx_buff,                    // 接收缓冲区
        msg->length,                                // 传输长度（字节）
        HAL_MAX_DELAY                               // 超时
    );
    if(ret != HAL_OK){
        KPrintf("Msg sent error\n");
    }

    if (msg->spi_cs_release)
        Spi_Release(msg->spi_chip_select);

    return (ret == HAL_OK) ? EOK : ERROR;
}

uint8_t ESC_ReadReg(struct SpiHardwareDevice *dev, uint16_t address)
{
    struct SpiDataStandard msg;
    uint8_t tx_buf[4] = {0};   // 发送缓存
    uint8_t rx_buf[4] = {0};   // 接收缓存
    UBYTETOWORD tmp;

    // 处理地址和命令
    tmp.Word = (address << 3) | ESC_RD;

    tx_buf[0] = tmp.Byte[1];   // 高字节
    tx_buf[1] = tmp.Byte[0];   // 低字节
    tx_buf[2] = 0x00;          // 占位，主机丢弃
    tx_buf[3] = 0x00;          // dummy，实际接收数据

    // 配置 SPI 消息
    msg.tx_buff = tx_buf;
    msg.rx_buff = rx_buf;
    msg.length  = 4;          // 一共发 4 字节
    msg.spi_chip_select = 0;  // 根据实际片选编号
    msg.spi_cs_release  = 1;  // 完成后释放 CS

    SpiTransmitReceive(dev, &msg);

    // rx_buf[3] 就是返回的寄存器内容
    return rx_buf[2];
}

/* ===================== 驱动/设备回调表 ===================== */

struct SpiDevDone spi_dev_done = {
    .dev_open  = SpiOpen,
    .dev_close = SpiClose,
    .dev_write = SpiTransmitMsg,
    .dev_read  = SpiRecvMsg,
};

static uint32 HalSpiInit()
{
    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    // hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;

    hspi1.Init.NSS = SPI_NSS_SOFT;
    // hspi1.Init.NSS = SPI_NSS_HARD_INPUT;

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

    return EOK;
}

/* ===================== MSP：GPIO/IRQ ===================== */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIO_SPI1_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_SPI1_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIO_SPI1_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_SPI1_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIO_SPI1_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_SPI1_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIO_SPI1_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIO_SPI1_CS_PORT, GPIO_SPI1_CS_PIN, GPIO_PIN_SET); // 空闲拉高（低有效）
    
    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

static uint32 HalSpiConfigure()
{

    return EOK;
}

static uint32 HalSpiDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    struct SpiDriver *spi_drv = (struct SpiDriver *)drv;

    switch(configure_info->configure_cmd){
        case OPE_INT:
            return HalSpiInit();
            break;
        case OPE_CFG:
            return HalSpiConfigure();
            break;
        default:
            break;
    }
 
    return EOK;
}



/* ===================== Bus/Driver/Device 装配工具 ===================== */

static int BoardSpiBusInit(struct SpiBus* spi_bus, struct SpiDriver* spi_driver)
{
    x_err_t ret = EOK;
    ret = SpiBusInit(spi_bus, SPI1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("Board_Spi_init SpiBusInit error %d\n", ret);
        return ERROR;
    }
    ret = SpiDriverInit(spi_driver, SPI1_DRV_NAME);
     if (ret != EOK) {
        KPrintf("Board_Spi_init SpiDriverInit error %d\n", ret);
        return ERROR;
    }

    ret = SpiDriverAttachToBus(SPI1_DRV_NAME, SPI1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("Board_Spi_init SpiDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

static x_err_t HwSpiDeviceAttach(const char *bus_name,
                                 const char *device_name)
{
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(device_name);
    x_err_t ret;

    struct SpiHardwareDevice *spi_device =
        (struct SpiHardwareDevice *)x_malloc(sizeof(struct SpiHardwareDevice));
    if (!spi_device) return ERROR;

    memset(spi_device, 0, sizeof(struct SpiHardwareDevice));
    spi_device->spi_dev_done = &spi_dev_done;
    spi_device->private_data = &hspi1;

    ret = SpiDeviceRegister(spi_device, NONE, device_name);
    if (ret != EOK) { 
        SYS_ERR("%s device %p register faild, %d\n", device_name, spi_device, ret);
        x_free(spi_device); return ERROR; 
    }   

    ret = SpiDeviceAttachToBus(device_name, bus_name);
    if (ret != EOK) { 
        SYS_ERR("%s attach to %s faild, %d\n", device_name, bus_name, ret);
        x_free(spi_device); return ERROR; 
    }   
    
    return ret;
}

int InitSPI(void)
{
    x_err_t ret = EOK;
    SPIClock_Config();
    
    static struct SpiBus spi_bus_1;
    static struct SpiDriver spi_driver_1;

    memset(&spi_driver_1, 0, sizeof(struct SpiDriver));

    spi_driver_1.configure            = HalSpiDrvConfigure;
    spi_driver_1.driver.configure     = HalSpiDrvConfigure;
    spi_driver_1.driver.private_data  = &hspi1;

    ret = BoardSpiBusInit(&spi_bus_1, &spi_driver_1);
    if (ret != EOK) return ERROR;

    ret = HwSpiDeviceAttach(SPI1_BUS_NAME, SPI1_DEVICE_NAME);
    if (ret != EOK) return ERROR;

    struct BusConfigureInfo bi1 = { .configure_cmd = OPE_INT, .private_data = NULL };
    spi_driver_1.configure(&spi_driver_1, &bi1);

    int set_int_cmd = OPER_SET_INT;
    struct BusConfigureInfo bi1_int = { .configure_cmd = OPE_CFG, .private_data = &set_int_cmd };
    spi_driver_1.configure(&spi_driver_1, &bi1_int);

    return ret;
}


void SPI1_IRQHandler(int irq_num, void *arg)
{
    HAL_SPI_IRQHandler(&hspi1);
}
DECLARE_HW_IRQ(SPI1_IRQn, SPI1_IRQHandler, NONE);


void TestSPI(void){
    struct SpiHardwareDevice *spi_dev = NULL;
    spi_dev = (struct SpiHardwareDevice *)SpiDeviceFind(SPI1_DEVICE_NAME, TYPE_SPI_DEV);
    // 待测试的 ESC 寄存器地址列表
    uint16_t test_addrs[] = {
        0x0000, // ESC ID
        0x0004, // ESC Control
        0x0130, // AL Status
        0x0220, // AL Event
        0x100,  // Device Type / Config
        0x101,  // Version Minor
        0x102,  // Version Major
        0x1F00, // Sync Manager 0 配置
        0x1F01, // Sync Manager 1 配置
        0x1F02  // PDO 配置示例
    };

    int num_addrs = sizeof(test_addrs) / sizeof(test_addrs[0]);

    KPrintf("==== ESC SPI Test Start ====\n");
    for(int i = 0; i < num_addrs; i++){
        uint8_t value = ESC_ReadReg(spi_dev, test_addrs[i]);
        KPrintf("Read ESC reg[0x%04X] = 0x%02X\n", test_addrs[i], value);
    }  
}

void TestSpiTransmit(void){
    struct SpiHardwareDevice *spi_dev = NULL;
    spi_dev = (struct SpiHardwareDevice *)SpiDeviceFind(SPI1_DEVICE_NAME, TYPE_SPI_DEV);
    if (SpiTransmitMsg(spi_dev, &spi1_data_std) == EOK) {
        KPrintf("SUCCESSFULLY Transmit. \n");
        
    } else {
        KPrintf("SPI transaction failed\n");
    }       
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    TestSPI, TestSPI, test spi);

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    TestSpiTransmit, TestSpiTransmit, test transmission);


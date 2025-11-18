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
* @file connect_spi.c
* @brief support stm32l476 spi function and register to bus framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2025-03-11
*/

/*************************************************
File name: connect_spi.c
Description: support stm32l476 spi function and register to bus framework
Others: 
History: 
1. Date: 2025-03-11
Author: AIIT XUOS Lab
Modification: 
1. support stm32l476 spi configure, write and read
2. support stm32l476 spi bus device and driver register
*************************************************/

#include <connect_spi.h>

/*Init the spi sdk intetface */
static uint32 SpiSdkInit(struct SpiDriver *spi_drv)
{
    NULL_PARAM_CHECK(spi_drv);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_drv->driver.private_data);

    SPI_HandleTypeDef *hspi = spi_drv->driver.owner_bus->private_data;

    struct SpiSlaveParam *slaveparam = dev_param->spi_slave_param;
    switch ((uint32)hspi->Instance)
    {
    
#ifdef BSP_USING_SPI2
    case (uint32)SPI2:
        __HAL_RCC_SPI2_CLK_ENABLE();
    
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**SPI2 GPIO Configuration    
        PC2     ------> SPI2_MISO
        PC3     ------> SPI2_MOSI
        PB10     ------> SPI2_SCK 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin :  CS port */
        GPIO_InitStruct.Pin = slaveparam->spi_cs_gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)slaveparam->spi_cs_gpio_port;
        HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

        HAL_GPIO_WritePin(gpio_port, slaveparam->spi_cs_gpio_pin, GPIO_PIN_SET);

        hspi->Init.Mode = SPI_MODE_MASTER;
        hspi->Init.Direction = SPI_DIRECTION_2LINES;
        hspi->Init.DataSize = (dev_param->spi_master_param->spi_data_bit_width- 1 ) << 8  ;
        // mode
        
        hspi->Init.NSS = SPI_NSS_SOFT;

        if (SPI_MODE_0 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        } else if (SPI_MODE_1 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        } else if (SPI_MODE_2 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        } else if (SPI_MODE_3 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        }

        if (dev_param->spi_master_param->spi_work_mode & 0x4)
        {
            hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
        } else {
            hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;
        }

        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        
        hspi->Init.TIMode = SPI_TIMODE_DISABLE;

        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

        hspi->Init.CRCPolynomial = 7;
        hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
        hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
        int ret = HAL_SPI_Init(hspi);
        if (ret != HAL_OK)
        {
            KPrintf("HwSPIInit SPI2 error ret %u\n", ret);
        }
        break;
#endif

#ifdef BSP_USING_SPI3
    case (uint32)SPI3:
        __HAL_RCC_SPI3_CLK_ENABLE();
    
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**SPI3 GPIO Configuration    
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PB5     ------> SPI3_MOSI 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin : CS port */
        GPIO_InitStruct.Pin = slaveparam->spi_cs_gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)slaveparam->spi_cs_gpio_port;
        HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

        HAL_GPIO_WritePin(gpio_port, slaveparam->spi_cs_gpio_pin, GPIO_PIN_SET);

        hspi->Init.Mode = SPI_MODE_MASTER;
        hspi->Init.Direction = SPI_DIRECTION_2LINES;
        hspi->Init.DataSize = (dev_param->spi_master_param->spi_data_bit_width- 1 ) << 8  ;
        // mode
        
        hspi->Init.NSS = SPI_NSS_SOFT;

        if (SPI_MODE_0 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        } else if (SPI_MODE_1 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        } else if (SPI_MODE_2 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        } else if (SPI_MODE_3 == dev_param->spi_master_param->spi_work_mode & 0x3) {
            hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        }

        if (dev_param->spi_master_param->spi_work_mode & 0x4)
        {
            hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
        } else {
            hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;
        }

        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        
        hspi->Init.TIMode = SPI_TIMODE_DISABLE;

        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

        hspi->Init.CRCPolynomial = 7;
        hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
        hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
        int ret = HAL_SPI_Init(hspi);
        if (ret != HAL_OK)
        {
            KPrintf("HwSPIInit SPI2 error ret %u\n", ret);
        }
        break;
#endif
    default:
        break;
    }

    return EOK;
}

static uint32 SpiSdkCfg(struct SpiDriver *spi_drv, struct SpiMasterParam *spi_param)
{
    NULL_PARAM_CHECK(spi_drv);
    NULL_PARAM_CHECK(spi_param);  

    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_drv->driver.private_data);

    dev_param->spi_master_param = spi_param;
    dev_param->spi_master_param->spi_work_mode = dev_param->spi_master_param->spi_work_mode & SPI_MODE_MASK;

    return EOK;
}

/*Configure the spi device param, make sure struct (configure_info->private_data) = (SpiMasterParam)*/
static uint32 SpiDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    struct SpiDriver *spi_drv = (struct SpiDriver *)drv;
    struct SpiMasterParam *spi_param;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = SpiSdkInit(spi_drv);
            break;
        case OPE_CFG:
            spi_param = (struct SpiMasterParam *)configure_info->private_data;
            ret = SpiSdkCfg(spi_drv, spi_param);
            break;
        default:
            break;
    }

    return ret;
}

static uint32 SpiWriteData(struct SpiHardwareDevice *spi_dev, struct SpiDataStandard *spi_datacfg)
{
    x_size_t message_length, already_send_length;
    uint16 send_length;
    const uint8 *WriteBuf;
    int state;

    NULL_PARAM_CHECK(spi_dev);
    NULL_PARAM_CHECK(spi_datacfg);
    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_dev->haldev.private_data);
    struct SpiSlaveParam *slaveparam = dev_param->spi_slave_param;
    SPI_HandleTypeDef *hspi = spi_dev->haldev.owner_bus->private_data;

    GPIO_TypeDef *slave_port = (GPIO_TypeDef *)slaveparam->spi_cs_gpio_port;
    uint16_t slave_pin = slaveparam->spi_cs_gpio_pin;

    uint8 device_id = dev_param->spi_slave_param->spi_slave_id;
    uint8 device_master_id = dev_param->spi_dma_param->spi_master_id;
    uint8 cs_gpio_pin = dev_param->spi_slave_param->spi_cs_gpio_pin;

    while (NONE != spi_datacfg)
    {
        if (spi_datacfg->spi_chip_select) {
            HAL_GPIO_WritePin(slave_port, slave_pin, GPIO_PIN_RESET);
        }

        message_length = spi_datacfg->length;
        WriteBuf = spi_datacfg->tx_buff;
        while (message_length) {
            if (message_length > 65535){
                send_length = 65535;
                message_length = message_length - 65535;
            } else {
                send_length = message_length;
                message_length = 0;
            }
            /* calculate the start address */
            already_send_length = spi_datacfg->length - send_length - message_length;
            WriteBuf = (uint8 *)spi_datacfg->tx_buff + already_send_length;
            if (spi_datacfg->tx_buff) {
                state = HAL_SPI_Transmit(hspi, (uint8_t *)WriteBuf, send_length, 1000);
            }

            if (state != HAL_OK) {
                KPrintf("spi write error : %d\n", state);
                spi_datacfg->length = 0;
            }
        }

        if (spi_datacfg->spi_cs_release) {
            HAL_GPIO_WritePin(slave_port, slave_pin, GPIO_PIN_SET);
        }

        spi_datacfg = spi_datacfg->next;
    }
    return 1;
}

static uint32 SpiReadData(struct SpiHardwareDevice *spi_dev, struct SpiDataStandard *spi_datacfg)
{
    int state;
    x_size_t message_length, already_send_length;
    uint16 read_length, spi_read_length = 0;
    const uint8 *ReadBuf;

    NULL_PARAM_CHECK(spi_dev);
    NULL_PARAM_CHECK(spi_datacfg);

    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_dev->haldev.private_data);
    struct SpiSlaveParam *slaveparam = dev_param->spi_slave_param;
    SPI_HandleTypeDef *hspi = spi_dev->haldev.owner_bus->private_data;

    GPIO_TypeDef *slave_port = (GPIO_TypeDef *)slaveparam->spi_cs_gpio_port;
    uint16_t slave_pin = slaveparam->spi_cs_gpio_pin;

    while (NONE != spi_datacfg) {
        if (spi_datacfg->spi_chip_select) {
            HAL_GPIO_WritePin(slave_port, slave_pin, GPIO_PIN_RESET);
        }

        message_length = spi_datacfg->length;
        ReadBuf = spi_datacfg->rx_buff;

        while (message_length) {
            if (message_length > 65535) {
                read_length = 65535;
                message_length = message_length - 65535;
            } else {
                read_length = message_length;
                message_length = 0;
            }
    
            /* calculate the start address */
            already_send_length = spi_datacfg->length - read_length - message_length;
            ReadBuf = (uint8 *)spi_datacfg->rx_buff + already_send_length;
    
            /* start once data exchange in DMA mode */
            if (spi_datacfg->rx_buff) {
                HAL_SPI_Receive(hspi, (uint8_t *)ReadBuf, read_length, 1000);
            }
    
            if (state != 0) {
                KPrintf("spi read error : %d\n", state);
                spi_datacfg->length = 0;
            }
        }

        if (spi_datacfg->spi_cs_release) {
            HAL_GPIO_WritePin(slave_port, slave_pin, GPIO_PIN_SET);
        }
        spi_read_length += spi_datacfg->length;
        spi_datacfg = spi_datacfg->next;
    }
    return spi_read_length;
}

/*manage the spi device operations*/
static const struct SpiDevDone spi_dev_done =
{
    .dev_open = NONE,
    .dev_close = NONE,
    .dev_write = SpiWriteData,
    .dev_read = SpiReadData,
};

static int BoardSpiBusInit(struct SpiBus *spi_bus, struct SpiDriver *spi_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the spi bus */
    ret = SpiBusInit(spi_bus, bus_name);
    if (EOK != ret) {
        KPrintf("Board_Spi_init SpiBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the spi driver*/
    ret = SpiDriverInit(spi_driver, drv_name);
    if (EOK != ret) {
        KPrintf("Board_Spi_init SpiDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the spi driver to the spi bus*/
    ret = SpiDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("Board_Spi_init SpiDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    // SpiSdkInit(spi_driver);

    return ret;
}

/*Attach the spi device to the spi bus*/
static int BoardSpiDevBend(void)
{
    x_err_t ret = EOK;

#ifdef SPI_2_DEVICE_NAME_0
    static struct SpiHardwareDevice spi2_device0;
    memset(&spi2_device0, 0, sizeof(struct SpiHardwareDevice));

    // SPI_HandleTypeDef hspi2;
    // hspi2.Instance = SPI2;
    // spi2_device0.private_data = &hspi2;

    static struct SpiSlaveParam spi2_slaveparam0;
    memset(&spi2_slaveparam0, 0, sizeof(struct SpiSlaveParam));

    // spi2_slaveparam0.spi_slave_id = SPI1_DEVICE_SLAVE_ID_0;
    spi2_slaveparam0.spi_cs_gpio_pin = GPIO_PIN_0;
    spi2_slaveparam0.spi_cs_gpio_port = GPIOC_BASE;

    spi2_device0.spi_param.spi_slave_param = &spi2_slaveparam0;

    spi2_device0.spi_dev_done = &(spi_dev_done);

    ret = SpiDeviceRegister(&spi2_device0, (void *)(&spi2_device0.spi_param), SPI_2_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("BoardSpiDevBend SpiDeviceRegister device %s error %d\n", SPI_2_DEVICE_NAME_0, ret);
        return ERROR;
    }  

    ret = SpiDeviceAttachToBus(SPI_2_DEVICE_NAME_0, SPI_BUS_NAME_2);
    if (EOK != ret) {
        KPrintf("BoardSpiDevBend SpiDeviceAttachToBus device %s error %d\n", SPI_2_DEVICE_NAME_0, ret);
        return ERROR;
    }  
#endif

#ifdef SPI_3_DEVICE_NAME_0
    static struct SpiHardwareDevice spi3_device0;
    memset(&spi3_device0, 0, sizeof(struct SpiHardwareDevice));

    static struct SpiSlaveParam spi3_slaveparam0;
    memset(&spi3_slaveparam0, 0, sizeof(struct SpiSlaveParam));

    spi3_slaveparam0.spi_cs_gpio_pin = GPIO_PIN_1;
    spi3_slaveparam0.spi_cs_gpio_port = GPIOC_BASE;

    spi3_device0.spi_param.spi_slave_param = &spi3_slaveparam0;

    spi3_device0.spi_dev_done = &(spi_dev_done);

    ret = SpiDeviceRegister(&spi3_device0, (void *)(&spi3_device0.spi_param), SPI_3_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("BoardSpiDevBend SpiDeviceRegister device %s error %d\n", SPI_3_DEVICE_NAME_0, ret);
        return ERROR;
    }  

    ret = SpiDeviceAttachToBus(SPI_3_DEVICE_NAME_0, SPI_BUS_NAME_3);
    if (EOK != ret) {
        KPrintf("BoardSpiDevBend SpiDeviceAttachToBus device %s error %d\n", SPI_3_DEVICE_NAME_0, ret);
        return ERROR;
    }  
#endif

    return ret;
}

int HwSpiInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_SPI2
    static struct SpiBus spi2_bus;
    memset(&spi2_bus, 0, sizeof(struct SpiBus));

    static struct SpiDriver spi2_driver;
    memset(&spi2_driver, 0, sizeof(struct SpiDriver));

    static SPI_HandleTypeDef hspi2;

    hspi2.Instance = SPI2;
    spi2_bus.private_data = &hspi2;
    spi2_driver.configure = SpiDrvConfigure;
    // spi2_driver.driver.private_data = &hspi2;

    ret = BoardSpiBusInit(&spi2_bus, &spi2_driver, SPI_BUS_NAME_2, SPI_2_DRV_NAME);
    if (EOK != ret) {
        KPrintf("BoardSpiBusInit error ret %u\n", ret);
        return ERROR;
    }

#endif

#ifdef BSP_USING_SPI3
    static struct SpiBus spi3_bus;
    memset(&spi3_bus, 0, sizeof(struct SpiBus));

    static struct SpiDriver spi3_driver;
    memset(&spi3_driver, 0, sizeof(struct SpiDriver));

    static SPI_HandleTypeDef hspi3;

    hspi3.Instance = SPI3;
    spi3_bus.private_data = &hspi3;
    spi3_driver.configure = SpiDrvConfigure;
    // spi2_driver.driver.private_data = &hspi2;

    ret = BoardSpiBusInit(&spi3_bus, &spi3_driver, SPI_BUS_NAME_3, SPI_3_DRV_NAME);
    if (EOK != ret) {
        KPrintf("BoardSpiBusInit error ret %u\n", ret);
        return ERROR;
    }

#endif

    ret = BoardSpiDevBend();
    if (EOK != ret) {
        KPrintf("BoardSpiDevBend error ret %u\n", ret);
        return ERROR;
    } 

    return ret;
}


#ifdef TEST_SPI
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

static uint32 SpiOpen(void)
{
    NULL_PARAM_CHECK(drv);

    KPrintf("SPI open start\n");

    x_err_t ret = EOK;

    struct BusConfigureInfo configure_info;
    struct SpiMasterParam spi_master_param;
    spi_master_param.spi_data_bit_width = 8;
    spi_master_param.spi_work_mode = SPI_MODE_0 | SPI_MSB;

    configure_info.configure_cmd = OPE_CFG;
    configure_info.private_data = (void *)&spi_master_param;
    ret = BusDrvConfigure(drv, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_CFG error drv %8p cfg %8p\n", drv, &spi_master_param);
        return ERROR;
    }

    configure_info.configure_cmd = OPE_INT;
    ret = BusDrvConfigure(drv, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_INT error drv %8p\n", drv);
        return ERROR;
    }
    
    return ret;
}

static void SpiRead(void)
{
    struct BusBlockWriteParam write_param;
    struct BusBlockReadParam read_param;

    uint8 write_data = 7;
    uint8 read_data = 0;

    BusDevOpen(dev);

    write_param.buffer = (void *)&write_data;
    write_param.size = 1;
    BusDevWriteData(dev, &write_param);

    read_param.buffer = (void *)&read_data;
    read_param.size = 1;
    BusDevReadData(dev, &read_param);

    BusDevClose(dev);

    KPrintf("write data 0x%x receive data 0x%x\n", write_data, read_data);
}

void SPIInit(void)
{
    x_err_t ret = EOK;

    bus = BusFind(SPI_BUS_NAME_3);
    dev = BusFindDevice(bus, SPI_3_DEVICE_NAME_0);
    drv = BusFindDriver(bus, SPI_3_DRV_NAME);

    bus->match(drv, dev);
 
    ret = SpiOpen();
    if (EOK != ret) {
        KPrintf("SPI init failed\n");
        return;
    }

    KPrintf("SPI init succeed\n");

    return;
}

void TestSPI(void)
{
    SPIInit();
    SpiRead();
    return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                    TestSPI, TestSPI, open spi device and write_read parameters);

#endif

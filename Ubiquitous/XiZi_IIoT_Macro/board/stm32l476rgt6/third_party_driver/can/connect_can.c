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
* @file connect_can.c
* @brief support stm32l476 can function and register to bus framework
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2025-03-14
*/

/*************************************************
File name: connect_can.c
Description: support stm32l476 can function and register to bus framework
Others: 
History: 
1. Date: 2025-03-14
Author: AIIT XUOS Lab
Modification: 
1. support stm32l476 can configure, write and read
2. support stm32l476 can bus device and driver register
*************************************************/

#include "connect_can.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"

CAN_HandleTypeDef hcan1;

#define GET_CAN_BS2_TQ(x)  ((x) == 1 ? CAN_BS2_1TQ : (x) == 2 ? CAN_BS2_2TQ : (x) == 3 ? CAN_BS2_3TQ : (x) == 4 ? CAN_BS2_4TQ : \
                            (x) == 5 ? CAN_BS2_5TQ : (x) == 6 ? CAN_BS2_6TQ : (x) == 7 ? CAN_BS2_7TQ : (x) == 8 ? CAN_BS2_8TQ : CAN_BS2_1TQ)

#define GET_CAN_BS1_TQ(x) ((x) == 1  ? CAN_BS1_1TQ  : (x) == 2  ? CAN_BS1_2TQ  : (x) == 3  ? CAN_BS1_3TQ  : (x) == 4  ? CAN_BS1_4TQ  : \
                            (x) == 5  ? CAN_BS1_5TQ  : (x) == 6  ? CAN_BS1_6TQ  : (x) == 7  ? CAN_BS1_7TQ  : (x) == 8  ? CAN_BS1_8TQ  : \
                            (x) == 9  ? CAN_BS1_9TQ  : (x) == 10 ? CAN_BS1_10TQ : (x) == 11 ? CAN_BS1_11TQ : (x) == 12 ? CAN_BS1_12TQ : \
                            (x) == 13 ? CAN_BS1_13TQ : (x) == 14 ? CAN_BS1_14TQ : (x) == 15 ? CAN_BS1_15TQ : (x) == 16 ? CAN_BS1_16TQ : CAN_BS1_1TQ)

#define GET_CAN_SJW_TQ(x) ((x) == 1 ? CAN_SJW_1TQ : (x) == 2 ? CAN_SJW_2TQ : (x) == 3 ? CAN_SJW_3TQ : (x) == 4 ? CAN_SJW_4TQ : CAN_SJW_1TQ )
#define GET_CAN_MODE(x) ((x) == 0 ? CAN_MODE_NORMAL : (x) == 1 ? CAN_MODE_LOOPBACK : (x) == 2 ? CAN_MODE_SILENT : (x) == 3 ? CAN_MODE_SILENT_LOOPBACK : CAN_MODE_NORMAL )

static void CanInit(struct CanDriverConfigure *can_drv_config)
{
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = can_drv_config->brp;
    hcan1.Init.Mode = GET_CAN_MODE(can_drv_config->mode);
    hcan1.Init.SyncJumpWidth = GET_CAN_SJW_TQ(can_drv_config->tsjw);
    hcan1.Init.TimeSeg1 = GET_CAN_BS1_TQ(can_drv_config->tbs1);
    hcan1.Init.TimeSeg2 = GET_CAN_BS2_TQ(can_drv_config->tbs2);
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    int ret = HAL_CAN_Init(&hcan1);
    if (ret != HAL_OK)
    {
        KPrintf("Can init failed! ret=%d\n", ret);
    }

    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static uint32 CanConfig(void *can_drv_config)
{
    x_err_t ret = EOK;
    return ret;        
}

static uint32 CanDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    x_err_t ret = EOK;
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);
    struct CanDriverConfigure *can_drv_config;
    switch (configure_info->configure_cmd)
    {
        case OPE_INT: // can basic init
            can_drv_config = (struct CanDriverConfigure *)configure_info->private_data;
            CanInit(can_drv_config);
            break;
        case OPE_CFG: 
            CanConfig(configure_info->private_data);
            break;
        default:
            break;
    }

    return ret;
}

static uint32 CanOpenDev(void * dev)
{
    HAL_CAN_Start(&hcan1);
}

static uint32 CanCloseDev(void * dev)
{
    HAL_CAN_Stop(&hcan1);
}

static uint32 CanWriteData(void * dev , struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(write_param);
    uint32_t TxMailbox;
    uint8  *data   = (uint8  * ) write_param->buffer;
    uint8  mbox;
    uint16  i = 0;
    uint16  timer_count = 1000;
    CAN_TxHeaderTypeDef TxHeader;
    int idx = 0;
    while (idx < write_param->size)
    {
        int r_len = write_param->size - idx;
        int size = r_len >= 8 ? 8 : r_len;
        TxHeader.StdId = 0x55;
        TxHeader.ExtId = 0x00;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = size;
        TxHeader.TransmitGlobalTime = DISABLE;

        // 发送消息
        int ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8 *)(write_param->buffer + idx), &TxMailbox);
        if ( ret != HAL_OK) {
            KPrintf("Send message error: %d\n",ret);
        }
        idx += size;
    }
    
    return EOK;
}

static uint32 CanReadData(void *dev , struct BusBlockReadParam *databuf)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(databuf);

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    x_err_t ret=EOK;
    uint32 len = -1;
    struct CanSendConfigure *p_can_config = (struct CanSendConfigure*)databuf->buffer;
    // while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0);
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            len = databuf->size < RxHeader.DLC ? databuf->size : RxHeader.DLC;
            uint8* buf = databuf->buffer;
            for (uint8_t i = 0; i < RxHeader.DLC; i++) {
                buf[i] = RxData[i];
            }
        }
    }
    
    return len;
}

static struct CanDevDone can_dev_done = 
{
    .open  = CanOpenDev,
    .close  = CanCloseDev,
    .write  = CanWriteData,
    .read   = CanReadData
};

static int BoardCanBusInit(struct CanBus *can_bus, struct CanDriver *can_driver)
{
    x_err_t ret = EOK;

    /*Init the can bus */
    ret = CanBusInit(can_bus, CAN_BUS_NAME);
    if (EOK != ret) {
        KPrintf("Board_can_init canBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the can driver*/
    ret = CanDriverInit(can_driver, CAN_DRIVER_NAME);
    if (EOK != ret) {
        KPrintf("Board_can_init canDriverInit error %d\n", ret);
        return ERROR;
    }
    /*Attach the can driver to the can bus*/
    ret = CanDriverAttachToBus(CAN_DRIVER_NAME, CAN_BUS_NAME);
    if (EOK != ret) {
        KPrintf("Board_can_init CanDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/* Attach the can device to the can bus*/
static int BoardCanDevBend(void)
{
    x_err_t ret = EOK;
    static struct CanHardwareDevice can_device;
    memset(&can_device, 0, sizeof(struct CanHardwareDevice));

    can_device.dev_done = &can_dev_done;

    ret = CanDeviceRegister(&can_device, NONE, CAN_DEVICE_NAME);
    if (EOK != ret) {
        KPrintf("board_can_init CanDeviceInit device %s error %d\n", CAN_DEVICE_NAME, ret);
        return ERROR;
    }  

    ret = CanDeviceAttachToBus(CAN_DEVICE_NAME, CAN_BUS_NAME);
    if (EOK != ret) {
        KPrintf("board_can_init CanDeviceAttachToBus device %s error %d\n", CAN_DEVICE_NAME, ret);
        return ERROR;
    }  

    return ret;
}

int HwCanInit(void)
{
    x_err_t ret = EOK;

    static struct CanBus can_bus;
    memset(&can_bus, 0, sizeof(struct CanBus));

    static struct CanDriver can_driver;
    memset(&can_driver, 0, sizeof(struct CanDriver));

    can_driver.configure = &(CanDrvConfigure);
    
    ret = BoardCanBusInit(&can_bus, &can_driver);
    if (EOK != ret) {
      KPrintf(" can_bus_init %s error ret %u\n", CAN_BUS_NAME, ret);
      return ERROR;
    }

    ret = BoardCanDevBend();
    if (EOK != ret) {
        KPrintf("board_can_init error ret %u\n", ret);
        return ERROR;
    }
    return EOK;
}

#ifdef TEST_CAN
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

static uint32 TestCan(void)
{
    x_err_t ret = EOK;
    bus = BusFind(CAN_BUS_NAME);
    dev = BusFindDevice(bus, CAN_DEVICE_NAME);
    drv = BusFindDriver(bus, CAN_DRIVER_NAME);

    struct BusConfigureInfo configure_info;
    struct CanDriverConfigure can_config;
    can_config.brp = 5;
    can_config.mode = 0;
    can_config.tbs1 = 7;
    can_config.tbs2 = 2;
    can_config.tsjw = 1;
    // spi_master_param.spi_data_bit_width = 8;
    // spi_master_param.spi_work_mode = SPI_MODE_0 | SPI_MSB;

    configure_info.configure_cmd = OPE_INT;
    configure_info.private_data = (void *)&can_config;
    ret = BusDrvConfigure(drv, &configure_info);
    if (ret) {
        // KPrintf("spi drv OPE_CFG error drv %8p cfg %8p\n", drv, &spi_master_param);
        return ERROR;
    }

    struct BusBlockWriteParam write_param;
    struct BusBlockReadParam read_param;

    uint8 write_data[10] = {1,2,3,4,5,6,7,8,9,10};
    uint8 read_data = 0;

    BusDevOpen(dev);

    write_param.buffer = (void *)write_data;
    write_param.size = 10;
    BusDevWriteData(dev, &write_param);

    read_param.buffer = (void *)&read_data;
    read_param.size = 1;
    int n = BusDevReadData(dev, &read_param);

    // BusDevClose(dev);
    KPrintf("write 1~10 data and receive %d data[0]=0x%x\n", n, read_data);
    
    return ret;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                    TestCan, TestCan, open can device and write_read parameters);

#endif

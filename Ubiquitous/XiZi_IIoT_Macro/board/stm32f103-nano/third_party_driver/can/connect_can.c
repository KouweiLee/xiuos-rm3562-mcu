

#include <board.h>
#include <connect_can.h>

// #ifdef BSP_USING_CAN1 - TODO change to struct Stm32Can

static struct Stm32Can *stm32_can1;

// #endif // BSP_USING_CAN1

static unsigned int CanModeInit(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);
    struct CanDriverConfigure *config = (struct CanDriverConfigure *)configure_info->private_data;
    struct CanDriver *can_drv = (struct CanDriver *)drv;
    CAN_HandleTypeDef *hcan1 = &(stm32_can1->instance);
    hcan1->Instance = CAN1;
    hcan1->Init.Prescaler = 6;
    hcan1->Init.Mode = CAN_MODE_NORMAL;
    hcan1->Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1->Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan1->Init.TimeSeg2 = CAN_BS2_5TQ;
    hcan1->Init.TimeTriggeredMode = DISABLE;
    hcan1->Init.AutoBusOff = DISABLE;
    hcan1->Init.AutoWakeUp = DISABLE;
    hcan1->Init.AutoRetransmission = DISABLE;
    hcan1->Init.ReceiveFifoLocked = DISABLE;
    hcan1->Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(hcan1) != HAL_OK)
    {
        return ERROR;
    }

    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;                                                                    /* 过滤器组0 */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                /* 屏蔽位模式 */
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                               /* 32位。*/
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                               /* 过滤器被关联到FIFO 0 */
    sFilterConfig.FilterActivation = ENABLE;                                                         /* 使能过滤器 */
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        return ERROR;
    }

    if (HAL_CAN_Start(hcan1) != HAL_OK)
    {
        /* Start Error */
        return ERROR;
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Start Error */
        return ERROR;
    }
}

static uint32 CanOpen(void *dev)
{
    KPrintf("Can open\n");
KTaskDescriptorType task = NONE;
task = GetKTaskDescriptor();
KPrintf("CanOpen MdelayKTask2 %x\n", task);
KPrintf("CanOpen task->Done %x \n", task->Done);
    return 0;
}

static uint32 CanClose(void *dev)
{
    KPrintf("Can close\n");
    return 0;
}

static uint32 CanSendMsg(void *dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(dev);

    CAN_HandleTypeDef *hcan1 = &(stm32_can1->instance);
    uint8_t i = 0;
    uint32_t TxMailbox;
    uint8_t message[8];
    uint8_t *data = (uint8 *)write_param->buffer;
    uint16_t timer_count = 1000;
    CAN_TxHeaderTypeDef TxHeader; // 发送

    TxHeader.ExtId = CAN_TxExtId; // 扩展标识符(29位)
    TxHeader.IDE = CAN_ID_EXT;    // 使用扩展帧
    TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
    TxHeader.DLC = write_param->size;
    TxHeader.TransmitGlobalTime = DISABLE;

    for (i = 0; i < TxHeader.DLC; i++)
    {
        message[i] = data[i];
    }

    if (HAL_CAN_AddTxMessage(hcan1, &TxHeader, message, &TxMailbox) != HAL_OK) // 发送
    {
        return ERROR;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan1) != 3 && timer_count)
    {
        timer_count--;
    }
    if (timer_count <= 0)
    {
        return ERROR;
    }
    return EOK;
}


static uint32 CanRecvMsg(void *dev, struct BusBlockReadParam *databuf)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(databuf);
    int size = stm32_can1->can_recv_flag;
    int i;
    uint8_t *buf = (uint8 *)(databuf->buffer);
    if (size != 0)
    {
        for(i=0;i<size;i++)
        {
            buf[i]=stm32_can1->can_Rx_Data[i];
            //KPrintf("0x%02x   ", buf[i]);
            stm32_can1->can_Rx_Data[i]=0;
        }
        //KPrintf("\n");

        stm32_can1->can_recv_flag = 0;
    }
    return size;
}

static struct CanDevDone can_dev_done =
    {
        .open = CanOpen,//None
        .close = CanClose,
        .write = CanSendMsg,
        .read = CanRecvMsg
    };


// #ifdef BSP_USING_CAN1

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&(stm32_can1->instance));
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanNum)
{
	uint32_t i;
    uint8_t	RxData[8];

    KPrintf("in CAN1_RX0_IRQHandler\n");
    
    CAN_HandleTypeDef *hcan1 = &(stm32_can1->instance);
    CAN_RxHeaderTypeDef RxHeader;
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, stm32_can1->can_Rx_Data);
    stm32_can1->can_recv_flag =RxHeader.DLC;  //接收标志位

    // for(i = 0; i<RxHeader.DLC;i++)
    // {
    //     KPrintf("0x%02x   ", stm32_can1->can_Rx_Data[i]);
    // }
    // KPrintf("\n%d",RxHeader.DLC);
}

// #endif // BSP_USING_CAN1


static int BoardCanBusInit(struct Stm32Can *stm32can_bus, struct CanDriver *can_driver)
{

    x_err_t ret = EOK;
    /*Init the can bus */
    ret = CanBusInit(&stm32can_bus->can_bus, stm32can_bus->bus_name);
    if (EOK != ret)
    {
        KPrintf("Board_can_init canBusInit error %d\n", ret);
        return ERROR;
    }
    /*Init the can driver*/
    ret = CanDriverInit(can_driver, CAN_DRIVER_NAME);
    if (EOK != ret)
    {
        KPrintf("Board_can_init canDriverInit error %d\n", ret);
        return ERROR;
    }
    /*Attach the can driver to the can bus*/
    ret = CanDriverAttachToBus(CAN_DRIVER_NAME, stm32can_bus->bus_name);
    if (EOK != ret)
    {
        KPrintf("Board_can_init CanDriverAttachToBus error %d\n", ret);
        return ERROR;
    }
    return ret;
}

static x_err_t HwCanDeviceAttach(const char *bus_name, const char *device_name, struct Stm32Can *can1)
{
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(device_name);
    x_err_t result;
    struct CanHardwareDevice *can_device;
    /* attach the device to can bus*/
    can_device = (struct CanHardwareDevice *)x_malloc(sizeof(struct CanHardwareDevice));
    memset(can_device, 0, sizeof(struct CanHardwareDevice));
    can_device->dev_done = &can_dev_done;
    can_device->private_data = stm32_can1;
    result = CanDeviceRegister(can_device, NONE, device_name);
    if (EOK != result)
    {
        KPrintf("board_can_init canDeviceInit device %s error %d\n", "can1", result);
        return ERROR;
    }
    result = CanDeviceAttachToBus(device_name, bus_name);
    if (result != EOK)
    {
        SYS_ERR("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }
    CHECK(result == EOK);

    KPrintf("%s attach to %s done\n", device_name, bus_name);

    return result;
}

int InitHwCan(void)
{
    x_err_t ret = EOK;
    struct CanDriver *can_driver;

    stm32_can1 = (struct Stm32Can *)x_malloc(sizeof(struct Stm32Can));
    memset(stm32_can1, 0, sizeof(struct Stm32Can));
    stm32_can1->bus_name = CAN_BUS_NAME_1;
    stm32_can1->can_bus.private_data = stm32_can1;

    can_driver = (struct CanDriver *)x_malloc(sizeof(struct CanDriver));
    memset(can_driver, 0, sizeof(struct CanDriver));
    can_driver->configure = CanModeInit;
    can_driver->private_data = stm32_can1;

    ret = BoardCanBusInit(stm32_can1, can_driver);
    if (EOK != ret)
    {
        KPrintf(" can_bus_init %s error ret %u\n", stm32_can1->bus_name, ret);
        return ERROR;
    }

    ret = HwCanDeviceAttach(CAN_BUS_NAME_1, CAN_1_DEVICE_NAME_1, stm32_can1);
    if (EOK != ret)
    {
        KPrintf(" HwCanDeviceAttach %s error ret %u\n", stm32_can1->bus_name, ret);
        return ERROR;
    }

    return EOK;
}

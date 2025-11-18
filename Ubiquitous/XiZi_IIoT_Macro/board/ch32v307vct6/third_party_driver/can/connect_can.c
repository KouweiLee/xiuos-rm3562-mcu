
#include <connect_can.h>
#include <ch32v30x_gpio.h>
#include <ch32v30x_rcc.h>
#include <ch32v30x_misc.h>

static struct CanSendConfigure can_send_deconfig = 
{
    .stdid = 0x12,
    .exdid = 0x12,
    .ide = 0 ,
    .rtr = 0,
    .data_lenth = 8
};

static void CanGPIOInit(void)
{
    CAN_FilterInitTypeDef  can1_filter;
    GPIO_InitTypeDef           gpio_initstructure;
    CAN_InitTypeDef             can_initstruction;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    gpio_initstructure.GPIO_Pin = GPIO_Pin_9;
	gpio_initstructure.GPIO_Mode = GPIO_Mode_AF_PP;		
	gpio_initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &gpio_initstructure);

	gpio_initstructure.GPIO_Pin = GPIO_Pin_8;
	gpio_initstructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_Init( GPIOB, &gpio_initstructure);
}

static void Can1NvicConfig(void)
{
    NVIC_InitTypeDef can_nvic_config;

    can_nvic_config.NVIC_IRQChannel = CAN1_RX1_IRQn;
    can_nvic_config.NVIC_IRQChannelPreemptionPriority = 2;
    can_nvic_config.NVIC_IRQChannelSubPriority = 2;
    can_nvic_config.NVIC_IRQChannelCmd = ENABLE;
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    NVIC_Init(&can_nvic_config);
}

static uint32 CanModeInit(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);
    CAN_FilterInitTypeDef can1_filter;
    CAN_InitTypeDef     can_initstruction;

    struct CanDriverConfigure  *config  = ( struct CanDriverConfigure  *)configure_info->private_data;

    can_initstruction.CAN_TTCM  = DISABLE;
    can_initstruction.CAN_ABOM = DISABLE;
    can_initstruction.CAN_AWUM = DISABLE;
    can_initstruction.CAN_NART   = ENABLE;
    can_initstruction.CAN_TXFP    = DISABLE;
    can_initstruction.CAN_Mode   = config->mode;
    can_initstruction.CAN_RFLM   = DISABLE;
    can_initstruction.CAN_SJW      = config->tsjw;
    can_initstruction.CAN_BS1       = config->tbs1;
    can_initstruction.CAN_BS2       = config->tbs2;
    can_initstruction.CAN_Prescaler  = config->brp;
    
    CAN_Init(CAN1, &can_initstruction);

    can1_filter.CAN_FilterNumber=0;
    can1_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can1_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can1_filter.CAN_FilterIdHigh=0x0000;
    can1_filter.CAN_FilterIdLow=0x0000;
    can1_filter.CAN_FilterMaskIdHigh=0x0000;
    can1_filter.CAN_FilterMaskIdLow=0x0006;
    can1_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
    can1_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can1_filter);

    #ifdef  CAN_USING_INTERRUPT
       Can1NvicConfig();
    #endif

    return 0;
}

static uint32 CanSendMsg(void * dev , struct BusBlockWriteParam *write_param )
{
    NULL_PARAM_CHECK(write_param);

    uint8  *data   = (uint8  * ) write_param->buffer;
    u8  messege_box;
    u16  i = 0;
    u16  timer_count = 1000;
    CanTxMsg  tx_data;
    tx_data.StdId  = 0x55;
    tx_data.ExtId   = 0x00;
    tx_data.IDE      =  0;
    tx_data.RTR     =  0;
    tx_data.DLC     =  write_param->size;

    for(i = 0;i < tx_data.DLC;i ++) {
        tx_data.Data[i] = data[i];
    }

    messege_box = CAN_Transmit(CAN1,&tx_data);

    while (CAN_TransmitStatus(CAN1,messege_box)==  CAN_TxStatus_Failed &&timer_count) {
        timer_count--;
    }

    if (timer_count<=0) {
        return ERROR;
    }
    return EOK;
}

static uint32 CanRecvMsg(void *dev , struct BusBlockReadParam *databuf)
{
    NULL_PARAM_CHECK(dev);
    int  i;
    uint8  * buf  = (uint8 *)databuf->buffer;
    CanRxMsg msg;
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
        return 0;
    CAN_Receive(CAN1, CAN_FIFO0, &msg);
    for(i = 0 ;i < msg.DLC;i ++)
         buf[i] = msg.Data[i];
    databuf->size  =  msg.DLC ; 

    return msg.DLC;
}

static struct CanDevDone dev_done = 
{
    .open  = NONE,
    .close  = NONE,
    .write  = CanSendMsg,
    .read   = CanRecvMsg
};

static struct CanHardwareDevice dev;

#ifdef  CAN_USING_INTERRUPT
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg   rxmsg;
    int i = 0;
    CAN_Receive(CAN1, 0, &rxmsg);
    for (i = 0;i < 8;i ++)
          KPrintf("rxbuf [%d] = :%d",i,rxmsg.Data[i]);
}
DECLARE_HW_IRQ(CAN1_RX0_IRQn, CAN1_RX0_IRQHandler, NONE);
#endif

static int BoardCanBusInit(struct CanDev *CanDev_bus, struct CanDriver *can_driver)
{
    x_err_t ret = EOK;

    /*Init the can bus */
    ret = CanBusInit(&CanDev_bus->can_bus, CanDev_bus->bus_name);
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
    ret = CanDriverAttachToBus(CAN_DRIVER_NAME, CanDev_bus->bus_name);
    if (EOK != ret) {
        KPrintf("Board_can_init CanDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

static x_err_t HwCanDeviceAttach(const char *bus_name, const char *device_name)
{
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(device_name);

    x_err_t result;
    struct CanHardwareDevice *can_device;

    /* attach the device to can bus*/
    can_device = (struct CanHardwareDevice *)x_malloc(sizeof(struct CanHardwareDevice));
    CHECK(can_device);
    memset(can_device, 0, sizeof(struct CanHardwareDevice));
    can_device->dev_done = &dev_done;

     result = CanDeviceRegister(can_device, NONE, device_name);
    if (EOK != result) {
        KPrintf("board_can_init canDeviceInit device %s error %d\n", "can1", result);
        return ERROR;
    }  

    result = CanDeviceAttachToBus(device_name, bus_name);
    if (result != EOK) {
        SYS_ERR("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    CHECK(result == EOK);

    KPrintf("%s attach to %s done\n", device_name, bus_name);

    return result;
}

struct CanDev can1;

int InitHwCan(void)
{
    x_err_t ret = EOK;
    struct CanDev *can_bus;

    static struct CanDriver can_driver;
    memset(&can_driver, 0, sizeof(struct CanDriver));
    can_driver.configure = CanModeInit;

    CanGPIOInit();
    can_bus = &can1;
    can_bus->instance = CAN1;
    can_bus->bus_name = CAN_BUS_NAME_1;
    can_bus->can_bus.private_data = &can1;
    
    ret = BoardCanBusInit(can_bus, &can_driver);
   
    if (EOK != ret) {
      KPrintf(" can_bus_init %s error ret %u\n", can_bus->bus_name, ret);
      return ERROR;
    }

    ret = HwCanDeviceAttach(CAN_BUS_NAME_1,CAN_1_DEVICE_NAME_1);  
    if (EOK != ret) {
      KPrintf(" HwCanDeviceAttach %s error ret %u\n", can_bus->bus_name, ret);
      return ERROR;
    }
    return EOK;
}
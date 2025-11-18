/*
* Copyright (c) 2020-2025 AIIT XUOS Lab
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
* @file connect_usb.c
* @brief Support USB device (CDC) function and register to bus-driver-device framework.
* @version 2.0
* @author AIIT XUOS Lab
* @date 2025-07-08
*/

#include <board.h>
#include <connect_usb.h>

/* Static device structures */
static struct UsbBus usb_bus;
static struct UsbDriver usb_driver;
struct UsbHardwareDevice usb_device;

uint8_t receivedByte = 0;
x_bool isDataValid = 0;

extern PCD_HandleTypeDef hpcd_USB_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

static int UsbGetchar(struct UsbHardwareDevice *usb_dev)
{
    (void)usb_dev; 

    if (!isDataValid) {
        return -ERROR; 
    }

    isDataValid = 0; 
    return (int)receivedByte;
}

static int UsbPutchar(struct UsbHardwareDevice *usb_dev, char c){
    (void *)usb_dev;
    uint8_t ch = (uint8_t)c;
    while (CDC_Transmit_FS(&ch, 1) == USBD_BUSY) {
        
    }
    return EOK;
}

struct UsbHwDevDone hwdev_done = {
    .get_char = UsbGetchar,
    .put_char = UsbPutchar,
};

/* ======================== dev_usb.c ============================= */

static int usb_isr_cnt = 0;

static int SerialWorkModeCheck(struct SerialDevParam *serial_dev_param)
{
    if (SIGN_OPER_INT_TX & serial_dev_param->serial_set_mode)
    {
        if (SIGN_OPER_INT_TX & serial_dev_param->serial_work_mode)
        {
            return EOK;
        }
        else
        {
            KPrintf("SerialWorkModeCheck set mode 0x%x work mode error 0x%x\n",
                    serial_dev_param->serial_set_mode, serial_dev_param->serial_work_mode);
            return ERROR;
        }
    }
    else if (SIGN_OPER_INT_RX & serial_dev_param->serial_set_mode)
    {
        if (SIGN_OPER_INT_RX & serial_dev_param->serial_work_mode)
        {
            return EOK;
        }
        else
        {
            KPrintf("SerialWorkModeCheck set mode 0x%x work mode error 0x%x\n",
                    serial_dev_param->serial_set_mode, serial_dev_param->serial_work_mode);
            return ERROR;
        }
    }

    else
    {
        serial_dev_param->serial_set_mode = serial_dev_param->serial_work_mode;
        return EOK;
    }
}

static inline int UsbDevIntWrite(struct UsbHardwareDevice *usb_dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(usb_dev);
    NULL_PARAM_CHECK(write_param);

    struct UsbHwDevDone *hwdev_done = usb_dev->hwdev_done;
    const uint8 *write_data = (const uint8 *)write_param->buffer;
    x_size_t write_length = write_param->size;
    x_size_t len_result = 0;

    while (write_length)
    {
        if (EOK != hwdev_done->put_char(usb_dev, *(char *)write_data))
        {
            KSemaphoreObtain(usb_dev->usb_fifo.usb_tx->usb_txfifo_sem, WAITING_FOREVER);
            continue;
        }

        KPrintf("UsbDevIntWrite data %d write_length %u\n", *(char *)write_data, write_length);
        len_result++;
        write_data++;
        write_length--;
    }

    return len_result;
}

static inline int UsbDevIntRead(struct UsbHardwareDevice *usb_dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(usb_dev);
    NULL_PARAM_CHECK(read_param);

    struct UsbHwDevDone *hwdev_done = usb_dev->hwdev_done;
    uint8 *read_data = (uint8 *)read_param->buffer;
    x_size_t read_length = read_param->size;

    while (read_length)
    {
        uint8 get_char;
        x_base lock;

        lock = CriticalAreaLock();

        if (usb_dev->usb_fifo.usb_rx->usb_recv_num == usb_dev->usb_fifo.usb_rx->usb_send_num)
        {
            if (RET_FALSE == usb_dev->usb_fifo.usb_rx->usb_rx_full)
            {
                CriticalAreaUnLock(lock);
                if (0 == usb_isr_cnt)
                {
                    KSemaphoreSetValue(usb_dev->haldev.dev_sem, 0);
                }
                break;
            }
        }

        get_char = usb_dev->usb_fifo.usb_rx->usb_rx_buffer[usb_dev->usb_fifo.usb_rx->usb_recv_num];
        usb_dev->usb_fifo.usb_rx->usb_recv_num += 1;
        if (usb_dev->usb_fifo.usb_rx->usb_recv_num >= USB_RX_BUFFER_SIZE)
        {
            usb_dev->usb_fifo.usb_rx->usb_recv_num = 0;
        }

        if (RET_TRUE == usb_dev->usb_fifo.usb_rx->usb_rx_full)
        {
            usb_dev->usb_fifo.usb_rx->usb_rx_full = RET_FALSE;
        }

        if (usb_isr_cnt > 0)
        {
            usb_isr_cnt--;
        }

        CriticalAreaUnLock(lock);

        *read_data = get_char;
        read_data++;
        read_length--;
        read_param->read_length++;
    }

    return EOK;
}



static inline int UsbDevPollingWrite(struct UsbHardwareDevice *usb_dev, struct BusBlockWriteParam *write_param, uint16 serial_stream_mode)
{
    NULL_PARAM_CHECK(usb_dev);
    NULL_PARAM_CHECK(write_param);

    struct UsbHwDevDone *hwdev_done = usb_dev->hwdev_done;
    const uint8 *write_data = (const uint8 *)write_param->buffer;
    x_size_t write_length = write_param->size;
    x_size_t len_result = 0;
    while (write_length)
    {
        if ((*write_data == '\n') && (SIGN_OPER_STREAM == serial_stream_mode))
        {
            hwdev_done->put_char(usb_dev, '\r');
        }

        hwdev_done->put_char(usb_dev, *write_data);
        len_result++;
        ++write_data;
        --write_length;
    }

    return len_result;
}

static inline int UsbDevPollingRead(struct UsbHardwareDevice *usb_dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(usb_dev);
    NULL_PARAM_CHECK(read_param);

    struct UsbHwDevDone *hwdev_done = usb_dev->hwdev_done;
    uint8 *read_data = (uint8 *)read_param->buffer;
    x_size_t read_length = read_param->size;

    uint8 get_char;

    get_char = hwdev_done->get_char(usb_dev);

    *read_data = get_char;
    read_data++; 
    read_length--;
    read_param->read_length++;

    return EOK;
}

static uint32 UsbDevOpen(void *dev)
{
    NULL_PARAM_CHECK(dev);

    int serial_operation_cmd;
    struct UsbHardwareDevice *usb_dev = (struct UsbHardwareDevice *)dev;
    struct Driver *drv = usb_dev->haldev.owner_bus->owner_driver;
    struct UsbDriver *usb_drv = (struct UsbDriver *)drv;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)usb_dev->haldev.private_data;

    if (EOK != SerialWorkModeCheck(serial_dev_param))
    {
        KPrintf("UsbDevOpen error!\n");
        return ERROR;
    }

    if (NONE == usb_dev->usb_fifo.usb_rx)
    {
        if (SIGN_OPER_INT_RX & serial_dev_param->serial_set_mode)
        {
            usb_dev->usb_fifo.usb_rx = (struct UsbRx *)x_malloc(sizeof(struct UsbRx));
            if (NONE == usb_dev->usb_fifo.usb_rx)
            {
                KPrintf("UsbDevOpen x_malloc serial_rx error\n");
                x_free(usb_dev->usb_fifo.usb_rx);
                return ERROR;
            }

            usb_dev->usb_fifo.usb_rx->usb_rx_buffer = (uint8 *)x_malloc(USB_RX_BUFFER_SIZE);
            if (NONE == usb_dev->usb_fifo.usb_rx->usb_rx_buffer)
            {
                KPrintf("UsbDevOpen x_malloc usb_rx_buffer error\n");
                x_free(usb_dev->usb_fifo.usb_rx->usb_rx_buffer);
                x_free(usb_dev->usb_fifo.usb_rx);
                return ERROR;
            }

            memset(usb_dev->usb_fifo.usb_rx->usb_rx_buffer, 0, USB_RX_BUFFER_SIZE);
            usb_dev->usb_fifo.usb_rx->usb_send_num = 0;
            usb_dev->usb_fifo.usb_rx->usb_recv_num = 0;
            usb_dev->usb_fifo.usb_rx->usb_rx_full = RET_FALSE;
            serial_dev_param->serial_work_mode |= SIGN_OPER_INT_RX;

        }

        else
        {
            usb_dev->usb_fifo.usb_rx = NONE;
        }
    }
    else
    {
        if (SIGN_OPER_INT_RX & serial_dev_param->serial_set_mode)
        {
            serial_dev_param->serial_work_mode |= SIGN_OPER_INT_RX;
        }
    }

    if (NONE == usb_dev->usb_fifo.usb_tx)
    {
        if (SIGN_OPER_INT_TX & serial_dev_param->serial_set_mode)
        {
            usb_dev->usb_fifo.usb_tx = (struct UsbTx *)x_malloc(sizeof(struct UsbTx));
            if (NONE == usb_dev->usb_fifo.usb_tx)
            {
                KPrintf("UsbDevOpen x_malloc serial_tx error\n");
                x_free(usb_dev->usb_fifo.usb_tx);
                return ERROR;
            }

            usb_dev->usb_fifo.usb_tx->usb_txfifo_sem = KSemaphoreCreate(0);
            serial_dev_param->serial_work_mode |= SIGN_OPER_INT_TX;

        }

        else
        {
            usb_dev->usb_fifo.usb_tx = NONE;
        }
    }
    else
    {
        if (SIGN_OPER_INT_TX & serial_dev_param->serial_set_mode)
        {
            serial_dev_param->serial_work_mode |= SIGN_OPER_INT_TX;
        }

    }

    usb_dev->haldev.dev_sem = KSemaphoreCreate(0);
    if (usb_dev->haldev.dev_sem < 0)
    {
        KPrintf("UsbDevOpen create sem failed .\n");

        if (usb_dev->usb_fifo.usb_rx->usb_rx_buffer)
        {
            x_free(usb_dev->usb_fifo.usb_rx->usb_rx_buffer);
        }
        if (usb_dev->usb_fifo.usb_rx)
        {
            x_free(usb_dev->usb_fifo.usb_rx);
        }
        if (usb_dev->usb_fifo.usb_tx)
        {
            x_free(usb_dev->usb_fifo.usb_tx);
        }

        return ERROR;
    }

    return EOK;
}

static uint32 UsbDevClose(void *dev)
{
    NULL_PARAM_CHECK(dev);

    int serial_operation_cmd = OPER_CLR_INT;
    struct UsbHardwareDevice *usb_dev = (struct UsbHardwareDevice *)dev;
    struct Driver *drv = usb_dev->haldev.owner_bus->owner_driver;
    struct UsbDriver *usb_drv = (struct UsbDriver *)drv;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)usb_dev->haldev.private_data;

    if (SIGN_OPER_INT_RX & serial_dev_param->serial_work_mode)
    {
        NULL_PARAM_CHECK(usb_dev->usb_fifo.usb_rx->usb_rx_buffer);
        NULL_PARAM_CHECK(usb_dev->usb_fifo.usb_rx);
        x_free(usb_dev->usb_fifo.usb_rx->usb_rx_buffer);
        x_free(usb_dev->usb_fifo.usb_rx);
        usb_dev->usb_fifo.usb_rx = NONE; 

       
    }


    if (SIGN_OPER_INT_TX & serial_dev_param->serial_work_mode)
    {
        NULL_PARAM_CHECK(usb_dev->usb_fifo.usb_tx);
        x_free(usb_dev->usb_fifo.usb_tx);

    }


    KSemaphoreDelete(usb_dev->haldev.dev_sem);
    return EOK;
}

static uint32 UsbDevWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(write_param);

    x_err_t ret = EOK;

    struct UsbHardwareDevice *usb_dev = (struct UsbHardwareDevice *)dev;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)usb_dev->haldev.private_data;

    if (serial_dev_param->serial_work_mode & SIGN_OPER_INT_TX)
    {
        ret = UsbDevIntWrite(usb_dev, write_param);
        if (ret < 0)
        {
            KPrintf("UsbDevIntWrite error %d\n", ret);
            return ERROR;
        }
    }

    else
    {
        ret = UsbDevPollingWrite(usb_dev, write_param, serial_dev_param->serial_stream_mode);
        if (ret < 0)
        {
            KPrintf("UsbDevPollingWrite error %d\n", ret);
            return ERROR;
        }
    }

    return ret;
}

static uint32 UsbDevRead(void *dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(read_param);

    x_err_t ret = EOK;

    struct UsbHardwareDevice *usb_dev = (struct UsbHardwareDevice *)dev;
    struct SerialDevParam *serial_dev_param = (struct SerialDevParam *)usb_dev->haldev.private_data;

    if (EOK == KSemaphoreObtain(usb_dev->haldev.dev_sem, serial_dev_param->serial_timeout))
    {
        if (serial_dev_param->serial_work_mode & SIGN_OPER_INT_RX)
        {
            ret = UsbDevIntRead(usb_dev, read_param);
            if (EOK != ret)
            {
                KPrintf("SerialDevIntRead error %d\n", ret);
                return ERROR;
            }
        }

        else
        {
            ret = UsbDevPollingRead(usb_dev, read_param);
            if (EOK != ret)
            {
                KPrintf("UsbDevPollingRead error %d\n", ret);
                return ERROR;
            }
        }
    }
    else
    {
        return ERROR;
    }
    return EOK;
}

void UsbSetIsr(struct UsbHardwareDevice *usb_dev, int event)
{
    switch (event & 0xff)
    {
    case SERIAL_EVENT_RX_IND:
    {
        int get_char;
        x_base lock;

        struct UsbHwDevDone *hwdev_done = usb_dev->hwdev_done;

        while (1)
        {
            get_char = hwdev_done->get_char(usb_dev);
            if (-ERROR == get_char)
            {
                break;
            }

            lock = CriticalAreaLock();

            usb_dev->usb_fifo.usb_rx->usb_rx_buffer[usb_dev->usb_fifo.usb_rx->usb_send_num] = (uint8)get_char;
            usb_dev->usb_fifo.usb_rx->usb_send_num += 1;
            if (usb_dev->usb_fifo.usb_rx->usb_send_num >= USB_RX_BUFFER_SIZE)
            {
                usb_dev->usb_fifo.usb_rx->usb_send_num = 0;
            }

            if (usb_dev->usb_fifo.usb_rx->usb_send_num == usb_dev->usb_fifo.usb_rx->usb_recv_num)
            {
                usb_dev->usb_fifo.usb_rx->usb_recv_num += 1;
                usb_dev->usb_fifo.usb_rx->usb_rx_full = RET_TRUE;
                if (usb_dev->usb_fifo.usb_rx->usb_recv_num >= USB_RX_BUFFER_SIZE)
                {
                    usb_dev->usb_fifo.usb_rx->usb_recv_num = 0;
                }
            }
            CriticalAreaUnLock(lock);
        }

        x_size_t serial_rx_length;

        lock = CriticalAreaLock();
        if (usb_dev->usb_fifo.usb_rx->usb_recv_num > usb_dev->usb_fifo.usb_rx->usb_send_num)
        {
            serial_rx_length = USB_RX_BUFFER_SIZE - usb_dev->usb_fifo.usb_rx->usb_recv_num + usb_dev->usb_fifo.usb_rx->usb_send_num;
        }
        else
        {
            serial_rx_length = usb_dev->usb_fifo.usb_rx->usb_send_num - usb_dev->usb_fifo.usb_rx->usb_recv_num;
        }
        CriticalAreaUnLock(lock);

        if (serial_rx_length)
        {
            if (usb_dev->haldev.dev_recv_callback)
            {
                usb_dev->haldev.dev_recv_callback((void *)usb_dev, serial_rx_length);
            }

            lock = CriticalAreaLock();
            usb_isr_cnt += 1;
            CriticalAreaUnLock(lock);

            KSemaphoreAbandon(usb_dev->haldev.dev_sem);
        }
        break;
    }
    case SERIAL_event_id_tX_DONE:
    {
        KSemaphoreAbandon(usb_dev->usb_fifo.usb_tx->usb_txfifo_sem);
        break;
    }
    }
}


const struct UsbDevDone usb_dev_done =
{
        .open = UsbDevOpen,
        .close = UsbDevClose,
        .write = UsbDevWrite,
        .read = UsbDevRead,
};


/* ============================================================================= */


static void USBClock_Config(void){

    __HAL_RCC_HSI48_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY)) {}

    __HAL_RCC_USB_CONFIG(RCC_USBCLKSOURCE_HSI48);

    __HAL_RCC_CRS_CLK_ENABLE();

    RCC_CRSInitTypeDef CRSInit = {0};
    CRSInit.Prescaler            = RCC_CRS_SYNC_DIV1;
    CRSInit.Source               = RCC_CRS_SYNC_SOURCE_USB;       
    CRSInit.Polarity             = RCC_CRS_SYNC_POLARITY_RISING;
    CRSInit.ReloadValue          = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    CRSInit.ErrorLimitValue      = 34;                            
    CRSInit.HSI48CalibrationValue= 32;                            

    HAL_RCCEx_CRSConfig(&CRSInit);

    __HAL_RCC_USB_CLK_ENABLE();    
}

static void MX_GPIO_Init(void){

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

static uint32 UsbConfigure(void *drv, struct BusConfigureInfo *configure_info){
    
    return EOK;
}

int InitUSB(void){
    x_err_t ret = EOK;

    USBClock_Config();
    MX_GPIO_Init();
    MX_USB_Device_Init();
    
    usb_driver.configure = UsbConfigure;

    usb_device.dev_done = &usb_dev_done;
    usb_device.hwdev_done = &hwdev_done;

    static struct SerialDevParam usb_dev_param_1;
    memset(&usb_dev_param_1, 0, sizeof(struct SerialDevParam));

    usb_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    
    /* Register USB bus */
    ret = UsbBusInit(&usb_bus, USB_BUS_NAME);
    if (ret != EOK) {
        KPrintf("InitHwUsb: USB bus init failed!\n");
        return ret;
    }

    /* Register USB driver */
    ret = UsbDriverInit(&usb_driver, USB_DRIVER_NAME);
    if (ret != EOK) {
        KPrintf("InitHwUsb: USB driver init failed!\n");
        return ret;
    }

    /* Attach driver to bus */
    ret = UsbDriverAttachToBus(USB_DRIVER_NAME, USB_BUS_NAME);
    if (ret != EOK) {
        KPrintf("InitHwUsb: USB driver attach failed!\n");
        return ret;
    }

    /* Register USB hardware device */
    ret = USBDeviceRegister(&usb_device, &usb_dev_param_1, USB_DEVICE_NAME);
    if (ret != EOK) {
        KPrintf("InitHwUsb: USB device register failed!\n");
        return ret;
    }

    /* Attach device to bus */
    ret = USBDeviceAttachToBus(USB_DEVICE_NAME, USB_BUS_NAME);
    if (ret != EOK) {
        KPrintf("InitHwUsb: USB device attach failed!\n");
        return ret;
    }
    KPrintf("InitHwUsb: USB device initialized successfully.\n");
    return EOK;
}

#ifdef  BSP_USING_USB
void USB_HP_IRQHandler(int irqn, void *arg){
  /* USER CODE BEGIN USB_HP_IRQn 0 */

  /* USER CODE END USB_HP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_HP_IRQn 1 */

  /* USER CODE END USB_HP_IRQn 1 */
}
DECLARE_HW_IRQ(USB_HP_IRQn, USB_HP_IRQHandler, NONE);
/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(int irqn, void *arg){
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}
DECLARE_HW_IRQ(USB_LP_IRQn, USB_LP_IRQHandler, NONE);
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
#endif

























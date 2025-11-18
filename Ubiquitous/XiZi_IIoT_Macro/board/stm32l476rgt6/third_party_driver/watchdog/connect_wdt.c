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
* @file connect_wdt.c
* @brief support stm32l476 watchdog function and register to bus framework
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2025-03-13
*/

#include <connect_wdt.h>

WWDG_HandleTypeDef hwwdg;

static uint32 WdtOpen(void *dev)
{
    NULL_PARAM_CHECK(dev);

    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
    hwwdg.Init.Window = 127;
    hwwdg.Init.Counter = 127;
    hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
    int ret = HAL_WWDG_Init(&hwwdg);
    if ( ret != HAL_OK)
    {
        KPrintf("WdtOpen error ret %u\n", ret);
    } else {
        KPrintf("WdtOpen success\n");
    }
    __HAL_RCC_WWDG_CLK_ENABLE();
    HAL_WWDG_Refresh(&hwwdg);
    __HAL_WWDG_ENABLE(&hwwdg);
    return EOK;
}

static uint32 WdtConfigure(void *drv, struct BusConfigureInfo *args)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(args);

    // stm32 window based on a 7-bit downcounter
    // The timing for feeding the dog must be greater than 0x3F
    // So the window minimal value is 0x40
    int window = *((int*)args->private_data);
    if(window > 127) {
        window = 127;
    } else if (window < 0x40) {
        window = 0x40;
    }

    switch (args->configure_cmd)
    {
        case OPER_WDT_SET_TIMEOUT:
            hwwdg.Instance = WWDG;
            hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
            hwwdg.Init.Window = window;
            hwwdg.Init.Counter = 127;
            hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
            if (HAL_WWDG_Init(&hwwdg) != 0) {
                return ERROR;
            }
        break;
        case OPER_WDT_KEEPALIVE:
            HAL_WWDG_Refresh(&hwwdg);
            break;
        default:
            return ERROR;
    }
    return EOK;
}

static const struct WdtDevDone dev_done =
{
    WdtOpen,
    NONE,
    NONE,
    NONE,
};

/**
 * @description: Watchdog function
 * @return success: EOK, failure: other
 */
int StartWatchdog(void)
{
    //add feed watchdog task function

    return EOK;
}

int HwWdtInit(void)
{
    x_err_t ret = EOK;

    static struct WdtBus wdt0;

    ret = WdtBusInit(&wdt0, WDT_BUS_NAME_0);
    if (ret != EOK) {
        KPrintf("Watchdog bus init error %d\n", ret);
        return ERROR;
    }

    static struct WdtDriver drv0;
    drv0.configure = WdtConfigure;
    
    ret = WdtDriverInit(&drv0, WDT_DRIVER_NAME_0);
    if (ret != EOK) {
        KPrintf("Watchdog driver init error %d\n", ret);
        return ERROR;
    }

    ret = WdtDriverAttachToBus(WDT_DRIVER_NAME_0, WDT_BUS_NAME_0);
    if (ret != EOK) {
        KPrintf("Watchdog driver attach error %d\n", ret);
        return ERROR;
    }

    static struct WdtHardwareDevice dev0;
    dev0.dev_done = &dev_done;

    ret = WdtDeviceRegister(&dev0, WDT_0_DEVICE_NAME_0);
    if (ret != EOK) {
        KPrintf("Watchdog device register error %d\n", ret);
        return ERROR;
    }

    ret = WdtDeviceAttachToBus(WDT_0_DEVICE_NAME_0, WDT_BUS_NAME_0);
    if (ret != EOK) {
        KPrintf("Watchdog device register error %d\n", ret);
        return ERROR;
    }

    return ret;
}


#ifdef TEST_WDT
static struct Bus *bus;
static struct HardwareDev *dev;
static struct Driver *drv;

void WDTTest(void)
{
    x_err_t ret = EOK;

    bus = BusFind(WDT_BUS_NAME_0);
    dev = BusFindDevice(bus, WDT_0_DEVICE_NAME_0);
    drv = BusFindDriver(bus, WDT_DRIVER_NAME_0);
    BusDevOpen(dev);

    struct BusConfigureInfo config;
    memset(&config, 0 , sizeof(struct BusConfigureInfo));
    config.configure_cmd = OPER_WDT_KEEPALIVE;

    for (int i = 0; i < 1000; i++)
    {
        BusDrvConfigure(drv, &config);
        HAL_Delay(5);
        if (i%100 == 0)
        {
            KPrintf("feed dog for %d times\n", i);
        }
        
    }
    KPrintf("Stop feed dog, and waiting for reset...\n");
    int count = 0;
    while(1);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                        WDTTest, WDTTest, watchdog test);
#endif

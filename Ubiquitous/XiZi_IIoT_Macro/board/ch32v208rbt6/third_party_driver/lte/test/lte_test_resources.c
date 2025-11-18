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
 * @file lte_test_resources.c
 * @brief test lte on the resources layer 
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-07-03
 */

#include <bus.h>
#include <bus_serial.h>
#include <dev_serial.h>
#include <stdio.h>
#include <string.h>
#include <xizi.h>

int lte_test_resources(void) {
    int ret = EOK;

    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    char sendString[] = "Hello World!\r\n";
    char receiveString[100] = {};

    /* find the lte bus pointer */
    bus = BusFind(LTE_BUS_NAME);
    if (NONE == bus) {
        KPrintf("BusFind %s failed\n", LTE_BUS_NAME);
        return 1;
    }
    /* find the lte driver pointer */
    drv = BusFindDriver(bus, LTE_DRV_NAME);
    if (NONE == drv) {
        KPrintf("BusFindDriver %s failed\n", LTE_DRV_NAME);
        return 1;
    }
    /* find the lte device pointer */
    dev = BusFindDevice(bus, LTE_DEVICE_NAME_1);
    if (NONE == dev) {
        KPrintf("BusFindDevice %s failed\n", LTE_DEVICE_NAME_1);
        return 1;
    }

    /*step 1: init bus_driver, change struct SerialCfgParam if necessary*/
    struct BusConfigureInfo configure_info;
    struct SerialCfgParam lteCfgParam;
    memset(&lteCfgParam, 0, sizeof(struct SerialCfgParam));
    configure_info.configure_cmd = OPE_INT;
    configure_info.private_data = &lteCfgParam;
    ret = BusDrvConfigure(drv, &configure_info);

    if (EOK != ret) {
        KPrintf("BusDrvConfigure OPE_INT failed error code %d\n", ret);
        return ret;
    }

    /*step 2: match serial bus_driver with bus_device*/
    ret = bus->match(drv, dev);
    if (EOK != ret) {
        KPrintf("BusMatch failed error code %d\n", ret);
        return ret;
    }

    /*step 3: open bus_device, configure struct SerialDevParam if necessary*/
    ret = BusDevOpen(dev);
    if (EOK != ret) {
        KPrintf("BusDevOpen failed error code %d\n", ret);
        return ret;
    }

    /*step 4: write serial data, configure struct BusBlockWriteParam*/
    struct BusBlockWriteParam writeParam;
    writeParam.buffer = (void *)sendString;
    writeParam.size = sizeof(sendString) - 1;
    BusDevWriteData(dev, &writeParam);

    Delay_Ms(10000);

    /*step 5: read serial data, configure struct BusBlockWriteParam*/
    struct BusBlockReadParam readParam;
    char buffer[100];
    readParam.buffer = (void *)buffer;
    readParam.size = 100;
    readParam.read_length = 0;
    int res = BusDevReadData(dev, &readParam);
    if (EOK == res) {
        KPrintf("read_length %d\n", readParam.read_length);
        for (int i = 0; i < readParam.read_length; i++) {
            KPrintf("%c ", buffer[i]);
        }
        KPrintf("\n");
    } else {
        KPrintf("read data failed\n");
    }

    /*step 6: close bus_device*/
    BusDevClose(dev);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), lte_test_resources,
                 lte_test_resources, test lte on the resources layer);
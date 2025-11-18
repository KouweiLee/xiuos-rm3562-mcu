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
 * @file rs485_test_resources.c
 * @brief test rs485 on the resources layer
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-07-03
 */

#include <bus.h>
#include <bus_serial.h>
#include <dev_serial.h>
#include <stdio.h>
#include <string.h>
#include <xsconfig.h>

int rs485_test_resources(void) {
    int ret = EOK;

    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    // char sendString[][8] = {{0x93, 0x03, 0x00, 0x00, 0x00, 0x02, 0xD8, 0xB9},
    //                         {0x93, 0x03, 0x00, 0x0A, 0x00, 0x02, 0xF8, 0xBB},
    // 						{0x93, 0x03, 0x00, 0x14, 0x00, 0x02, 0x98, 0xBD},
    // 						{0x93, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xB8, 0xBF},
    // 						{0x93, 0x03, 0x00, 0x28, 0x00, 0x02, 0x58, 0xB1},
    // 						{0x93, 0x03, 0x00, 0x32, 0x00, 0x02, 0x79, 0x76}};
    char sendString[][8] = {{0x93, 0x03, 0x00, 0x00, 0x00, 0x3C, 0x59, 0x69},
                            {0x93, 0x03, 0x00, 0x3C, 0x00, 0x03, 0xD9, 0x75}};
    char receiveString[100] = {};

    /* find the rs485 bus pointer */
    bus = BusFind(RS485_BUS_NAME);
    if (NONE == bus) {
        KPrintf("BusFind %s failed\n", RS485_BUS_NAME);
        return 1;
    }
    /* find the rs485 driver pointer */
    drv = BusFindDriver(bus, RS485_DRV_NAME);
    if (NONE == drv) {
        KPrintf("BusFindDriver %s failed\n", RS485_DRV_NAME);
        return 1;
    }
    /* find the rs485 device pointer */
    dev = BusFindDevice(bus, RS485_DEVICE_NAME_1);
    if (NONE == dev) {
        KPrintf("BusFindDevice %s failed\n", RS485_DEVICE_NAME_1);
        return 1;
    }

    /*step 1: init bus_driver, change struct SerialCfgParam if necessary*/
    struct BusConfigureInfo configure_info;
    struct SerialCfgParam serialCfgParam;
    memset(&serialCfgParam, 0, sizeof(struct SerialCfgParam));
    configure_info.configure_cmd = OPE_INT;
    configure_info.private_data = &serialCfgParam;
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
    for (int i = 0; i < sizeof(sendString) / sizeof(sendString[0]); i++) {
        struct BusBlockWriteParam writeParam;
        writeParam.buffer = (void *)sendString[i];
        writeParam.size = sizeof(sendString[i]);
        BusDevWriteData(dev, &writeParam);

        Delay_Ms(200);
        struct BusBlockReadParam readParam;
        char buffer[256];
        readParam.buffer = (void *)buffer;
        readParam.size = 256;
        readParam.read_length = 0;
        int res = BusDevReadData(dev, &readParam);
        for (int i = 0; i < readParam.read_length; i++) {
            KPrintf("0x%02X ", buffer[i]);
        }
        KPrintf("\n");
    }

    /*step 5: read serial data, configure struct BusBlockWriteParam*/
    // Delay_Ms(1000);
    // struct BusBlockReadParam readParam;
    // char buffer[100];
    // readParam.buffer = (void *)buffer;
    // readParam.size = 256;
    // readParam.read_length = 0;
    // int res = BusDevReadData(dev, &readParam);
    // for (int i = 0; i < readParam.read_length; i++) {
    //     KPrintf("0x%02X ", buffer[i]);
    // }
    // KPrintf("\n");
    // if (EOK == res) {
    //     KPrintf("read_length %d\n", readParam.read_length);
    //     for (int i = 0; i < readParam.read_length; i++) {
    //         KPrintf("0x%02X ", buffer[i]);
    //     }
    //     KPrintf("\n");
    // } else {
    //     KPrintf("read data failed\n");
    // }

    /*step 6: close bus_device*/
    BusDevClose(dev);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), rs485_test_resources,
                 rs485_test_resources, test rs485 on the resources layer);
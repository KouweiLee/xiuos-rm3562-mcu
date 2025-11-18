/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 * PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the
 * Mulan PSL v2 for more details.
 */

/**
 * @file connect_ble.c
 * @brief support ch32v208 rbt6 ble function
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-12
 */

#include <connect_ble.h>

/**
 * @brief  初始化蓝牙模块
 * @return int 暂无意义
 */
int InitHwBle(void) {
    KPrintf("%s\n", VER_LIB);
    WCHBLE_Init();             // 库初始化
    HAL_Init();                // 蓝牙意见初始化
    GAPRole_PeripheralInit();  // GAP从机角色初始化
    Peripheral_Init();         // 从机初始化
    app_uart_init();           // 初始化蓝牙底层缓存
    KPrintf("BLE Peripheral Slave Init Success.\n");
}
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
* @file connect_can.h
* @brief define aiit-arm32-board can function and struct
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2025-02-22
*/

#ifndef CONNECT_CAN_H
#define CONNECT_CAN_H

#include <device.h>
#include "stm32f1xx_hal_can.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_def.h"


#ifdef __cplusplus
extern "C" {
#endif

struct Stm32Can
{
    CAN_HandleTypeDef instance;
    char *bus_name;
    struct CanBus can_bus;
    uint8 can_recv_flag;
    uint8_t	can_Rx_Data[8];
};

#define CAN_RxExtId 0x1800D8D0
#define CAN_TxExtId 0x1800D0D8

int InitHwCan(void);

#ifdef __cplusplus
}
#endif

#endif
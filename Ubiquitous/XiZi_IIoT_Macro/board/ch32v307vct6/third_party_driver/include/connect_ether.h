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
 * @file connect_ethernet.h
 * @brief define rvstar uart function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2022-08-01
 */

#ifndef CONNECT_ETH_H
#define CONNECT_ETH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t InitHwEth();
void WCHNET_CreateTcpSocket(uint8_t* DESIP, uint16_t srcport, uint16_t desport, uint8_t* SocketId);
void WCHNET_CreateTcpSocketListen(uint16_t srcport, uint8_t* SocketId);

int WCHNET_HandleGlobalInt(void);

#define TIME_OUT -1

#ifdef __cplusplus
}
#endif

#endif
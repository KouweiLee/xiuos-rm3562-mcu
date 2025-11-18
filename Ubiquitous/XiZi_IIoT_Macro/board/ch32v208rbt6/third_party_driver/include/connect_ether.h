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

#include <ModuleConfig.h>
#include <eth_driver.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum WchnetSocketStatus {
    WCHNET_SOCKET_UNDEFINED,        // SOCKET未定义
    WCHNET_SOCKET_CONNECT_SUCCESS,  // SOCKET连接成功
    WCHNET_SOCKET_CONNECT_TIMEOUT   // SOCKET连接超时
};

enum WchnetDhcpStatus {
    WCHNET_DHCP_UNDEFINED,    // DHCP未开始
    WCHNET_DHCP_SUCCESS_OLD,  // DHCP成功，但是获取到的IP地址和之前的IP地址一致
    WCHNET_DHCP_SUCCESS_NEW,  // DHCP成功，但获取到的IP地址和之前的IP地址不一致
    WCHNET_DHCP_FAIL          // DHCP失败
};

struct WchnetSocketConfiguration {
    pthread_t wchnetMainThread;  // wchnet主任务线程号

    unsigned char socketId;                 // 由WCHNET_SocketCreat函数创建时参数套接字id
    unsigned char destinationIpAddress[4];  // 目的IP地址
    unsigned char destinationPort[2];       // 目的端口号
    unsigned char dhcpSwitch;               // DHCP开关
    unsigned char macAddress[6];            // MAC地址
    unsigned char sourceIpAddress[4];       // 源IP地址
    unsigned char sourcePort[2];            // 源端口号
    unsigned char sourceSubnetMask[4];      // 子网掩码
    unsigned char sourceGateway[4];         // 网关地址

    unsigned char dhcpStatus;    // DHCP状态
    unsigned char socketStatus;  // socket状态
};

void mStopIfError(uint8_t iError);
void TIM2_Init(void);
uint8_t InitHwEth();
int WCHNET_CreateTcpSocket(uint8_t *destinationIP, uint16_t destinationPort, uint16_t sourcePort, uint8_t *socketId);
void WCHNET_HandleGlobalInt(struct WchnetSocketConfiguration *WchnetSocketConfiguration);
uint8_t WCHNET_DHCPCallBack(u8 status, void *arg);

#ifdef __cplusplus
}
#endif

#endif
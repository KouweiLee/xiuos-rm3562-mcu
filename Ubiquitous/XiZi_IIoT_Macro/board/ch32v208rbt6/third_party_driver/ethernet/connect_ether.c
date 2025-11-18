/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/05/31
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <ModuleConfig.h>
#include <connect_ether.h>
#include <shell.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_tim.h"
#include "core_riscv.h"
#include "eth_driver.h"
#include "ping.h"
#include "string.h"
#include "xs_base.h"

extern uint32_t SystemCoreClock;
#define KEEPALIVE_ENABLE 1  // Enable keep alive function

uint8_t MyBuf[RECE_BUF_LEN];
uint8_t SocketRecvBuf[WCHNET_MAX_SOCKET_NUM][RECE_BUF_LEN];  // socket receive buffer

/**
 * @brief  检查wchnet库函数调用结果，并输出错误原因
 * @param  iError wchnet库函数调用返回结果
 */
void mStopIfError(uint8_t iError) {
    if (iError == WCHNET_ERR_SUCCESS) return;
    printf("Error: %02X\r\n", (uint16_t)iError);
    switch (iError) {
        case WCHNET_ERR_SUCCESS:
            printf("No error, everything OK");
            break;
        case WCHNET_ERR_BUSY:
            printf("busy");
            break;
        case WCHNET_ERR_MEM:
            printf("Out of memory error");
            break;
        case WCHNET_ERR_BUF:
            printf("Buffer error");
            break;
        case WCHNET_ERR_TIMEOUT:
            printf("Timeout");
            break;
        case WCHNET_ERR_RTE:
            printf("Routing problem");
            break;
        case WCHNET_ERR_ABRT:
            printf("Connection aborted");
            break;
        case WCHNET_ERR_RST:
            printf("Connection reset");
            break;
        case WCHNET_ERR_CLSD:
            printf("Connection closed");
            break;
        case WCHNET_ERR_CONN:
            printf("Not connected");
            break;
        case WCHNET_ERR_VAL:
            printf("Illegal value");
            break;
        case WCHNET_ERR_ARG:
            printf("Illegal argument");
            break;
        case WCHNET_ERR_USE:
            printf("Address in use");
            break;
        case WCHNET_ERR_IF:
            printf("Low-level netif error");
            break;
        case WCHNET_ERR_ISCONN:
            printf("Already connected");
            break;
        case WCHNET_ERR_INPROGRESS:
            printf("Operation in progress");
            break;
        case WCHNET_ERR_SOCKET_MEM:
            printf("Socket information error");
            break;
        case WCHNET_ERR_UNSUPPORT_PROTO:
            printf("Unsupported protocol type");
            break;
        case WCHNET_RET_ABORT:
            printf("Command process fail");
            break;
        case WCHNET_ERR_UNKNOW:
            printf("Unknown error");
            break;
        default:
            printf("Unknown error code");
            break;
    }
    printf("\n");
}

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   Initializes TIM2.
 *
 * @return  none
 */
void TIM2_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = SystemCoreClock / 1000000;
    TIM_TimeBaseStructure.TIM_Prescaler = WCHNETTIMERPERIOD * 1000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief  创建并连接TCPsocket会话
 * @param  destinationIP 目标IP地址
 * @param  destinationPort 目标端口号
 * @param  socketId socket会话ID指针，最后会将创建的socketId保存到该指针中
 * @return int 如果会话创建并且执行连接会话成功，则返回WCHNET_ERR_SUCCESS，否则返回其他错误代码
 * @note
 * 该方法不保证socket连接建立成功或者失败，需要通过查询中断得知结果。连接成功后会产生连接中断SINT_STAT_CONNECT。如果远端不在线或
 * 端口未打开，库会自动重试一定次数，仍然不成功会产生超时中断 SINT_STAT_TIM_OUT。
 */
int WCHNET_CreateTcpSocket(uint8_t *destinationIP, uint16_t destinationPort, uint16_t sourcePort, uint8_t *socketId) {
    /* 配置socket会话 */
    SOCK_INF TmpSocketInf;
    memset((void *)&TmpSocketInf, 0, sizeof(SOCK_INF));
    memcpy((void *)TmpSocketInf.IPAddr, destinationIP, 4);
    TmpSocketInf.DesPort = destinationPort;
    TmpSocketInf.SourPort = sourcePort;
    TmpSocketInf.ProtoType = PROTO_TYPE_TCP;
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;

    /* 创建socket会话 */
    uint8_t res = WCHNET_SocketCreat(socketId, &TmpSocketInf);
    mStopIfError(res);
    if (res == WCHNET_ERR_SUCCESS) {
        KPrintf("create socketId %d\r\n", *socketId);
    } else {
        return res;
    }

    /* 连接socket会话 */
    res = WCHNET_SocketConnect(*socketId);  // make a TCP connection
    mStopIfError(res);
    if (res == WCHNET_ERR_SUCCESS) {
        KPrintf("connecting socketId %d\r\n", *socketId);
    }
    return res;
}

/*********************************************************************
 * @fn      WCHNET_DataLoopback
 *
 * @brief   Data loopback function.
 *
 * @param   id - socket id.
 *
 * @return  none
 */
void WCHNET_DataLoopback(uint8_t id) {
    u32 len, totallen;
    uint8_t *p = MyBuf, TransCnt = 255;

    len = WCHNET_SocketRecvLen(id, NULL);  // query length
    KPrintf("Receive Len = %d\r\n", len);
    totallen = len;
    WCHNET_SocketRecv(id, MyBuf, &len);  // Read the data of the receive buffer into MyBuf
    while (1) {
        len = totallen;
        WCHNET_SocketSend(id, p, &len);  // Send the data
        totallen -= len;                 // Subtract the sent length from the total length
        p += len;                        // offset buffer pointer
        if (!--TransCnt) break;          // Timeout exit
        if (totallen) continue;          // If the data is not sent, continue to send
        break;                           // After sending, exit
    }
}

/**
 * @brief  socket套接字中断服务程序
 * @param  socketid
 * @param  intstat 中断状态
 * @return int
 */

/**
 * @brief  socket套接字中断服务程序
 * @param  socketid
 * @param  intstat 中断状态
 * @param  pWchnetSocketConfiguration wchnet配置指针
 */
void WCHNET_HandleSockInt(uint8_t socketid, uint8_t intstat, struct WchnetSocketConfiguration *pWchnetSocketConfiguration) {
    uint8_t i;
    u32 len;

    /* 检查是否有接收数据的中断 */
    if (intstat & SINT_STAT_RECV) {
        // WCHNET_DataLoopback(socketid);  // 数据回环
        // len = WCHNET_SocketRecvLen(socketid, NULL);  // 获取接收缓冲区数据长度
        // WCHNET_SocketRecv(socketid, MyBuf, &len);    // 将接收缓冲区的数据读取到 MyBuf
        // WCHNET_ICMPRecvData(len, MyBuf);
    }

    /* 检查是否有连接成功的中断 */
    if (intstat & SINT_STAT_CONNECT) {
#if KEEPALIVE_ENABLE
        WCHNET_SocketSetKeepLive(socketid, ENABLE);  // 启用保活机制
#endif
        WCHNET_ModifyRecvBuf(socketid, (u32)SocketRecvBuf[socketid], RECE_BUF_LEN);  // 修改接收缓冲区
        pWchnetSocketConfiguration->socketStatus = WCHNET_SOCKET_CONNECT_SUCCESS;    // 更新socket连接状态
        KPrintf("[socketid-%d]TCP Connect Success\n", socketid);
    }

    /* 检查是否有断开连接的中断 */
    if (intstat & SINT_STAT_DISCONNECT) {
        pWchnetSocketConfiguration->socketStatus = WCHNET_SOCKET_UNDEFINED;  // 更新socket连接状态
        KPrintf("[socketid-%d]TCP Disconnect\n", socketid);                  // 打印断开连接信息
    }

    /* 检查是否有超时断开的中断 */
    if (intstat & SINT_STAT_TIM_OUT) {
        pWchnetSocketConfiguration->socketStatus = WCHNET_SOCKET_CONNECT_TIMEOUT;  // 更新socket连接状态
        KPrintf("[socketid-%d]TCP Timeout Error\n", socketid);                     // 打印超时信息
    }
}

/**
 * @brief  wchnet中断服务程序
 * @param  pWchnetSocketConfiguration wchnet配置指针
 */
void WCHNET_HandleGlobalInt(struct WchnetSocketConfiguration *pWchnetSocketConfiguration) {
    uint8_t intstat;
    uint16_t i;

    intstat = WCHNET_GetGlobalInt();  // 获取全局中断标志

    /* 不可达中断 */
    if (intstat & GINT_STAT_UNREACH) {
        KPrintf("GINT_STAT_UNREACH\n");
    }

    /* IP冲突 */
    if (intstat & GINT_STAT_IP_CONFLI) {
        KPrintf("GINT_STAT_IP_CONFLI\r\n");
    }

    /* PHY状态变化，例如插入网线 */
    if (intstat & GINT_STAT_PHY_CHANGE) {
        i = WCHNET_GetPHYStatus();  // 获取PHY状态
        if (i & PHY_Linked_Status) {
            // ICMPSuc = ICMP_SOKE_CON;        // 设置ICMP成功状态
            KPrintf("PHY Link Success\n");  // 打印PHY连接成功信息
        }
    }

    /* socket套接字相关中断 */
    if (intstat & GINT_STAT_SOCKET) {
        uint8_t socketint;
        for (int socketid = 0; socketid < WCHNET_MAX_SOCKET_NUM; socketid++) {
            socketint = WCHNET_GetSocketInt(socketid);  // 获取套接字中断标志
            if (socketint) {
                WCHNET_HandleSockInt(socketid, socketint, pWchnetSocketConfiguration);  // 处理套接字中断
            }
        }
    }
}

/**
 * @brief  DHCP回调函数
 * @param  status DHCP状态，0为成功，其他值失败
 * @param  arg 当DHCP成功时，该指针指向的地址依次保存了IP地址，网关地址，子网掩码，主DNS和次DNS，一共20个字节。
 * @return u8
 */
u8 WCHNET_DHCPCallBack(u8 status, void *arg) {
    u8 *p;
    extern struct WchnetSocketConfiguration wchnetSocketConfiguration;
    if (!status) {  // DHCP成功
        p = arg;
        printf("*-*-*-*-*-*-*DHCP Success*-*-*-*-*-*-*\n");
        if (!memcmp(wchnetSocketConfiguration.sourceIpAddress, p,
                    sizeof(wchnetSocketConfiguration.sourceIpAddress))) {  // 如果DHCP获取到的动态IP地址和当前的地址一致
            wchnetSocketConfiguration.dhcpStatus = WCHNET_DHCP_SUCCESS_OLD;
            return READY;
        } else {  // 如果DHCP获取到的动态IP地址和当前的地址不一致
            memcpy(wchnetSocketConfiguration.sourceIpAddress, p, 4);
            memcpy(wchnetSocketConfiguration.sourceGateway, &p[4], 4);
            memcpy(wchnetSocketConfiguration.sourceSubnetMask, &p[8], 4);
            printf("sourceIpAddress  = %d.%d.%d.%d \r\n", (u16)wchnetSocketConfiguration.sourceIpAddress[0],
                   (u16)wchnetSocketConfiguration.sourceIpAddress[1], (u16)wchnetSocketConfiguration.sourceIpAddress[2],
                   (u16)wchnetSocketConfiguration.sourceIpAddress[3]);
            printf("sourceGateway    = %d.%d.%d.%d \r\n", (u16)wchnetSocketConfiguration.sourceGateway[0],
                   (u16)wchnetSocketConfiguration.sourceGateway[1], (u16)wchnetSocketConfiguration.sourceGateway[2],
                   (u16)wchnetSocketConfiguration.sourceGateway[3]);
            printf("sourceSubnetMask = %d.%d.%d.%d \r\n", (u16)wchnetSocketConfiguration.sourceSubnetMask[0],
                   (u16)wchnetSocketConfiguration.sourceSubnetMask[1], (u16)wchnetSocketConfiguration.sourceSubnetMask[2],
                   (u16)wchnetSocketConfiguration.sourceSubnetMask[3]);
            printf("*-*-*-*-*-*-**-*-*-*-*-*-*-*-*-*-*-*-*\n");
            WCHNET_DHCPStop();  // 停止DHCP
            wchnetSocketConfiguration.dhcpStatus = WCHNET_DHCP_SUCCESS_NEW;
            return READY;
        }
    } else {  // DHCP失败
        printf("DHCP Fail %02x \r\n", status);
        WCHNET_DHCPStop();
        wchnetSocketConfiguration.dhcpStatus = WCHNET_DHCP_FAIL;
        return NoREADY;
    }
}

/**
 * @brief  命令行显示网络配置信息
 * @return int 暂无意义
 * @note 命令形式：ifconfig
 */
int ifconfig() {
    extern struct WchnetSocketConfiguration wchnetSocketConfiguration;
    KPrintf("=== wchnet version %3d ===\n", WCHNET_GetVer());
    WCHNET_GetMacAddr(wchnetSocketConfiguration.macAddress);  // 查询MAC地址
    KPrintf("MACAddr:  %02X:%02X:%02X:%02X:%02X:%02X\n", wchnetSocketConfiguration.macAddress[0], wchnetSocketConfiguration.macAddress[1],
            wchnetSocketConfiguration.macAddress[2], wchnetSocketConfiguration.macAddress[3], wchnetSocketConfiguration.macAddress[4],
            wchnetSocketConfiguration.macAddress[5]);
    KPrintf("IPAddr:   %d.%d.%d.%d\n", wchnetSocketConfiguration.sourceIpAddress[0], wchnetSocketConfiguration.sourceIpAddress[1],
            wchnetSocketConfiguration.sourceIpAddress[2], wchnetSocketConfiguration.sourceIpAddress[3]);
    KPrintf("GWIPAddr: %d.%d.%d.%d\n", wchnetSocketConfiguration.sourceGateway[0], wchnetSocketConfiguration.sourceGateway[1],
            wchnetSocketConfiguration.sourceGateway[2], wchnetSocketConfiguration.sourceGateway[3]);
    KPrintf("IPMask:   %d.%d.%d.%d\n", wchnetSocketConfiguration.sourceSubnetMask[0], wchnetSocketConfiguration.sourceSubnetMask[1],
            wchnetSocketConfiguration.sourceSubnetMask[2], wchnetSocketConfiguration.sourceSubnetMask[3]);
    KPrintf("==========================\n");
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ifconfig, ifconfig, printf wchnet configurations);

/**
 * @brief  进行以太网初始化
 * @return uint8_t
 */
uint8_t InitHwEth() {
    /* 以太网必须运行在系统时钟为60M或者120M的系统上 */
    if ((SystemCoreClock == 60000000) || (SystemCoreClock == 120000000)) {
        printf("SystemClk:%d\n", SystemCoreClock);
    } else {
        printf("Error: Please choose 60MHz and 120MHz clock when using Ethernet!\n");
        return -1;
    }

    /* 比对库版本号 */
    KPrintf("net version:%x\n", WCHNET_GetVer());
    KPrintf("WCHNET_LIB_VER: %x\n", WCHNET_LIB_VER);
    if (WCHNET_LIB_VER != WCHNET_GetVer()) {
        KPrintf("Error: wchnet lib version error.\n");
        return -1;
    }

    /* 库函数需要外部提供时钟，用于给时间相关的任务提供时钟源，例如刷新ARP列表，TCP超时等，通过调用WCHNET_TimeIsr
     * 函数更新时间，该函数传递的参数为最近一次调用的时间差值，单位毫秒。*/
    TIM2_Init();

    /* 进行库初始化 */
    extern pmodule_cfg CFG;
    extern struct WchnetSocketConfiguration wchnetSocketConfiguration;
    memcpy(wchnetSocketConfiguration.sourceIpAddress, CFG->sourceIpAddress_Ethernet,
           sizeof(CFG->sourceIpAddress_Ethernet));  // IP地址
    memcpy(wchnetSocketConfiguration.sourceGateway, CFG->sourceGateway_Ethernet,
           sizeof(CFG->sourceGateway_Ethernet));  // 默认网关
    memcpy(wchnetSocketConfiguration.sourceSubnetMask, CFG->sourceSubnetMask_Ethernet,
           sizeof(CFG->sourceSubnetMask_Ethernet));           // 子网掩码
    WCHNET_GetMacAddr(wchnetSocketConfiguration.macAddress);  // MAC地址
    uint8_t res = ETH_LibInit(wchnetSocketConfiguration.sourceIpAddress, wchnetSocketConfiguration.sourceGateway,
                              wchnetSocketConfiguration.sourceSubnetMask, wchnetSocketConfiguration.macAddress);  // 库初始化
    mStopIfError(res);
    if (res == WCHNET_ERR_SUCCESS) KPrintf("WCHNET_LibInit Success\r\n");

#if KEEPALIVE_ENABLE  // Configure keep alive parameters
                      /* 保活报文配置 */
    {
        struct _KEEP_CFG cfg;
        cfg.KLIdle = 20000;
        cfg.KLIntvl = 15000;
        cfg.KLCount = 9;
        WCHNET_ConfigKeepLive(&cfg);
    }
#endif

    return res;
}

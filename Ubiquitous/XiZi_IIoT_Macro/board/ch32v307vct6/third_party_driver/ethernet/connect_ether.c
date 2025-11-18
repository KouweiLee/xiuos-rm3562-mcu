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
/*
 *@Note
TCP Client example, demonstrating that TCP Client connects
to the server and receives data and then sends it back.
For details on the selection of engineering chips,
please refer to the "CH32V30x Evaluation Board Manual" under the CH32V307EVT\EVT\PUB folder.
 */

#include "connect_ether.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_tim.h"
#include "core_riscv.h"
#include "eth_driver.h"
#include "string.h"
#include "xs_base.h"

extern uint32_t SystemCoreClock;
#define KEEPALIVE_ENABLE 1 // Enable keep alive function

uint8_t MACAddr[6]; // MAC address
uint8_t IPAddr[4] = { 192, 168, 1, 10 }; // IP address
uint8_t GWIPAddr[4] = { 192, 168, 1, 1 }; // Gateway IP address
uint8_t IPMask[4] = { 255, 255, 255, 0 }; // subnet mask

uint8_t MyBuf[RECE_BUF_LEN];
uint8_t socket[WCHNET_MAX_SOCKET_NUM]; // Save the currently connected socket
uint8_t SocketRecvBuf[WCHNET_MAX_SOCKET_NUM][RECE_BUF_LEN]; // socket receive buffer

/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   check if error.
 *
 * @param   iError - error constants.
 *
 * @return  none
 */
void mStopIfError(uint8_t iError)
{
    if (iError == WCHNET_ERR_SUCCESS)
        return;
    KPrintf("Error: %02X\r\n", (uint16_t)iError);
}

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   Initializes TIM2.
 *
 * @return  none
 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };

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

/*********************************************************************
 * @fn      WCHNET_CreateTcpSocket
 *
 * @brief   Create TCP Socket
 *
 * @return  none
 */
void WCHNET_CreateTcpSocket(uint8_t* DESIP, uint16_t srcport, uint16_t desport, uint8_t* SocketId)
{
    uint8_t i;
    SOCK_INF TmpSocketInf;

    memset((void*)&TmpSocketInf, 0, sizeof(SOCK_INF));
    memcpy((void*)TmpSocketInf.IPAddr, DESIP, 4);
    TmpSocketInf.DesPort = desport;
    TmpSocketInf.SourPort = srcport++;
    TmpSocketInf.ProtoType = PROTO_TYPE_TCP;
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;
    i = WCHNET_SocketCreat(SocketId, &TmpSocketInf);
    KPrintf("SocketId %d\r\n", *SocketId);
    mStopIfError(i);
    i = WCHNET_SocketConnect(*SocketId); // make a TCP connection
    mStopIfError(i);
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
void WCHNET_DataLoopback(uint8_t id)
{
    u32 len, totallen;
    uint8_t *p = MyBuf, TransCnt = 255;

    len = WCHNET_SocketRecvLen(id, NULL); // query length
    KPrintf("Receive Len = %d\r\n", len);
    totallen = len;
    WCHNET_SocketRecv(id, MyBuf, &len); // Read the data of the receive buffer into MyBuf
    while (1) {
        len = totallen;
        WCHNET_SocketSend(id, p, &len); // Send the data
        totallen -= len; // Subtract the sent length from the total length
        p += len; // offset buffer pointer
        if (!--TransCnt)
            break; // Timeout exit
        if (totallen)
            continue; // If the data is not sent, continue to send
        break; // After sending, exit
    }
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  0 or TIME_OUT
 */
int WCHNET_HandleSockInt(uint8_t socketid, uint8_t intstat)
{
    uint8_t i;

    if (intstat & SINT_STAT_RECV) // receive data
    {
        WCHNET_DataLoopback(socketid); // Data loopback
    }
    if (intstat & SINT_STAT_CONNECT) // connect successfully
    {
#if KEEPALIVE_ENABLE
        WCHNET_SocketSetKeepLive(socketid, ENABLE);
#endif
        WCHNET_ModifyRecvBuf(socketid, (u32)SocketRecvBuf[socketid], RECE_BUF_LEN);
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
            if (socket[i] == 0xff) { // save connected socket id
                socket[i] = socketid;
                break;
            }
        }
        KPrintf("TCP Connect Success\r\n");
        KPrintf("socket id: %d\r\n", socket[i]);
    }
    if (intstat & SINT_STAT_DISCONNECT) // disconnect
    {
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) { // delete disconnected socket id
            if (socket[i] == socketid) {
                socket[i] = 0xff;
                break;
            }
        }
        KPrintf("TCP Disconnect\r\n");
    }
    if (intstat & SINT_STAT_TIM_OUT) // timeout disconnect
    {
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) { // delete disconnected socket id
            if (socket[i] == socketid) {
                socket[i] = 0xff;
                break;
            }
        }
        KPrintf("TCP Timeout\r\n");
        return TIME_OUT;
    }
    return 0;
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global Interrupt Handle
 *
 * @return  0 or SockInt
 */
int WCHNET_HandleGlobalInt(void)
{
    uint8_t intstat;
    uint16_t i;
    uint8_t socketint;

    intstat = WCHNET_GetGlobalInt(); // get global interrupt flag
    if (intstat & GINT_STAT_UNREACH) // Unreachable interrupt
    {
        KPrintf("GINT_STAT_UNREACH\r\n");
    }
    if (intstat & GINT_STAT_IP_CONFLI) // IP conflict
    {
        KPrintf("GINT_STAT_IP_CONFLI\r\n");
    }
    if (intstat & GINT_STAT_PHY_CHANGE) // PHY status change
    {
        i = WCHNET_GetPHYStatus();
        if (i & PHY_Linked_Status)
            KPrintf("PHY Link Success\r\n");
    }
    if (intstat & GINT_STAT_SOCKET) { // socket related interrupt
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
            socketint = WCHNET_GetSocketInt(i);
            if (socketint) {
                return WCHNET_HandleSockInt(i, socketint);
            }
        }
    }
    return 0;
}

uint8_t InitHwEth()
{
    uint8_t i = 0;

    KPrintf("net version:%x\n", WCHNET_GetVer());
    if (WCHNET_LIB_VER != WCHNET_GetVer()) {
        KPrintf("version error.\n");
    }

    WCHNET_GetMacAddr(MACAddr); // get the chip MAC address
    KPrintf("mac addr:");
    for (i = 0; i < 6; i++)
        KPrintf("%x ", MACAddr[i]);
    KPrintf("\n");

    TIM2_Init();

    i = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr); // Ethernet library initialize
    mStopIfError(i);
    if (i == WCHNET_ERR_SUCCESS)
        KPrintf("WCHNET_LibInit Success\r\n");
#if KEEPALIVE_ENABLE // Configure keep alive parameters
    {
        struct _KEEP_CFG cfg;

        cfg.KLIdle = 20000;
        cfg.KLIntvl = 15000;
        cfg.KLCount = 9;
        WCHNET_ConfigKeepLive(&cfg);
    }
#endif

    memset(socket, 0xff, WCHNET_MAX_SOCKET_NUM);

    return i;
}

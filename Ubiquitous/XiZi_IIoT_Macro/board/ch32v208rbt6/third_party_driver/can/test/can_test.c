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
 * @file can_test.c
 * @brief test ch32v208 can
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-03-14
 */
#include <ch32v20x_gpio.h>
#include <ch32v20x_misc.h>
#include <ch32v20x_rcc.h>

#include "ch32v20x.h"
#include "connect_can.h"
#include "debug.h"
#include "shell.h"

/* CAN Mode Definition */
#define TX_MODE 0
#define RX_MODE 1

/* Frame Format Definition */
#define Standard_Frame 0
#define Extended_Frame 1

/* CAN Communication Mode Selection */
#define CAN_MODE TX_MODE
// #define CAN_MODE   RX_MODE

/* Frame Formate Selection */
#define Frame_Format Standard_Frame
// #define Frame_Format   Extended_Frame

/*********************************************************************
 * @fn      CAN_Mode_Init
 *
 * @brief   Initializes CAN communication test mode.
 *          Bps =Fpclk1/((tpb1+1+tbs2+1+1)*brp)
 *          波特率设置https://www.cnblogs.com/wchmcu/p/17546797.html
 *
 * @param   tsjw - CAN synchronisation jump width.
 *          tbs2 - CAN time quantum in bit segment 1.
 *          tbs1 - CAN time quantum in bit segment 2.
 *          brp - Specifies the length of a time quantum.
 *          mode - Test mode.
 *            CAN_Mode_Normal.
 *            CAN_Mode_LoopBack.
 *            CAN_Mode_Silent.
 *            CAN_Mode_Silent_LoopBack.
 *
 * @return  none
 */
static int init_can(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode) {
    CAN_InitTypeDef CAN_InitSturcture = {0};
    CAN_FilterInitTypeDef CAN_FilterInitSturcture = {0};

    CAN_InitSturcture.CAN_TTCM = DISABLE;
    CAN_InitSturcture.CAN_ABOM = DISABLE;
    CAN_InitSturcture.CAN_AWUM = DISABLE;
    CAN_InitSturcture.CAN_NART = ENABLE;
    CAN_InitSturcture.CAN_RFLM = DISABLE;
    CAN_InitSturcture.CAN_TXFP = DISABLE;
    CAN_InitSturcture.CAN_Mode = mode;
    CAN_InitSturcture.CAN_SJW = tsjw;
    CAN_InitSturcture.CAN_BS1 = tbs1;
    CAN_InitSturcture.CAN_BS2 = tbs2;
    CAN_InitSturcture.CAN_Prescaler = brp;
    CAN_Init(CAN1, &CAN_InitSturcture);

    CAN_FilterInitSturcture.CAN_FilterNumber = 0;

#if (Frame_Format == Standard_Frame)
    /* identifier/mask mode, One 32-bit filter, StdId: 0x317 */
    CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x62E0;
    CAN_FilterInitSturcture.CAN_FilterIdLow = 0;
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0xFFE0;
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0x0006;

    /* identifier/mask mode, Two 16-bit filters, StdId: 0x317锟斤拷0x316 */
    //	CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;
    //	CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_16bit;
    //	CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x62E0;
    //	CAN_FilterInitSturcture.CAN_FilterIdLow = 0xFFF8;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0x62C0;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0xFFF8;

    /* identifier list mode, One 32-bit filter, StdId: 0x317锟斤拷0x316 */
    //	CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdList;
    //	CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;
    //	CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x62E0;
    //	CAN_FilterInitSturcture.CAN_FilterIdLow = 0;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0x62C0;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0;

    /* identifier list mode, Two 16-bit filters, StdId: 0x317,0x316,0x315,0x314
     */
    //	CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdList;
    //	CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_16bit;
    //	CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x62E0;
    //	CAN_FilterInitSturcture.CAN_FilterIdLow = 0x62C0;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0x62A0;
    //	CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0x6280;

#elif (Frame_Format == Extended_Frame)
    /* identifier/mask mode, One 32-bit filter, ExtId: 0x12124567 */
    CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x9092;
    CAN_FilterInitSturcture.CAN_FilterIdLow = 0x2B3C;
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0xFFFE;

#endif

    CAN_FilterInitSturcture.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitSturcture.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitSturcture);

    return 0;
}

static u8 CAN_Send_Msg(u8 *msg, u8 len) {
    u8 mbox;
    u16 i = 0;

    CanTxMsg CanTxStructure;

    CanTxStructure.StdId = 0x317;
    CanTxStructure.IDE = CAN_Id_Standard;
    CanTxStructure.RTR = CAN_RTR_Data;
    CanTxStructure.DLC = len;

    for (i = 0; i < len; i++) {
        CanTxStructure.Data[i] = msg[i];
    }

    mbox = CAN_Transmit(CAN1, &CanTxStructure);

    i = 0;

    while ((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (i < 0xFFF)) {
        i++;
    }

    if (i == 0xFFF) {
        return 1;
    } else {
        return 0;
    }
}

static u8 CAN_Receive_Msg(u8 *buf) {
    u8 i;

    CanRxMsg CanRxStructure;

    if (CAN_MessagePending(CAN1, CAN_FIFO1) == 0) {
        return 0;
    }

    CAN_Receive(CAN1, CAN_FIFO1, &CanRxStructure);

    for (i = 0; i < 8; i++) {
        buf[i] = CanRxStructure.Data[i];
    }

    return CanRxStructure.DLC;
}

int test_can(int argc, char *argv[]) {
    u8 i;
    u8 cnt = 0;
    u8 tx, rx;
    u8 txbuf[8];
    u8 rxbuf[8];
    init_can(
        CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_6tq, 20,
        CAN_Mode_Normal);  // 在这种参数下，设置时钟源为外部时钟120M，对应的波特率是250kbps
#if (CAN_MODE == TX_MODE)
    for (cnt = 0; cnt < 8; cnt++) {
        for (i = 0; i < 8; i++) {
            txbuf[i] = cnt + i;
        }
        tx = CAN_Send_Msg(txbuf, 8);

        if (tx) {
            KPrintf("CAN1 Send Failed\r\n");
        } else {
            KPrintf("CAN1 Send Success\r\n");
            KPrintf("CAN1 Send Data:\r\n");

            for (i = 0; i < 8; i++) {
                KPrintf("%02x\r\n", txbuf[i]);
            }
        }
		Delay_Ms(1000);
    }
#elif (CAN_MODE == RX_MODE)
    rx = CAN_Receive_Msg(rxbuf);

    if (rx) {
        KPrintf("CAN1 Receive Data:\r\n");

        for (i = 0; i < 8; i++) {
            KPrintf("%02x\r\n", txbuf[i]);
        }
    } else {
        KPrintf("CAN1 No Receive Data\r\n");
    }
#endif
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 test_can, test_can, test CAN);
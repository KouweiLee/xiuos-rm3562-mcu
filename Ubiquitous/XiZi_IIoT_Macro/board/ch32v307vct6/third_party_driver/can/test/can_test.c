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
 * @file can_test.c
 * @brief test ch32v307 can
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-03-14
 */
#include "shell.h"
#include "ch32v30x.h"
#include "connect_can.h"
#include <ch32v30x_gpio.h>
#include <ch32v30x_rcc.h>
#include <ch32v30x_misc.h>

static int init_can()
{
    KPrintf("init can\r\n");

    CAN_FilterInitTypeDef can1_filter;
    CAN_InitTypeDef can_initstruction;

    can_initstruction.CAN_TTCM = DISABLE;
    can_initstruction.CAN_ABOM = DISABLE;
    can_initstruction.CAN_AWUM = DISABLE;
    can_initstruction.CAN_NART = ENABLE;
    can_initstruction.CAN_TXFP = DISABLE;
    can_initstruction.CAN_Mode = CAN_Mode_Silent_LoopBack;
    can_initstruction.CAN_RFLM = DISABLE;
    can_initstruction.CAN_SJW = CAN_SJW_1tq;
    can_initstruction.CAN_BS1 = CAN_BS1_6tq;
    can_initstruction.CAN_BS2 = CAN_BS2_5tq;
    can_initstruction.CAN_Prescaler = 12;
    CAN_Init(CAN1, &can_initstruction);

    can1_filter.CAN_FilterNumber = 0;
    can1_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can1_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can1_filter.CAN_FilterIdHigh = 0x0000;
    can1_filter.CAN_FilterIdLow = 0x0000;
    can1_filter.CAN_FilterMaskIdHigh = 0x0000;
    can1_filter.CAN_FilterMaskIdLow = 0x0006;
    can1_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
    can1_filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&can1_filter);

    return 0;
}

static u8 CAN_Send_Msg(u8 *msg, u8 len)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CanTxStructure;

    CanTxStructure.StdId = 0x317;
    CanTxStructure.IDE = CAN_Id_Standard;
    CanTxStructure.RTR = CAN_RTR_Data;
    CanTxStructure.DLC = len;

    for (i = 0; i < len; i++)
    {
        CanTxStructure.Data[i] = msg[i];
    }

    mbox = CAN_Transmit(CAN1, &CanTxStructure);

    i = 0;

    while ((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (i < 0xFFF))
    {
        i++;
    }

    if (i == 0xFFF)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static u8 CAN_Receive_Msg(u8 *buf)
{
    u8 i;

    CanRxMsg CanRxStructure;

    if (CAN_MessagePending(CAN1, CAN_FIFO1) == 0)
    {
        return 0;
    }

    CAN_Receive(CAN1, CAN_FIFO1, &CanRxStructure);

    for (i = 0; i < 8; i++)
    {
        buf[i] = CanRxStructure.Data[i];
    }

    return CanRxStructure.DLC;
}

int test_can(int argc, char *argv[])
{
    u8 i;
    u8 cnt = 0;
    u8 tx, rx;
    u8 txbuf[8];
    u8 rxbuf[8];
    init_can();

    for (i = 0; i < 8; i++)
    {
        txbuf[i] = cnt + i;
    }
    tx = CAN_Send_Msg(txbuf, 8);

    if (tx)
    {
        KPrintf("CAN1 Send Failed\r\n");
    }
    else
    {
        KPrintf("CAN1 Send Success\r\n");
        KPrintf("CAN1 Send Data:\r\n");

        for (i = 0; i < 8; i++)
        {
            KPrintf("%02x\r\n", txbuf[i]);
        }
    }

    rx = CAN_Receive_Msg(rxbuf);

    if (rx)
    {
        KPrintf("CAN1 Receive Data:\r\n");

        for (i = 0; i < 8; i++)
        {
            KPrintf("%02x\r\n", txbuf[i]);
        }
    }
    else
    {
        KPrintf("CAN1 No Receive Data\r\n");
    }

    cnt++;
    if (cnt == 0xFF)
    {
        cnt = 0;
    }

    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 test_can, test_can, test CAN);
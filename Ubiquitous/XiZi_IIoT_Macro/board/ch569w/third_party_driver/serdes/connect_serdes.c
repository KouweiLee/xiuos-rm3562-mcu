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
 * @file connect_usart.c
 * @brief support ch569 uart function and register to bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-04-07
 */

#include "CH56x_common.h"
#include "xizi.h"
#include "board.h"
#include "shell.h"
#include "connect_serdes.h"


//#define Trans 1
//#define Recei 1

//UINT32  Tx_DMAaddr   = 0x20020000;
//UINT32  RX_DMA0_addr = 0x20033330;
//UINT32  RX_DMA1_addr = 0x20034330;
UINT32 Tx_DMAaddr[512] __attribute__((aligned(16))) = {0};
UINT32 RX_DMA0_addr[512] __attribute__((aligned(16))) = {0};
UINT32 RX_DMA1_addr[512] __attribute__((aligned(16))) = {0};

volatile UINT32  i=0;
volatile UINT32  k=0;
/*******************************************************************************
* Function Name  : __attribute__((interrupt()))
* Description    : SERDES_IRQHandler hard push
* Input          : baudrate: UART1 communication baud rate.
* Return         : None
*******************************************************************************/
void SERDES_IRQHandler (void) __attribute__((interrupt()));

/*******************************************************************************
* Function Name  : SerdesTxTest
* Description    : SerdesTxTest program.
* Input          : None
* Return         : None
*******************************************************************************/
int SerdesTxTest(void)
{
    UINT32 *p32_txdma = (UINT32 *)Tx_DMAaddr;
    UINT32 n=0;
    UINT32 data=0;

    KPrintf("SerdesTxTest start\r\n");
    KPrintf("SerdesTxTest Tx_DMAaddr=0x%08x\r\n", (UINT32)Tx_DMAaddr);

#ifdef SERDES_TRANS        //Serdes query sent.
    Serdes_Tx_Init();

    do
    {
        *p32_txdma++ = data;
        data += 0x01010101;
        n++;
    }while(n!=512);
    mDelaymS(500);

    DMA_Tx_CFG( (UINT32)Tx_DMAaddr,2048,0x555555);               //512 address spaces  2048 bytes

    while(1)
    {
        KPrintf("SerdesTxTest DMA_Tx\r\n");
        DMA_Tx();
        Wait_Txdone();
        ClearITFlag(SDS_TX_INT_FLG);
        mDelaymS(5000);
    }
#endif

}

/*******************************************************************************
* Function Name  : SerdesRxTest
* Description    : SerdesRxTest program.
* Input          : None
* Return         : None
*******************************************************************************/
int SerdesRxTest(void)
{
	KPrintf("SerdesRxTest start\r\n");
	KPrintf("SerdesRxTest RX_DMA0_addr=0x%08x RX_DMA1_addr=0x%08x\r\n", (UINT32)RX_DMA0_addr, (UINT32)RX_DMA1_addr);

#ifdef SERIAL_RECEI       // Serdes interrupt reception.
	PFIC_EnableIRQ(INT_ID_SERDES);

	DoubleDMA_Rx_CFG((UINT32)RX_DMA0_addr , (UINT32)RX_DMA1_addr );
	Serdes_Rx_Init(10*1000*1000);
    Wait_commadone(10*1000*1000);
    Serdes_Rx_Init(10*1000*1000);
	EnableIT(SDS_RX_INT_EN|SDS_COMMA_INT_EN);
	
	KPrintf("SerdesRxTest init OK\r\n");
    while(1)
    {
        if(k==2)        // After each completion of the double-buffered reception, print the received data.
        {
            for(i=0;i<512;i++)
            {
                KPrintf("%x\r\n",*(UINT32 *)(0x20033330+4*i));
                KPrintf("%x\r\n",*(UINT32 *)(0x20034330+4*i));
            }
            k=0;
        }
    }
#endif
}

/*******************************************************************************
* Function Name  : SERDES_IRQHandler
* Description    : Interruption function
* Input          : None
* Return         : None
*******************************************************************************/
void SERDES_IRQHandler (void)
{
	if(ReadITFlagBit(SDS_RX_INT_FLG))
	{
		ClearITFlag(SDS_RX_INT_FLG);
		k++;
		DMA_Rx_check((UINT32)RX_DMA0_addr, (UINT32)RX_DMA1_addr, 0x555555);
		KPrintf("\nbuffer received ....... !\n");
	}
	if(ReadITFlagBit(SDS_COMMA_INT_FLG))
	{
		KPrintf("\nreceive COMMA, init....... !\n");
		Serdes_Rx_Init( 1000*30 );
		i = 0;
		mDelaymS(50);
		ClearITFlag(SDS_COMMA_INT_FLG);
	}
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 SerdesTxTest, SerdesTxTest, test serdes tx);

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 SerdesRxTest, SerdesRxTest, test serdes rx);

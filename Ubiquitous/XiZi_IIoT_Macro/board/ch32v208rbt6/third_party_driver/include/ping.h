/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/06/11
* Description        : Definition for PING.c.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __PINC_H__
#define __PINC_H__
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "wchnet.h"
#include "debug.h"

// ICMP常量定义
// 用于标识ICMP消息的不同类型和状态
#define ICMP_SOKE_CON                0 // 初始化状态
#define ICMP_SEND_ERR                1 // 发送错误
#define ICMP_SEND_SUC                2 // 发送成功
#define ICMP_RECV_ERR                3 // 接收错误
#define ICMP_RECV_SUC                4 // 接收成功
#define ICMP_UNRECH                  5 // 无法到达
#define ICMP_REPLY                   6 // 回复
#define ICMP_REPLY_SUC               7 // 回复成功
#define ICMP_KEEP_NO                 10 // 保持连接数

// ICMP头部常量定义
// 用于构建和解析ICMP头部信息
#define ICMP_HEAD_TYPE               8 // ICMP头部类型
#define ICMP_HEAD_REPLY              0 // 回复消息类型
#define ICMP_HEAD_CODE               0 // 头部代码
#define ICMP_HEAD_ID                 512 // 消息ID
#define ICMP_HEAD_SEQ                100 // 序列号
#define ICMP_DATA_BYTES              32 // 数据区字节数

// Ping发送计数定义
// 用于控制Ping请求发送的次数
#define PING_SEND_CNT                5 // 默认发送5次请求

/*ICMP header field data structure*/
typedef struct _icmphdr 
{
    u8   i_type;                 //ICMP message type
    u8   i_code;                 //code number in the type
    u16  i_cksum;                //checksum
    u16  i_id;                   //identifier
    u16  i_seq;                  //sequence
    u8   i_data[32];             //data area
}IcmpHeader,*IcmpHead;

extern u8 DESIP[4];
extern u8 ICMPSuc;

extern void InitPING( void );

extern void InitParameter( void );

extern void WCHNET_PINGCmd( void );

extern void WCHNET_PINGInit( void );

extern void mStopIfError(u8 iError);

extern void Respond_PING( u8 *pDat );

extern void WCHNET_ICMPRecvData( u32 len, u8 *pDat );

extern void WCHNET_PINGSendData( u8 *PSend, u32 Len,u8 id );

extern void WCHNET_ProcessReceDat( char *recv_buff,u8 check_type,u8 socketid );

#endif

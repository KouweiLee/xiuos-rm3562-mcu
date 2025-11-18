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

#ifndef __AMP_MSG_CFG_H__
#define __AMP_MSG_CFG_H__

/**
 * 使用共享内存管理模块传输核间通信的数据
 * 使用消息队列传递核间通知消息
 * 优势：相较于直接将数据嵌入消息，使用共享内存管理模块能够更有效的利用共享内存
*/
#define PROTOCOL_AMP (100U)
/**
 * 将待发送数据直接嵌入消息队列里面，通知、数据一体传输
*/
#define PROTOCOL_PRIVATE (200U)

#define PROTOCOL_CHOICE (PROTOCOL_AMP) /* 消息协议选择 */

#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE
/* 使用数据嵌入消息队列协议时允许用户嵌入的最大消息数量（字节） */
#define MSG_PAYLOAD_SIZE (256U) 
#endif /* PROTOCOL_CHOICE == PROTOCOL_PRIVATE */

#endif /* __AMP_MSG_CFG_H__ */
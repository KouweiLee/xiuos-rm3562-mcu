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

#ifndef __AMP_MSG_QUEUE_CFG_H__
#define __AMP_MSG_QUEUE_CFG_H__

#define MSG_QUEUE_MUTEX_SIZE (128U) /* 位于内核中的消息队列互斥区域将按照这个大小进行划分 */

#define MSG_QUEUE_MARK_BUSY (0xFFFFFFFFU) /* 负责处理该消息队列的核心正在处理（16位） */
#define MSG_QUEUE_MARK_IDLE (0xAAAAAAAAU) /* 负责处理该消息队列的核心正在闲置（16位） */

#define MSG_QUEUE_MAX_ENTRY_CNT (0xFFFFU) /* 消息队列中能够管理的最大消息数量：uint16_t */
#endif /* __AMP_MSG_QUEUE_CFG_H__ */
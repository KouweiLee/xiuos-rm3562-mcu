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

#include "config/config.h"
#include "msgqueue.h"

#ifndef __AMP_CHANNEL_H__
#define __AMP_CHANNEL_H__

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
struct Channel /* 核间通道结构：应用层结构 */
{
    struct ChannelInfo* channel_info;      /* 通道配置信息 */
    struct AmpMsgQueue* msg_queue;         /* 发往远程核心的消息队列 */
    struct MsgQueueMutex* msg_queue_mutex; /* 消息队列互斥体 */

    uint32_t *reg_msg;                     /* 该地址写1触发核间消息通知 */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
    void* msg_queue_mutex_start;           /* 内核中的消息队列互斥体群 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */
};

struct ChannelOps
{
    /* 初始化所有的通道信息 */
    int32_t (*channels_init)(void);

    /* 检查指定通道是否已经可以使用 */
    int32_t (*channel_is_ready)(struct Channel* channel);

    /* 获取发往目标核心的通道 */
    struct Channel* (*target_channel_get)(uint32_t target_core_id);

    /* 向指定通道发送核间消息通知 */
    int32_t (*channel_notify)(struct Channel* channel);

    /* 从通道中获取一个空闲消息缓冲区 */
    struct Msg* (*empty_msg_get)(struct Channel* channel);
    
    /* 将一个不再使用的缓冲区归还到通道 */
    int32_t (*empty_msg_put)(struct Channel* channel,struct Msg* msg);

    /* 将一个填充过的消息缓冲区放入待处理消息队列 */
    int32_t (*msg_send)(struct Channel* channel,struct Msg* msg);
    
    /* 将一个填充过的消息缓冲区放入待处理消息队列并直接通知 */
    int32_t (*msg_send_and_notify)(struct Channel* channel,struct Msg* msg);
};

extern struct ChannelOps channel_ops;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
#endif /* __AMP_CHANNEL_H__ */
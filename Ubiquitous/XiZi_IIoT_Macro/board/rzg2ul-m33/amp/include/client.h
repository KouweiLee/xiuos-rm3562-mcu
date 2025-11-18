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

#include "shm.h"
#include "channel.h"
#include "msg.h"

#ifndef __AMP_CLIENT_H__
#define __AMP_CLIENT_H__

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

/* 核间通信交互客户端 */
struct Client
{
    uint32_t init_mark; /* 初始化标记 */
    struct Channel* remote_channel; /* 当前客户端是与那个核心的连接通道在通信 */
    uint32_t msg_cnt; /* 当前客户端申请了几个消息缓冲区 */
    uint32_t shm_cnt; /* 当前客户端申请了几块内存 */
};

struct ClientOps
{
    /* 初始化客户端 */
    int32_t (*client_init)(struct Client* raw_client,uint32_t remote_core_id);
    /* 销毁客户端 */
    int32_t (*client_destory)(struct Client* client);

    /* 获取一个空闲的消息缓冲区 */
    struct Msg* (*empty_msg_get)(struct Client* client, uint32_t remote_service_id);
    /* 归还空闲消息缓冲区：归任务管理的缓冲区才需要手动归还 */
    int32_t (*empty_msg_put)(struct Client* client, struct Msg* empty_msg);

    /* 将消息放入待处理消息队列*/
    int32_t (*msg_send)(struct Client* client, struct Msg* full_msg);
    /* 将消息放入待处理消息队列，并通知远程核心立即进行处理 */
    int32_t (*msg_send_and_notify)(struct Client* client, struct Msg* full_msg);
    /* 通知远程核心处理待处理消息队列 */
    int32_t (*msg_notify)(struct Client* client);

    /* 查看指定消息是否已经被处理过了 */
    int32_t (*msg_poll)(struct Msg* aim_msg);

#if PROTOCOL_CHOICE == PROTOCOL_AMP
    /* 从共享内存中分配内存，可以分配长期内存和短期内存 */
    void* (*shm_malloc)(struct Client* client, uint32_t size,enum MallocType type);
    /* 释放短期共享内存 */
    void (*shm_free)(struct Client* client, void* ptr);

    /* 地址转偏移量 */
    uint32_t (*shm_addr_to_offset)(void* ptr);
    /* 偏移量转地址 */
    void* (*shm_offset_to_addr)(uint32_t offset);
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
};

extern struct ClientOps client_ops;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_XIUOS */
#endif /* __AMP_CLIENT_H__ */
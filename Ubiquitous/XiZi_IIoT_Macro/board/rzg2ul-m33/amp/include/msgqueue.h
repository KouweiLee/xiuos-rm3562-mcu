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
#include "msg.h"
#include "spinlock.h"

#ifndef __AMP_MSG_QUEUE_H__
#define __AMP_MSG_QUEUE_H__

/**
 * 消息队列里面实际存储的结构，继承于Msg，可以指针互换
 * 使用当前坐标和下一个坐标将消息连成链表
*/
struct MsgEntry
{
    struct Msg msg;   /* 消息实体：必须放在第一个位置，相当于继承于 Msg */
    uint16_t cur_idx; /* 消息实体在消息队列里面的下标 */
    uint16_t nxt_idx; /* 消息实体所在消息队列的下一个消息实体的下标 */
}__attribute__((aligned(MEMORY_ALIGN_SIZE)));

/**
 * 消息队列的本体，该结构完全位于共享内存中，在系统启动时，该块内存被初始化为零
 * 在Linux中，该结构在内核启动时由该消息队列的接受者，也就是接受者进行初始化
*/
struct AmpMsgQueue
{
	volatile uint32_t working_mark; /* 标记该消息队列的接收者是否准备完毕 */
	uint16_t buf_size; /* 缓冲区队列的缓冲区数量，必须大于 0 才算初始化成功，初始状态为 0 */
	volatile uint16_t empty_h; /* 空闲缓冲区链头下标 */
	volatile uint16_t wait_h; /* 待处理缓冲区链头下标 */
	volatile uint16_t proc_ing_h; /* 服务端正在处理的缓冲区链头下标 */

	struct MsgEntry entries[0];   /* 实际存放的消息 */
}__attribute__((aligned(MEMORY_ALIGN_SIZE)));

struct MsgQueueMutex /* 消息队列互斥结构：内核与用户空间共享该数据结构 */
{
    /* 初始化标记：需要由用户层进行初始化，Linux内核无法进行初始化 */
#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
    atomic_char32_t mutex_init_mark;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
    volatile uint32_t mutex_init_mark; /* 内核占位 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */

    /**
     * 用于标记处理该消息队列的核心的工作状态
     * 在发送端通知接收端时设为：MSG_QUEUE_MARK_BUSY
     * 在发送端接收到接收端响应时设为：MSG_QUEUE_MARK_IDLE
    */
#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
    atomic_char32_t msg_queue_mark;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
    volatile uint32_t msg_queue_mark; /* 响应后修改 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */
    /**
     * 记录当前消息队列中等待处理的消息数量
     * 在放置待处理消息时增加，在核间通知后清空
    */
    volatile uint16_t msg_wait_cnt;

    /* 用户空间特有的互斥结构体 */
#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
    ByteFlag empty_lock; /* 空闲队列锁 */
    ByteFlag wait_lock;  /* 等待队列锁 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
};

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

struct MsgQueueMutexOps
{
    /* 互斥体初始化 */
    int32_t (*mutex_init)(struct MsgQueueMutex* mutex, uint16_t msg_queue_buf_size);
    /* 互斥体是否初始化 */
    int32_t (*mutex_is_init)(struct MsgQueueMutex* mutex);
};

extern struct MsgQueueMutexOps msg_queue_mutex_ops;
#endif /* #if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

struct MsgQueueOps /* 消息队列操作集合：本层为基础层，不进行参数校验，由上层进行相关参数的有效性保证 */
{
#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
    /* 初始化消息队列 */
    int32_t (*init)(struct AmpMsgQueue* msg_queue,uint32_t mem_len);
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

    /* 判断消息队列的接收核心是否已经做好准备处理数据 */
    int32_t (*is_ready)(struct AmpMsgQueue* msg_queue);

    /* 从指定队列中拿出一个消息，这个消息会从链中摘除，返回 buf_size 表示摘取失败 */
    uint16_t (*pop)(struct AmpMsgQueue* msg_queue,uint16_t* head);

    /* 向队列中放入一个消息，这个消息会被压入链头 */
    int32_t (*push)(struct AmpMsgQueue* msg_queue,uint16_t* head,uint16_t msg_index);

    /* 将一个队列的数据覆盖到另外一个队列，to_head 将被完全覆盖掉 */
    int32_t (*transfer)(struct AmpMsgQueue* msg_queue, uint16_t* from_head, uint16_t* to_head);
};

extern struct MsgQueueOps msg_queue_ops;
#endif /* __AMP_KERNEL_MSG_QUEUE_H__ */
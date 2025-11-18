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

#include "../include/config/config.h"
#include "../include/channel.h"
#include "../include/spinlock.h"

#if PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
/* 这三个函数在XiUOS的驱动里面 */
extern int32_t channels_init(void);
extern struct Channel* target_channel_get(uint32_t target_core_id);

#endif /* PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <signal.h>

struct Channel channels[] = 
{
    { /* 0: channel 1 -> Linux to XiUOS */
        .channel_info = LINUX_2_XIUOS_CHANNEL_INFO,
        .msg_queue = NULL,
        .msg_queue_mutex = NULL,
        .reg_msg = NULL,
        .msg_queue_mutex_start = NULL,
    },
    { /* 1: channel 4 -> XiUOS to Linux */
        .channel_info = XIUOS_2_LINUX_CHANNEL_INFO,
        .msg_queue = NULL,
        .msg_queue_mutex = NULL,
        .reg_msg = NULL,
        .msg_queue_mutex_start = NULL,
    },
    { /* 空 */
        .channel_info = NULL
    }
};

/* 通道初始化相关数据 */
static volatile uint32_t channel_init_mark = INIT_MARK_RAW_STATE;
static MarkFlag channel_init_lock = ATOMIC_FLAG_INIT;

/* IPI驱动、内存映射相关数据 */
static int mem_fd = -1;
static int ipi_fd = -1;

int32_t channels_init(void)
{
	mark_flag_ops.lock(&channel_init_lock);

	if (channel_init_mark == INIT_MARK_INITIALIZED)
	{
		mark_flag_ops.unlock(&channel_init_lock);
		LOGINFO("channels_init_info: channels init before\n");
		return 0;
	}

    if (channel_init_mark == INIT_MARK_DESTORYED)
    {
		mark_flag_ops.unlock(&channel_init_lock);
		INFOS("channels_init_error: channels init error\n");
		return -1;
    }
    
    mem_fd = open(MEM_DRIVER,O_RDWR | O_SYNC); /* 打开全映射驱动，用于映射消息队列 */
    if (mem_fd < 0)
    {
        INFOS("channels_init_error: open mem dev fail %d\n",mem_fd);
        goto fd_open_err;
    }
    LOGINFO("channels_init_info: open mem driver success\n");

    ipi_fd = open(IPI_DRIVER_NAME,O_RDWR | O_SYNC); /* 打开IPI核间通信硬件驱动 */
    if (ipi_fd < 0)
    {
        INFOS("channels_init_error: open ipi dev fail %d\n",ipi_fd);
        goto fd_open_err;
    }
    LOGINFO("channels_init_info: open ipi driver success\n");

	uint32_t self_core_id = core_info->id; /* 获取当前核心的ID */

	int i = 0;
    for (i = 0; channels[i].channel_info != NULL; i++)
    {
        struct Channel* channel = &channels[i];

        if (self_core_id == channel->channel_info->src_core->id) /* 该通道起点是本节点 */
        {
            /* 映射消息队列，这个消息队列由目标核心进行处理 */
	        channel->msg_queue = mmap(NULL, channel->channel_info->src_queue->len, 
                PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
                channel->channel_info->src_queue->start);

	        if (channel->msg_queue < 0) /* 映射失败 */
            {
                INFOS("channels_init_error: src msg_queue mmap = %p, mem_fd = %d, origin_start = %p, length = %u\n",
                    channel->msg_queue, mem_fd,
                    channel->channel_info->src_queue->start,
                    channel->channel_info->src_queue->len);

                goto mmap_err;
            }

            LOGINFO("channels_init_info: src msg_queue mmap = %p, mem_fd = %d, origin_start = %p, length = %u\n",
                channel->msg_queue, mem_fd,
                channel->channel_info->src_queue->start,
                channel->channel_info->src_queue->len);
            
            /* 映射消息队列互斥体 */
            channel->msg_queue_mutex_start = mmap(NULL, MEM_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, ipi_fd, 0);
            
            if (channel->msg_queue_mutex_start == NULL)
            {
                INFOS("channels_init_error: msg queue mutex mmap\n");
                goto mmap_err;
            }
            LOGINFO("channels_init_info: msg queue mutex mmap = %p\n",channel->msg_queue_mutex_start);

            channel->msg_queue_mutex = channel->msg_queue_mutex_start + (channel->channel_info->channel_num - 1) * MSG_QUEUE_MUTEX_SIZE;
            /* 初始化互斥体 */
            if (msg_queue_mutex_ops.mutex_is_init(channel->msg_queue_mutex) != 0)
            {
                LOGINFO("channels_init_info: prepare msg queue mutex init\n");
                if (msg_queue_mutex_ops.mutex_init(channel->msg_queue_mutex, channel->msg_queue->buf_size) != 0)
                {
                    INFOS("channels_init_error: msg queue mutex init\n");
                    goto mmap_err;
                }
            }
            LOGINFO("channels_init_info: msg queue mutex init\n");

            uint64_t base_addr = (channel->channel_info->reg_msg / MEM_PAGE_SIZE) * MEM_PAGE_SIZE;
            uint32_t offset = channel->channel_info->reg_msg - base_addr;

            void* addr = mmap(NULL,MEM_PAGE_SIZE,PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base_addr);
            
            if (addr == NULL)
            {
                INFOS("channels_init_error: msg reg mmap fail\n");
                goto mmap_err;
            }
            /* 映射通知寄存器 */
            channel->reg_msg = (uint32_t*)(addr + offset);
            LOGINFO("channels_init_info: reg_msg = %p\n",channel->reg_msg);

            LOGINFO("src: msg queue mark = %u\n",channel->msg_queue->working_mark);
        }
        else /* 该通道终点是本节点 */
        {   
            /* 映射消息队列，由本节点处理该消息队列，不需要消息队列互斥体 */
	        channel->msg_queue = mmap(NULL, channel->channel_info->dst_queue->len,
                PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
                channel->channel_info->dst_queue->start);

            if (channel->msg_queue < 0) /* 映射失败 */
            {
                INFOS("channels_init_error: dst msg_queue mmap = %p, mem_fd = %d, origin_start = %p, length = %u\n",
                channel->msg_queue, mem_fd,
                channel->channel_info->dst_queue->start,
                channel->channel_info->dst_queue->len);

                goto mmap_err;
            }

            LOGINFO("channels_init_success: dst msg_queue mmap = %p, mem_fd = %d, origin_start = %p, length = %u\n",
                channel->msg_queue, mem_fd,
                channel->channel_info->dst_queue->start,
                channel->channel_info->dst_queue->len);

            LOGINFO("dst: msg queue mark = %u\n",channel->msg_queue->working_mark);
        }
    }

    channel_init_mark = INIT_MARK_INITIALIZED;
	mark_flag_ops.unlock(&channel_init_lock);
    
	LOGINFO("channels_init_success: init channel success\n");

	return 0;

mmap_err:

    for (i = 0; channels[i].channel_info != NULL; i++)
    {
        struct Channel* channel = &channels[i];
		if (self_core_id == channel->channel_info->src_core->id)
		{ /* 该通道起点是本节点 */
            if (channel->msg_queue != NULL)
            {
                munmap(channel->msg_queue,channel->channel_info->src_queue->len);
                channel->msg_queue = NULL;
            }
            if (channel->msg_queue_mutex_start != NULL)
            {
                munmap(channel->msg_queue_mutex_start,MEM_PAGE_SIZE);
                channel->msg_queue_mutex_start = NULL;
                channel->msg_queue_mutex = NULL;
            }
            if (channel->reg_msg != NULL)
            {
                uint64_t addr = (uint64_t)channel->reg_msg;
                munmap((void*)((addr / MEM_PAGE_SIZE) * MEM_PAGE_SIZE),sizeof(uint32_t));
                channel->reg_msg = NULL;
            }
		}
		else
		{ /* 该通道终点是本节点 */
            if (channel->msg_queue != NULL)
            {
                munmap(channel->msg_queue,channel->channel_info->dst_queue->len);
                channel->msg_queue->working_mark = INIT_MARK_DESTORYED;
                channel->msg_queue = NULL;
            }
		}
    }
fd_open_err:
    if (mem_fd > 0)
    {
        close(mem_fd);
        mem_fd = -1;
    }
    if (ipi_fd > 0)
    {
        close(ipi_fd);
        ipi_fd = -1;
    }

    channel_init_mark = INIT_MARK_DESTORYED;
    mark_flag_ops.unlock(&channel_init_lock);
    
    return -1;
}

struct Channel* target_channel_get(uint32_t target_core_id)
{
    uint32_t self_core_id = core_info->id;

    int i = 0;
    for (i = 0; channels[i].channel_info != NULL; i++)
    {
        struct Channel* channel = &channels[i];
        if (self_core_id == channel->channel_info->src_core->id && target_core_id == channel->channel_info->dst_core->id)
        {
            LOGINFO("channel_get_send_channel_success: target_core_id = %u\n",target_core_id);
            return channel;
        }
    }
    INFOS("channel_get_send_channel_error: target_core_id = %u\n",target_core_id);

    return NULL;
}
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

static int32_t channel_is_ready(struct Channel* channel)
{
    LOGINFO("channel_is_ready_info: enter mark = %u\n", channel->msg_queue->working_mark);

    if (msg_queue_ops.is_ready(channel->msg_queue) == 0)
    {
        LOGINFO("channel_is_ready_info: msg queue is ready\n");

        if (msg_queue_mutex_ops.mutex_is_init(channel->msg_queue_mutex) == 0)
        {
            LOGINFO("channel_is_ready_info: msg queue mutex is ready\n");
            return 0;
        }
    }
    
    return -1;
}

static int32_t channel_notify(struct Channel* channel)
{
    if (msg_queue_ops.is_ready(channel->msg_queue) != 0)
    {
		INFOS("channel_notify_warn: msg queue is not ready\n");
		return -1;
    }

	/* 锁定待处理消息队列，防止其他线程继续往里面添加数据 */
	byte_flag_ops.lock(&channel->msg_queue_mutex->wait_lock);

	if (channel->msg_queue_mutex->msg_wait_cnt == 0) /* 没有需要处理的消息 */
	{
		byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

		LOGINFO("channel_notify_info: have no wait msg\n");
		return 0;
	}

	while (channel->msg_queue_mutex->msg_queue_mark != MSG_QUEUE_MARK_IDLE) /* 等待远程核心处理完上一批消息 */
	{ 
		;
	}
	LOGINFO("channel_notify_info: remote idel, prepare to notify\n");

	/* 将待处理队列放入处理队列 */
	if (msg_queue_ops.transfer(channel->msg_queue,&channel->msg_queue->wait_h,&channel->msg_queue->proc_ing_h) != 0)
	{
		byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

		INFOS("channel_notify_error: add wait msg to process queue\n");
		return -1;
	}

	/* 清空等待消息队列，设置工作标志，防止重复通知 */
	channel->msg_queue_mutex->msg_wait_cnt = 0;
	channel->msg_queue_mutex->msg_queue_mark = MSG_QUEUE_MARK_BUSY;

	/* 释放等待队列，其他线程可以继续往里面添加消息 */
	byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

	/* 向可以触发核间中断的寄存器中写入数据以触发核间中断 */
	*(channel->reg_msg) = 1U;

	return 0;
}

static struct Msg* channel_empty_msg_get(struct Channel* channel) /* 从通道中获取一个空的消息缓冲区 */
{
    struct Msg* empty_msg = NULL;

    byte_flag_ops.lock(&channel->msg_queue_mutex->empty_lock); /* 对空闲队列加锁 */
    uint16_t empty_index = msg_queue_ops.pop(channel->msg_queue,&channel->msg_queue->empty_h);

    if (empty_index < channel->msg_queue->buf_size) /* 还有空闲消息 */
    {   
        empty_msg = (struct Msg*)(&channel->msg_queue->entries[empty_index]);
        msg_ops.msg_reset(empty_msg); /* 重置该消息 */
    }

    byte_flag_ops.unlock(&channel->msg_queue_mutex->empty_lock);

    LOGINFO("empty_msg_get_info: result = %p\n",empty_msg);

    return empty_msg;
}

static int32_t channel_empty_msg_put(struct Channel* channel,struct Msg* msg) /* 将一个不再使用的缓冲区归还给通道 */
{
    struct MsgEntry* entry = (struct MsgEntry*)msg;
    int ret = -1;

    msg_ops.msg_reset(msg); /* 归还前重置该消息 */

    byte_flag_ops.lock(&channel->msg_queue_mutex->empty_lock); /* 对空闲队列加锁 */

    ret = msg_queue_ops.push(channel->msg_queue,&channel->msg_queue->empty_h,entry->cur_idx);

    byte_flag_ops.unlock(&channel->msg_queue_mutex->empty_lock);

    return ret;
}

static int32_t channel_msg_send(struct Channel* channel,struct Msg* msg)
{
    struct MsgEntry* entry = (struct MsgEntry*)msg;
    int ret = -1;

    byte_flag_ops.lock(&channel->msg_queue_mutex->wait_lock); /* 对等待队列加锁 */

    ret = msg_queue_ops.push(channel->msg_queue,&channel->msg_queue->wait_h,entry->cur_idx);

    if (ret == 0)
    {
        channel->msg_queue_mutex->msg_wait_cnt ++;
    }
    else
    {
        LOGINFO("channel_send_msg_buffer_error: add msg fail, idx = %u\n",entry->cur_idx);
    }
    
    byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

    return ret;
}

static int32_t channel_msg_send_and_notify(struct Channel* channel,struct Msg* msg)
{
    struct MsgEntry* entry = (struct MsgEntry*)msg;
    int ret = -1;

    byte_flag_ops.lock(&channel->msg_queue_mutex->wait_lock); /* 对等待队列加锁 */

    ret = msg_queue_ops.push(channel->msg_queue,&channel->msg_queue->wait_h,entry->cur_idx);

    if (ret == 0)
    {
        channel->msg_queue_mutex->msg_wait_cnt ++;
    }
    else
    {
        LOGINFO("channel_send_msg_buffer_error: add msg fail, idx = %u\n",entry->cur_idx);
    }
    
    if (channel->msg_queue_mutex->msg_wait_cnt == 0) /* 没有需要处理的消息 */
	{
		byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

		LOGINFO("msg_send_and_notify: have no wait msg\n");
		return 0;
	}

	while (channel->msg_queue_mutex->msg_queue_mark != MSG_QUEUE_MARK_IDLE) /* 等待远程核心处理完上一批消息 */
	{ 
		;
	}
	LOGINFO("msg_send_and_notify: remote idel, prepare to notify\n");

	/* 将待处理队列放入处理队列 */
	if (msg_queue_ops.transfer(channel->msg_queue,&channel->msg_queue->wait_h,&channel->msg_queue->proc_ing_h) != 0)
	{
		byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);

		INFOS("msg_send_and_notify: add wait msg to process queue\n");
		return -1;
	}
	/* 清空等待消息队列，设置工作标志，防止重复通知 */
	channel->msg_queue_mutex->msg_wait_cnt = 0;
	channel->msg_queue_mutex->msg_queue_mark = MSG_QUEUE_MARK_BUSY;

	/* 释放等待队列，其他线程可以继续往里面添加消息 */
	byte_flag_ops.unlock(&channel->msg_queue_mutex->wait_lock);
    
	/* 向可以触发核间中断的寄存器中写入数据以触发核间中断 */
	*(channel->reg_msg) = 1U;

    return 0;
}

struct ChannelOps channel_ops = 
{
    .channels_init = channels_init,
    .channel_is_ready = channel_is_ready,
    .target_channel_get = target_channel_get,
    .channel_notify = channel_notify,
    .empty_msg_get = channel_empty_msg_get,
    .empty_msg_put = channel_empty_msg_put,
    .msg_send = channel_msg_send,
    .msg_send_and_notify = channel_msg_send_and_notify
};
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

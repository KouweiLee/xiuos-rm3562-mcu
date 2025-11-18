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

#include "../../../include/config/config.h"
#include "../../../include/msgqueue.h"
#include "../../../include/channel.h"
#include "../../../include/spinlock.h"
#include "../../../include/service.h"

/* XiUOS的MHU驱动 */
#if PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

#include <bsp_api.h>
#include <fsp_features.h>
#include "vector_data.h"

struct RegGroup   /* 寄存器组 */
{
	uint32_t STS; /* 中断状态寄存器，只读 */
	uint32_t SET; /* 中断触发寄存器，只写 1 */
	uint32_t CLR; /* 中断清除寄存器，只写1 */
};

struct MhuChannel
{
	struct Channel channel;	  /* 通道信息：必须放在第一个成员位置 */
	struct RegGroup* msg_reg; /* 消息发送寄存器组 */
	struct RegGroup* rsp_reg; /* 消息响应寄存器组 */
};

/* 寄存器基地址 */
#define REG_BASE (0x40400000U)
/* 消息队列互斥锁 */
static struct MsgQueueMutex xiuos_to_linux_msg_queue_mutex = { 0 };

struct MhuChannel mhu_channels[] = 
{
	{ /* 0: channel 1 -> Linux to XiUOS */
		.channel = 
		{
			.channel_info = LINUX_2_XIUOS_CHANNEL_INFO,
			.msg_queue = NULL,
			.msg_queue_mutex = NULL,
			.reg_msg = NULL
		},
		.msg_reg = (struct RegGroup*) (REG_BASE + 0x20U),
		.rsp_reg = (struct RegGroup*) (REG_BASE + 0x30U)
	},
	{ /* 1: channel 4 -> XiUOS to Linux */
		.channel = 
		{
			.channel_info = XIUOS_2_LINUX_CHANNEL_INFO,
			.msg_queue = NULL,
			.msg_queue_mutex = &xiuos_to_linux_msg_queue_mutex,
			.reg_msg = NULL,
		},
		.msg_reg = (struct RegGroup*) (REG_BASE + 0x80U),
		.rsp_reg = (struct RegGroup*) (REG_BASE + 0x90U)
	},
	{ /* 2: 空*/
		.channel = 
		{
			.channel_info = NULL
		}
	}
};

#define MHU_LINUX_2_XIUOS_CHANNEL (&mhu_channels[0])
#define MHU_XIUOS_2_LINUX_CHANNEL (&mhu_channels[1])

/* 通道初始化只能由一个任务完成 */
static volatile uint32_t channels_init_mark = INIT_MARK_RAW_STATE;
static MarkFlag channels_init_lock = ATOMIC_FLAG_INIT;

int32_t channels_init(void)
{
	mark_flag_ops.lock(&channels_init_lock);

	if (channels_init_mark == INIT_MARK_INITIALIZED)
	{
		mark_flag_ops.unlock(&channels_init_lock);
		LOGINFO("xiuos_channels_init_info: init before\n");
		return 0;
	}

	if (channels_init_mark == INIT_MARK_DESTORYED)
	{
		mark_flag_ops.unlock(&channels_init_lock);
		INFOS("xiuos_channels_init_error: init fail before\n");
		return -1;
	}

	uint32_t self_core_id = core_info->id;

	int i = 0;
	for (i = 0; mhu_channels[i].channel.channel_info != NULL; i++)
	{
		struct MhuChannel* channel = &mhu_channels[i];

		/* 该通道起点是本节点 */
		if (self_core_id == channel->channel.channel_info->src_core->id)
		{
			/* 映射出该通道的消息队列 */
			channel->channel.msg_queue = (struct AmpMsgQueue *)(channel->channel.channel_info->src_queue->start);
			/* 初始化该消息队列：在目标核心没有准备好之前，可以缓存，不能直接通知 */
			if (msg_queue_ops.init(channel->channel.msg_queue, channel->channel.channel_info->src_queue->len) != 0)
			{
				channels_init_mark = INIT_MARK_DESTORYED;
				mark_flag_ops.unlock(&channels_init_lock);

				INFOS("xiuos_channels_init_error: init msg_queue fail\n");
				return -1;
			}
			/* 初始化消息队列的互斥结构 */
			if (msg_queue_mutex_ops.mutex_init(channel->channel.msg_queue_mutex, channel->channel.msg_queue->buf_size) != 0)
			{
				channels_init_mark = INIT_MARK_DESTORYED;
				mark_flag_ops.unlock(&channels_init_lock);

				INFOS("xiuos_channels_init_error: init msg_queue_mutex fail\n");
				return -1;
			}
			/* 映射可以直接进行通知的寄存器地址 */
			channel->channel.reg_msg = (uint32_t*)channel->channel.channel_info->reg_msg;

			/* 开启通道中断 */
			R_BSP_IrqEnable(channel->channel.channel_info->irq_rsp);

			LOGINFO("src: msg queue mark = %u\n",channel->channel.msg_queue->working_mark);
		}
		else /* 该通道终点是本节点 */
		{
			/* 映射需要本通道处理的消息队列 */
			channel->channel.msg_queue = (struct AmpMsgQueue *)(channel->channel.channel_info->dst_queue->start);
			/* 告知该消息队列的发送方该消息队列可以进行通知了 */
			channel->channel.msg_queue->working_mark = INIT_MARK_INITIALIZED;

			/* 开启通道中断 */
			R_BSP_IrqEnable(channel->channel.channel_info->irq_msg);
			LOGINFO("dst: msg queue mark = %u\n",channel->channel.msg_queue->working_mark);
		}

		/* 开启通道时钟 */
		R_BSP_MODULE_START(FSP_IP_MHU, channel->channel.channel_info->channel_num);
	}
	channels_init_mark = INIT_MARK_INITIALIZED;
	mark_flag_ops.unlock(&channels_init_lock);

	LOGINFO("xiuos_channels_init_success: init channel success\n");

	return 0;
}

struct Channel* target_channel_get(uint32_t target_core_id)
{
    int i = 0;

    uint32_t self_core_id = core_info->id;

    for (i = 0; mhu_channels[i].channel.channel_info != NULL; i++)
    {
        struct MhuChannel* channel = &mhu_channels[i];
        if (self_core_id == channel->channel.channel_info->src_core->id && target_core_id == channel->channel.channel_info->dst_core->id)
        {
			LOGINFO("get_send_channel_success: self = %u, target = %u\n",self_core_id,target_core_id);
            return &mhu_channels[i].channel;
        }
    }
    
	INFOS("get_send_channel_error: self = %u, target = %u\n",self_core_id,target_core_id);
    return NULL;
}

/* 通道接收到发送的消息时调用 */
int channel_msg_recv_interrupt(struct Channel* channel)
{   
	LOGINFO("channel_msg_recv_interrupt_info: enter\n");

    struct AmpMsgQueue* msg_queue = channel->msg_queue;
    struct Msg* msg = NULL;
	struct AmpService* aim_service = NULL;

	/* 获取消息队列中待处理的第一个消息：不能破坏链接结构 */
    uint16_t head_index = msg_queue_ops.pop(msg_queue,&msg_queue->proc_ing_h);
	int service_result = -1;
	
    while (1)
    {
        if (head_index != msg_queue->buf_size)
        {
			msg = &msg_queue->entries[head_index].msg; /* 需要进行处理的消息 */
			aim_service = amp_service_ops.get_by_id(msg->service_id); /* 获取需要调用的服务 */

			if (aim_service == NULL)
			{
				LOGINFO("channel_msg_recv_interrupt_warn: service not hit, msg_index = %u, service_id = %u\n",head_index,msg->service_id);

				/* 该消息已经被处理，但是消息请求的服务不存在 */
				msg->flag.deal_state = MSG_DEAL_STATE_YES;
				msg->flag.service_result = MSG_SERVICE_RET_NOT_EXITS;
			}
			else
			{
				LOGINFO("channel_msg_recv_interrupt_info: service hit, msg_index = %u, service_id = %u\n",head_index,msg->service_id);

				service_result = aim_service->service(msg); /* 将消息转交给服务进行处理 */

				if (service_result == 0) /* 设置服务响应的结果 */
				{
					msg->flag.service_result = MSG_SERVICE_RET_SUCCESS;
				}
				else
				{
					msg->flag.service_result = MSG_SERVICE_RET_FAIL;
				}
				
				msg->flag.deal_state = MSG_DEAL_STATE_YES; /* 标记该条信息已经被处理，发送核心可以直接开始下一步的操作 */

				LOGINFO("channel_msg_recv_interrupt_info: service processed, msg_index = %u, service_id = %u, result = %s\n",
					head_index, msg->service_id,
					service_result == 0 ? "success" : "fail");
			}
        }
        else
        {
			LOGINFO("channel_msg_recv_interrupt_info: no more msg exist, head_index = %u\n",head_index);
            break;
        }

        head_index = msg_queue_ops.pop(msg_queue,&msg_queue->proc_ing_h);
    }

    /* 响应对方已经处理完毕 */
	((struct MhuChannel*)channel)->rsp_reg->SET = 1U;
    
    LOGINFO("channel_msg_recv_interrupt_info: exit\n");

    return 0;
}

/* 通道接收到响应的消息时调用 */
int channel_msg_rsp_interrupt(struct Channel* channel)
{
    LOGINFO("channel_msg_rsp_interrupt_info: enter\n");

	/* 可以继续发送消息通知了 */
	channel->msg_queue_mutex->msg_queue_mark = MSG_QUEUE_MARK_IDLE;

    LOGINFO("channel_msg_rsp_interrupt_info: exit\n");

    return 0;
}

void msg_ch1_ns(void)
{	
	LOGINFO("msg_ch1_ns_info: xiuos recv linux msg interrupt enter\n");

	/* 通道1消息接收中断：Linux给XiUOS发消息时触发 */
	MHU_LINUX_2_XIUOS_CHANNEL->msg_reg->CLR = 1U; /* 清除中断 */
	channel_msg_recv_interrupt(&MHU_LINUX_2_XIUOS_CHANNEL->channel);  /* 处理接收到的消息 */

	LOGINFO("msg_ch1_ns_info: xiuos recv linux msg interrupt over\n");
}

void rsp_ch4_ns(void)
{
	LOGINFO("rsp_ch4_ns_info: xiuos recv linux rsp interrupt enter\n");

	/* 通道4响应接收中断：Linux给XiUOS消息响应时触发*/
	MHU_XIUOS_2_LINUX_CHANNEL->rsp_reg->CLR = 1U; /* 清除中断 */
	channel_msg_rsp_interrupt(&MHU_XIUOS_2_LINUX_CHANNEL->channel);	  /* 处理通道响应信息*/
	
	LOGINFO("rsp_ch4_ns_info: xiuos recv linux rsp interrupt over\n");

}
#endif /* PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS*/
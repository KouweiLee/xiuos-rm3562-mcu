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
#include "../../../include/service.h"
#include "../../../include/service/service_linux.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL /* Linux内核态代码 */

#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/of_address.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/types.h> /* 类型\输出 */
#include <linux/kernel.h>
#include <linux/module.h> /* 模块相关 */
#include <linux/init.h>
#include <linux/slab.h> /* kmalloc */
#include <linux/mm.h> /* remap_pfn_range */
#include <linux/miscdevice.h> /* 混杂驱动 */
#include <linux/of.h> /* 设备树解析函数 */
#include <linux/of_irq.h> 
#include <linux/interrupt.h> /* 中断相关函数 */
#include <linux/irq.h>
#include <asm/io.h> /* virt_to_phy */
#include <linux/io.h>
#include <asm/uaccess.h>

#define REG_BASE (0x10400000U) /* 寄存器基地址 */

struct RegGroup   /* 寄存器组 */
{
	uint32_t STS; /* 中断状态寄存器，只读 */
	uint32_t SET; /* 中断触发寄存器，只写 1 */
	uint32_t CLR; /* 中断清除寄存器，只写1 */
};

struct MhuChannel /* 内核空间的通道信息抽象 */
{
    struct ChannelInfo* channel_info;  /* 通道配置信息 */
    struct AmpMsgQueue* msg_queue;     /* 发往远程核心的消息队列 */
	struct MsgQueueMutex* msg_queue_mutex; /* 消息队列互斥体 */

	uint64_t msg_reg_start;   /* 消息发送寄存器组起始地址 */
	struct RegGroup *msg_reg; /* 消息发送寄存器组 */
	uint64_t rsp_reg_start;   /* 消息响应寄存器组起始地址 */
	struct RegGroup *rsp_reg; /* 消息响应寄存器组 */

	int32_t irq_num; /* 通道处理中断的中断号 */
	irqreturn_t (*interrupt_handler)(int irq_num, void *mhu_channel); /* 通道中断处理函数 */
};

int channel_msg_recv_interrupt(struct MhuChannel* mhu_channel); /* 通道接受消息的中断处理函数 */
int channel_msg_rsp_interrupt(struct MhuChannel* mhu_channel);  /* 通道消息响应的中断处理函数 */
static irqreturn_t msg_ch4_ns(int irq_num, void* mhu_channel);  /* 通道4数据接收中断 */
static irqreturn_t rsp_ch1_ns(int irq_num, void* mhu_channel);  /* 通道1数据响应接收中断 */

static int mhu_open(struct inode *inode, struct file *filp); /* 设备打开 */
static int mhu_release(struct inode *inode, struct file *filp); /* 设备释放 */
static long mhu_ioctl(struct file *file, unsigned int cmd, unsigned long arg); /* 设备控制 */
static int mhu_mmap(struct file *file, struct vm_area_struct *vma); /* 内存映射 */

static void* msg_mutex_area; /* 映射到用户空间的内存,用于存放各个通道的消息队列互斥体，大小 @MEM_PAGE_ZISE */

static struct MhuChannel mhu_channels[] = 
{
	{ /* 0: channel 1 -> Linux to XiUOS */
		.channel_info = LINUX_2_XIUOS_CHANNEL_INFO,
		.msg_queue = NULL,
		.msg_queue_mutex = NULL,
		.msg_reg_start = REG_BASE + 0x20U,
		.msg_reg = NULL,
		.rsp_reg_start = REG_BASE + 0x30U,
		.rsp_reg = NULL,
		.irq_num = 0,
		.interrupt_handler = rsp_ch1_ns
	},
	{ /* 1: channel 4 -> XiUOS to Linux */
		.channel_info = XIUOS_2_LINUX_CHANNEL_INFO,
		.msg_queue = NULL,
		.msg_queue_mutex = NULL,
		.msg_reg_start = REG_BASE + 0x80U,
		.msg_reg = NULL,
		.rsp_reg_start = REG_BASE + 0x90U,
		.rsp_reg = NULL,
		.irq_num = 0,
		.interrupt_handler = msg_ch4_ns
	},
	{ /* 2: 空*/
		.channel_info = NULL
	}
};

static struct file_operations arm_mhu_ops = 
{
	.owner = THIS_MODULE,
	.open = mhu_open,
	.release = mhu_release,
	.unlocked_ioctl = mhu_ioctl,
	.mmap = mhu_mmap
};

static struct miscdevice arm_mhu_dev = 
{
	.minor = MISC_DYNAMIC_MINOR, /* 次设备号自动生成 */
	.name = IPI_DEV_NAME,        /* 设备名称 */
	.fops = &arm_mhu_ops         /* 设备操作合集 */
};

/* 通道接收到发送的消息时调用 */
int channel_msg_recv_interrupt(struct MhuChannel* channel)
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
				LOGINFO("channel_msg_recv_interrupt_info: service processed, msg_index = %u, service_id = %u, result = %s\n",head_index, msg->service_id,service_result == 0 ? "success" : "fail");
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
int channel_msg_rsp_interrupt(struct MhuChannel* channel)
{
    LOGINFO("channel_msg_rsp_interrupt_info: enter\n");

	/* 可以继续发送数据了 */
	channel->msg_queue_mutex->msg_queue_mark = MSG_QUEUE_MARK_IDLE;

    LOGINFO("channel_msg_rsp_interrupt_info: exit\n");

    return 0;
}

/* XiUOS发送给Linux的消息由该中断负责处理 */
static irqreturn_t msg_ch4_ns(int irq_num, void* chan)
{
	LOGINFO("msg_ch4_ns_info: enter\n");

	struct MhuChannel* mhu_channel = (struct MhuChannel*) chan;
	mhu_channel->msg_reg->CLR = 1U; /* 清除中断标志 */
	channel_msg_recv_interrupt(mhu_channel); /* 处理消息接受处理 */
	
	LOGINFO("msg_ch4_ns_info: exit\n");

	return IRQ_RETVAL(IRQ_HANDLED);
}

/* Linux发送给XiUOS的数据响应消息由该中断处理 */
static irqreturn_t rsp_ch1_ns(int irq_num, void* chan)
{
	LOGINFO("rsp_ch1_ns: enter\n");

	struct MhuChannel* mhu_channel = (struct MhuChannel*) chan;
	mhu_channel->rsp_reg->CLR = 1U; /* 清除中断标志 */
	channel_msg_rsp_interrupt(mhu_channel); /* 处理消息接受处理 */

	LOGINFO("rsp_ch1_ns: exit\n");

	return IRQ_RETVAL(IRQ_HANDLED);
}

/* 打开 MHU 设备 */
static int mhu_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &arm_mhu_dev;

	LOGINFO("mhu_open_info: mhu open\n");

	return 0;
}

/* 关闭 MHU 设备 */
static int mhu_release(struct inode *inode, struct file *filp)
{
	LOGINFO("mhu_release_info: mhu release\n");

	return 0;
} 

/* MHU 设备指令 */
static long mhu_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	LOGINFO("mhu_ioctl_info: cmd = %u, arg = %u\n",cmd,arg);

	switch (cmd)
	{
		case IOCTL_CMD_NOTIFY: /* 通知具体消息通道 */
		{
			int i = 0;
			uint32_t channel_num = (uint32_t) arg;

			for (i = 0; mhu_channels[i].channel_info != NULL; i++) 
			{
				struct MhuChannel *mhu_channel = &mhu_channels[i];

				if (mhu_channel->channel_info->channel_num == channel_num)
				{
					mhu_channel->msg_reg->SET = 1U; /* 触发核间中断 */

					LOGINFO("mhu_ioctl_info: channel notify, num = %u\n",channel_num);

					return 0;
				}
			}
			INFOS("mhu_ioctl_warn: channel not exist, num = %u\n",channel_num);

			return -ENOTTY;
		}
		default:
		{
			INFOS("mhu_ioctl_warn: unknown cmd = %u, arg = %u\n",cmd,arg);

			return -ENOTTY;
		}
	}

	return 0;
}

static int mhu_mmap(struct file* file, struct vm_area_struct* vma)
{
	LOGINFO("mhu_mmap_info: enter\n");

	if (remap_pfn_range(vma, /* 虚拟内存区域 */
						vma->vm_start, /* 虚拟空间起始地址 */
						virt_to_phys(msg_mutex_area) >> PAGE_SHIFT, /* 于物理内存对应的页帧号 */
						vma->vm_end - vma->vm_start, /* 映射区域大小 */
						vma->vm_page_prot /* 页面保护属性 */))
	{
		return -EAGAIN;
	}
	
	LOGINFO("mhu_mmap_info: exit\n");

	return 0;
}

static int arm_mhu_init(void)
{
	int result_ret,i = 0;

	struct device_node* dev_node = of_find_node_by_name(NULL,"mhu"); /* 从设备树中找到MHU节点 */
	if (dev_node == NULL)
	{
		INFOS("arm_mhu_init_error: find node(mhu) fail\n");
		return -EINVAL;
	}

	const char* dev_status = NULL;
	result_ret = of_property_read_string(dev_node,"status",&dev_status); /* 读取设备的状态 */
	if (result_ret < 0 || strcmp(dev_status,"okay") != 0)
	{
		INFOS("arm_mhu_init_error: get status or status is not okay\n");
		return -EINVAL;
	}
	
	const char* dev_compat = NULL;
	result_ret = of_property_read_string(dev_node,"compatible",&dev_compat); /* 读取设备匹配属性并匹配 */
	if (result_ret < 0 || strcmp(dev_compat,"arm,rzg2ul-mhu") != 0)
	{
		INFOS("arm_mhu_init_error: get compatible or compatible match fail\n");
		return -EINVAL;
	}

	/* 开辟消息队列的互斥公共内存 */
	msg_mutex_area = kzalloc(MEM_PAGE_SIZE,GFP_KERNEL);
	if (msg_mutex_area == NULL)
	{
		INFOS("arm_mhu_init_error: malloc msg_mutex_area fail\n");
		return -EINVAL;
	}
	/* 设置内存页常驻 */
	SetPageReserved(msg_mutex_area);
	*((uint32_t*)msg_mutex_area) = 0U; /* 第一个字节非0，需要清零 */

	for (i = 0; mhu_channels[i].channel_info != NULL; i++) /* 初始化设备的中断 */
	{
		struct MhuChannel* mhu_channel = &mhu_channels[i];
		/* 物理地址映射 */
		mhu_channel->msg_reg = ioremap(mhu_channel->msg_reg_start,sizeof(struct RegGroup));
		mhu_channel->rsp_reg = ioremap(mhu_channel->rsp_reg_start,sizeof(struct RegGroup));

		uint32_t self_core_id = core_info->id;

		/* 注册相关中断 */
		uint32_t irq_index = -1;

		if (self_core_id == mhu_channel->channel_info->src_core->id) /* 如果当前核心是通道的发送方 */
		{
			irq_index = mhu_channel->channel_info->irq_rsp;
		}
		else
		{
			irq_index = mhu_channel->channel_info->irq_msg;
		}

		int irq_num = irq_of_parse_and_map(dev_node, irq_index); /* 读取通道的中断号 */
		if (irq_num <= 0)
		{
			INFOS("mhu_init_error: get irq_num fail = %d\n",irq_num);
			return -EINVAL;
		}

		mhu_channel->irq_num = irq_num; /* 记录下当前通道的中断号 */

		unsigned long irq_flags = irq_get_trigger_type(irq_num); /* 读取设备的中断触发类型 */
		
		const char* irq_name = NULL;
		of_property_read_string_index(dev_node,"interrupt-names",irq_index,&irq_name); /* 读取中断的名称 */
		
		result_ret = request_irq(irq_num,mhu_channel->interrupt_handler,irq_flags,irq_name,mhu_channel); /* 申请中断 */

		if (result_ret != 0)
		{
			INFOS("mhu_init_error: request irq_num = %d, irq_name = %s fail\n",irq_num,irq_name);

			return result_ret;
		}
		LOGINFO("mhu_init_info: request irq_num = %d, irq_name = %s success\n",irq_num,irq_name);

		struct AddrInfo* msg_queue_addr_info = NULL;
		if (self_core_id == mhu_channel->channel_info->src_core->id) /* 如果当前核心是通道的发送方 */
		{
			msg_queue_addr_info = mhu_channel->channel_info->src_queue;
			mhu_channel->msg_queue = ioremap(msg_queue_addr_info->start, msg_queue_addr_info->len);
			LOGINFO("arm_mhu_init_info: before msg_working = %u\n",mhu_channel->msg_queue->working_mark);
			/* 初始化消息队列 */
			if (0 != msg_queue_ops.init(mhu_channel->msg_queue,msg_queue_addr_info->len))
			{
				INFOS("arm_mhu_init_error: msg_queue_init fail\n");
				return -EINVAL;
			}
			LOGINFO("arm_mhu_init_info: after msg_working = %u\n",mhu_channel->msg_queue->working_mark);
			/* 计算互斥体位置：这里没有初始化，需要用户空间进行初始化 */
			mhu_channel->msg_queue_mutex = msg_mutex_area + (mhu_channel->channel_info->channel_num - 1) * MSG_QUEUE_MUTEX_SIZE;
		}
		else /* 当前核心是通道的接收方 */
		{
			msg_queue_addr_info = mhu_channel->channel_info->dst_queue;
			mhu_channel->msg_queue = ioremap(msg_queue_addr_info->start, msg_queue_addr_info->len);
			/* 接收方没有互斥体结构: 告知发送方可以发送数据了 */
			mhu_channel->msg_queue->working_mark = INIT_MARK_INITIALIZED;
		}
	}

	misc_register(&arm_mhu_dev); /* 注册杂项驱动 */

	LOGINFO("mhu_init_success: mhu init success!\n");

	return 0;
}

static void arm_mhu_exit(void)
{
	int i;
	for (i = 0; mhu_channels[i].channel_info != NULL; i++) /* 取消映射物理地址信息 */
	{
		struct MhuChannel* mhu_channel = &mhu_channels[i];

		iounmap(mhu_channel->msg_reg);
		iounmap(mhu_channel->rsp_reg);
		iounmap(mhu_channel->msg_queue);

		free_irq(mhu_channel->irq_num,mhu_channel);
	}

	ClearPageReserved(msg_mutex_area);
	kfree(msg_mutex_area);

	misc_deregister(&arm_mhu_dev); /* 注销杂项设备 */

	LOGINFO("arm_mhu_exit_success: mhu exit success!\n");
}

module_init(arm_mhu_init);
module_exit(arm_mhu_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("anguoyoula");

#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */
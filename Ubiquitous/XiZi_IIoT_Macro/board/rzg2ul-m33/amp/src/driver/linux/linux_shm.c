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

#include "../../../include/amp.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL && PROTOCOL_CHOICE == PROTOCOL_AMP  /* Linux内核态代码：没有开启AMP协议时，该模块也会被加载到内核 */
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

static void* shm_block_info = NULL; /* 映射到用户空间的内存，大小：MEM_PAGE_SIZE */
static void* shm_total_start = NULL; /* 共享内存总的起始地址信息 */

void* kshm_offset_to_addr(uint32_t offset) /* 给内核中的服务从消息中提取出地址信息 */
{
	LOGINFO("kshm_offset_to_addr: offset = %u\n",offset);

	return shm_total_start + offset;
}

EXPORT_SYMBOL(shm_total_start);
EXPORT_SYMBOL(kshm_offset_to_addr);

static int shm_open(struct inode *inode, struct file *filp); /* 设备打开 */
static int shm_release(struct inode *inode, struct file *filp); /* 设备释放 */
static int shm_mmap(struct file *file, struct vm_area_struct *vma); /* 内存映射 */

static struct file_operations arm_shm_ops = 
{
	.owner = THIS_MODULE,
	.open = shm_open,
	.release = shm_release,
	.mmap = shm_mmap
};

static struct miscdevice arm_shm_dev = 
{
	.minor = MISC_DYNAMIC_MINOR, /* 次设备号自动生成 */
	.name = SHM_DEV_NAME,        /* 设备名称 */
	.fops = &arm_shm_ops         /* 设备操作合集 */
};

static int shm_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &arm_shm_dev;

	LOGINFO("shm_open_info: shm open\n");
	return 0;
}

static int shm_release(struct inode *inode, struct file *filp)
{
	LOGINFO("shm_release_info: shm release\n");

	return 0;
} 

static int shm_mmap(struct file* file, struct vm_area_struct* vma)
{
	LOGINFO("shm_mmap_info: enter\n");

	if (remap_pfn_range(vma, /* 虚拟内存区域 */
						vma->vm_start, /* 虚拟空间起始地址 */
						virt_to_phys(shm_block_info) >> PAGE_SHIFT, /* 于物理内存对应的页帧号 */
						vma->vm_end - vma->vm_start, /* 映射区域大小 */
						vma->vm_page_prot /* 页面保护属性 */))
	{
		return -EAGAIN;
	}
	
	LOGINFO("shm_mmap_info: exit\n");

	return 0;
}

static int arm_shm_init(void)
{
	/* 申请的同时直接清零 */
	shm_block_info = kzalloc(MEM_PAGE_SIZE,GFP_KERNEL);

	if (shm_block_info == NULL)
	{
		INFOS("arm_shm_init_error: malloc shm_block_info fail\n");
		return -EINVAL;
	}
	LOGINFO("arm_shm_init_info: shm_block_info = %p [%u]\n",shm_block_info,(uint64_t)shm_block_info);

	/* 管理信息常驻内存 */
	SetPageReserved(virt_to_page(shm_block_info));

	/* 映射全局共享内存 */
	struct ShmCfg *shm_cfg = shm_cfg_ops.get_by_id(core_info->id);
	shm_total_start = ioremap(shm_cfg->total_shm->start,shm_cfg->total_shm->len);
	
	if (shm_total_start == NULL)
	{
		INFOS("arm_shm_init_error: remap shm total fail\n");
		return -EINVAL;
	}
	LOGINFO("arm_shm_init_info: shm_total_start = %p [%u]\n",shm_total_start,(uint64_t)shm_total_start);

	misc_register(&arm_shm_dev); /* 注册杂项驱动 */

	LOGINFO("arm_shm_init_success: shm init success!\n");
	return 0;
}

static void arm_shm_exit(void)
{
	ClearPageReserved(virt_to_page(shm_block_info));
	kfree(shm_block_info); /* 归还映射内存区域 */
	
	iounmap(shm_total_start);

	misc_deregister(&arm_shm_dev); /* 注销杂项设备 */

	LOGINFO("arm_shm_exit_success: shm exit success!\n");
}

module_init(arm_shm_init);
module_exit(arm_shm_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("anguoyoula");

#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */
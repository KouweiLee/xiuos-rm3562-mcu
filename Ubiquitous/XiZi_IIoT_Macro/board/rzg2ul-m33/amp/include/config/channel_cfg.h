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

#include "addr_cfg.h"
#include "types_cfg.h"
#include "core_cfg.h"

#ifndef __AMP_CHANNEL_CFG_H__
#define __AMP_CHANNEL_CFG_H__

#if PLATFORM_CHOICE / PLATFORM_LINUX_BASE == 1 /* Linux平台 */
#define IOCTL_CMD_NOTIFY (0xAU) /* Linux用户程序向IPI驱动传递触发核间中断的命令 */
#endif /* PLATFORM_CHOICE / 10 == 1 */

/**
 * 通道配置信息，每个配置代表一个物理通道，通信的各个核心均需要该信息
 * 每个通道有两个中断，分别需要两个核心进行处理: 消息接受中断和消息响应中断
*/
struct ChannelInfo
{
    uint32_t channel_num; /* 通道编号 */

    uint32_t irq_msg; /* 通道接受消息中断：中断号或者设备树中中断下标，由通道接收方负责实现 */
    uint32_t irq_rsp; /* 通道响应消息中断：中断号或者设备树中中断下标，由通道发送方负责实现 */

    uint64_t reg_msg; /* 该通道消息发送的触发寄存器地址 */

    struct CoreInfo* src_core;  /* 通道发送核心 */
    struct CoreInfo* dst_core;  /* 通道接收核心 */

    struct AddrInfo* src_queue; /* 位于发送端地址空间：发送端向该消息队列发送信息，接收端从这个消息队列里面接收并处理信息，由接收端负责初始化 */
    struct AddrInfo* dst_queue; /* 位于接收端地址空间：接收端从该消息队列获取消息，由接收端负责初始化 */
};

/* 所有的核间通信通道信息，每个通道都存在一个发送端和接收端，同时，每个核心既可以是发送端也可以是接收端*/
extern struct ChannelInfo channel_infos[];

/* 为方便配置而起的别名 */
#define LINUX_2_XIUOS_CHANNEL_INFO (&channel_infos[0])
#define XIUOS_2_LINUX_CHANNEL_INFO (&channel_infos[1])
#endif /* __AMP_CHANNEL_CFG_H__ */
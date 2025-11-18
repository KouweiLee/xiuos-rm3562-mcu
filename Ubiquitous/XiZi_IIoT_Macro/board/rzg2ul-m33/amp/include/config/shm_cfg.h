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

#include "core_cfg.h"
#include "addr_cfg.h"
#include "types_cfg.h"
#include "platform_cfg.h"

#ifndef __AMP_SHM_CFG_H__
#define __AMP_SHM_CFG_H__

#if PROTOCOL_CHOICE == PROTOCOL_AMP /* 需要AMP协议 */

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL /* Linux平台 */
#define SHM_DEV_NAME ("amp_shm_dev") /* 设备名称 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
#define SHM_DRIVER_NAME ("/dev/amp_shm_dev") /* 驱动名称 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */

/* 短期块数量会影响到共享内存的使用效率，需要酌情考虑 */
#define SHM_VBLOCK_CNT (8U)

/* 共享内存配置信息：配置的时候保证字节对齐 */
struct ShmCfg
{
    struct CoreInfo *core;       /* 那个核心的共享内存配置 */
    struct AddrInfo *total_shm;  /* 全部共享内存的地址空间：用于偏移量计算 */
    struct AddrInfo *core_shm;   /* 属于本节点的共享内存地址空间：用于内存分配 */

    uint32_t pblock_size;        /* 长期区块总大小 */
    uint32_t vblock_size;        /* 短期区块总大小 */
    uint32_t min_block_size;     /* 最小区块大小，如果配置的块太多每个块小于该值时初始化失败 */

    uint32_t bit_align;          /* 该共享内存分配时的字节对齐配置： 2 4 8 16 ... 2^n*/
};

/* 所有配置有共享内存的核心配置信息（不是所有的核间通信核心都配置有共享内存） */
extern struct ShmCfg shm_cfgs[];

struct ShmCfgOps /* 共享内存配置信息操作集合 */
{
    /* 通过核心ID获取核心的配置信息 */
    struct ShmCfg* (*get_by_id)(uint16_t target_id);
};

extern struct ShmCfgOps shm_cfg_ops;
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */

#endif /* __AMP_SHM_CFG_H__ */
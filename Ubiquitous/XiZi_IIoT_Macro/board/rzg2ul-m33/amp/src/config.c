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

/* addr_cfg.h */
struct AddrInfo addr_infos[] =
{
    { /* 0：Linux 地址空间下所有的共享内存地址信息 8MB */
        .start = 0x43000000U,
        .len = 8 * MB
    },
    { /* 1：Linux 地址空间下使用的共享内存地址信息 4MB */
        .start = 0x43000000U,
        .len = 4 * MB
    },
    { /* 2: XiUOS 地址空间下所有的共享内存地址信息 8MB*/
        .start = 0x63000000U,
        .len = 8 * MB
    },
    { /* 3: XiUOS 地址空间下使用的共享内存地址信息 4MB*/
        .start = 0x63400000U,
        .len = 4 * MB
    },
    { /* 4: Linux 地址空间下发送给Linux的环形缓冲区 4KB */
        .start = 0x42F00000U,
        .len = 4 * KB
    },
    { /* 5: Linux 地址空间下发送给XiUOS的环形缓冲区 4KB */
        .start = 0x42F01000U,
        .len = 4 * KB
    },
    { /* 6: XiUOS 地址空间下发送给Linux的环形缓冲区 4KB */
        .start = 0x62F00000U,
        .len = 4 * KB
    },
    { /* 7: XiUOS 地址空间下发送给XiUOS的环形缓冲区 4KB */
        .start = 0x62F01000U,
        .len = 4 * KB
    }
};

/* core_cfg.h */
struct CoreInfo core_infos[] = 
{
    { /* 0: linux 节点信息 */
        .name = CORE_LINUX_NAME,
        .id = CORE_LINUX_ID
    },
    { /* 1： xiuos 节点信息*/
        .name = CORE_XIUOS_NAME,
        .id = CORE_XIUOS_ID
    },
    { /* 2: 虚无节点 */
        .name = CORE_NONE_NAME,
        .id = CORE_NONE_ID
    }
};

#if (PLATFORM_CHOICE / PLATFORM_LINUX_BASE) == 1
struct CoreInfo* core_info = LINUX_CORE_INFO;
#elif (PLATFORM_CHOICE / PLATFORM_RTOS_XIUOS) == 1
struct CoreInfo* core_info = XIUOS_CORE_INFO;
#else
#error please configure right platform
#endif

/* channel_cfg.h */
struct ChannelInfo channel_infos[] = 
{
    { /* 通道1： A55 -> M33 */
        .channel_num = 1,
        .irq_rsp = 0,  /* Linux: 响应中断配置在设备树第一个中断 */
        .irq_msg = 69, /* XiUOS: 接收中断配置为69号 */
        .reg_msg = 0x10400000U + 0x24U,
        .src_core = LINUX_CORE_INFO,
        .dst_core = XIUOS_CORE_INFO,
        .src_queue = LINUX_2_XIUOS_MSG_QUEUE_ADDR_INFO,
        .dst_queue = XIUOS_2_XIUOS_MSG_QUEUE_ADDR_INFO
    },
    { /* 通道4： M33 -> A55 */
        .channel_num = 4,
        .irq_msg = 1,  /* Linux: 接收中断配置在设备树的第二个 */
        .irq_rsp = 78, /* XiUOS: 响应中断配置为78号 */
        .reg_msg = 0x40400000U + 0x84U,
        .src_core = XIUOS_CORE_INFO,
        .dst_core = LINUX_CORE_INFO,
        .src_queue = XIUOS_2_LINUX_MSG_QUEUE_ADDR_INFO,
        .dst_queue = LINUX_2_LINUX_MSG_QUEUE_ADDR_INFO
    },
    { /* 尾 */
        .src_core = NULL
    }   
};

/* shm_cfg.h */
#if PROTOCOL_CHOICE == PROTOCOL_AMP
struct ShmCfg shm_cfgs[] = 
{
    { /* Linux 共享内存配置信息 */
        .core = LINUX_CORE_INFO,
        .total_shm = LINUX_SHM_TOTAL_ADDR_INFO,
        .core_shm = LINUX_SHM_USED_ADDR_INFO,
        .pblock_size = (2 * MB),
        .vblock_size = (2 * MB),
        .min_block_size = (512 * B),
        .bit_align = MEMORY_ALIGN_SIZE
    },
    { /* XiUOS 共享内存配置信息 */
        .core = XIUOS_CORE_INFO,
        .total_shm = XIUOS_SHM_TOTAL_ADDR_INFO,
        .core_shm = XIUOS_SHM_USED_ADDR_INFO,
        .pblock_size = (2 * MB),
        .vblock_size = (2 * MB),
        .min_block_size = (512 * B),
        .bit_align = MEMORY_ALIGN_SIZE
    },
    {
        .core = NULL
    }
};

static struct ShmCfg* shm_cfg_get_by_id(uint16_t target_id)
{
    int i = 0;
    for (i = 0; shm_cfgs[i].core != NULL; i++)
    {
        if (shm_cfgs[i].core->id == target_id)
        {
            LOGINFO("shm_cfg_get_by_id_info: find shm cfg success, target id = %u\n",target_id);
            
            return &shm_cfgs[i];
        }
    }
    return NULL;
}

struct ShmCfgOps shm_cfg_ops = 
{
    .get_by_id = shm_cfg_get_by_id
};
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
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
#include "spinlock.h"

#ifndef __AMP_SHM_H__
#define __AMP_SHM_H__

/* 该模块需要AMP协议，且位于Linux用户空间或者RTOS空间 */
#if PROTOCOL_CHOICE == PROTOCOL_AMP && (PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS)

enum MallocType /* 内存申请时用于表示在那个区块进行申请 */
{
    MALLOC_TYPE_P = 1, /* 长期区块里面进行申请: 01 */
    MALLOC_TYPE_V = 2, /* 短期区块里面进行申请: 10 */
};

struct PBlockInfo /* 长期内存块管理信息数据结构 */
{
    volatile uint32_t available_size;    /* 内存块还有多少空闲未分配内存，没有空闲空间时分配失败 */
    volatile uint32_t next_alloc_offset; /* 下一次分配内存时从那个地址偏移量开始 */
} __attribute__((aligned(MEMORY_ALIGN_SIZE)));

enum VBlockState /* 短期内存块当前状态枚举 */
{
    VBLOCK_STATE_DISENABLE = 0, /* 该块已经被分配完毕或者无法满足最近的一次内存分配请求且分配出去的内存尚未归还完毕 */
    VBLOCK_STATE_ENABLE = 1,    /* 存在可以分配的空闲内存 */
};

struct VBlockInfo /* 短期内存块管理信息 */
{
    volatile uint32_t state; /* 当前内存块的分配情况：@VBlockState */

    volatile uint32_t alloc_count; /* 当前块已经分配出去的块数 */
    volatile uint32_t free_count;  /* 当前块被分配出去的内存块已经有多少已经释放了 */

    uint32_t alloc_start_offset;         /* 区块起始地址相对于共享内存起始地址的偏移量 */
    volatile uint32_t next_alloc_offset; /* 下一块内存分配起始地址相对于共享内存起始地址的偏移量 */

    volatile uint32_t available_size; /* 当前最大可分配内存大小 */
    uint32_t total_length;            /* 记录分区的总长度，用于归还后的区块恢复 */
} __attribute__((aligned(MEMORY_ALIGN_SIZE)));

enum BlockState /* 内存管理区块的状态 */
{
    BLOCK_STATE_UNUSED = 0,     /* 共享内存划分失败 */
    BLOCK_STATE_PERSISTENT = 1, /* 共享内存存在长期区块：01 */
    BLOCK_STATE_VOLATILE = 2,   /* 共享内存存在短期区块：10 */
    BLOCK_STATE_BOTH_USED = 3,  /* 共享内存存在长期和短期区块：01 | 10 = 11 */
};

struct ShmInfos /* 总的区块管理信息数据结构：放在内核中 */
{
    atomic_char32_t block_init_mark; /* 共享内存初始化标记 */
 
    ByteFlag pblock_lock; /* 长期区块内存分配核间互斥标志 */
    ByteFlag vblock_lock; /* 短期区块内存分配核间互斥标志 */

    uint8_t block_state_info; /* 标记内存块的长期和短期划分状态: @BlockState */

    struct PBlockInfo pblock_info; /* 存放长期区块的管理信息 */
    
    uint32_t vblock_each_size; /* 每个短期块所占大小 */
    volatile uint32_t vblock_current; /* 当前使用的虚拟块下标*/
    struct VBlockInfo vblock_infos[SHM_VBLOCK_CNT]; /* 短期区块管理信息组 */
} __attribute__((aligned(MEMORY_ALIGN_SIZE)));

struct Shm /* 共享内存管理信息结构 */
{
    struct ShmInfos* shm_infos; /* 共享内存的信息，该信息共享于内核中 */

    struct ShmCfg *shm_cfg; /* 共享内存配置信息 */
    
    void *shm_total_start;  /* 全部共享内存起始地址 */
    void *shm_core_start;   /* 当前节点共享内存起始地址 */
};

struct ShmOps /* 共享内存操作集合 */
{
    /* 通过传入的指向共享内存的地址计算出相对于总共享内存起始地址的偏移量 */
    int32_t (*addr_to_offset)(void* ptr);

    /* 通过相对于总共享内存起始地址的偏移量，获取一个指向共享内存的指针 */
    void* (*offset_to_addr)(uint32_t offset);

    /* 根据相关配置初始化共享内存 */
    int32_t (*shm_init)(void);

    /* 从共享内存中分配内存空间 */
    void* (*shm_malloc)(uint32_t size,enum MallocType type);

    /* 将申请的短期共享内存归还管理池管理 */
    void (*shm_free)(void* ptr);
};

extern struct ShmOps shm_ops;
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP && (PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS) */
#endif /* __AMP_SHM_H__ */
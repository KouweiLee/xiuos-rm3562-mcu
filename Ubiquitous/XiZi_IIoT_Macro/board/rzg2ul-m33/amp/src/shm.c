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

#include "../include/shm.h"
#include "../include/spinlock.h"

#if PROTOCOL_CHOICE == PROTOCOL_AMP && (PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS)

/* 共享内存管理信息 */
static struct Shm shm = { 0 };

/* 初始化相关标记信息 */
static volatile uint32_t shm_init_mark = INIT_MARK_RAW_STATE;
static MarkFlag shm_init_lock = ATOMIC_FLAG_INIT;

static int32_t shm_addr_to_offset(void* ptr)
{
    /* 指针范围不对 */
    if (ptr < shm.shm_total_start || ptr > shm.shm_total_start + shm.shm_cfg->total_shm->len)
    {
        INFOS("shm_addr_to_offset_error: error range\n");
        return -1;
    }
    
    return ptr - shm.shm_total_start;
}

static void* shm_offset_to_addr(uint32_t offset)
{
    if (offset > shm.shm_cfg->total_shm->len)
    {
        INFOS("shm_offset_to_addr_error: error range\n");
        return NULL;
    }
    
    return (void*)(shm.shm_total_start + offset);
}

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

/* SHM模块、内存映射相关数据 */
static int mem_fd = -1;
static int shm_fd = -1;
#endif /* #if PLATFORM_CHOICE == PLATFORM_LINUX_USER */

#if PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
/* XiUOS的直接放在这里 */
static struct ShmInfos shm_infos = { 0 };
#endif /* PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

static int32_t malloc_from_volatile_block(int start_index,int end_index,int size)
{
    for (int i = start_index; i < end_index; i++)
    {
        if (shm.shm_infos->vblock_infos[i].state == VBLOCK_STATE_ENABLE) /* 当前区块还可以尝试继续进行分配 */
        {
            if (size <= shm.shm_infos->vblock_infos[i].available_size) /* 当前块可以满足此次内存分配请求 */
            {
                
                int32_t result = shm.shm_infos->vblock_infos[i].next_alloc_offset;

                shm.shm_infos->vblock_infos[i].available_size -= size;
                shm.shm_infos->vblock_infos[i].alloc_count += 1;
                shm.shm_infos->vblock_infos[i].next_alloc_offset += size;

                shm.shm_infos->vblock_current = i; 

                return result;
            }
            else /* 当前块无法满足分配要求 */
            {
                shm.shm_infos->vblock_infos[i].state = VBLOCK_STATE_DISENABLE; /* 当前块弃用，开始启用下一块 */
            }
        }
        else /* 查看当前块能不能重新加入分配 */
        {
            if (shm.shm_infos->vblock_infos[i].free_count == shm.shm_infos->vblock_infos[i].alloc_count)
            {
                shm.shm_infos->vblock_infos[i].next_alloc_offset = shm.shm_infos->vblock_infos[i].alloc_start_offset;
                shm.shm_infos->vblock_infos[i].available_size = shm.shm_infos->vblock_infos[i].total_length;
                shm.shm_infos->vblock_infos[i].free_count = 0;
                shm.shm_infos->vblock_infos[i].alloc_count = 0;
                shm.shm_infos->vblock_infos[i].state = VBLOCK_STATE_ENABLE;

                i --; /* 当前块重新加入分配 */
            }
        }
    }

    return -1; /* 分配未成功 */
}

static int32_t shm_init(void)
{
    mark_flag_ops.lock(&shm_init_lock); /* 这个锁在RTOS上可以锁全部线程，但是在Linux只能锁同一个进程内的线程 */

    if (shm_init_mark == INIT_MARK_INITIALIZED) /* 已初始化过了 */
    {
        mark_flag_ops.unlock(&shm_init_lock);
        LOGINFO("shm_init_info: shm init before\n");
        return 0;
    }

    if (shm_init_mark == INIT_MARK_DESTORYED) /* 已经初始化损坏 */
    {
        mark_flag_ops.unlock(&shm_init_lock);
        INFOS("shm_init_error: shm init error before\n");
        return -1;
    }
    
    struct ShmCfg* shm_cfg = shm_cfg_ops.get_by_id(core_info->id); /* 获取当前核心的共享内存配置信息 */
    if (shm_cfg == NULL)
    {
        shm_init_mark = INIT_MARK_DESTORYED;
        mark_flag_ops.unlock(&shm_init_lock);
        INFOS("shm_init_error: have no shm cfg for core = %u\n",core_info->id);
        return -1;
    }
    shm.shm_cfg = shm_cfg;

    if (shm_cfg->total_shm == NULL)
    {
        shm_init_mark = INIT_MARK_DESTORYED;
        mark_flag_ops.unlock(&shm_init_lock);
        INFOS("shm_init_error: must have total shm addr info even have no core shm\n");
        return -1;
    }
    
    /* 提取各项共享内存配置信息 */
    uint64_t total_shm_start = shm_cfg->total_shm->start;
    uint32_t total_shm_length = shm_cfg->total_shm->len;
    uint64_t total_shm_end = total_shm_start + total_shm_length;
    INFOS("shm_init_info: total_shm [start = %p, length = %u KB]\n",(void*)total_shm_start,total_shm_length / KB);

    uint64_t core_shm_start = 0U;
    uint32_t core_shm_length = 0U;
    uint64_t core_shm_end = 0U;

    if (shm_cfg->core_shm == NULL) /* 当前节点没有共享内存 */
    {
        LOGINFO("shm_init_warn: have no core shm\n");
    }
    else /* 当前节点存在共享内存 */
    {
        core_shm_start = shm_cfg->core_shm->start;
        core_shm_length = shm_cfg->core_shm->len;
        core_shm_end = core_shm_start + core_shm_length;
        LOGINFO("shm_init_info: core_shm [start = %p, length = %u KB]\n",(void*)core_shm_start,core_shm_length / KB);
    }

    /* 初始化共享内存管理指针 */
#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
    mem_fd = open(MEM_DRIVER, O_RDWR | O_SYNC); /* 打开全映射驱动 */
    if (mem_fd < 0) 
    {
        shm_init_mark = INIT_MARK_DESTORYED;
	    mark_flag_ops.unlock(&shm_init_lock);

        INFOS("shm_init_error: open mem dev fail %d\n", mem_fd);
        return -1;
    }
    LOGINFO("shm_init_info: open mem dev success %d\n", mem_fd);

    shm_fd = open(SHM_DRIVER_NAME, O_RDWR | O_SYNC); /* 打开SHM模块 */
    if (shm_fd < 0) 
    {
        shm_init_mark = INIT_MARK_DESTORYED;
	    mark_flag_ops.unlock(&shm_init_lock);

        INFOS("shm_init_error: open shm dev fail %d\n", shm_fd);
        return -1;
    }
    LOGINFO("shm_init_info: open shm dev success %d\n", shm_fd);

    /* 将内核中的共有管理信息映射到用户空间 */
    shm.shm_infos = mmap(NULL, MEM_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shm.shm_infos == NULL)
    {
        shm_init_mark = INIT_MARK_DESTORYED;
	    mark_flag_ops.unlock(&shm_init_lock);

	    INFOS("shm_init_error: mmap shm manage info fail\n");
	    return -1;
    }

    /* 映射出内核中的管理信息块 */
    shm.shm_total_start = mmap(NULL, total_shm_length, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, total_shm_start);
    shm.shm_core_start = shm.shm_total_start + (core_shm_start - total_shm_start);

    if (shm.shm_total_start == NULL)
    {
        shm_init_mark = INIT_MARK_DESTORYED;
	    mark_flag_ops.unlock(&shm_init_lock);

        INFOS("shm_init_error: mmap shm fail\n");
        return -1;
    }
    LOGINFO("shm_init_info: virtual addr [total_start = %p, core_start = %p]\n",shm.shm_total_start,shm.shm_core_start);
#endif /* #if PLATFORM_CHOICE == PLATFORM_LINUX_USER */
#if PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
    shm.shm_infos = &shm_infos;

    shm.shm_total_start = (void *)total_shm_start;
    shm.shm_core_start = (void *)core_shm_start;
#endif /* PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

    if (atomic_load(&shm.shm_infos->block_init_mark) == INIT_MARK_INITIALIZED) /* 在整个节点内存探测是否已经被初始化过了 */
    {
        shm_init_mark = INIT_MARK_INITIALIZED;
        mark_flag_ops.unlock(&shm_init_lock);

        LOGINFO("shm_init_info: shm init before 2\n");
        return 0;
    }

    uint32_t except_state = INIT_MARK_RAW_STATE; /* 假定该共享内存还未被初始化 */
    /* 试图获取初始化权 */
    if (!atomic_compare_exchange_strong(&shm.shm_infos->block_init_mark,&except_state,INIT_MARK_INITIALIZING))
    {
        while (1)
        {
            uint8_t init_state = atomic_load(&shm.shm_infos->block_init_mark);
            if (init_state == INIT_MARK_INITIALIZING)
            {
                LOGINFO("shm_init_info: wait others shm init over\n");
            }
            else if (init_state == INIT_MARK_INITIALIZED)
            {
                shm_init_mark = INIT_MARK_INITIALIZED;
                mark_flag_ops.unlock(&shm_init_lock);

                LOGINFO("shm_init_info: init by others\n");
                return 0;
            }
            else if (init_state == INIT_MARK_DESTORYED)
            {
                shm_init_mark = INIT_MARK_DESTORYED;
                mark_flag_ops.unlock(&shm_init_lock);

                INFOS("shm_init_error: init shm error by others or init mark error\n");
                return -1;
            }
        }
    }

    /* 开始进行共享内存的初始化 */
    shm.shm_infos->block_state_info = BLOCK_STATE_UNUSED; /* 初始共享内存状态 */

    if (core_shm_length == 0) /* 当前节点没有共享内存 */
    {
        atomic_store(&shm.shm_infos->block_init_mark,INIT_MARK_INITIALIZED); /* 通知其他线程共享内存初始化完毕 */
        shm_init_mark = INIT_MARK_INITIALIZED;
        mark_flag_ops.unlock(&shm_init_lock);

        LOGINFO("shm_init_info: init success ,but have no core shm\n");
        return 0;
    }
    /* 初始化当前节点的共享内存 */

    uint32_t min_block_size = shm_cfg->min_block_size < MEMORY_ALIGN_SIZE ? MEMORY_ALIGN_SIZE : shm_cfg->min_block_size; /* 短期区块最小区块大小 */
    uint32_t pblock_size = shm_cfg->pblock_size > min_block_size ? shm_cfg->pblock_size : 0; /* 长期区块总大小 */
    if (pblock_size > 0)
    {
        shm.shm_infos->block_state_info = shm.shm_infos->block_state_info | BLOCK_STATE_PERSISTENT; /* 共享内存里面有长期块 */
    }

    uint32_t vblock_size = shm_cfg->vblock_size > min_block_size ? shm_cfg->vblock_size : 0; /* 短期区块总大小 */
    uint32_t vblock_each_size = 0U;
    if (vblock_size > 0 && SHM_VBLOCK_CNT > 0)
    {
        shm.shm_infos->block_state_info = shm.shm_infos->block_state_info | BLOCK_STATE_VOLATILE; /* 共享内存里面有短期块 */
        vblock_each_size = vblock_size / SHM_VBLOCK_CNT;
        if (vblock_each_size < min_block_size) /* 查看内存配置是否合理 */
        {
            atomic_store(&shm.shm_infos->block_init_mark,INIT_MARK_DESTORYED); /* 告知其他线程共享内存初始化失败 */
            shm_init_mark = INIT_MARK_DESTORYED;
            mark_flag_ops.unlock(&shm_init_lock);

            INFOS("shm_init_error: each vblock is too small vblock_each_size =%u, min_block_size = %u, vblock_size = %u, vblock_count = %u\n",vblock_each_size, min_block_size, vblock_size, SHM_VBLOCK_CNT);
            return -1;
        }
        shm.shm_infos->vblock_each_size = vblock_each_size;
    }
    INFOS("shm_init_info: pblock_size = %u KB, vblock_size = %u KB, vblock_each_size = %u KB, min_block_size = %u B\n", pblock_size / KB, vblock_size / KB, vblock_each_size / KB, min_block_size);

    /* 初始化互斥标记 */
    byte_flag_ops.init(&shm.shm_infos->pblock_lock);
    byte_flag_ops.init(&shm.shm_infos->vblock_lock);

    uint32_t init_offset = core_shm_start - total_shm_start; /* 开始初始化共享内存的管理信息 */

    /* 先初始化长期区 */
    shm.shm_infos->pblock_info.next_alloc_offset = init_offset;
    shm.shm_infos->pblock_info.available_size = pblock_size;

    init_offset += pblock_size;

    /* 初始化短期区 */
    if (vblock_each_size > min_block_size) /* 存在短期区 */
    {
        for (int i = 0; i < SHM_VBLOCK_CNT; i++)
        {
            shm.shm_infos->vblock_infos[i].state = VBLOCK_STATE_ENABLE;
            shm.shm_infos->vblock_infos[i].alloc_count = 0U;
            shm.shm_infos->vblock_infos[i].free_count = 0U;
            shm.shm_infos->vblock_infos[i].alloc_start_offset = init_offset;
            shm.shm_infos->vblock_infos[i].next_alloc_offset = init_offset;

            if (i != SHM_VBLOCK_CNT - 1) /* 非最后一个短期块 */
            {
                shm.shm_infos->vblock_infos[i].total_length = vblock_each_size;
                shm.shm_infos->vblock_infos[i].available_size = vblock_each_size;
                init_offset += vblock_each_size;
            }
            else /* 最后一个短期块 */
            {
                shm.shm_infos->vblock_infos[i].total_length = vblock_each_size + vblock_size % SHM_VBLOCK_CNT;
                shm.shm_infos->vblock_infos[i].available_size = shm.shm_infos->vblock_infos[i].total_length;
                init_offset += shm.shm_infos->vblock_infos[i].total_length;
            }
            INFOS("shm_init_info: index = %d, start_offset = %u, total_length = %u KB\n", i, shm.shm_infos->vblock_infos[i].alloc_start_offset, shm.shm_infos->vblock_infos[i].total_length / KB);
        }

        if (init_offset != core_shm_end - total_shm_start) /* 是否正确初始化 */
        {
            atomic_store(&shm.shm_infos->block_init_mark,INIT_MARK_DESTORYED);
            shm_init_mark = INIT_MARK_DESTORYED;
            mark_flag_ops.unlock(&shm_init_lock);

            INFOS("shm_init_error: core_shm_end = %u, init_offset = %u\n", core_shm_end, init_offset);
            return -1;
        }
    }

    atomic_store(&shm.shm_infos->block_init_mark,INIT_MARK_INITIALIZED); /* 通知其他线程共享内存初始化完毕 */
    shm_init_mark = INIT_MARK_INITIALIZED;
    mark_flag_ops.unlock(&shm_init_lock);

    LOGINFO("shm_init_success: init shm success\n");
    return 0;
}

static void* shm_malloc(uint32_t size,enum MallocType type)
{
    if (shm_init_mark != INIT_MARK_INITIALIZED)
    {
        INFOS("shm_malloc_error: init shm before use\n");
        return NULL;
    }
    
    if (size == 0 || (shm.shm_infos->block_state_info & type) == 0)
    {
        INFOS("shm_malloc_warn: malloc size = 0 or unsupport malloc type\n");
        return NULL;
    }
    /* 按照配置信息进行字节对齐 */
    size = (size + shm.shm_cfg->bit_align -1) & (~(shm.shm_cfg->bit_align - 1));

    switch (type)
    {
        case MALLOC_TYPE_P:
        {
            byte_flag_ops.lock(&shm.shm_infos->pblock_lock); /* 长期区锁 */

            if (size > shm.shm_infos->pblock_info.available_size) /* 申请的内存大小超过了最大内存限制 */
            {
                byte_flag_ops.unlock(&shm.shm_infos->pblock_lock); /* 解锁 */

                INFOS("shm_malloc_warn: the size is large than max avi size, size = %d, avi size = %d\n", size, shm.shm_infos->pblock_info.available_size);
                return NULL;
            }
            void* result = shm.shm_total_start + shm.shm_infos->pblock_info.next_alloc_offset; /* 记录下需要返回的地址 */
            shm.shm_infos->pblock_info.available_size -= size; /* 减去已经被分配的空间大小 */
            shm.shm_infos->pblock_info.next_alloc_offset += size; /* 将待分配地址指向下一个位置 */

            byte_flag_ops.unlock(&shm.shm_infos->pblock_lock); /* 解锁 */

            LOGINFO("shm_malloc_success: MALLOC_TYPE_P [start address: %p, length = %d B]\n", result, size);

            return result;
        }
        case MALLOC_TYPE_V:
        {
            byte_flag_ops.lock(&shm.shm_infos->vblock_lock); /* 短期锁 */

            uint32_t current_volatile_block_index = shm.shm_infos->vblock_current; /* 记录开始查找前短期块的当前位置下标 */

            int32_t result = malloc_from_volatile_block(current_volatile_block_index,SHM_VBLOCK_CNT,size); /* 从当前位置往后找 */

            if (result < 0) /* 没找到 */
            {
                result = malloc_from_volatile_block(0,current_volatile_block_index,size); /* 从前往当前位置找 */
            }

            if (result < 0) /* 无法分配空间 */
            {
                byte_flag_ops.unlock(&shm.shm_infos->vblock_lock); /* 解锁 */

                INFOS("shm_malloc_warn: have no such size space free now!\n");
                return NULL;
            }

            byte_flag_ops.unlock(&shm.shm_infos->vblock_lock); /* 解锁 */

            LOGINFO("shm_malloc_success: MALLOC_TYPE_V [start address: %p, length = %d B]\n", shm.shm_total_start + result, size);

            return shm.shm_total_start + result;
        }
        default:
        {
            INFOS("shm_malloc_warn: not suported alloc type!\n");
            return NULL;
        }
    }
}

static void shm_free(void* ptr)
{
    if (shm_init_mark != INIT_MARK_INITIALIZED)
    {
        INFOS("shm_free: shm not init\n");
        return ;
    }

    if (shm.shm_infos->block_state_info & BLOCK_STATE_VOLATILE == 0)
    {
        INFOS("shm_free_error: have no volatile blocks\n");
        return;
    }

    if (ptr < shm.shm_core_start || ptr >= shm.shm_core_start + shm.shm_cfg->core_shm->len)
    {
        INFOS("shm_free_error: not such shm\n");
        return ;
    }

    uint32_t data_offset = ptr - shm.shm_core_start - shm.shm_cfg->pblock_size;

    uint32_t block_index = data_offset / shm.shm_infos->vblock_each_size; /* 读取短期块的控制信息 */

    byte_flag_ops.lock(&shm.shm_infos->vblock_lock);

    shm.shm_infos->vblock_infos[block_index].free_count += 1;

    LOGINFO("shm_free_success: start = %p, index = %u, free_count = %u\n",ptr ,block_index, shm.shm_infos->vblock_infos[block_index].free_count);
    byte_flag_ops.unlock(&shm.shm_infos->vblock_lock);
}

struct ShmOps shm_ops =
{
    .addr_to_offset = shm_addr_to_offset,
    .offset_to_addr = shm_offset_to_addr,
    .shm_init = shm_init,
    .shm_malloc = shm_malloc,
    .shm_free = shm_free
};
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP && (PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS) */
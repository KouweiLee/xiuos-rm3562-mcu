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

/**
 * @file:    kalloc.c
 * @brief:   the general management of system memory
 * @version: 3.0
 * @author:  AIIT XUOS Lab
 * @date:    2023/8/28
 */
/*************************************************
File name: kalloc.c
Description: the general management of system memory
Others:
History:
1. Date: 2023-08-28
Author: AIIT XUOS Lab
Modification:
1. first version
*************************************************/
#include "kalloc.h"
#include "assert.h"

#include "memlayout.h"
#include "pagetable.h"

#include "actracer.h"
#include "buddy.h"


struct KBuddy kern_virtmem_buddy;
struct KBuddy user_phy_freemem_buddy;

extern uintptr_t kernel_data_end[];
bool module_phymem_init()
{
#ifndef __riscv
    uintptr_t kern_freemem_start = V2P(kernel_data_end);
#else
    uintptr_t kern_freemem_start = V2P_LINK(kernel_data_end);
#endif
    uintptr_t kern_freemem_end = PHY_KERN_STOP;
    uintptr_t user_freemem_start = PHY_USER_FREEMEM_BASE;
    uintptr_t user_freemem_end = PHY_USER_FREEMEM_TOP;
    user_phy_freemem_buddy.pages = NULL;
    KBuddySysInit(&kern_virtmem_buddy, kern_freemem_start, kern_freemem_end);
    KBuddyInit(&user_phy_freemem_buddy, user_freemem_start, user_freemem_end);
    return true;
}

char* kalloc(uintptr_t size)
{
    char* mem_alloc = KBuddyAlloc(&kern_virtmem_buddy, size);
    if (mem_alloc == NULL) {
        return NULL;
    }
#ifndef __riscv
    assert((uintptr_t)mem_alloc >= V2P(&kernel_data_end) && (uintptr_t)mem_alloc < PHY_USER_FREEMEM_BASE);
#else
    assert((uintptr_t)mem_alloc >= V2P_LINK(&kernel_data_end) && (uintptr_t)mem_alloc < PHY_USER_FREEMEM_BASE);
#endif
    mem_alloc = P2V(mem_alloc);
    if ((uintptr_t)mem_alloc < KERN_MEM_BASE) {
        DEBUG("Error Alloc: %x by size: %d (Caused by double free)\n", mem_alloc, size);
    }
    memset(mem_alloc, 0, size);
    return mem_alloc;
}

void* kalloc_by_ownership(TraceTag owner, uintptr_t size)
{
    void* new_mem = kalloc(size);
    if (NULL == new_mem) {
        return NULL;
    }

    struct MemUsage* usage = GetSysObject(struct MemUsage, &owner);
    if (0 != rbt_insert(&usage->mem_block_map, (uintptr_t)new_mem, NULL)) {
        kfree(new_mem);
        return NULL;
    }
    return new_mem;
}

bool kfree(char* vaddr)
{
    return KBuddyFree(&kern_virtmem_buddy, V2P_WO(vaddr));
}

bool kfree_by_ownership(TraceTag owner, void* vaddr)
{
    struct MemUsage* usage = GetSysObject(struct MemUsage, &owner);
    // DEBUG("%p %p %p %p\n", usage, usage->mem_block_root, usage->tag, vaddr);
    RbtNode* node = rbt_search(&usage->mem_block_map, (uintptr_t)vaddr);
    assert(NULL != node);
    assert(0 == rbt_delete(&usage->mem_block_map, node->key));

    return kfree(vaddr);
}

bool raw_kfree(char* paddr)
{
    return KBuddyFree(&kern_virtmem_buddy, paddr);
}

char* raw_alloc(size_t size)
{
    char* mem_alloc = KBuddyAlloc(&user_phy_freemem_buddy, size);
    if (mem_alloc == NULL) {
        return NULL;
    }
    return mem_alloc;
}

void* raw_alloc_by_ownership(TraceTag owner, uintptr_t size)
{
    void* new_mem = raw_alloc(size);
    if (!new_mem) {
        return NULL;
    }

    struct MemUsage* usage = GetSysObject(struct MemUsage, &owner);
    if (0 != rbt_insert(&usage->mem_block_map, (uintptr_t)new_mem, NULL)) {
        raw_free(new_mem);
        return NULL;
    }
    return new_mem;
}

bool raw_free(char* paddr)
{
    return KBuddyFree(&user_phy_freemem_buddy, paddr);
}

bool raw_free_by_ownership(TraceTag owner, void* vaddr)
{
    struct MemUsage* usage = GetSysObject(struct MemUsage, &owner);
    RbtNode* node = rbt_search(&usage->mem_block_map, (uintptr_t)vaddr);
    assert(NULL != node);
    assert(0 == rbt_delete(&usage->mem_block_map, node->key));

    return raw_free(vaddr);
}

void show_phymem_info()
{
    KFreePagesInfo(&user_phy_freemem_buddy);
}
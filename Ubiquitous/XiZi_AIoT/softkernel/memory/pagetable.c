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
 * @file pagetable.c
 * @brief build page table
 * @version 3.0
 * @author AIIT XUOS Lab
 * @date 2023.08.25
 */

/*************************************************
File name: pagetable.c
Description: build page table
Others:
History:
1. Date: 2023-08-28
Author: AIIT XUOS Lab
Modification:
1. first version
*************************************************/
#include "memlayout.h"

#include "trap_common.h"

#include "assert.h"
#include "buddy.h"
#include "kalloc.h"
#include "pagetable.h"

static struct PagerRightGroup right_group;
struct MmuCommonDone* _p_pgtbl_mmu_access = NULL;
static bool _new_pgdir(struct TopLevelPageDirectory* pgdir)
{
    void* new_pgdir_addr = 0;
    if (UNLIKELY((new_pgdir_addr = kalloc(TOPLEVLE_PAGEDIR_SIZE)) == NULL)) {
        return false;
    }

    pgdir->pd_addr = new_pgdir_addr;

    memset(new_pgdir_addr, 0, TOPLEVLE_PAGEDIR_SIZE);
    return true;
}

static bool _map_pages(uintptr_t* pgdir, uintptr_t vaddr, uintptr_t paddr, intptr_t len, uintptr_t attr)
{
    assert(len >= 0);
    vaddr = ALIGNDOWN(vaddr, LEVEL4_PTE_SIZE);
    paddr = ALIGNDOWN(paddr, LEVEL4_PTE_SIZE);
    uintptr_t vaddr_last = ALIGNDOWN(vaddr + len - 1, LEVEL4_PTE_SIZE);

    while (true) {
        uintptr_t* pte = NULL;
        if ((pte = _page_walk(pgdir, vaddr, true)) == NULL) {
            ERROR("pte not found for vaddr %p.\n", vaddr);
            return false;
        }

        if (UNLIKELY(*pte != 0)) {
            ERROR("remapping: vaddr: %p | paddr: %p | pte: %p |\n", vaddr, paddr, *pte);
            return false;
        }

        *pte = paddr | attr;

        if (vaddr == vaddr_last) {
            break;
        }

        vaddr += PAGE_SIZE;
        paddr += PAGE_SIZE;
    }

    assert(vaddr == vaddr_last);
    return true;
}

static bool _unmap_pages(uintptr_t* pgdir, uintptr_t vaddr, int len)
{
    assert(len >= 0);
    vaddr = ALIGNDOWN(vaddr, LEVEL4_PTE_SIZE);
    uintptr_t vaddr_last = ALIGNDOWN(vaddr + len - 1, LEVEL4_PTE_SIZE);

    while (true) {
        uintptr_t* pte = NULL;
        if ((pte = _page_walk(pgdir, vaddr, false)) == NULL) {
            ERROR("pte not found for vaddr %p.\n", vaddr);
            return false;
        }

        if (*pte == 0) {
            ERROR("unmap a unmapped page, vaddr: %p, pte: %p\n", vaddr, *pte);
            return false;
        }

        *pte = 0;

        if (vaddr == vaddr_last) {
            break;
        }

        vaddr += PAGE_SIZE;
    }

    assert(vaddr == vaddr_last);
    return true;
}

/// @brief map paddr to vaddr for given pgdir (user only)
/// @param pgdir vaddr of pgdir
/// @param vaddr
/// @param paddr
/// @param len
/// @param is_dev
/// @return
static bool _map_user_pages(struct MemSpace* pmemspace, uintptr_t vaddr, uintptr_t paddr, int len, bool is_dev)
{
    if (len < 0) {
        return false;
    }

    if (UNLIKELY(vaddr >= USER_MEM_TOP)) {
        ERROR("mapping kernel space.\n");
        return false;
    }

    uintptr_t mem_attr = 0;
    if (LIKELY(!is_dev)) {
        _p_pgtbl_mmu_access->MmuUsrPteAttr(&mem_attr);
    } else {
        _p_pgtbl_mmu_access->MmuUsrDevPteAttr(&mem_attr);
    }

    return _map_pages(pmemspace->pgdir.pd_addr, vaddr, paddr, (intptr_t)len, mem_attr);
}

bool _map_customizable_page(struct MemSpace* pmemspace, uintptr_t vaddr, uintptr_t paddr, int len, uintptr_t attr)
{
    if (len < 0) {
        return false;
    }

    if (UNLIKELY(vaddr >= USER_MEM_TOP)) {
        ERROR("mapping kernel space.\n");
        return false;
    }

    return _map_pages(pmemspace->pgdir.pd_addr, vaddr, paddr, (intptr_t)len, attr);
}

/// assume that a user pagedir is allocated from [0, size)
/// if new_size > old_size, allocate more space,
/// if old_size > new_size, free extra space, to avoid unnecessary alloc/free.
static uintptr_t _resize_user_pgdir(struct MemSpace* pmemspace, uintptr_t old_size, uintptr_t new_size)
{
    if (UNLIKELY(new_size > USER_MEM_TOP)) {
        ERROR("user size out of range.\n");
        return old_size;
    }
    if (UNLIKELY(new_size < old_size)) {
        /// @todo: free extra space.
        return old_size;
    }

    uintptr_t cur_size = ALIGNUP(old_size, PAGE_SIZE);
    uintptr_t size_needed = ALIGNUP(new_size, PAGE_SIZE) - cur_size;

    // char* new_page = kalloc(size_needed);
    char* new_page = kalloc_by_ownership(pmemspace->kernspace_mem_usage.tag, size_needed);
    if (new_page == NULL) {
        ERROR("No memory\n");
        return cur_size;
    }
    memset(new_page, 0, size_needed);
    if (!xizi_pager.map_pages(pmemspace, cur_size, V2P(new_page), size_needed, false)) {
        return cur_size;
    }

    return new_size;
}

/// @brief translate virt address to phy address with pgdir
/// @param pgdir
/// @param vaddr accept only page aligned address
/// @return paddr of pgdir(vaddr); zero for unmapped addr
static uintptr_t _address_translate(struct TopLevelPageDirectory* pgdir, uintptr_t vaddr)
{
    assert(vaddr % PAGE_SIZE == 0);
    const uintptr_t* const pte = _page_walk(pgdir->pd_addr, vaddr, false);
    if (pte == NULL || *pte == 0) {
        return 0;
    }
    return (uintptr_t)ALIGNDOWN(*pte, PAGE_SIZE);
}

static uintptr_t _cross_vspace_data_copy_in_page(struct TopLevelPageDirectory* pgdir, uintptr_t cross_dest, uintptr_t src, uintptr_t len)
{
    uintptr_t cross_dest_end = cross_dest + len;
    assert(ALIGNUP(cross_dest, PAGE_SIZE) == ALIGNUP(cross_dest_end, PAGE_SIZE));

    uintptr_t paddr = xizi_pager.address_translate(pgdir, ALIGNDOWN(cross_dest, PAGE_SIZE));
    uintptr_t offset = cross_dest - ALIGNDOWN(cross_dest, PAGE_SIZE);
    uintptr_t* vdest = (uintptr_t*)((uintptr_t)P2V(paddr) + offset);
    uintptr_t* vsrc = (uintptr_t*)src;
    memcpy(vdest, vsrc, len);
    return len;
}

/// @brief copy data from src(kernel vspace) to dest of pgdir vspace
/// @param pgdir
/// @param cross_dest vaddress in pgdir
/// @param src
/// @param len
/// @return
static uintptr_t _cross_vspace_data_copy(struct TopLevelPageDirectory* pgdir, uintptr_t cross_dest, uintptr_t src, uintptr_t len)
{
    uintptr_t len_to_top = ALIGNUP(cross_dest, PAGE_SIZE) - cross_dest;

    uintptr_t copied_len = 0;
    while (copied_len < len) {
        uintptr_t current_copy_len = len_to_top >= len ? len : len_to_top;

        current_copy_len = _cross_vspace_data_copy_in_page(pgdir, cross_dest, src, current_copy_len);

        // update variables
        copied_len += current_copy_len;
        cross_dest += current_copy_len;
        src += current_copy_len;
        len_to_top = ALIGNDOWN(cross_dest + PAGE_SIZE, PAGE_SIZE) - ALIGNDOWN(cross_dest, PAGE_SIZE); // actually PAGE_SIZE
        assert(len_to_top == PAGE_SIZE);
    }

    return len;
}

struct XiziPageManager xizi_pager = {
    .new_pgdir = _new_pgdir,
    .free_user_pgdir = _free_user_pgdir,
    .map_pages = _map_user_pages,
    .unmap_pages = _unmap_pages,

    .resize_user_pgdir = _resize_user_pgdir,
    .address_translate = _address_translate,
    .cross_vspace_data_copy = _cross_vspace_data_copy,
};

bool module_pager_init(struct PagerRightGroup* _right_group)
{
    right_group = *_right_group;
    _p_pgtbl_mmu_access = AchieveResource(&right_group.mmu_driver_tag);
    return _p_pgtbl_mmu_access != NULL;
}

/// @brief kernel pagedir
struct TopLevelPageDirectory kern_pgdir;

void load_kern_pgdir(struct TraceTag* mmu_driver_tag, struct TraceTag* intr_driver_tag)
{
    if (mmu_driver_tag->meta == NULL) {
        ERROR("Invalid mmu driver tag.\n");
        return;
    }

    if (!_new_pgdir(&kern_pgdir)) {
        panic("cannot alloc kernel page directory");
    }
    uintptr_t kern_attr = 0;
    _p_pgtbl_mmu_access->MmuKernPteAttr(&kern_attr);
    uintptr_t dev_attr = 0;
    _p_pgtbl_mmu_access->MmuDevPteAttr(&dev_attr);

    // kern mem
    _map_pages((uintptr_t*)kern_pgdir.pd_addr, KERN_MEM_BASE, PHY_MEM_BASE, (PHY_KERN_STOP - PHY_MEM_BASE), kern_attr);
    // dev mem
    _map_pages((uintptr_t*)kern_pgdir.pd_addr, DEV_VRTMEM_BASE, DEV_PHYMEM_BASE, DEV_MEM_SIZE, dev_attr);

    _p_pgtbl_mmu_access->LoadPgdir((uintptr_t)V2P(kern_pgdir.pd_addr));
}

void secondary_cpu_load_kern_pgdir(struct TraceTag* mmu_driver_tag, struct TraceTag* intr_driver_tag)
{
    _p_pgtbl_mmu_access->LoadPgdir((uintptr_t)V2P(kern_pgdir.pd_addr));
}
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
 * @file main.c
 * @brief main
 * @version 3.0
 * @author AIIT XUOS Lab
 * @date 2023.08.25
 */

/*************************************************
File name: main.c
Description: main
Others:
History:
1. Date: 2023-08-28
Author: AIIT XUOS Lab
Modification:
1. first version
*************************************************/
/// @todo use hardkernel

#include "kern_init.h"
#include "multicores.h"

#include "assert.h"
#include "kalloc.h"
#include "task.h"

struct spinlock whole_kernel_lock;

extern uintptr_t _binary_init_start[], _binary_default_fs_start[];
extern int sys_spawn(char* img_start, char* name, char** argv);

static struct TraceTag hardkernel_tag, softkernel_tag;
static volatile int core_para_init = 0;

static void main_sync_cores()
{
    while (core_para_init != ((1 << NR_CPU) - 1))
        ;
    return;
}

int main(void)
{
    /* init tracer */
    uint32_t cpu_id = cur_cpuid();
    core_para_init |= (1 << cpu_id);

    if (cpu_id == 0) {
        /* init memory management first */

        module_phymem_init(); // init buddy management system
        /* init tracer system */
        sys_tracer_init();
        if (!CreateResourceTag(&hardkernel_tag, RequireRootTag(), "hardkernel", TRACER_OWNER, NULL) || //
            !CreateResourceTag(&softkernel_tag, RequireRootTag(), "softkernel", TRACER_OWNER, NULL)) {
            ERROR("Failed to create hardkernel owner and softkernel owner.\n");
            return -1;
        }
        /* init hardkernel (print need after init hardkernel)*/
        if (!hardkernel_init(&hardkernel_tag)) {
            return -1;
        }

        spinlock_init(&whole_kernel_lock, "wklock");
    } else {
        spinlock_lock(&whole_kernel_lock);
        secondary_cpu_hardkernel_init(cpu_id, &hardkernel_tag);
        spinlock_unlock(&whole_kernel_lock);
    } 
    spinlock_lock(&whole_kernel_lock);
    if (cpu_id == 0) {
        /* init softkernel */
        if (!softkernel_init(&hardkernel_tag, &softkernel_tag)) {
            return -1;
        }
        show_xizi_bar();

        for (int i = 1; i < NR_CPU; i++) {
            // start secondary cpus
            while ((core_para_init & (1 << (i - 1))) == 0)
                ;
            cpu_start_secondary(i);
        }

        /* start first task */
#ifndef __riscv
        char* init_task_param[2] = { "/app/shell", 0 };
        sys_spawn((char*)_binary_init_start, "shell", init_task_param);
        char* fs_server_task_param[2] = { "/app/fs_server", 0 };
        sys_spawn((char*)_binary_default_fs_start, "memfs", fs_server_task_param);
#else
        char* fs_server_task_param[2] = { "/app/fs_server", 0 };
        sys_spawn((char*)_binary_default_fs_start, "memfs", fs_server_task_param);
        char* init_task_param[2] = { "/app/shell", 0 };
        sys_spawn((char*)_binary_init_start, "shell", init_task_param);
#endif
    }
    /* start scheduler */
    struct SchedulerRightGroup scheduler_rights;
    assert(AchieveResourceTag(&scheduler_rights.mmu_driver_tag, &hardkernel_tag, "mmu-ac-resource"));
    assert(AchieveResourceTag(&scheduler_rights.intr_driver_tag, &hardkernel_tag, "intr-ac-resource"));
    spinlock_unlock(&whole_kernel_lock);

    // sync memory
    __sync_synchronize();
    main_sync_cores();
    start_smp_cache_broadcast(cpu_id);
    // enter kernel seriously
    xizi_enter_kernel();
    xizi_task_manager.task_scheduler(scheduler_rights);

    // never reached
    return 0;
}

__attribute__((weak)) void _exit(int32_t status)
{
    (void)status;
    while (1) {
        ;
    }
}
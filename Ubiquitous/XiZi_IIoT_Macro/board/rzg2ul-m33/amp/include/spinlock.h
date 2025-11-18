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

#ifndef __AMP_SPINLOCK_H__
#define __AMP_SPINLOCK_H__

/* 基于 stdatomic ，只能运行在有 stdatomic.h 的环境 */
#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
#include <stdatomic.h>

#define BYTE_LOCKED_STATE (0xFU)
#define BYTE_UNLOCK_STATE (0xAU)

typedef atomic_uchar ByteFlag;

/* 不支持递归加锁 */
struct ByteFlagOps /* CAS原子锁操作集合 */
{
    /* 初始化原子锁 */
    void (*init)(ByteFlag* flag);
    /* 查看原子锁是否被锁定 */
    int32_t (*is_locked)(ByteFlag* flag);
    /* 原子锁锁定 */
    void (*lock)(ByteFlag* flag);
    /* 原子锁释放 */
    int32_t (*unlock)(ByteFlag* flag);
    /* 原子锁试图加锁 */
    int32_t (*try_lock)(ByteFlag* flag);
};

extern struct ByteFlagOps byte_flag_ops;

/* 适合存放在代码区的互斥量 */
typedef atomic_flag MarkFlag;

struct MarkFlagOps
{
    /* 原子锁锁定 */
    void (*lock)(MarkFlag* flag);
    /* 原子锁释放 */
    void (*unlock)(MarkFlag* flag);
    /* 原子锁试图加锁 */
    int32_t (*try_lock)(MarkFlag* flag);
};

extern struct MarkFlagOps mark_flag_ops;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */

#endif /* __AMP_SPINLOCK_H__ */
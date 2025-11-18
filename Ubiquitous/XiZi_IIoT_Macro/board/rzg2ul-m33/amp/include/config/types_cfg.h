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

#include "platform_cfg.h"

#ifndef __AMP_TYPES_CFG_H__
#define __AMP_TYPES_CFG_H__

/* 保证允许在各个环境下的代码看到的数据结构大小一致：编译链工具使用GCC */
#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
#include <linux/types.h>
#elif PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
#include <stdint.h>
#else
#error please configure std types header files
#endif

#endif /* __AMP_TYPES_CFG_H__ */
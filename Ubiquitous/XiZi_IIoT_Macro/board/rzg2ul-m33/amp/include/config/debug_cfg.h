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

#ifndef __AMP_DEBUG_CFG_H__
#define __AMP_DEBUG_CFG_H__

/* 调试输出信息开关 */
#define DEBUG_ENABLE (1U)
#define DEBUG_DISABLE (0U)

#define DEBUG_CHOICE (DEBUG_DISABLE)

/* 错误输出接口：必输出，平台相关 */
#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
#include <linux/kernel.h>
#define INFOS(format, ...) (printk(format, ##__VA_ARGS__))
#elif PLATFORM_CHOICE == PLATFORM_LINUX_USER
#include <stdio.h>
#define INFOS(format, ...) (printf(format, ##__VA_ARGS__))
#elif PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS
#include <xizi.h>
#define INFOS(format, ...) (KPrintf(format, ##__VA_ARGS__))
#else
#error please configure the standard output functions of this platform for error infos
#endif

/* 调试输出接口：受调试输出信息开关影响 */
#if DEBUG_CHOICE == DEBUG_ENABLE
#define LOGINFO(format, ...) (INFOS(format, ##__VA_ARGS__))
#else
#define LOGINFO(format, ...) { }
#endif

#endif /* __AMP_DEBUG_CFG_H__ */
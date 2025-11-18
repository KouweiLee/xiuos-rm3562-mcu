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
#include "msg.h"

#ifndef __AMP_SERVICE_H__
#define __AMP_SERVICE_H__

/* 标识不指定特定服务 */
#define SERVICE_NONE_ID (0U)

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

struct AmpService /* 核间服务 */
{
    uint32_t service_id; /* 服务编号 */
    int (*service)(struct Msg *msg); /* 服务入口函数 */
};

extern struct AmpService services[]; /* 本节点所有的服务集合 */

struct AmpServiceOps
{
    /* 通过服务ID 获取服务 */
    struct AmpService* (*get_by_id)(uint32_t id);
};

extern struct AmpServiceOps amp_service_ops;
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
#endif /* __AMP_SERVICE_H__ */
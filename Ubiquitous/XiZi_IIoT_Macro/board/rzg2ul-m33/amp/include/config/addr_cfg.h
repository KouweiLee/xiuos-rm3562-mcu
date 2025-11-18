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

#include "types_cfg.h"

#ifndef __AMP_ADDR_CFG_H__
#define __AMP_ADDR_CFG_H__

/* 特定核心地址空间下的内存块地址信息，范围为：[start, start + len) */
struct AddrInfo
{
   uint64_t start; /* 内存块起始地址：兼容32/64位 */
   uint32_t len;   /* 内存块长度 */
};

/* 所有核间通信可能会用到的内存块地址信息 */
extern struct AddrInfo addr_infos[];

/* 为方便配置而起的别名 */
#define LINUX_SHM_TOTAL_ADDR_INFO (&addr_infos[0])
#define LINUX_SHM_USED_ADDR_INFO (&addr_infos[1])
#define XIUOS_SHM_TOTAL_ADDR_INFO (&addr_infos[2])
#define XIUOS_SHM_USED_ADDR_INFO (&addr_infos[3])
#define LINUX_2_LINUX_MSG_QUEUE_ADDR_INFO (&addr_infos[4])
#define LINUX_2_XIUOS_MSG_QUEUE_ADDR_INFO (&addr_infos[5])
#define XIUOS_2_LINUX_MSG_QUEUE_ADDR_INFO (&addr_infos[6])
#define XIUOS_2_XIUOS_MSG_QUEUE_ADDR_INFO (&addr_infos[7])
#endif /* __AMP_ADDR_CFG_H__ */
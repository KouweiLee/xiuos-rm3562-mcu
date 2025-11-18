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

#include "common_cfg.h"
#include "types_cfg.h"

#ifndef __AMP_CORE_CFG_H__
#define __AMP_CORE_CFG_H__

#define CORE_NONE_ID (0U) /*  不存在的空节点 */
#define CORE_NONE_NAME "None"

#define CORE_LINUX_ID (10U) /* Linux 节点 */
#define CORE_LINUX_NAME "Linux"

#define CORE_XIUOS_ID (20U) /* XiUOS 节点 */
#define CORE_XIUOS_NAME "XiUOS"

/* 参与核间通信的核心信息，只要参与了，就需要说明 */
struct CoreInfo
{
   uint32_t id;                /* 节点ID */
   char name[MAX_NAME_LENGTH]; /* 节点名字 */
};

/* 当前运行节点的节点信息：该信息随着允许平台的选择而变化 */
extern struct CoreInfo* core_info; 
/* 所有支持核间通信的节点信息 */
extern struct CoreInfo core_infos[];

/* 为方便配置而起的别名 */
#define LINUX_CORE_INFO (&core_infos[0])
#define XIUOS_CORE_INFO (&core_infos[1])
#endif /* __AMP_CORE_CFG_H__ */
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

#ifndef __AMP_COMMON_CFG_H__
#define __AMP_COMMON_CFG_H__

#define MAX_NAME_LENGTH (20U) /* 框架内可能存在的命名允许的最大长度 */

/* 内存大小单位相关宏定义 */
#define B  (1U)
#define KB ((B) << 10)
#define MB ((KB) << 10)
#define GB ((MB) << 10)

#define MEMORY_ALIGN_SIZE (4U) /* 结构体字节对齐配置，防止32/64位机器之间数据大小不一致 */

/* 初始化相关标记 */
#define INIT_MARK_INITIALIZED  (0xFFFFFFFFU) /* 已正确初始化 */
#define INIT_MARK_RAW_STATE    (0x00000000U) /* 未被初始化 */
#define INIT_MARK_INITIALIZING (0xAAAAAAAAU) /* 正在初始化 */
#define INIT_MARK_DESTORYED    (0xDDDDDDDDU) /* 初始化失败 */

#endif /* __AMP_COMMON_CFG_H__ */
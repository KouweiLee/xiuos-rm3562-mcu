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

#ifndef __AMP_PLATFORM_CFG_H__
#define __AMP_PLATFORM_CFG_H__

#define PLATFORM_NONE_BASE    (000U) /* 运行在裸机环境 */

#define PLATFORM_LINUX_BASE   (100U) /* 运行在Linux节点 */
#define PLATFORM_LINUX_KERNEL (101U) /* 运行在Linux内核空间 */
#define PLATFORM_LINUX_USER   (102U) /* 运行在Linux用户空间，内核空间和用户空间存在区别，但是Linux都是10开头 */

#define PLATFORM_RTOS_BASE    (200U) /* 运行在RTOS环境 */
#define PLATFORM_RTOS_XIUOS   (201U) /* 运行在XIUOS环境 */

#define PLATFORM_CHOICE (PLATFORM_RTOS_XIUOS) /* 运行环境选择 */

#if PLATFORM_CHOICE / PLATFORM_LINUX_BASE == 1 /* Linux平台 */
#define MEM_PAGE_SIZE (4096U)   /* Linux 内存页的大小，在内核中申请内存时使用该大小，方便将其映射到用户空间 */
#endif /* PLATFORM_CHOICE / PLATFORM_LINUX_BASE == 1  */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
#define MEM_DRIVER ("/dev/mem") /* 内存全映射驱动名称，用于 mmap，几乎所有的Linux系统都有该驱动 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */
#endif /* __AMP_PLATFORM_CFG_H__ */
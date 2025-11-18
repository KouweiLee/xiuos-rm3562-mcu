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

#ifndef __AMP_IPI_CFG_H__
#define __AMP_IPI_CFG_H__

/* 核间中断硬件选择：硬件不一样需要的驱动不一样 */
#define IPI_MHU (1U)    /* 使用MHU进行核间中断 */
#define IPI_IPCC (2U)   /* 使用IPCC进行核间中断 */

#define IPI_CHOICE (IPI_MHU) /* 核间中断硬件选择 */

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER
#define IPI_DRIVER_NAME ("/dev/amp_ipi_dev") /* 驱动名称 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
#define IPI_DEV_NAME ("amp_ipi_dev") /* 设备名称 */
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */

#endif /* __AMP_IPI_CFG_H__ */
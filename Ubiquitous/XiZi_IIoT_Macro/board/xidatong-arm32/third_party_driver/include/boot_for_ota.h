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
* @file boot_for_ota.h
* @brief support bootloader function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2023-11-20
*/

#ifndef __BOOT_FOR_OTA_H__
#define __BOOT_FOR_OTA_H__

#include "fsl_common.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "common.h"

void mcuboot_bord_init(void);
void mcuboot_reset(void);
void mcuboot_jump(void);
void mcuboot_delay(uint32_t ms);

#endif


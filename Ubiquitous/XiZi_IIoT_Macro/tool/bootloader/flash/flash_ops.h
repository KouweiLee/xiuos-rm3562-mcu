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
* @file flash_ops.h
* @brief support flash function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2023-04-03
*/
#ifndef __FLASH_OPS_H__
#define __FLASH_OPS_H__

#include "flash_for_ota.h"
typedef struct
{
    /* board init function*/
    void (*board_init)(void);
    void (*serial_init)(void);
    void (*print_string)(uint8_t *s);

    /* flash Driver operation */
    void (*flash_init)(void);
    void (*flash_deinit)(void);

    /* flash operation */
    status_t (*op_flash_erase)(uint32_t start_addr, uint32_t imageSize);
    status_t (*op_flash_write)(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite);
    status_t (*op_flash_read)(uint32_t addr, uint8_t *buf, uint32_t len);
    status_t (*op_flash_copy)(uint32_t srcAddr,uint32_t dstAddr, uint32_t imageSize);

    /* Burn the initialization version */
    int32_t (*download_by_serial)(const uint32_t addr);

    /* system operation */
    void(*op_reset)(void);
    void(*op_jump)(void);
    void(*op_delay)(uint32_t ms);
}mcuboot_t;

#endif
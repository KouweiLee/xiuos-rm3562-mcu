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
* @file flash_for_ota.h
* @brief support flash function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2023-11-20
*/

#ifndef __FLASH_FOR_OTA_H__
#define __FLASH_FOR_OTA_H__

#include <xs_base.h>
#include <stdint.h>
#include "fsl_romapi.h"
#include "flash.h"

#define USE_HIGHT_SPEED_TRANS  1

status_t Flash_Erase(uint32_t start_addr, uint32_t imageSize);
status_t Flash_Write(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite);
status_t Flash_Read(uint32_t addr, uint8_t *buf, uint32_t len);
status_t Flash_Copy(uint32_t srcAddr,uint32_t dstAddr, uint32_t imageSize);

#ifndef  USE_HIGHT_SPEED_TRANS
status_t NOR_FLASH_Write(uint32_t* FlashAddress, uint8_t* Data ,uint16_t DataLength);
#else
status_t NOR_FLASH_Write(uint32_t* FlashAddress, uint8_t* Data ,uint16_t DataLength,uint8_t doneFlag);
#endif

#endif


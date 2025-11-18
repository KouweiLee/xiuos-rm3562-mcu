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
* @file:    ymodem.c
* @brief:   file ymodem.c
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2023/3/24
*/
#include <xs_base.h>
#include "ymodem.h"
#include "string.h"
#include "flash_for_ota.h"
#include "boot_for_ota.h"

/*******************************************************************************
* 函 数 名: Ymodem_Receive
* 功能描述: 使用ymodem协议接收文件
* 形    参: buf:数据buffer
            addr:下载flash起始地址
            timeout:超时时间
* 返 回 值: 文件的大小
*******************************************************************************/
int32_t Ymodem_Receive(uint8_t *buf, const uint32_t addr)
{
  int32_t size = 0;
  return size;
}

/*******************************************************************************
* 函 数 名: SerialDownload
* 功能描述: 通过串口下载文件
* 形    参: addr:存储文件的flash起始地址
* 返 回 值: 文件的大小
*******************************************************************************/
int32_t SerialDownload(const uint32_t addr)
{
    int32_t Size = 1; //TODO: 208 does not support serial download
    return Size;
}

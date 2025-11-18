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
* @file:    ymodem.h
* @brief:   file ymodem.h
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2023/3/24
*/


#ifndef _YMODEM_H_
#define _YMODEM_H_

#include <stdint.h>

int32_t SerialDownload(const uint32_t addr);
int32_t Ymodem_Receive(uint8_t *buf, const uint32_t addr);

#endif

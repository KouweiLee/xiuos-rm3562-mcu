/*
 * Copyright (c) 2021 AIIT XUOS Lab
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
 * @file connect_ethernet.h
 * @brief Adapted network software protocol stack and hardware operation functions
 * @version 2.0
 * @author AIIT XUOS Lab
 * @date 2022-12-05
 */

#ifndef CONNECT_ETHERNET_H
#define CONNECT_ETHERNET_H

#include "ethernetif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NETIF_ENET0_INIT_FUNC ethernetif_init

int HwEthInit(void);
void *ethernetif_config_enet_set(uint8_t enet_port);
void Time_Update_LwIP(void);

#ifdef __cplusplus
}
#endif

#endif

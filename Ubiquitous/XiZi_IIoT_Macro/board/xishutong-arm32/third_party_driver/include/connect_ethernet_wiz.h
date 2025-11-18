/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _W5x00_LWIP_H_
#define _W5x00_LWIP_H_

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include "lwip/netif.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* LWIP */
#define ETHERNET_MTU 1500
#define SOCKET_MACRAW 0

int32_t send_lwip(uint8_t sn, uint8_t* buf, uint16_t len);

int32_t recv_lwip(uint8_t sn, uint8_t* buf, uint16_t len);

err_t netif_output(struct netif* netif, struct pbuf* p);

void netif_link_callback(struct netif* netif);

void netif_status_callback(struct netif* netif);

err_t wiz_netif_initialize(struct netif* netif);

static uint32_t ethernet_frame_crc(const uint8_t* data, int length);

void ethernetif_input2(void* netif_arg);
#define NETIF_ENET1_INIT_FUNC wiz_netif_initialize

#endif /* _W5x00_LWIP_H_ */

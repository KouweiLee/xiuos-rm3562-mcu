/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "connect_ethernet.h"

#include "socket.h"

#include "netif/etharp.h"
#include <sys.h>

#include <netdev.h>
#include <xizi.h>
#include <xs_kdbg.h>
/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
uint8_t wiz_mac[6] = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 };

static const uint32_t ethernet_polynomial_le = 0xedb88320U;

static sys_mutex_t wiz_trans_mtx;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

static inline void spi_if_clr(void)
{
    setSn_IR(0, 0x1F);
    setSIR(0);
}

void netif_link_callback(struct netif* netif)
{
    printf("netif link status changed %s\n", netif_is_link_up(netif) ? "up" : "down");
}

void netif_status_callback(struct netif* netif)
{
    printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

static int32_t wiz_transmit_pbuf(struct pbuf* p)
{
    static uint8_t addr[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    int32_t send_ret = 0;

    while (1) {
        spi_if_clr();
        if ((send_ret = wiz_sock_sendto(0, (uint8_t*)p->payload, p->len, addr, 0)) <= 0) {
            SYS_KDEBUG_LOG(WIZNET_DEBUG, ("[%s] data send failed: %d, sock: %x\n", __func__, send_ret, getSn_SR(0)));
            return -ERROR;
        }
        if (p->len == p->tot_len) {
            break;
        }

        p = p->next;
    }

    return EOK;
}

static struct pbuf* wiz_read_receive_pbuf(struct pbuf* buf)
{
#define RX_FRAME_SIZE 1542
    static uint8_t rx_frame[RX_FRAME_SIZE];
    static uint8_t addr[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    uint16_t port = 0;

    uint16_t lowlevel_len = getSn_RX_RSR(0);
    if (lowlevel_len <= 0) {
        return NULL;
    }

    int32_t data_len = wiz_sock_recvfrom(0, rx_frame, RX_FRAME_SIZE, addr, &port);
    if (data_len > 0 && data_len <= RX_FRAME_SIZE) {
        buf = pbuf_alloc(PBUF_RAW, data_len, PBUF_POOL);
        if (buf == NULL) {
            return NULL;
        }
        memcpy(buf->payload, rx_frame, data_len);
    } else {
        return NULL;
    }

    return buf;
}

void ethernetif_input2(void* netif_arg)
{
    struct netif* netif = (struct netif*)netif_arg;
    struct pbuf* p = NULL;
    for (;;) {
        sys_arch_sem_wait(get_eth_recv_sem2(), WAITING_FOREVER);
        sys_mutex_lock(&wiz_trans_mtx);
        while (1) {

            spi_if_clr();
            p = NULL;
            p = wiz_read_receive_pbuf(p);

            if (p != NULL) {
                SYS_KDEBUG_LOG(WIZNET_DEBUG, ("[%s:%d] Info Recved package with size %d\n", __func__, __LINE__, p->len));
                if (ERR_OK != netif->input(p, netif)) {
                    pbuf_free(p);
                }
                p = NULL;
            } else {
                break;
            }
        }
        sys_mutex_unlock(&wiz_trans_mtx);
    }
}

static err_t spi_if_linkoutput(struct netif* netif, struct pbuf* p)
{
    sys_mutex_lock(&wiz_trans_mtx);

    if (!(getSn_SR(0) & SOCK_MACRAW)) {
        SYS_KDEBUG_LOG(WIZNET_DEBUG, ("[%s:%d] err socket state %d\n", __func__, __LINE__, p->len));
        wiz_sock_close(0);
        setSn_MR(0, Sn_MR_MFEN | Sn_MR_MACRAW | Sn_MR_MIP6B | Sn_MR_MMB);
        wiz_socket(0, Sn_MR_MACRAW, 0, SOCK_IO_NONBLOCK);
    }

    int32_t ret = wiz_transmit_pbuf(p);

    sys_mutex_unlock(&wiz_trans_mtx);

    if (ret != EOK) {
        return ERR_USE;
    }

    /* TODO: Set up result value */
    return ERR_OK;
}

err_t wiz_netif_initialize(struct netif* netif)
{
    sys_mutex_new(&wiz_trans_mtx);

    netif->linkoutput = spi_if_linkoutput;
    netif->output = etharp_output;
    netif->mtu = ETHERNET_MTU;
    // netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    netif->flags |= NETIF_FLAG_LINK_UP;
    SMEMCPY(netif->hwaddr, wiz_mac, sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);
    netif->name[0] = 'w';
    netif->name[1] = 'z';

    if (EOK != lwip_netdev_add(netif)) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("LWIP add netdev failed.\n"));
    } else {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("Add Netdev successful\n"));
    }
    return ERR_OK;
}
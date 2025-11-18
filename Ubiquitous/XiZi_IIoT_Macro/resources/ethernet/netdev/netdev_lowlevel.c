/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-18     ChenYong     First version
 */

/**
* @file netdev_lowlevel.c
* @brief register low-level net dev function
* @version 3.0 
* @author AIIT XUOS Lab
* @date 2023-08-07
*/

/*************************************************
File name: netdev_lowlevel.c
Description: register low-level net dev function
Others: take RT-Thread v4.0.2/components/driver/serial/serial.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2023-08-07
Author: AIIT XUOS Lab
Modification: 
1. support low-level net dev set-ip, set-netmask, set-gw, set-status, set-link-status, set-dns-server and set-dhcp-status
*************************************************/

#include <netdev.h>
#include <string.h>
#include <xs_kdbg.h>

/**
 * This function will set network interface device IP address.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param ip_addr the new IP address
 */
void netdev_low_level_set_ipaddr(struct netdev* netdev, const ip_addr_t* ip_addr)
{
    CHECK(ip_addr);

    if (netdev && ip_addr_cmp((netdev->ip_addr), ip_addr) == 0) {
        ip_addr_copy(*netdev->ip_addr, *ip_addr);

        /* execute IP address change callback function */
        if (netdev->addr_callback) {
            netdev->addr_callback(netdev, NETDEV_CB_ADDR_IP);
        }
    }
}

/**
 * This function will set network interface device netmask address.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param netmask the new netmask address
 */
void netdev_low_level_set_netmask(struct netdev* netdev, const ip_addr_t* netmask)
{
    CHECK(netmask);

    if (netdev && ip_addr_cmp((netdev->netmask), netmask) == 0) {
        ip_addr_copy(*netdev->netmask, *netmask);

        /* execute netmask address change callback function */
        if (netdev->addr_callback) {
            netdev->addr_callback(netdev, NETDEV_CB_ADDR_NETMASK);
        }
    }
}

/**
 * This function will set network interface device gateway address.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param gw the new gateway address
 */
void netdev_low_level_set_gw(struct netdev* netdev, const ip_addr_t* gw)
{
    CHECK(gw);

    if (netdev && ip_addr_cmp((netdev->gw), gw) == 0) {
        ip_addr_copy(*netdev->gw, *gw);

        /* execute gateway address change callback function */
        if (netdev->addr_callback) {
            netdev->addr_callback(netdev, NETDEV_CB_ADDR_GATEWAY);
        }
    }
}

/**
 * This function will set network interface device DNS server address.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param dns_num the number of the DNS server
 * @param dns_server the new DNS server address
 *
 */
void netdev_low_level_set_dns_server(struct netdev* netdev, uint8_t dns_num, const ip_addr_t* dns_server)
{
    unsigned int index;

    CHECK(dns_server);

    if (netdev == NULL) {
        return;
    }
    /* check DNS servers is exist */
    for (index = 0; index < NETDEV_DNS_SERVERS_NUM; index++) {
        if (ip_addr_cmp(&(netdev->dns_servers[index]), dns_server)) {
            return;
        }
    }

    if (dns_num < NETDEV_DNS_SERVERS_NUM) {
        ip_addr_copy(netdev->dns_servers[dns_num], *dns_server);

        /* execute DNS servers address change callback function */
        if (netdev->addr_callback) {
            netdev->addr_callback(netdev, NETDEV_CB_ADDR_DNS_SERVER);
        }
    }
}

/* Change to the first link_up network interface device automatically */
static void netdev_auto_change_default(struct netdev* netdev)
{
    struct netdev* new_netdev = NULL;

    if (netdev->flags & NETDEV_FLAG_LINK_UP) {
        if (!(NETDEV_DEFAULT->flags & NETDEV_FLAG_LINK_UP)) {
            netdev_set_default(netdev);
        }
        return;
    }
    if (memcmp(netdev, NETDEV_DEFAULT, sizeof(struct netdev)) == 0) {
        new_netdev = netdev_get_first_by_flags(NETDEV_FLAG_LINK_UP);
        if (new_netdev) {
            netdev_set_default(new_netdev);
        }
    }
}

/**
 * This function will set network interface device status.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param is_up the new status
 */
void netdev_low_level_set_status(struct netdev* netdev, bool is_up)
{
    if (netdev && netdev_is_up(netdev) != is_up) {
        if (is_up) {
            netdev->flags |= NETDEV_FLAG_UP;
        } else {
            netdev->flags &= ~NETDEV_FLAG_UP;

            /* change to the first link_up network interface device automatically */
            netdev_auto_change_default(netdev);
        }

        /* execute  network interface device status change callback function */
        if (netdev->status_callback) {
            netdev->status_callback(netdev, is_up ? NETDEV_CB_STATUS_UP : NETDEV_CB_STATUS_DOWN);
        }
    }
}

/**
 * This function will set network interface device active link status.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param is_up the new link status
 */
void netdev_low_level_set_link_status(struct netdev* netdev, bool is_up)
{
    if (netdev && netdev_is_link_up(netdev) != is_up) {
        if (is_up) {
            netdev->flags |= NETDEV_FLAG_LINK_UP;
        } else {
            netdev->flags &= ~NETDEV_FLAG_LINK_UP;

            /* set network interface device flags to internet down */
            netdev->flags &= ~NETDEV_FLAG_INTERNET_UP;

            netdev_auto_change_default(netdev);
        }

        /* execute link status change callback function */
        if (netdev->status_callback) {
            netdev->status_callback(netdev, is_up ? NETDEV_CB_STATUS_LINK_UP : NETDEV_CB_STATUS_LINK_DOWN);
        }
    }
}

/**
 * This function will set network interface device active internet status.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param is_up the new internet status
 */
void netdev_low_level_set_internet_status(struct netdev* netdev, bool is_up)
{
    if (netdev && netdev_is_internet_up(netdev) != is_up) {
        if (is_up) {
            netdev->flags |= NETDEV_FLAG_INTERNET_UP;
        } else {
            netdev->flags &= ~NETDEV_FLAG_INTERNET_UP;
        }

        /* execute  network interface device status change callback function */
        if (netdev->status_callback) {
            netdev->status_callback(netdev, is_up ? NETDEV_CB_STATUS_INTERNET_UP : NETDEV_CB_STATUS_INTERNET_DOWN);
        }
    }
}

/**
 * This function will set network interface device DHCP status.
 * @NOTE it can only be called in the network interface device driver.
 *
 * @param netdev the network interface device to change
 * @param is_up the new DHCP status
 */
void netdev_low_level_set_dhcp_status(struct netdev* netdev, bool is_enable)
{
    if (netdev && netdev_is_dhcp_enabled(netdev) != is_enable) {
        if (is_enable) {
            netdev->flags |= NETDEV_FLAG_DHCP;
        } else {
            netdev->flags &= ~NETDEV_FLAG_DHCP;
        }

        /* execute DHCP status change callback function */
        if (netdev->status_callback) {
            netdev->status_callback(netdev, is_enable ? NETDEV_CB_STATUS_DHCP_ENABLE : NETDEV_CB_STATUS_DHCP_DISABLE);
        }
    }
}
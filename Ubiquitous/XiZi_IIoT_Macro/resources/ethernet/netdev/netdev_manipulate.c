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
* @file netdev_manipulate.c
* @brief register net dev function
* @version 3.0 
* @author AIIT XUOS Lab
* @date 2023-08-07
*/

/*************************************************
File name: netdev_manipulate.c
Description: register net dev function
Others: take RT-Thread v4.0.2/components/driver/serial/serial.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2023-08-07
Author: AIIT XUOS Lab
Modification: 
1. support net dev set-ip, set-netmask, set-gw, set-addr-callback, set-status-callback, set-up and set-down
*************************************************/

#include <inet.h>

#include <netdev.h>
#include <string.h>

#include <shell.h>
#include <xs_isr.h>
#include <xs_kdbg.h>

inline int netdev_check_set_addr_condition(struct netdev* netdev)
{
    if (NULL == netdev->ops || NULL == netdev->ops->set_addr_info) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The network interface device(%s) not support to set IP address.\n", netdev->name));
        return -ERROR;
    }

    // check whether dhcp enabled
    if (netdev_is_dhcp_enabled(netdev)) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The network interface device(%s) DHCP capability is enable, not support set IP address.\n", netdev->name));
        return -ERROR;
    }

    return EOK;
}

/**
 * This function will set network interface device IP address.
 *
 * @param netdev the network interface device to change
 * @param ip_addr the new IP address
 *
 * @return  0: set IP address successfully
 *         -1: set IP address failed
 */
int netdev_set_ipaddr(struct netdev* netdev, const ip_addr_t* ipaddr)
{
    CHECK(netdev);
    CHECK(ipaddr);

    if (EOK != netdev_check_set_addr_condition(netdev)) {
        return -ERROR;
    }

    return netdev->ops->set_addr_info(netdev, (ip_addr_t*)ipaddr, NULL, NULL);
}

/**
 * This function will set network interface device netmask address.
 *
 * @param netdev the network interface device to change
 * @param netmask the new netmask address
 *
 * @return  0: set netmask address successfully
 *         -1: set netmask address failed
 */
int netdev_set_netmask(struct netdev* netdev, const ip_addr_t* netmask)
{
    CHECK(netdev);
    CHECK(netmask);

    if (EOK != netdev_check_set_addr_condition(netdev)) {
        return -ERROR;
    }

    return netdev->ops->set_addr_info(netdev, NULL, (ip_addr_t*)netmask, NULL);
}

/**
 * This function will set network interface device gateway address.
 *
 * @param netdev the network interface device to change
 * @param gw the new gateway address
 *
 * @return  0: set gateway address successfully
 *         -1: set gateway address failed
 */
int netdev_set_gw(struct netdev* netdev, const ip_addr_t* gw)
{
    CHECK(netdev);
    CHECK(gw);

    if (EOK != netdev_check_set_addr_condition(netdev)) {
        return -ERROR;
    }

    /* execute network interface device set gateway address operations */
    return netdev->ops->set_addr_info(netdev, NULL, NULL, (ip_addr_t*)gw);
}

/**
 * This function will set network interface device DNS server address.
 *
 * @param netdev the network interface device to change
 * @param dns_num the number of the DNS server
 * @param dns_server the new DNS server address
 *
 * @return  0: set netmask address successfully
 *         -1: set netmask address failed
 */
int netdev_set_dns_server(struct netdev* netdev, uint8_t dns_num, const ip_addr_t* dns_server)
{
    CHECK(netdev);
    CHECK(dns_server);

    // check dns server number
    if (dns_num >= NETDEV_DNS_SERVERS_NUM) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The number of DNS servers(%d) set exceeds the maximum number(%d).\n", dns_num + 1, NETDEV_DNS_SERVERS_NUM));
        return -ERROR;
    }

    // check dns set function existence
    if (NULL == netdev->ops || NULL == netdev->ops->set_dns_server) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The network interface device(%s) not support to set DNS server address.\n", netdev->name));
        return -ERROR;
    }

    return netdev->ops->set_dns_server(netdev, dns_num, (ip_addr_t*)dns_server);
}

/**
 * This function will set callback to be called when the network interface device status has been changed.
 *
 * @param netdev the network interface device to change
 * @param status_callback the callback be called when the status has been changed.
 */
void netdev_set_status_callback(struct netdev* netdev, netdev_callback_fn status_callback)
{
    CHECK(netdev);
    CHECK(status_callback);

    netdev->status_callback = status_callback;
}

/**
 * This function will set callback to be called when the network interface device address has been changed.
 *
 * @param netdev the network interface device to change
 * @param addr_callback the callback be called when the address has been changed.
 */
void netdev_set_addr_callback(struct netdev* netdev, netdev_callback_fn addr_callback)
{
    CHECK(netdev);
    CHECK(addr_callback);

    netdev->addr_callback = addr_callback;
}

/**
 * This function will enable network interface device .
 *
 * @param netdev the network interface device to change
 *
 * @return  0: set status successfully
 *         -1: set status failed
 */
int netdev_set_up(struct netdev* netdev)
{
    CHECK(netdev);

    if (!netdev->ops || !netdev->ops->set_up) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The network interface device(%s) not support to set status.\n", netdev->name));
        return -ERROR;
    }

    /* network interface device status flags check */
    if (netdev_is_up(netdev)) {
        return EOK;
    }

    /* execute enable network interface device operations by network interface device driver */
    return netdev->ops->set_up(netdev);
}

/**
 * This function will disable network interface device.
 *
 * @param netdev the network interface device to change
 *
 * @return  0: set status successfully
 *         -1: set sttaus failed
 */
int netdev_set_down(struct netdev* netdev)
{
    CHECK(netdev);

    if (!netdev->ops || !netdev->ops->set_down) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("The network interface device(%s) not support to set status.\n", netdev->name));
        return -ERROR;
    }

    /* network interface device status flags check */
    if (!netdev_is_up(netdev)) {
        return EOK;
    }

    /* execute disable network interface device operations by network interface driver */
    return netdev->ops->set_down(netdev);
}

/**
 * This function will get the first network interface device
 * with the flags in network interface device list.
 *
 * @param flags the network interface device flags
 *
 * @return != NULL: network interface device object
 *            NULL: get failed
 */
struct netdev* netdev_get_first_by_flags(uint16_t flags)
{
    if (NETDEV_LISTHEAD == NULL) {
        return NULL;
    }

    // get netdev from list
    x_base lock = DISABLE_INTERRUPT();
    struct netdev* current_dev = NETDEV_LISTHEAD;
    SINGLE_LINKLIST_FOR_EACH_ENTRY(current_dev, &(NETDEV_LISTHEAD->list), list)
    {
        if (NULL != current_dev && 0 != (current_dev->flags & flags)) {
            ENABLE_INTERRUPT(lock);
            return current_dev;
        }
    }
    ENABLE_INTERRUPT(lock);

    return NULL;
}

/**
 * This function will get the first network interface device
 * in network interface device list by IP address.
 *
 * @param ip_addr the network interface device IP address
 *
 * @return != NULL: network interface device object
 *            NULL: get failed
 */
struct netdev* netdev_get_by_ipaddr(ip_addr_t* ip_addr)
{
    if (NETDEV_LISTHEAD == NULL) {
        return NULL;
    }

    // get netdev from list
    x_base lock = DISABLE_INTERRUPT();
    struct netdev* current_dev = NETDEV_LISTHEAD;
    SINGLE_LINKLIST_FOR_EACH_ENTRY(current_dev, &(NETDEV_LISTHEAD->list), list)
    {
        if (NULL != current_dev && ip_addr_cmp((current_dev->ip_addr), ip_addr)) {
            ENABLE_INTERRUPT(lock);
            return current_dev;
        }
    }
    ENABLE_INTERRUPT(lock);

    return NULL;
}

/**
 * This function will get network interface device
 * in network interface device list by netdev name.
 *
 * @param name the network interface device name
 *
 * @return != NULL: network interface device object
 *            NULL: get failed
 */
struct netdev* netdev_get_by_name(const char* name)
{
    if (NETDEV_LISTHEAD == NULL) {
        return NULL;
    }

    uint32_t search_name_len = strlen(name);
    SYS_KDEBUG_LOG(NETDEV_DEBUG, ("Netdev search name: %s, len: %d\n", name, search_name_len));
    // get netdev from list
    x_base lock = DISABLE_INTERRUPT();
    struct netdev* current_dev = NETDEV_LISTHEAD;
    SINGLE_LINKLIST_FOR_EACH_ENTRY(current_dev, &(NETDEV_LISTHEAD->list), list)
    {
        if (NULL == current_dev) {
            continue;
        }

        uint32_t name_len = strlen(current_dev->name);
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("Netdev current dev name: %s, len: %d\n", current_dev->name, name_len));
        if (NULL != current_dev && name_len == search_name_len && 0 == strncmp(current_dev->name, name, name_len)) {
            ENABLE_INTERRUPT(lock);
            return current_dev;
        }
    }
    ENABLE_INTERRUPT(lock);

    return NULL;
}

void netdev_list_dev()
{
    if (NETDEV_LISTHEAD == NULL) {
        return;
    }

    char ip[IP4ADDR_STRLEN_MAX], netmask[IP4ADDR_STRLEN_MAX], gw[IP4ADDR_STRLEN_MAX], dns[IP4ADDR_STRLEN_MAX];

    // get netdev from list
    x_base lock = DISABLE_INTERRUPT();
    struct netdev* current_dev = NETDEV_LISTHEAD;
    SINGLE_LINKLIST_FOR_EACH_ENTRY(current_dev, &(NETDEV_LISTHEAD->list), list)
    {
        if (NULL == current_dev) {
            continue;
        }

        strncpy(ip, inet_ntoa(*current_dev->ip_addr), IP4ADDR_STRLEN_MAX);
        strncpy(netmask, inet_ntoa(*current_dev->netmask), IP4ADDR_STRLEN_MAX);
        strncpy(gw, inet_ntoa(*current_dev->gw), IP4ADDR_STRLEN_MAX);
        strncpy(dns, inet_ntoa(current_dev->dns_servers[0]), IP4ADDR_STRLEN_MAX);
        KPrintf("Netdev %s: ip: %s, mask: %s, gw: %s, dns: %s\n",
            current_dev->name,
            ip, netmask, gw, dns);
    }
    ENABLE_INTERRUPT(lock);

    return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(5),
    netdev_list, netdev_list_dev, list sys netdev);
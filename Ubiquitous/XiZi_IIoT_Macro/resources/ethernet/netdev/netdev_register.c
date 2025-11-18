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
* @file netdev_register.c
* @brief register net dev function for net driver
* @version 3.0 
* @author AIIT XUOS Lab
* @date 2023-08-07
*/

/*************************************************
File name: netdev_register.c
Description: register net dev function for net driver
Others: take RT-Thread v4.0.2/components/driver/serial/serial.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History: 
1. Date: 2023-08-07
Author: AIIT XUOS Lab
Modification: 
1. support net dev register, unregister function
*************************************************/

#include <assert.h>
#include <def.h>
#include <netdev.h>
#include <netdev_ipaddr.h>
#include <string.h>
#include <xs_isr.h>
#include <xs_kdbg.h>

struct netdev** get_netdev_listhead()
{
    static bool init = false;
    static struct netdev* netdev_listhead;
    if (!init) {
        static struct netdev netdev_guard;
        strncpy(netdev_guard.name, "guard\0", 6);
        InitSingleLinkList(&(netdev_guard.list));
        netdev_listhead = &netdev_guard;
    }
    init = true;

    return &netdev_listhead;
}
struct netdev** get_default_netdev()
{
    static struct netdev* netdev_default = NULL;
    return &netdev_default;
}
static netdev_callback_fn g_netdev_register_callback = NULL;
static netdev_callback_fn g_netdev_default_change_callback = NULL;

int netdev_register(struct netdev* netdev, const char* name, void* user_data)
{
    CHECK(netdev != NULL);
    CHECK(name != NULL);

    // set flag mask, assert network is down
    uint16_t flag_mask = 0;
    flag_mask = NETDEV_FLAG_UP | NETDEV_FLAG_LINK_UP | NETDEV_FLAG_INTERNET_UP;
    netdev->flags &= ~flag_mask;

    // clear dev setting
    netdev->ip_addr = NULL;
    netdev->netmask = NULL;
    netdev->gw = NULL;

#if NETDEV_IPV6
    for (index = 0; index < NETDEV_IPV6_NUM_ADDRESSES; index++) {
        ip_addr_set_zero(&(netdev->ip6_addr[index]));
        IP_SET_TYPE_VAL(netdev->ip_addr, IPADDR_TYPE_V6);
    }
#endif /* NETDEV_IPV6 */

    // clear DNS servers
    for (uint16_t idx = 0; idx < NETDEV_DNS_SERVERS_NUM; idx++) {
        ip_addr_set_zero(&(netdev->dns_servers[idx]));
    }
    // clear callback fn
    netdev->addr_callback = NULL;
    netdev->status_callback = NULL;

    // validate name
    uint32_t name_len = strlen(name);
    if (name_len < NAME_NUM_MAX) {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("Register Netdev %s, name len: %d.\n", name, name_len));
        strncpy(netdev->name, name, (name_len > 31 ? 31 : name_len));
        netdev->name[name_len] = '\0';
    } else {
        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("[%s] name too long.\n", __func__));
        strncpy(netdev->name, name, NAME_NUM_MAX - 1);
        netdev->name[NAME_NUM_MAX - 1] = '\0';
    }

    netdev->user_data = user_data;

    InitSingleLinkList(&(netdev->list));

    // insert netdev to global list
    x_base lock = DISABLE_INTERRUPT();
    if (NETDEV_LISTHEAD == NULL) {
        NETDEV_LISTHEAD = netdev;

    } else {
        SingleLinkListNodeInsert(&(NETDEV_LISTHEAD->list), &(netdev->list));
    }
    ENABLE_INTERRUPT(lock);

    if (NETDEV_DEFAULT == NULL) {
        // set first met netdev to default netdev
        netdev_set_default(netdev);
    }

    if (g_netdev_register_callback) {
        g_netdev_register_callback(netdev, NETDEV_CB_REGISTER);
    }

    return EOK;
}

/**
 * This function will unregister network interface device and
 * delete it from network interface device list.
 *
 * @param netdev the network interface device object
 *
 * @return  0: unregistered successfully
 *         -1: unregistered failed
 */
int netdev_unregister(struct netdev* netdev)
{
    CHECK(netdev);

    if (NETDEV_LISTHEAD == NULL) {
        return -ERROR;
    }

    // remove netdev from netdev list
    x_base lock = DISABLE_INTERRUPT();
    struct netdev* current_dev = NETDEV_LISTHEAD;
    SINGLE_LINKLIST_FOR_EACH_ENTRY(current_dev, &(NETDEV_LISTHEAD->list), list)
    {
        // found netdev in list
        if (current_dev == netdev) {
            if (NETDEV_LISTHEAD == current_dev && NULL == SingleLinkListGetNextNode(&(current_dev->list))) {
                // netdev is the only one in list
                NETDEV_LISTHEAD = NULL;
            } else {
                SingleLinkListRmNode(&(NETDEV_LISTHEAD->list), &(current_dev->list));
            }

            // deal default netdev
            if (current_dev == NETDEV_DEFAULT) {
                NETDEV_DEFAULT = NULL;
            }
            break;
        }
    }
    ENABLE_INTERRUPT(lock);

    if (NETDEV_DEFAULT == NULL) {
        netdev_set_default(NETDEV_LISTHEAD);
    }

    // clean netdev if found one
    if (current_dev == netdev) {
        memset(netdev, 0, sizeof(*netdev));
    }

    return EOK;
}

/**
 * This function will set default network interface device.
 *
 * @param netdev the network interface device to change
 */
void netdev_set_default(struct netdev* netdev)
{
    if (netdev && (netdev != NETDEV_DEFAULT)) {
        NETDEV_DEFAULT = netdev;

        // set default function
        if (netdev->ops && netdev->ops->set_default) {
            netdev->ops->set_default(netdev);
        }

        // default change callback
        if (g_netdev_default_change_callback) {
            g_netdev_default_change_callback(netdev, NETDEV_CB_DEFAULT_CHANGE);
        }

        SYS_KDEBUG_LOG(NETDEV_DEBUG, ("Setting default network interface device name(%s) successfully.\n", netdev->name));
    }
}

/**
 * This function will set register callback
 *
 * @param register_callback the network register callback
 *
 */
void netdev_set_register_callback(netdev_callback_fn register_callback)
{
    g_netdev_register_callback = register_callback;
}

void netdev_set_default_change_callback(netdev_callback_fn default_change_cb)
{
    g_netdev_default_change_callback = default_change_cb;
}
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
 * @file lwip_config_demo.c
 * @brief Demo for ping function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.12.15
 */

#include <board.h>
#include <shell.h>
#include <string.h>
#include <sys.h>
#include <sys_arch.h>
#include <xs_kdbg.h>

#include "argparse.h"

#include "inet.h"
#include "netdev.h"

/******************************************************************************/
enum NETWORK_ACTIVE_E {
    LWIP_ACTIVE = 'e',
    LWIP_ACTIVE_ALL = 'a',
};

void LwipNetworkActive(int argc, char* argv[])
{
    static char usage_info[] = "select a eport to start.";
    static char program_info[] = "Active lwip network function.";
    static const char* const net_active_usages[] = {
        "LwipNetworkActive -e 0",
        "LwipNetworkActive -e 1",
        "LwipNetworkActive -a",
    };

    int eport = -1;
    bool all = false;
    bool is_help = false;
    struct argparse_option options[] = {
        OPT_HELP(&is_help),
        OPT_INTEGER(LWIP_ACTIVE, "eport", &eport, "eport to start network.", NULL, 0, 0),
        OPT_BIT(LWIP_ACTIVE_ALL, "all", &all, "start network at both eport 0 and 1.", NULL, true, 0),
        OPT_END(),
    };

    struct argparse argparse;
    argparse_init(&argparse, options, net_active_usages, 0);
    argparse_describe(&argparse, usage_info, program_info);
    argc = argparse_parse(&argparse, argc, (const char**)argv);
    if (is_help) {
        return;
    }

    if (all) {
        lwip_config_tcp(0, lwip_eth0_ipaddr, lwip_eth0_netmask, lwip_eth0_gwaddr);
        lwip_config_tcp(1, lwip_eth1_ipaddr, lwip_eth1_netmask, lwip_eth1_gwaddr);
        return;
    }

    if (eport != 0 && eport != 1) {
        printf("[%s] Err: Support only eport 0 and eport 1.\n", __func__);
    }

    if (eport == 0) {
        lwip_config_tcp(0, lwip_eth0_ipaddr, lwip_eth0_netmask, lwip_eth0_gwaddr);
    } else if (eport == 1) {
        lwip_config_tcp(1, lwip_eth1_ipaddr, lwip_eth1_netmask, lwip_eth1_gwaddr);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(5),
    LwipNetworkActive, LwipNetworkActive, start lwip);

enum NETWORK_CONFIG_E {
    NETWORK_DEV = 'd',
    NETWORK_IP = 'i',
    NETWORK_NM = 'n',
    NETWORK_GW = 'g',
};

void LwipSetNetwork(int argc, char* argv[])
{
    static char usage_info[] = "config network information.";
    static char program_info[] = "set network ip, netmask, gateway";
    static const char* const net_set_usages[] = {
        "LwipSetNetwork -d [dev] -i [ip]",
        "LwipSetNetwork -d [dev] -n [netmask]",
        "LwipSetNetwork -d [dev] -g [gw]",
        "LwipSetNetwork -d [dev] -i [ip] -n [netmask] -g [gw]",
    };

    char* dev_ptr = NULL;
    char* ip_ptr = NULL;
    char* nm_ptr = NULL;
    char* gw_ptr = NULL;
    bool is_help = false;
    struct argparse_option options[] = {
        OPT_HELP(&is_help),
        OPT_GROUP("Param Options"),
        OPT_STRING(NETWORK_DEV, "dev", &dev_ptr, "netdev", NULL, 0, 0),
        OPT_STRING(NETWORK_IP, "ip", &ip_ptr, "change ip", NULL, 0, 0),
        OPT_STRING(NETWORK_NM, "nm", &nm_ptr, "change netmask", NULL, 0, 0),
        OPT_STRING(NETWORK_GW, "gw", &gw_ptr, "change gateway", NULL, 0, 0),
        OPT_END(),
    };

    struct argparse argparse;
    argparse_init(&argparse, options, net_set_usages, 0);
    argparse_describe(&argparse, usage_info, program_info);
    argc = argparse_parse(&argparse, argc, (const char**)argv);
    /* help task */
    if (is_help) {
        return;
    }

    if (dev_ptr == NULL) {
        printf("[%s] Err: must given a netdev name.\n", __func__);
        return;
    }
    struct netdev* netdev = netdev_get_by_name(dev_ptr);
    if (netdev == NULL) {
        printf("[%s] Err: Netdev not found by name: %s\n", __func__, dev_ptr);
        return;
    }

    ip_addr_t ipaddr, maskaddr, gwaddr;
    if (ip_ptr != NULL) {
        inet_aton(ip_ptr, &ipaddr);
        if (0 != netdev_set_ipaddr(netdev, &ipaddr)) {
            printf("[%s] Err: set ip: %s failed.\n", __func__, ip_ptr);
        }
    }
    if (nm_ptr != NULL) {
        inet_aton(nm_ptr, &maskaddr);
        if (0 != netdev_set_netmask(netdev, &maskaddr)) {
            printf("[%s] Err: set netmask: %s failed.\n", __func__, nm_ptr);
        }
    }
    if (gw_ptr != NULL) {
        inet_aton(gw_ptr, &gwaddr);
        if (0 != netdev_set_gw(netdev, &gwaddr)) {
            printf("[%s] Err: set gateway: %s failed.\n", __func__, gw_ptr);
        }
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(11),
    LwipSetNetwork, LwipSetNetwork, config lwip);

void LwipShowNetwork(int argc, char* argv[])
{
    extern void netdev_list_dev();
    netdev_list_dev();
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
    LwipShowNetwork, LwipShowNetwork, show lwip network configuration);

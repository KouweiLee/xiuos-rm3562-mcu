
#ifndef __NETDEV_H__
#define __NETDEV_H__

#include <netdev_ipaddr.h>
#include <stdbool.h>
#include <xs_klist.h>
#include <xs_kdbg.h>
#include <xsconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NETDEV_DEBUG_PRINT_IP_INFO(ip, netmask, gw) \
        ("[%s %d]ip: %u.%u.%u.%u, netmask: %u.%u.%u.%u, gw: %u.%u.%u.%u\n", \
        __func__, __LINE__, \
        ((u8_t*)&ip)[0], ((u8_t*)&ip)[1], ((u8_t*)&ip)[2], ((u8_t*)&ip)[3], \
        ((u8_t*)&netmask)[0], ((u8_t*)&netmask)[1], ((u8_t*)&netmask)[2], ((u8_t*)&netmask)[3], \
        ((u8_t*)&gw)[0], ((u8_t*)&gw)[1], ((u8_t*)&gw)[2], ((u8_t*)&gw)[3])

/* the maximum of all used hardware address lengths */
#ifndef NETDEV_HWADDR_MAX_LEN
#define NETDEV_HWADDR_MAX_LEN 8U
#endif

/* the maximum of dns server number supported */
#ifndef NETDEV_DNS_SERVERS_NUM
#define NETDEV_DNS_SERVERS_NUM 2U
#endif

/* whether the network interface device is 'up' (set by the network interface driver or application) */
#define NETDEV_FLAG_UP 0x01U
/* if set, the network interface device has broadcast capability, only supported in the 'lwIP' stack */
#define NETDEV_FLAG_BROADCAST 0x02U
/* if set, the network interface device has an active link (set by the network interface driver) */
#define NETDEV_FLAG_LINK_UP 0x04U
/* if set, the network interface device is an ethernet device using ARP, only supported in the 'lwIP' stack */
#define NETDEV_FLAG_ETHARP 0x08U
/* if set, the network interface device is an ethernet device, only supported in the 'lwIP' stack */
#define NETDEV_FLAG_ETHERNET 0x10U
/* if set, the network interface device has IGMP capability, only supported in the 'lwIP' stack */
#define NETDEV_FLAG_IGMP 0x20U
/* if set, the network interface device has MLD6 capability, only supported in the 'lwIP' stack */
#define NETDEV_FLAG_MLD6 0x40U
/* if set, the network interface device connected to internet successfully (set by the network interface driver) */
#define NETDEV_FLAG_INTERNET_UP 0x80U
/* if set, the network interface device has DHCP capability (set by the network interface device driver or application) */
#define NETDEV_FLAG_DHCP 0x100U

enum netdev_cb_type {
    NETDEV_CB_ADDR_IP, /* IP address */
    NETDEV_CB_ADDR_NETMASK, /* subnet mask */
    NETDEV_CB_ADDR_GATEWAY, /* netmask */
    NETDEV_CB_ADDR_DNS_SERVER, /* dns server */
    NETDEV_CB_STATUS_UP, /* changed to 'up' */
    NETDEV_CB_STATUS_DOWN, /* changed to 'down' */
    NETDEV_CB_STATUS_LINK_UP, /* changed to 'link up' */
    NETDEV_CB_STATUS_LINK_DOWN, /* changed to 'link down' */
    NETDEV_CB_STATUS_INTERNET_UP, /* changed to 'internet up' */
    NETDEV_CB_STATUS_INTERNET_DOWN, /* changed to 'internet down' */
    NETDEV_CB_STATUS_DHCP_ENABLE, /* enable DHCP capability */
    NETDEV_CB_STATUS_DHCP_DISABLE, /* disable DHCP capability */
    NETDEV_CB_REGISTER, /* netdev register */
    NETDEV_CB_DEFAULT_CHANGE, /* netdev default change */
};

struct netdev;
typedef void (*netdev_callback_fn)(struct netdev* netdev, enum netdev_cb_type type);

struct netdev_ops {
    /* set network interface device hardware status operations */
    int (*set_up)(struct netdev* netdev);
    int (*set_down)(struct netdev* netdev);

    /* set network interface device address information operations */
    int (*set_addr_info)(struct netdev* netdev, ip_addr_t* ip_addr, ip_addr_t* netmask, ip_addr_t* gw);
    int (*set_dns_server)(struct netdev* netdev, uint8_t dns_num, ip_addr_t* dns_server);
    int (*set_dhcp)(struct netdev* netdev, bool is_enabled);

#ifdef RT_USING_FINSH
    /* set network interface device common network interface device operations */
    int (*ping)(struct netdev* netdev, const char* host, size_t data_len, uint32_t timeout, struct netdev_ping_resp* ping_resp);
    void (*netstat)(struct netdev* netdev);
#endif

    /* set default network interface device in current network stack*/
    int (*set_default)(struct netdev* netdev);
};

/* network interface device object */
struct netdev {
    SysSingleLinklistType list;

    char name[NAME_NUM_MAX]; /* network interface device name */
    ip_addr_t* ip_addr; /* IP address */
    ip_addr_t* netmask; /* subnet mask */
    ip_addr_t* gw; /* gateway */
#if NETDEV_IPV6
    ip_addr_t ip6_addr[NETDEV_IPV6_NUM_ADDRESSES]; /* array of IPv6 addresses */
#endif /* NETDEV_IPV6 */
    ip_addr_t dns_servers[NETDEV_DNS_SERVERS_NUM]; /* DNS server */
    uint8_t hwaddr_len; /* hardware address length */
    uint8_t hwaddr[NETDEV_HWADDR_MAX_LEN]; /* hardware address */

    uint16_t flags; /* network interface device status flag */
    uint16_t mtu; /* maximum transfer unit (in bytes) */
    const struct netdev_ops* ops; /* network interface device operations */

    netdev_callback_fn status_callback; /* network interface device flags change callback */
    netdev_callback_fn addr_callback; /* network interface device address information change callback */

#ifdef RT_USING_SAL
    void* sal_user_data; /* user-specific data for SAL */
#endif /* RT_USING_SAL */
    void* user_data; /* user-specific data */
};

// netdev global list
#define NETDEV_LISTHEAD (*get_netdev_listhead())
#define NETDEV_DEFAULT (*get_default_netdev())
extern struct netdev** get_netdev_listhead();
extern struct netdev** get_default_netdev();

// netdev register functions: netdev_register.c
int netdev_register(struct netdev* netdev, const char* name, void* user_data);
int netdev_unregister(struct netdev* netdev);
void netdev_set_default(struct netdev* netdev);
void netdev_set_register_callback(netdev_callback_fn status_callback);
void netdev_set_default_change_callback(netdev_callback_fn default_change_cb);

// netdev manipulate functions: netdev_manipulate.c
struct netdev* netdev_get_first_by_flags(uint16_t flags);
struct netdev* netdev_get_by_ipaddr(ip_addr_t* ip_addr);
struct netdev* netdev_get_by_name(const char* name);

int netdev_set_ipaddr(struct netdev* netdev, const ip_addr_t* ipaddr);
int netdev_set_netmask(struct netdev* netdev, const ip_addr_t* netmask);
int netdev_set_gw(struct netdev* netdev, const ip_addr_t* gw);
int netdev_set_dns_server(struct netdev* netdev, uint8_t dns_num, const ip_addr_t* dns_server);

void netdev_set_status_callback(struct netdev* netdev, netdev_callback_fn status_callback);
void netdev_set_addr_callback(struct netdev* netdev, netdev_callback_fn addr_callback);

/*  Set network interface device status */
int netdev_set_up(struct netdev* netdev);
int netdev_set_down(struct netdev* netdev);
int netdev_dhcp_enabled(struct netdev* netdev, bool is_enabled);

// low level functions which should only be called by net drivers
void netdev_low_level_set_ipaddr(struct netdev* netdev, const ip_addr_t* ipaddr);
void netdev_low_level_set_netmask(struct netdev* netdev, const ip_addr_t* netmask);
void netdev_low_level_set_gw(struct netdev* netdev, const ip_addr_t* gw);
void netdev_low_level_set_dns_server(struct netdev* netdev, uint8_t dns_num, const ip_addr_t* dns_server);
void netdev_low_level_set_status(struct netdev* netdev, bool is_up);
void netdev_low_level_set_link_status(struct netdev* netdev, bool is_up);
void netdev_low_level_set_internet_status(struct netdev* netdev, bool is_up);
void netdev_low_level_set_dhcp_status(struct netdev* netdev, bool is_enable);

static inline bool netdev_is_dhcp_enabled(struct netdev* netdev)
{
    return (netdev->flags & NETDEV_FLAG_DHCP) ? true : false;
}
static inline bool netdev_is_up(struct netdev* netdev)
{
    return (netdev->flags & NETDEV_FLAG_UP) ? true : false;
}
static inline bool netdev_is_internet_up(struct netdev* netdev)
{
    return (netdev->flags & NETDEV_FLAG_INTERNET_UP) ? true : false;
}
static inline bool netdev_is_link_up(struct netdev* netdev)
{
    return (netdev->flags & NETDEV_FLAG_LINK_UP) ? true : false;
}

#ifdef __cplusplus
}
#endif

#endif
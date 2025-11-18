
#ifndef LWIP_HDR_IP4_ADDR_H
#ifndef __NETDEV_IPADDR_H__
#define __NETDEV_IPADDR_H__

#include <def.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Currently using netdev ipv4
#define NETDEV_IPV4 1

/* IP address types for use in ip_addr_t.type member */
enum netdev_ip_addr_type {
    /** IPv4 */
    IPADDR_TYPE_V4 = 0U,
    /** IPv6 */
    IPADDR_TYPE_V6 = 6U,
    /** IPv4+IPv6 ("dual-stack") */
    IPADDR_TYPE_ANY = 46U
};

#if (NETDEV_IPV4 && NETDEV_IPV6) /* Both IPV4 and IPV6 */
// Todo
#elif NETDEV_IPV4 /* NETDEV_IPV4 */

#define IP_SET_TYPE_VAL(ipaddr, iptype)
#define IP_SET_TYPE(ipaddr, iptype)
#define IP_GET_TYPE(ipaddr) IPADDR_TYPE_V4

#define ip_addr_copy(dest, src) ip4_addr_copy(dest, src)
#define ip_addr_cmp(addr1, addr2) ip4_addr_cmp(addr1, addr2)
#define ip_addr_set(dest, src) ip4_addr_set(dest, src)
#define ip_addr_set_zero(ipaddr) ip4_addr_set_zero(ipaddr)
#define ip_addr_set_any(is_ipv6, ipaddr) ip4_addr_set_any(ipaddr)
#define ip_addr_isany_val(ipaddr) ip4_addr_isany_val(ipaddr)
#define ip_addr_isany(ipaddr) ip4_addr_isany(ipaddr)

/* directly map this to the lwip internal functions */
#define inet_addr(cp) netdev_ipaddr_addr(cp)
#define inet_aton(cp, addr) netdev_ip4addr_aton(cp, (ip4_addr_t*)addr)
#define inet_ntoa(addr) netdev_ip4addr_ntoa((const ip4_addr_t*)&(addr))
#define inet_ntoa_r(addr, buf, buflen) netdev_ip4addr_ntoa_r((const ip4_addr_t*)&(addr), buf, buflen)

#else /* NETDEV_IPV6 */
// Todo
#endif /* NTDEV_IPV4 && NTDEV_IPV6 */

#ifdef NETDEV_IPV4
/** IPv4 only: set the IP address given as an u32_t */
#define ip4_addr_set_u32(dest_ipaddr, src_u32) ((dest_ipaddr)->addr = (src_u32))
/** IPv4 only: get the IP address as an u32_t */
#define ip4_addr_get_u32(src_ipaddr) ((src_ipaddr)->addr)

/** 255.255.255.255 */
#define IPADDR_NONE ((uint32_t)0xffffffffUL)
/** 127.0.0.1 */
#define IPADDR_LOOPBACK ((uint32_t)0x7f000001UL)
/** 0.0.0.0 */
#define IPADDR_ANY ((uint32_t)0x00000000UL)
/** 255.255.255.255 */
#define IPADDR_BROADCAST ((uint32_t)0xffffffffUL)

#define IP4ADDR_STRLEN_MAX 16

#if !defined(in_addr_t) && !defined(IN_ADDR_T_DEFINED)
typedef uint32_t in_addr_t;
#endif

struct in_addr {
    in_addr_t s_addr;
};

typedef struct ip4_addr {
    uint32_t addr;
} ip4_addr_t;

typedef ip4_addr_t ip_addr_t;

/** Copy IP address - faster than ip4_addr_set: no NULL check */
static inline void ip4_addr_copy(ip4_addr_t dest_addr, const ip4_addr_t src_addr)
{
    dest_addr.addr = src_addr.addr;
}
static inline bool ip4_addr_cmp(const ip4_addr_t* left_addr, const ip4_addr_t* right_addr)
{
    return left_addr->addr == right_addr->addr;
}
/** Safely copy one IP address to another (src may be NULL) */
static inline void ip4_addr_set(ip4_addr_t* dest_addr, const ip4_addr_t* src_addr)
{
    dest_addr->addr = (src_addr == NULL) ? 0 : src_addr->addr;
}
/** Set complete address to zero */
static inline void ip4_addr_set_zero(ip4_addr_t* ip4_addr)
{
    ip4_addr->addr = 0;
}
/** Set address to IPADDR_ANY (no need for htonl()) */
static inline void ip4_addr_set_any(ip4_addr_t* ipaddr)
{
    ipaddr->addr = IPADDR_ANY;
}
static inline bool ip4_addr_isany_val(const ip4_addr_t ipaddr)
{
    return ipaddr.addr == IPADDR_ANY;
}
static inline bool ip4_addr_isany(const ip4_addr_t* ipaddr)
{
    return ipaddr == NULL || ip4_addr_isany_val(*ipaddr);
}

#endif

#ifdef __cplusplus
}
#endif

#endif
#endif
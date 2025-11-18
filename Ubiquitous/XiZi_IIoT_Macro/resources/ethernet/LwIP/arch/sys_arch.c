/*
 * Copyright (c) 2017 Simon Goldschmidt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */

/**
 * @file sys_arch.c
 * @brief In order to adapt to XiZi, some changes have been made to implement the LwIP interface.
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021-05-29
 */
#include "debug.h"

#include <lwip/arch.h>
#include <lwip/opt.h>

#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/sio.h"
#include "tcpip.h"
#include "tcpip_priv.h"
#include <lwip/debug.h>
#include <lwip/stats.h>
#include <lwip/sys.h>

#if !NO_SYS
#include "sys_arch.h"
#endif

#include <string.h>
#include <xs_assign.h>
#include <xs_ktask.h>
#include <xs_ktick.h>
#include <xs_msg.h>
#include <xs_mutex.h>
#include <xs_sem.h>

#include "board.h"
#include "connect_ethernet.h"
#include "ethernet.h"

char lwip_ipaddr[20] = { 192, 168, 130, 77 };
char lwip_netmask[20] = { 255, 255, 254, 0 };
char lwip_gwaddr[20] = { 192, 168, 130, 1 };

char lwip_eth0_ipaddr[20] = { 192, 168, 130, 77 };
char lwip_eth0_netmask[20] = { 255, 255, 254, 0 };
char lwip_eth0_gwaddr[20] = { 192, 168, 130, 1 };

char lwip_eth1_ipaddr[20] = { 192, 168, 131, 88 };
char lwip_eth1_netmask[20] = { 255, 255, 254, 0 };
char lwip_eth1_gwaddr[20] = { 192, 168, 131, 1 };

char lwip_flag = 0;

x_ticks_t lwip_sys_now;

#define SYS_THREAD_MAX 4

struct netif gnetif;
sys_sem_t* get_eth_recv_sem()
{
    static sys_sem_t g_recv_sem = 0;
    return &g_recv_sem;
}

struct netif gnetif2;
sys_sem_t* get_eth_recv_sem2()
{
    static sys_sem_t g_recv_sem = 0;
    return &g_recv_sem;
}

void sys_init(void)
{
    // do nothing
}

u32_t sys_jiffies(void)
{
    lwip_sys_now = CurrentTicksGain();
    return lwip_sys_now;
}

u32_t sys_now(void)
{
    lwip_sys_now = CurrentTicksGain();
    return CalculateTimeMsFromTick(lwip_sys_now);
}

sys_prot_t sys_arch_protect(void)
{
    return CriticalAreaLock();
}

void sys_arch_unprotect(sys_prot_t pval)
{
    CriticalAreaUnLock(pval);
}

#if !NO_SYS

err_t sys_sem_new(sys_sem_t* sem, u8_t count)
{
    *sem = KSemaphoreCreate((uint16)count);

#if SYS_STATS
    ++lwip_stats.sys.sem.used;
    if (lwip_stats.sys.sem.max < lwip_stats.sys.sem.used) {
        lwip_stats.sys.sem.max = lwip_stats.sys.sem.used;
    }
#endif /* SYS_STATS */

    if (*sem >= 0)
        return ERR_OK;
    else {
#if SYS_STATS
        ++lwip_stats.sys.sem.err;
#endif /* SYS_STATS */
        KPrintf("[sys_arch]:new sem fail!\n");
        return ERR_MEM;
    }
}

void sys_sem_free(sys_sem_t* sem)
{
#if SYS_STATS
    --lwip_stats.sys.sem.used;
#endif /* SYS_STATS */
    KSemaphoreDelete(*sem);
    *sem = SYS_SEM_NULL;
}

int sys_sem_valid(sys_sem_t* sem)
{
    return (*sem > SYS_SEM_NULL);
}

void sys_sem_set_invalid(sys_sem_t* sem)
{
    *sem = SYS_SEM_NULL;
}

u32_t sys_arch_sem_wait(sys_sem_t* sem, u32_t timeout)
{
    x_ticks_t start_tick = 0;
    int32 wait_time = 0;

    if (*sem == SYS_SEM_NULL)
        return SYS_ARCH_TIMEOUT;

    start_tick = CurrentTicksGain();

    if (0 == timeout)
        wait_time = WAITING_FOREVER;
    else
        wait_time = timeout;

    if (KSemaphoreObtain(*sem, wait_time) == EOK)
        return CalculateTimeMsFromTick(CurrentTicksGain() - start_tick);
    else
        return SYS_ARCH_TIMEOUT;
}

void sys_sem_signal(sys_sem_t* sem)
{
    if (KSemaphoreAbandon(*sem) != EOK)
        KPrintf("[sys_arch]:sem signal fail!\n");
}

err_t sys_mutex_new(sys_mutex_t* mutex)
{
    *mutex = KMutexCreate();
    if (*mutex > SYS_MRTEX_NULL)
        return ERR_OK;
    else {
        KPrintf("[sys_arch]:new mutex fail!\n");
        return ERR_MEM;
    }
}

void sys_mutex_free(sys_mutex_t* mutex)
{
    KMutexDelete(*mutex);
}

void sys_mutex_set_invalid(sys_mutex_t* mutex)
{
    *mutex = SYS_MRTEX_NULL;
}

void sys_mutex_lock(sys_mutex_t* mutex)
{
    KMutexObtain(*mutex, WAITING_FOREVER);
}

void sys_mutex_unlock(sys_mutex_t* mutex)
{
    KMutexAbandon(*mutex);
}

sys_thread_t sys_thread_new(const char* name, lwip_thread_fn function, void* arg, int stacksize, int prio)
{
    sys_thread_t handle = -1;
    handle = KTaskCreate(name,
        function,
        arg,
        (uint32)stacksize,
        (uint8)prio);
    if (handle >= 0) {
        StartupKTask(handle);
        lw_print("lw: [%s] create %s handle %x\n", __func__, name, handle);
        return handle;
    }
    lw_print("lw: [%s] create %s failed\n", __func__, name);
    return -ERROR;
}

err_t sys_mbox_new(sys_mbox_t* mbox, int size)
{
    *mbox = KCreateMsgQueue(sizeof(void*), size);

#if SYS_STATS
    ++lwip_stats.sys.mbox.used;
    if (lwip_stats.sys.mbox.max < lwip_stats.sys.mbox.used) {
        lwip_stats.sys.mbox.max = lwip_stats.sys.mbox.used;
    }
#endif /* SYS_STATS */
    if (*mbox < 0) {
        lw_error("lw: [%s] alloc %d mbox %p failed\n", __func__, size, mbox);
        return ERR_MEM;
    }

    // lw_print("lw: [%s] alloc %d mbox %p ok!\n", __func__, size, mbox);
    return ERR_OK;
}

void sys_mbox_free(sys_mbox_t* mbox)
{
    KDeleteMsgQueue(*mbox);
}

int sys_mbox_valid(sys_mbox_t* mbox)
{
    if (*mbox <= SYS_MBOX_NULL)
        return 0;
    else
        return 1;
}

void sys_mbox_set_invalid(sys_mbox_t* mbox)
{
    *mbox = SYS_MBOX_NULL;
}

void sys_mbox_post(sys_mbox_t* q, void* msg)
{
    KMsgQueueSendwait(*q, &msg, sizeof(void*), WAITING_FOREVER);
}

err_t sys_mbox_trypost(sys_mbox_t* q, void* msg)
{
    // if(KMsgQueueSend(*q, &msg, sizeof(void *)) == EOK)
    if (KMsgQueueSend(*q, &msg, sizeof(void*)) == EOK)
        return ERR_OK;
    else
        return ERR_MEM;
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t* q, void* msg)
{
    return sys_mbox_trypost(q, msg);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t* q, void** msg, u32_t timeout)
{
    x_ticks_t start_tick = 0;
    int32 wait_time = 0;

    start_tick = CurrentTicksGain();

    if (0 == timeout)
        wait_time = WAITING_FOREVER;
    else
        wait_time = timeout;

    if (KMsgQueueRecv(*q, &(*msg), sizeof(void*), wait_time) == EOK) {
        return CalculateTimeMsFromTick(CurrentTicksGain() - start_tick);
    } else {
        return SYS_ARCH_TIMEOUT;
    }
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t* q, void** msg)
{
    if (KMsgQueueRecv(*q, &(*msg), sizeof(void*), 0) == EOK)
        return ERR_OK;
    else
        return SYS_MBOX_EMPTY;
}

#if LWIP_NETCONN_SEM_PER_THREAD
#error LWIP_NETCONN_SEM_PER_THREAD==1 not supported
#endif /* LWIP_NETCONN_SEM_PER_THREAD */

#endif /* !NO_SYS */

/* Variables Initialization */
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

void lwip_config_input(int eport, struct netif* net)
{
    sys_thread_t th_id = 0;

    if (eport == 0) {
        th_id = sys_thread_new("eth_input", ethernetif_input, net, LWIP_TASK_STACK_SIZE, 30);
    } else if (eport == 1) {
#ifdef NETIF_ENET1_INIT_FUNC
        th_id = sys_thread_new("eth_input2", ethernetif_input2, net, LWIP_TASK_STACK_SIZE, 30);
#endif
    }
}

void lwip_config_tcp(uint8_t enet_port, char* ip, char* mask, char* gw)
{
    static char is_init_0 = 0;
    static char is_init_1 = 0;
    if (is_init_0 == 0 && is_init_1 == 0) {
        sys_sem_new(get_eth_recv_sem(), 0);
        sys_sem_new(get_eth_recv_sem2(), 0);

        if (chk_lwip_bit(LWIP_INIT_FLAG)) {
            lw_print("lw: [%s] already ...\n", __func__);
        }

        set_lwip_bit(LWIP_INIT_FLAG);

        tcpip_init(NULL, NULL);
    }

    lw_print("lw: [%s] start ...\n", __func__);

    char* eth_cfg = ethernetif_config_enet_set(enet_port);
    ip4_addr_t net_ipaddr, net_netmask, net_gw;
    IP4_ADDR(&net_ipaddr, ip[0], ip[1], ip[2], ip[3]);
    IP4_ADDR(&net_netmask, mask[0], mask[1], mask[2], mask[3]);
    IP4_ADDR(&net_gw, gw[0], gw[1], gw[2], gw[3]);

    if (0 == enet_port) {
        if (is_init_0 == 1) {
            return;
        }
#ifndef NETIF_ENET0_INIT_FUNC
        lw_print("Not Netif driver for Eport 0\n");
        return;
#endif
#ifdef NETIF_ENET0_INIT_FUNC
        lw_print("Add netif eport 0\n");
        netif_add(&gnetif, &net_ipaddr, &net_netmask, &net_gw, eth_cfg, NETIF_ENET0_INIT_FUNC,
            tcpip_input);

        netif_set_default(&gnetif);
        netif_set_up(&gnetif);

        lw_print("\r\n************************************************\r\n");
        lw_print(" Network Configuration\r\n");
        lw_print("************************************************\r\n");
        lw_print(" IPv4 Address   : %u.%u.%u.%u\r\n", ((u8_t*)&net_ipaddr)[0], ((u8_t*)&net_ipaddr)[1],
            ((u8_t*)&net_ipaddr)[2], ((u8_t*)&net_ipaddr)[3]);
        lw_print(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t*)&net_netmask)[0], ((u8_t*)&net_netmask)[1],
            ((u8_t*)&net_netmask)[2], ((u8_t*)&net_netmask)[3]);
        lw_print(" IPv4 Gateway   : %u.%u.%u.%u\r\n", ((u8_t*)&net_gw)[0], ((u8_t*)&net_gw)[1],
            ((u8_t*)&net_gw)[2], ((u8_t*)&net_gw)[3]);
        lw_print("************************************************\r\n");

        lwip_config_input(enet_port, &gnetif);
        is_init_0 = 1;
#endif

    } else if (1 == enet_port) {
        if (is_init_1 == 1) {
            return;
        }
#ifndef NETIF_ENET1_INIT_FUNC
        lw_print("Not Netif driver for Eport 1\n");
        return;
#endif
#ifdef NETIF_ENET1_INIT_FUNC
        lw_print("Add netif eport 1\n");
        netif_add(&gnetif2, &net_ipaddr, &net_netmask, &net_gw, eth_cfg, NETIF_ENET1_INIT_FUNC,
            tcpip_input);

        // netif_set_default(&gnetif2);
        netif_set_up(&gnetif2);

        lw_print("\r\n************************************************\r\n");
        lw_print(" Network Configuration\r\n");
        lw_print("************************************************\r\n");
        lw_print(" IPv4 Address   : %u.%u.%u.%u\r\n", ((u8_t*)&net_ipaddr)[0], ((u8_t*)&net_ipaddr)[1],
            ((u8_t*)&net_ipaddr)[2], ((u8_t*)&net_ipaddr)[3]);
        lw_print(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t*)&net_netmask)[0], ((u8_t*)&net_netmask)[1],
            ((u8_t*)&net_netmask)[2], ((u8_t*)&net_netmask)[3]);
        lw_print(" IPv4 Gateway   : %u.%u.%u.%u\r\n", ((u8_t*)&net_gw)[0], ((u8_t*)&net_gw)[1],
            ((u8_t*)&net_gw)[2], ((u8_t*)&net_gw)[3]);
        lw_print("************************************************\r\n");

        lwip_config_input(enet_port, &gnetif2);
        is_init_1 = 1;
#endif
    }
}
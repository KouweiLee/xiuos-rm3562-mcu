/*
* Copyright (c) 2022 AIIT XUOS Lab
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
* @file lwip_udp_demo.c
* @brief One UDP demo based on LwIP
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-21
*/
#include "board.h"
#include "sys_arch.h"
#include <sys.h>
#include <xizi.h>

#include <stdbool.h>
#include <unistd.h>

#include "lwip/sockets.h"
#include "lwip/udp.h"

#include <argparse.h>
#include <shell.h>

#define PBUF_SIZE 27

static struct udp_pcb *udpecho_raw_pcb;

u16_t udp_server_port = LWIP_TARGET_PORT;
#define UDP_BUFFER_SIZE 50
char hello_str[] = {"hello world\r\n"};
char udp_demo_buffer[UDP_BUFFER_SIZE] = { '\0' };

/******************************************************************************/
enum LwipUdpSendParamEnum {
    TARGET_IP = 0,
    TARGET_PORT = 'p',
    SEND_MESSAGE = 'm',
    SEND_NUM = 'n',
    SEND_INTERVAL = 'i',
};

struct LwipUdpSendParam {
    uint32_t num;
    uint32_t interval;
    uint16_t port;
    uint8_t ip[4];
    bool task_on;
    bool given_ip;
    bool given_port;
    bool given_msg;
};

struct LwipUdpSendParam* get_udp_test_info()
{
    /* init once and init when used. */
    static struct LwipUdpSendParam g_udp_send_param = {
        .interval = 100,
        .num = 10,
        .port = LWIP_TARGET_PORT,
        .ip = { 127, 0, 0, 1 },
        .task_on = false,
        .given_ip = false,
        .given_port = false,
        .given_msg = false,
    };
    return &g_udp_send_param;
}

static const char* const usages[] = {
    "UDPSend [--options arg] [-option arg]",
    NULL,
};

static void LwipUDPSendTask(void *arg)
{
    int cnt = LWIP_DEMO_TIMES;

    KPrintf("udp_send_demo start.\n");

    int socket_fd = -1;
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        KPrintf("Socket error\n");
        return;
    }

    struct sockaddr_in udp_sock;
    udp_sock.sin_family = AF_INET;
    udp_sock.sin_port = htons(get_udp_test_info()->port);
    udp_sock.sin_addr.s_addr = PP_HTONL(LWIP_MAKEU32(get_udp_test_info()->ip[0], get_udp_test_info()->ip[1], get_udp_test_info()->ip[2], get_udp_test_info()->ip[3]));
    memset(&(udp_sock.sin_zero), 0, sizeof(udp_sock.sin_zero));

    if (connect(socket_fd, (struct sockaddr *)&udp_sock, sizeof(struct sockaddr))) {
        KPrintf("Unable to connect\n");
        closesocket(socket_fd);
        return;
    }

    KPrintf("UDP connect success, start to send.\n");
    KPrintf("\n\nTarget Port:%d\n\n", udp_sock.sin_port);
    get_udp_test_info()->task_on = true;

    while (get_udp_test_info()->num > 0 || get_udp_test_info()->num == -1) {
        sendto(socket_fd, udp_demo_buffer, strlen(udp_demo_buffer), 0, (struct sockaddr*)&udp_sock, sizeof(struct sockaddr));
        KPrintf("Send UDP msg: %s \n", udp_demo_buffer);
        MdelayKTask(get_udp_test_info()->interval);
        get_udp_test_info()->num--;
    }
    closesocket(socket_fd);
    get_udp_test_info()->task_on = false;
    return;
}

static int LwipUdpSend(int argc, char* argv[])
{
    static char usage_info[] = "Send udp NUM message to IP:PORT with time INTERVAL between each message send.";
    static char program_info[] = "UDP SEND TEST DEMO.";

    /* Wait if there are former udp task */
    if (get_udp_test_info()->task_on) {
        KPrintf("[%s] Waiting former udp send task to exit.\n");
    }
    while (get_udp_test_info()->task_on) {
        MdelayKTask(1000);
    }

    get_udp_test_info()->given_ip = false;
    get_udp_test_info()->given_port = false;
    get_udp_test_info()->given_msg = false;

    /* Parse options */
    char* msg_ptr = NULL;
    char* ip_ptr = NULL;
    bool is_help = false;
    struct argparse_option options[] = {
        OPT_HELP(&is_help),
        OPT_STRING(SEND_MESSAGE, "message", &msg_ptr, "MESSAGE to send", NULL, 0, 0),
        OPT_STRING(TARGET_IP, "ip", &ip_ptr, "target IP to send upd messages", NULL, 0, 0),
        OPT_INTEGER(TARGET_PORT, "port", &get_udp_test_info()->port, "target PORT to send udp messages", NULL, 0, 0),
        OPT_INTEGER(SEND_NUM, "num", &get_udp_test_info()->num, "send NUM udp messages", NULL, 0, 0),
        OPT_INTEGER(SEND_INTERVAL, "interval", &get_udp_test_info()->interval, "time INTERVAL between messages", NULL, 0, 0),
        OPT_END(),
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usages, 0);
    argparse_describe(&argparse, usage_info, program_info);
    argc = argparse_parse(&argparse, argc, (const char**)argv);
    if (argc < 0) {
        KPrintf("Error options.\n");
        return -ERROR;
    }
    if (is_help) {
        return EOK;
    }

    // translate string to array
    sscanf(ip_ptr, "%hhd.%hhd.%hhd.%hhd", &get_udp_test_info()->ip[0], &get_udp_test_info()->ip[1], &get_udp_test_info()->ip[2], &get_udp_test_info()->ip[3]);
    int msg_len = strlen(msg_ptr);
    strncpy(udp_demo_buffer, msg_ptr, msg_len < UDP_BUFFER_SIZE ? msg_len : UDP_BUFFER_SIZE);

    /* start task */
    KPrintf("[%s] gw %hhd.%hhd.%hhd.%hhd:%d send time %d udp_interval %d\n", __func__,
        get_udp_test_info()->ip[0], get_udp_test_info()->ip[1], get_udp_test_info()->ip[2], get_udp_test_info()->ip[3],
        get_udp_test_info()->port,
        get_udp_test_info()->num,
        get_udp_test_info()->interval);

    lwip_config_net(0, lwip_ipaddr, lwip_netmask, lwip_gwaddr);
    sys_thread_new("udp send", LwipUDPSendTask, NULL, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
    return EOK;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(16),
    UDPSend, LwipUdpSend, UDPSend Demo);

void LwipUdpRecvTest(void)
{
    uint8_t enet_port = 0; ///< test enet port 0

    //init lwip and net dirver
    lwip_config_net(enet_port, lwip_ipaddr, lwip_netmask, lwip_gwaddr);

    uint8_t* recv_data = NULL;
    socklen_t sin_size;
    int sock = -1, connected, bytes_received, i;
    struct sockaddr_in server_addr, client_addr;
    fd_set readset;
    struct timeval timeout;

    sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        KPrintf("[%s:%d] Socket error!\n", __FILE__, __LINE__);
        goto __exit;
    }

    recv_data = (uint8_t *)malloc(128);
    if (recv_data == NULL) {
        KPrintf("No memory!\n");
        goto __exit;
    }

    //configure udp server param
    server_addr.sin_family = PF_INET;
    server_addr.sin_port = htons(udp_server_port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(&(server_addr.sin_zero), 0x0, sizeof(server_addr.sin_zero));

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
        KPrintf("Unable to bind!\n");
        goto __exit;
    }

    timeout.tv_sec = 30;
    timeout.tv_usec = 0;

    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        KPrintf("setsockopt failed!");
        goto __exit;
    }

    while (1) {
        bytes_received = recvfrom(sock, recv_data, 128, 0, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size);
        if (bytes_received == 0) {
            KPrintf("client disconnected (%s, %d)\n",
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            break;
        } else if (bytes_received < 0) {
            KPrintf("recv error, client: (%s, %d)\n",
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            break;
        } else {
            KPrintf("new client connected from (%s, %d)\n",
                inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            KPrintf("recv data length %d Bytes\n", bytes_received);
            for (i = 0; i < bytes_received; i ++) {
                KPrintf("data 0x%x\n", recv_data[i]);
            }
            if (i = bytes_received) {
                KPrintf("\r\n");
                memset(recv_data, 0, sizeof(recv_data));
            }
        }
    }

__exit:
    if (sock >= 0) closesocket(sock);
    if (recv_data) free(recv_data);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
     UDPRecv, LwipUdpRecvTest, UDP Receive echo);


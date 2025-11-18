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
* @file lwip_tcp_demo.c
* @brief TCP demo based on LwIP
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-21
*/

#include "board.h"
#include "sys_arch.h"
#include <shell.h>
#include <sys.h>
#include <xizi.h>
#include "lwip_demo.h"
#include "lwip/sockets.h"
#include "tcpecho_raw.h"
char tcp_demo_msg[LWIP_TEST_MSG_SIZE] = { 0 };
char tcp_server_ip[] = {192, 168, 130, 2};
u32_t tcp_server_port = 80;
int tcp_send_num = 0;
int tcp_send_task_on = 0;
uint32 tcp_interval = 50;

/******************************************************************************/

static void LwipTcpSendTask(void *arg)
{
    int fd = -1;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        KPrintf("Socket error\n");
        return;
    }

    LwipTcpSocketParamType *param = (LwipTcpSocketParamType *)arg;

    //configure tcp client param
    struct sockaddr_in tcp_sock;
    tcp_sock.sin_family = AF_INET;
    tcp_sock.sin_port = htons(param->port);
    tcp_sock.sin_addr.s_addr = PP_HTONL(LWIP_MAKEU32(param->ip[0], param->ip[1], param->ip[2], param->ip[3]));
    memset(&(tcp_sock.sin_zero), 0, sizeof(tcp_sock.sin_zero));

    if (connect(fd, (struct sockaddr *)&tcp_sock, sizeof(struct sockaddr)))
    {
        KPrintf("Unable to connect\n");
        closesocket(fd);
        return;
    }

    KPrintf("tcp connect success, start to send.\n");
    KPrintf("\n\nTarget Port:%d\n\n", tcp_sock.sin_port);
    tcp_send_task_on = 1;

    while(tcp_send_num > 0 || tcp_send_num == -1) {
        sendto(fd, tcp_demo_msg, strlen(tcp_demo_msg), 0, (struct sockaddr*)&tcp_sock, sizeof(struct sockaddr));
        KPrintf("Send tcp msg: %s \n", tcp_demo_msg);
        if(tcp_send_num > 0) {
            tcp_send_num--;
        }
        MdelayKTask(tcp_interval);
    }

    closesocket(fd);
    tcp_send_task_on = 0;
    return;
}

void LwipTcpSendTest(int argc, char *argv[])
{
    if(tcp_send_task_on) {
        tcp_send_num = 0;
        printf("waitting send task exit...\n");
        while(tcp_send_task_on) {
            MdelayKTask(1000);
        }
        tcp_send_num = 1;
    }
    LwipTcpSocketParamType param;
    uint8_t enet_port = 0;
    memset(tcp_demo_msg, 0, LWIP_TEST_MSG_SIZE);
    if(argc >= 2) {
        strncpy(tcp_demo_msg, argv[1], strlen(argv[1]));
    } else {
        strncpy(tcp_demo_msg, "hello world", strlen("hello world"));
        tcp_send_num = 10;//send 10 msg to server
        tcp_interval = 100;//100ms
    }
    strcat(tcp_demo_msg, "\r\n");

    if(argc >= 3) {
        if (sscanf(argv[2], "%hhd.%hhd.%hhd.%hhd:%hhd", &tcp_server_ip[0], &tcp_server_ip[1], &tcp_server_ip[2], &tcp_server_ip[3], &tcp_server_port) == EOK) {
            sscanf(argv[2], "%hhd.%hhd.%hhd.%hhd", &tcp_server_ip[0], &tcp_server_ip[1], &tcp_server_ip[2], &tcp_server_ip[3]);
        }
        sscanf(argv[3], "%d", &tcp_send_num);
        sscanf(argv[4], "%d", &tcp_interval);
    }
    KPrintf("connect ipaddr %hhd.%hhd.%hhd.%hhd:%hhd send msg %d times\n", tcp_server_ip[0], tcp_server_ip[1], tcp_server_ip[2], tcp_server_ip[3], tcp_server_port, tcp_send_num);
    lwip_config_tcp(enet_port, lwip_ipaddr, lwip_netmask, lwip_gwaddr);

    memcpy(param.ip, tcp_server_ip, 4);
    param.port = tcp_server_port;
    param.buf = malloc(LWIP_TEST_MSG_SIZE);
    memcpy(param.buf, tcp_demo_msg, LWIP_TEST_MSG_SIZE);

    sys_thread_new("tcp send", LwipTcpSendTask, &param, LWIP_TASK_STACK_SIZE, LWIP_DEMO_TASK_PRIO);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     TCPSend, LwipTcpSendTest, TCPSend msg [ip:port [num [interval]]]);

void LwipTcpRecvTest(void)
{
    uint8_t enet_port = 0; ///< test enet port 0

    lwip_config_tcp(enet_port, lwip_ipaddr, lwip_netmask, lwip_gwaddr);

    uint8_t* recv_data = NULL;
    socklen_t sin_size;
    int sock = -1, connected, bytes_received, i;
    struct sockaddr_in server_addr, client_addr;
    fd_set readset;
    struct timeval timeout;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        KPrintf("[%s:%d] Socket error!\n", __FILE__, __LINE__);
        goto __exit;
    }

    recv_data = (uint8_t *)malloc(128);
    if (recv_data == NULL) {
        KPrintf("No memory!\n");
        goto __exit;
    }

    //configure tcp server param
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(tcp_server_port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(&(server_addr.sin_zero), 0x0, sizeof(server_addr.sin_zero));

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
        KPrintf("Unable to bind!\n");
        goto __exit;
    }

    if (listen(sock, 5) == -1){
        KPrintf("Listen error!\n");
        goto __exit;
    }

    timeout.tv_sec = 30;
    timeout.tv_usec = 0;

    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        KPrintf("setsockopt failed!");
        goto __exit;
    }

    while (1) {
        FD_ZERO(&readset);
        FD_SET(sock, &readset);

        if (select(sock + 1, &readset, NULL, NULL, &timeout) == 0) {
            continue;
        }

        sin_size = sizeof(struct sockaddr_in);

        connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);

        while (1) {
            bytes_received = recv(connected, recv_data, 128, 0);
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

        if (connected >= 0) {
            closesocket(connected);
            connected = -1;
            break;
        }
    }

__exit:
    if (sock >= 0) closesocket(sock);
    if (recv_data) free(recv_data);
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
    TCPRecv, LwipTcpRecvTest, TCP Recv message);


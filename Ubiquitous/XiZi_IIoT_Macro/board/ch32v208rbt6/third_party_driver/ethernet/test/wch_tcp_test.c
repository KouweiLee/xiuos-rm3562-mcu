/**
 * @file wch_tcp_test.c
 * @brief  该文件所有函数无效，仅作TCP客户端连接参考
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-19
 */
#include "connect_ether.h"
#include "eth_driver.h"
#include "shell.h"
#include "wchnet.h"

/**
 * @brief  TCP客户端连接
 * @return int   暂无意义
 * @note 此函数已无效，仅作TCP客户端连接参考
 */
int Tcp_Client(void) {
    uint16_t desport = 1000;  // destination port
    uint16_t srcport = 1000;  // source port
    uint8_t SocketId;
    uint8_t i;
    uint8_t DESIP[4] = {192, 168, 1, 100};  // destination IP
    // for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++)
    WCHNET_CreateTcpSocket(DESIP, desport, 1000, &SocketId);  // Create TCP Socket

    while (1) {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if (WCHNET_QueryGlobalInt()) {
            WCHNET_HandleGlobalInt(NULL);
        }
    }
}

/**
 * @brief  测试TCP客户端连接
 * @param  argc  暂无意义
 * @param  argv  暂无意义
 * @return int   暂无意义
 * @note 此函数已无效
 */
int test_tcp_client(int argc, char* argv[]) {
    KPrintf("TCPClient Test\r\n");
    Tcp_Client();
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), test_tcp_client, test_tcp_client, test tcp client);
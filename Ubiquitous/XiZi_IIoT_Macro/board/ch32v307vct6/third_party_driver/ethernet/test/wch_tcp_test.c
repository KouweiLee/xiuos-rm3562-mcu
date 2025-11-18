#include "connect_ether.h"
#include "eth_driver.h"
#include "shell.h"
#include "wchnet.h"
#include "xs_base.h"

uint8_t DESIP[4] = { 192, 168, 1, 100 }; // destination IP address
uint16_t desport = 1000; // destination port
uint16_t srcport = 1000; // source port

uint8_t SocketId;

/*********************************************************************
 * @fn      TCP client
 *
 *
 * @return  none
 */
int Tcp_Client(void)
{
    uint8_t i;
    for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++)
        WCHNET_CreateTcpSocket(DESIP, srcport, desport, &SocketId); // Create TCP Socket

    while (1) {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if (WCHNET_QueryGlobalInt()) {
            if (WCHNET_HandleGlobalInt() == TIME_OUT) {
                WCHNET_CreateTcpSocket(DESIP, srcport, desport, &SocketId);
            }
        }
    }
}

int test_tcp_client(int argc, char* argv[])
{
    KPrintf("TCPClient Test\r\n");
    Tcp_Client();
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    test_tcp_client, test_tcp_client, test tcp client);
#include "ch32v30x.h"
#include "connect_uart.h"

extern RxBuffer2;
extern uint16_t UART_ReceiveData(USART_TypeDef* USARTx);
extern void UART_SendData(USART_TypeDef* USARTx, uint16_t Data);

void send_rensa()
{
    char* s = "ch uart";
    while (*s) {
        UART_SendData(USART3, (uint16_t)*s++);
        KPrintf("%c", *s);
    }
}

void recv_rensa()
{
    uint16_t ans = 0;
    char recv_data = ' ';

    int cnt = 0;
    while (recv_data != '\n') {
        recv_data = (char)UART_ReceiveData(USART3);
        KPrintf("%c", recv_data);
    }
    recv_data = ' ';
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    recv_rensa, recv_rensa, test receive renesa);

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    send_rensa, send_rensa, test send renesa);
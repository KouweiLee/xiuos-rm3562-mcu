/*
 * Copyright (c) 2020 AIIT XUOS Lab
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
 * @file rs485_test.c
 * @brief test ch32v307 485
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-03-14
 */
#include "shell.h"
#include "ch32v20x.h"
#include "ch32v20x_usart.h"
#include "connect_uart.h"

// RS485发送数据
static void RS485_SendData(uint8_t data)
{
    // 等待发送缓冲区为空
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
    {
    }

    // 发送数据
    USART_SendData(USART3, data);
}

// RS485接收数据
static uint16_t RS485_ReceiveData(void)
{
    // 等待接收缓冲区非空
    while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
    {
    }

    // 读取接收数据
    return USART_ReceiveData(USART3);
}

static void RS485_SendString(const char *buffer)
{
    size_t len = strlen(buffer);
    for (size_t i = 0; i < len; i++)
    {
        if (buffer[i] != '\r' || buffer[i] != '\n')
        {
            RS485_SendData(buffer[i]);
        }
    }
}

static void RS485_ReceiveString(char *buffer, size_t bufferSize)
{
    size_t i = 0;

    while (i < bufferSize - 1)
    {
        char receivedChar = RS485_ReceiveData() & 0xFF;

        if (receivedChar == '\n')
        {
            // 收到回车或换行符，表示字符串接收完毕
            break;
        }

        buffer[i] = receivedChar;
        i++;
    }

    buffer[i] = '\0'; // 在字符串末尾添加空字符，表示字符串结束

    while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
    {
        char dummy = USART_ReceiveData(USART3);
        (void)dummy; // 避免编译器警告
    }
}

int test_rs485(int argc, char *argv[])
{
    KPrintf("485 Test\r\n");
    char receiveBuffer[100];
    char *sendString = "hello aiit 485\r\n";
    int n = 10;
    RS485_SendString(sendString);
    while (n--)
    {
        // 接收字符串
        RS485_ReceiveString(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("%s\n", receiveBuffer);
        if (receiveBuffer[0] == 0xfd)
        {
            RS485_SendString(sendString);
        }
        else
        {
            USART_ClearFlag(USART3, USART_FLAG_TC);
            RS485_SendString(receiveBuffer);
        }
    }

    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 test_rs485, test_rs485, test rs485);
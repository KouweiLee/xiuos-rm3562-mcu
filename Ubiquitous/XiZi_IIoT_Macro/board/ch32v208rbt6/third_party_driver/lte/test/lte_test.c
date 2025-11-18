/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 * PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the
 * Mulan PSL v2 for more details.
 */

/**
 * @file lte_test.c
 * @brief test ch32v208 lte
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-03-14
 */
#include "ch32v20x.h"
#include "ch32v20x_usart.h"
#include "connect_uart.h"
#include "debug.h"
#include "string.h"

#define TRANSPARENT_TRANSMISSION_MODE 0
#define COMMAND_LINE_MODE 1
#define WORK_CONFIGURATION_MODE 0
#define WORK_COMMUNICATION_MODE 1
#define SERVER_IP_ADDRESS "115.238.53.59"
#define SERVER_PORT "10001"

int testLTE = 0;
char receiveBuffer[100];                    // 接收缓冲区
int prevChar = '\0';                        // 接收到的前一个字符
int lteMode = COMMAND_LINE_MODE;            // 传输模式
int lteWorkMode = WORK_CONFIGURATION_MODE;  // 工作模式
char *lteStartMessage = "WH-GM800TF";       // LTE模块启动时的提示信息

// LTE发送数据
static void lteSendData(uint8_t data) {
    // 等待发送缓冲区为空
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    }

    // 发送数据
    USART_SendData(USART2, data);
}

// LTE接收数据
uint16_t lteReceiveData(void) {
    // 等待接收缓冲区非空
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
        if (testLTE == 0) {
            return 0;
        }
    }
    // 读取接收数据
    return USART_ReceiveData(USART2);
}

static void lteSendString(const char *buffer) {
    size_t len = strlen(buffer);
    for (size_t i = 0; i < len; i++) {
        // 发送字符
        lteSendData(buffer[i]);
    }
}

static void lteReceiveString(char *buffer, size_t bufferSize) {
    size_t i = 0;

    while (i < bufferSize - 1) {
        char receivedChar = lteReceiveData() & 0xFF;
        if (receivedChar == '\n') {
            // 收到回车或换行符，表示字符串接收完毕
            break;
        }

        buffer[i] = receivedChar;
        i++;
    }

    buffer[i] = '\0';  // 在字符串末尾添加空字符，表示字符串结束

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
        char dummy = USART_ReceiveData(USART2);
        (void)dummy;  // 避免编译器警告
    }
}

/**
 * @brief  获取LTE响应内容
 * @note
 * 指令的响应信息分为有回显和无回显两种。回显的含义是在输入指令的时候，模块会先把输入的内容返回来，然后再对该指令做出响应。无回显则是模块不会返回输入的内容，只对指令做出响应。在以下说明中，均以无回显模式为例。
 * @note 命令串：[CR][LF][+CMD][OP][PARA][CR][LF]<CR><LF>[OK]<CR><LF>
 * @param  buffer
 * @param  bufferSize
 */
static void lteReceiveResponse(char *buffer, size_t bufferSize) {
    int count = 2;                                   // 出现[CR][LF]2次代表接收响应结束
    while (--bufferSize) {                           // 最多能够存储bufferSize-1个字符（最后要存储'\0'）
        char receiveChar = lteReceiveData() & 0xFF;  // 接收一个字符
        if (receiveChar == '\0' || receiveChar == '\n' && prevChar == '\r' && --count == 0) {  // 出现'\0'也代表接收结束
            break;
        } else if (receiveChar != '\r' && receiveChar != '\n') {  // 接收的字符存入缓冲区，不包含'\r'或者'\n'
            *buffer++ = receiveChar;
        }
        prevChar = receiveChar;
    }
    *buffer = '\0';
}

/**
 * @brief  指令回显功能开启或关闭
 * @param  sta 1：开启，0：关闭
 * @return int 0：开启或关闭成功，1：开启或关闭失败
 * @note AT+E=<sta>{CR}{LF}
 * @note {CR}{LF}OK{CR}{LF}
 */
int lteEnableEchoOrDisableEcho(int sta) {
    lteSendString(sta ? "AT+E=ON\r\n" : "AT+E=OFF\r\n");
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "OK") == 0) {
        return 0;
    } else {
        KPrintf("%s echo failed\n", sta ? "enable" : "disable");
        return 1;
    }
}

/**
 * @brief  设置串口打包时间
 * @param  time  串口打包时间，单位为ms
 * @return int   0：设置成功，1：设置失败
 * @note AT+UARTFT=<time>{CR}{LF}
 */
int lteSetUartFt(int time) {
    char temp[100];
    sprintf(temp, "AT+UARTFT=%d\r\n", time);
    lteSendString(temp);
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "OK") == 0) {
        return 0;
    } else {
        KPrintf("uartft set failed\n");
        return 1;
    }
}

/**
 * @brief  设置启动信息
 * @param  msg  启动信息字符串
 * @return int  0：设置成功，1：设置失败
 */
int lteSetStartMessage(char *msg) {
    char temp[100];
    sprintf(temp, "AT+STMSG=%s\r\n", msg);
    lteSendString(temp);
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "OK") == 0) {
        return 0;
    } else {
        KPrintf("start message set failed\n");
        return 1;
    }
}

/**
 * @brief  设置socketA参数以及设置socketA使能
 * @param  ip    服务器IP地址
 * @param  port  服务器端口号
 * @return int   0：设置参数并使能成功，1：设置参数并使能失败
 * @note 设置socketA参数：AT+SOCKA=<protocol>,<address>,<port>{CR}{LF}
 * @note 设置socketA使能：AT+SOCKAEN=<sta>{CR}{LF}
 */
static int lteSetServerParamAndEnableSocketA(char *ip, char *port) {
    char command[100];
    sprintf(command, "AT+SOCKA=TCP,%s,%s\r\n", ip, port);  // 设置socketA参数
    lteSendString(command);
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "OK") == 0) {  // 设置socketA参数成功
        lteSendString("AT+SOCKAEN=ON\r\n");  // 设置socketA使能
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        if (strcmp(receiveBuffer, "OK") == 0) {  // 设置socketA使能成功
            return 0;
        } else {  // 设置socketA使能失败
            KPrintf("enable socket A failed\n");
            return 1;
        }
    } else {  // 设置socketA参数失败
        KPrintf("server ip address and port set failed\n");
        return 1;
    }
}

/**
 * @brief  查询 socket A 连接状态
 * @return int   1：4G模块已和远程服务器连接成功，0:4G模块未与远程服务器连接
 * @note AT+SOCKALK?{CR}{LF}
 */
static int lteQuerySocketAConnectStatus(void) {
    lteSendString("AT+SOCKALK?\r\n");
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "+SOCKALK:Connected") == 0) {
        return 1;
    } else {
        return 0;
    }
}
/**
 * @brief  从网络透传切换至指令模式
 * @note 1.串口设备给模块连续发送“+++”，模块收到“+++”后，会给设备发送一个‘a’。
 * @note 2.在发送“+++”之前的一个串口打包间隔时间内不可发送任何数据。
 * @note 3.当设备接收‘a’后，必须在 3 秒内给模块发送一个‘a’。
 * @note 4.模块在接收到‘a’后，给设备发送“+ok”，并进入“临时指令模式”。
 * @note 5.设备接收到“+ok”后，知道模块已进入“临时指令模式”，可以向其发送 AT
 * 指令。
 */
void lteEnterConfigMode(void) {
    // 发送"+++"字符串
    lteSendString("+++");
    // 等待接收到"a"字符串
    int receivedChar;
    while ((receivedChar = lteReceiveData() & 0xFF) != 'a' && receivedChar != '\r') {
        KPrintf("%c", receivedChar);
    }
    if (receivedChar == 'a') {  // 之前仍未进入配置模式
        lteSendString("a");
        lteReceiveString(receiveBuffer, sizeof(receiveBuffer));
        if (strcmp(receiveBuffer, "+ok") == 0 || strcmp(receiveBuffer, "+ok\r") == 0) {
            lteWorkMode = WORK_CONFIGURATION_MODE;  // 设置工作模式为配置模式
        } else {
            KPrintf("enter LTE config mode failed, receive response:");
            int i = 0;
            while (receiveBuffer[i]) {
                KPrintf("%c", receiveBuffer[i++]);
            }
            KPrintf("\n");
        }
    } else if (receivedChar == '\r') {  // 之前已进入配置模式
        prevChar = receivedChar;
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    } else {  // 接收未知字符
        KPrintf("received unexpected char: %c\n", receivedChar);
    }
}

/**
 * @brief  保存参数并且重新启动
 * @return int 0：保存参数并重启成功，1：保存参数并重启失败
 * @note AT+S{CR}{LF}
 */
int lteSaveConfigAndRestart(void) {
    lteSendString("AT+S\r\n");  // 重新启动
    lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
    if (strcmp(receiveBuffer, "OK") == 0) {
        KPrintf("LTE init and restarting...\n");
        lteReceiveString(receiveBuffer, sizeof(receiveBuffer));
        char temp[100];
        sprintf(temp, "%s\r", lteStartMessage);
        if (strcmp(receiveBuffer, temp) == 0) {  // 若接收到启动信息，则表示重启成功
            return 0;
        } else {
            KPrintf("LTE init and restart failed, receive response:");
            for (int i = 0; i < 100; i++) {
                if (receiveBuffer[i] == '\0') {
                    break;
                }
                KPrintf("%d ", receiveBuffer[i]);
            }
        }
    } else {
        KPrintf("LTE init and restart failed, receive response:");
        for (int i = 0; i < 100; i++) {
            if (receiveBuffer[i] == '\0') {
                break;
            }
            KPrintf("%d ", receiveBuffer[i]);
        }
    }
    return 1;
}

/**
 * @brief  初始化4G模块
 * @note 关闭回显，保存参数并重新启动
 */
void lteInit(void) {
    lteEnterConfigMode();                                               // 进入配置模式
    lteEnableEchoOrDisableEcho(0);                                      // 关闭回显
    lteSetStartMessage(lteStartMessage);                                // 设置启动信息
    lteSetServerParamAndEnableSocketA(SERVER_IP_ADDRESS, SERVER_PORT);  // 设置服务器地址和端口，使能SocketA
    lteSendString("AT+S\r\n");                                          // 重新启动
    if (lteSaveConfigAndRestart() == 0) {                               // 设置参数并重启成功
        KPrintf("LTE init and restart success\n");
        lteEnterConfigMode();
        KPrintf("*-*-*-*-*-*-*-*-*-*-*lte  configuration*-*-*-*-*-*-*-*-*-*-*-*-*\n");
        /* 查询启动信息 */
        lteSendString("AT+STMSG\r\n");
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("start message:\t%s\n", receiveBuffer);
        /* 查询回显模式 */
        lteSendString("AT+E\r\n");
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("echo mode:\t%s\n", receiveBuffer);
        /* 查询工作模式 */
        lteSendString("AT+WKMOD\r\n");
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("current mode:\t%s\n", receiveBuffer);
        /* 查询串口参数 */
        lteSendString("AT+UART?\r\n");
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("uart config:\t%s\n", receiveBuffer);
        /* 查询socketA参数 */
        lteSendString("AT+SOCKA?\r\n");
        lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
        KPrintf("socketA config:\t%s\n", receiveBuffer);
        /* 查询socketA连接状态，查询10次直至连接成功，否则视为连接失败 */
        for (int i = 0; i < 10; i++) {
            lteSendString("AT+SOCKALK?\r\n");
            lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
            if (strcmp(receiveBuffer, "+SOCKALK:Connected") == 0) {
                break;
            }
            Delay_Ms((i + 1) * 1000);
        }
        KPrintf("socketA status:\t%s\n", receiveBuffer);

        if (lteQuerySocketAConnectStatus() == 1) {  // 已和服务器建立连接，进入通信模式
            lteSendString("AT+ENTM\r\n");
            lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
            if (strcmp(receiveBuffer, "OK") == 0) {
                lteWorkMode = WORK_COMMUNICATION_MODE;
                KPrintf("*-*-*-*-*-*-*-*-*-*-*communication mode*-*-*-*-*-*-*-*-*-*-*-*-*\n");
            } else {
                lteWorkMode = WORK_CONFIGURATION_MODE;
                KPrintf("*-*-*-*-*-*-*-*-*-*-*configuration mode*-*-*-*-*-*-*-*-*-*-*-*-*\n# ");
            }
        } else {  // 未与服务器建立连接，保持配置模式
            lteWorkMode = WORK_CONFIGURATION_MODE;
            KPrintf("*-*-*-*-*-*-*-*-*-*-*configuration mode*-*-*-*-*-*-*-*-*-*-*-*-*\n# ");
        }
    } else {                   // 设置参数并重启失败
        lteEnterConfigMode();  // 进入配置模式
        KPrintf("*-*-*-*-*-*-*-*-*-*-*configuration mode*-*-*-*-*-*-*-*-*-*-*-*-*\n# ");
    }
}

/**
 * @brief  lte_test的命令行模式下，对DBG串口的数据进行处理后再发送给MAIN串口
 * @param  ch    从DBG串口接收到的数据
 * @note   该方法在Ubiquitous/XiZi_IIoT/board/ch32v208rbt6/third_party_driver/uart/connect_uart.c中进行调用
 */
void lteRedirectUart(char ch) {
    /* 将键盘按键转发至LTE 4G模块 */
    static char sendBuffer[100];  // 接收缓存
    static int index = 0;         // 缓存数据的下一个即将保存的位置下标
    extern int testLTE;           // 是否处于lte_test
    if (testLTE && ch != -1) {
        sendBuffer[index++] = ch;  // 缓存数据
        KPrintf("%c", ch);         // 打印到shell中
        if (ch == '\r') {          // 接收到回车键
            sendBuffer[index] = '\0';
            KPrintf("\n");
            if (strcmp(sendBuffer, "quit\r") == 0) {        // 退出lte，返回命令行
                testLTE = 0;                                // 退出lte_test
                index = 0;                                  // 逻辑清空缓存
            } else if (strcmp(sendBuffer, "+++\r") == 0) {  // 从网络透传模式返回配置模式
                void lteEnterConfigMode(void);
                lteEnterConfigMode();  // 进入配置模式
                KPrintf("OK\n");
                KPrintf("*-*-*-*-*-*-*-*-*-*-*configuration mode*-*-*-*-*-*-*-*-*-*-*-*-*\n");
                KPrintf("# ");  // 打印AT命令提示符
                index = 0;      // 逻辑清空缓存
            } else if (lteWorkMode == WORK_CONFIGURATION_MODE &&
                       strcmp(sendBuffer, "AT+S\r") == 0) {  // 配置模式下保存参数并且重启
                int lteSaveConfigAndRestart(void);           // 保存参数并且重启4G模块
                if (lteSaveConfigAndRestart() == 0) {
                    void lteEnterConfigMode(void);
                    lteEnterConfigMode();
                    KPrintf("OK\n");
                    KPrintf("# ");
                } else {
                    KPrintf("save config and restart error\n");
                }
                index = 0;  // 逻辑清空缓存
            } else {
                if (lteWorkMode == WORK_CONFIGURATION_MODE &&
                    strcmp(sendBuffer, "AT+ENTM\r") == 0) {  // 配置模式下试图进入通信模式
                    lteWorkMode = WORK_COMMUNICATION_MODE;   // 更换工作模式
                }
                sendBuffer[index++] = '\n';  // 命令结尾为"\r\n"，敲击一次回车只有一个"\r"
                                             /* 将缓存中的所有数据发送到MAIN串口 */
                for (int i = 0; i < index; i++) {
                    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
                    }
                    USART_SendData(USART2, sendBuffer[i]);
                }
                index = 0;  // 逻辑清空缓存
            }
        } else if (ch == '\b') {  // 键盘输入退格键
            KPrintf(" \b");
            if (index > 1) {  // 从缓存中删除前一个字符
                index -= 2;
            } else {  // 缓存中无字符可以删除
                index = 0;
            }
        }
    }
}

/**
 * @brief  test_lte命令
 * @param  argc  参数个数
 * @param  argv  参数数组
 * @return int   命令返回值，0表示命令执行正常
 */
int test_lte(int argc, char *argv[]) {
    char receiveBuffer[100];
    testLTE = 1;  // 表示当前正在测试LTE模块，需要将DBG串口和MAIN串口的数据进行转发
    lteInit();    // 初始化4G模块
    while (1) {
        if (lteMode == TRANSPARENT_TRANSMISSION_MODE) {  // 透传模式直接接收数据，不做任何处理
            KPrintf("%c", lteReceiveData());
        } else if (lteMode == COMMAND_LINE_MODE) {         // 命令行模式
            if (lteWorkMode == WORK_CONFIGURATION_MODE) {  // 当前处于配置模式
                lteReceiveResponse(receiveBuffer, sizeof(receiveBuffer));
                KPrintf("%s\n", receiveBuffer);
                if (lteWorkMode == WORK_CONFIGURATION_MODE) {
                    KPrintf("# ");  // 等待输入AT指令
                } else {            // 之前输入AT+ENTM命令，从配置模式切换到通信模式
                    KPrintf("*-*-*-*-*-*-*-*-*-*-*communication mode*-*-*-*-*-*-*-*-*-*-*-*-*\n");
                }
            } else if (lteWorkMode == WORK_COMMUNICATION_MODE) {  // 当前处于通信模式
                KPrintf("%c", lteReceiveData());
            }
        }
        if (testLTE == 0) {  // 结束LTE测试，testLTE的值可以通过输入quit回车赋值为0
            break;
        }
    }
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), test_lte, test_lte, test LTE);
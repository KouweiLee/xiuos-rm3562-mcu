/********************************** (C) COPYRIGHT *******************************
 * File Name          : peripheral.C
 * Author             : WCH
 * Version            : v1.0
 * Date               : 2020/11/26
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/**
 * @file app_uart.c
 * @brief  解析蓝牙包，将其拼接成蓝牙请求报文
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-19
 */
#include "app_uart.h"

#include <config.h>
#include <devinfoservice.h>
#include <gattprofile.h>
#include <peripheral.h>
#include <transform.h>

/*********************************************************************
 * MACROS
 */
// The buffer length should be a power of 2
#define APP_UART_TX_BUFFER_LENGTH 128U
#define APP_UART_RX_BUFFER_LENGTH 512U

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8_t to_test_buffer[BLE_BUFF_MAX_LEN - 4 - 3];

app_drv_fifo_t app_uart_tx_fifo;
app_drv_fifo_t app_uart_rx_fifo;

// interupt uart rx flag ,clear at main loop
bool uart_rx_flag = false;

// for interrupt rx blcak hole ,when uart rx fifo full
uint8_t for_uart_rx_black_hole = 0;

// fifo length less that MTU-3, retry times
uint32_t uart_to_ble_send_evt_cnt = 0;

// The tx buffer and rx buffer for app_drv_fifo
// length should be a power of 2
static uint8_t app_uart_tx_buffer[APP_UART_TX_BUFFER_LENGTH] = {0};
static uint8_t app_uart_rx_buffer[APP_UART_RX_BUFFER_LENGTH] = {0};

#define MAX_BLE_REQUEST_BUFFER_SIZE 256

/**
 * @brief  用于接收蓝牙请求的缓冲区结构体
 */
struct BleRequestBuffer {
    uint8_t buffer[MAX_BLE_REQUEST_BUFFER_SIZE];  // 缓冲区
    uint8_t length;                               // 缓冲区中已使用的字节数
    uint8_t expectdPacketId;                      // 期望的下一个蓝牙包所属的报文ID
    uint8_t expectedPacketNumber;                 // 期望的下一个蓝牙包编号
    uint8_t isExpected;                           // 是否期望下一个蓝牙包
} bleRequestBuffer;

void startParseBleRequestTask(uint8_t *request);  // 声明启动解析蓝牙请求任务的函数

/**
 * @brief 解析蓝牙包，将其拼接成蓝牙请求报文
 */
void app_uart_process(void) {
    static uint8_t blePacketBuffer[20];   // 蓝牙包缓存
    static int blePacketBufferIndex = 0;  // 蓝牙包缓存索引
    uint8_t data;                         // 接收到的1字节数据

    __disable_irq();
    if (uart_rx_flag) {
        tmos_start_task(Peripheral_TaskID, UART_TO_BLE_SEND_EVT, 2);
        uart_rx_flag = false;
    }
    __enable_irq();

    /* 解析蓝牙包 */
    while (app_drv_fifo_length(&app_uart_tx_fifo)) {
        /* 从蓝牙缓存中读取1字节 */
        app_drv_fifo_read_to_same_addr(&app_uart_tx_fifo, (uint8_t *)&data, 1);
        blePacketBuffer[blePacketBufferIndex++] = data;

        /* 如果接收到的字节数达到20字节，则表示接收到了1个完整的蓝牙包 */
        if (blePacketBufferIndex == 20) {
            uint8_t packetId = blePacketBuffer[0];                              // 报文ID
            uint8_t hasNextPacket = blePacketBuffer[1] & 0x80;                  // 该报文是否有下一个蓝牙包
            uint8_t packetNumber = blePacketBuffer[1] & 0x7f;                   // 蓝牙包序号
            if (bleRequestBuffer.length + 18 <= MAX_BLE_REQUEST_BUFFER_SIZE) {  // 判断缓冲区是否足够大
                if (bleRequestBuffer.expectedPacketNumber != packetNumber ||
                    (bleRequestBuffer.isExpected && bleRequestBuffer.expectdPacketId != packetId)) {  // 蓝牙包乱序
                    printf("Packet sequence mismatch, expected packetId and packetNumber: [%u-%u], but got [%u-%u]\n",
                           bleRequestBuffer.expectdPacketId, bleRequestBuffer.expectedPacketNumber, packetId, packetNumber);
                    if (packetNumber == 0) {  // 如果接收到的新的蓝牙包序号为0
                        if (hasNextPacket) {  // 如果还有该蓝牙包所属的报文还有下一个蓝牙包，则丢弃之前已存储的包
                            memset(bleRequestBuffer.buffer, 0, MAX_BLE_REQUEST_BUFFER_SIZE);
                            memcpy(bleRequestBuffer.buffer, blePacketBuffer + 2, 18);
                            bleRequestBuffer.length = 18;
                            bleRequestBuffer.isExpected = 1;
                            bleRequestBuffer.expectdPacketId = packetId;
                            bleRequestBuffer.expectedPacketNumber = packetNumber + 1;
                        } else {  // 如果该蓝牙包所属的报文没有下一个蓝牙包，则直接解析，并且不丢弃之前存储的蓝牙包
                            uint8_t *bleRequest = (uint8_t *)PrivMalloc(18);
                            memcpy(bleRequest, blePacketBuffer + 2, 18);
                            startParseBleRequestTask(bleRequest);
                        }
                    }
                } else {                  // 蓝牙包不乱序
                    if (hasNextPacket) {  // 该蓝牙包所属的报文还有下一个蓝牙包
                        memcpy(bleRequestBuffer.buffer + bleRequestBuffer.length, blePacketBuffer + 2, 18);
                        bleRequestBuffer.expectdPacketId = packetId;
                        bleRequestBuffer.expectedPacketNumber = packetNumber + 1;
                        bleRequestBuffer.isExpected = 1;
                        bleRequestBuffer.length += 18;
                        printf("Received packet [%u-%u], expected next packet [%u-%u]\n", packetId, packetNumber,
                               bleRequestBuffer.expectdPacketId, bleRequestBuffer.expectedPacketNumber);
                    } else {  // 该蓝牙包所属的报文没有下一个蓝牙包，可以解析该报文
                        printf("Received last packet %u\n", packetNumber);
                        memcpy(bleRequestBuffer.buffer + bleRequestBuffer.length, blePacketBuffer + 2, 18);
                        bleRequestBuffer.length += 18;
                        uint8_t *bleRequest = (uint8_t *)PrivMalloc(bleRequestBuffer.length);
                        memcpy(bleRequest, bleRequestBuffer.buffer, bleRequestBuffer.length);
                        startParseBleRequestTask(bleRequest);  // 解析蓝牙报文
                                                               /* 重置缓冲区 */
                        memset(bleRequestBuffer.buffer, 0, MAX_BLE_REQUEST_BUFFER_SIZE);
                        bleRequestBuffer.isExpected = 0;
                        bleRequestBuffer.expectedPacketNumber = 0;
                        bleRequestBuffer.length = 0;
                    }
                }
            } else {  // 报文缓冲区已满，丢弃该包所属的报文已存储的所有数据
                printf("Buffer overflow, packet dropped\n");
                memset(bleRequestBuffer.buffer, 0, MAX_BLE_REQUEST_BUFFER_SIZE);
                bleRequestBuffer.isExpected = 0;
                bleRequestBuffer.expectedPacketNumber = 0;
                bleRequestBuffer.length = 0;
            }
            blePacketBufferIndex = 0;        // 蓝牙包缓冲区索引重置
            memset(blePacketBuffer, 0, 20);  // 蓝牙包缓冲区清零
        }
    }
}

/**
 * @brief  初始化蓝牙底层缓冲区
 */
void app_uart_init() {
    // The buffer length should be a power of 2
    app_drv_fifo_init(&app_uart_tx_fifo, app_uart_tx_buffer, APP_UART_TX_BUFFER_LENGTH);
    app_drv_fifo_init(&app_uart_rx_fifo, app_uart_rx_buffer, APP_UART_RX_BUFFER_LENGTH);
}

/*********************************************************************
 * @fn      app_uart_tx_data
 *
 * @brief   app_uart_tx_data
 *
 * @return  NULL
 */
void app_uart_tx_data(uint8_t *data, uint16_t length) {
    uint16_t write_length = length;
    KPrintf("BLE receive data: ");
    for (int i = 0; i < write_length; i++) {
        KPrintf("0x%02x ", data[i]);
    }
    KPrintf("\n");
    app_drv_fifo_write(&app_uart_tx_fifo, data, &write_length);
}

/*********************************************************************
 * @fn      UART3_IRQHandler
 *
 * @brief   Not every uart reception will end with a UART_II_RECV_TOUT
 *          UART_II_RECV_TOUT can only be triggered when R8_UARTx_RFC is not 0
 *          Here we cannot rely UART_II_RECV_TOUT as the end of a uart reception
 *
 * @return  NULL
 * @note 注意USART3已被BLE占用，因此BLE启用时，该函数不能开启
 */
// __attribute__((interrupt()))
// void USART3_IRQHandler(void)
// {
//     uint16_t error;
//     uint8_t data;
//     if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//     {
//         data = USART_ReceiveData(USART3);
//         error = app_drv_fifo_write_from_same_addr(&app_uart_rx_fifo, (uint8_t *)&data, 1);
//         if(error != APP_DRV_FIFO_RESULT_SUCCESS)
//         {
//             KPrintf("APP_DRV_FIFO_RESULT_NOT_MEM\r\n");
//         }
//         uart_rx_flag = true;
//     }
// }

/*********************************************************************
 * @fn      on_bleuartServiceEvTMOS_SystemProcesst
 *
 * @brief   ble uart service callback handler
 *
 * @return  NULL
 */
void on_bleuartServiceEvt(uint16_t connection_handle, ble_uart_evt_t *p_evt) {
    switch (p_evt->type) {
        case BLE_UART_EVT_TX_NOTI_DISABLED:
            KPrintf("%02x:bleuart_EVT_TX_NOTI_DISABLED\r\n", connection_handle);
            break;
        case BLE_UART_EVT_TX_NOTI_ENABLED:
            KPrintf("%02x:bleuart_EVT_TX_NOTI_ENABLED\r\n", connection_handle);
            break;
        case BLE_UART_EVT_BLE_DATA_RECIEVED:
            KPrintf("BLE RX DATA len:%d\r\n", p_evt->data.length);

            // for notify back test
            // to ble
            uint16_t to_write_length = p_evt->data.length;
            app_drv_fifo_write(&app_uart_rx_fifo, (uint8_t *)p_evt->data.p_data, &to_write_length);
            tmos_start_task(Peripheral_TaskID, UART_TO_BLE_SEND_EVT, 2);
            // end of nofify back test

            // ble to uart
            app_uart_tx_data((uint8_t *)p_evt->data.p_data, p_evt->data.length);

            break;
        default:
            break;
    }
}

/*********************************************************************
*********************************************************************/

/**
 * @file peripheral_main.c
 * @brief  蓝牙外设主任务
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-19
 */
#include <transform.h>

#include "HAL.h"
#include "app_uart.h"
#include "config.h"
#include "debug.h"
#include "gattprofile.h"
#include "peripheral.h"
#include "shell.h"

__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
uint8_t const MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/**
 * @brief  蓝牙循环主任务
 */
__attribute__((section(".highcode"))) __attribute__((noinline)) void *Main_Circulation(void *arg) {
    while (1) {
        TMOS_SystemProcess();  // 蓝牙底层交互
        app_uart_process();    // 处理小程序蓝牙请求报文
    }
}

/**
 * @brief  开启蓝牙交互的线程，此方法在main方法中被调用，表示开机或复位启动
 */
void startUpBleTask(void) {
    pthread_attr_t bleTaskAttr;
    pthread_args_t bleTaskArgs;
    bleTaskAttr.schedparam.sched_priority = 16;   // 线程优先级
    bleTaskAttr.stacksize = 2048;                 // 线程栈大小
    bleTaskArgs.pthread_name = "wchbleMainTask";  // 线程名字
    pthread_t bleThread;                          // 线程ID
    PrivTaskCreate(&bleThread, &bleTaskAttr, Main_Circulation, &bleTaskArgs);
    PrivTaskStartup(&bleThread);
}
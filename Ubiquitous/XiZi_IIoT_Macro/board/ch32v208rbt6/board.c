/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
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
 * @file board.c
 * @brief support ch32v208 init configure and start-up
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2024-4-29
 */
#include "board.h"

#include <ModuleConfig.h>
#include <device.h>
#include <stdint.h>
#include <xizi.h>

#include "HAL.h"
#include "adc.h"
#include "ch32v20x.h"
#include "connect_ble.h"
#include "connect_can.h"
#include "connect_ether.h"
#include "connect_lte.h"
#include "connect_rs485.h"
#include "connect_uart.h"
#include "core_riscv.h"
#include "wchble.h"
#include "xsconfig.h"

static uint32_t _SysTick_Config(uint32_t ticks) {
    // SystemCoreClockUpdate();
    NVIC_SetPriority(SysTicK_IRQn, 0x01);
    NVIC_SetPriority(Software_IRQn, 0xf0);
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_EnableIRQ(Software_IRQn);
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = ticks - 1;
    SysTick->CTLR = 0xF;
    return 0;
}

/**
 * @brief  从FLASH中读取配置信息。若FLASH中的配置信息无效，则将默认的配置信息写入FLASH，再复位ch32v208。
 * @note 目前的配置信息仅包含RS485、以太网、4G（LTE）模块的配置信息。
 * @note FLASH中的配置信息存放在PAGE_WRITE_START_ADDR(478K)地址处，大小为1KB。
 */
void readRomConfiguration(void) {
    extern module_cfg defaultConfiguration;  // 默认配置信息
    extern u8 Configbuf[MODULE_CFG_LEN];     // 内存存放从FLASH中读取的配置信息
    extern pmodule_cfg CFG;                  // 指向配置信息的指针

    /* 从EEPROM中读取网络配置信息 */
    CFG_READ(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);

    /* 如果存储在EEPROM中的网络配置信息无效，或者WCHNET还没有被主机配置过，通过默认配置信息初始化WCHNET */
    if ((CFG->cfgFlag[0] != checkcode1) || (CFG->cfgFlag[1] != checkcode2)) {
        CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
        CFG_WRITE(PAGE_WRITE_START_ADDR, (u8 *)&defaultConfiguration, MODULE_CFG_LEN);
        KPrintf("%s NVIC_SystemReset\r\n\r\n", __func__);
        NVIC_SystemReset(); // 复位ch32v208
    }

    /* 输出配置信息 */
    KPrintf("\n*************rom configuration****************\n");
    KPrintf("moduleName:\t%s\n", CFG->moduleName);
    KPrintf("type:\t\t%u\n", CFG->type);

    /* 4G模块目的IP和端口号配置 */
    KPrintf("----------------------------------------------\n");
    KPrintf("                      4G                      \n");
    KPrintf("destinationIpAddress:\t\t%u.%u.%u.%u\n", CFG->destinationIpAddress_4G[0], CFG->destinationIpAddress_4G[1],
            CFG->destinationIpAddress_4G[2], CFG->destinationIpAddress_4G[3]);
    KPrintf("destinationPort: \t\t%hu\n", (unsigned short)CFG->destinationPort_4G[0] | CFG->destinationPort_4G[1] << 8);

    /* 4G模块MQTT配置 */
    KPrintf("mqttSwitch: \t\t\t%s\n", CFG->mqttSwitch_4G ? "on" : "off");
    KPrintf("mqttTopic: \t\t\t%s\n", CFG->mqttTopic_4G);
    KPrintf("mqttUsername:\t\t\t%s\n", CFG->mqttUsername_4G);
    KPrintf("mqttPassword:\t\t\t%s\n", CFG->mqttPassword_4G);
    KPrintf("mqttClientId:\t\t\t%s\n", CFG->mqttClientId_4G);

    /* Ethernet模块配置 */
    KPrintf("----------------------------------------------\n");
    KPrintf("                   Ethernet                   \n");
    KPrintf("destinationIpAddress: \t\t%u.%u.%u.%u\n", CFG->destinationIpAddress_Ethernet[0], CFG->destinationIpAddress_Ethernet[1],
            CFG->destinationIpAddress_Ethernet[2], CFG->destinationIpAddress_Ethernet[3]);
    KPrintf("destinationPort: \t\t%hu\n", (unsigned short)CFG->destinationPort_Ethernet[0] | CFG->destinationPort_Ethernet[1] << 8);
    KPrintf("dhcpSwitch: \t\t\t%s\n", CFG->dhcpSwitch_Ethernet ? "on" : "off");
    KPrintf("sourceIpAddress: \t\t%u.%u.%u.%u\n", CFG->sourceIpAddress_Ethernet[0], CFG->sourceIpAddress_Ethernet[1],
            CFG->sourceIpAddress_Ethernet[2], CFG->sourceIpAddress_Ethernet[3]);
    KPrintf("sourcePort: \t\t\t%hu\n", (unsigned short)CFG->sourcePort_Ethernet[0] | CFG->sourcePort_Ethernet[1] << 8);
    KPrintf("sourceSubnetMask: \t\t%u.%u.%u.%u\n", CFG->sourceSubnetMask_Ethernet[0], CFG->sourceSubnetMask_Ethernet[1],
            CFG->sourceSubnetMask_Ethernet[2], CFG->sourceSubnetMask_Ethernet[3]);
    KPrintf("sourceClientGateway: \t\t%u.%u.%u.%u\n", CFG->sourceGateway_Ethernet[0], CFG->sourceGateway_Ethernet[1],
            CFG->sourceGateway_Ethernet[2], CFG->sourceGateway_Ethernet[3]);

    /* RS485模块配置 */
    KPrintf("----------------------------------------------\n");
    KPrintf("                    Rs485                     \n");
    int baudRate_Rs485;
    int dataBits_Rs485;
    int stopBits_Rs485;
    char *parity_Rs485;
    switch (CFG->baudRate_Rs485) {
        case 1:
            baudRate_Rs485 = BAUD_RATE_2400;
            break;
        case 2:
            baudRate_Rs485 = BAUD_RATE_4800;
            break;
        case 3:
            baudRate_Rs485 = BAUD_RATE_9600;
            break;
        case 4:
            baudRate_Rs485 = BAUD_RATE_19200;
            break;
        case 5:
            baudRate_Rs485 = BAUD_RATE_38400;
            break;
        case 6:
            baudRate_Rs485 = BAUD_RATE_57600;
            break;
        case 7:
            baudRate_Rs485 = BAUD_RATE_115200;
            break;
        case 8:
            baudRate_Rs485 = BAUD_RATE_230400;
            break;
        default:
            baudRate_Rs485 = BAUD_RATE_9600;
            break;
    }
    switch (CFG->dataBits_Rs485) {
        case 1:
            dataBits_Rs485 = DATA_BITS_8;
            break;
        case 2:
            dataBits_Rs485 = DATA_BITS_9;
            break;
        default:
            dataBits_Rs485 = DATA_BITS_8;
            break;
    }
    switch (CFG->stopBits_Rs485) {
        case 1:
            stopBits_Rs485 = STOP_BITS_1;
            break;
        case 2:
            stopBits_Rs485 = STOP_BITS_2;
            break;
        default:
            stopBits_Rs485 = STOP_BITS_1;
    }
    switch (CFG->parity_Rs485) {
        case 1:
            parity_Rs485 = "NONE";
            break;
        case 2:
            parity_Rs485 = "ODD";
            break;
        case 3:
            parity_Rs485 = "EVEN";
            break;
        default:
            parity_Rs485 = "NONE";
            break;
    }
    KPrintf("baudRate:\t\t\t%d\n", baudRate_Rs485);
    KPrintf("dataBits:\t\t\t%d\n", dataBits_Rs485);
    KPrintf("stopBits:\t\t\t%d\n", stopBits_Rs485);
    KPrintf("parity:\t\t\t\t%s\n", parity_Rs485);
    /* 配置有效位校验 */
    KPrintf("----------------------------------------------\n");
    KPrintf("cfg_flag: 0x%02x 0x%02x\n", CFG->cfgFlag[0], CFG->cfgFlag[1]);
    KPrintf("**********************************************\n\n");
}

/**
 * This function will initial your board.
 */
void InitBoardHardware() {
    Delay_Init();
    USART_Printf_Init(115200);
    /* System Tick Configuration */
	extern uint32_t SystemCoreClock;
    _SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
    /* initialize memory system */
    InitBoardMemory(MEMORY_START_ADDRESS, (void *)MEMORY_END_ADDRESS);

#ifdef BSP_USING_UART
    InitHwUart();
    InstallConsole("uart1", SERIAL_DRV_NAME_1, SERIAL_1_DEVICE_NAME_0);
    KPrintf("\nconsole init completed.\n");
    KPrintf("compiled on: %s at %s\n", __DATE__, __TIME__);
#endif

    readRomConfiguration();  // 读取配置信息到外部变量CFG中

    KPrintf("board initialization......\n");
#ifdef BSP_USING_RS485
    InitHwRs485();
#endif

#ifdef BSP_USING_ETH
    InitHwEth();
#endif

#ifdef BSP_USING_ADC
    ADC_Function_Init();  // 当前CH32V208的ADC引脚和CAN引脚有功能重叠，因此ADC和CAN不能同时使用
#endif

#ifdef BSP_USING_BLE
    InitHwBle();
#endif

#ifdef BSP_USING_CAN
    InitHwCan();
#endif

#ifdef BSP_USING_LTE
    InitHwLte();
#endif

    KPrintf("memory address range: [0x%08x - 0x%08x] SRAM_SIZE: %ld, ssize: %ld\n", (x_ubase)MEMORY_START_ADDRESS,
            (x_ubase)MEMORY_END_ADDRESS, SRAM_SIZE, __stack_size);
    KPrintf("board init done.\n");
    KPrintf("start kernel...\n");

#ifdef TOOL_USING_OTA
    extern void app_clear_jumpflag(void);
    app_clear_jumpflag();  // set lastjumpflag to JUMP_SUCCESS_FLAG
#endif
}

/********************************** (C) COPYRIGHT *******************************
 * File Name          : ModuleConfig.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/06/10
 * Description        : Module configuration related command codes
 *                      and configuration structures.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __MODULECONFIG_H__
#define __MODULECONFIG_H__

#include <pthread.h>

#include "debug.h"

/*Address where configuration information is stored*/
// #define PAGE_WRITE_START_ADDR     ((uint32_t)0x0801FF00) /* Start from (128K - 256B) */
// #define PAGE_WRITE_END_ADDR       ((uint32_t)0x08020000) /* End at 128K */
// #define FLASH_PAGE_SIZE           256
#define PAGE_WRITE_START_ADDR ((uint32_t)0x08077800) /* Start from (479K - 1K) */
#define PAGE_WRITE_END_ADDR ((uint32_t)0x08077C00)   /* End at 479K */
#define FLASH_PAGE_SIZE 1024

#define NET_MODULE_DATA_LENGTH 60  // Maximum length of the data area
#define MODULE_CFG_LEN 272         // The length of configuration buff

/*Communication command code*/
#define NET_MODULE_CMD_SET 0X01     // Configure module parameters
#define NET_MODULE_CMD_GET 0X02     // Get module configuration
#define NET_MODULE_CMD_RESET 0X03   // reset module
#define NET_MODULE_CMD_SEARCH 0X04  // search module

// #define NET_MODULE_CMD_RESET 0x00            // reset module
// #define NET_MODULE_CMD_GET 0x01              // Get module configuration
// #define NET_MODULE_CMD_SET_4G 0x02           // Configure 4G module parameters
// #define NET_MODULE_CMD_SET_ETH_NO_DHCP 0x03  // Configure Ethernet module parameters without DHCP
// #define NET_MODULE_CMD_SET_ETH_DHCP 0x04     // Configure Ethernet module parameters with DHCP

#define checkcode1 0X11  // Identity configuration information
#define checkcode2 0x22

/*communication structure*/
typedef struct NET_COMM {
    /*The communication identifier,
     *  because they are all communicated
     *  by broadcasting, a fixed value is
     *  added here.*/
    unsigned char cmd;                          // command header
    unsigned char len;                          // data length
    unsigned char dat[NET_MODULE_DATA_LENGTH];  // data buffer
} net_comm, *pnet_comm;

/*Module Communication Mode Identification*/
#define NET_MODULE_TYPE_TCP_S 0X00       // TCP SERVER
#define NET_MODULE_TYPE_TCP_C 0X01       // TCP CLIENT
#define NET_MODULE_TYPE_UDP_S 0X02       // UDP SERVER
#define NET_MODULE_TYPE_UDP_C 0X03       // UDP CLIENT
#define NET_MODULE_TYPE_NONE 0X04        // no mode is enabled
#define NET_MODULE_TYPE_TCP_C_DHCP 0X05  // TCP CLIENT DHCP
#define NET_MODULE_TYPE_4G 0x06          // TCP CLIENT 4G

/*Configuration structure*/
typedef struct MODULE_CFG {
    unsigned char moduleName[21];                    // module name
    unsigned char type;                              // The module is in default mode (no mode is enabled by default)
    unsigned char destinationIpAddress_4G[4];        // The IP address of the server connected by 4G module
    unsigned char destinationPort_4G[2];             // The port number of the server connected by 4G module
    unsigned char mqttSwitch_4G;                     // MQTT switch
    unsigned char mqttTopic_4G[64];                  // MQTT topic
    unsigned char mqttUsername_4G[64];               // MQTT username
    unsigned char mqttPassword_4G[64];               // MQTT password
    unsigned char mqttClientId_4G[24];               // MQTT client ID
    unsigned char destinationIpAddress_Ethernet[4];  // The IP address of the server connected by ethernet module
    unsigned char destinationPort_Ethernet[2];       // The port number of the server connected by ethernet module
    unsigned char dhcpSwitch_Ethernet;               // DHCP switch
    unsigned char sourceIpAddress_Ethernet[4];       // The IP address of the ethernet module
    unsigned char sourcePort_Ethernet[2];            // The port number of the ethernet module
    unsigned char sourceSubnetMask_Ethernet[4];      // The subnet mask of the ethernet module
    unsigned char sourceGateway_Ethernet[4];         // The gateway address of the ethernet module
    unsigned char baudRate_Rs485;                    // baud rate of RS485, `2400` -> `0x01`|`4800` -> `0x02`|`9600` -> `0x03`|`19200` -> `0x04`|`38400` -> `0x05|`57600` -> `0x06`|`115200` -> `0x07`|`230400` -> `0x08`
    unsigned char dataBits_Rs485;                    // data bits of RS485, `8` -> `0x01`|`9` -> `0x02`
    unsigned char stopBits_Rs485;                    // stop bits of RS485, `1` -> `0x01`|`2` -> `0x02`
    unsigned char parity_Rs485;                      // parity of RS485, `None` -> `0x01`|` Odd` -> `0x02`|`Even` -> `0x03`
    unsigned char cfgFlag[2];                        // Verification code, used to verify configuration information
} module_cfg, *pmodule_cfg;

extern u8 Configbuf[MODULE_CFG_LEN];

extern module_cfg defaultConfiguration;

extern pmodule_cfg CFG;

extern u8 ParseConfigbuf(u8 *buf);

extern pthread_mutex_t romConfigurationMutex;

extern void CFG_ERASE(uint32_t Page_Address, u32 Length);

extern void CFG_READ(u32 StartAddr, u8 *Buffer, u32 Length);

extern FLASH_Status CFG_WRITE(u32 StartAddr, u8 *Buffer, u32 Length);

#endif
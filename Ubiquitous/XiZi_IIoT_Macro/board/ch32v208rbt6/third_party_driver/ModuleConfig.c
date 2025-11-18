/**
 * @file ModuleConfig.c
 * @brief  从FLASH中读取配置信息；解析蓝牙请求，将配置信息写入到FLASH中
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-19
 * @note 目前的配置信息仅包含4G、以太网、RS485三个模块的内容
 */

#include <ModuleConfig.h>
#include <adapter.h>
#include <connect_ether.h>
#include <peripheral.h>
#include <stdio.h>
#include <string.h>
#include <transform.h>
#include <wchble.h>

__attribute__((aligned(4))) module_cfg defaultConfiguration = {.moduleName = "XiZi",
                                                               .type = NET_MODULE_TYPE_TCP_C_DHCP,
                                                               .destinationIpAddress_4G = {115, 238, 53, 59},
                                                               .destinationPort_4G[0] = 10208 % 256,
                                                               .destinationPort_4G[1] = 10208 / 256,
                                                               .mqttSwitch_4G = 0,
                                                               .mqttTopic_4G = "/SubTopic1",
                                                               .mqttUsername_4G = "username",
                                                               .mqttPassword_4G = "password",
                                                               .mqttClientId_4G = "clientId",
                                                               .destinationIpAddress_Ethernet = {115, 238, 53, 59},
                                                               .destinationPort_Ethernet[0] = 10208 % 256,
                                                               .destinationPort_Ethernet[1] = 10208 / 256,
                                                               .dhcpSwitch_Ethernet = 1,
                                                               .sourceIpAddress_Ethernet = {192, 168, 137, 10},
                                                               .sourcePort_Ethernet[0] = 1000 % 256,
                                                               .sourcePort_Ethernet[1] = 1000 / 256,
                                                               .sourceSubnetMask_Ethernet = {255, 255, 255, 0},
                                                               .sourceGateway_Ethernet = {192, 168, 137, 1},
                                                               .baudRate_Rs485 = 3,
                                                               .dataBits_Rs485 = 1,
                                                               .stopBits_Rs485 = 1,
                                                               .parity_Rs485 = 1,
                                                               .cfgFlag[0] = checkcode1,
                                                               .cfgFlag[1] = checkcode2};  // 默认配置信息

u8 Configbuf[MODULE_CFG_LEN];              // 从flash中读取的配置信息数组
pmodule_cfg CFG = (pmodule_cfg)Configbuf;  // 从flash中读取的配置信息指针
pthread_mutex_t romConfigurationMutex;     // CFG的互斥锁

/**
 * @brief  擦除FLASH块，最小的块为256B
 * @param  Page_Address 擦除页的开始地址
 * @param  Length 擦除数据长度
 */
void CFG_ERASE(uint32_t Page_Address, u32 Length) {
    u32 NbrOfPage, EraseCounter;

    FLASH_Unlock_Fast();
    NbrOfPage = Length / FLASH_PAGE_SIZE;

    for (EraseCounter = 0; EraseCounter < NbrOfPage; EraseCounter++) {
        FLASH_ErasePage_Fast(Page_Address + (FLASH_PAGE_SIZE * EraseCounter));
    }
    FLASH_Lock_Fast();
}

/**
 * @brief  写FLASH数据块
 * @param  StartAddr 写入页的开始地址
 * @param  Buffer 写入的数据缓冲区
 * @param  Length 写入的数据长度
 * @return FLASH_Status FLASH状态，定义位置：Ubiquitous/XiZi_IIoT/board/ch32v208rbt6/third_party_driver/Peripheral/inc/ch32v20x_flash.h
 */
FLASH_Status CFG_WRITE(u32 StartAddr, u8 *Buffer, u32 Length) {
    u32 address = StartAddr;
    u32 *p_buff = (u32 *)Buffer;
    FLASH_Status FLASHStatus = FLASH_COMPLETE;

    FLASH_Unlock();
    while ((address < (StartAddr + Length)) && (FLASHStatus == FLASH_COMPLETE)) {
        FLASHStatus = FLASH_ProgramWord(address, *p_buff);
        address += 4;
        p_buff++;
    }
    FLASH_Lock();
    return FLASHStatus;
}

/**
 * @brief  从FLASH中读取数据块
 * @param  StartAddr 读取页的开始地址
 * @param  Buffer 读取的数据缓冲区
 * @param  Length 读取的数据长度
 */
void CFG_READ(u32 StartAddr, u8 *Buffer, u32 Length) {
    u32 address = StartAddr;
    u32 *p_buff = (u32 *)Buffer;

    while (address < (StartAddr + Length)) {
        *p_buff = (*(u32 *)address);
        address += 4;
        p_buff++;
    }
}

#ifdef BSP_USING_BLE
/**
 * @brief  尝试使用配置通过4G连接到服务器，并且蓝牙响应连接状态报文。
 * @param  requestId 蓝牙请求报文ID
 * @note 在解析完成蓝牙请求和写入FLASH配置之后调用该函数
 */
void connectServer_4G(uint8_t requestId) {
    uint8_t destinationIpAddress[16] = {};  // 字符串形式的目的IP地址
    uint8_t destinationPort[6] = {};        // 字符串形式的目的端口号

    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_4G_NAME);  // 查找以太网模块适配器

    /* 如果4G模块仍未初始化，则返回连接失败报文 */
    if (adapter->agent == NULL) {
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x01;
        notificationReport[3] = 0x02;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
        return;
    }

    /* 获取互斥锁 */
    PrivMutexObtain(&adapter->lock);          // 若其他线程正在使用adapter，则阻塞等待
    PrivMutexObtain(&romConfigurationMutex);  // 若其他线程正在读取或者写入CFG，则阻塞等待

    /* 尝试连接到服务器 */
    sprintf(destinationIpAddress, "%u.%u.%u.%u", CFG->destinationIpAddress_4G[0], CFG->destinationIpAddress_Ethernet[1],
            CFG->destinationIpAddress_4G[2], CFG->destinationIpAddress_4G[3]);
    sprintf(destinationPort, "%hu", (unsigned short)CFG->destinationPort_4G[0] | CFG->destinationPort_4G[1] << 8);
    printf("-*-*-*-*-AdapterDeviceConnect*-*-*-*-\n");
    printf("destinationIpAddress: %s\n", destinationIpAddress);
    printf("destinationPort:      %s\n", destinationPort);
    printf("-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*\n");
    if (CFG->mqttSwitch_4G == 1) {  // 如果使能MQTT
        AdapterDeviceMqttConnect(adapter, destinationIpAddress, destinationPort, CFG->mqttClientId_4G, CFG->mqttUsername_4G,
                                 CFG->mqttPassword_4G);
    } else {  // 如果禁用MQTT
        AdapterDeviceConnect(adapter, CLIENT, destinationIpAddress, destinationPort, IPV4);
    }

    /* 蓝牙反馈连接状态 */
    AdapterDeviceNetstat(adapter);  // 读取网络状态
    if (CFG->mqttSwitch_4G == 0 && adapter->network_info.is_connected ||
        CFG->mqttSwitch_4G == 1 && adapter->network_info.mqttIsConnected) {  // 如果连接成功
        /* 释放互斥锁 */
        PrivMutexAbandon(&romConfigurationMutex);
        PrivMutexAbandon(&adapter->lock);
        /* 发送网络连接成功报文 */
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x01;
        notificationReport[3] = 0x01;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
    } else {  // 如果连接失败
        printf("4G connect to server failed\n");
        /* 释放互斥锁 */
        PrivMutexAbandon(&romConfigurationMutex);
        PrivMutexAbandon(&adapter->lock);
        /* 发送网络连接失败报文 */
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x01;
        notificationReport[3] = 0x02;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
    }

    return;
}

/**
 * @brief  尝试使用配置通过以太网连接到服务器，并且蓝牙响应连接状态报文。
 * @param  requestId 蓝牙请求报文ID
 * @note 在解析完成蓝牙请求和写入FLASH配置之后调用该函数
 */
void connectServer_Ethernet(uint8_t requestId) {
    uint8_t destinationIpAddress[16] = {};  // 字符串形式的目的IP地址
    uint8_t destinationPort[6] = {};        // 字符串形式的目的端口号

    struct Adapter *adapter = AdapterDeviceFindByName(ADAPTER_ETHERNET_NAME);  // 查找以太网模块适配器

    /* 如果以太网主任务线程仍未启动，则返回连接失败报文 */
    if (adapter->adapter_param == NULL) {
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x02;
        notificationReport[3] = 0x02;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
        return;
    }

    /* 获取互斥锁 */
    PrivMutexObtain(&adapter->lock);          // 若其他线程正在使用adapter，则阻塞等待
    PrivMutexObtain(&romConfigurationMutex);  // 若其他线程正在读取或者写入CFG，则阻塞等待

    /* DHCP配置 */
    if (CFG->dhcpSwitch_Ethernet == 1) {  // 使能DHCP
        AdapterDeviceSetDhcp(adapter, 1);
    } else {  // 禁用DHCP
        uint8_t sourceIpAddress[16] = {};
        uint8_t sourcePort[6] = {};
        uint8_t sourceSubnetMask[16] = {};
        uint8_t sourceGateway[16] = {};
        memcpy(((struct WchnetSocketConfiguration *)adapter->adapter_param)->sourcePort, CFG->sourcePort_Ethernet, 2);
        sprintf(sourceIpAddress, "%u.%u.%u.%u", CFG->sourceIpAddress_Ethernet[0], CFG->sourceIpAddress_Ethernet[1],
                CFG->sourceIpAddress_Ethernet[2], CFG->sourceIpAddress_Ethernet[3]);
        sprintf(sourcePort, "%u", (unsigned short)CFG->sourcePort_Ethernet[0] | CFG->sourcePort_Ethernet[1] << 8);
        sprintf(sourceSubnetMask, "%u.%u.%u.%u", CFG->sourceSubnetMask_Ethernet[0], CFG->sourceSubnetMask_Ethernet[1],
                CFG->sourceSubnetMask_Ethernet[2], CFG->sourceSubnetMask_Ethernet[3]);
        sprintf(sourceGateway, "%u.%u.%u.%u", CFG->sourceGateway_Ethernet[0], CFG->sourceGateway_Ethernet[1],
                CFG->sourceGateway_Ethernet[2], CFG->sourceGateway_Ethernet[3]);
        AdapterDeviceSetAddr(adapter, sourceIpAddress, sourceGateway, sourceSubnetMask);  // 配置静态IP地址
        AdapterDeviceSetDhcp(adapter, 0);                                                 // 禁用DHCP
    }

    /* 尝试连接到服务器 */
    sprintf(destinationIpAddress, "%u.%u.%u.%u", CFG->destinationIpAddress_Ethernet[0], CFG->destinationIpAddress_Ethernet[1],
            CFG->destinationIpAddress_Ethernet[2], CFG->destinationIpAddress_Ethernet[3]);
    sprintf(destinationPort, "%u", (unsigned short)CFG->destinationPort_Ethernet[0] | CFG->destinationPort_Ethernet[1] << 8);
    printf("-*-*-*-*-AdapterDeviceConnect*-*-*-*-\n");
    printf("destinationIpAddress: %s\n", destinationIpAddress);
    printf("destinationPort:      %s\n", destinationPort);
    printf("-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*\n");
    int res = AdapterDeviceConnect(adapter, CLIENT, destinationIpAddress, destinationPort, IPV4);

    /* 释放互斥锁 */
    PrivMutexAbandon(&romConfigurationMutex);
    PrivMutexAbandon(&adapter->lock);

    /* 蓝牙反馈连接状态 */
    if (res == 0 || res == 0x1D) {  // 网络连接成功或之前已经连接
                                    /* 发送网络连接成功报文 */
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x02;
        notificationReport[3] = 0x01;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
    } else {  // 网络连接失败
        printf("Ethernet connect to server failed\n");
        /* 发送网络连接失败报文 */
        uint8_t notificationReport[4];
        notificationReport[0] = requestId;
        notificationReport[1] = 0x02;
        notificationReport[2] = 0x02;
        notificationReport[3] = 0x02;
        bStatus_t res;
        while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
            if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
                printf("peripheralNotify failed, res: 0x%02x\n", res);
                break;
            } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
                PrivTaskDelay(10);
            }
        }
    }

    return;
}

/**
 * @brief  解析蓝牙请求报文线程
 * @param  arg   指向蓝牙请求报文的指针
 * @return void* 返回值暂无意义
 */
void *parseBleRequest(void *arg) {
    uint8_t *request = (uint8_t *)arg;  // 蓝牙请求报文指针
    uint8_t requestId = request[0];     // 请求ID
    uint8_t reportType = request[1];    // 报文类型
    uint8_t moduleType = request[2];    // 模块类型

    /* 蓝牙反馈ch32v208已收到蓝牙请求 */
    uint8_t notificationReport[3];
    notificationReport[0] = requestId;
    notificationReport[1] = 0x01;
    notificationReport[2] = 0x02;
    bStatus_t res;
    while ((res = peripheralNotify(notificationReport, sizeof(notificationReport))) != SUCCESS) {
        if (res != blePending) {  // 发送失败的原因不是阻塞，放弃继续发送
            printf("peripheralNotify failed, res: 0x%02x\n", res);
            break;
        } else {  // 发送失败的原因是阻塞，则等待10ms后再次发送
            PrivTaskDelay(10);
        }
    }

    /* 解析蓝牙请求报文  */
    PrivMutexObtain(&romConfigurationMutex);  // // 若其他线程正在读取或者写入CFG，则阻塞等待
    switch (reportType) {
        /* 4G模块配置目的IP地址、目的端口号、禁用MQTT */
        case 0x01: {
            uint8_t *destinationIpAddress_4G = request + 3;
            uint8_t *destinationPort_4G = request + 7;
            uint8_t *mqttSwitch_4G = request + 9;
            memcpy(CFG->destinationIpAddress_4G, destinationIpAddress_4G, 4);
            memcpy(CFG->destinationPort_4G, destinationPort_4G, 2);
            CFG->mqttSwitch_4G = *mqttSwitch_4G;
            /* 配置写入FLASH */
            CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
            CFG_WRITE(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);
            PrivMutexAbandon(&romConfigurationMutex);  // 释放互斥锁
            connectServer_4G(requestId);               // 尝试连接服务器
            break;
        }

        /* 4G模块配置目的IP地址、目的端口号、使能MQTT、MQTT发布主题、MQTT用户名、MQTT密码、MQTT客户端ID */
        case 0x02: {
            uint8_t *destinationIpAddress_4G = request + 3;
            uint8_t *destinationPort_4G = request + 7;
            uint8_t *mqttSwitch_4G = request + 9;
            uint8_t *mqttTopic_4G = request + 10;
            uint8_t *mqttUsername_4G = request + 74;
            uint8_t *mqttPassword_4G = request + 138;
            uint8_t *mqttClientId_4G = request + 202;
            memcpy(CFG->destinationIpAddress_4G, destinationIpAddress_4G, 4);
            memcpy(CFG->destinationPort_4G, destinationPort_4G, 2);
            CFG->mqttSwitch_4G = *mqttSwitch_4G;
            memcpy(CFG->mqttTopic_4G, mqttTopic_4G, 64);
            memcpy(CFG->mqttUsername_4G, mqttUsername_4G, 64);
            memcpy(CFG->mqttPassword_4G, mqttPassword_4G, 64);
            memcpy(CFG->mqttClientId_4G, mqttClientId_4G, 24);
            /* 配置写入FLASH */
            CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
            CFG_WRITE(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);
            PrivMutexAbandon(&romConfigurationMutex);  // 释放互斥锁
            connectServer_4G(requestId);               // 尝试连接服务器
            break;
        }

        /* 以太网模块配置目的IP地址、目的端口号、使能DHCP */
        case 0x03: {
            uint8_t *destinationIpAddress_Ethernet = request + 3;
            uint8_t *destinationPort_Ethernet = request + 7;
            uint8_t *dhcpSwitch_Ethernet = request + 9;
            memcpy(CFG->destinationIpAddress_Ethernet, destinationIpAddress_Ethernet, 4);
            memcpy(CFG->destinationPort_Ethernet, destinationPort_Ethernet, 2);
            CFG->dhcpSwitch_Ethernet = *dhcpSwitch_Ethernet;
            /* 配置写入FLASH */
            CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
            CFG_WRITE(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);
            PrivMutexAbandon(&romConfigurationMutex);  // 释放互斥锁
            connectServer_Ethernet(requestId);         // 尝试连接服务器
            break;
        }

        /* 以太网模块配置目的IP地址、目的端口号、禁用DHCP、源IP地址、源端口号、子网掩码、网关地址 */
        case 0x04: {
            uint8_t *destinationIpAddress_Ethernet = request + 3;
            uint8_t *destinationPort_Ethernet = request + 7;
            uint8_t *dhcpSwitch_Ethernet = request + 9;
            uint8_t *sourceIpAddress_Ethernet = request + 10;
            uint8_t *sourcePort_Ethernet = request + 14;
            uint8_t *sourceSubnetMask_Ethernet = request + 16;
            uint8_t *sourceGateway_Ethernet = request + 20;
            memcpy(CFG->destinationIpAddress_Ethernet, destinationIpAddress_Ethernet, 4);
            memcpy(CFG->destinationPort_Ethernet, destinationPort_Ethernet, 2);
            CFG->dhcpSwitch_Ethernet = *dhcpSwitch_Ethernet;
            memcpy(CFG->sourceIpAddress_Ethernet, sourceIpAddress_Ethernet, 4);
            memcpy(CFG->sourcePort_Ethernet, sourcePort_Ethernet, 2);
            memcpy(CFG->sourceSubnetMask_Ethernet, sourceSubnetMask_Ethernet, 4);
            memcpy(CFG->sourceGateway_Ethernet, sourceGateway_Ethernet, 4);
            /* 配置写入FLASH */
            CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
            CFG_WRITE(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);
            PrivMutexAbandon(&romConfigurationMutex);  // 释放互斥锁
            connectServer_Ethernet(requestId);         // 尝试连接服务器
            break;
        }

        /* RS485模块配置波特率、数据位、停止位、校验方式 */
        case 0x10: {
            struct SerialDataCfg rs485Configuration;
            memset(&rs485Configuration, 0, sizeof(struct SerialDataCfg));
            int baudRateOption = request[3];
            int dataBitsOption = request[4];
            int stopBitsOption = request[5];
            int parityOption = request[6];
            switch (baudRateOption) {
                case 1:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_2400;
                    break;
                case 2:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_4800;
                    break;
                case 3:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_9600;
                    break;
                case 4:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_19200;
                    break;
                case 5:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_38400;
                    break;
                case 6:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_57600;
                    break;
                case 7:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_115200;
                    break;
                case 8:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_230400;
                    break;
                default:
                    rs485Configuration.serial_baud_rate = BAUD_RATE_9600;
                    break;
            }
            switch (dataBitsOption) {
                case 1:
                    rs485Configuration.serial_data_bits = DATA_BITS_8;
                    break;
                case 2:
                    rs485Configuration.serial_data_bits = DATA_BITS_9;
                    break;
                default:
                    rs485Configuration.serial_data_bits = DATA_BITS_8;
                    break;
            }
            switch (stopBitsOption) {
                case 1:
                    rs485Configuration.serial_stop_bits = STOP_BITS_1;
                    break;
                case 2:
                    rs485Configuration.serial_stop_bits = STOP_BITS_2;
                    break;
                default:
                    rs485Configuration.serial_stop_bits = STOP_BITS_1;
                    break;
            }
            switch (parityOption) {
                case 1:
                    rs485Configuration.serial_parity_mode = PARITY_NONE;
                    break;
                case 2:
                    rs485Configuration.serial_parity_mode = PARITY_ODD;
                    break;
                case 3:
                    rs485Configuration.serial_parity_mode = PARITY_EVEN;
                    break;
            }
            struct PrivIoctlCfg ioctl_cfg;
            ioctl_cfg.ioctl_driver_type = SERIAL_TYPE;
            ioctl_cfg.args = (void *)&rs485Configuration;

            int rs485_fd = PrivOpen("/dev/rs485_dev1", O_RDWR);   // 打开设备文件
            if (0 != PrivIoctl(rs485_fd, OPE_INT, &ioctl_cfg)) {  // 进行配置
                printf("ioctl uart fd error %d\n", rs485_fd);
                PrivClose(rs485_fd);
                return NULL;
            }

            /* 配置写入FLASH */
            CFG->baudRate_Rs485 = baudRateOption;
            CFG->dataBits_Rs485 = dataBitsOption;
            CFG->stopBits_Rs485 = stopBitsOption;
            CFG->parity_Rs485 = parityOption;
            CFG_ERASE(PAGE_WRITE_START_ADDR, FLASH_PAGE_SIZE);
            CFG_WRITE(PAGE_WRITE_START_ADDR, Configbuf, MODULE_CFG_LEN);
            PrivMutexAbandon(&romConfigurationMutex);
            break;
        }

        /* 报文类型识别失败 */
        default:
            PrivMutexAbandon(&romConfigurationMutex);
            break;
    }
    PrivFree(request);  // 释放内存
    return NULL;
}

/**
 * @brief  启动解析蓝牙请求报文的线程
 * @param  request 指向蓝牙请求报文的指针
 */
void startParseBleRequestTask(uint8_t *request) {
    /* 启动蓝牙交互的线程 */
    pthread_attr_t parseBleRequestTaskAttr;
    pthread_args_t parseBleRequestTaskArgs;
    parseBleRequestTaskAttr.schedparam.sched_priority = 16;        // 线程优先级
    parseBleRequestTaskAttr.stacksize = 2048;                      // 线程栈大小
    parseBleRequestTaskArgs.pthread_name = "parseBleRequestTask";  // 线程名字
    parseBleRequestTaskArgs.arg = request;                         // 线程参数
    pthread_t parseBleRequestThread;                               // 线程ID
    PrivTaskCreate(&parseBleRequestThread, &parseBleRequestTaskAttr, parseBleRequest, &parseBleRequestTaskArgs);
    PrivTaskStartup(&parseBleRequestThread);
}
#endif /* BSP_USING_BLE */

/**
 * @brief  命令行显示配置信息
 * @return int 返回值暂无意义
 * @note 命令形式：ShowConfig
 */
int ShowConfig(void) {
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

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ShowConfig, ShowConfig, list configuration information);
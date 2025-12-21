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

#include <stdio.h>
#include <string.h>
#include <user_api.h>
#include <transform.h>
#include "hal_bsp.h"
#include "hal_base.h"

#ifdef CONFIG_LIB_USING_PIKAPYTHON
#include "../lib/pikapython/pikascript-api/pikaScript.h"
#endif

extern int FrameworkInit();
extern void ApplicationOtaTaskInit(void);
extern void RPMsgTask_Entry(void *param);

#ifdef CONFIG_LIB_USING_PIKAPYTHON
/* PikaPython application interfaces */
extern PikaObj* pikaPythonInit(void);
#endif

#ifdef OTA_BY_PLATFORM
extern int OtaTask(void);
#endif

#ifdef APPLICATION_WEBSERVER
extern int webserver(void);
#endif


static struct SPI_HANDLE g_spi_handle = {
    .config = {
        .nBytes = 1, // 8 bits
        .speed = 1*MHZ, // 1 MHz
    },
};

#define SPI_CONTROLLER_BASE    (uint32_t)SPI2   
#define TRANSFER_SIZE          4

static void Spi2GPIOConfiguration(void)
{
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_D2,
                         PIN_CONFIG_MUX_FUNC4);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3,
                         GPIO_PIN_D3,
                         PIN_CONFIG_MUX_FUNC4);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK4,
                         GPIO_PIN_B6,
                         PIN_CONFIG_MUX_FUNC4);

}

// 把MISO和MOSI短接，进行环回测试
static int spitest_loopback(void)
{
    Spi2GPIOConfiguration();
    printf("Starting SPI test...\n");
    HAL_Status status;
    uint8_t tx_buffer[TRANSFER_SIZE] = {0x20, 0x25, 0x12, 0x31}; 
    uint8_t rx_buffer[TRANSFER_SIZE] = {0};

    HAL_CRU_ClkEnable(CLK_SPI2_GATE);
    HAL_CRU_ClkEnable(PCLK_SPI2_GATE);
    HAL_CRU_ClkEnable(SCLK_IN_SPI2_GATE);
    g_spi_handle.maxFreq = HAL_SPI_MASTER_MAX_SCLK_OUT; // 50 MHz
    g_spi_handle.config.speed = 1000000; // 1 MHz
    g_spi_handle.config.nBytes = 1; // 8 bits

    status = HAL_SPI_Init(&g_spi_handle, SPI_CONTROLLER_BASE, false); // false = master
    if (status != HAL_OK) {
        goto error;
    }

    if (HAL_SPI_IsSlave(&g_spi_handle)) {
        status = HAL_ERROR;
        goto deinit;
    }

    printf("SPI Work Mode:%s\n",HAL_SPI_IsDmaXfer(&g_spi_handle) ? "DMA" : "PIO");
    printf("is Slave:%s\n",HAL_SPI_IsSlave(&g_spi_handle) ? "Yes" : "No");
    status = HAL_SPI_Configure(&g_spi_handle, tx_buffer, rx_buffer, TRANSFER_SIZE);
    if (status != HAL_OK) {
        goto deinit;
    }

    HAL_SPI_SetCS(&g_spi_handle, 0, true); // 拉低 CS, 必须设置其中一位，否则不能工作
    for (int i = 0; i < 5; i++) {
        printf("Preparing to transmit data via SPI...\n");
        HAL_SPI_FlushFifo(&g_spi_handle);

        printf("Transmitting data via SPI...\n");
        status = HAL_SPI_PioTransfer(&g_spi_handle);
        if (status != HAL_OK) {
            goto deinit;
        }
        
        printf("Received: %02X %02X %02X %02X\n", 
           rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
        UserTaskDelay(1000);        
    }


    HAL_SPI_DeInit(&g_spi_handle);
    return 0;

deinit:
    HAL_SPI_DeInit(&g_spi_handle);
error:
    
    printf("SPI test failed with status: %d\n", status);
    return -1;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0), spitest_loopback, spitest_loopback, SPI loopback test with HAL);


int main(void)
{
    printf("\nHello, world! %s\n" __TIME__);
    FrameworkInit();
    
#ifdef CONFIG_LIB_USING_PIKAPYTHON
    /* Initialize and run PikaPython */
    printf("\n initializing PikaPython...\n");
    PikaObj* pikaMain = pikaPythonInit();
    if (pikaMain != NULL) {
        printf("\n PikaPython test completed successfully!\n");
    } else {
        printf("\n PikaPython initialization failed!\n");
    }
#endif
#ifdef APPLICATION_OTA
    ApplicationOtaTaskInit();
#endif

#ifdef OTA_BY_PLATFORM
    OtaTask();
#endif

#ifdef APPLICATION_WEBSERVER
    webserver();
#endif

    RPMsgTask_Entry(NULL);
    return 0;
}
// int cppmain(void);



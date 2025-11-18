/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-07-15     Emuzit            first version
 */
#ifndef __CONNECT_SOC_H__
#define __CONNECT_SOC_H__

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#if !defined(SOC_CH567) && \
    !defined(SOC_CH568) && \
    !defined(SOC_SERIES_CH569)
#define SOC_SERIES_CH569
#endif

#ifndef __packed
#define __packed    __attribute__((packed))
#endif

//#define CHECK_STRUCT_SIZE(s, size) \
//        static_assert(sizeof(s) == size, #s " has wrong size")

#define BITS_SET(x, bits)   do x |=  bits; while(0)
#define BITS_CLR(x, bits)   do x &= ~bits; while(0)

#define FLASH_BASE_ADDRESS  0x00000000
#define RAMS_BASE_ADDRESS   0x20000000
#define BUS8_BASE_ADDRESS   0x80000000

#ifdef SOC_SERIES_CH569
#define RAMX_BASE_ADDRESS   0x20020000
#define RAMS_SIZE           16
#else
#define RAMS_SIZE           32
#endif
#define RAMS_END            (RAMS_BASE_ADDRESS + RAMS_SIZE * 1024)

#define SYS_REG_BASE        0x40001000
#define GPIO_REG_BASE       0x40001000
#define GPIO_REG_BASE_PA    0x40001040
#define GPIO_REG_BASE_PB    0x40001060

#define GPIO_PORTS          2           // 2 ports : PA & PB
#define GPIO_PA_PIN_START   0           // PA : pin number 0~31
#define GPIO_PB_PIN_START   32          // PB : pin number 32~63

#ifdef SOC_SERIES_CH569
#define GPIO_PA_PIN_MARK    0x00ffffff  // PA : bits 0~23
#define GPIO_PB_PIN_MARK    0x01ffffff  // PB : bits 0~24
#else
#define GPIO_PA_PIN_MARK    0x0000ffff  // PA : bits 0~15
#define GPIO_PB_PIN_MARK    0x00003fff  // PB : bits 0~13
#endif

#define TMR0_REG_BASE       0x40002000
#define TMR1_REG_BASE       0x40002400
#define TMR2_REG_BASE       0x40002800

#define UART0_REG_BASE      0x40003000
#define UART1_REG_BASE      0x40003400
#define UART2_REG_BASE      0x40003800
#define UART3_REG_BASE      0x40003c00

#define SPI0_REG_BASE       0x40004000
#define SPI1_REG_BASE       0x40004400

#define PWMX_REG_BASE       0x40005000

#define PFIC_REG_BASE       0xe000e000
#define SysTick_REG_BASE    0xe000f000

#ifdef SOC_SERIES_CH569
#define HSPI_REG_BASE       0x40006000  // CH569W
#define ECDC_REG_BASE       0x40007000
#define USBSS_REG_BASE      0x40008000
#define USBHS_REG_BASE      0x40009000
#define EMMC_REG_BASE       0x4000a000
#define SERDES_REG_BASE     0x4000b000
#define ETH_REG_BASE        0x4000c000  // CH565W/CH569W
#define DVP_REG_BASE        0x4000e000  // CH565W/CH565M
#else
#define LED_REG_BASE        0x40006000
#define USB0_REG_BASE       0x40008000  // CH567
#define USB1_REG_BASE       0x40009000  // CH567
#define USB_REG_BASE        0x40009000  // CH568
#define SDC_REG_BASE        0x4000a000
#define SATA_REG_BASE       0x4000b000  // CH568
#define ECDC_REG_BASE       0x4000c400
#endif

#endif

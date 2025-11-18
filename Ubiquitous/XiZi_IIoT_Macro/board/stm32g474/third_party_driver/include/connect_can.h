/**
 * @file connect_can.h
 * @brief HAL-based CAN interface for STM32G474 on XiUOS
 * @version 1.0
 * @date 2025-08-07
 * 
 * @note This file declares the CAN initialization interface for XiUOS
 *       serial framework using STM32 HAL library.
 */

#ifndef CONNECT_CAN_H
#define CONNECT_CAN_H

#include "stm32g4xx_hal_fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 每路 CAN 引脚 ===== */
#define GPIO_CAN1_RX_PIN        GPIO_PIN_8
#define GPIO_CAN1_TX_PIN        GPIO_PIN_9
#define GPIO_CAN1_RX_PORT       GPIOB
#define GPIO_CAN1_TX_PORT       GPIOB

#define GPIO_CAN2_RX_PIN        GPIO_PIN_5
#define GPIO_CAN2_TX_PIN        GPIO_PIN_6
#define GPIO_CAN2_RX_PORT       GPIOB
#define GPIO_CAN2_TX_PORT       GPIOB

#define GPIO_CAN3_RX_PIN        GPIO_PIN_8
#define GPIO_CAN3_TX_PIN        GPIO_PIN_4
#define GPIO_CAN3_RX_PORT       GPIOA
#define GPIO_CAN3_TX_PORT       GPIOB

/* ===== 每路 CAN 的发送默认参数 ===== */
#define CAN1_TX_ID        0x321
#define CAN1_TX_BYTES     8
#define CAN1_TX_IDTYPE    FDCAN_STANDARD_ID      /* 或 FDCAN_EXTENDED_ID */
#define CAN1_TX_FORMAT    FDCAN_CLASSIC_CAN      /* 或 FDCAN_FD_CAN */
#define CAN1_TX_DLC       FDCAN_DLC_BYTES_8      /* 固定 DLC；如用 FD 可换成 12/16... */

#define CAN2_TX_ID        0x322
#define CAN2_TX_BYTES     8
#define CAN2_TX_IDTYPE    FDCAN_STANDARD_ID
#define CAN2_TX_FORMAT    FDCAN_CLASSIC_CAN
#define CAN2_TX_DLC       FDCAN_DLC_BYTES_8

#define CAN3_TX_ID        0x323
#define CAN3_TX_BYTES     8
#define CAN3_TX_IDTYPE    FDCAN_STANDARD_ID
#define CAN3_TX_FORMAT    FDCAN_CLASSIC_CAN
#define CAN3_TX_DLC       FDCAN_DLC_BYTES_8

/* ===== 每路 CAN 的接收默认参数 ===== */
#define CAN1_RX_FIFO      FDCAN_RX_FIFO0
#define CAN1_RX_BYTES     8

#define CAN2_RX_FIFO      FDCAN_RX_FIFO0
#define CAN2_RX_BYTES     8

#define CAN3_RX_FIFO      FDCAN_RX_FIFO0
#define CAN3_RX_BYTES     8




/**
 * @brief Initialize CAN hardware and register with XiUOS serial framework.
 * @return int EOK on success, ERROR otherwise
 */
int InitCAN(void);

#ifdef __cplusplus
}
#endif

#endif /* CONNECT_CAN_H */

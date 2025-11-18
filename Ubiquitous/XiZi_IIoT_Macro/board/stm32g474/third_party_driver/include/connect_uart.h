/**
 * @file connect_uart.h
 * @brief HAL-based UART interface for STM32G474 on XiUOS
 * @version 1.0
 * @date 2025-08-07
 * 
 * @note This file declares the UART initialization interface for XiUOS
 *       serial framework using STM32 HAL library.
 */

#ifndef CONNECT_UART_H
#define CONNECT_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_USART1_RX      GPIO_PIN_5
#define GPIO_USART1_TX      GPIO_PIN_4
#define GPIO_USART1_PORT        GPIOC


/**
 * @brief Initialize UART hardware and register with XiUOS serial framework.
 * 
 * This function should:
 * 1. Enable USART/GPIO clocks.
 * 2. Configure TX/RX pins and alternate functions.
 * 3. Initialize the HAL UART peripheral (baudrate, mode, etc).
 * 4. Register the UART as a SerialHardwareDevice in XiUOS.
 * 5. Enable NVIC interrupt and start interrupt-based reception.
 * 
 * Call this in InitBoardHardware().
 * 
 * @return int EOK on success, ERROR otherwise
 */

/* 串口硬件配置结构体 */
struct Stm32UartHwCfg{
    USART_TypeDef *uart_device;
    IRQn_Type irq_type;
    UART_HandleTypeDef uart_handle;
};


int InitUART(void);
void USART1_IRQHandler(int irq_num, void *arg);
#ifdef __cplusplus
}
#endif

#endif /* CONNECT_UART_H */



#include "sys.h"
#include "usart.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_cortex.h"


uint8_t g_usart_rx_buf[USART_REC_LEN];


uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RXBUFFERSIZE];   

UART_HandleTypeDef g_uart1_handle;   



void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                    
    g_uart1_handle.Init.BaudRate = baudrate;               
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;   
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;        
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;         
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;   
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;            
    HAL_UART_Init(&g_uart1_handle);                        
    

    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
   
}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    if(huart->Instance == USART1)                                   
    {
        rcc_periph_clk_init_struct.PeriphClockSelection |= RCC_PERIPHCLK_USART1;
        rcc_periph_clk_init_struct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PCLK2;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init_struct);
        
        USART_UX_CLK_ENABLE();                                      
        USART_TX_GPIO_CLK_ENABLE();                                 
        USART_RX_GPIO_CLK_ENABLE();                                 

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;                   
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    
        gpio_init_struct.Pull = GPIO_PULLUP;                        
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;              
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);       
        gpio_init_struct.Pin = USART_RX_GPIO_PIN;                  
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;             
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);      

#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);
#endif                  
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)                             
    {
        if((g_usart_rx_sta & 0x8000) == 0)                    
        {
            if(g_usart_rx_sta & 0x4000)                       
            {
                if(g_rx_buffer[0] != 0x0a) 
                {
                    g_usart_rx_sta = 0;                       
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;                 
                }
            }
            else                                              
            {
                if(g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if(g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;                   
                    }
                }
            }
        }

        HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
    }
}




 

 





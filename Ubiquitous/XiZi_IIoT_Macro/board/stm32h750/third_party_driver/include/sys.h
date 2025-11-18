
#ifndef __SYS_H
#define __SYS_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc_ex.h"


#define SYS_SUPPORT_OS 0


extern RCC_PeriphCLKInitTypeDef rcc_periph_clk_init_struct;


void sys_cache_enable(void);                                                                
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);   
void sys_qspi_enable_memmapmode(void);                                                     

#endif

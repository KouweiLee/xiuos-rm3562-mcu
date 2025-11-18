/* interrupt_vector_g474_isrentry.S
 * STM32G474 — Vector table that routes *all peripheral IRQs* to IsrEntry.
 * Make sure SCB->VTOR points here; ensure bit[0] (Thumb) is 1 for each entry.
 */

    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

    .globl  InterruptVectors

    .section .isr_vector,"a",%progbits
    .type   InterruptVectors, %object

InterruptVectors:
    /* Initial MSP */
    .word   __stack_end__          /* or _estack, per your linker script */

    /* Cortex-M core exceptions */
    .word   Reset_Handler
    .word   NMI_Handler
    .word   HardFault_Handler
    .word   MemManage_Handler
    .word   BusFault_Handler
    .word   UsageFault_Handler
    .word   IsrEntry               /* Reserved */
    .word   IsrEntry               /* Reserved */
    .word   IsrEntry               /* Reserved */
    .word   IsrEntry               /* Reserved */
    .word   SVC_Handler
    .word   DebugMon_Handler
    .word   IsrEntry               /* Reserved */
    .word   PendSV_Handler
    .word   SysTick_Handler

    /* External interrupts: order strictly follows startup_stm32g474xx.s */
    .word   IsrEntry   /* WWDG_IRQHandler */
    .word   IsrEntry   /* PVD_PVM_IRQHandler */
    .word   IsrEntry   /* RTC_TAMP_LSECSS_IRQHandler */
    .word   IsrEntry   /* RTC_WKUP_IRQHandler */
    .word   IsrEntry   /* FLASH_IRQHandler */
    .word   IsrEntry   /* RCC_IRQHandler */
    .word   IsrEntry   /* EXTI0_IRQHandler */
    .word   IsrEntry   /* EXTI1_IRQHandler */
    .word   IsrEntry   /* EXTI2_IRQHandler */
    .word   IsrEntry   /* EXTI3_IRQHandler */
    .word   IsrEntry   /* EXTI4_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel1_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel2_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel3_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel4_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel5_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel6_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel7_IRQHandler */
    .word   IsrEntry   /* ADC1_2_IRQHandler */
    .word   IsrEntry   /* USB_HP_IRQHandler */
    .word   IsrEntry   /* USB_LP_IRQHandler */
    .word   IsrEntry   /* FDCAN1_IT0_IRQHandler */
    .word   IsrEntry   /* FDCAN1_IT1_IRQHandler */
    .word   IsrEntry   /* EXTI9_5_IRQHandler */
    .word   IsrEntry   /* TIM1_BRK_TIM15_IRQHandler */
    .word   IsrEntry   /* TIM1_UP_TIM16_IRQHandler */
    .word   IsrEntry   /* TIM1_TRG_COM_TIM17_IRQHandler */
    .word   IsrEntry   /* TIM1_CC_IRQHandler */
    .word   IsrEntry   /* TIM2_IRQHandler */
    .word   IsrEntry   /* TIM3_IRQHandler */
    .word   IsrEntry   /* TIM4_IRQHandler */
    .word   IsrEntry   /* I2C1_EV_IRQHandler */
    .word   IsrEntry   /* I2C1_ER_IRQHandler */
    .word   IsrEntry   /* I2C2_EV_IRQHandler */
    .word   IsrEntry   /* I2C2_ER_IRQHandler */
    .word   IsrEntry   /* SPI1_IRQHandler */
    .word   IsrEntry   /* SPI2_IRQHandler */
    .word   IsrEntry   /* USART1_IRQHandler */
    .word   IsrEntry   /* USART2_IRQHandler */
    .word   IsrEntry   /* USART3_IRQHandler */
    .word   IsrEntry   /* EXTI15_10_IRQHandler */
    .word   IsrEntry   /* RTC_Alarm_IRQHandler */
    .word   IsrEntry   /* USBWakeUp_IRQHandler */
    .word   IsrEntry   /* TIM8_BRK_IRQHandler */
    .word   IsrEntry   /* TIM8_UP_IRQHandler */
    .word   IsrEntry   /* TIM8_TRG_COM_IRQHandler */
    .word   IsrEntry   /* TIM8_CC_IRQHandler */
    .word   IsrEntry   /* ADC3_IRQHandler */
    .word   IsrEntry   /* FMC_IRQHandler */
    .word   IsrEntry   /* LPTIM1_IRQHandler */
    .word   IsrEntry   /* TIM5_IRQHandler */
    .word   IsrEntry   /* SPI3_IRQHandler */
    .word   IsrEntry   /* UART4_IRQHandler */
    .word   IsrEntry   /* UART5_IRQHandler */
    .word   IsrEntry   /* TIM6_DAC_IRQHandler */
    .word   IsrEntry   /* TIM7_DAC_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel1_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel2_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel3_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel4_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel5_IRQHandler */
    .word   IsrEntry   /* ADC4_IRQHandler */
    .word   IsrEntry   /* ADC5_IRQHandler */
    .word   IsrEntry   /* UCPD1_IRQHandler */
    .word   IsrEntry   /* COMP1_2_3_IRQHandler */
    .word   IsrEntry   /* COMP4_5_6_IRQHandler */
    .word   IsrEntry   /* COMP7_IRQHandler */
    .word   IsrEntry   /* HRTIM1_Master_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIMA_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIMB_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIMC_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIMD_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIME_IRQHandler */
    .word   IsrEntry   /* HRTIM1_FLT_IRQHandler */
    .word   IsrEntry   /* HRTIM1_TIMF_IRQHandler */
    .word   IsrEntry   /* CRS_IRQHandler */
    .word   IsrEntry   /* SAI1_IRQHandler */
    .word   IsrEntry   /* TIM20_BRK_IRQHandler */
    .word   IsrEntry   /* TIM20_UP_IRQHandler */
    .word   IsrEntry   /* TIM20_TRG_COM_IRQHandler */
    .word   IsrEntry   /* TIM20_CC_IRQHandler */
    .word   IsrEntry   /* FPU_IRQHandler */
    .word   IsrEntry   /* I2C4_EV_IRQHandler */
    .word   IsrEntry   /* I2C4_ER_IRQHandler */
    .word   IsrEntry   /* SPI4_IRQHandler */
    .word   IsrEntry   /* Reserved (0 in ST file) */
    .word   IsrEntry   /* FDCAN2_IT0_IRQHandler */
    .word   IsrEntry   /* FDCAN2_IT1_IRQHandler */
    .word   IsrEntry   /* FDCAN3_IT0_IRQHandler */
    .word   IsrEntry   /* FDCAN3_IT1_IRQHandler */
    .word   IsrEntry   /* RNG_IRQHandler */
    .word   IsrEntry   /* LPUART1_IRQHandler */
    .word   IsrEntry   /* I2C3_EV_IRQHandler */
    .word   IsrEntry   /* I2C3_ER_IRQHandler */
    .word   IsrEntry   /* DMAMUX_OVR_IRQHandler */
    .word   IsrEntry   /* QUADSPI_IRQHandler */
    .word   IsrEntry   /* DMA1_Channel8_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel6_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel7_IRQHandler */
    .word   IsrEntry   /* DMA2_Channel8_IRQHandler */
    .word   IsrEntry   /* CORDIC_IRQHandler */
    .word   IsrEntry   /* FMAC_IRQHandler */

    .size   InterruptVectors, .-InterruptVectors

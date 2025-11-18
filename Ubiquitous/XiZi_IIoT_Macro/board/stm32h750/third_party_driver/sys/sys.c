
#include "sys.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_qspi.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_pwr_ex.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_gpio.h"

RCC_PeriphCLKInitTypeDef rcc_periph_clk_init_struct = {0};



void sys_cache_enable(void)
{
    SCB_EnableICache();                 
    SCB_EnableDCache();                
    SCB->CACR |= SCB_CACR_FORCEWT_Msk;  
}



uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    RCC_OscInitTypeDef rcc_osc_init_struct = {0};
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};
    
    
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    
    
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY));
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY));
    
    
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    rcc_osc_init_struct.HSEState = RCC_HSE_ON;
    rcc_osc_init_struct.HSI48State = RCC_HSI48_ON;
    rcc_osc_init_struct.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init_struct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_struct.PLL.PLLM = pllm;
    rcc_osc_init_struct.PLL.PLLN = plln;
    rcc_osc_init_struct.PLL.PLLP = pllp;
    rcc_osc_init_struct.PLL.PLLQ = pllq;
    rcc_osc_init_struct.PLL.PLLR = 2;
    rcc_osc_init_struct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    rcc_osc_init_struct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    rcc_osc_init_struct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
    {
        return 1;
    }
 
    rcc_clk_init_struct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                    RCC_CLOCKTYPE_HCLK |
                                    RCC_CLOCKTYPE_D1PCLK1 |
                                    RCC_CLOCKTYPE_PCLK1 |
                                    RCC_CLOCKTYPE_PCLK2 |
                                    RCC_CLOCKTYPE_D3PCLK1;
    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init_struct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.AHBCLKDivider = RCC_HCLK_DIV2;
    rcc_clk_init_struct.APB3CLKDivider = RCC_APB3_DIV2;
    rcc_clk_init_struct.APB1CLKDivider = RCC_APB1_DIV2;
    rcc_clk_init_struct.APB2CLKDivider = RCC_APB2_DIV2;
    rcc_clk_init_struct.APB4CLKDivider = RCC_APB4_DIV2;
    HAL_RCC_ClockConfig(&rcc_clk_init_struct, FLASH_LATENCY_4);
    
  
    rcc_periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_PLL2_DIVP |
                                                      RCC_PERIPHCLK_PLL2_DIVQ |
                                                      RCC_PERIPHCLK_PLL2_DIVR;
    rcc_periph_clk_init_struct.PLL2.PLL2M = 5;
    rcc_periph_clk_init_struct.PLL2.PLL2N = 192;
    rcc_periph_clk_init_struct.PLL2.PLL2P = 12;
    rcc_periph_clk_init_struct.PLL2.PLL2Q = 2;
    rcc_periph_clk_init_struct.PLL2.PLL2R = 2;
    rcc_periph_clk_init_struct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    rcc_periph_clk_init_struct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    rcc_periph_clk_init_struct.PLL2.PLL2FRACN = 0;
    HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init_struct);
    
    sys_qspi_enable_memmapmode();
    
  
    HAL_SYSCFG_CompensationCodeSelect(SYSCFG_CELL_CODE);
    HAL_EnableCompensationCell();
    
    return 0;
}


void sys_qspi_enable_memmapmode(void)
{
    QSPI_HandleTypeDef qspi_handle = {0};
    QSPI_CommandTypeDef qspi_command_struct = {0};
    QSPI_AutoPollingTypeDef qspi_autopolling_struct = {0};
    QSPI_MemoryMappedTypeDef qspi_memory_mapped_struct = {0};
    MPU_Region_InitTypeDef mpu_region_init_struct = {0};
    
 
    qspi_handle.Instance = QUADSPI;
    qspi_handle.Init.ClockPrescaler = 2 - 1;
    qspi_handle.Init.FifoThreshold = 32;
    qspi_handle.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    qspi_handle.Init.FlashSize = 23;
    qspi_handle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
    qspi_handle.Init.ClockMode = QSPI_CLOCK_MODE_0;
    qspi_handle.Init.FlashID = QSPI_FLASH_ID_1;
    qspi_handle.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
    HAL_QSPI_DeInit(&qspi_handle);
    HAL_QSPI_Init(&qspi_handle);
   
    qspi_command_struct.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    qspi_command_struct.Instruction = 0x06;
    qspi_command_struct.AddressMode = QSPI_ADDRESS_NONE;
    qspi_command_struct.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspi_command_struct.DummyCycles = 0;
    qspi_command_struct.DataMode = QSPI_DATA_NONE;
    qspi_command_struct.DdrMode = QSPI_DDR_MODE_DISABLE;
    qspi_command_struct.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    qspi_command_struct.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    HAL_QSPI_Command(&qspi_handle, &qspi_command_struct, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    
    qspi_command_struct.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    qspi_command_struct.Instruction = 0x05;
    qspi_command_struct.AddressMode = QSPI_ADDRESS_NONE;
    qspi_command_struct.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspi_command_struct.DummyCycles = 0;
    qspi_command_struct.DataMode = QSPI_DATA_1_LINE;
    qspi_command_struct.DdrMode = QSPI_DDR_MODE_DISABLE;
    qspi_command_struct.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    qspi_command_struct.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    qspi_autopolling_struct.Match = 0x02;
    qspi_autopolling_struct.Mask = 0x02;
    qspi_autopolling_struct.Interval = 0x10;
    qspi_autopolling_struct.StatusBytesSize = 1;
    qspi_autopolling_struct.MatchMode = QSPI_MATCH_MODE_AND;
    qspi_autopolling_struct.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
    HAL_QSPI_AutoPolling(&qspi_handle, &qspi_command_struct, &qspi_autopolling_struct, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    
 
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = MPU_REGION_NUMBER0;
    mpu_region_init_struct.BaseAddress = QSPI_BASE;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_256MB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL0;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_Disable();
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    
 
    qspi_command_struct.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    qspi_command_struct.Instruction = 0xEB;
    qspi_command_struct.AddressMode = QSPI_ADDRESS_4_LINES;
    qspi_command_struct.AddressSize = QSPI_ADDRESS_24_BITS;
    qspi_command_struct.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspi_command_struct.DummyCycles = 6;
    qspi_command_struct.DataMode = QSPI_DATA_4_LINES;
    qspi_command_struct.DdrMode = QSPI_DDR_MODE_DISABLE;
    qspi_command_struct.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    qspi_command_struct.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    qspi_memory_mapped_struct.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    HAL_QSPI_MemoryMapped(&qspi_handle, &qspi_command_struct, &qspi_memory_mapped_struct);
}


void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    if (hqspi->Instance == QUADSPI)
    {
       
        rcc_periph_clk_init_struct.PeriphClockSelection |= RCC_PERIPHCLK_QSPI;
        rcc_periph_clk_init_struct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init_struct);
        
       
        __HAL_RCC_QSPI_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        
      
        gpio_init_struct.Pin = GPIO_PIN_6;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Alternate = GPIO_AF10_QUADSPI;
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);
        
        gpio_init_struct.Pin = GPIO_PIN_2;
        gpio_init_struct.Alternate = GPIO_AF9_QUADSPI;
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);
        
        gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio_init_struct.Alternate = GPIO_AF9_QUADSPI;
        HAL_GPIO_Init(GPIOF, &gpio_init_struct);
        
        gpio_init_struct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        gpio_init_struct.Alternate = GPIO_AF10_QUADSPI;
        HAL_GPIO_Init(GPIOF, &gpio_init_struct);
    }
}


void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
{
    if (hqspi->Instance == QUADSPI)
    {
        __HAL_RCC_QSPI_CLK_DISABLE();
        
      
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_6);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    }
}

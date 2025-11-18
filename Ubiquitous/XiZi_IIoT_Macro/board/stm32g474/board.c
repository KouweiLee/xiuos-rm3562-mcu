#include "board.h"
#include <xsconfig.h>
#include <xizi.h>       
#include <device.h>     
#include <connect_usb.h>
#include <connect_uart.h>
#include <connect_can.h>
#include <connect_spi.h>
#include <stm32g4xx.h>

// /**
//  * @brief 配置 STM32G474 的主系统时钟
//  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}



void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/**
 * @brief XiUOS 板级硬件初始化入口
 */
void InitBoardHardware(void)
{
    HAL_Init();
    SystemClock_Config();

    __set_PRIMASK(0);  // 开中断


    InitBoardMemory((void*)MEMORY_START_ADDRESS, (void*)MEMORY_END_ADDRESS);  // XiUOS 内存分配器初始化



#ifdef BSP_USING_USB
    InitUSB();       
#endif
#ifdef KERNEL_CONSOLE
    InstallConsole(USB_BUS_NAME,
                   USB_DRIVER_NAME,
                   USB_DEVICE_NAME);   
#endif

#ifdef BSP_USING_UART
    InitUART();
#endif
// #ifdef KERNEL_CONSOLE
//     InstallConsole(SERIAL1_BUS_NAME,
//                    SERIAL1_DRV_NAME,
//                    SERIAL1_DEVICE_NAME);   
// #endif


#ifdef BSP_USING_CAN
    InitCAN();
#endif

#ifdef BSP_USING_SPI
    InitSPI();
#endif

    KPrintf("InitBoardHardware: STM32G474 HAL init done.\n");
}

#ifndef YMODEM_H
#define YMODEM_H


#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>


/*=====用户配置(根据自己的分区进行配置)=====*/
#define BootLoader_Size 		0x4000U			///< BootLoader的大小 16K
#define Application_Size		0xc000U			///< 应用程序的大小 48K

#define Application_1_Addr		0x08004000U		///< 应用程序1的首地址
#define Application_2_Addr		0x08010000U		///< 应用程序2的首地址
/*==========================================*/
#define UPGRADE_FLAG_ADDR          ((uint32_t)0x0801FD00)


#define SOH		0x01
#define STX		0x02
#define ACK		0x06
#define NACK	0x15
#define EOT		0x04
#define CCC		0x43



/* 升级的步骤 */
enum UPDATE_STATE
{
	TO_START = 0x01,
	TO_RECEIVE_DATA = 0x02,
	TO_RECEIVE_EOT1 = 0x03,
	TO_RECEIVE_EOT2 = 0x04,
	TO_RECEIVE_END = 0x05
};



void ymodem_fun(void);

#endif /* YMODEM_H */

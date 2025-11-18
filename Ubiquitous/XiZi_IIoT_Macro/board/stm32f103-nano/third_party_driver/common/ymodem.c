//#include <stdio.h>
//#include <string.h>
#include <ymodem.h>
#include "stm32f1xx_hal.h"

#define Rx_Max 2048 

uint8_t Rx_Flag;
uint16_t Rx_Len;
uint8_t Rx_Buf[Rx_Max] ;	
uint8_t flag_reset;

uint64_t NotUpgrade = {0xFFFFFFFFFFFFFFFF};
uint64_t Upgrade =    {0xAAAAAAAAAAAAAAAA};

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

void ymodem_uart_init(void)
{
	// need UART1 IRQ --> void UartIsr1(int vector, void *param)
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __HAL_AFIO_REMAP_USART1_ENABLE();
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);


	/* DMA controller clock enable */
  	__HAL_RCC_DMA1_CLK_ENABLE();
	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	// MX_USART2_UART_Init();
    huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		// Error_Handler();
	}

	HAL_UART_Receive_DMA(&huart1, Rx_Buf, Rx_Max);  
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
}

/* 发送指令 */
void send_command(unsigned char command)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&command,1 , 0xFFFF);
	HAL_Delay(10);
}

/**
 * @bieaf 擦除页
 *
 * @param pageaddr  页起始地址	
 * @param num       擦除的页数
 * @return 1
 */
static int Erase_page(uint32_t pageaddr, uint32_t num)
{
	HAL_FLASH_Unlock();
	
	/* 擦除FLASH*/
	FLASH_EraseInitTypeDef FlashSet;
	FlashSet.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashSet.PageAddress = pageaddr;
	FlashSet.NbPages = num;
	
	/*设置PageError，调用擦除函数*/
	uint32_t PageError = 0;
	if(HAL_FLASHEx_Erase(&FlashSet, &PageError) == HAL_OK){
		KPrintf("erase APP2 success\r\n");
	}
	else{
		KPrintf("erase APP2 failed\r\n");
	}
	
	HAL_FLASH_Lock();
	return 1;
}


/**
 * @bieaf 写若干个数据
 *
 * @param addr       写入的地址
 * @param buff       写入数据的数组指针
 * @param word_size  长度
 * @return 
 */
static void WriteFlash(uint32_t addr, uint32_t * buff, int word_size)
{	
	/* 1/4解锁FLASH*/
	HAL_FLASH_Unlock();	
	for(int i = 0; i < word_size; i++)	
	{
		/* 3/4对FLASH烧写*/
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, buff[i]);	
	}

	/* 4/4锁住FLASH*/
	HAL_FLASH_Lock();
}


/*获取当前地址所在页*/
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}


void InFlashWrite(uint32_t Address, uint64_t data)
{
  HAL_FLASH_Unlock();//开锁
  
  uint32_t FirstPage = 0, NbOfPages = 0;
  uint32_t PageError = 0;

  FLASH_EraseInitTypeDef EraseInitStruct;
  
  FirstPage = GetPage(Address);//首页地址
  KPrintf("FirstPage = %d\r\n", FirstPage);

  NbOfPages = GetPage(Address+sizeof(data)) - FirstPage + 1;//页数
  KPrintf("NbOfPages = %d\r\n", NbOfPages);
  
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = Address;
  EraseInitStruct.NbPages     = NbOfPages;
  
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    KPrintf("ErasePageError\r\n");
  }
  
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data) == HAL_OK)
  {
    KPrintf("Flash write success\r\n");
  }
  HAL_FLASH_Lock();//上锁
  
}

/* 标记升级完成 */
void Set_Update_Down(void)
{
	unsigned int update_flag = 0xAAAAAAAA;				///< 对应bootloader的启动步骤
	WriteFlash((Application_2_Addr + Application_Size - 4), &update_flag,1 );
}



/* 临时存储的buff */
unsigned char save_buf[128] = {0};

/**
 * @bieaf CRC-16 校验
 *
 * @param addr 开始地址
 * @param num   长度
 * @param num   CRC
 * @return crc  返回CRC的值
 */
#define POLY        0x1021  
uint16_t crc16(unsigned char *addr, int num, uint16_t crc)  
{  
    int i;  
    for (; num > 0; num--)					/* Step through bytes in memory */  
    {  
        crc = crc ^ (*addr++ << 8);			/* Fetch byte from memory, XOR into CRC top byte*/  
        for (i = 0; i < 8; i++)				/* Prepare to rotate 8 bits */  
        {
            if (crc & 0x8000)				/* b15 is set... */  
                crc = (crc << 1) ^ POLY;  	/* rotate and XOR with polynomic */  
            else                          	/* b15 is clear... */  
                crc <<= 1;					/* just rotate */  
        }									/* Loop for 8 bits */  
        crc &= 0xFFFF;						/* Ensure CRC remains 16-bit value */  
    }										/* Loop until num=0 */  
    return(crc);							/* Return updated CRC */  
}



/**
 * @bieaf 获取数据包的类型, 顺便进行校验
 *
 * @param buf 开始地址
 * @param len 长度
 * @return 
 */
unsigned char Check_CRC(unsigned char* buf, int len)
{
	unsigned short crc = 0;
	
	/* 进行CRC校验 */
	if((buf[0]==0x00)&&(len >= 133))
	{
		crc = crc16(buf+3, 128, crc);
		if(crc != (buf[131]<<8|buf[132]))
		{
			return 0;///< 没通过校验
		}
		
		/* 通过校验 */
		return 1;
	}
}



/* 设置升级的步骤 */
static enum UPDATE_STATE update_state = TO_START;
void Set_state(enum UPDATE_STATE state)
{
	update_state = state;
}


/* 查询升级的步骤 */
unsigned char Get_state(void)
{
	return update_state;
}


unsigned char temp_buf[512] = {0};
uint16_t temp_len = 0;

/**
 * @bieaf YModem升级
 *
 * @param none
 * @return none
 */
void ymodem_fun(void)
{
	int i;

	KPrintf("in ymodem func\n");
	ymodem_uart_init();
	while (1)
	{
	if(Get_state()==TO_START)
	{
		KPrintf("in ymodem wait\n");
		send_command(CCC);
		HAL_Delay(1000);
	}
	if(Rx_Flag)    	// Receive flag
	{
		Rx_Flag=0;	// clean flag
				
		/* 拷贝 */
		temp_len = Rx_Len;
		KPrintf("---Rx-len: %#x--------:\r\n", Rx_Len);
		for(i = 0; i < temp_len; i++)
		{
			temp_buf[i] = Rx_Buf[i];
		}
		switch(temp_buf[0])
		{
			case SOH:///<数据包开始
			{
				static unsigned char data_state = 0;
				static unsigned int app2_size = 0;
				if(Check_CRC(temp_buf, temp_len)==1)///< 通过CRC16校验
				{					
					if((Get_state()==TO_START)&&(temp_buf[1] == 0x00)&&(temp_buf[2] == (unsigned char)(~temp_buf[1])))///< 开始
					{
						KPrintf("> Receive start...\r\n");
						Set_state(TO_RECEIVE_DATA);
						data_state = 0x01;						
						send_command(ACK);
						send_command(CCC);
						KPrintf("1111111111\r\n");

						/* 擦除App2 */							
						Erase_page(Application_2_Addr, Application_Size/1024);    // 要擦除100页
						KPrintf("2222222222\r\n");
					}
					else if((Get_state()==TO_RECEIVE_END)&&(temp_buf[1] == 0x00)&&(temp_buf[2] == (unsigned char)(~temp_buf[1])))///< 结束
					{
						KPrintf("> Receive end...\r\n");
            InFlashWrite(UPGRADE_FLAG_ADDR, Upgrade);       //将升级标志设置为升级
						KPrintf("33333333333\r\n");
						Set_state(TO_START);
						KPrintf("44444444444\r\n");
						send_command(ACK);
						KPrintf("55555555555\r\n");
					
						HAL_NVIC_SystemReset();
												
					}					
					else if((Get_state()==TO_RECEIVE_DATA)&&(temp_buf[1] == data_state)&&(temp_buf[2] == (unsigned char)(~temp_buf[1])))///< 接收数据
					{
						KPrintf("> Receive data bag:%d byte\r\n",data_state * 128);
						
						/* 烧录程序 */
						WriteFlash((Application_2_Addr + (data_state-1) * 128), (uint32_t *)(&temp_buf[3]), 32);
						data_state++;
						
						send_command(ACK);		
					}
				}
				else
				{
					KPrintf("> Notpass crc\r\n");
				}
				
			}break;
			case EOT://数据包开始
			{
				if(Get_state()==TO_RECEIVE_DATA)
				{
					KPrintf("> Receive EOT1...\r\n");
					
					Set_state(TO_RECEIVE_EOT2);					
					send_command(NACK);
				}
				else if(Get_state()==TO_RECEIVE_EOT2)
				{
					KPrintf("> Receive EOT2...\r\n");
					
					Set_state(TO_RECEIVE_END);					
					send_command(ACK);
					send_command(CCC);
				}
				else
				{
					KPrintf("> Receive EOT, But error...\r\n");
				}
			}break;	
		}
		
	}
	}
}




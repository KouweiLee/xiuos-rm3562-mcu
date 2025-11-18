/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "board.h"
#include "at32f435_437_clock.h"
#include "at32_emac.h"
#include "ethernetif.h"
#include "shell.h"
#include "xs_isr.h"
#include "lwip/sys.h"

/** @addtogroup AT32F437_periph_examples
  * @{
  */

/* Private variables ---------------------------------------------------------*/
#define MAC_ADDR_LENGTH                  (6)
static uint8_t mac_address[MAC_ADDR_LENGTH] = {0, 0, 0x44, 0x45, 0x56, 1};
// Local IP see xiuos/Ubiquitous/XiZi_IIoT/resources/ethernet/LwIP/arch/sys_arch.c

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int HwEthInit(void)
{
  error_status status;

  KPrintf("HwEthInit\n");
  status = emac_system_init();
  if(status == AT_ERROR) {
    KPrintf("emac_system_init fail\n");
    return ERROR;
  }

  lwip_set_mac_address(mac_address);

  return EOK;
}


void* ethernetif_config_enet_set(uint8_t enet_port)
{
    return NONE;
}

void Time_Update_LwIP(void)
{
    // no need to do
}


/**
  * @brief  this function handles emac handler.
  * @param  none
  * @retval none
  */
void EMAC_IRQHandler(void)
{
  static x_base eth_irq_lock;

  eth_irq_lock = DISABLE_INTERRUPT();
  if (RESET != emac_dma_interrupt_flag_get(EMAC_DMA_RI_FLAG)) {
    sys_sem_signal(get_eth_recv_sem());
    emac_dma_flag_clear(EMAC_DMA_RI_FLAG | EMAC_DMA_NIS_FLAG);
  }
  ENABLE_INTERRUPT(eth_irq_lock);
}

/*
The test program uses xiuos/Ubiquitous/XiZi_IIoT/resources/ethernet/cmd_lwip/lwip_tcp_demo.c
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(3),
     TCPSend, LwipTcpSendTest, TCPSend msg [ip:port [num [interval]]]);
  */

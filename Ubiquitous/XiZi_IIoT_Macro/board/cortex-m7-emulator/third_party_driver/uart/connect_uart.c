/* -----------------------------------------------------------------------------
 * Copyright (c) 2016 - 2017 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        26. October 2017
 * $Revision:    V1.0.3
 *
 * -------------------------------------------------------------------------- */

/**
* @file connect_uart.c
* @brief support cortex-m7-emulator uart function and register to bus framework
* @version 1.0
* @author AIIT XUOS Lab
* @date 2023-09-27
*/

/*************************************************
File name: connect_uart.c
Description: support cortex-m7-emulator uart function and register to bus framework
Others: take Arm\Packs\Keil\V2M-MPS2_CMx_BSP\1.8.0\CMSIS\Driver\USART_V2M-MPS2.c for references
History:
1. Date: 2023-09-27
Author: AIIT XUOS Lab
Modification:
1. support cortex-m7-emulator uart configure, write and read
2. support  cortex-m7-emulator uart bus device and driver register
*************************************************/

#include <board.h>
#include <connect_uart.h>
#include <CMSDK_CM7_SP.h>

#include "USART_V2M-MPS2.h"
// #include "board.h"

extern uint32_t SystemCoreClock;


#define UNUSED(x) (void)(x)           /* macro to get rid of 'unused parameter' warning */

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };


// USART0
#ifdef BSP_USING_LPUART1

// USART0 Run-Time Information
static USART_INFO          USART0_Info         = { 0U, { 0U, 0U, 0U }, 0U };
static USART_TRANSFER_INFO USART0_TransferInfo = { 0U, 0U, 0U, 0U, 0U, 0U };
static PIN                 USART0_pin_rx       = { CMSDK_GPIO1, 0U };
static PIN                 USART0_pin_tx       = { CMSDK_GPIO1, 1U };


// USART0 Resources
static const USART_RESOURCES USART0_Resources = {
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    0,  // supports UART Single-wire mode
    0,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
    0,  // RTS Flow Control available
    0,  // CTS Flow Control available
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,  // RTS Line: 0=not available, 1=available
    0,  // CTS Line: 0=not available, 1=available
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0,  // Signal RI change event: \ref ARM_USART_EVENT_RI
    0   // Reserved (must be zero)
  },
  {
    &USART0_pin_rx,
    &USART0_pin_tx
  },
  CMSDK_UART0,
  UART0RX_IRQn,
  UART0TX_IRQn,
  &USART0_Info,
  &USART0_TransferInfo
};

#endif




// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION USARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USARTx_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart) {
  return usart->capabilities;
}

/**
  \fn          int32_t USART_Initialize (      ARM_USART_SignalEvent_t  cb_event
                                         const USART_RESOURCES         *usart)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Initialize (      ARM_USART_SignalEvent_t  cb_event,
                                 const USART_RESOURCES         *usart) {

  if (usart->info->flags & USART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize callback
  usart->info->cb_event = cb_event;

  // Clear transfer information
  memset(usart->xfer, 0U, sizeof(USART_TRANSFER_INFO));

  // configure USART pins
  usart->pins.pin_rx->PinGpio->ALTFUNCSET |= (1u << usart->pins.pin_rx->PinNum);
  usart->pins.pin_tx->PinGpio->ALTFUNCSET |= (1u << usart->pins.pin_tx->PinNum);

  usart->info->flags = USART_FLAG_INITIALIZED;                  // Set status flags

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Uninitialize (const USART_RESOURCES *usart)
  \brief       De-initialize USART Interface.
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Uninitialize (const USART_RESOURCES *usart) {

  // unconfigure USART pins
  usart->pins.pin_rx->PinGpio->ALTFUNCSET &= ~(1u << usart->pins.pin_rx->PinNum);
  usart->pins.pin_tx->PinGpio->ALTFUNCSET &= ~(1u << usart->pins.pin_tx->PinNum);

  usart->info->flags = 0U;                                      // Reset status flags

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_PowerControl (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_PowerControl (      ARM_POWER_STATE  state,
                                   const USART_RESOURCES *usart) {

  switch (state) {
    case ARM_POWER_OFF:
      // Clear powered flag
      usart->info->flags &= ~USART_FLAG_POWERED;

      // Clear and disable USART IRQ
      NVIC_DisableIRQ(usart->irq_num_rx);
      NVIC_ClearPendingIRQ(usart->irq_num_rx);
      NVIC_DisableIRQ(usart->irq_num_tx);
      NVIC_ClearPendingIRQ(usart->irq_num_tx);
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((usart->info->flags & USART_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // Initialize status flags
      usart->info->status.tx_busy       = 0U;
      usart->info->status.rx_busy       = 0U;
      usart->info->status.rx_overflow   = 0U;

      // Set flag initialized
      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

      // Clear and enable USART IRQ
      NVIC_ClearPendingIRQ(usart->irq_num_rx);
      NVIC_EnableIRQ(usart->irq_num_rx);
      NVIC_ClearPendingIRQ(usart->irq_num_tx);
      NVIC_EnableIRQ(usart->irq_num_tx);
      break;

//    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Send (const void            *data,
                                         uint32_t         num,
                                   const USART_RESOURCES *usart)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Send (const void            *data,
                                 uint32_t         num,
                           const USART_RESOURCES *usart) {

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if ((usart->info->status.tx_busy != 0U) ||
     ((usart->reg->STATE & CMSDK_UART_STATE_TXBF_Msk) != 0U)) {
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Set tx_busy flag
  usart->info->status.tx_busy = 0U;

  // Save transmit buffer info
  usart->xfer->tx_buf = (const uint8_t *)data;
  usart->xfer->tx_num = num;
  usart->xfer->tx_cnt = 0U;

  // TX interrupt enable
  usart->reg->CTRL |= CMSDK_UART_CTRL_TXIRQEN_Msk;

  // Start sending
  usart->reg->DATA = usart->xfer->tx_buf[usart->xfer->tx_cnt];

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Receive (      void            *data,
                                            uint32_t         num,
                                      const USART_RESOURCES *usart)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Receive (      void            *data,
                                    uint32_t         num,
                              const USART_RESOURCES *usart) {

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  uint32_t state = 0;
    state = usart->reg->STATE;
    int ch=-1;
    if (state){
       ch=usart->reg->DATA& 0xff;
        usart->reg->STATE=0;
    }
    

  return ch;
}

/**
  \fn          int32_t USART_Transfer (const void             *data_out,
                                             void             *data_in,
                                             uint32_t          num,
                                       const USART_RESOURCES  *usart)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Transfer (const void             *data_out,
                                     void             *data_in,
                                     uint32_t          num,
                               const USART_RESOURCES  *usart) {
  UNUSED(data_out);
  UNUSED(data_in);
  UNUSED(num);
  UNUSED(usart);

  // Synchronous mode not supported
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          uint32_t USART_GetTxCount (const USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items transmitted
*/
static uint32_t USART_GetTxCount (const USART_RESOURCES *usart) {

  return usart->xfer->tx_cnt;

}

/**
  \fn          uint32_t USART_GetRxCount (const USART_RESOURCES *usart)
  \brief       Get received data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items received
*/
static uint32_t USART_GetRxCount (const USART_RESOURCES *usart) {

  return usart->xfer->rx_cnt;

}

/**
  \fn          int32_t USART_Control (      uint32_t          control,
                                            uint32_t          arg,
                                      const USART_RESOURCES  *usart)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   usart    Pointer to USART resources
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART_Control (      uint32_t          control,
                                    uint32_t          arg,
                              const USART_RESOURCES  *usart) {

  if ((usart->info->flags & USART_FLAG_POWERED) == 0U) {
    // USART not powered
    return ARM_DRIVER_ERROR;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    // Control break
    case ARM_USART_CONTROL_BREAK:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    // Abort Send
    case ARM_USART_ABORT_SEND:
      // TX interrupt disable
      usart->reg->CTRL &= ~CMSDK_UART_CTRL_TXIRQEN_Msk;
      // Clear tx_busy status
      usart->info->status.tx_busy = 0U;
      return ARM_DRIVER_OK;

    // Abort receive
    case ARM_USART_ABORT_RECEIVE:
      // RX interrupt disable
      usart->reg->CTRL &= ~CMSDK_UART_CTRL_RXIRQEN_Msk;
      // Clear rx_busy status
      usart->info->status.rx_busy = 0U;
      return ARM_DRIVER_OK;

    // Abort transfer
    case ARM_USART_ABORT_TRANSFER:
      // Synchronous mode not supported
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    // Control TX
    case ARM_USART_CONTROL_TX:
      usart->info->flags |= USART_FLAG_TX_ENABLED;
      // Transmitter enable
      usart->reg->CTRL |= ((arg & 1U) << CMSDK_UART_CTRL_TXEN_Pos);
      return ARM_DRIVER_OK;

    // Control RX
    case ARM_USART_CONTROL_RX:
      usart->info->flags |= USART_FLAG_RX_ENABLED;
      // Receiver enable
      usart->reg->CTRL |= ((arg & 1U) << CMSDK_UART_CTRL_RXEN_Pos);
      return ARM_DRIVER_OK;
    default: break;
  }

  // Check if busy
  if ((usart->info->status.tx_busy != 0U) ||
      (usart->info->status.rx_busy != 0U) ||
     ((usart->reg->STATE & (CMSDK_UART_STATE_TXBF_Msk | CMSDK_UART_STATE_RXBF_Msk)) != 0U)) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_ASYNCHRONOUS: break;
    // Unsupported command
    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  // USART Data bits
  switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_8: break;
    default: return ARM_USART_ERROR_DATA_BITS;
  }

  // USART Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE: break;
    default: return ARM_USART_ERROR_PARITY;
  }

  // USART Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1: break;
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // USART Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE: break;
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // USART Baudrate
  if ((SystemCoreClock/arg) < 16U)
    return ARM_USART_ERROR_BAUDRATE;
  else
    usart->reg->BAUDDIV = SystemCoreClock/arg;

  // Set configured flag
  usart->info->flags |= USART_FLAG_CONFIGURED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart) {
  ARM_USART_STATUS status;

  status.tx_busy          = usart->info->status.tx_busy;
  status.rx_busy          = usart->info->status.rx_busy;
  status.rx_overflow      = usart->info->status.rx_overflow;

  return status;
}

/**
  \fn          int32_t USART_SetModemControl (      ARM_USART_MODEM_CONTROL  control,
                                              const USART_RESOURCES         *usart)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_SetModemControl (      ARM_USART_MODEM_CONTROL  control,
                                      const USART_RESOURCES         *usart) {
  UNUSED(control);
  UNUSED(usart);

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart)
  \brief       Get USART Modem Status lines state.
  \param[in]   usart     Pointer to USART resources
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart) {
  ARM_USART_MODEM_STATUS modem_status = { 0U, 0U, 0U, 0U };

  UNUSED(usart);

  return modem_status;
}


static void UartIsr(struct SerialBus *serial, struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev);

#ifdef BSP_USING_LPUART1
struct SerialBus serial_bus_1;
struct SerialDriver serial_driver_1;
struct SerialHardwareDevice serial_device_1;
#endif



/**
  \fn          void USARTRX_IRQHandler (UART_RESOURCES *usart)
  \brief       USART RX Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
static void USARTRX_IRQHandler (USART_RESOURCES *usart) {
  uint32_t status;

  status = usart->reg->INTSTATUS;

 usart->xfer->rx_num = 0U;
  usart->info->status.rx_busy = 0U;
  usart->reg->CTRL &= ~CMSDK_UART_CTRL_RXIRQEN_Msk;
  usart->reg->INTCLEAR = status;

#ifdef BSP_USING_LPUART1
    DisableIRQ(UART0RX_IRQn);
    UartIsr(&serial_bus_1, &serial_driver_1, &serial_device_1);
    EnableIRQ(UART0RX_IRQn);
#endif

    usart->reg->BAUDDIV = 16;
    usart->reg->CTRL = CMSDK_UART_CTRL_RXIRQEN_Msk | CMSDK_UART_CTRL_RXEN_Msk | CMSDK_UART_CTRL_TXEN_Msk;
    usart->reg->STATE = 0U;

}

/**
  \fn          void USARTTX_IRQHandler (UART_RESOURCES *usart)
  \brief       USART TX Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
static void USARTTX_IRQHandler (USART_RESOURCES *usart) {
  uint32_t status;
    
  // read interrupt status
  status = usart->reg->INTSTATUS;

  // clear interrupt
  usart->reg->INTCLEAR = status;

  if (usart->xfer->tx_num > 0) {

    usart->xfer->tx_cnt++;

    if (usart->xfer->tx_cnt < usart->xfer->tx_num) {
      // Send next byte
      usart->reg->DATA = usart->xfer->tx_buf[usart->xfer->tx_cnt];
    } else {
      // TX is completed
      usart->xfer->tx_num = 0U;
      usart->info->status.tx_busy = 0U;

      // TX interrupt disable
      usart->reg->CTRL &= ~CMSDK_UART_CTRL_TXIRQEN_Msk;

      // Send event
      if (usart->info->cb_event!=NULL)
        usart->info->cb_event(ARM_USART_EVENT_SEND_COMPLETE);
    }
  }

}

#ifdef BSP_USING_LPUART1
// USART0 Driver Wrapper functions
       void                    UART0RX_Handler        (void);
       void                    UART0TX_Handler        (void);
static ARM_USART_CAPABILITIES  USART0_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART0_Resources); }
static int32_t                 USART0_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART0_Resources); }
static int32_t                 USART0_Uninitialize    (void)                                                { return USART_Uninitialize (&USART0_Resources); }
static int32_t                 USART0_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART0_Resources); }
static int32_t                 USART0_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART0_Resources); }
static int32_t                 USART0_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART0_Resources); }
static int32_t                 USART0_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART0_Resources); }
static uint32_t                USART0_GetTxCount      (void)                                                { return USART_GetTxCount (&USART0_Resources); }
static uint32_t                USART0_GetRxCount      (void)                                                { return USART_GetRxCount (&USART0_Resources); }
static int32_t                 USART0_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART0_Resources); }
static ARM_USART_STATUS        USART0_GetStatus       (void)                                                { return USART_GetStatus (&USART0_Resources); }
static int32_t                 USART0_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART0_Resources); }
static ARM_USART_MODEM_STATUS  USART0_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART0_Resources); }
       void                    UART0RX_Handler        (void)                                                {        USARTRX_IRQHandler (&USART0_Resources); }
       void                    UART0TX_Handler        (void)                                                {        USARTTX_IRQHandler (&USART0_Resources); }

// USART0 Driver Control Block
extern ARM_DRIVER_USART Driver_USART0;
       ARM_DRIVER_USART Driver_USART0 = {
    USARTx_GetVersion,
    USART0_GetCapabilities,
    USART0_Initialize,
    USART0_Uninitialize,
    USART0_PowerControl,
    USART0_Send,
    USART0_Receive,
    USART0_Transfer,
    USART0_GetTxCount,
    USART0_GetRxCount,
    USART0_Control,
    USART0_GetStatus,
    USART0_SetModemControl,
    USART0_GetModemStatus
};
#endif




#define _USART_Driver_(n)  Driver_USART##n
#define  USART_Driver_(n) _USART_Driver_(n)
 
extern ARM_DRIVER_USART  USART_Driver_(0);
#define ptrUSART1       (&USART_Driver_(0))

#define USART_BAUDRATE          115200



static void SerialCfgParamCheck(struct SerialCfgParam *serial_cfg_default, struct SerialCfgParam *serial_cfg_new)
{
    struct SerialDataCfg *data_cfg_default = &serial_cfg_default->data_cfg;
    struct SerialDataCfg *data_cfg_new = &serial_cfg_new->data_cfg;

    if ((data_cfg_default->serial_baud_rate != data_cfg_new->serial_baud_rate) && (data_cfg_new->serial_baud_rate)) {
        data_cfg_default->serial_baud_rate = data_cfg_new->serial_baud_rate;
    }

    if ((data_cfg_default->serial_bit_order != data_cfg_new->serial_bit_order) && (data_cfg_new->serial_bit_order)) {
        data_cfg_default->serial_bit_order = data_cfg_new->serial_bit_order;
    }

    if ((data_cfg_default->serial_buffer_size != data_cfg_new->serial_buffer_size) && (data_cfg_new->serial_buffer_size)) {
        data_cfg_default->serial_buffer_size = data_cfg_new->serial_buffer_size;
    }

    if ((data_cfg_default->serial_data_bits != data_cfg_new->serial_data_bits) && (data_cfg_new->serial_data_bits)) {
        data_cfg_default->serial_data_bits = data_cfg_new->serial_data_bits;
    }

    if ((data_cfg_default->serial_invert_mode != data_cfg_new->serial_invert_mode) && (data_cfg_new->serial_invert_mode)) {
        data_cfg_default->serial_invert_mode = data_cfg_new->serial_invert_mode;
    }

    if ((data_cfg_default->serial_parity_mode != data_cfg_new->serial_parity_mode) && (data_cfg_new->serial_parity_mode)) {
        data_cfg_default->serial_parity_mode = data_cfg_new->serial_parity_mode;
    }

    if ((data_cfg_default->serial_stop_bits != data_cfg_new->serial_stop_bits) && (data_cfg_new->serial_stop_bits)) {
        data_cfg_default->serial_stop_bits = data_cfg_new->serial_stop_bits;
    }

    if ((data_cfg_default->serial_timeout != data_cfg_new->serial_timeout) && (data_cfg_new->serial_timeout)) {
        data_cfg_default->serial_timeout = data_cfg_new->serial_timeout;
    }
}

static void UartIsr(struct SerialBus *serial, struct SerialDriver *serial_drv, struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;

    SerialSetIsr(serial_dev, SERIAL_EVENT_RX_IND);
    
}



static uint32 SerialInit(struct SerialDriver *serial_drv, struct BusConfigureInfo *configure_info)
{
  NULL_PARAM_CHECK(serial_drv);

  struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
  // LPUART_Type *uart_base = (LPUART_Type *)serial_cfg->hw_cfg.private_data;

	struct SerialHardwareDevice *serial_dev = (struct SerialHardwareDevice *)serial_drv->driver.owner_bus->owner_haldev;
	struct SerialDevParam *dev_param = (struct SerialDevParam *)serial_dev->haldev.private_data;

  if (configure_info->private_data) {
      struct SerialCfgParam *serial_cfg_new = (struct SerialCfgParam *)configure_info->private_data;
      SerialCfgParamCheck(serial_cfg, serial_cfg_new);

      if (serial_cfg_new->data_cfg.dev_recv_callback) {
          BusDevRecvCallback(&(serial_dev->haldev), serial_cfg_new->data_cfg.dev_recv_callback);
      }
  }

	// config serial receive sem timeout
	dev_param->serial_timeout = serial_cfg->data_cfg.serial_timeout;

    int32_t status;
 
    status = ptrUSART1->Initialize(NULL);
    if (status != ARM_DRIVER_OK) return (-1);
    
    status = ptrUSART1->PowerControl(ARM_POWER_FULL);
    if (status != ARM_DRIVER_OK) return (-1);
    
    status = ptrUSART1->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                ARM_USART_DATA_BITS_8       |
                                ARM_USART_PARITY_NONE       |
                                ARM_USART_STOP_BITS_1       |
                                ARM_USART_FLOW_CONTROL_NONE,
                                USART_BAUDRATE);
    if (status != ARM_DRIVER_OK) return (-1);

    status = ptrUSART1->Control(ARM_USART_CONTROL_TX, 1);
    if (status != ARM_DRIVER_OK) return (-1);

    status = ptrUSART1->Control(ARM_USART_CONTROL_RX, 1);
    if (status != ARM_DRIVER_OK) return (-1);
    // stdout_init();
    // stdout_putchar(36);

    if (configure_info->private_data) {
        DisableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
        // LPUART_EnableInterrupts(uart_base, kLPUART_RxDataRegFullInterruptEnable);
        NVIC_SetPriority(serial_cfg->hw_cfg.serial_irq_interrupt, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
        EnableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
    }

    

    return EOK;
}

static uint32 SerialConfigure(struct SerialDriver *serial_drv, int serial_operation_cmd)
{
    NULL_PARAM_CHECK(serial_drv);

    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_drv->private_data;
    // LPUART_Type *uart_base = (LPUART_Type *)serial_cfg->hw_cfg.private_data;
    struct BusConfigureInfo configure_info;
    configure_info.private_data = NONE;

    SerialInit(serial_drv, &configure_info);

    switch (serial_operation_cmd)
    {
        case OPER_CLR_INT:
            DisableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
            break;

        case OPER_SET_INT:
            // LPUART_EnableInterrupts(uart_base, kLPUART_RxDataRegFullInterruptEnable);
            NVIC_SetPriority(serial_cfg->hw_cfg.serial_irq_interrupt, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
            EnableIRQ(serial_cfg->hw_cfg.serial_irq_interrupt);
            break;
    }
    
    return EOK;
}


static int SerialPutChar(struct SerialHardwareDevice *serial_dev, char c)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
    
    uint8_t buf[1];
 
    buf[0] = c;
    if (ptrUSART1->Send(buf, 1) != ARM_DRIVER_OK) {
        return (-1);
    }

    UART0TX_Handler();


    return 1;
}

static int SerialGetChar(struct SerialHardwareDevice *serial_dev)
{
    struct SerialCfgParam *serial_cfg = (struct SerialCfgParam *)serial_dev->private_data;
   
    int c=-1;
    int buf[1];
    buf[0]=0;
    do{
        buf[0]=ptrUSART1->Receive(buf, 1);
    }while (buf[0]==0);
  
    c=buf[0];


    return c;
}

static uint32 SerialDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    int serial_operation_cmd;
    struct SerialDriver *serial_drv = (struct SerialDriver *)drv;

    switch (configure_info->configure_cmd)
    {
        case OPE_INT:
            ret = SerialInit(serial_drv, configure_info);
            break;
        case OPE_CFG:
            serial_operation_cmd = *(int *)configure_info->private_data;
            ret = SerialConfigure(serial_drv, serial_operation_cmd);
            break;
        default:
            break;
    }

    return ret;
}

static const struct SerialDataCfg data_cfg_init = 
{
    .serial_baud_rate = BAUD_RATE_115200,
    .serial_data_bits = DATA_BITS_8,
    .serial_stop_bits = STOP_BITS_1,
    .serial_parity_mode = PARITY_NONE,
    .serial_bit_order = BIT_ORDER_LSB,
    .serial_invert_mode = NRZ_NORMAL,
    .serial_buffer_size = SERIAL_RB_BUFSZ,
    .serial_timeout = WAITING_FOREVER,
};

/*manage the serial device operations*/
static const struct SerialDrvDone drv_done =
{
    .init = SerialInit,
    .configure = SerialConfigure,
};

/*manage the serial device hal operations*/
static struct SerialHwDevDone hwdev_done =
{
    .put_char = SerialPutChar,
    .get_char = SerialGetChar,
};

static int BoardSerialBusInit(struct SerialBus *serial_bus, struct SerialDriver *serial_driver, const char *bus_name, const char *drv_name)
{
    x_err_t ret = EOK;

    /*Init the serial bus */
    ret = SerialBusInit(serial_bus, bus_name);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit SerialBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the serial driver*/
    ret = SerialDriverInit(serial_driver, drv_name);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit SerialDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the serial driver to the serial bus*/
    ret = SerialDriverAttachToBus(drv_name, bus_name);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit SerialDriverAttachToBus error %d\n", ret);
        return ERROR;
    } 

    return ret;
}

/*Attach the serial device to the serial bus*/
static int BoardSerialDevBend(struct SerialHardwareDevice *serial_device, void *serial_param, const char *bus_name, const char *dev_name)
{
    x_err_t ret = EOK;

    ret = SerialDeviceRegister(serial_device, serial_param, dev_name);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit SerialDeviceInit device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    ret = SerialDeviceAttachToBus(dev_name, bus_name);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit SerialDeviceAttachToBus device %s error %d\n", dev_name, ret);
        return ERROR;
    }  

    return  ret;
}

int Imxrt1052HwUartInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_LPUART1
    static struct SerialCfgParam serial_cfg_1;
    memset(&serial_cfg_1, 0, sizeof(struct SerialCfgParam));

    static struct SerialDevParam serial_dev_param_1;
    memset(&serial_dev_param_1, 0, sizeof(struct SerialDevParam));
    
    serial_driver_1.drv_done = &drv_done;
    serial_driver_1.configure = &SerialDrvConfigure;
    serial_device_1.hwdev_done = &hwdev_done;

    serial_cfg_1.data_cfg = data_cfg_init;

    CMSDK_UART0->BAUDDIV = 16;
    CMSDK_UART0->CTRL = CMSDK_UART_CTRL_RXIRQEN_Msk | CMSDK_UART_CTRL_RXEN_Msk | CMSDK_UART_CTRL_TXEN_Msk;
    CMSDK_UART0->STATE = 0U;

    serial_cfg_1.hw_cfg.private_data   = (void *)CMSDK_UART0;
    serial_cfg_1.hw_cfg.serial_irq_interrupt = UART0RX_IRQn ;
    serial_driver_1.private_data = (void *)&serial_cfg_1;

    

    serial_dev_param_1.serial_work_mode = SIGN_OPER_INT_RX;
    serial_device_1.haldev.private_data = (void *)&serial_dev_param_1;

    ret = BoardSerialBusInit(&serial_bus_1, &serial_driver_1, SERIAL_BUS_NAME_1, SERIAL_DRV_NAME_1);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit uart error ret %u\n", ret);
        return ERROR;
    }

    ret = BoardSerialDevBend(&serial_device_1, (void *)&serial_cfg_1, SERIAL_BUS_NAME_1, SERIAL_1_DEVICE_NAME_0);
    if (EOK != ret) {
        KPrintf("Imxrt1052HwUartInit uart error ret %u\n", ret);
        return ERROR;
    }    
#endif



    return ret;
}

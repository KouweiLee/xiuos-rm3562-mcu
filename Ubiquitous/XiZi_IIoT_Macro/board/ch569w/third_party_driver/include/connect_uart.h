/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
* @file connect_uart.h
* @brief define rvstar uart function 
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-08-01
*/

#ifndef CONNECT_UART_H
#define CONNECT_UART_H

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif


union _uart_mcr
{
    uint8_t reg;
    struct
    {
        uint8_t dtr         : 1;  // B.0 : RW, DTR output (UART0 only)
        uint8_t rts         : 1;  // B.1 : RW, RTS output (UART0 only)
        uint8_t out1        : 1;  // B.2 : RW, user defined modem control (UART0 only)
        uint8_t int_oe      : 1;  // B.3 : RW, interrupt output enable / OUT2
        uint8_t loop        : 1;  // B.4 : RW, enable internal loop test (UART0 only)
        uint8_t au_flow_en  : 1;  // B.5 : RW, enable CTS/RTS autoflow control
        uint8_t tnow        : 1;  // B.6 : RW, enable DTR TNOW output (UART0 only)
        uint8_t half        : 1;  // B.7 : RW, enable half-duplex mode (UART0 only)
    };
};

union _uart_ier
{
    uint8_t reg;
    struct
    {
        uint8_t recv_rdy    : 1;  // B.0 : RW, enable rx data ready intr
        uint8_t thr_empty   : 1;  // B.1 : RW, enable THR empty intr
        uint8_t line_stat   : 1;  // B.2 : RW, enable rx line status intr
        uint8_t modem_chg   : 1;  // B.3 : RW, enable modem status change intr (UART0 only)
        uint8_t dtr_en      : 1;  // B.4 : RW, DTR/TNOW output pin enable (UART0 only)
        uint8_t rts_en      : 1;  // B.5 : RW, RTS output pin enable (UART0 only)
        uint8_t txd_en      : 1;  // B.6 : RW, TXD pin enable
        uint8_t reset       : 1;  // B.7 : WZ, software reset control, active high, auto clear
    };
};

union _uart_fcr
{
    uint8_t reg;
    struct
    {
        uint8_t fifo_en     : 1;  // B.0   : RW, FIFO enable
        uint8_t rx_fifo_clr : 1;  // B.1   : WZ, write 1 to clear rx FIFO, auto clear
        uint8_t tx_fifo_clr : 1;  // B.2   : WZ, write 1 to clear tx FIFO, auto clear
        uint8_t resv_3      : 3;
        uint8_t fifo_trig   : 2;  // B.7-6 : RW, rx FIFO trigger level, 1/2/4/7 bytes
    };
};

union _uart_lcr
{
    uint8_t reg;
    struct
    {
        uint8_t word_sz     : 2;  // B.1-0 : RW, word bit length, 5/6/7/8 bits
        uint8_t stop_bit    : 1;  // B.2   : RW, stop bit length, 1/2 bits
        uint8_t par_en      : 1;  // B.3   : RW, parity enable
        uint8_t par_mod     : 2;  // B.5-4 : RW, parity mode, odd/even/mark/space
        uint8_t break_en    : 1;  // B.6   : RW, force BREAK line condition
        uint8_t dlab        : 1;  // B.7   : RW, user defined general purpose bit
    };
};


#define LCR_DATA_BITS_5     0
#define LCR_DATA_BITS_6     1
#define LCR_DATA_BITS_7     2
#define LCR_DATA_BITS_8     3

#define LCR_STOP_BITS_1     0
#define LCR_STOP_BITS_2     1

#define LCR_PARITY_ODD      0
#define LCR_PARITY_EVEN     1
#define LCR_PARITY_MARK     2
#define LCR_PARITY_SPACE    3

union _uart_iir
{
    uint8_t reg;
    struct
    {
        uint8_t int_mask    : 4;  // B.3-0 : RO, interrupt mask (intr if B.0 is 0)
        uint8_t resv_4      : 2;
        uint8_t fifo_id     : 2;  // B.7-6 : RO, FIFO enabled flag
    };
};


union _uart_lsr
{
    uint8_t reg;
    struct
    {
        uint8_t data_rdy    : 1;  // B.0 : RO, rx FIFO data ready
        uint8_t over_err    : 1;  // B.1 : RZ, rx FIFO data overrun
        uint8_t par_err     : 1;  // B.2 : RZ, rx parity error
        uint8_t frame_err   : 1;  // B.3 : RZ, rx frame error
        uint8_t break_err   : 1;  // B.4 : RZ, rx BREAK detected
        uint8_t tx_fifo_emp : 1;  // B.5 : RO, tx FIFO empty
        uint8_t tx_all_emp  : 1;  // B.6 : RO, THR/TSR all empty
        uint8_t err_rx_fifo : 1;  // B.7 : RO, PAR/FRAME/BREAK ERR in rx FIFO
    };
};

union _uart_msr
{
    uint8_t reg;
    struct
    {
        uint8_t cts_chg     : 1;  // B.0 : RZ, CTS input changed
        uint8_t dsr_chg     : 1;  // B.1 : RZ, DSR input changed
        uint8_t ri_chg      : 1;  // B.2 : RZ, RI  input changed
        uint8_t dcd_chg     : 1;  // B.3 : RZ, DCD input changed
        uint8_t cts         : 1;  // B.4 : RO, CTS action status
        uint8_t dsr         : 1;  // B.5 : RO, DSR action status
        uint8_t ri          : 1;  // B.6 : RO, RI  action status
        uint8_t dcd         : 1;  // B.7 : RO, DCD action status
    };
};

struct uart_registers
{
    union _uart_mcr MCR;
    union _uart_ier IER;
    union _uart_fcr FCR;
    union _uart_lcr LCR;
    union _uart_iir IIR;
    union _uart_lsr LSR;
    union _uart_lsr MSR;
    uint8_t         resv_7;
    union
    {
        uint8_t     RBR;
        uint8_t     THR;
    };
    uint8_t         resv_9;
    uint8_t         RFC;
    uint8_t         TFC;
    uint16_t        DL;
    uint8_t         DIV;
    uint8_t         ADR;
} __packed;


int InitHwUart(void);

#ifdef __cplusplus
}
#endif

#endif
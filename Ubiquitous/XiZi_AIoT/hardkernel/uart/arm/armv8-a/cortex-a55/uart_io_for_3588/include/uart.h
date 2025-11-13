#pragma once

#include "memlayout.h"
#include "mmio_access.h"

#define UART0_BASE (0x09000000ULL)
#define UART0_REG(reg) ((volatile uint32_t*)(MMIO_P2V_WO(UART0_BASE + reg)))

// the UART control registers.
// pl011
#define DR 0x00
#define FR 0x18
#define FR_RXFE (1 << 4) // recieve fifo empty
#define FR_TXFF (1 << 5) // transmit fifo full
#define FR_RXFF (1 << 6) // recieve fifo full
#define FR_TXFE (1 << 7) // transmit fifo empty
#define IBRD 0x24
#define FBRD 0x28
#define LCRH 0x2c
#define LCRH_FEN (1 << 4)
#define LCRH_WLEN_8BIT (3 << 5)
#define CR 0x30
#define IMSC 0x38
#define INT_RX_ENABLE (1 << 4)
#define INT_TX_ENABLE (1 << 5)
#define ICR 0x44

#define UART_READ_REG(reg) (*(UART0_REG(reg)))
#define UART_WRITE_REG(reg, v) (*(UART0_REG(reg)) = (v))
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

/// this file is only used for debug
#include <stddef.h>
#include <stdint.h>

#include "libserial.h"
#include "usyscall.h"

/*
 * For driver model we always use one byte per register, and sort out the
 * differences in the driver
 */
#define CONFIG_SYS_NS16550_REG_SIZE (-1)

#define UART_REG(x)  \
    unsigned char x; \
    unsigned char postpad_##x[-CONFIG_SYS_NS16550_REG_SIZE - 1];

/**
 * struct ns16550_platdata - information about a NS16550 port
 *
 * @base:		Base register address
 * @reg_shift:		Shift size of registers (0=byte, 1=16bit, 2=32bit...)
 * @clock:		UART base clock speed in Hz
 */
struct ns16550_platdata {
    unsigned long base;
    int reg_shift;
    int clock;
    int reg_offset;
    uint32_t fcr;
};

struct udevice;

struct NS16550 {
    UART_REG(rbr); /* 0 */
    UART_REG(ier); /* 1 */
    UART_REG(fcr); /* 2 */
    UART_REG(lcr); /* 3 */
    UART_REG(mcr); /* 4 */
    UART_REG(lsr); /* 5 */
    UART_REG(msr); /* 6 */
    UART_REG(spr); /* 7 */
#ifdef CONFIG_SOC_DA8XX
    UART_REG(reg8); /* 8 */
    UART_REG(reg9); /* 9 */
    UART_REG(revid1); /* A */
    UART_REG(revid2); /* B */
    UART_REG(pwr_mgmt); /* C */
    UART_REG(mdr1); /* D */
#else
    UART_REG(mdr1); /* 8 */
    UART_REG(reg9); /* 9 */
    UART_REG(regA); /* A */
    UART_REG(regB); /* B */
    UART_REG(regC); /* C */
    UART_REG(regD); /* D */
    UART_REG(regE); /* E */
    UART_REG(uasr); /* F */
    UART_REG(scr); /* 10*/
    UART_REG(ssr); /* 11*/
#endif
#ifdef CONFIG_DM_SERIAL
    struct ns16550_platdata* plat;
#endif
};

#define thr rbr
#define iir fcr
#define dll rbr
#define dlm ier

typedef struct NS16550* NS16550_t;

/*
 * These are the definitions for the FIFO Control Register
 */
#define UART_FCR_FIFO_EN 0x01 /* Fifo enable */
#define UART_FCR_CLEAR_RCVR 0x02 /* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT 0x04 /* Clear the XMIT FIFO */
#define UART_FCR_DMA_SELECT 0x08 /* For DMA applications */
#define UART_FCR_TRIGGER_MASK 0xC0 /* Mask for the FIFO trigger range */
#define UART_FCR_TRIGGER_1 0x00 /* Mask for trigger set at 1 */
#define UART_FCR_TRIGGER_4 0x40 /* Mask for trigger set at 4 */
#define UART_FCR_TRIGGER_8 0x80 /* Mask for trigger set at 8 */
#define UART_FCR_TRIGGER_14 0xC0 /* Mask for trigger set at 14 */

#define UART_FCR_RXSR 0x02 /* Receiver soft reset */
#define UART_FCR_TXSR 0x04 /* Transmitter soft reset */

/* Ingenic JZ47xx specific UART-enable bit. */
#define UART_FCR_UME 0x10

/* Clear & enable FIFOs */
#define UART_FCR_DEFVAL (UART_FCR_FIFO_EN | UART_FCR_RXSR | UART_FCR_TXSR)

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_DTR 0x01 /* DTR   */
#define UART_MCR_RTS 0x02 /* RTS   */
#define UART_MCR_OUT1 0x04 /* Out 1 */
#define UART_MCR_OUT2 0x08 /* Out 2 */
#define UART_MCR_LOOP 0x10 /* Enable loopback test mode */
#define UART_MCR_AFE 0x20 /* Enable auto-RTS/CTS */

#define UART_MCR_DMA_EN 0x04
#define UART_MCR_TX_DFR 0x08

/*
 * These are the definitions for the Line Control Register
 *
 * Note: if the word length is 5 bits (UART_LCR_WLEN5), then setting
 * UART_LCR_STOP will select 1.5 stop bits, not 2 stop bits.
 */
#define UART_LCR_WLS_MSK 0x03 /* character length select mask */
#define UART_LCR_WLS_5 0x00 /* 5 bit character length */
#define UART_LCR_WLS_6 0x01 /* 6 bit character length */
#define UART_LCR_WLS_7 0x02 /* 7 bit character length */
#define UART_LCR_WLS_8 0x03 /* 8 bit character length */
#define UART_LCR_STB 0x04 /* # stop Bits, off=1, on=1.5 or 2) */
#define UART_LCR_PEN 0x08 /* Parity eneble */
#define UART_LCR_EPS 0x10 /* Even Parity Select */
#define UART_LCR_STKP 0x20 /* Stick Parity */
#define UART_LCR_SBRK 0x40 /* Set Break */
#define UART_LCR_BKSE 0x80 /* Bank select enable */
#define UART_LCR_DLAB 0x80 /* Divisor latch access bit */

/*
 * These are the definitions for the Line Status Register
 */
#define UART_LSR_DR 0x01 /* Data ready */
#define UART_LSR_OE 0x02 /* Overrun */
#define UART_LSR_PE 0x04 /* Parity error */
#define UART_LSR_FE 0x08 /* Framing error */
#define UART_LSR_BI 0x10 /* Break */
#define UART_LSR_THRE 0x20 /* Xmit holding register empty */
#define UART_LSR_TEMT 0x40 /* Xmitter empty */
#define UART_LSR_ERR 0x80 /* Error */

#define UART_MSR_DCD 0x80 /* Data Carrier Detect */
#define UART_MSR_RI 0x40 /* Ring Indicator */
#define UART_MSR_DSR 0x20 /* Data Set Ready */
#define UART_MSR_CTS 0x10 /* Clear to Send */
#define UART_MSR_DDCD 0x08 /* Delta DCD */
#define UART_MSR_TERI 0x04 /* Trailing edge ring indicator */
#define UART_MSR_DDSR 0x02 /* Delta DSR */
#define UART_MSR_DCTS 0x01 /* Delta CTS */

/*
 * These are the definitions for the Interrupt Identification Register
 */
#define UART_IIR_NO_INT 0x01 /* No interrupts pending */
#define UART_IIR_ID 0x06 /* Mask for the interrupt ID */

#define UART_IIR_MSI 0x00 /* Modem status interrupt */
#define UART_IIR_THRI 0x02 /* Transmitter holding register empty */
#define UART_IIR_RDI 0x04 /* Receiver data interrupt */
#define UART_IIR_RLSI 0x06 /* Receiver line status interrupt */

/*
 * These are the definitions for the Interrupt Enable Register
 */
#define UART_IER_MSI 0x08 /* Enable Modem status interrupt */
#define UART_IER_RLSI 0x04 /* Enable receiver line status interrupt */
#define UART_IER_THRI 0x02 /* Enable Transmitter holding register int. */
#define UART_IER_RDI 0x01 /* Enable receiver data interrupt */

/* useful defaults for LCR */
#define UART_LCR_8N1 0x03

#define UART_ADDR (0xfeb50000)

#define UART_LCRVAL UART_LCR_8N1 /* 8 data, 1 stop, no parity */
#define UART_MCRVAL (UART_MCR_DTR | UART_MCR_RTS) /* RTS/DTR */

#define out_le32(a, v) (*(volatile uint32_t*)(a) = (v))
#define in_le32(a) (*(volatile uint32_t*)(a))

#ifndef CONFIG_SYS_NS16550_IER
#define CONFIG_SYS_NS16550_IER 0x00
#endif /* CONFIG_SYS_NS16550_IER */

#define serial_dout(reg, value)                                                   \
    serial_out_shift((char*)com_port + ((char*)reg - (char*)com_port) * (1 << 2), \
        2, value)
#define serial_din(reg)                                                          \
    serial_in_shift((char*)com_port + ((char*)reg - (char*)com_port) * (1 << 2), \
        2)

static inline void serial_out_shift(void* addr, int shift, int value)
{
    out_le32(addr, value);
}

static inline int serial_in_shift(void* addr, int shift)
{
    return in_le32(addr);
}

#ifndef CONFIG_SYS_NS16550_CLK
#define CONFIG_SYS_NS16550_CLK 0
#endif

bool init_uart_mmio()
{
    static int mapped = 0;
    if (mapped == 0) {
        if (-1 == mmap(UART_ADDR, UART_ADDR, 4096, true)) {
            return false;
        }
        mapped = 1;
    }
    return true;
}

void putc(char ch)
{
    static struct NS16550* com_port = (struct NS16550*)UART_ADDR;

    if (ch == '\n') {
        putc('\r');
    }

    while (!(serial_din(&com_port->lsr) & UART_LSR_THRE))
        ;
    serial_dout(&com_port->thr, ch);
}

char getc(void)
{
    static struct NS16550* com_port = (struct NS16550*)UART_ADDR;

    while (!(serial_din(&com_port->lsr) & UART_LSR_DR))
        ;

    return serial_din(&com_port->rbr);
}
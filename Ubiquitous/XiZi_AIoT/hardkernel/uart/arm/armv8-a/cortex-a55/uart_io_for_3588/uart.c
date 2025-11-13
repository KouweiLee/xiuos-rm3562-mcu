//
// low-level driver routines for pl011 UART.
//

#include "uart.h"
#include "actracer.h"
#include "ns16550.h"
#include "uart_common_ope.h"

// the UART control registers are memory-mapped
// at address UART0. this macro returns the
// address of one of the registers.

void uartinit(void)
{
    _debug_uart_init();
}

void uartputc(uint8_t c)
{
    _debug_uart_putc((int)c);
}

static uint8_t uartgetc(void)
{
    return (uint8_t)_debug_uart_getc();
}

static uint32_t UartGetIrqnum()
{
    return 0;
}

static struct XiziSerialDriver hardkernel_serial_driver = {
    .sys_serial_init = uartinit,
    .get_serial_irqnum = UartGetIrqnum,
    .putc = uartputc,
    .getc = uartgetc,
};

struct XiziSerialDriver* hardkernel_uart_init(struct TraceTag* hardkernel_tag)
{
    hardkernel_serial_driver.sys_serial_init();
    return &hardkernel_serial_driver;
}
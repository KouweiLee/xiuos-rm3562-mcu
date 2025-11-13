/*
 * COM1 NS16550 support
 * originally from linux source (arch/powerpc/boot/ns16550.c)
 * modified to use CONFIG_SYS_ISA_MEM and new defines
 */

#include <stdint.h>

#include "mmio_access.h"
#include "ns16550.h"

#define UART_ADDR MMIO_P2V_WO(0xfeb50000)
// #define UART_ADDR (0xFE660000)

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

#define DIV_ROUND_CLOSEST(x, divisor) (                                                                                                      \
    {                                                                                                                                        \
        typeof(x) __x = x;                                                                                                                   \
        typeof(divisor) __d = divisor;                                                                                                       \
        (((typeof(x))-1) > 0 || ((typeof(divisor))-1) > 0 || (__x) > 0) ? (((__x) + ((__d) / 2)) / (__d)) : (((__x) - ((__d) / 2)) / (__d)); \
    })

int ns16550_calc_divisor(NS16550_t port, int clock, int baudrate)
{
    const unsigned int mode_x_div = 16;

    return DIV_ROUND_CLOSEST(clock, mode_x_div * baudrate);
}

void _debug_uart_init(void)
{
    struct NS16550* com_port = (struct NS16550*)UART_ADDR;

    /*
     * We copy the code from above because it is already horribly messy.
     * Trying to refactor to nicely remove the duplication doesn't seem
     * feasible. The better fix is to move all users of this driver to
     * driver model.
     */
    int baud_divisor = ns16550_calc_divisor(com_port, 24000000,
        1500000);
    serial_dout(&com_port->ier, CONFIG_SYS_NS16550_IER);
    serial_dout(&com_port->mcr, UART_MCRVAL);
    serial_dout(&com_port->fcr, UART_FCR_DEFVAL);

    serial_dout(&com_port->lcr, UART_LCR_BKSE | UART_LCRVAL);
    serial_dout(&com_port->dll, baud_divisor & 0xff);
    serial_dout(&com_port->dlm, (baud_divisor >> 8) & 0xff);
    serial_dout(&com_port->lcr, UART_LCRVAL);
}

void _debug_uart_putc(int ch)
{
    static struct NS16550* com_port = (struct NS16550*)UART_ADDR;

    if (ch == '\n') {
        _debug_uart_putc('\r');
    }

    while (!(serial_din(&com_port->lsr) & UART_LSR_THRE))
        ;
    serial_dout(&com_port->thr, ch);
}

int _debug_uart_getc(void)
{
    static struct NS16550* com_port = (struct NS16550*)UART_ADDR;

    while (!(serial_din(&com_port->lsr) & UART_LSR_DR))
        ;

    return serial_din(&com_port->rbr);
}
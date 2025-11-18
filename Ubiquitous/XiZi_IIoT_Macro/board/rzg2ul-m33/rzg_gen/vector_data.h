/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (7)
#endif

/** Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/* ISR prototypes */
void gtm_int_isr(void);
// void metal_irq_isr_wrapper(void);
void scif_uart_eri_isr(void);
void scif_uart_bri_isr(void);
void scif_uart_rxi_isr(void);
void scif_uart_txi_isr(void);
void scif_uart_tei_isr(void);

void msg_ch1_ns(void);
void rsp_ch4_ns(void);
/** Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif

/* generated HAL source file - do not edit */
#include "hal_data.h"
#define FSP_NOT_DEFINED (UINT32_MAX)
#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac_b.h"
extern void scif_uart_tx_dmac_callback(scif_uart_instance_ctrl_t const *const p_ctrl);

void g_uart0_tx_transfer_callback(dmac_b_callback_args_t *p_args)
{
	FSP_PARAMETER_NOT_USED(p_args);
	scif_uart_tx_dmac_callback(&g_uart0_ctrl);
}
#endif

#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac_b.h"
extern void scif_uart_rx_dmac_callback(scif_uart_instance_ctrl_t const *const p_ctrl);

void g_uart0_rx_transfer_callback(dmac_b_callback_args_t *p_args)
{
	FSP_PARAMETER_NOT_USED(p_args);
	scif_uart_rx_dmac_callback(&g_uart0_ctrl);
}
#endif
#undef FSP_NOT_DEFINED

scif_uart_instance_ctrl_t g_uart2_ctrl;

scif_baud_setting_t g_uart2_baud_setting = {
	/* Baud rate calculated with 0.469% error. */ 
	.semr_baudrate_bits_b.abcs = 0,
	.semr_baudrate_bits_b.bgdm = 1,
	.semr_baudrate_bits_b.cks = 0,
	.brr = 53,
	.mddr = (uint8_t)256,
	.semr_baudrate_bits_b.brme = false
};

/** UART extended configuration for UARTonSCIF HAL driver */
const scif_uart_extended_cfg_t g_uart2_cfg_extend = {
	.bri_ipl = 14,
	.bri_irq = SCIF2_BRK_IRQn,
	.clock = SCIF_UART_CLOCK_INT,
	.noise_cancel = SCIF_UART_NOISE_CANCELLATION_DISABLE,
	.p_baud_setting = &g_uart2_baud_setting,
	.rx_fifo_trigger = SCIF_UART_RECEIVE_TRIGGER_MAX,
	.rts_fifo_trigger = SCIF_UART_RTS_TRIGGER_14,
	.uart_mode = SCIF_UART_MODE_RS232,
	.flow_control = SCIF_UART_FLOW_CONTROL_NONE,
	.rs485_setting = {
		.enable = (sci_uart_rs485_enable_t)NULL,
		.polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
		.de_control_pin = (bsp_io_port_pin_t)SCIF_UART_INVALID_16BIT_PARAM,
	},
};

/** UART interface configuration */
const uart_cfg_t g_uart2_cfg = {
	.channel = 2,
	.data_bits = UART_DATA_BITS_8,
	.parity = UART_PARITY_OFF,
	.stop_bits = UART_STOP_BITS_1,
	.p_callback = NULL,
	.p_context = NULL,
	.p_extend = &g_uart2_cfg_extend,
	.p_transfer_tx = g_uart0_P_TRANSFER_TX,
	.p_transfer_rx = g_uart0_P_TRANSFER_RX,
	.rxi_ipl = 14,
	.txi_ipl = 14,
	.tei_ipl = 14,
	.eri_ipl = 14,
	.rxi_irq = SCIF2_RXI_IRQn,
	.txi_irq = SCIF2_TXI_IRQn,
	.tei_irq = SCIF2_TEI_DRI_IRQn,
	.eri_irq = SCIF2_RERR_IRQn,
};

/* Instance structure to use this module. */
const uart_instance_t g_uart2 = {
	.p_ctrl = &g_uart2_ctrl,
	.p_cfg = &g_uart2_cfg,
	.p_api = &g_uart_on_scif
};

mhu_ns_instance_ctrl_t g_mhu_ns0_ctrl;
const mhu_cfg_t g_mhu_ns0_cfg = {
	.channel = 1,
	.rx_ipl = 12,
	.rx_irq = MHU1_NS_IRQn,
	.p_callback = NULL,
	.p_shared_memory = 0,
	.p_context = NULL,
};
/* Instance structure to use this module. */
const mhu_instance_t g_mhu_ns0 = {
	.p_ctrl = &g_mhu_ns0_ctrl, 
	.p_cfg = &g_mhu_ns0_cfg, 
	.p_api = &g_mhu_ns_on_mhu_ns
};

gtm_instance_ctrl_t g_timer2_ctrl;
const gtm_extended_cfg_t g_timer2_extend = {
	.generate_interrupt_when_starts = GTM_GIWS_TYPE_DISABLED,
	.gtm_mode = GTM_TIMER_MODE_INTERVAL,
};

const timer_cfg_t g_timer2_cfg = {
	.mode = TIMER_MODE_PERIODIC,
	.period_counts = (uint32_t)0x1869f /* Actual period: 0.001 seconds. */,
	.channel = 2,
	.p_callback = NULL,
	.p_context = NULL,
	.cycle_end_ipl = 255,
	.p_extend = &g_timer2_extend,
#if defined(VECTOR_NUMBER_GTM2_COUNTER_OVERFLOW)
	.cycle_end_irq = VECTOR_NUMBER_GTM2_COUNTER_OVERFLOW,
#else
	.cycle_end_irq = GTM2_OSTM2INT_IRQn,
#endif
};

/* Instance structure to use this module. */
const timer_instance_t g_timer2 = {
	.p_ctrl = &g_timer2_ctrl, 
	.p_cfg = &g_timer2_cfg, 
	.p_api = &g_timer_on_gtm
};

void g_hal_init(void)
{
	g_common_init();
}

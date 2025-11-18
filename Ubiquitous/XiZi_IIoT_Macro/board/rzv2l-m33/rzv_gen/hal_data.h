/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_uart_api.h"
#include "r_scif_uart.h"
#include "r_timer_api.h"
#include "r_mhu_ns.h"
#include "r_gtm.h"

FSP_HEADER

/** UART on SCIF Instance. */
extern const uart_instance_t g_uart2;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern scif_uart_instance_ctrl_t g_uart2_ctrl;
extern const uart_cfg_t g_uart2_cfg;
extern const scif_uart_extended_cfg_t g_uart2_cfg_extend;

/** MHU Instance */
extern const mhu_instance_t g_mhu_ns0;

/** Access the MHU instance using these structures when calling API functions directly (::p_api is not used). */
extern mhu_ns_instance_ctrl_t g_mhu_ns0_ctrl;
extern const mhu_cfg_t g_mhu_ns0_cfg;

/** GTM Timer Instance */
extern const timer_instance_t g_timer2;

/** Access the GTM instance using these structures when calling API functions directly (::p_api is not used). */
extern gtm_instance_ctrl_t g_timer2_ctrl;
extern const timer_cfg_t g_timer2_cfg;

void hal_entry(void);
void g_hal_init(void);

FSP_FOOTER
#endif /* HAL_DATA_H_ */

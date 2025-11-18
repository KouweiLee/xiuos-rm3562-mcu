/* generated pin source file - do not edit */
#include "bsp_api.h"
#include "r_ioport_api.h"
#include "r_ioport.h"
#include "bsp_io.h"

const ioport_pin_cfg_t g_bsp_pin_cfg_data[] =
{
    // UART
    {
        .pin = BSP_IO_PORT_05_PIN_00,
        .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN | (uint32_t) IOPORT_PERIPHERAL_MODE2)
    },
    {
        .pin = BSP_IO_PORT_05_PIN_01,
        .pin_cfg = ((uint32_t) IOPORT_CFG_PERIPHERAL_PIN | (uint32_t) IOPORT_PERIPHERAL_MODE2)
    },
};

extern const ioport_extend_cfg_t g_ioport_cfg_extend;

const ioport_cfg_t g_bsp_pin_cfg = { 
    .number_of_pins = sizeof(g_bsp_pin_cfg_data) / sizeof(ioport_pin_cfg_t),
	.p_pin_cfg_data = &g_bsp_pin_cfg_data[0], 
    .p_extend = &g_ioport_cfg_extend 
};
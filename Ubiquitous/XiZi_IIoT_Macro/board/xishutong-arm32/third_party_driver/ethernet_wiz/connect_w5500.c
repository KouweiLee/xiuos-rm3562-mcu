
#include "connect_w5500.h"
#include "connect_ethernet_wiz.h"

#include <bus.h>
#include <dev_pin.h>
#include <xs_base.h>
#include <xsconfig.h>

#include <stdlib.h>
#include <string.h>

#include "socket.h"
#include "w5500.h"

#include <connect_gpio.h>

#include <sys.h>

#define SPI_FREQUENCY 10000000

#define WIZ_RESET_PORT (GPIO_PORT_G)
#define WIZ_RESET_PIN (GPIO_PIN_07)

// spi operations
extern void spi_enter_cris(void);
extern void spi_exit_cris(void);
extern void spi_select_cs(void);
extern void spi_deselete_cs(void);

// global configurations for w5500 tcp connection
uint32_t get_gbuf_size()
{
    static const uint32_t g_wiznet_buf_size = 2048;
    return g_wiznet_buf_size;
}

wiz_NetInfo* get_gnetinfo()
{
    static wiz_NetInfo g_wiz_netinfo = {
        .mac = { 0x00, 0x08, 0xdc, 0x11, 0x11, 0x11 },
        .ip = { 192, 168, 130, 78 },
        .sn = { 255, 255, 254, 0 },
        .gw = { 192, 168, 130, 1 },
        .dns = { 0, 0, 0, 0 },
        .dhcp = NETINFO_STATIC
    };
    return &g_wiz_netinfo;
}

/****************** spi init ******************/
static struct Bus* w5500_spi_bus;
int w5500_spi_init()
{
    x_err_t ret = EOK;

    w5500_spi_bus = BusFind(SPI_BUS_NAME_1);
    w5500_spi_bus->owner_haldev = BusFindDevice(w5500_spi_bus, SPI_1_DEVICE_NAME_0);
    w5500_spi_bus->owner_driver = BusFindDriver(w5500_spi_bus, SPI_1_DRV_NAME);

    w5500_spi_bus->match(w5500_spi_bus->owner_driver,
        w5500_spi_bus->owner_haldev);

    struct BusConfigureInfo configure_info;
    struct SpiMasterParam spi_master_param;
    spi_master_param.spi_data_bit_width = 8;
    spi_master_param.spi_work_mode = SPI_MODE_0 | SPI_MSB;
    spi_master_param.spi_maxfrequency = SPI_FREQUENCY;
    spi_master_param.spi_data_endian = 0;

    configure_info.configure_cmd = OPE_CFG;
    configure_info.private_data = (void*)&spi_master_param;
    ret = BusDrvConfigure(w5500_spi_bus->owner_driver, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_CFG error drv %8p cfg %8p\n",
            w5500_spi_bus->owner_driver, &spi_master_param);
        return ERROR;
    }

    configure_info.configure_cmd = OPE_INT;
    ret = BusDrvConfigure(w5500_spi_bus->owner_driver, &configure_info);
    if (ret) {
        KPrintf("spi drv OPE_INT error drv %8p\n", w5500_spi_bus->owner_driver);
        return ERROR;
    }

    return EOK;
}

void spi_write_byte(uint8_t tx_data)
{
    struct BusBlockWriteParam write_param;
    uint8_t data = tx_data;
    write_param.buffer = &data;
    write_param.size = 1;
    BusDevWriteData(w5500_spi_bus->owner_haldev, &write_param);
}
uint8_t spi_read_byte(void)
{
    uint8_t result = 0;
    struct BusBlockReadParam read_param;
    read_param.buffer = &result;
    read_param.size = 1;
    BusDevReadData(w5500_spi_bus->owner_haldev, &read_param);
    return result;
}
void spi_write_burst(uint8_t* tx_buf, uint16_t len)
{
    struct BusBlockWriteParam write_param;
    write_param.buffer = tx_buf;
    write_param.size = len;
    BusDevWriteData(w5500_spi_bus->owner_haldev, &write_param);
}
void spi_read_burst(uint8_t* rx_buf, uint16_t len)
{
    struct BusBlockReadParam read_param;
    read_param.buffer = rx_buf;
    read_param.size = len;
    BusDevReadData(w5500_spi_bus->owner_haldev, &read_param);
}

/****************** chip init ******************/

void wiz_reset()
{
    // init wiz reset pin
    BusType pin = PinBusInitGet();
    x_base wiz_rst_pin = 111; // PG7

    struct PinParam wiz_rst_param = {
        .cmd = GPIO_CONFIG_MODE,
        .pin = wiz_rst_pin, // PG7
        .mode = GPIO_CFG_OUTPUT,
    };
    struct BusConfigureInfo config_info = {
        .configure_cmd = OPE_CFG,
        .private_data = (void*)&wiz_rst_param,
    };
    int ret = BusDrvConfigure(pin->owner_driver, &config_info);
    if (ret != EOK) {
        SYS_KDEBUG_LOG(WIZNET_DEBUG, ("GPIO Failed to set Reset pin.\n"));
        return;
    }

    // reset wiz chip
    struct PinStat rst_pin_stat;
    struct BusBlockWriteParam write_param = { .buffer = (void*)&rst_pin_stat };

    rst_pin_stat.pin = wiz_rst_pin;
    rst_pin_stat.val = GPIO_LOW;
    BusDevWriteData(pin->owner_haldev, &write_param);
    MdelayKTask(20);

    rst_pin_stat.pin = wiz_rst_pin;
    rst_pin_stat.val = GPIO_HIGH;
    BusDevWriteData(pin->owner_haldev, &write_param);
    MdelayKTask(20);
}

void wiz_spi_handler_reg()
{
    // spi ops registration
#if (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_) || (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_)
    /* register SPI device CS select callback function */
    wiz_cs_bus = PinBusInitGet();
    wiz_cs_pin = 107; // PG3

    struct PinParam wiz_cs_param = {
        .cmd = GPIO_CONFIG_MODE,
        .pin = wiz_cs_pin, // PG7
        .mode = GPIO_CFG_OUTPUT,
    };
    struct BusConfigureInfo config_info = {
        .configure_cmd = OPE_CFG,
        .private_data = (void*)&wiz_cs_param,
    };
    int ret = BusDrvConfigure(wiz_cs_bus->owner_driver, &config_info);
    if (ret != EOK) {
        SYS_KDEBUG_LOG(WIZNET_DEBUG, ("GPIO Failed to set Reset pin.\n"));
        return;
    }

    reg_wizchip_cs_cbfunc(spi_select_cs, spi_deselete_cs);
#else
#if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
#error "Unknown _WIZCHIP_IO_MODE_"
#else
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
#endif
#endif

    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_cris_cbfunc(spi_enter_cris, spi_exit_cris);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);
}

int wiz_chip_cfg_init()
{
    uint8_t mem_size[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };

    /* reset WIZnet chip internal PHY, configures PHY mode. */
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)mem_size) == -1) {
        KPrintf("WIZCHIP initialize failed.");
        return ERROR;
    }

    struct wiz_NetTimeout_t net_timeout;
    net_timeout.retry_cnt = 5;
    net_timeout.time_100us = 20000;
    ctlnetwork(CN_SET_TIMEOUT, (void*)&net_timeout);

    return EOK;
}

/****************** interrupt handle ******************/
#ifdef BSP_USING_LWIP
#include <sys.h>
static inline void spi_if_clr(void)
{
    setSn_IR(0, 0x1F);
    setSIR(0);
}

void wiz_irq_handler()
{
    static x_base eth_irq_lock;
    eth_irq_lock = DISABLE_INTERRUPT();

    if (*get_eth_recv_sem2() > 0) {
        sys_sem_signal(get_eth_recv_sem2());
    }

    ENABLE_INTERRUPT(eth_irq_lock);
}
#else
void wiz_irq_handler()
{
    static x_base eth_irq_lock;
    eth_irq_lock = DISABLE_INTERRUPT();

    printf("=");
    uint8_t ir = getIR();
    setSIR(0x00);
    setIR(0x00);

    ENABLE_INTERRUPT(eth_irq_lock);
}
#endif

void wizchip_interrupt_init(uint8_t socket, void (*callback)(void*))
{
    int ret_val;

    uint8_t reg_val1;
    reg_val1 = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT); // except SendOK
    ret_val = wiz_ctlsocket(socket, CS_SET_INTMASK, (void*)&reg_val1);

#if (_WIZCHIP_ == W5100S)
    reg_val = (1 << socket);
#elif (_WIZCHIP_ == W5500)
    uint16_t reg_val2 = ((1 << socket) << 8);
#endif
    ret_val = ctlwizchip(CW_SET_INTRMASK, (void*)&reg_val2);

    (void)ret_val;
}

int wiz_interrupt_init()
{
#define BSP_WIZ_INT_PIN 106
    int32_t ret = -ERROR;

    wizchip_interrupt_init(0, wiz_irq_handler);

    struct Bus* pin_bus = PinBusInitGet();

    struct PinParam pin_param;
    struct BusConfigureInfo pin_configure_info;

    pin_configure_info.configure_cmd = OPE_CFG;
    pin_configure_info.private_data = (void*)&pin_param;

    pin_param.cmd = GPIO_CONFIG_MODE;
    pin_param.pin = BSP_WIZ_INT_PIN;
    pin_param.mode = GPIO_CFG_INPUT_PULLUP;
    ret = BusDrvConfigure(pin_bus->owner_driver, &pin_configure_info);
    if (ret != EOK) {
        KPrintf("config pin_param  %d input failed!\n", pin_param.pin);
        return -ERROR;
    }

    pin_param.cmd = GPIO_IRQ_REGISTER;
    pin_param.pin = BSP_WIZ_INT_PIN;
    pin_param.irq_set.irq_mode = GPIO_IRQ_EDGE_FALLING;
    pin_param.irq_set.hdr = wiz_irq_handler;
    pin_param.irq_set.args = NONE;
    ret = BusDrvConfigure(pin_bus->owner_driver, &pin_configure_info);
    if (ret != EOK) {
        KPrintf("register pin_param  %d  irq failed!\n", pin_param.pin);
        return -ERROR;
    }

    pin_param.cmd = GPIO_IRQ_DISABLE;
    pin_param.pin = BSP_WIZ_INT_PIN;
    ret = BusDrvConfigure(pin_bus->owner_driver, &pin_configure_info);
    if (ret != EOK) {
        KPrintf("disable pin_param  %d  irq failed!\n", pin_param.pin);
        return -ERROR;
    }

    // 4. enable interuption
    pin_param.cmd = GPIO_IRQ_ENABLE;
    pin_param.pin = BSP_WIZ_INT_PIN;
    ret = BusDrvConfigure(pin_bus->owner_driver, &pin_configure_info);
    if (ret != EOK) {
        KPrintf("enable pin_param  %d  irq failed!\n", pin_param.pin);
        return -ERROR;
    }

    return EOK;
}

int HwWiznetInit(void)
{
    wiz_reset();

    if (EOK != w5500_spi_init()) {
        return ERROR;
    }

    wiz_spi_handler_reg();

    if (EOK != wiz_chip_cfg_init()) {
        return ERROR;
    }

    extern uint8_t wiz_mac[6];
    setSHAR(wiz_mac);
    ctlwizchip(CW_RESET_PHY, 0);

    wiz_interrupt_init(0, wiz_irq_handler);

    setSn_RXBUF_SIZE(0, 16);
    setSn_TXBUF_SIZE(0, 16);
#define SOCK_ANY_PORT_NUM 0xC000
    setSn_MR(0, Sn_MR_MFEN | Sn_MR_MACRAW | Sn_MR_MIP6B | Sn_MR_MMB);
    wiz_socket(0, Sn_MR_MACRAW, SOCK_ANY_PORT_NUM, SOCK_IO_NONBLOCK);

    uint8_t sock_iomode = SOCK_IO_NONBLOCK;
    wiz_ctlsocket(0, CS_SET_IOMODE, &sock_iomode);

    uint8_t sock_sr = 0;
    while (1) {
        sock_sr = getSn_SR(0);
        SYS_KDEBUG_LOG(WIZNET_DEBUG, ("[%s] sock_sr: %x, MACRAW: %x\n", __func__, sock_sr, SOCK_MACRAW));
        if (sock_sr == SOCK_MACRAW) {
            SYS_KDEBUG_LOG(WIZNET_DEBUG, ("Socket 0 MACRAW mode established\r\n"));
            break;
        }
    }

    return EOK;
}
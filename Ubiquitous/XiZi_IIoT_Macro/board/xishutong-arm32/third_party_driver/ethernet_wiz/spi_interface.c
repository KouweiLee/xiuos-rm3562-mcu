
#include <assert.h>
#include <connect_w5500.h>
// #include <sleep.h>
#include <xs_base.h>

// #define SPI1_CS_GPIONUM 24

static x_base g_w5500_spi_lock;
/**
 * @brief  进入临界区
 * @retval None
 */
void spi_enter_cris(void) { g_w5500_spi_lock = DisableLocalInterrupt(); }
/**
 * @brief  退出临界区
 * @retval None
 */
void spi_exit_cris(void) { EnableLocalInterrupt(g_w5500_spi_lock); }

/**
 * @brief  片选信号输出低电平
 * @retval None
 */
void spi_select_cs(void)
{
    struct PinStat cs_pin_stat = {
        .pin = wiz_cs_pin,
        .val = GPIO_LOW,
    };
    struct BusBlockWriteParam write_param = { .buffer = (void*)&cs_pin_stat };
    BusDevWriteData(wiz_cs_bus->owner_haldev, &write_param);
}
/**
 * @brief  片选信号输出高电平
 * @retval None
 */
void spi_deselete_cs(void)
{
    struct PinStat cs_pin_stat = {
        .pin = wiz_cs_pin,
        .val = GPIO_HIGH,
    };
    struct BusBlockWriteParam write_param = { .buffer = (void*)&cs_pin_stat };
    BusDevWriteData(wiz_cs_bus->owner_haldev, &write_param);
}
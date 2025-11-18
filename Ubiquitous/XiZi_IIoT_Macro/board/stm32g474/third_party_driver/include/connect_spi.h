#ifndef CONNECT_SPI_H
#define CONNECT_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xs_base.h"
#include "dev_spi.h"

struct SpiHardwareDevice;
struct SpiDataStandard;

#define GPIO_SPI1_SCK_PIN       GPIO_PIN_3
#define GPIO_SPI1_SCK_PORT      GPIOB

#define GPIO_SPI1_MISO_PIN      GPIO_PIN_6
#define GPIO_SPI1_MISO_PORT     GPIOA

#define GPIO_SPI1_MOSI_PIN      GPIO_PIN_7
#define GPIO_SPI1_MOSI_PORT     GPIOA

#define GPIO_SPI1_CS_PIN        GPIO_PIN_2
#define GPIO_SPI1_CS_PORT       GPIOD


int InitSPI(void);
uint32 SpiTransmitReceive(struct SpiHardwareDevice *dev, struct SpiDataStandard *msg);
#ifdef __cplusplus
}
#endif

#endif /* CONNECT_SPI_H */

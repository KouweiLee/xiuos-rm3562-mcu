SRC_DIR := Peripheral

SRC_DIR += sys

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
endif

ifeq ($(CONFIG_BSP_USING_SPI),y)
  SRC_DIR += spi
endif

ifeq ($(CONFIG_BSP_USING_SERDES),y)
  SRC_DIR += serdes
endif

ifeq ($(CONFIG_BSP_USING_USB),y)
  SRC_DIR += usb
endif

ifeq ($(CONFIG_BSP_USING_WDT),y)
  SRC_DIR += wdt
endif

include $(KERNEL_ROOT)/compiler.mk

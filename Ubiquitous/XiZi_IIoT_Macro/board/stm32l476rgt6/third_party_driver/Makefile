SRC_DIR := libraries

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
endif

ifeq ($(CONFIG_BSP_USING_SPI),y)
  SRC_DIR += spi
endif

ifeq ($(CONFIG_BSP_USING_USB),y)
  SRC_DIR += usb
endif

ifeq ($(CONFIG_BSP_USING_WDT),y)
  SRC_DIR += watchdog
endif

ifeq ($(CONFIG_BSP_USING_CAN),y)
  SRC_DIR += can
endif

include $(KERNEL_ROOT)/compiler.mk

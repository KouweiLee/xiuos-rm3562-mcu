SRC_DIR := stm32cube_g4 

ifeq ($(CONFIG_BSP_USING_UART),y)
	SRC_DIR += uart
endif

ifeq ($(CONFIG_BSP_USING_USB),y)
	SRC_DIR += usb
endif

ifeq ($(CONFIG_BSP_USING_CAN),y)
	SRC_DIR += can
endif

ifeq ($(CONFIG_BSP_USING_SPI),y)
	SRC_DIR += spi
endif

include $(KERNEL_ROOT)/compiler.mk

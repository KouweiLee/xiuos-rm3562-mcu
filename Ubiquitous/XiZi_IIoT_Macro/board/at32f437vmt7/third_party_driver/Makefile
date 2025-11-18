SRC_DIR := libraries
SRC_DIR += common

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
endif

ifeq ($(CONFIG_BSP_USING_ETH),y)
  SRC_DIR += ethernet
endif

include $(KERNEL_ROOT)/compiler.mk

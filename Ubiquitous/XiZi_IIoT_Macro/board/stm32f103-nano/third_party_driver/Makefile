SRC_DIR := libraries 

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
  SRC_DIR += can
endif

include $(KERNEL_ROOT)/compiler.mk

# SRC_FILES := connect_uart.c test_uart.c
ifeq ($(CONFIG_LTE_TEST),y)
  SRC_DIR := test
endif
SRC_FILES := connect_lte.c

include $(KERNEL_ROOT)/compiler.mk

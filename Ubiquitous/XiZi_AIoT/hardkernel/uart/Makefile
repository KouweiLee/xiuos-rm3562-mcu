ifneq ($(findstring $(BOARD), 3568 3588 imx6q-sabrelite zynq7000-zc702), )
SRC_DIR := arm
endif
ifneq ($(findstring $(BOARD), jh7110), )
SRC_DIR := riscv
endif

SRC_FILES := uart_common_ope.c

include $(KERNEL_ROOT)/compiler.mk
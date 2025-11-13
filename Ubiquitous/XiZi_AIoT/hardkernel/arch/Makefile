ifneq ($(findstring $(BOARD), 3588 3568 imx6q-sabrelite zynq7000-zc702), )
SRC_DIR := arm
endif
ifneq ($(findstring $(BOARD), jh7110), )
SRC_DIR := riscv
endif

include $(KERNEL_ROOT)/compiler.mk

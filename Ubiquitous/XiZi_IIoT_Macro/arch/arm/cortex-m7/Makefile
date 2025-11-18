ifeq ($(CONFIG_BOARD_CORTEX_M7_EVB),y)
SRC_FILES := boot.S  interrupt.c interrupt_vector.S
endif

ifeq ($(CONFIG_BOARD_STM32H750),y)
SRC_DIR += stm32h750
endif

ifeq ($(CONFIG_BOARD_IMXRT1176_SBC_EVB),y)
SRC_FILES := boot.S  interrupt.c interrupt_vector.S
endif

ifeq ($(CONFIG_BOARD_CORTEX_V2M_EVB),y)
SRC_DIR += V2M
endif
include $(KERNEL_ROOT)/compiler.mk

# The following three platforms support compatiable instructions.

ifeq ($(CONFIG_BOARD_CORTEX_M3_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m3
endif

ifeq ($(CONFIG_BOARD_CORTEX_M4_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_STM32F103_NANO),y)
SRC_DIR := shared
SRC_DIR += cortex-m3
endif

ifeq ($(CONFIG_BOARD_STM32F407_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_STM32G474),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_STM32L476_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_CORTEX_M4_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_EDU_ARM32_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_XISHUTONG_ARM32),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_GD32F415RG),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_AT32F437VMT7),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_CORTEX_M7_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m7
endif

ifeq ($(CONFIG_BOARD_CORTEX_V2M_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m7
endif

ifeq ($(CONFIG_BOARD_IMXRT1176_SBC_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m7
endif

ifeq ($(CONFIG_BOARD_STM32H750),y)
SRC_DIR := shared
SRC_DIR += cortex-m7
endif



# cortex-m0 is ARMv6-m

ifeq ($(CONFIG_BOARD_CORTEX_M0_EVB),y)
SRC_DIR += cortex-m0
endif

ifeq ($(CONFIG_BOARD_NUVOTON_M2354),y)
SRC_DIR += cortex-m23
endif

ifeq ($(CONFIG_BOARD_RZV2L_M33),y)
SRC_DIR += cortex-m33
endif

ifeq ($(CONFIG_BOARD_RZG2UL_M33),y)
SRC_DIR += cortex-m33
endif

include $(KERNEL_ROOT)/compiler.mk

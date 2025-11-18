SRC_FILES := ModuleConfig.c
SRC_DIR := Peripheral

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
endif
ifeq ($(CONFIG_BSP_USING_ETH),y)
  SRC_DIR += ethernet
endif
ifeq ($(CONFIG_BSP_USING_ADC),y)
  SRC_DIR += adc
endif
ifeq ($(CONFIG_BSP_USING_BLE),y)
  SRC_DIR += ble
endif
ifeq ($(CONFIG_BSP_USING_CAN),y)
  SRC_DIR += can
endif
ifeq ($(CONFIG_BSP_USING_RS485),y)
  SRC_DIR += rs485
endif
ifeq ($(CONFIG_BSP_USING_LTE),y)
  SRC_DIR += lte
endif
ifeq ($(CONFIG_TOOL_USING_OTA),y)
  SRC_DIR += ota
  SRC_DIR += common
endif
include $(KERNEL_ROOT)/compiler.mk

# SRC_DIR := lvgl
SRC_DIR := 
ifeq ($(CONFIG_APP_SELECT_NEWLIB),y)
	ifeq ($(CONFIG_SEPARATE_COMPILE),y)
		SRC_DIR += app_newlib
	endif	
endif

ifeq ($(CONFIG_LIB_LV),y)
	SRC_DIR += lvgl
endif

ifeq ($(CONFIG_LIB_USING_CJSON),y)
	SRC_DIR += cJSON
endif

ifeq ($(CONFIG_LIB_USING_LORAWAN),y)
	SRC_DIR += lorawan
endif

ifeq ($(CONFIG_LIB_USING_MQTT),y)
    SRC_DIR += mqtt
endif

ifeq ($(CONFIG_LIB_USING_JERRYSCRIPT),y)
    SRC_DIR += JerryScript
endif

ifeq ($(CONFIG_LIB_USING_SQLITE),y)
  SRC_DIR += SQLite
endif

ifeq ($(CONFIG_LIB_USING_PIKAPYTHON),y)
  SRC_DIR += pikapython
endif

# ifeq ($(CONFIG_LIB_USING_FREEMODBUS),y)
# 	SRC_DIR += freemodbus
# endif

include $(KERNEL_ROOT)/compiler.mk

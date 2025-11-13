SRC_FILES := trampoline.S $(BOARD)/trap_common.c $(BOARD)/trap.c error_debug.c hard_spinlock.S

ifeq ($(BOARD), 3588)
SRC_DIR := gicv3
SRC_FILES += $(BOARD)/
endif

ifeq ($(BOARD), 3568)
SRC_DIR := gicv3
SRC_FILES += $(BOARD)/
endif
include $(KERNEL_ROOT)/compiler.mk
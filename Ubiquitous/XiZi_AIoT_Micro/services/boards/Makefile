ifneq ($(findstring $(BOARD), 3568 3588), )
SRC_DIR := $(BOARD)-board
else ifneq ($(findstring $(BOARD), jh7110), )
SRC_DIR := VisionFive2
else
SRC_DIR := $(BOARD)
endif


include $(KERNEL_ROOT)/compiler.mk

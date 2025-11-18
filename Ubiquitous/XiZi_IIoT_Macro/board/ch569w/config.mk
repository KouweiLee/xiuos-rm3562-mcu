export MARCH := rv32imac

# export CFLAGS :=  -march=$(MARCH) -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -g

export CFLAGS := -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g -std=gnu99
export AFLAGS :=  -march=$(MARCH) -mabi=ilp32 -x assembler-with-cpp -ggdb
export LFLAGS :=  -march=$(MARCH) -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi-ch569w.map,-cref,-u,_start -T $(BSP_ROOT)/link.ld

export APPLFLAGS :=  -nostartfiles -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds
export CXXFLAGS :=  -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common # -std=gnu99

export DEFINES := -DDEBUG=1

export ARCH = risc-v
export MCU =  CH569W

ifeq ($(CONFIG_BSP_USING_SERDES), y)
export LINK_BOARD += $(KERNEL_ROOT)/board/ch569w/third_party_driver/serdes/libSERDES.a
endif

ifeq ($(CONFIG_BSP_USING_USBD), y)
export LINK_BOARD += $(KERNEL_ROOT)/board/ch569w/third_party_driver/usb/usbd/simulate_cdc/USB30/libCH56x_USB30_device_lib.a
endif

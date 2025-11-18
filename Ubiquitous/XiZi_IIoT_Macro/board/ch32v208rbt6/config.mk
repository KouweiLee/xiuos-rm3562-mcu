# newer risc-v toolchain requires `zicsr` extension for csr-related ops
export MARCH := rv32imac_zicsr
# export MARCH := rv32imacxw_zicsr

export CFLAGS :=  -march=$(MARCH) -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -g
export AFLAGS :=  -march=$(MARCH) -mabi=ilp32 -x assembler-with-cpp -ggdb

ifeq ($(CONFIG_MCUBOOT_BOOTLOADER),y)
export LFLAGS :=  -march=$(MARCH) -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi-ch32v208vct6.map,-cref,-u,_start -T $(BSP_ROOT)/link_bootloader.ld
else ifeq ($(CONFIG_MCUBOOT_APPLICATION),y)
export LFLAGS :=  -march=$(MARCH) -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi-ch32v208vct6.map,-cref,-u,_start -T $(BSP_ROOT)/link_application.ld
else
export LFLAGS :=  -march=$(MARCH) -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi-ch32v208vct6.map,-cref,-u,_start -T $(BSP_ROOT)/link.ld
endif

# export CFLAGS :=  -march=$(MARCH) -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -g -std=gnu99
# export AFLAGS :=  -march=$(MARCH) -mabi=ilp32 -x assembler-with-cpp -ggdb
# export LFLAGS :=  -march=$(MARCH) -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -g -T $(BSP_ROOT)/link.ld -nostartfiles -Xlinker --gc-sections -Wl,-Map,"XiZi-ch32v307vct6.map" --specs=nano.specs --specs=nosys.specs

export APPLFLAGS :=  -nostartfiles -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds

export CXXFLAGS :=  -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common # -std=gnu99

export CROSS_COMPILE ?=/opt/riscv64-toolchain/bin/riscv64-unknown-elf-

export DEFINES := -DHAVE_CCONFIG_H -DHAVE_SIGINFO
ifeq ($(CONFIG_MCUBOOT_BOOTLOADER),y)
export DEFINES += -D__BOOTLOADER
endif

export LINK_WCH_NET := $(KERNEL_ROOT)/board/ch32v208rbt6/third_party_driver/ethernet/libwchnet.a
export LINK_WCH_BLE := $(KERNEL_ROOT)/board/ch32v208rbt6/third_party_driver/ble/lib/libwchble.a

export ARCH = risc-v
export MCU =  CH32V208




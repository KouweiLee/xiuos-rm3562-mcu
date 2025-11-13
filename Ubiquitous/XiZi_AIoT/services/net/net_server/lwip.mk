ifeq ($(BOARD), imx6q-sabrelite)
toolchain ?= arm-none-eabi-
user_ldflags = --specs=nosys.specs -Wl,-Map=user.map,-cref -N
cflags = -std=c11 -march=armv7-a -mtune=cortex-a9 -nostdlib -ffreestanding -nodefaultlibs -mfloat-abi=soft -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -Waddress -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
endif
ifeq ($(BOARD), zynq7000-zc702)
toolchain ?= arm-xilinx-eabi-
user_ldflags = --start-group,-lgcc,-lc,--end-group
cflags = -std=c11 -march=armv7-a -mtune=cortex-a9 -nostdlib -nodefaultlibs -mfloat-abi=soft -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
board_specs = stub.o
#cflags = -Wall -g -std=c11 
endif

ifeq ($(BOARD), ok1028a-c)
toolchain ?= aarch64-none-elf-
user_ldflags = -N -Ttext 0
cflags = -Wall -g -std=c11 -mtune=cortex-a72 -nostdlib -nodefaultlibs -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie  
board_specs = $(KERNEL_ROOT)/services/app/stub.o
endif

ifeq ($(BOARD), 3588)
toolchain ?= aarch64-none-elf-
user_ldflags = -N -Ttext 0
cflags = -Wall -g -std=c11 -mtune=cortex-a55 -nostdlib -nodefaultlibs -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie  
board_specs = $(KERNEL_ROOT)/services/app/stub.o
endif

cc = ${toolchain}gcc
ld = ${toolchain}g++
objdump = ${toolchain}objdump
ar = ${toolchain}ar

c_useropts = -O2

INC_DIR = 	-I$(KERNEL_ROOT)/services/net/libnet \
			-I$(KERNEL_ROOT)/services/net/net_server \
			-I$(KERNEL_ROOT)/services/net/net_server/arch \
			-I$(KERNEL_ROOT)/services/net/net_server/include \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/apps \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/priv \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/prot \
			-I$(KERNEL_ROOT)/services/net/net_server/include/netif \
			-I$(KERNEL_ROOT)/services/net/net_server/include/compat \
			-I$(KERNEL_ROOT)/services//semaphore \
			-I$(KERNEL_ROOT)/services/lib/ipc \
		  	-I$(KERNEL_ROOT)/services/lib/memory \
			-I$(KERNEL_ROOT)/services/lib/serial \
			-I$(KERNEL_ROOT)/services/lib/usyscall \
			-I$(KERNEL_ROOT)/services/boards/$(BOARD) \
			-I$(KERNEL_ROOT)/services/app

ifeq ($(BOARD), imx6q-sabrelite)
toolchain ?= arm-none-eabi-
user_ldflags = --specs=nosys.specs -Wl,-Map=user.map,-cref -N
cflags = -std=c11 -O2 -march=armv7-a -mtune=cortex-a9 -nostdlib -nodefaultlibs -mfloat-abi=soft -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
endif

ifeq ($(BOARD), zynq7000-zc702)
toolchain ?= arm-xilinx-eabi-
user_ldflags = -Wl,--start-group,-lgcc,-lc,--end-group -N
cflags = -std=c11 -O2 -march=armv7-a -mtune=cortex-a9 -nostdlib -nodefaultlibs -mfloat-abi=soft -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -ggdb -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
board_specs = stub.o
#cflags = -Wall -g -std=c11 
endif

ifeq ($(BOARD), ok1028a-c)
toolchain ?= aarch64-none-elf-
user_ldflags = -N -Ttext 0
cflags =  -g -std=c11 -mtune=cortex-a72 -nostdlib -nodefaultlibs -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -Werror -ggdb -Wno-unused  -fno-omit-frame-pointer -fno-stack-protector -fno-pie  
board_specs = stub.o
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
c_useropts = -O2
# c_useropts = -O0

INC_DIR = 	-I$(KERNEL_ROOT)/services/app \
		  	-I$(KERNEL_ROOT)/services/boards/$(BOARD) \
			-I$(KERNEL_ROOT)/services/lib/serial \
			-I$(KERNEL_ROOT)/services/drivers/3588/include \
			-I$(KERNEL_ROOT)/services/lib/usyscall \
		  	-I$(KERNEL_ROOT)/services/lib/ipc \
		  	-I$(KERNEL_ROOT)/services/lib/memory 


objs =  hal_gmac.o \
		hal_gmac_3588.o \
		test_gmac.o \
		hal_base.o \
		hal_bsp.o \
		hal_pinctrl_v2.o \
		hal_cru.o \
		hal_gpio.o \
		hal_timer.o \
		hal_cru_rk3588.o \
		hal_debug.o \
		system_rk3588.o \

all: ${objs}
	@mv $^ $(KERNEL_ROOT)/services/app

%.o: %.c
	@echo "cc $^"
	@${cc} ${cflags} ${c_useropts} ${INC_DIR} -o $@ -c $^
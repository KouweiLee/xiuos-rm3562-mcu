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
ifeq ($(BOARD), 3588)
toolchain ?= aarch64-none-elf-
user_ldflags = -N -Ttext 0
cflags = -Wall -O2 -std=c11 -mtune=cortex-a55 -nostdlib -nodefaultlibs -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
board_specs = stub.o
endif

ifeq ($(BOARD), 3568)
toolchain ?= aarch64-none-elf-
user_ldflags = -N -Ttext 0
cflags = -Wall -O2 -std=c11 -mtune=cortex-a55 -nostdlib -nodefaultlibs -fno-pic -static -fno-builtin -fno-strict-aliasing -Wall -Wno-unused -Werror -fno-omit-frame-pointer -fno-stack-protector -fno-pie
board_specs = stub.o
endif
ifeq ($(BOARD), jh7110)
toolchain ?= riscv64-unknown-elf-
user_ldflags = -N -Ttext 0 -T $(KERNEL_ROOT)/services/boards/jh7110/jh7110_user.lds
CFLAGS = -Wall -Werror -O -fno-omit-frame-pointer -ggdb -gdwarf-2 -MD -mcmodel=medany -fno-common -nostdlib -fno-builtin-strncpy -fno-builtin-strncmp -fno-builtin-strlen -fno-builtin-memset -fno-builtin-memmove -fno-builtin-memcmp -fno-builtin-log -fno-builtin-bzero -fno-builtin-strchr -fno-builtin-exit -fno-builtin-malloc -fno-builtin-putc -fno-builtin-free -fno-builtin-memcpy -Wno-main -fno-builtin-printf -fno-builtin-fprintf -fno-builtin-vprintf -I. -fno-stack-protector -fno-pie -no-pie
cflags = $(CFLAGS) -Wno-unused -fno-strict-aliasing
board_specs = stub.o
endif

cc = ${toolchain}gcc
ld = ${toolchain}g++
objdump = ${toolchain}objdump
c_useropts = -O2

INC_DIR = 	-I$(KERNEL_ROOT)/services/shell/letter-shell \
			-I$(KERNEL_ROOT)/services/lib/ipc \
			-I$(KERNEL_ROOT)/services/lib/memory \
			-I$(KERNEL_ROOT)/services/lib/serial \
			-I$(KERNEL_ROOT)/services/lib/usyscall \
			-I$(KERNEL_ROOT)/services/fs/libfs \
			-I$(KERNEL_ROOT)/services/net/libnet \
			-I$(KERNEL_ROOT)/services/net/net_server \
			-I$(KERNEL_ROOT)/services/net/net_server/arch \
			-I$(KERNEL_ROOT)/services/net/net_server/include \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/apps \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/priv \
			-I$(KERNEL_ROOT)/services/net/net_server/include/lwip/prot \
			-I$(KERNEL_ROOT)/services/net/net_server/include/netif \
			-I$(KERNEL_ROOT)/services/net/net_server/include/compat \
			-I$(KERNEL_ROOT)/services/semaphore \
			-I$(KERNEL_ROOT)/services/boards/$(BOARD) \
			-I$(KERNEL_ROOT)/services/app 

ifeq ($(BOARD), imx6q-sabrelite)
all: pingpong_client pingpong_server \
	test_fault simple_client simple_server \
	shell fs_server semaphore_server \
	test_sleep test_semaphore test_ipc_null test_thread test_irq_hdlr test_irq_block test_irq_send \
	eth_driver epit_server  readme.txt | bin
else 
ifeq ($(BOARD), jh7110)
all: shell fs_server \
	| bin
else 
all: pingpong_client pingpong_server \
	test_fault simple_client simple_server \
	shell fs_server semaphore_server \
	test_irq_hdlr \
	test_sleep test_ipc_null test_thread test_semaphore readme.txt \
	test_context_user \
	| bin
endif
endif
	../tools/mkfs/mkfs ./fs.img $^
	@mv $(filter-out readme.txt, $^) bin
	@mv *.o bin
	@mv *.asm bin

bin:
	@mkdir -p bin

ifeq ($(BOARD), imx6q-sabrelite)
test_irq_send: test_irq_sender.o usyscall.o arch_usyscall.o libserial.o printf.o 
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

eth_driver: enet_drv.o enet_test.o board_network.o enet_iomux_config.o imx6dq_gpio_map.o gpio.o ccm_pll.o libserial.o printf.o libmem.o usyscall.o arch_usyscall.o session.o libipc.o sabrelite_libtimer.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

epit_server: timer.o epit.o ccm_pll.o usyscall.o arch_usyscall.o libserial.o printf.o libipc.o session.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm
endif

pingpong_client: pingpong_service.o pingpong_client.o libserial.o printf.o libipc.o session.o libfs.o usyscall.o arch_usyscall.o  libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

pingpong_server: pingpong_service.o pingpong_server.o libserial.o printf.o libipc.o session.o libfs.o usyscall.o arch_usyscall.o  libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_sleep: test_sleep.o libserial.o printf.o usyscall.o arch_usyscall.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm
	
test_context_user: test_context_user.o libserial.o printf.o usyscall.o arch_usyscall.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_semaphore: test_semaphore.o libserial.o printf.o usyscall.o arch_usyscall.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_ipc_null: test_ipc_null.o libserial.o printf.o usyscall.o arch_usyscall.o libipc.o session.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

semaphore_server: semaphore_server.o libserial.o printf.o usyscall.o arch_usyscall.o libipc.o session.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_thread: test_thread.o libserial.o printf.o usyscall.o arch_usyscall.o libfs.o libsemaphore.o libipc.o session.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_irq_block: test_irq_block.o libserial.o printf.o libipc.o session.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_irq_hdlr: test_irq_handler.o libserial.o printf.o libipc.o session.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

shell: shell_port.o libserial.o printf.o shell_cmd_list.o shell.o shell_ext.o libfs.o libipc.o session.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm
	
test_fault: test_fault.o libserial.o printf.o usyscall.o arch_usyscall.o 
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

simple_client: simple_client.o libserial.o printf.o libipc.o session.o simple_service.o libfs.o usyscall.o arch_usyscall.o  libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

simple_server: simple_server.o libserial.o printf.o libipc.o session.o simple_service.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

fs_server: fs_server.o libfs.o fs.o libserial.o printf.o libipc.o session.o block_io.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_priority: test_priority.o libserial.o printf.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

test_net: test_net.o lwip_service.o libipc.o session.o libserial.o printf.o usyscall.o arch_usyscall.o libmem.o
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs} -llwip -L$(KERNEL_ROOT)/services/net/net_server
	@${objdump} -S $@ > $@.asm

%.o: %.c
	@${cc} ${cflags} ${c_useropts} ${INC_DIR} -o $@ -c $< 

%.o: %.S
	@${cc} ${cflags} ${c_useropts} -o $@ -c $< 

eth_hal: test_gmac.o hal_gmac.o hal_gmac_3588.o hal_base.o hal_bsp.o hal_pinctrl_v2.o hal_cru.o hal_gpio.o hal_timer.o hal_cru_rk3588.o system_rk3588.o hal_debug.o libserial.o printf.o libmem.o usyscall.o arch_usyscall.o session.o libipc.o 
	@${ld} ${user_ldflags} -e main -o $@ $^ ${board_specs}
	@${objdump} -S $@ > $@.asm

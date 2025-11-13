MAKEFLAGS += --no-print-directory

.PHONY:all clean distclean show_info menuconfig
.PHONY:COMPILE_APP COMPILE_KERNEL

riscv_support := jh7110 
arm_support += imx6q-sabrelite zynq7000-zc702 3568 3588
emulator_support += 
support := $(riscv_support) $(arm_support) $(emulator_support) 
SRC_DIR :=

export BOARD ?= imx6q-sabrelite
# This is the environment variable for kconfig-mconf
export KCONFIG_CONFIG ?= .config

ifeq ($(filter $(BOARD),$(support)),)
$(warning "You should choose board like this: make BOARD=imx6q-sabrelite")
$(warning "This is what we support:")
$(warning "RISCV EVB:    $(riscv_support)")
$(warning "ARM EVB:      $(arm_support)")
$(warning "EMULATORS:    $(emulator_support)")
$(error "break" )
endif

export TARGET 
export COMPILE_TYPE
export KERNEL_ROOT ?=$(strip $(shell pwd))

MAKEFILES  =$(KERNEL_ROOT)/.config
-include $(KERNEL_ROOT)/.config

export BSP_ROOT ?= $(KERNEL_ROOT)/services/boards/$(BOARD)
export UBIQUITOUS_ROOT ?= ..
ifneq ($(findstring $(BOARD), imx6q-sabrelite zynq7000-zc702), )
include $(KERNEL_ROOT)/hardkernel/arch/arm/armv7-a/cortex-a9/preboot_for_$(BOARD)/config.mk
endif
ifneq ($(findstring $(BOARD), 3588), )
include $(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/preboot_for_$(BOARD)/config.mk
endif
ifneq ($(findstring $(BOARD), 3568), )
include $(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/preboot_for_$(BOARD)/config.mk
endif
ifneq ($(findstring $(BOARD), jh7110), )
include $(KERNEL_ROOT)/hardkernel/arch/riscv/rv64gc/preboot_for_$(BOARD)/config.mk
endif
export BSP_BUILD_DIR := $(KERNEL_ROOT)
export HOSTTOOLS_DIR ?= $(KERNEL_ROOT)/services/tools/hosttools
export CONFIG2H_EXE ?= $(HOSTTOOLS_DIR)/xsconfig.sh

export CPPPATHS
export SRC_APP_DIR := ../../APP_Framework
export SRC_KERNEL_DIR := hardkernel kernel_actracer services softkernel
export SRC_DIR := $(SRC_KERNEL_DIR)
export LIBCC 

PART :=

all:
ifeq ($(CONFIG_COMPILER_APP)_$(CONFIG_COMPILER_KERNEL),y_)
PART += COMPILE_APP

else ifeq ($(CONFIG_COMPILER_APP)_$(CONFIG_COMPILER_KERNEL),_y)
PART += COMPILE_KERNEL

else ifeq ($(CONFIG_COMPILER_APP)_$(CONFIG_COMPILER_KERNEL),y_y)
PART := COMPILE_APP COMPILE_KERNEL

else

ifeq ($(CONFIG_LIB_MUSLLIB), y)
PART += COMPILE_MUSL
endif

PART += COMPILE_ALL
endif


all: $(PART)


COMPILE_ALL:
	@for dir in $(SRC_DIR);do    \
               $(MAKE) -C $$dir;          \
       done
	@cp link.mk build/Makefile
	@$(MAKE) -C build TARGET=XiZi-$(BOARD).elf LINK_FLAGS=LFLAGS
	@rm build/Makefile build/make.obj

COMPILE_KERNEL:
	@for dir in $(SRC_KERNEL_DIR);do    \
               $(MAKE) -C $$dir;          \
       done
	@cp link.mk build/Makefile
	@$(MAKE) -C build COMPILE_TYPE="_kernel" TARGET=XiZi-$(BOARD)_kernel.elf LINK_FLAGS=LFLAGS
	@rm build/Makefile build/make.obj

COMPILE_APP:
	@echo $(SRC_APP_DIR)
	@for dir in $(SRC_APP_DIR);do    \
               $(MAKE) -C $$dir USE_APP_INCLUDEPATH=y;          \
       done
	@cp link.mk build/Makefile
	@$(MAKE) -C build COMPILE_TYPE="_app" TARGET=XiZi-$(BOARD)_app.elf LINK_FLAGS=APPLFLAGS
	@rm build/Makefile build/make.obj

show_info:
	@echo "CONFIG_COMPILER_APP is :" $(CONFIG_COMPILER_APP)
	@echo "CONFIG_COMPILER_KERNEL is :" $(CONFIG_COMPILER_KERNEL)
	@echo "KERNELPATHS is :" $(KERNELPATHS)
	@echo "TARGET is :" $(TARGET)
	@echo "VPATH is :" $(VPATH)
	@echo "BSP_ROOT is :" $(BSP_ROOT)
	@echo "KERNEL_ROOT is :" $(KERNEL_ROOT)
	@echo "CPPPATHS is :" $(CPPPATHS)
	@echo "SRC_DIR is :" $(SRC_DIR)
	@echo "BUILD_DIR is :" $(BUILD_DIR)
	@echo "BSP_BUILD_DIR is :" $(BSP_BUILD_DIR)
	@echo "OBJS is :" $(OBJS)
	@for f in $(CPPPATHS); do \
		echo $$f;                \
	done

menuconfig: 
	@if [  -f "$(BSP_ROOT)/.config" ]; then \
  		cp $(BSP_ROOT)/.config $(KERNEL_ROOT)/.config; \
		else if [  -f "$(BSP_ROOT)/.defconfig" ]; then \
		cp $(BSP_ROOT)/.defconfig $(KERNEL_ROOT)/.config ;\
	fi ;fi
	@kconfig-mconf $(BSP_ROOT)/Kconfig
	@$(CONFIG2H_EXE) .config
	@cp $(KERNEL_ROOT)/.config $(BSP_ROOT)/.config

clean:
	@echo Clean target and build_dir
	@rm -rf build
	@rm -rf temp.txt
	@rm -rf services/app/bin
	@rm -f services/app/*.o
	@rm -rf services/tools/mkfs/mkfs
	@rm -rf services/app/fs.img
	@rm -rf services/app/user.map
	@rm -rf services/net/net_server/user.map
	@rm -rf services/net/net_server/bin

distclean:
	@echo Clean all configuration
	@make clean
	@rm -f .config*
	@rm -f $(KERNEL_ROOT)/lib/musllib/libmusl.a
	@rm -f $(KERNEL_ROOT)/board/*/.config


# Run qemu with config discribed in README.md.
.PHONY: qemu-default
qemu-default:
	qemu-system-arm -M sabrelite -m 1G -smp 4 -cpu cortex-a9 \
    -display none -serial null -serial stdio \
    -kernel ./build/XiZi-imx6q-sabrelite.elf


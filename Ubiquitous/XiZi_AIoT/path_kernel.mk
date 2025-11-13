
export KERNELPATHS:= -I$(BSP_ROOT)

ifeq ($(CONFIG_LIB_NEWLIB),y)
KERNELPATHS += -I$(KERNEL_ROOT)/services/lib/newlib/include #
endif

ifeq ($(BOARD), imx6q-sabrelite)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv7-a/cortex-a9/imx6q-sabrelite/include \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv7-a/cortex-a9/include  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv7-a/cortex-a9/$(BOARD)  \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/gicv2 \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/$(BOARD) \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv7-a/cortex-a9/preboot_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv7-a/cortex-a9/uart_io_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L1/arm/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L2/pl310/ 
endif

ifeq ($(BOARD), zynq7000-zc702)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv7-a/cortex-a9/$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv7-a/cortex-a9/include  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv7-a/cortex-a9/$(BOARD)  \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/gicv2 \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/gicv3 \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv7-a/cortex-a9/$(BOARD) \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv7-a/cortex-a9/preboot_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv7-a/cortex-a9/uart_io_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv7-a/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L1/arm/cortex-a9/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L2/pl310/ 
endif

ifeq ($(BOARD), 3588)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv8-a/cortex-a55/$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/preboot_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv8-a/cortex-a55/$(BOARD)  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv8-a/cortex-a55/include  \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv8-a/cortex-a55/include  \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/$(BOARD) \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/gicv3 \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv8-a/cortex-a55/uart_io_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L1/arm/cortex-a55/ 
endif

ifeq ($(BOARD), 3568)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv8-a/cortex-a55/$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/preboot_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv8-a/cortex-a55/$(BOARD)  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/arm/armv8-a/cortex-a55/include  \
	-I$(KERNEL_ROOT)/hardkernel/clock/arm/armv8-a/cortex-a55/include  \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/$(BOARD) \
	-I$(KERNEL_ROOT)/hardkernel/intr/arm/armv8-a/cortex-a55/gicv3 \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv8-a/cortex-a55/uart_io_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/uart/arm/armv8-a/cortex-a55/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/L1/arm/cortex-a55/ 
endif


ifeq ($(BOARD), jh7110)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel/arch/riscv/rv64gc/ \
	-I$(KERNEL_ROOT)/hardkernel/arch/riscv/rv64gc/include \
	-I$(KERNEL_ROOT)/hardkernel/arch/riscv/rv64gc/preboot_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/cache/L1/riscv/rv64gc/ \
	-I$(KERNEL_ROOT)/hardkernel/clock/riscv/rv64gc/$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/intr/riscv/rv64gc/ \
	-I$(KERNEL_ROOT)/hardkernel/intr/riscv/rv64gc/$(BOARD) \
	-I$(KERNEL_ROOT)/hardkernel/intr/riscv/rv64gc/gicv3 \
	-I$(KERNEL_ROOT)/hardkernel/mmu/riscv/rv64gc/include  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/riscv/rv64gc/$(BOARD)  \
	-I$(KERNEL_ROOT)/hardkernel/uart/riscv/rv64gc/uart_io_for_$(BOARD)/include \
	-I$(KERNEL_ROOT)/hardkernel/uart/riscv/rv64gc/
endif

KERNELPATHS += \
	-I$(KERNEL_ROOT)/hardkernel \
	-I$(KERNEL_ROOT)/hardkernel/clock/  \
	-I$(KERNEL_ROOT)/hardkernel/mmu/  \
	-I$(KERNEL_ROOT)/hardkernel/intr/ \
	-I$(KERNEL_ROOT)/hardkernel/uart/ \
	-I$(KERNEL_ROOT)/hardkernel/cache/ 

KERNELPATHS += \
	-I$(KERNEL_ROOT)/softkernel/include \
	-I$(KERNEL_ROOT)/kernel_actracer/ \
	-I$(KERNEL_ROOT)/include

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/general_functions/list #



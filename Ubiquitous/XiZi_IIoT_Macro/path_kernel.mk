
export KERNELPATHS:= -I$(BSP_ROOT)

ifeq ($(CONFIG_LIB_MUSLLIB), y)
KERNELPATHS += -I$(KERNEL_ROOT)/lib/musllib/src/include \
	-I$(KERNEL_ROOT)/lib/musllib/include \
	-I$(KERNEL_ROOT)/lib/musllib/src/internal #
# chose arch for musl
ifeq ($(ARCH), arm)
KERNELPATHS += -I$(KERNEL_ROOT)/lib/musllib/arch/arm
endif
ifeq ($(ARCH), risc-v)
KERNELPATHS += -I$(KERNEL_ROOT)/lib/musllib/arch/riscv64
endif

endif # end of musl include path

ifeq ($(CONFIG_LIB_NEWLIB),y)
KERNELPATHS += -I$(KERNEL_ROOT)/lib/newlib/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rzv2l-m33)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m33 \
	-I$(BSP_ROOT)/rzv/arm/CMSIS_5/CMSIS/Core/Include \
	-I$(BSP_ROOT)/rzv/board/rzv2l_smarc \
	-I$(BSP_ROOT)/rzv/fsp/inc \
	-I$(BSP_ROOT)/rzv/fsp/inc/api \
	-I$(BSP_ROOT)/rzv/fsp/inc/instances \
	-I$(BSP_ROOT)/rzv/fsp/src/bsp/cmsis/Device/RENESAS/Include \
	-I$(BSP_ROOT)/rzv/fsp/src/bsp/cmsis/Device/RENESAS/Include/iobitmasks  \
	-I$(BSP_ROOT)/rzv/fsp/src/bsp/cmsis/Device/RENESAS/Include/iodefines  \
	-I$(BSP_ROOT)/rzv/fsp/src/bsp/mcu/all  \
	-I$(BSP_ROOT)/rzv/fsp/src/bsp/mcu/rzv2l  \
	-I$(BSP_ROOT)/rzv_cfg/fsp_cfg  \
	-I$(BSP_ROOT)/rzv_cfg/fsp_cfg/bsp  \
	-I$(BSP_ROOT)/rzv_gen \
	-I$(BSP_ROOT)/rzv/linaro/libmetal/include \
	-I$(BSP_ROOT)/rzv/linaro/open-amp/lib/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rzg2ul-m33)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m33 \
	-I$(BSP_ROOT)/amp/include \
	-I$(BSP_ROOT)/amp/include/config \
	-I$(BSP_ROOT)/amp/include/config/ipi \
	-I$(BSP_ROOT)/amp/include/config/service \
	-I$(BSP_ROOT)/rzg/arm/CMSIS_5/CMSIS/Core/Include \
	-I$(BSP_ROOT)/rzg/board/rzg2ul_smarc \
	-I$(BSP_ROOT)/rzg/fsp/inc \
	-I$(BSP_ROOT)/rzg/fsp/inc/api \
	-I$(BSP_ROOT)/rzg/fsp/inc/instances \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G043U \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G043U/iobitmasks \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G043U/iodefines \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G044L \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G044L/iobitmasks \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A07G044L/iodefines \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A08G045S \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A08G045S/iobitmasks \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/cmsis/Device/RENESAS/Include/R9A08G045S/iodefines \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/mcu/all  \
	-I$(BSP_ROOT)/rzg/fsp/src/bsp/mcu/rzg2ul  \
	-I$(BSP_ROOT)/rzg_cfg/fsp_cfg  \
	-I$(BSP_ROOT)/rzg_cfg/fsp_cfg/bsp  \
	-I$(BSP_ROOT)/rzg_gen \
	-I$(BSP_ROOT)/rzg/linaro/libmetal/include \
	-I$(BSP_ROOT)/rzg/linaro/open-amp/lib/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/xidatong-arm32)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/inc \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/port \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host/class \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/osa \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/phy \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/lan8720 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052/drivers \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/xip \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include 

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif 
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m7-emulator)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/CMSIS \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include\
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/V2M \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include 


endif 



ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32h750)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32H7xx_HAL/inc \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32H7xx_HAL/inc/Legacy \
	-I$(BSP_ROOT)/third_party_driver/libraries/CMSIS \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include \
	-I$(KERNEL_ROOT)/tool/shell/letter-shell \
	-I$(KERNEL_ROOT)/fs/shared/include \
	-I$(KERNEL_ROOT)/arch/cortex-m7/stm32h750 \
	-I$(BSP_ROOT)/include 
endif 


ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/xiwangtong-arm32)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/inc \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/port \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host/class \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/osa \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/phy \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/lan8720 \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/xip \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include 

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif 
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/xidatong-riscv64)
KERNELPATHS += \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/edu-riscv64)
KERNELPATHS += \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif 
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/kd233)
KERNELPATHS += \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/maix-go)
KERNELPATHS += \
	-I$(BSP_ROOT)/include \
    -I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f407-st-discovery)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 
endif
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/aiit-arm32-board)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f407zgt6)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio\
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32g474)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT) \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/stm32cube_g4/STM32G4xx_HAL_Driver/Inc \
	-I$(BSP_ROOT)/third_party_driver/usb/inc \
	-I$(KERNEL_ROOT)/include 
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32l476rgt6)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32L4xx_HAL_Driver/inc \
	-I$(BSP_ROOT)/third_party_driver/usb/usb_device/include \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/at32f437vmt7)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/libraries/cmsis/cm4/core_support \
	-I$(BSP_ROOT)/third_party_driver/libraries/cmsis/cm4/device_support \
	-I$(BSP_ROOT)/third_party_driver/libraries/drivers/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/gapuino)
KERNELPATHS += \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/gd32vf103-rvstar)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/risc-v/gd32vf103-rvstar \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/ch32v307vct6)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v307vct6/Core \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v307vct6/Debug \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v307vct6/User \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v307vct6 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/usb_drv/inc \
	-I$(BSP_ROOT)/third_party_driver/Peripheral/inc \
	-I$(BSP_ROOT)/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/ch32v208rbt6)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v208rbt6/Core \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v208rbt6/Debug \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v208rbt6/User \
	-I$(KERNEL_ROOT)/arch/risc-v/ch32v208rbt6 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/Peripheral/inc \
	-I$(BSP_ROOT)/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/ch569w)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/risc-v/ch569w \
	-I$(KERNEL_ROOT)/arch/risc-v/ch569w/RVMSIS \
	-I$(KERNEL_ROOT)/arch/risc-v/ch569w/User \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/Peripheral/inc \
	-I$(BSP_ROOT)/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rv32m1-vega)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/risc-v/rv32m1-vega \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/hifive1-rev-B)
KERNELPATHS += \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/sifive \
	-I$(BSP_ROOT)/third_party_driver/include/sifive/devices \
	-I$(BSP_ROOT)/third_party_driver/fe300prci \
	-I$(BSP_ROOT)/third_party_driver/plic\
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/hifive1-emulator)
KERNELPATHS += \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/sifive \
	-I$(BSP_ROOT)/third_party_driver/include/sifive/devices \
	-I$(BSP_ROOT)/third_party_driver/fe300prci \
	-I$(BSP_ROOT)/third_party_driver/plic\
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/k210-emulator)
KERNELPATHS += \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/aiit-riscv64-board)
KERNELPATHS += \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/src/radio \
	-I$(BSP_ROOT)/third_party_driver/camera \
	-I$(BSP_ROOT)/third_party_driver/drivers \
	-I$(BSP_ROOT)/third_party_driver/lcd \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m0-emulator)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m0 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/Libraries/driverlib \
	-I$(BSP_ROOT)/third_party_driver/Libraries/ \
	-I$(BSP_ROOT)/third_party_driver/Libraries/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m3-emulator)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m3 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/Libraries/driverlib \
	-I$(BSP_ROOT)/third_party_driver/Libraries/ \
	-I$(BSP_ROOT)/third_party_driver/Libraries/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/cortex-m4-emulator)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/sdio\
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/STM32_USB_OTG_Driver/inc \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/ok1052-c)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/inc \
	-I$(BSP_ROOT)/third_party_driver/sdio/sdmmc/port \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/host/class \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/include \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/osa \
	-I$(BSP_ROOT)/third_party_driver/usb/nxp_usb_driver/phy \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/ksz8081 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052 \
	-I$(BSP_ROOT)/third_party_driver/MIMXRT1052/drivers \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/xip \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include 

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rt1062xb)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/periph \
	-I$(BSP_ROOT)/third_party_driver/CMSIS \
	-I$(BSP_ROOT)/include \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/imxrt1176-sbc)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/Include \
	-I$(BSP_ROOT)/third_party_driver/ethernet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/mdio/enet \
	-I$(BSP_ROOT)/third_party_driver/ethernet/mdio/enet_qos \
	-I$(BSP_ROOT)/third_party_driver/ethernet/phy/ksz8081 \
	-I$(BSP_ROOT)/third_party_driver/ethernet/phy/ksz9131rnx \
	-I$(BSP_ROOT)/third_party_driver/cm7 \
	-I$(BSP_ROOT)/xip \
	-I$(KERNEL_ROOT)/include \
	-I$(KERNEL_ROOT)/resources/include \
	
ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/stm32f103-nano)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m3 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32F1xx_HAL/inc \
	-I$(BSP_ROOT)/third_party_driver/libraries/STM32F1xx_HAL/inc/Legacy \
	-I$(BSP_ROOT)/third_party_driver/libraries/CMSIS \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/nuvoton-m2354)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m23 \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/include/Drivers \
	-I$(BSP_ROOT)/third_party_driver/include/Regs \
	-I$(BSP_ROOT)/third_party_driver/include/NuUtils/inc \
	-I$(BSP_ROOT)/third_party_driver/common/NuClockConfig \
	-I$(BSP_ROOT)/third_party_driver/common/NuPinConfig \
	-I$(KERNEL_ROOT)/include \
	-I$(BSP_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/edu-arm32)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4/hc32f4a0 \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/common/hc32_ll_driver/inc \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/sx12xx/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/sx12xx/src/radio \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib/host_class/msc \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib/host_core \
	-I$(KERNEL_ROOT)/include #

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif 
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/imx8mp)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m7 \
	-I$(BSP_ROOT)/ \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/rpmsg_lite/lib/include \
	-I$(BSP_ROOT)/rpmsg_lite/lib/include/platform/imx8mp_m7 \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/gd32f415rgo6)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4/hc32f4a0 \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/rk3562-mcu)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m0 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/hal/inc \
	-I$(BSP_ROOT)/rpmsg-lite/lib/include \
	-I$(KERNEL_ROOT)/include #
endif

ifeq ($(BSP_ROOT),$(KERNEL_ROOT)/board/xishutong-arm32)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4/hc32f4a0 \
	-I$(KERNEL_ROOT)/arch/arm/cortex-m4 \
	-I$(BSP_ROOT)/third_party_driver \
	-I$(BSP_ROOT)/third_party_driver/common/hc32_ll_driver/inc \
	-I$(BSP_ROOT)/include \
	-I$(BSP_ROOT)/third_party_driver/include \
	-I$(BSP_ROOT)/third_party_driver/CMSIS/include \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/sx12xx/inc \
	-I$(BSP_ROOT)/third_party_driver/spi/third_party_spi_lora/sx12xx/src/radio \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib/host_class/msc \
	-I$(BSP_ROOT)/third_party_driver/usb/hc32_usb_driver/usb_host_lib/host_core \
	-I$(KERNEL_ROOT)/include #

ifeq ($(CONFIG_RESOURCES_LWIP),y)
KERNELPATHS += \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/compat \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/netif \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/apps \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/priv \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/include/lwip/prot \
	-I$(KERNEL_ROOT)/resources/ethernet/LwIP/arch 

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include/netdev
endif 
endif

KERNELPATHS += -I$(KERNEL_ROOT)/arch \
            -I$(KERNEL_ROOT)/arch/risc-v/shared/kernel_service #

ifeq ($(CONFIG_FS_VFS), y)
KERNELPATHS += -I$(KERNEL_ROOT)/fs/devfs \
	-I$(KERNEL_ROOT)/fs/fatfs \
	-I$(KERNEL_ROOT)/fs/shared/include #
endif

ifeq ($(CONFIG_FS_CH376), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/fs/compatibility_ch376 #
endif

ifeq ($(CONFIG_TRANSFORM_LAYER_ATTRIUBUTE), y)
ifeq ($(CONFIG_ADD_XIZI_FEATURES), y)
ifeq ($(CONFIG_LIB_MUSLLIB), )
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi/user_api/posix_support/include #
endif
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/transform_layer/xizi #
endif

ifeq ($(CONFIG_ADD_NUTTX_FEATURES), y)
#
endif

ifeq ($(CONFIG_ADD_RTTHREAD_FEATURES), y)
#
endif

endif

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/general_functions/circular_area #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/general_functions/list #

ifeq ($(CONFIG_SUPPORT_SENSOR_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/sensor #
endif

ifeq ($(CONFIG_SUPPORT_CONNECTION_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/zigbee #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_network/freemodbus_tcp #

ifeq ($(CONFIG_CONNECTION_MODBUS), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_fieldbus/modbus/freemodbus-latest/modbus/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_fieldbus/modbus/freemodbus-latest/port #

ifeq ($(CONFIG_CONNECTION_MODBUS_USING_RTU), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_fieldbus/modbus/freemodbus-latest/modbus/rtu #
endif

ifeq ($(CONFIG_CONNECTION_MODBUS_USING_TCP), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_fieldbus/modbus/freemodbus-latest/modbus/tcp #
endif
endif
endif

ifeq ($(CONFIG_ADAPTER_HFA21_ETHERCAT), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/connection/industrial_network/ethercat #
endif

ifeq ($(CONFIG_SUPPORT_KNOWING_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/gemmlowp #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/flatbuffers/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/tensorflow-lite/tensorflow-lite-for-mcu/source/third_party/ruy #

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/kpu/k210_yolov2_detect_procedure #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/kpu/yolov2 #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/kpu/yolov2_json #

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/nnom/inc #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/nnom/inc/layers #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/nnom/port #

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/cmsis_5/Core/Include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/cmsis_5/DSP/Include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/cmsis_5/DSP/Include/dsp #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/knowing/cmsis_5/NN/Include #

KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/knowing_app/cmsis_5_demo/cmsisnn-cifar10/model/m4 #
endif

ifeq ($(CONFIG_LIB_LV),y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lvgl #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lvgl/examples/porting #
endif

ifeq ($(CONFIG_SUPPORT_CONTROL_FRAMEWORK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/shared #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/modbus_tcp #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/modbus_uart #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/ethercat #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/ethercat/comm #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/ethercat/soem #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/fins #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/melsec #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/opcua #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/s7 #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/ipc_protocol/freemodbustcpserver #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Framework/control/plc_protocol/cip #
endif

ifeq ($(CONFIG_LIB_USING_CJSON), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/cJSON
endif

ifeq ($(CONFIG_LIB_USING_FREEMODBUS), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/freemodbus
endif

ifeq ($(CONFIG_LIB_USING_SQLITE), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/SQLite #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/SQLite/xizi_port #
endif

ifeq ($(CONFIG_LIB_USING_LORAWAN), y)
ifeq ($(CONFIG_LIB_USING_LORA_RADIO), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/lora-radio/common #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/lora-radio/include #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/lora-radio/sx126x #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/lora-radio/sx127x #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/ports/lora-module/inc #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lora_radio_driver/ports/samples/lora-radio-tester #
endif

ifeq ($(CONFIG_LIB_USING_LORAWAN_ED_STACK), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_devicenode/lorawan-ed-stack/Aps/Basic #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_devicenode/lorawan-ed-stack/Common #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_devicenode/lorawan-ed-stack/Mac #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_devicenode/lorawan-ed-stack/Mac/Crypto #
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_devicenode/lorawan-ed-stack/Phy/region #
endif

ifeq ($(CONFIG_LIB_USING_LORAWAN_GATEWAY_SC),y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/lorawan/lorawan_gateway_single_channel/inc #
endif
endif

ifeq ($(CONFIG_CRYPTO), y)
KERNELPATHS += -I$(KERNEL_ROOT)/framework/security/crypto/include #
endif

KERNELPATHS += -I$(KERNEL_ROOT)/resources/include #

ifeq ($(CONFIG_RESOURCES_SPI), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/resources/spi #

ifeq ($(CONFIG_RESOURCES_SPI_SFUD),y)
KERNELPATHS += -I$(KERNEL_ROOT)/resources/spi/third_party_spi/SFUD/sfud/inc #
endif

endif

ifeq ($(CONFIG_APP_USING_WEBNET),y)
	KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/Applications/webnet/WebNet_XiUOS/inc \
				   -I$(KERNEL_ROOT)/../../APP_Framework/Applications/webnet/WebNet_XiUOS/samples #
endif


ifeq ($(CONFIG_TOOL_SHELL), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/tool/shell/letter-shell \
	-I$(KERNEL_ROOT)/tool/shell/letter-shell/file_ext \
	-I$(KERNEL_ROOT)/tool/shell/
endif

ifeq ($(CONFIG_TOOL_USING_OTA), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/tool/bootloader/flash \
	-I$(KERNEL_ROOT)/tool/bootloader/ota #
endif

ifeq ($(CONFIG_LIB_USING_MQTT), y)
KERNELPATHS +=-I$(KERNEL_ROOT)/../../APP_Framework/lib/mqtt
endif

ifeq ($(CONFIG_LIB_USING_JERRYSCRIPT), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/JerryScript
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/JerryScript/jerryscript/jerry-core/include
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/JerryScript/jerryscript/jerry-ext/include
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/JerryScript/jerryscript/jerry-math/include
endif

ifeq ($(CONFIG_LIB_USING_PIKAPYTHON), y)
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/pikapython/pikascript-core
KERNELPATHS += -I$(KERNEL_ROOT)/../../APP_Framework/lib/pikapython/pikascript-api
endif

ifeq ($(CONFIG_FS_LWEXT4),y)
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/blockdev/xiuos #
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/include #
KERNELPATHS += -I$(KERNEL_ROOT)/fs/lwext4/lwext4_submodule/include/misc 
endif


ifeq ($(ARCH), risc-v)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/shared
ifeq ($(MCU), k210)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/k210
endif
ifeq ($(MCU), FE310)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/fe310
endif
ifeq ($(MCU), GAP8)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/gap8
endif
ifeq ($(MCU), GD32VF103)
	KERNELPATHS +=-I$(KERNEL_ROOT)/arch/risc-v/gd32vf103
endif
endif


ifeq ($(ARCH), arm)
KERNELPATHS +=-I$(KERNEL_ROOT)/arch/arm/shared \
	-I$(KERNEL_ROOT)/lib/comlibc/common #
endif


KERNELPATHS += -I$(KERNEL_ROOT)/kernel/include #


# COMPILE_KERNEL:
# 	@$(eval CPPPATHS=$(KERNELPATHS))
# 	@for dir in $(SRC_KERNEL_DIR);do    \
#                $(MAKE) -C $$dir;          \
#        done
# 	@cp link.mk build/Makefile
# 	@$(MAKE) -C build COMPILE_TYPE="_kernel" TARGET=XiZi-$(BOARD)_kernel.elf LINK_FLAGS=LFLAGS
# 	@rm build/Makefile build/make.obj


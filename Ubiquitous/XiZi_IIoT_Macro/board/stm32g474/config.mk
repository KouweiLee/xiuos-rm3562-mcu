export CROSS_COMPILE ?=/usr/bin/arm-none-eabi-

SRC_KERNEL_DIR += board/stm32g474

export CFLAGS := -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -Dgcc -DSTM32G474xx -Os -gdwarf-2 -g -fgnu89-inline -Wa,-mimplicit-it=thumb 
export AFLAGS := -c -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb -gdwarf-2
export LFLAGS := -mcpu=cortex-m4 -specs=nano.specs -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-stm32g474.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds
export CXXFLAGS := -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -Dgcc -DSTM32G474xx -Os -gdwarf-2 -g 

export APPLFLAGS := -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds

export DEFINES := -DHAVE_CCONFIG_H

export ARCH = arm
export MCU = cortex-m4

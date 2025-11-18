export CROSS_COMPILE ?=/usr/bin/arm-none-eabi-

export CFLAGS := -mcpu=cortex-m7 -mthumb  -ffunction-sections -fdata-sections -Dgcc -Os -gdwarf-2 -g -fgnu89-inline -Wa,-mimplicit-it=thumb 
export AFLAGS := -c -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2
export LFLAGS := -mcpu=cortex-m7 -specs=nano.specs -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-stm32h750.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds
export CXXFLAGS := -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -Dgcc -Os -gdwarf-2 -g 

export APPLFLAGS := -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds

export DEFINES := -DHAVE_CCONFIG_H  

export ARCH = arm
export MCU = cortex-m7

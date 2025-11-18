export CROSS_COMPILE ?=/usr/bin/arm-none-eabi-

export CFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -fgnu89-inline -Wa,-mimplicit-it=thumb -Werror
export AFLAGS := -c -mcpu=cortex-m7 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2
export LFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-cortex-m7-emulator.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds
export CXXFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -Werror


export APPLFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds


export DEFINES := -DHAVE_CCONFIG_H  -DCPU_MIMXRT1052CVL5B

export ARCH = arm
export MCU = cortex-m7
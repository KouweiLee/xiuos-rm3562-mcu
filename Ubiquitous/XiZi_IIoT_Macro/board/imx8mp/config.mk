export CROSS_COMPILE ?=/usr/bin/arm-none-eabi-

export CFLAGS := -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Dgcc -O0 -g -gdwarf-2 -fgnu89-inline -Wa,-mimplicit-it=thumb
export LFLAGS += -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -mfloat-abi=hard -mfpu=fpv5-sp-d16 -Wl,--gc-sections,-Map=imx8mp.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds

export AFLAGS := -c -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -mfloat-abi=hard -mfpu=fpv5-sp-d16 -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2

export CXXFLAGS := -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g

export APPLFLAGS := -mcpu=cortex-m7 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-app.map,-cref,-u, -T $(BSP_ROOT)/link.lds


export DEFINES := -DHAVE_CCONFIG_H -D__STARTUP_INITIALIZE_NONCACHEDATA -D__STARTUP_CLEAR_BSS -DNDEBUG -DCPU_MIMX8ML8DVNLZ -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DSERIAL_PORT_TYPE_UART=1
export ARCH = arm
export MCU = cortex-m7

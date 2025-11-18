export CROSS_COMPILE ?=/usr/local/arm/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-

# DEBUG
# CFLAGS := -mthumb -mcpu=cortex-m33+nodsp+nofp -fdiagnostics-parseable-fixits -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections 
# CFLAGS += -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal
# CFLAGS += -Wnull-dereference 
# CFLAGS += -g -std=c99 -mcmse

# RELEASE
CFLAGS := -mthumb -mcpu=cortex-m33+nodsp+nofp -fdiagnostics-parseable-fixits -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
CFLAGS += -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op
CFLAGS += -Waggregate-return -Wfloat-equal -Wnull-dereference -g -std=c99 -mcmse

export CFLAGS

export LFLAGS := -mthumb -mcpu=cortex-m33+nodsp+nofp -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi-rzv2l-m33.map,-cref,-u,Warm_Reset_S -T $(BSP_ROOT)/link.lds

export DEFINES := -DHAVE_CCONFIG_H  

export ARCH = arm
export MCU = cortex-m33

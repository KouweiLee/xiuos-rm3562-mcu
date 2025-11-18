source /usr/local/arm/arm64/environment-setup-aarch64-poky-linux

aarch64-poky-linux-gcc \
    -mtune=cortex-a55 -fstack-protector-strong -D_FORTIFY_SOURCE=2 \
    -Wformat -Wformat-security -Werror=format-security \
    --sysroot=/usr/local/arm/arm64/sysroots/aarch64-poky-linux \
    -o amp_echo_test \
    amp_echo_test.c \
    ../src/channel.c \
    ../src/client.c \
    ../src/config.c \
    ../src/msgqueue.c \
    ../src/msg.c \
    ../src/shm.c \
    ../src/spinlock.c \
    -Wall -O2 -g -DCFG_CA5X -pthread

sudo chmod 755 amp_echo_test

set +e
sudo rm /media/ubuntu/RZ_ext/home/root/amp_echo_test
set -e

sudo mv amp_echo_test /media/ubuntu/RZ_ext/home/root

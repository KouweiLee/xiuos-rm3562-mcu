#! /bin/env sh
export CROSS_COMPILE=/home/syg/data/ch32v208/xpack-riscv-none-elf-gcc-11.3.0-1/bin/riscv-none-elf-
make BOARD=ch32v208rbt6 distclean
make BOARD=ch32v208rbt6 menuconfig
make BOARD=ch32v208rbt6


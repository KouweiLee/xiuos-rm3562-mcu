echo "Post build script started"

inputfilename=/home/ubuntu/XiUOS/xiuos/Ubiquitous/XiZi_IIoT/build/XiZi-rzv2l-m33.elf

filebody=rzv2l_cm33_demo

/usr/local/arm/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy -O binary -j .text.secure_vector ${inputfilename} ${filebody}_secure_vector.bin
/usr/local/arm/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy -O binary -j .text.non_secure_vector ${inputfilename} ${filebody}_non_secure_vector.bin
/usr/local/arm/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy -O binary -j .text.secure_code ${inputfilename} ${filebody}_secure_code.bin
/usr/local/arm/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy -O binary -j .text.non_secure_code -j .data ${inputfilename} ${filebody}_non_secure_code.bin

echo "Post build script complete"

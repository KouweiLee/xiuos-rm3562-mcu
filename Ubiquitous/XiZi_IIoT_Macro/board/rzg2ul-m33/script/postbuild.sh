echo "Post build script started"

inputfilename=/home/ubuntu/Desktop/XiUOS/xiuos/Ubiquitous/XiZi_IIoT/build/XiZi-rzg2ul-m33.elf

filebody=rzg2ul_cm33_rpmsg_demo

tool_chain_dir=/usr/local/arm/gcc-arm-none-eabi-10.3-2021.10

${tool_chain_dir}/bin/arm-none-eabi-objcopy -O binary -j .text.secure_vector ${inputfilename} ${filebody}_secure_vector.bin
${tool_chain_dir}/bin/arm-none-eabi-objcopy -O binary -j .text.non_secure_vector ${inputfilename} ${filebody}_non_secure_vector.bin
${tool_chain_dir}/bin/arm-none-eabi-objcopy -O binary -j .text.secure_code ${inputfilename} ${filebody}_secure_code.bin
${tool_chain_dir}/bin/arm-none-eabi-objcopy -O binary -j .text.non_secure_code -j .data ${inputfilename} ${filebody}_non_secure_code.bin

echo "Post build script complete"
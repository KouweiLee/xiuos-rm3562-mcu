OBJS := $(shell cat make.obj)

$(TARGET): $(OBJS)
	@echo ------------------------------------------------
	@echo link $(TARGET)
	@$(CROSS_COMPILE)g++ -o $@ $($(LINK_FLAGS)) $(OBJS) $(LINK_LWIP) $(LINK_MUSLLIB) $(LINK_MONGOOSE) $(LINK_WCH_NET) $(LIBCC) $(LINK_WCH_BLE) $(LINK_BOARD)
	@echo ------------------------------------------------
	@$(CROSS_COMPILE)objcopy -O binary $@ XiZi-$(BOARD)$(COMPILE_TYPE).bin
	@$(CROSS_COMPILE)objdump -S $@ > XiZi-$(BOARD)$(COMPILE_TYPE).asm
	@$(CROSS_COMPILE)objcopy -O ihex $@  XiZi-$(BOARD)$(COMPILE_TYPE).hex
	@$(CROSS_COMPILE)size $@
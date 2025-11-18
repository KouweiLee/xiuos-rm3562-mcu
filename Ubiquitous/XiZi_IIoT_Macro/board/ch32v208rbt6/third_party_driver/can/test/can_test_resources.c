#include <bus.h>
#include <transform.h>

/**
 * @brief  ch32v208测试can resources
 * @param  argc   
 * @param  argv   
 * @return int    
 * @note   208时钟频率为120MHz，这样默认CAN波特率为250k。当前设置无过滤，如需过滤帧，需要修改CanModeInit函数
 */
int test_can_resources(int argc, char *argv[]) {
    int ret = EOK;
    KPrintf("Test can start\n");
    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    char test_str[] = "Hello AIIT!\r\n";
    /* find the can bus pointer */
    bus = BusFind(CAN_BUS_NAME);
    if (NONE == bus)
    {
        KPrintf("BusFind %s failed\n", CAN_BUS_NAME);
        return 1;
    }
    /* find the can driver pointer */
    drv = BusFindDriver(bus, CAN_DRIVER_NAME);
    if (NONE == drv)
    {
        KPrintf("BusFindDriver %s failed\n", CAN_DRIVER_NAME);
        return 1;
    }
    /* find the can device pointer */
    dev = BusFindDevice(bus, CAN_DEVICE_NAME_1);
    if (NONE == dev)
    {
        KPrintf("BusFindDevice %s failed\n", CAN_DEVICE_NAME_1);
        return 1;
    }

    /*step 1: init bus_driver, change struct CanDriverConfigure if necessary*/
    struct BusConfigureInfo configure_info;
    struct CanDriverConfigure can_cfg;
    memset(&can_cfg, 0, sizeof(struct CanDriverConfigure));
	/* the baud rate is 250kHz */
    can_cfg.tsjw = CAN_SJW_1tq;
    can_cfg.tbs2 = CAN_BS2_5tq;
    can_cfg.tbs1 = CAN_BS1_6tq;
    can_cfg.brp = 20;
    can_cfg.mode = CAN_Mode_Normal;
    configure_info.private_data = &can_cfg;
    ret = BusDrvConfigure(drv, &configure_info); // it will call the method [CanModeInit]
    if (EOK != ret) {
        KPrintf("BusDrvConfigure  OPE_INT failed error code %d\n", ret);
        return ret;
    }

    /*step 2: match can bus_driver with bus_device*/
    bus->match(drv, dev);

    /*step 3: open bus_device*/
    ret = BusDevOpen(dev);
    if (EOK != ret) {
        KPrintf("BusDevOpen failed error code %d\n", ret);
        return ret;
    }

    /*step 4: write can data, configure struct BusBlockWriteParam*/
    struct BusBlockWriteParam write_param;
    write_param.pos = 0;
    write_param.buffer = (void *)test_str;
    write_param.size = sizeof(test_str) - 1;
    BusDevWriteData(dev, &write_param);

	/*step5: read data*/
	struct BusBlockReadParam read_param;
	unsigned char read_str[128] = {};	
	read_param.buffer = read_str;
	int read_len = 0;
	while (1) {
		BusDevWriteData(dev, &write_param);
		memset(read_str, 0, sizeof(read_str));
		read_len = BusDevReadData(dev, &read_param);
		KPrintf("read %d bytes:", read_len);
		for (int i = 0; i < read_len; i++) {
			KPrintf("0x%02x ", read_str[i]);
		}
		KPrintf("\n");
		Delay_Ms(1000);
	}

    return EOK;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), test_can_resources, test_can_resources,
                 test CAN resources);
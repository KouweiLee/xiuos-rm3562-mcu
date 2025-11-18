
0.该测试在Ubiquitous中测试通过（将驱动中静态的收发初始化操作在测试文件中实现），在APP_Framework的测试中BusFind，BusFindDriver，BusFindDevice可找到相应的设备，但无法通过其进行读写等操作

```
// APP_Framework/Applications/main.c

#include <stdio.h>
#include <string.h>
#include <transform.h>
#include <bus.h>
#define CAN_BUS "can1"

#define CAN_DRIVER "can1_drv"

#define CAN_DEVICE "can1_dev1"

void TestCAN(void)
{
    KPrintf("Test can start\n");
    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    char test_str[] = "Hello AIIT!\r\n";
    /* find the serial bus pointer */
    bus = BusFind(CAN_BUS);
    if (NONE == bus)
    {
        KPrintf("BusFind %s failed\n", CAN_BUS);
        return;
    }
    /* find the serial driver pointer */
    drv = BusFindDriver(bus, CAN_DRIVER);
    if (NONE == drv)
    {
        KPrintf("BusFindDriver %s failed\n", CAN_DRIVER);
        return;
    }
    /* find the serial device pointer */
    dev = BusFindDevice(bus, CAN_DEVICE);
    if (NONE == dev)
    {
        KPrintf("BusFindDevice %s failed\n", CAN_DEVICE);
        return;
    } 
}

PRIV_SHELL_CMD_FUNCTION(TestCAN, a can test sample, PRIV_SHELL_CMD_MAIN_ATTR);
```

1.make BOARD=ch32v307vct6 menuconfig中配置can相关参数 ch32v307vct6 feature  ---> Using CAN device  ---> 

2.在can_test.c中can_initstruction.CAN_Mode表示can的收发模式，修改该参数可实现不同的收发方式，有CAN_Mode_Normal,CAN_Mode_LoopBack,CAN_Mode_Silent,CAN_Mode_Silent_LoopBack四种模式，此处用CAN_Mode_Silent_LoopBack该模式，实现自收自发，如果需要通过主机收发可修改为CAN_Mode_Normal

3.编译后将其烧录至开发板上，执行test_can命令，运行can测试。


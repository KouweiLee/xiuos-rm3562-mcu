0.该测试在Ubiquitous中测试通过（将驱动中静态的收发操作在测试文件中实现），在APP_Framework的测试中BusFind，BusFindDriver，BusFindDevice可找到相应的设备，但无法通过其进行读写等操作

```
// APP_Framework/Applications/main.c

#include <stdio.h>
#include <string.h>
#include <transform.h>
#include <bus.h>
#define RS485_UART_BUS "uart5"
#define RS485_UART_DRV "uart5_drv"
#define RS485_UART_DEV "uart5_dev5"

void Test485(void)
{
    /*
    485的驱动，使用UART5，在connect_uart.c中添加UART5的初始化，添加UART5的Kconfig，borad.c中使用InstallConsole用uart5串口正常输出控制台信息；在app_famework中加入app_test的485测试用例后出现

    HardFault_Handler.
    mepc  :000f448
    mcause:0000007
    mtval :200100d4
    
    报错，控制台停止运行，最后将Test485用例摘出放置main.c中，发现将PrivOpen编译进bin就会报错（没有实际执行）
    
    试了试busfind，bus,bus_drv,bus_dev都能找到，但是BusDevWriteData(dev, &write_param)编译进去后无法运行（OS无法起来/没有实际执行）；Busdevclose导致程序崩溃
    
    Test 485 start
    [ERR][x_free] Freeing a unallocated address.
    HardFault_Handler.
    mepc  :0000012
    mcause:0000002
    mtval :0000000
    
    在Ubiquitous/XiZi_IIoT/board/ch32v307vct6/third_party_driver/uart/test测试中
    可以通过485串口正常输出接收
    
    */
    
    KPrintf("Test 485 start\n");
    struct Bus *bus;
    struct HardwareDev *dev;
    struct Driver *drv;
    char test_str[] = "Hello AIIT!\r\n";
    /* find the serial bus pointer */
    bus = BusFind(RS485_UART_BUS);
    if (NONE == bus)
    {
        KPrintf("BusFind %s failed\n", RS485_UART_BUS);
        return;
    }
    /* find the serial driver pointer */
    drv = BusFindDriver(bus, RS485_UART_DRV);
    if (NONE == drv)
    {
        KPrintf("BusFindDriver %s failed\n", RS485_UART_DRV);
        return;
    }
    /* find the serial device pointer */
    dev = BusFindDevice(bus, RS485_UART_DEV);
    if (NONE == dev)
    {
        KPrintf("BusFindDevice %s failed\n", RS485_UART_DEV);
        return;
    }
    struct BusBlockWriteParam write_param;
    write_param.pos = 0;
    write_param.buffer = (void *)test_str;
    write_param.size = sizeof(test_str) - 1;
    
    // BusDevWriteData(dev, &write_param);
    // BusDevClose(dev);
    
    return;
}

PRIV_SHELL_CMD_FUNCTION(Test485, a 485 test sample, PRIV_SHELL_CMD_MAIN_ATTR);
```


1.make BOARD=ch32v307vct6 menuconfig中配置UART相关参数 ch32v307vct6 feature  ---> Enable UART5  --->

2.编译后将其烧录至开发板上，连接485转USB硬件至主机，主机使用串口工具打开执行test_rs485命令，运行485收发测试。


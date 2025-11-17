# PikaPython 移植到 XiUOS

本文档描述了如何将 PikaPython 嵌入式 Python 解释器移植到 XiUOS 操作系统。

## 概述

PikaPython 是一个专为嵌入式系统设计的轻量级 Python 解释器，具有以下特点：
- 内存占用小（ARM 内核至少 64KB Flash + 8KB RAM）
- 支持标准 Python 语法
- 可扩展的硬件抽象层
- 支持预编译脚本

## 移植架构

```
APP_Framework/lib/pikapython/
├── pikascript-core/        # PikaPython 核心解释器
│   ├── PikaVM.c           # 虚拟机
│   ├── PikaObj.c          # 对象系统
│   ├── PikaParser.c       # 解析器
│   ├── PikaCompiler.c     # 编译器
│   ├── PikaPlatform.c     # 默认平台实现（使用标准C库）
│   └── ...                # 其他核心文件
├── pikascript-api/         # 预编译的Python API
│   ├── pikaScript.c       # 主脚本执行器
│   ├── pikaScript.h       # 主脚本头文件
│   ├── __pikaBinding.c    # Python绑定
│   └── __asset_pikaModules_py_a.c # 预编译的Python模块
├── pikascript-lib/         # PikaPython 标准库
│   └── PikaStdLib/        # 标准库实现
├── main.py                # Python测试脚本（已预编译）
├── Kconfig                # 配置选项
├── Makefile              # 构建文件
└── README.md             # 说明文档
```

## 为什么不需要额外的平台适配层？

PikaPython**不依赖操作系统和文件系统**，只依赖标准C库，并且：

### 1. **内置默认平台实现**
- `pikascript-core/PikaPlatform.c` 提供了完整的默认平台实现
- 直接使用标准C库函数（malloc、printf、strcpy等）
- 无需额外的平台适配代码

### 2. **预编译的优势**
- Python脚本预编译为C代码
- 无需运行时解释器
- 直接调用预编译的C函数

### 3. **最小化依赖**
- 只依赖标准C库
- 无需操作系统特定代码
- 跨平台兼容

## 配置选项

### 基本配置
- `CONFIG_LIB_USING_PIKAPYTHON`: 启用 PikaPython 支持

**注意**: PikaPython 使用其内部配置系统，无需额外的配置选项。

## 使用方法

### 1. 配置编译选项

在项目配置中启用 PikaPython：
```bash
# 使用 menuconfig 或直接编辑 .config 文件
CONFIG_LIB_USING_PIKAPYTHON=y
```

### 2. 在应用程序中使用

```c
#ifdef CONFIG_LIB_USING_PIKAPYTHON
#include "pikaScript.h"
#endif

int main(void)
{
#ifdef CONFIG_LIB_USING_PIKAPYTHON
    // 初始化并运行 PikaPython（自动执行预编译的main.py）
    PikaObj* pikaMain = pikaPythonInit();
    
    if (pikaMain != NULL) {
        printf("PikaPython test completed successfully!\n");
    } else {
        printf("PikaPython initialization failed!\n");
    }
#else
    printf("PikaPython is disabled in configuration.\n");
#endif
    
    return 0;
}
```

**注意**: `CONFIG_LIB_USING_PIKAPYTHON` 宏是通过编译器命令行传递的，无需包含额外的头文件。

### 3. 测试脚本

运行测试：
```bash
# 编译并运行测试程序
make
./test_pikapython
```

## 移植步骤详解

### 第一步：获取 PikaPython 源码

1. 从 [PikaPython GitHub](https://github.com/pikastech/pikapython) 下载源码
2. 将 `pikascript-core` 和 `PikaStdLib` 复制到 `src/` 目录

### 第二步：实现平台适配

1. **内存管理**：实现 `pika_malloc()`, `pika_free()`, `pika_realloc()`
2. **字符串操作**：实现 `pika_strdup()`, `pika_strcmp()` 等
3. **打印函数**：实现 `pika_printf()`, `pika_sprintf()` 等
4. **时间函数**：实现 `pika_get_tick()`, `pika_delay_ms()`

### 第三步：硬件抽象层适配

1. **GPIO 操作**：`pika_hal_gpio_init()`, `pika_hal_gpio_write()`, `pika_hal_gpio_read()`
2. **UART 操作**：`pika_hal_uart_init()`, `pika_hal_uart_write()`, `pika_hal_uart_read()`
3. **I2C 操作**：`pika_hal_i2c_init()`, `pika_hal_i2c_write()`, `pika_hal_i2c_read()`
4. **SPI 操作**：`pika_hal_spi_init()`, `pika_hal_spi_write()`, `pika_hal_spi_read()`
5. **ADC 操作**：`pika_hal_adc_init()`, `pika_hal_adc_read()`
6. **PWM 操作**：`pika_hal_pwm_init()`, `pika_hal_pwm_set_duty()`

### 第四步：集成到构建系统

1. 更新 `APP_Framework/lib/Kconfig` 包含 PikaPython 配置
2. 更新 `APP_Framework/lib/Makefile` 包含 PikaPython 源码
3. 创建应用程序测试代码

## 注意事项

### 内存要求
- **ARM 内核**：至少 64KB Flash + 8KB RAM
- **RISC-V 内核**：至少 128KB Flash + 8KB RAM
- 确保编译器支持 C99 标准

### 依赖关系
- 需要完整的标准 C 库（libc）支持
- 需要基本的输入输出功能
- 可选的文件系统支持

### 调试建议
1. 启用 `CONFIG_PIKAPYTHON_ENABLE_DEBUG` 查看调试信息
2. 使用串口输出查看 Python 脚本执行结果
3. 检查内存使用情况，避免栈溢出

## 扩展功能

### 文件系统支持
如果启用文件系统支持，可以：
- 从文件加载 Python 脚本
- 保存 Python 脚本执行结果
- 支持模块化 Python 代码

### 硬件访问
如果启用硬件访问，可以：
- 在 Python 中控制 GPIO
- 使用 UART、I2C、SPI 等外设
- 读取 ADC 值、控制 PWM 输出

## 故障排除

### 常见问题
1. **内存不足**：检查系统可用内存
2. **栈溢出**：检查系统栈配置
3. **编译错误**：检查 C99 标准支持
4. **运行时错误**：启用调试输出查看详细信息

### 调试技巧
1. 使用 `pika_printf()` 输出调试信息
2. 检查内存分配是否成功
3. 验证平台接口实现是否正确
4. 测试基本的 Python 语法支持

## 参考资料

- [PikaPython 官方文档](https://pikapython.com/)
- [PikaPython GitHub 仓库](https://github.com/pikastech/pikapython)
- [XiUOS 官方文档](https://www.xiuos.com/)

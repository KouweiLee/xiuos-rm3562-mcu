# 构建矽璓工业物联操作系统：使用ARM架构的rk3562系列处理器

# Rockchip RK3562系列
MCU:RK3562


## 1. 简介

| 硬件 | 描述 |
| -- | -- |
|芯片型号| RK3562 |
|架构| cortex-m0 |

## 2. 开发环境搭建

### 推荐使用：

**操作系统：** ubuntu

### 依赖包安装：

```
$ sudo apt install build-essential pkg-config  git
$ sudo apt install gcc make libncurses5-dev openssl libssl-dev bison flex libelf-dev autoconf libtool gperf libc6-dev
```

**XiUOS操作系统源码下载：** XiUOS [https://www.gitlink.org.cn/xuos/xiuos](https://www.gitlink.org.cn/xuos/xiuos)

新建一个空文件夹并进入文件夹中，并下载源码，具体命令如下：

```c
mkdir test  &&  cd test
git clone https://github.com/KouweiLee/xiuos-rm3562-mcu.git
```

### 裁减配置工具的下载

裁减配置工具：

**工具地址：** kconfig-frontends [https://www.gitlink.org.cn/xuos/kconfig-frontends](https://www.gitlink.org.cn/xuos/kconfig-frontends)，下载与安装的具体命令如下：

```c
mkdir kfrontends  && cd kfrontends
git clone https://gitlink.org.cn/xuos/kconfig-frontends.git
```

下载源码后按以下步骤执行软件安装：

```c
cd kconfig-frontends
./xs_build.sh
```

### 编译工具链：

ARM： arm-none-eabi(`gcc version 6.3.1`)，默认安装到Ubuntu的/usr/bin/arm-none-eabi-，使用如下命令行下载和安装。

```shell
$ sudo apt install gcc-arm-none-eabi
```

## 编译说明

### 编辑环境：`Ubuntu`

### 编译工具链：`arm-none-eabi-gcc`


编译步骤：

1.在命令终端中执行以下命令，生成配置文件

```c
cd ./Ubiquitous/XiZi_IIoT_Macro/
make BOARD=rk3562-mcu distclean
make BOARD=rk3562-mcu menuconfig
```

2.在menuconfig界面配置需要关闭和开启的功能，按回车键进入下级菜单，按Y键选中需要开启的功能，按N键选中需要关闭的功能，配置结束后保存并退出（本例旨在演示简单的输出例程，所以没有需要配置的选项，双击快捷键ESC退出配置）

退出时选择`yes`保存上面所配置的内容。

3.继续执行以下命令，进行编译

```
make BOARD=rk3562-mcu
cd board/rk3562-mcu && ./mkimage.sh
```

注意：最好查看mkimage.sh，根据实际情况修改image/amp.its文件。

4.如果编译正确无误，会产生amp.img文件。

### 3.2 运行结果

将amp.img文件烧录到RK3562开发板上的M核即可。

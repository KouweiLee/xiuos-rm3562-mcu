/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */

#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

// _exit: 用于退出程序
void _exit(int status)
{
    while (1) { }
}

// _sbrk: 用于增加程序的数据空间
void* _sbrk(ptrdiff_t incr)
{
    extern char end; /* Defined by the linker */
    static char* heap_end;
    char* prev_heap_end;

    if (heap_end == 0) {
        heap_end = &end;
    }
    prev_heap_end = heap_end;

    // 在这里，你应该添加一些检查来确保堆不会与栈或其他内存区域冲突
    // 例如，检查 incr 是否会导致堆超出预定的内存区域

    heap_end += incr;
    return (void*)prev_heap_end;
}

// _write: 用于将数据写入文件描述符
ssize_t _write(int file, const void* ptr, size_t len)
{
    // 在这里，你需要实现将数据写入文件描述符的逻辑
    // 如果你的系统不支持文件系统，你可以将数据发送到串口或其他输出

    return len; // 假设所有数据都被写入
}

// _close: 用于关闭文件描述符
int _close(int file)
{
    return -1; // 表示失败，因为没有实际关闭文件的功能
}

// _fstat: 用于获取文件状态
int _fstat(int file, struct stat* st)
{
    return 0; // 表示成功
}

// _isatty: 检查文件描述符是否指向TTY设备
int _isatty(int file)
{
    return 1; // 表示是TTY设备
}

// _lseek: 用于重新定位文件读/写的位置
off_t _lseek(int file, off_t offset, int whence)
{
    return -1; // 表示失败，因为不支持文件定位
}

// _read: 用于从文件描述符读取数据
ssize_t _read(int file, void* ptr, size_t len)
{
    return 0; // 表示没有数据被读取
}

// _kill: 发送信号给进程
int _kill(int pid, int sig)
{
    return -1; // 表示失败，因为不支持信号
}

// _getpid: 获取进程ID
int _getpid()
{
    return 1; // 返回假设的进程ID
}

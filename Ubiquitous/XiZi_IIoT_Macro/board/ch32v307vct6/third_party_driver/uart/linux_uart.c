#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

int main()
{
    int fd;
    struct termios tty;
    char buffer[256];

    // 打开串口设备
    fd = open("/dev/ttySC3", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("打开串口失败");
        return -1;
    }

    // 获取当前串口配置
    if (tcgetattr(fd, &tty) != 0) {
        perror("获取串口配置失败");
        close(fd);
        return -1;
    }

    // 设置波特率为9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // 8位数据位，无校验位，1位停止位
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // 非规范模式，禁止软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁止输出回车换行和换行符的映射
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // 设置新的串口配置
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("设置串口配置失败");
        close(fd);
        return -1;
    }

    // 获取待读取数据的字节数
    int bytes_available;
    printf("读取中\n");
    while (1) {
        int res = ioctl(fd, TIOCINQ, &bytes_available);

        if (res != -1) {
            if (bytes_available > 0) {
                // 读取串口数据
                printf("read\n");
                int bytes_read = read(fd, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    // 在这里处理读取到的数据
                    printf("读取到的数据: %s\n", buffer);
                }
            }
        }
    }

    // 关闭串口设备
    close(fd);

    return 0;
}
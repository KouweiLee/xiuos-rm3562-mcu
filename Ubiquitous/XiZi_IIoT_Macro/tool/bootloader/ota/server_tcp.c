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

/**
* @file:    ota_server.c
* @brief:   a application ota task of system running in Linux
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2023/5/26
*
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <libgen.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/time.h>
#include <assert.h>
#include <netdb.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ifaddrs.h>

#define STARTFLAG  0x1A2B   //数据帧开始信号标记
#define DATAFLAG   0x3C4D   //数据帧数据信号标记
#define ENDTFLAG   0x5E6F   //数据帧结束信号标记
#define PORT         7777   //socket端口号
#define SIZE          100   //socket链接限制为100
#define LENGTH        1024  //每帧数据的数据包长度
#define BIN_PATH     "/home/aep05/wgz/XiZi-xidatong-arm32-app.bin"  //bin包的路径

typedef struct
{
    uint16_t frame_flag;          // frame start flag 2 Bytes
    uint16_t dev_sid;             // device software version
    uint32_t total_len;           // send data total length caculated from each frame_len 
} ota_header_t;

typedef struct
{
    uint32_t frame_id;               // Current frame id
    uint8_t  frame_data[LENGTH];     // Current frame data
    uint16_t frame_len;              // Current frame data length
    uint16_t crc;                    // Current frame data crc
} ota_frame_t;

typedef struct
{
    ota_header_t header;
    ota_frame_t frame;
} ota_data;


static int serverfd;             // 服务器socket
static int clientfd[SIZE] = {0}; // 客户端的socketfd,100个元素，clientfd[0]~clientfd[99]


/*******************************************************************************
* 函 数 名: calculate_crc16
* 功能描述: 计算给定长度的数据的crc16的值,用于OTA传输过程中数据帧的校验
* 形    参: data:数据buffer
            len:表示需要计算CRC16的数据长度
* 返 回 值: 计算得到的CRC16值
*******************************************************************************/
static uint16_t calculate_crc16(uint8_t * data, uint32_t len)
{
    uint16_t reg_crc=0xFFFF;
    while(len--)
    {
        reg_crc ^= *data++;
        for(int j=0;j<8;j++) 
        {
            if(reg_crc & 0x01)
                reg_crc=reg_crc >>1 ^ 0xA001;
            else
                reg_crc=reg_crc >>1;
        }
    }
    printf("crc = [0x%x]\n",reg_crc);
    return reg_crc;
}


/*******************************************************************************
* 函 数 名: sockt_init
* 功能描述: 用于在TCP Server上创建socet监听
* 形    参: data:数据buffer
            len:表示需要计算CRC16的数据长度
* 返 回 值: 计算得到的CRC16值
*******************************************************************************/
void sockt_init(void)
{
    struct sockaddr_in addr, *sa;//存储套接字的信息
    struct ifaddrs *ifap, *ifa;
    char *ipaddr;

    serverfd = socket(AF_INET,SOCK_STREAM,0);

    if(serverfd == -1)
    {
        perror("Failed to create socket");
        exit(-1);
    }

    //为套接字设置ip协议 设置端口号并自动获取本机ip转化为网络ip
    addr.sin_family = AF_INET;//地址族
    addr.sin_port = htons(PORT);//设置server端端口号,随便设置,当sin_port = 0时，系统随机选择一个未被使用的端口号
    addr.sin_addr.s_addr = htons(INADDR_ANY);//当sin_addr=INADDR_ANY时表示从本机的任一网卡接收数据

    /*显示当前TCP server的*/
    getifaddrs(&ifap);
    for(ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) 
    {
        if(ifa->ifa_addr->sa_family == AF_INET) 
        {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            ipaddr = inet_ntoa(sa->sin_addr);
            printf("Interface:%-16s Address:%-16s\n", ifa->ifa_name, ipaddr);
        }
    }
    freeifaddrs(ifap);
  
    //绑定套接字
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if(setsockopt(serverfd, SOL_SOCKET, SO_REUSEADDR, &timeout, sizeof(struct timeval)) < 0)
    {
        perror("Failed to set setsock option");
        exit(-1);
    }

    if(bind(serverfd,(struct sockaddr*)&addr,sizeof(addr)) == -1)
    {
       perror("Failed to bind socket port");
       exit(-1);
    }

    //监听最大连接数
    if(listen(serverfd,SIZE) == -1)
    {
        perror("Failed to set socket listening");
        exit(-1);
    }
}


/*******************************************************************************
* 函 数 名: ota_start_signal
* 功能描述: 发送开始信号,等待接收端回应ready
* 形    参: fd:监听的客户端连接的fd
* 返 回 值: 0:成功,-1:失败
*******************************************************************************/
void ota_start_signal(int fd)
{
    ota_data data;
    struct stat st;
    uint8_t buf[32];
    int file_size = 0, file_frame_cnt = 0, length = 0;

    if (access(BIN_PATH, F_OK) == 0) 
    {
        printf("%s exists.\n", basename(BIN_PATH));
    } 
    else 
    {
        printf("%s does not exist,please cheack!\n", BIN_PATH);
        exit(-1);
    }

    //获取文件大小(以字节为单位)
    if(stat(BIN_PATH, &st) == 0)
    {
        file_size = st.st_size;
        file_frame_cnt = ((file_size%LENGTH) != 0)? (file_size/LENGTH + 1):(file_size/LENGTH);
        printf("%s size is %d bytes,frame count is %d!\n",basename(BIN_PATH), file_size, file_frame_cnt);
    } 
    else 
    {
        printf("Failed to get file size\n");
        exit(-1);
    }

    while(1)
    {
        memset(&data, 0x0, sizeof(ota_data));
        data.header.frame_flag = STARTFLAG;
        //发送起始帧时把bin文件的大小一并发送出去
        data.header.total_len = file_size;

        while(send(fd, &data, sizeof(data), MSG_NOSIGNAL) <= 0);
        printf("send start signal to client %d.\n", fd);
        
        memset(buf, 0, sizeof(buf));
        length = recv(fd, buf, sizeof(buf), 0);
        if(length == 0)
        {
            printf("The current socket %d is disconnected,please check it!\n",fd);
            close(fd);
            pthread_exit(0);
        }
        else if(length > 0 && (0 == strncmp(buf, "ready", length)))
        {
            printf("recv buf %s length %d from client %d.\n", buf, length, fd);
            break;
        }
        else 
        {
            continue;
        }

    }
    
}


/*******************************************************************************
* 函 数 名: ota_file_send
* 功能描述: 用于在TCP Server发送bin文件
* 形    参: fd:监听的客户端连接的fd
* 返 回 值: 发送成功返回0，失败返回-1
*******************************************************************************/
int ota_file_send(int fd)
{
    unsigned char buf[32] = { 0 };
    ota_data data;
    FILE *file_fd;
    int length = 0;
    int try_times;
    int recv_end_times = 3;
    int ret = 0;
    int frame_cnt = 0;
    int file_length = 0;
    char * file_buf = NULL;

    file_fd = fopen(BIN_PATH, "r");
    if(NULL == file_fd)
    {
        printf("open file failed.\n");
        fclose(file_fd);
        return -1;
    }
    fseek(file_fd, 0, SEEK_SET);
    printf("start send bin file to client %d.\n", fd);


    while(!feof(file_fd))
    {
        memset(&data, 0, sizeof(data));

        data.header.frame_flag = DATAFLAG;
        length = fread(data.frame.frame_data, 1, LENGTH, file_fd);
        if(length == LENGTH) 
        {
            printf("read %d bytes\n",length);
            data.frame.frame_id = frame_cnt;
            data.frame.frame_len = length;
            data.frame.crc = calculate_crc16(data.frame.frame_data, length);
            file_length += length;
        }
        else if(length > 0 && length < LENGTH)
        {
             if(ferror(file_fd)) 
             {
                printf("read %s file error!\n", basename(BIN_PATH));
                ret = -1;
                break;
             }
             else
             {
                printf("read %d bytes\n",length);
                data.frame.frame_id = frame_cnt;
                data.frame.frame_len = length;
                data.frame.crc = calculate_crc16(data.frame.frame_data, length);
                file_length += length;
             }
        }
        //fread返回值为0,此时是个空包,不需要再发送了否则是冗余数据
        else
        {
            printf("read %s file done!\n", basename(BIN_PATH));
            break;
        }

send_again:
        printf("send frame[%d] to client %d.\n", frame_cnt, fd);
        length = send(fd, &data, sizeof(data), MSG_NOSIGNAL);
        if(length < 0)
        {
            printf("send frame[%d] to client %d failed,send again\n", frame_cnt, fd);
            goto send_again;
        }
        
recv_again:
        memset(buf, 0, sizeof(buf));
        length = recv(fd, buf, sizeof(buf), 0);
        if(length == 0)
        {
            printf("current socket %d is disconnected,please check it!\n", fd);
            ret = -1;
            close(fd);
            pthread_exit(0);
            break;
        }
        else if(length < 0 )
        {
            printf("send frame[%d] to client %d waiting for ok timeout,receive again.\n", frame_cnt, fd);
            goto recv_again;
        }
        else if(0 == strncmp(buf, "ok", length))
        {
            printf("receive buf[%s] length %d from client %d.\n", buf, length, fd);
            try_times = 5;
            printf("send to client %d frame[%d] data send done.\n",fd, frame_cnt);
            frame_cnt++;
            continue;
        }

        //接收到的回复不是ok,说明刚发的包有问题,需要再发一次 
        else
        {
            if(try_times > 0)
            {
                try_times--;
                goto send_again;
            } 
            else
            {
                printf("send to client %d frame[%d] 5 times failed.\n",fd, frame_cnt);
                ret = -1;
                break;
            }
        }
    }

    /* finally,crc check total bin file.*/
    if(ret == 0)
    {
        printf("total send file length %d bytes, %d frames to client %d.\n", file_length, frame_cnt, fd);
        printf("now crc check total bin file.\n");
        file_buf = malloc(file_length);
        memset(file_buf, 0, file_length);
        memset(&data, 0, sizeof(data));

        data.header.frame_flag = ENDTFLAG;

        file_fd = fopen(BIN_PATH, "r");
        if(NULL == file_fd)
        {
            printf("open file failed.\n");
            return -1;
        }
        fseek(file_fd, 0, SEEK_SET);
        length = fread(file_buf,1, file_length, file_fd);
        printf("read file length = %d\n",length);
        if(length > 0) 
        {
            data.frame.frame_id = frame_cnt;
            data.header.total_len = file_length;
            data.frame.crc = calculate_crc16(file_buf, length);
        }

send_end_signal:
        printf("send ota end signal to client %d.\n", fd);
        length = send(fd, &data, sizeof(data), MSG_NOSIGNAL);
        if(length < 0)
        {
            printf("send to client %d ota end signal faile,send again\n",fd);
            goto send_end_signal;
        }

recv_end_signal:
        memset(buf, 0, sizeof(buf));
        length = recv(fd, buf, sizeof(buf), 0);
        if(length == 0)
        {
            printf("current socket %d is disconnected,please check it!\n",fd);
            ret = -1;
        	free(file_buf);
    		fclose(file_fd);
            close(fd);
            pthread_exit(0);

        }
        if(length < 0 || (0 != strncmp(buf, "ok", length)))
        {
            recv_end_times--;
            printf("from client %d end signal waiting for ok timeout,receive again.\n", fd);
            if(recv_end_times > 0)
            {
                goto recv_end_signal;
            }
            else
            {
                printf("client %d error end !!!\n", fd);
                ret = -1;
            }
        }

        free(file_buf);
    }

    fclose(file_fd);
    return ret;
}


/*******************************************************************************
* 函 数 名: server_thread
* 功能描述: TCP Server的服务线程入口函数
* 形    参: p:入口函数的参数
* 返 回 值: 无
*******************************************************************************/
void* server_thread(void* p)
{
    int fd = *(int*)p;
    unsigned char buf[32] = { 0 };
    ota_data data;

    printf("pthread = %d\n",fd);
    sleep(3);
    while(1)
    {
        /* if ota failed then restart the ota process */
        ota_start_signal(fd);
        sleep(5);
        if(0 == ota_file_send(fd)) 
        {
            printf("ota file send to client %d successful.\n", fd);
            break;
        }
    }
    printf("exit fd = %d\n",fd);
    close(fd);
    pthread_exit(0);
}


/*******************************************************************************
* 函 数 名: server
* 功能描述: TCP Server的服务函数
* 形    参: 无
* 返 回 值: 无
*******************************************************************************/
void server(void)
{
    int i = 0;
    printf("ota Server startup\n");
    while(1)
    {
        struct sockaddr_in fromaddr;
        socklen_t len = sizeof(fromaddr);
        int fd = accept(serverfd,(struct sockaddr*)&fromaddr,&len);//调用accept进入堵塞状态，等待客户端的连接

        if(fd == -1)
        {
            printf("The client connection is wrong...\n");
            continue;
        }

        for(i = 0;i < SIZE;i++)
        {
            if(clientfd[i] == 0)
            {
                //记录客户端的socket
                clientfd[i] = fd;

                //有客户端连接之后，启动线程给此客户服务
                pthread_t tid;
                pthread_create(&tid,0,server_thread,&fd);
                break;
            }

            if(SIZE == i)
            {
                //发送给客户端聊天室满了
                char* str = "Devices full";
                printf("%s", str);
                send(fd,str,strlen(str),0);
                close(fd);
            }
        }
    }
}

int main(void)
{
    sockt_init();
    server();
}


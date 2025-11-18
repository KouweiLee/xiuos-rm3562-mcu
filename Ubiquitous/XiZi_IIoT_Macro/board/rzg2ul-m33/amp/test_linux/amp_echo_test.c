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

#include "../include/amp.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER

#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "sys/mman.h"
#include "sys/ioctl.h"
#include "sys/times.h"

/**
 * 参数1：需要发送的次数【数字】
 * 参数2：需要发送的数据【字符串】
*/
int main(int argc, char *argv[])
{
    clock_t ticks_start = times(NULL);

	int send_count = atoi(argv[1]); /* 获取需要发送的次数 */
	char *data = (char*)argv[2]; /* 需要发送的数据 */
	int data_size = strlen(data); /* 需要发送数据的大小 */

	INFOS("info : send [data = %s, times = %d, data_size = %d]\n", data, send_count, data_size);

	struct Client amp_client = { 0 };

    /* 初始化客户端：连接Linux核心 */
    if (client_ops.client_init(&amp_client, CORE_XIUOS_ID) != 0)
    {
        INFOS("error : client init [target_channel = %u]\n",CORE_XIUOS_ID);
        return;
    }
    LOGINFO("info : client init [target_channel = %u]\n",CORE_XIUOS_ID);

    /* 从通道中拿取一个发送给回显服务的长期消息缓冲区 */
    struct Msg *msg = client_ops.empty_msg_get(&amp_client,XIUOS_SERVICE_ECHO_ID);
    if (msg == NULL)
    {
        INFOS("error : empty msg get [service_id = %u]\n",XIUOS_SERVICE_ECHO_ID);
        return;
    }
    LOGINFO("info : empty msg get [service_id = %u]\n",XIUOS_SERVICE_ECHO_ID);

    /* 开始循环发送测试 */
    for (int i = 0; i < send_count; i++) /* 准备多次发送 */
    {
        /* 重置该消息体 */
        msg_ops.msg_reset(msg);

        /* 数据填充 */
#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE
        int j = 0;
        for (j = 0; j < data_size; j++)
        {
            msg->data[j] = data[j];
        }
        msg->data[data_size] = '\0';
        msg->data_size = data_size + 1;

        /* 消息发送 */
        if (client_ops.msg_send_and_notify(&amp_client,msg) != 0)
        {
            INFOS("error : msg send [idx = %d, data_size = %u]\n",i, data_size + 1);
            return;
        }
        LOGINFO("info : msg send [idx = %d, data_size = %u]\n",i, data_size + 1);

        /* 等待消息响应 */
        while (client_ops.msg_poll(msg) != 0)
        {
            ;
        }
        LOGINFO("info : msg result [idx = %d, service_id = %u, result = %u]\n",i, msg->service_id,msg->flag.service_result);
#endif /* PROTOCOL_CHOICE == PROTOCOL_PRIVATE */

#if PROTOCOL_CHOICE == PROTOCOL_AMP
        /* 共享内存分配 */
        char* shm_data = (char*)client_ops.shm_malloc(&amp_client,data_size + 1,MALLOC_TYPE_V);
        if (shm_data == NULL)
        {
            INFOS("error : shm malloc [idx = %d, size = %u, type = %u, ptr = NULL]\n",i, data_size + 1,MALLOC_TYPE_V);
            return;
        }
        LOGINFO("info : shm malloc [idx = %d, size = %u, type = %u, ptr = %p]\n",i, data_size + 1,MALLOC_TYPE_V,shm_data);

        /* 数据填充 */
        int j = 0;
        for (j = 0; j < data_size; j++)
        {
            shm_data[j] = data[j];
        }
        shm_data[data_size] = '\0';

        /* 消息构建 */
        msg->offset = client_ops.shm_addr_to_offset(shm_data);
        msg->length = data_size + 1;
        LOGINFO("info : msg fill [idx = %d, offset = %u, length = %u]\n",i, msg->offset,msg->length);

        /* 消息发送 */
        if (client_ops.msg_send_and_notify(&amp_client,msg) != 0)
        {
            INFOS("error : msg send [idx = %d, offset = %u, length = %u]\n",i, msg->offset,msg->length);
            return;
        }
        LOGINFO("info : msg send [idx = %d, offset = %u, length = %u]\n",i, msg->offset,msg->length);

        /* 等待消息响应 */
        while (client_ops.msg_poll(msg) != 0)
        {
            ;
        }
        LOGINFO("info : msg result [idx = %d, service_id = %u, result = %u]\n",i, msg->service_id,msg->flag.service_result);

        /* 释放临时共享内存 */
        client_ops.shm_free(&amp_client,shm_data);
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
    }
    if (client_ops.empty_msg_put(&amp_client, msg) != 0)
    {
        LOGINFO("error : return empty msg\n");
        return;
    }
    LOGINFO("info : return empty msg\n");

    if (client_ops.client_destory(&amp_client) != 0)
    {
        INFOS("error : client destory\n");
        return ;
    }
    LOGINFO("info : client destory\n");
    INFOS("info : test over\n");

    clock_t ticks_end = times(NULL);

    INFOS("ticks = %u\n",ticks_end - ticks_start);
    
	return 0;
}

#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER */
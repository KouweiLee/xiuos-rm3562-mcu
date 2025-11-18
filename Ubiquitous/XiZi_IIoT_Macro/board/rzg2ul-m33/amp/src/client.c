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

#include "../include/client.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

#define CHECK_PTR_NOT_NULL(ptr, ret)  \
    do                                \
    {                                 \
        if (ptr == NULL)              \
        {                             \
            INFOS("param is null\n"); \
            return ret;               \
        }                             \
    } while (0);

static int32_t client_init(struct Client* raw_client,uint32_t remote_core_id)
{
    CHECK_PTR_NOT_NULL(raw_client,-1); /* 参数校验 */

    if (raw_client->init_mark == INIT_MARK_INITIALIZED)
    {
        LOGINFO("client_init_info: client init before\n");
        return 0;
    }

    if (raw_client->init_mark == INIT_MARK_DESTORYED)
    {
        INFOS("client_init_error: client is null or broken\n");
        return -1;
    }

    if (channel_ops.channels_init() != 0) /* 初始化所有的通道 */
    {
        raw_client->init_mark = INIT_MARK_DESTORYED;
        INFOS("client_init_error: channels init fail\n");
        return -1;
    }
    LOGINFO("client_init_info: channels init success\n");

    /* 获取目标通道 */
    struct Channel* target_channel = channel_ops.target_channel_get(remote_core_id);

    if (target_channel == NULL) /* 没有获取到目标核心的通道 */
    {
        raw_client->init_mark = INIT_MARK_DESTORYED;
        INFOS("client_init_error: get target channel fail = %u\n",remote_core_id);
        return -1;
    }
    LOGINFO("client_init_info: get target channel success = %u\n",remote_core_id);

    while (channel_ops.channel_is_ready(target_channel) != 0) /* 检查目标通道是否已经被启用 */
    {
        ;
    }
    
    LOGINFO("client_init_info: target channel [%u] is ready\n",remote_core_id);
    
    raw_client->remote_channel = target_channel;
    raw_client->msg_cnt = 0U;
    raw_client->shm_cnt = 0U;

#if PROTOCOL_CHOICE == PROTOCOL_AMP /* 如果使用的共享内存，需要对共享内存进行初始化 */
    if (shm_ops.shm_init() != 0)
    {
        raw_client->init_mark = INIT_MARK_DESTORYED;
        INFOS("client_init_error: init shm fail\n");
        return -1;
    }
    
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
    raw_client->init_mark = INIT_MARK_INITIALIZED;

    return 0;
}

static int32_t client_destory(struct Client* client)
{
    CHECK_PTR_NOT_NULL(client,-1);

    if (client->init_mark == INIT_MARK_INITIALIZED)
    {
        if (client->msg_cnt > 0)
        {
            INFOS("client_destory: hold msg\n");
            return -1;
        }
#if PROTOCOL_CHOICE == PROTOCOL_AMP
        if (client->shm_cnt > 0)
        {
            INFOS("client_destory: hold shm\n");
            return -1;
        }
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
        client->init_mark = INIT_MARK_RAW_STATE;
    }

    return 0;
}

static struct Msg* client_empty_msg_get(struct Client* client,uint32_t remote_service_id)
{
    CHECK_PTR_NOT_NULL(client,NULL);

    struct Msg* msg = channel_ops.empty_msg_get(client->remote_channel);

    if (msg != NULL)
    {
        msg->service_id = remote_service_id;
        client->msg_cnt ++;
    }
    
    return msg;
}

static int32_t client_empty_msg_put(struct Client* client, struct Msg* empty_msg)
{
    CHECK_PTR_NOT_NULL(client,-1);
    CHECK_PTR_NOT_NULL(empty_msg, -1);

    if (channel_ops.empty_msg_put(client->remote_channel,empty_msg) == 0)
    {
        client->msg_cnt --;
        return 0;
    }
    
    return -1;
}

static int32_t client_msg_send(struct Client* client, struct Msg* full_msg)
{
    CHECK_PTR_NOT_NULL(client,-1);
    CHECK_PTR_NOT_NULL(full_msg, -1);

    return channel_ops.msg_send(client->remote_channel,full_msg);
}

static int32_t client_msg_send_and_notify(struct Client* client, struct Msg* full_msg)
{
    CHECK_PTR_NOT_NULL(client,-1);
    CHECK_PTR_NOT_NULL(full_msg, -1);

    return channel_ops.msg_send_and_notify(client->remote_channel,full_msg);
}

static int32_t client_msg_notify(struct Client* client)
{
    CHECK_PTR_NOT_NULL(client,-1);

    return channel_ops.channel_notify(client->remote_channel);
}

static int32_t client_msg_poll(struct Msg* aim_msg)
{
    CHECK_PTR_NOT_NULL(aim_msg,-1);

    return msg_ops.msg_is_dealt(aim_msg);
}

#if PROTOCOL_CHOICE == PROTOCOL_AMP
static void* client_shm_malloc(struct Client* client, uint32_t size,enum MallocType type)
{
    void* result = NULL;

    result = shm_ops.shm_malloc(size,type);

    if (result != NULL)
    {
        client->shm_cnt ++;
    }
    
    return result;
}

static void client_shm_free(struct Client* client, void* ptr)
{
    if (ptr != NULL)
    {
        client->shm_cnt --;
    }
    
    shm_ops.shm_free(ptr);
}

static uint32_t client_shm_addr_to_offset(void* ptr)
{
    return shm_ops.addr_to_offset(ptr);
}

static void* client_shm_offset_to_addr(uint32_t offset)
{
    return shm_ops.offset_to_addr(offset);
}

#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */

struct ClientOps client_ops = 
{
    .client_init = client_init,
    .client_destory = client_destory,
    .empty_msg_get = client_empty_msg_get,
    .empty_msg_put = client_empty_msg_put,
    .msg_send = client_msg_send,
    .msg_send_and_notify = client_msg_send_and_notify,
    .msg_notify = client_msg_notify,
    .msg_poll = client_msg_poll,
#if PROTOCOL_CHOICE == PROTOCOL_AMP
    .shm_malloc = client_shm_malloc,
    .shm_free = client_shm_free,
    .shm_addr_to_offset = client_shm_addr_to_offset,
    .shm_offset_to_addr = client_shm_offset_to_addr
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
};
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_USER || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
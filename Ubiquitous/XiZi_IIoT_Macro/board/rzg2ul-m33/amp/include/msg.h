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

#include "config/config.h"

#ifndef __AMP_MSG_H__
#define __AMP_MSG_H__

enum MsgDealState /* 消息是否被处理 */
{
    MSG_DEAL_STATE_NO = 0, /* 未被处理 */
    MSG_DEAL_STATE_YES = 1 /* 已被处理 */
};

enum MsgServiceResult /* 消息是否被正确处理 */
{
    MSG_SERVICE_RET_NONE = 0,     /* 消息还未被处理 */
    MSG_SERVICE_RET_SUCCESS = 1,  /* 服务正确响应 */
    MSG_SERVICE_RET_FAIL = 2,     /* 服务未曾正确服务，或参数错误等 */
    MSG_SERVICE_RET_NOT_EXITS = 3 /* 请求的服务不存在 */
};

struct Msg
{
    /* 消息标记：用于标记消息状态信息，共 16 位可使用位域进行扩展 */
    struct MsgFlag 
    {
        uint16_t deal_state : 1;     /* 消息是否被处理 */
        uint16_t service_result : 2; /* 消息对应的服务是否被正确服务 */
    } flag;
    
    uint16_t service_id; /* 请求的服务端服务ID */

#if PROTOCOL_CHOICE == PROTOCOL_AMP /* AMP协议 */
    uint32_t offset; /* 共享内存起始偏移量 */
    uint32_t length; /* 共享内存长度 */
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */

#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE /* 私有协议 */
    uint32_t data_size; /* 本条消息携带的数据量 */
    uint8_t data[MSG_PAYLOAD_SIZE];    /* 数据本身：配置消息大小时最好四字节对齐 */
#endif /* PROTOCOL_CHOICE == PROTOCOL_PRIVATE */
}__attribute__((aligned(MEMORY_ALIGN_SIZE)));

/* 消息支持的操作 */
struct MsgOps
{
    /* 将消息中的相关信息清除，重置 */
    void (*msg_reset)(struct Msg *msg);
    
    /* 查看消息是否已经被处理了 */
    int32_t (*msg_is_dealt)(struct Msg *msg);
};

extern struct MsgOps msg_ops;
#endif /* __AMP_COM_MSG_H__ */
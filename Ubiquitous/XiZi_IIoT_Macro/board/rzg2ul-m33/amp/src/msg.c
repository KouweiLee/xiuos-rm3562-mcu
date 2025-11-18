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

#include "../include/msg.h"
#include "../include/service.h"

static void msg_reset(struct Msg *msg)
{
    msg->flag.deal_state = MSG_DEAL_STATE_NO; /* 设置当前消息未被处理 */
    msg->flag.service_result = MSG_SERVICE_RET_NONE; /* 设置当前消息未被服务 */
    
    /* 重置消息数据标记 */
#if PROTOCOL_CHOICE == PROTOCOL_AMP
    msg->length = 0; /* 数据标记清空 */
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE
    msg->data_size = 0; /* 数据标记清空 */
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */
}

static int32_t msg_is_dealt(struct Msg *msg)
{
    return msg->flag.deal_state == MSG_DEAL_STATE_YES ? 0 : -1;
}

struct MsgOps msg_ops = 
{
    .msg_reset = msg_reset,
    .msg_is_dealt = msg_is_dealt
};
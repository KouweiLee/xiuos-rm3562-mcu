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

#include "../../../include/amp.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL
extern void* kshm_offset_to_addr(uint32_t offset);

int linux_echo_service(struct Msg *msg)
{
    LOGINFO("linux_echo_service: enter\n");

#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE
    LOGINFO("linux_echo_service recv: pri data = %s,data_size = %u\n",msg->data,msg->data_size);
#endif /* PROTOCOL_CHOICE == PROTOCOL_PRIVATE */

#if PROTOCOL_CHOICE == PROTOCOL_AMP
    char* data = kshm_offset_to_addr(msg->offset);
    LOGINFO("linux_echo_service recv: amp data = %s, data_size = %u, offset = %u\n",data,msg->length,msg->offset);
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */

    LOGINFO("linux_echo_service: over\n");
    
    return 0;
}
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */
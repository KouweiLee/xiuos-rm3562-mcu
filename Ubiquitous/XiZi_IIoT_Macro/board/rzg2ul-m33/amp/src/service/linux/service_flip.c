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

int linux_flip_service(struct Msg *msg)
{
    LOGINFO("linux_flip_service: enter\n");
    int i = 0;
    uint8_t* data = NULL;
    uint32_t data_length = 0;

#if PROTOCOL_CHOICE == PROTOCOL_PRIVATE
    data = msg->data;
    data_length = msg->data_size;
#endif /* PROTOCOL_CHOICE == PROTOCOL_PRIVATE */

#if PROTOCOL_CHOICE == PROTOCOL_AMP
    data = kshm_offset_to_addr(msg->offset);
    data_length = msg->length;
#endif /* PROTOCOL_CHOICE == PROTOCOL_AMP */

    for (i = 0; i < data_length; i++)
    {
        data[i] = ~data[i];
    }

    LOGINFO("linux_flip_service: over\n");
    
    return 0;
}
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */
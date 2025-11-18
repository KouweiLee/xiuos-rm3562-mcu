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

#include "../../include/amp.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL

extern int linux_echo_service(struct Msg *msg);
extern int linux_flip_service(struct Msg *msg);

struct AmpService services[] = 
{
    {
        .service_id = LINUX_SERVICE_ECHO_ID,
        .service = linux_echo_service
    },
    {
        .service_id = LINUX_SERVICE_FLIP_ID,
        .service = linux_flip_service
    },
    {
        .service = NULL
    }
};

#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL */

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

#include "../include/service.h"

#if PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS

static struct AmpService* get_service_by_id(uint32_t id)
{
    int i = 0;
    for (i = 0; services[i].service != NULL; i++)
    {
        if (services[i].service_id == id)
        {
            LOGINFO("get_service_by_id_success: id = %u\n",id);
            
            return &services[i];
        }
    }

    INFOS("get_service_by_id_error: have not such service, id = %u\n",id);
    
    return NULL;
}

struct AmpServiceOps amp_service_ops = 
{
    .get_by_id = get_service_by_id
};
#endif /* PLATFORM_CHOICE == PLATFORM_LINUX_KERNEL || PLATFORM_CHOICE == PLATFORM_RTOS_XIUOS */
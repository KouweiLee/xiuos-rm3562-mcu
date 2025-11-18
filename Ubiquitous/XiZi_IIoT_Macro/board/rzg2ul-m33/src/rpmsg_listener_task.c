
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
* @file rpmsg_listener_task.c
* @brief rpmsg listen task
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2024.1.10
*/

#include <openamp/open_amp.h>
#include "rpmsg_task.h"
#include "platform_info.h"
#include <xizi.h>
#include <shell.h>

extern int init_system (void);
extern void cleanup_system (void);
extern int app (struct rpmsg_device * rdev, void * platform, unsigned long svcno);

void* RPMsgTask_Entry(void* param)
{
    unsigned long         proc_id = 0UL;
    unsigned long         rsc_id  = 0UL;
    struct rpmsg_device * rpdev;
    void                * platform;
    int result;

    result = init_system();
    if (result)
    {
        KPrintf("RPMsgTask_Entry: fail to init remoteproc device\n");
        goto err;
    }
    KPrintf("RPMsgTask_Entry: success to init remoteproc device\n");
    

    result = platform_init(proc_id, rsc_id, &platform);
    if (result)
    {
        KPrintf("RPMsgTask_Entry: fail to create remoteproc device\n");
        goto err;
    }
    KPrintf("RPMsgTask_Entry: success to create remoteproc device\n");

    
    /* RTOS is Master, but this setting must remote in this release. */
    rpdev = platform_create_rpmsg_vdev(platform, 0x0U, VIRTIO_DEV_SLAVE, NULL, NULL);
    if (!rpdev)
    {
        LPERROR("Fail, platform_create_rpmsg_vdev.\n");
        goto err;
    }
    KPrintf("RPMsgTask_Entry: success platform_create_rpmsg_vdev\n");
    
    // 进入APP代码
    (void) app(rpdev, platform, proc_id);

    platform_release_rpmsg_vdev(platform, rpdev);
    KPrintf("RPMsgTask_Entry: success platform_release_rpmsg_vdev\n");

    platform_cleanup(platform);
    KPrintf("RPMsgTask_Entry: success platform_cleanup\n");

    cleanup_system();
    KPrintf("RPMsgTask_Entry: success cleanup_system\n");

    return NULL;
err:
    KPrintf("RPMsgTask_Entry: something err!\n");
    return NULL;
}

void CreateRPMsgTask(void)
{
    int32 rpmsg_task_id = KTaskCreate("rpmsg_task",RPMsgTask_Entry,NULL,2048,SHELL_TASK_PRIORITY + 1);

    if (rpmsg_task_id < 0)
    {
        KPrintf("CreateRPMsgTask: create rpmsg task fail!\n");
        return ;
    }
    KPrintf("CreateRPMsgTask: create rpmsg task success!\n");

    int32 start_up_result = StartupKTask(rpmsg_task_id);

    if (start_up_result != 0)
    {
        KPrintf("CreateRPMsgTask: startup rpmsg task fail\n");
    }
    KPrintf("CreateRPMsgTask: startup rpmsg task success\n");
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_PARAM_NUM(0)|SHELL_CMD_DISABLE_RETURN, CreateRPMsgTask, CreateRPMsgTask, create rpmsg listener task);



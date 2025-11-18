#ifndef __RPMSG_TASK_H__
#define __RPMSG_TASK_H__
#include "bsp_api.h"
#include "hal_data.h"

#include <xizi.h>

#define RECONNECT_FLG       (1)
#define RECONNECT_DLY       (1000U)

// 创建 RPMsg 任务的接口
void CreateRPMsgTask(void);

#endif /* MAINTASK_H_ */

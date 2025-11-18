/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 * PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the
 * Mulan PSL v2 for more details.
 */

/**
 * @file connect_usart.c
 * @brief support ch569 uart function and register to bus framework
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2025-04-07
 */

#include "CH56x_common.h"
#include "xizi.h"
#include "board.h"
#include "shell.h"

#define WDOG_MSECOND_MAX    ( (0xff << 19) / (FREQ_SYS / 1000) )  // watchdog timer is clocked at Fsys/524288, arround 3~228Hz
int g_feed_wdt_stop = 0;

int InitHwWdt(void)
{
    KPrintf("InitHwWdt WDOG_MSECOND_MAX is %d ms\n", WDOG_MSECOND_MAX);

    WWDG_SetCounter(0);
    WWDG_ClearFlag();
    WWDG_ResetCfg(ENABLE);
    while(1) {
        WWDG_SetCounter(0);
        //mDelaymS(100);
        if (g_feed_wdt_stop == 1) {
            KPrintf("%s watchdog feed stop!\n", __func__);
            break;
        }
    }

    return 0;
}

/**
 * @description: Watchdog function
 * @return success: EOK, failure: other
 */
int StartWatchdog(void)
{
    int ret = EOK;

    int32 WdtTask = KTaskCreate("WdtTask", (void *)InitHwWdt, NONE, 1024, 19); 
    StartupKTask(WdtTask);

    return EOK;
}

int WdtTest(int argc, char *argv[])
{
    KPrintf("WdtTest Start\n");
    g_feed_wdt_stop = 1;

    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 WdtTest, WdtTest, test wdt);

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

#include <stdio.h>
#include <string.h>
// #include <user_api.h>
#include <transform.h>

#ifdef CONFIG_LIB_USING_PIKAPYTHON
#include "../lib/pikapython/pikascript-api/pikaScript.h"
#endif

extern int FrameworkInit();
extern void ApplicationOtaTaskInit(void);

#ifdef CONFIG_LIB_USING_PIKAPYTHON
/* PikaPython application interfaces */
extern PikaObj* pikaPythonInit(void);
#endif

#ifdef OTA_BY_PLATFORM
extern int OtaTask(void);
#endif

#ifdef APPLICATION_WEBSERVER
extern int webserver(void);
#endif

int main(void)
{
    printf("\nHello, world!\n");
    FrameworkInit();
    
#ifdef CONFIG_LIB_USING_PIKAPYTHON
    /* Initialize and run PikaPython */
    printf("\n initializing PikaPython...\n");
    PikaObj* pikaMain = pikaPythonInit();
    if (pikaMain != NULL) {
        printf("\n PikaPython test completed successfully!\n");
    } else {
        printf("\n PikaPython initialization failed!\n");
    }
#endif
#ifdef APPLICATION_OTA
    ApplicationOtaTaskInit();
#endif

#ifdef OTA_BY_PLATFORM
    OtaTask();
#endif

#ifdef APPLICATION_WEBSERVER
    webserver();
#endif

    return 0;
}
// int cppmain(void);



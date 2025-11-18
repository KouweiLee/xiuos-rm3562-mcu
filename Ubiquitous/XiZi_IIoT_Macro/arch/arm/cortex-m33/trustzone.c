/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-28     tyx          the first version.
 */

#include <xizi.h>

#ifdef ARM_CM33_ENABLE_TRUSTZONE
extern void TZ_InitContextSystem_S(void);
extern uint32 TZ_AllocModuleContext_S (uint32 module);
extern uint32 TZ_FreeModuleContext_S(uint32 id);
extern uint32 TZ_LoadContext_S(uint32 id);
extern uint32 TZ_StoreContext_S(uint32 id);
#else
void TZ_InitContextSystem_S(void){}
uint32 TZ_AllocModuleContext_S (uint32 module){return 0;}
uint32 TZ_FreeModuleContext_S(uint32 id) {return 0;}
uint32 TZ_LoadContext_S(uint32 id){return 0;};
uint32 TZ_StoreContext_S(uint32 id){return 0;};
#endif
extern int tzcall(int id, x_ubase arg0, x_ubase arg1, x_ubase arg2);

#define TZ_INIT_CONTEXT_ID     (0x1001)
#define TZ_ALLOC_CONTEXT_ID    (0x1002)
#define TZ_FREE_CONTEXT_ID     (0x1003)

x_ubase xz_trustzone_current_context;

void xz_trustzone_init(void)
{
    static uint8 _init;

    if (_init)
        return;
    tzcall(TZ_INIT_CONTEXT_ID, 0, 0, 0);
    _init = 1;
}

int64 xz_trustzone_enter(x_ubase module)
{
    xz_trustzone_init();
    if (tzcall(TZ_ALLOC_CONTEXT_ID, module, 0, 0))
    {
        return EOK;
    }
    return -ERROR;
}

int64 xz_trustzone_exit(void)
{
    tzcall(TZ_FREE_CONTEXT_ID, 0, 0, 0);
    return EOK;
}

void xz_trustzone_context_store(x_ubase context)
{
    TZ_StoreContext_S(context);
}

void xz_trustzone_context_load(x_ubase context)
{
    TZ_LoadContext_S(context);
}

int xz_secure_svc_handle(int svc_id, x_ubase arg0, x_ubase arg1, x_ubase arg2)
{
    int res = 0;

    switch (svc_id)
    {
    case TZ_INIT_CONTEXT_ID:
        TZ_InitContextSystem_S();
        break;
    case TZ_ALLOC_CONTEXT_ID:
        res = TZ_AllocModuleContext_S(arg0);
        if (res <= 0)
        {
            KPrintf("Alloc Context Failed\n");
        }
        else
        {
            xz_trustzone_current_context = res;
            TZ_LoadContext_S(res);
        }
        break;
    case TZ_FREE_CONTEXT_ID:
        TZ_FreeModuleContext_S(xz_trustzone_current_context);
        xz_trustzone_current_context = 0;
        break;
    }
    return res;
}


/***********************************************************************************************************************
 * Copyright [2020-2021] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics Corporation and/or its affiliates and may only
 * be used with products of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.
 * Renesas products are sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for
 * the selection and use of Renesas products and Renesas assumes no liability.  No license, express or implied, to any
 * intellectual property right is granted by Renesas.  This software is protected under all applicable laws, including
 * copyright laws. Renesas reserves the right to change or discontinue this software and/or this documentation.
 * THE SOFTWARE AND DOCUMENTATION IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND
 * TO THE FULLEST EXTENT PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY,
 * INCLUDING WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE
 * SOFTWARE OR DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.
 * TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR
 * DOCUMENTATION (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER,
 * INCLUDING, WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY
 * LOST PROFITS, OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/
/**
 * @file    sample_rpmsg.c
 * @brief   rpmsg sample program for FreeRTOS
 * @date    2020.03.24
 * @author  Copyright (c) 2020, eForce Co., Ltd. All rights reserved.
 *
 ****************************************************************************
 * @par     History
 *          - rev 1.0 (2020.01.28) Imada
 *            Initial version.
 *          - rev 1.1 (2020.03.24) Imada
 *            Employed a dedicated function to set a string for Shared memory API.
 ****************************************************************************
 */
#include <openamp/open_amp.h>
#include "platform_info.h"
#include "rsc_table.h"
#include <xizi.h>

extern int  init_system(void);
extern void cleanup_system(void);

#define SHUTDOWN_MSG     (0xEF56A55A)

/* Local variables */

/*-----------------------------------------------------------------------------*
 *  RPMSG callbacks setup by remoteproc_resource_init()
 *-----------------------------------------------------------------------------*/

/* Local variables */
static struct rpmsg_endpoint rp_ept[CFG_RPMSG_SVCNO] = {0};

volatile static int evt_svc_unbind[CFG_RPMSG_SVCNO] = {0};

/**
 *  Callback Function: rpmsg_endpoint_cb
 *
 *  @param[in] rp_svc
 *  @param[in] data
 *  @param[in] len
 *  @param[in] priv
 *  @param[in] src
 */
static int rpmsg_endpoint_cb0 (struct rpmsg_endpoint * cb_rp_ept, void * data, size_t len, uint32_t src, void * priv)
{
    /* service 0 */
    (void) priv;
    (void) src;

    /* On reception of a shutdown we signal the application to terminate */
    if ((*(unsigned int *) data) == SHUTDOWN_MSG)
    {
        evt_svc_unbind[0] = 1;
        return RPMSG_SUCCESS;
    }
    KPrintf("rpmsg_endpoint_cb0: recv &data = %p, *data = %d,len = %d\n",data,*((int*)(data)),len);
    /* Send data back to master */
    if (rpmsg_send(cb_rp_ept, data, (int) len) < 0)
    {
        KPrintf("rpmsg_endpoint_cb0: rpmsg_send back failed\n");
        return -1;
    }
    return RPMSG_SUCCESS;
}

static int rpmsg_endpoint_cb1 (struct rpmsg_endpoint * cb_rp_ept, void * data, size_t len, uint32_t src, void * priv)
{
    /* service 1 */
    (void) priv;
    (void) src;

    /* On reception of a shutdown we signal the application to terminate */
    if ((*(unsigned int *) data) == SHUTDOWN_MSG)
    {
        evt_svc_unbind[1] = 1;

        return RPMSG_SUCCESS;
    }

    /* Send data back to master */
    if (rpmsg_send(cb_rp_ept, data, (int) len) < 0)
    {
        LPERROR("rpmsg_send failed \n");

        return -1;
    }

    return RPMSG_SUCCESS;
}

/**
 *  Callback Function: rpmsg_service_unbind
 *
 *  @param[in] ept
 */
static void rpmsg_service_unbind0 (struct rpmsg_endpoint * ept)
{
    (void) ept;

    /* service 0 */
    rpmsg_destroy_ept(&rp_ept[0]);
    memset(&rp_ept[0], 0x0, sizeof(struct rpmsg_endpoint));
    evt_svc_unbind[0] = 1;
}

static void rpmsg_service_unbind1 (struct rpmsg_endpoint * ept)
{
    (void) ept;

    /* service 1 */
    rpmsg_destroy_ept(&rp_ept[1]);
    memset(&rp_ept[1], 0x0, sizeof(struct rpmsg_endpoint));
    evt_svc_unbind[1] = 1;
}

/*-----------------------------------------------------------------------------*
 *  Application
 *-----------------------------------------------------------------------------*/
int app (struct rpmsg_device * rdev, void * platform, unsigned long svcno)
{
    (void) platform;
    int ret;

    if (svcno == 0UL)
    {
        ret = rpmsg_create_ept(&rp_ept[0],rdev,CFG_RPMSG_SVC_NAME0,APP_EPT_ADDR,RPMSG_ADDR_ANY,rpmsg_endpoint_cb0,rpmsg_service_unbind0);
        if (ret)
        {
            KPrintf("app: fail to create endpoint[0]\n");
            return -1;
        }
        KPrintf("app: Success to create endpoint[0]\n");
    }
    else
    {
        ret = rpmsg_create_ept(&rp_ept[1],rdev,CFG_RPMSG_SVC_NAME1,APP_EPT_ADDR,RPMSG_ADDR_ANY,rpmsg_endpoint_cb1,rpmsg_service_unbind1);
        if (ret)
        {
            KPrintf("app: fail to create endpoint[1]\n");
            return -1;
        }
        KPrintf("app: success to create endpoint[1]\n");
    }

    LPRINTF("Waiting for master data ...\n");

    while (1)
    {
        DelayKTask(100);

        /* we got a shutdown request, exit */
        if (evt_svc_unbind[svcno])
        {
            // 销毁该节点
            rpmsg_destroy_ept(&rp_ept[svcno]);
            break;
        }
    }

    /* Clear shutdown flag */
    evt_svc_unbind[svcno] = 0;

    return 0;
}

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
/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 * Copyright (c) 2020, eForce Co., Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       platform_info.c
 *
 * DESCRIPTION
 *
 *       This file define platform specific data and implements APIs to set
 *       platform specific information for OpenAMP.
 *
 * @par  History
 *       - rev 1.0 (2020.10.21) Imada
 *         Initial version.
 *
 **************************************************************************/

#include <metal/atomic.h>
#include <metal/assert.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include "platform_info.h"
#include "rsc_table.h"
 #include <metal/sys.h>
 #include <sys/errno.h>

static int32 ipi_tsk_id[CFG_RPMSG_SVCNO] = {-1, -1};

/* IPI(MBX) information */
struct ipi_info ipi =
{
    MBX_DEV_NAME,                      // name
    DEV_BUS_NAME,                      // bus_name
    NULL,                              // dev
    NULL,                              // io
    MBX_INT_NUM,                       // irq_info
    0,                                 // registered
    {0,    0         },                // mbx_chn (not used on this SoC)
    0,                                 // chn_mask (not used on this SoC)
    {-1,    -1},                       // ipi_mutx_id
};

/* vring information */
struct vring_info vrinfo[CFG_RPMSG_SVCNO] =
{
    {                                  // vinfo[0]
        {                              // rsc
            CFG_RSCTBL_DEV_NAME,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
        {                              // ctl
            CFG_VRING_CTL_NAME0,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
        {                              // shm
            CFG_VRING_SHM_NAME0,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
    },
    {                                  // vinfo[1]
        {                              // rsc
            CFG_RSCTBL_DEV_NAME,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
        {                              // ctl
            CFG_VRING_CTL_NAME1,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
        {                              // shm
            CFG_VRING_SHM_NAME1,       // name
            DEV_BUS_NAME,              // bus_name
            NULL,                      // dev
            NULL,                      // io
            {0},                       // mem
        },
    },
};

/* processor operations at RZ/G2. It defines
 * notification operation and remote processor managementi operations. */
extern struct remoteproc_ops rzg2_proc_ops;

/* RPMsg virtio shared buffer pool */
static struct rpmsg_virtio_shm_pool shpool;

static void start_ipi_task(void * platform);

static struct remoteproc * platform_create_proc (int proc_index, int rsc_index)
{
    void                   * rsc_table;
    unsigned int             rsc_size;
    int                      ret;
    struct remoteproc      * rproc_inst;
    struct remoteproc_priv * rproc_priv;
#ifdef __linux__
    metal_phys_addr_t pa;
#endif

    /* Allocate and initialize remoteproc_priv instance */
    rproc_priv = metal_allocate_memory(sizeof(struct remoteproc_priv));
    if (!rproc_priv)
    {
        return NULL;
    }

    memset(rproc_priv, 0, sizeof(*rproc_priv));
    rproc_priv->notify_id = (unsigned int) proc_index;
    rproc_priv->vr_info   = &vrinfo[rsc_index];

    /* Allocate remoteproc instance */
    rproc_inst = metal_allocate_memory(sizeof(struct remoteproc));
    if (!rproc_inst)
    {
        goto err1;
    }

    memset(rproc_inst, 0, sizeof(*rproc_inst));

    /* remoteproc initialization */
    if (!remoteproc_init(rproc_inst, &rzg2_proc_ops, rproc_priv))
    {
        goto err2;
    }

    /*
     * Mmap shared memories
     * Or shall we constraint that they will be set as carved out
     * in the resource table?
     */                              /* FreeRTOS */
    rsc_table          = get_resource_table(rsc_index, &rsc_size);
    rproc_inst->rsc_io = rproc_priv->vr_info->rsc.io;

    /* Parse resource table to remoteproc */
    ret = remoteproc_set_rsc_table(rproc_inst, rsc_table, rsc_size);
    if (ret)
    {
        LPRINTF("Failed to initialize remoteproc\n");
        goto err2;
    }

    LPRINTF("Initialize remoteproc successfully.\n");

    return rproc_inst;
err2:
    (void) remoteproc_remove(rproc_inst);
    metal_free_memory(rproc_inst);
err1:
    metal_free_memory(rproc_priv);

    return NULL;
}

struct  rpmsg_device * platform_create_rpmsg_vdev (void           * platform,
                                                   unsigned int     vdev_index,
                                                   unsigned int     role,
                                                   void (         * rst_cb)(struct virtio_device * vdev),
                                                   rpmsg_ns_bind_cb ns_bind_cb)
{
    struct remoteproc          * rproc = platform;
    struct remoteproc_priv     * prproc;
    struct rpmsg_virtio_device * rpmsg_vdev;
    struct virtio_device       * vdev;
    struct metal_io_region     * shbuf_io;
    metal_phys_addr_t            pa;

    int ret;

    prproc = rproc->priv;

    rpmsg_vdev = metal_allocate_memory(sizeof(*rpmsg_vdev));
    if (!rpmsg_vdev)
    {
        return NULL;
    }

    memset(rpmsg_vdev, 0, sizeof(*rpmsg_vdev));

    LPRINTF("creating remoteproc virtio\n");
    vdev = remoteproc_create_virtio(rproc, (int) vdev_index, role, rst_cb);
    if (!vdev)
    {
        LPRINTF("failed remoteproc_create_virtio\n");
        goto err;
    }

    pa       = metal_io_phys(prproc->vr_info->shm.io, 0x0U);
    shbuf_io = remoteproc_get_io_with_pa(rproc, pa);
    if (!shbuf_io)
    {
        LPRINTF("failed remoteproc_get_io_with_pa\n");
        goto err;
    }

    LPRINTF("initializing rpmsg vdev\n");

    /* RPMsg virtio slave can set shared buffers pool argument to NULL */
    ret = rpmsg_init_vdev(rpmsg_vdev, vdev, ns_bind_cb, shbuf_io, &shpool);
    if (ret)
    {
        LPRINTF("failed rpmsg_init_vdev\n");
        goto err;
    }

    start_ipi_task(rproc);

    return rpmsg_virtio_get_rpmsg_device(rpmsg_vdev);
err:
    remoteproc_remove_virtio(rproc, vdev);
    metal_free_memory(rpmsg_vdev);

    return NULL;
}

int platform_poll (void * priv)
{
    (void) priv;

    return 0;
}

int platform_init (unsigned long proc_id, unsigned long rsc_id, void ** platform)
{
    struct remoteproc * rproc;

    if (!platform)
    {
        LPRINTF("Failed to initialize platform," "NULL pointer to store platform data.\n");

        return -EINVAL;
    }

    if ((proc_id >= RSC_MAX_NUM) || (rsc_id >= RSC_MAX_NUM))
    {
        LPRINTF("Invalid rproc number specified.\n");

        return -EINVAL;
    }

    rproc = platform_create_proc((int) proc_id, (int) rsc_id);
    if (!rproc)
    {
        LPRINTF("Failed to create remoteproc device.\n");

        return -EINVAL;
    }

    *platform = rproc;

    return 0;
}

void platform_release_rpmsg_vdev (void * platform, struct rpmsg_device * rpdev)
{
    /* Need to free memory regions already allocated but not used anymore? */
    struct remoteproc          * rproc = platform;
    struct rpmsg_virtio_device * rpmsg_vdev;
    struct remoteproc_priv     * prproc = rproc->priv;

    ipi.ipi_mutx_id[prproc->notify_id] = -1;

    KTaskDelete(ipi_tsk_id[prproc->notify_id]);
    ipi_tsk_id[prproc->notify_id] = -1;

    rpmsg_vdev = metal_container_of(rpdev, struct rpmsg_virtio_device, rdev);
    rpmsg_deinit_vdev(rpmsg_vdev);
    remoteproc_remove_virtio(rproc, rpmsg_vdev->vdev);
    metal_free_memory(rpmsg_vdev);
}

void platform_cleanup (void * platform)
{
    struct remoteproc      * rproc = platform;
    struct remoteproc_priv * prproc;
    struct metal_device    * dev;

    if (rproc)
    {
        prproc = rproc->priv;
        if (rproc->priv)
        {
            /* Release allocated resource */
            /* Shared memory devices */
            metal_list_del(&(prproc->vr_info->ctl.mem.node));
            dev = prproc->vr_info->ctl.dev;
            if (dev)
            {
                metal_device_close(dev);
            }

            metal_list_del(&(prproc->vr_info->shm.mem.node));
            dev = prproc->vr_info->shm.dev;
            if (dev)
            {
                metal_device_close(dev);
            }

            /* Resource table device */
            metal_list_del(&(prproc->vr_info->rsc.mem.node));
            dev = prproc->vr_info->rsc.dev;
            if (dev)
            {
                metal_device_close(dev);
            }

            /* Release the private area */
            metal_free_memory(rproc->priv);
        }

        (void) remoteproc_remove(rproc);
        metal_free_memory(rproc);
    }
}

static void* IpiTask(void * exinf);

static void* IpiTask (void * exinf)
{
    struct remoteproc      * rproc  = exinf;
    struct remoteproc_priv * prproc = rproc->priv;
    int ret;

    while (1)
    {
        if (ipi.ipi_mutx_id[prproc->notify_id] != 1)
        {
            __WFI();
            continue;
        }
        // KMutexObtain(ipi.ipi_mutx_id[prproc->notify_id], WAITING_FOREVER);

        /* Ignore a incoming interrupt if the virtio layer of a target remoteproc is not yet initialized */
        if (metal_list_is_empty(&rproc->vdevs))
        {
            continue;
        }
        ret = remoteproc_get_notification(rproc, RSC_NOTIFY_ID_ANY);
        // KPrintf("IpiTask: after remoteproc_get_notification\n");
        if (ret)
        {
            KPrintf("remoteproc_get_notification() failed with %d", ret);
            break;
        }
        ipi.ipi_mutx_id[prproc->notify_id] = 0;
        // DelayKTask(10);
    }
}

static void start_ipi_task (void * platform)
{
    struct remoteproc      * rproc  = platform;
    struct remoteproc_priv * prproc = rproc->priv;

    // int32 mutx_id = KMutexCreate();
    // if (mutx_id < 0)
    // {
    //     KPrintf("start_ipi_task: create sem fail!\n");
    //     return ;
    // }
    // ipi.ipi_mutx_id[prproc->notify_id] = mutx_id;
    ipi.ipi_mutx_id[prproc->notify_id] = 0;

    // KPrintf("start_ipi_task: prproc->notify_id = %d, ipi.ipi_mutx_id[prproc->notify_id] = %d \n",prproc->notify_id,ipi.ipi_mutx_id[prproc->notify_id]);

    int32 ipi_task_id = -1;
    ipi_task_id = KTaskCreate("ipi_task",IpiTask,platform,4096,SHELL_TASK_PRIORITY + 1);

    if (ipi_task_id < 0)
    {
        LPRINTF("Failed to register an interrupt service routine.\n");
        return ;
    }
    
    ipi_tsk_id[prproc->notify_id] = ipi_task_id;

    StartupKTask(ipi_task_id);
}
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
 *       rzv2_rproc.c
 *
 * DESCRIPTION
 *
 *       This file defines RZ/V2 CA5X/CR7 remoteproc implementation.
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
#include <metal/list.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include "platform_info.h"
#include <xizi.h>
#include "hal_data.h"

extern struct ipi_info ipi;

/* Inline functions to add accessing address check to corresponding
 * libmetal functions to avoid accessing a reserved region.
 * They are mainly required because of larger (uio) mmap size due the
 * 4KB page size on Verified Linux. They are also utilized for FreeRTOS because
 * accessing a reserved area of registers can cause system failure.
 */
static int init_memory_device (struct remoteproc * rproc, struct shm_info * info) {
    struct metal_device * dev;
    metal_phys_addr_t     mem_pa;
    int ret;

    ret = metal_device_open(info->bus_name, info->name, &dev);
    if (ret)
    {
        LPERROR("Failed to open uio device %s: %d.\n", info->name, ret);
        goto err;
    }

    LPRINTF("Successfully open uio device: %s.\n", info->name);

    info->dev = dev;
    info->io  = metal_device_io_region(dev, 0x0U);
    if (!info->io)
    {
        goto err;
    }

    mem_pa = metal_io_phys(info->io, 0x0U);
    remoteproc_init_mem(&info->mem, info->name, mem_pa, mem_pa, metal_io_region_size(info->io), info->io);
    remoteproc_add_mem(rproc, &info->mem);
    LPRINTF("Successfully added memory device %s.\n", info->name);

    return 0;
err:
    if (info->dev)
    {
        metal_device_close(info->dev);
    }

    return ret;
}

static int rzv2_mhu_callback (mhu_callback_args_t * p_args)
{
    unsigned int         val = 0U;
    /* Clear the interrupt: Already done by MHU driver */

    /* Get a massage from the mailbox */
    val = p_args->msg;

    if (val >= RSC_MAX_NUM)            /* val should have the notify_id of the sender */
    {
        return METAL_IRQ_NOT_HANDLED;  /* Invalid message arrived */
    }

    // if (ipi.ipi_mutx_id[val] >= 0)
    // {
    //     KMutexAbandon(ipi.ipi_mutx_id[val]);
    // }
    if (ipi.ipi_mutx_id[val] != -1)
    {
        ipi.ipi_mutx_id[val] = 1;
    }
    else
    {
        return METAL_IRQ_NOT_HANDLED;
    }

    return METAL_IRQ_HANDLED;
}

static int rzv2_enable_interrupt (struct remoteproc * rproc, struct metal_device * ipi_dev)
{
    unsigned int irq_vect;
    int          ret = 0;

    if (!ipi.registered)
    {
        /* Register interrupt handler and enable interrupt for RZ/V2 CA5X or CR7 */
        irq_vect = (uintptr_t) ipi_dev->irq_info;
        ret      = metal_irq_register((int) irq_vect, R_MHU_NS_IsrSub, ipi_dev, rproc);
        if (ret)
        {
            LPRINTF("metal_irq_register() failed with %d", ret);

            return ret;
        }

        metal_irq_enable(irq_vect);
    }

    return ret;
}

static void rzv2_disable_interrupt (struct remoteproc * rproc)
{
    (void) rproc;
    struct metal_device * dev;
    int ret;

    dev = ipi.dev;
    if (dev)
    {
        metal_irq_disable((uintptr_t) dev->irq_info);
        ret = metal_irq_unregister((int) dev->irq_info, NULL, NULL, NULL);
        if (ret)
        {
            LPRINTF("metal_irq_unregister() failed with %d", ret);

            return;
        }

        metal_device_close(dev);
        ipi.registered = 0;
    }
}

static struct remoteproc * rzv2_proc_init (struct remoteproc * rproc, struct remoteproc_ops * ops, void * arg)
{
    struct remoteproc_priv * prproc = arg;
    struct metal_device    * dev;
    int ret;

    /* Initialize MHU driver */
    g_mhu_ns0.p_api->open(g_mhu_ns0.p_ctrl, g_mhu_ns0.p_cfg);
    g_mhu_ns0.p_api->callbackSet(g_mhu_ns0.p_ctrl, rzv2_mhu_callback, NULL, NULL);

    if ((!rproc) || (!prproc) || (!ops))
    {
        return NULL;
    }

    rproc->priv = prproc;
    rproc->ops  = ops;

    if (!ipi.registered)
    {
        /* Get an IPI device (Mailbox) */
        ret = metal_device_open(ipi.bus_name, ipi.name, &dev);
        if (ret)
        {
            LPERROR("Failed to open ipi device: %d.\n", ret);

            return NULL;
        }

        ipi.dev = dev;
        ipi.io  = metal_device_io_region(dev, 0x0U);
        if (!ipi.io)
        {
            goto err1;
        }

        LPRINTF("Successfully probed IPI device\n");
    }

    ret = rzv2_enable_interrupt(rproc, ipi.dev);
    if (ret)
    {
        LPERROR("Failed to register the interrupt handler.\n");
        goto err1;
    }

    ipi.registered++;

    /* Get the resource table device */
    if (init_memory_device(rproc, &prproc->vr_info->rsc))
    {
        goto err1;
    }

    /* Get VRING related devices */
    if (init_memory_device(rproc, &prproc->vr_info->ctl))
    {
        goto err1;
    }

    if (init_memory_device(rproc, &prproc->vr_info->shm))
    {
        goto err1;
    }

    return rproc;
err1:
    metal_device_close(ipi.dev);

    return NULL;
}

static void rzv2_proc_remove (struct remoteproc * rproc)
{
    if (!rproc)
    {
        return;
    }

    if (ipi.registered > 1)
    {
        ipi.registered--;

        return;
    }

    /* Disable interrupts */
    rzv2_disable_interrupt(rproc);
}

static int rzv2_proc_notify (struct remoteproc * rproc, uint32_t id)
{
    struct remoteproc_priv * prproc = rproc->priv;

    g_mhu_ns0.p_api->msgSend(g_mhu_ns0.p_ctrl, prproc->notify_id);

    return 0;
}

/* processor operations in rzv2. It defines
 * notification operation and remote processor managementi operations. */
struct remoteproc_ops rzv2_proc_ops =
{
    .init   = rzv2_proc_init,
    .remove = rzv2_proc_remove,
    .mmap     = NULL,
    .notify   = rzv2_proc_notify,
    .start    = NULL,
    .stop     = NULL,
    .shutdown = NULL,
};

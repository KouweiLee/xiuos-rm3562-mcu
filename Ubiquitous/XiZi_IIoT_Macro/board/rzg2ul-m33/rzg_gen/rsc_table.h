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
 * @file    rsc_table.h
 * @brief   Configurations for the resource table.
 * @date    2020.03.24
 * @author  Copyright (c) 2020, eForce Co., Ltd. All rights reserved.
 *
 ****************************************************************************
 * @par     History
 *          - rev 1.0 (2020.03.24) Imada
 *            First release for AM65xx
 ****************************************************************************
 */

#ifndef RSC_TABLE_H_
#define RSC_TABLE_H_

#include <stddef.h>
#include "openamp/open_amp.h"
#include "platform_info.h"

/* Place resource table in special ELF section */
#define __section_t(S)    __attribute__((__section__(#S)))
#define __resource              __section_t(.resource_table)

#define RPMSG_IPU_C0_FEATURES    (1U)

/* VirtIO rpmsg device id */
#define VIRTIO_ID_RPMSG_         (7U)

#define NUM_VRINGS               (2U)
#define NUM_TABLE_ENTRIES        (2U)
#define NO_RESOURCE_ENTRIES      (2U)

/* Resource table UIO device */
#define CFG_RSCTBL_DEV_NAME      "42F00000.rsctbl"
#define CFG_RSCTBL_MEM_VA        (0x62F00000U)
#define CFG_RSCTBL_MEM_PA        (0x42F00000U)
#define CFG_RSCTBL_MAP_SIZE      (0x00001000U) // 4KB

/* MDK_ARM compiler does not apply __packed__ for this */
struct remote_resource_table
{
    unsigned int version;
    unsigned int num;
    unsigned int reserved[2];
    unsigned int offset[NO_RESOURCE_ENTRIES];

    /* rproc memory entry */
    struct fw_rsc_rproc_mem rproc_mem;

    /* rpmsg vdev entry */
    struct fw_rsc_vdev       rpmsg_vdev;
    struct fw_rsc_vdev_vring rpmsg_vring0;
    struct fw_rsc_vdev_vring rpmsg_vring1;
};

void * get_resource_table(int rsc_id, unsigned int * len);

#endif                                 /* RSC_TABLE_H_ */

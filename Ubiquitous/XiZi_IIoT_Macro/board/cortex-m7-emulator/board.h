/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
* @file board.h
* @brief definecortex-m7-emulator init configure and start-up function
* @version 1.0
* @author AIIT XUOS Lab
* @date 2021-05-28
*/

/*************************************************
File name: board.h
Description: define cortex-m7-emulator init function and struct
Others:
History:
1. Date: 2023-09-27
Author: AIIT XUOS Lab
Modification:
1. define imxrt-board InitBoardHardware
2. define imxrt-board heap struct
*************************************************/

#ifndef __BOARD_H__
#define __BOARD_H__

#define M7_EMULATOR

#include <stdint.h>

#include <xizi.h>
#include <arch_interrupt.h>
#include "fsl_common.h"
#include "xsconfig.h"

#define NVIC_PRIORITYGROUP_0         0x00000007U /*!< 0 bits for pre-emption priority
                                                      4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         0x00000006U /*!< 1 bits for pre-emption priority
                                                      3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         0x00000005U /*!< 2 bits for pre-emption priority
                                                      2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         0x00000004U /*!< 3 bits for pre-emption priority
                                                      1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         0x00000003U /*!< 4 bits for pre-emption priority*/




extern int heap_start;
extern int heap_end;
#define HEAP_BEGIN          (&heap_start)
#define HEAP_END            (&heap_end)

#define HEAP_SIZE           ((uint32_t)HEAP_END - (uint32_t)HEAP_BEGIN)

void InitBoardHardware(void);



#endif /* _BOARD_H_ */

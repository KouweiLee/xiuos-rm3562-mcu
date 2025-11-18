/*
 * Copyright (c) 2015, Xilinx Inc. and Contributors. All rights reserved.
 * Copyright (c) 2019, eForce.Co.Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file	errno.h
 */

#ifndef __METAL_ERRNO__H__
#define __METAL_ERRNO__H__

#if defined(__CC_ARM) && defined(__GNUC__)
# include <metal/compiler/armcc/errno.h>
#elif defined(__GNUC__)
# include <sys/errno.h>
#elif defined(__ICCARM__)
# include <metal/compiler/iar/errno.h>
#endif

#endif /* __METAL_ERRNO_H__ */

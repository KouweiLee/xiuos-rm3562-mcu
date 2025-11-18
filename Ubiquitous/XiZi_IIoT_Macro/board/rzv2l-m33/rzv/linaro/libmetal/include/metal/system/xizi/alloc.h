/*
 * Copyright (c) 2016, Xilinx Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file	freertos/alloc.c
 * @brief	FreeRTOS libmetal memory allocattion definitions.
 */

#ifndef __METAL_ALLOC__H__
#error "Include metal/alloc.h instead of metal/xizi/alloc.h"
#endif

#ifndef __METAL_FREERTOS_ALLOC__H__
#define __METAL_FREERTOS_ALLOC__H__

#include <xizi.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void *metal_allocate_memory(unsigned int size)
{
	return (x_malloc(size));
}

static inline void metal_free_memory(void *ptr)
{
	x_free(ptr);
}

#ifdef __cplusplus
}
#endif

#endif /* __METAL_FREERTOS_ALLOC__H__ */

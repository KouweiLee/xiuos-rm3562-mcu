/*
 * Copyright (c) 2018, Xilinx Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file	assert.h
 * @brief	FreeRTOS assertion support.
 */
#ifndef __METAL_ASSERT__H__
#error "Include metal/assert.h instead of metal/xizi/assert.h"
#endif

#ifndef __METAL_FREERTOS_ASSERT__H__
#define __METAL_FREERTOS_ASSERT__H__

#ifdef double_assert
#error recursive2
#endif

#define double_assert

//#include </opt/arm/gcc-arm-8.2-2018.11-x86_64-arm-eabi/arm-eabi/include/assert.h>
#include <assert.h>


#undef double_assert

/**
 * @brief Assertion macro for FreeRTOS applications.
 * @param cond Condition to evaluate.
 */
#define metal_sys_assert(cond) assert(cond)

#endif /* __METAL_FREERTOS_ASSERT__H__ */


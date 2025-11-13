/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *    http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */
.global __xizi_smccc_smc

.func __xizi_smccc_smc
__xizi_smccc_smc:
    smc #0
    ldr	x4, [sp]
    stp	x0, x1, [x4, #0]
    stp	x2, x3, [x4, #16]
    ldr	x4, [sp, #8]
    cbz	x4, 1f /* no quirk structure */
    ldr	x9, [x4, #0]
    cmp	x9, #1
    b.ne	1f
    str	x6, [x4, 8]
1:  ret
.endfunc
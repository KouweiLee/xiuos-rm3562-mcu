/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 * See the Mulan PSL v2 for more details.
 */
#include "actracer.h"
#include "core.h"
#include "generic_timer.h"

#include "clock_common_op.h"

// armv8 generic timer driver
#define CNTV_CTL_ENABLE (1 << 0)
#define CNTV_CTL_IMASK (1 << 1)
#define CNTV_CTL_ISTATUS (1 << 2)

static void enable_timer()
{
    uint32_t c = r_cntp_ctl_el0();
    c |= CNTV_CTL_ENABLE;
    c &= ~CNTV_CTL_IMASK;
    w_cntp_ctl_el0(c);
}

static void disable_timer()
{
    uint32_t c = r_cntp_ctl_el0();
    c |= CNTV_CTL_IMASK;
    c &= ~CNTV_CTL_ENABLE;
    w_cntp_ctl_el0(c);
}

static void reload_timer()
{
    // interval 1ms
    static uint32_t ms = 1;
    uint32_t interval = ms * 1000;
    uint32_t interval_clk = interval * (r_cntfrq_el0() / 1000000);
    w_cntp_tval_el0(interval_clk);
}

void _sys_clock_init()
{
    disable_timer();
    reload_timer();
    enable_timer();
}

static uint32_t _get_clock_int()
{
    return 30;
}

static uint64_t _get_tick()
{
    return r_cntvct_el0();
}

static uint64_t _get_second()
{
    return _get_tick() / r_cntfrq_el0();
}

static bool _is_timer_expired()
{
    return true;
}

static void _clear_clock_intr()
{
    disable_timer();
    reload_timer();
    enable_timer();
}

static struct XiziClockDriver hardkernel_clock_driver = {
    .sys_clock_init = _sys_clock_init,
    .get_clock_int = _get_clock_int,
    .get_tick = _get_tick,
    .get_second = _get_second,
    .is_timer_expired = _is_timer_expired,
    .clear_clock_intr = _clear_clock_intr,
};

struct XiziClockDriver* hardkernel_clock_init(struct TraceTag* hardkernel_tag)
{
    hardkernel_clock_driver.sys_clock_init();
    return &hardkernel_clock_driver;
}
/*
 * Copyright (c) 2018 omSquare s.r.o.                       (sam0_rtc_timer.c)
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>  (counter_sam0_tc32.c)
 * Copyright (c) 2022 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_rtc

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_sam0_rtc, CONFIG_COUNTER_LOG_LEVEL);

struct counter_sam0_rtc_ch_data {
    counter_alarm_callback_t callback;
    void* user_data;
};

struct counter_sam0_rtc_data {
    counter_top_callback_t top_cb;
    void* top_user_data;
    uint16_t intflag;           // Target INTFLAG of periodic alarm

    struct counter_sam0_rtc_ch_data ch;
};

struct counter_sam0_rtc_config {
    struct counter_config_info info;
    RtcMode0* regs;
    uint16_t prescaler;

    void (*irq_config_func)(const struct device* dev);
};

/* Helper macro to get the correct GCLK GEN based on configuration. */
#define GCLK_GEN(n)     GCLK_EVAL(n)
#define GCLK_EVAL(n)    GCLK_CLKCTRL_GEN_GCLK##n

static inline void rtc_sync(RtcMode0* regs) {
    /* Wait for bus synchronization... */
    #ifdef RTC_STATUS_SYNCBUSY
    while (regs->STATUS.reg & RTC_STATUS_SYNCBUSY) {
        arch_nop();
    }
    #else
    while (regs->SYNCBUSY.reg) {
        arch_nop();
    }
    #endif
}

static uint32_t read_synchronize_count(RtcMode0* regs) {
    #ifdef RTC_READREQ_RREQ
    RTC0->READREQ.reg = RTC_READREQ_RREQ;
    #endif
    rtc_sync(regs);

    return (regs->COUNT.reg);
}

static void rtc_reset(RtcMode0* regs) {
    rtc_sync(regs);

    /* Disable interrupt. */
    regs->INTENCLR.reg = RTC_MODE0_INTENCLR_MASK;
    /* Clear interrupt flag. */
    regs->INTFLAG.reg  = RTC_MODE0_INTFLAG_MASK;

    /* Disable RTC module. */
    #ifdef RTC_MODE0_CTRL_ENABLE
    regs->CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
    #else
    regs->CTRLA.reg &= ~RTC_MODE0_CTRLA_ENABLE;
    #endif

    rtc_sync(regs);

    /* Initiate software reset. */
    #ifdef RTC_MODE0_CTRL_SWRST
    regs->CTRL.bit.SWRST = 1U;
    while (regs->CTRL.bit.SWRST) {
        arch_nop();
    }
    #else
    regs->CTRLA.bit.SWRST = 1U;
    while (regs->CTRLA.bit.SWRST) {
        arch_nop();
    }
    #endif
}

static int counter_sam0_rtc_start(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

    /* Disable RTC module. */
    #ifdef RTC_MODE0_CTRL_ENABLE
    rtc->CTRL.bit.ENABLE  = 1U;
    #else
    rtc->CTRLA.bit.ENABLE = 1U;
    #endif

    rtc_sync(rtc);

    return (0);
}

static int counter_sam0_rtc_stop(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

    /* Disable RTC module. */
    #ifdef RTC_MODE0_CTRL_ENABLE
    rtc->CTRL.bit.ENABLE  = 0U;
    #else
    rtc->CTRLA.bit.ENABLE = 0U;
    #endif

    rtc_sync(rtc);

    return (0);
}

static uint32_t counter_sam0_rtc_read(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

    return read_synchronize_count(rtc);
}

static int counter_sam0_rtc_get_value(const struct device* dev, uint32_t* ticks) {
    *ticks = counter_sam0_rtc_read(dev);

    return (0);
}

static int counter_sam0_rtc_set_alarm(const struct device* dev,
                                      uint8_t chan_id,
                                      const struct counter_alarm_cfg* alarm_cfg) {
    struct counter_sam0_rtc_data* data = dev->data;
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;
    uint16_t intflag;

    ARG_UNUSED(chan_id);

    unsigned int key = irq_lock();

    data->ch.callback  = alarm_cfg->callback;
    data->ch.user_data = alarm_cfg->user_data;

    rtc_sync(rtc);

    // Determine alarm period via ticks value
    switch (alarm_cfg->ticks) {
        case 32U :
            intflag = RTC_MODE0_INTFLAG_PER2;             // 32/1024 = 31.25 ms (period)
            break;

        case 64U :
            intflag = RTC_MODE0_INTFLAG_PER3;             // 64/1024 = 62.50 ms (period)
            break;

        case 128U :
            intflag = RTC_MODE0_INTFLAG_PER4;             // 128/1024 = 125 ms (period)
            break;

        case 256U :
            intflag = RTC_MODE0_INTFLAG_PER5;             // 256/1024 = 250 ms (period)
            break;

        case 512U :
            intflag = RTC_MODE0_INTFLAG_PER6;             // 512/1024 = 500 ms (period)
            break;

        default : // 1024U
            intflag = RTC_MODE0_INTFLAG_PER7;             // 1024/1024 = 1 s (period)
            break;
    }

    data->intflag     = intflag;
    rtc->INTFLAG.reg  = intflag;
    rtc->INTENSET.reg = intflag;

    irq_unlock(key);

    return (0);
}

static int counter_sam0_rtc_cancel_alarm(const struct device* dev,
                                         uint8_t chan_id) {
    struct counter_sam0_rtc_data* data = dev->data;
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;
    uint16_t intflag;

    unsigned int key = irq_lock();

    ARG_UNUSED(chan_id);

    data->ch.callback = NULL;

    intflag = data->intflag;
    rtc->INTENCLR.reg = intflag;
    rtc->INTFLAG.reg  = intflag;

    irq_unlock(key);

    return (0);
}

static int counter_sam0_rtc_set_top_value(const struct device* dev,
                                          const struct counter_top_cfg* top_cfg) {
    struct counter_sam0_rtc_data* data = dev->data;
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;
    int err = 0;
    unsigned int key = irq_lock();

    if (data->ch.callback != NULL) {
        irq_unlock(key);
        return (-EBUSY);
    }

    if (top_cfg->callback != NULL) {
        data->top_cb        = top_cfg->callback;
        data->top_user_data = top_cfg->user_data;
        rtc->INTENSET.reg   = RTC_MODE0_INTFLAG_CMP0;
    }
    else {
        rtc->INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;
    }

    rtc->COMP[0].reg = top_cfg->ticks;

    if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
        /*
         * Top trigger is on equality of the rising edge only, so
         * manually reset it if the counter has missed the new top.
         */
        if (counter_sam0_rtc_read(dev) >= top_cfg->ticks) {
            err = -ETIME;
            if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
                rtc->COUNT.reg = 0U;
            }
        }
    }
    else {
        rtc->COUNT.reg = 0U;
    }

    rtc_sync(rtc);

    rtc->INTFLAG.reg = RTC_MODE0_INTENCLR_CMP0;
    irq_unlock(key);

    return (err);
}

static uint32_t counter_sam0_rtc_get_pending_int(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

    return (rtc->INTFLAG.reg & RTC_MODE0_INTFLAG_MASK);
}

static uint32_t counter_sam0_rtc_get_top_value(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

    /*
     * Unsync read is safe here because we're not using
     * capture mode, so things are only set from the CPU
     * end.
     */
    return (rtc->COMP[0].reg);
}

static void counter_sam0_rtc_isr(const struct device* dev) {
    struct counter_sam0_rtc_data* data = dev->data;
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc     = cfg->regs;
    uint16_t  status  = rtc->INTFLAG.reg;
    uint16_t  intflag = data->intflag;

    /* Acknowledge all interrupts */
    rtc->INTFLAG.reg = status;
    (void) rtc->INTFLAG.reg;

    if ((status & intflag) != 0U) {
        if (data->ch.callback) {
            counter_alarm_callback_t cb = data->ch.callback;

            rtc->INTFLAG.reg = intflag;
            cb(dev, 0, rtc->COMP[0].reg, data->ch.user_data);
        }
    }

    if ((status & RTC_MODE0_INTFLAG_CMP0) != 0U) {
        rtc->INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;
        if (data->top_cb) {
            data->top_cb(dev, data->top_user_data);
        }
    }
}

static int counter_sam0_rtc_initialize(const struct device* dev) {
    const struct counter_sam0_rtc_config* const cfg = dev->config;
    RtcMode0* rtc = cfg->regs;

#if defined(MCLK)
    MCLK->APBAMASK.reg |= MCLK_APBAMASK_RTC;
    OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K;
#else
    /* Set up bus clock and GCLK generator. */
    PM->APBAMASK.reg |= PM_APBAMASK_RTC;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(RTC_GCLK_ID) | GCLK_CLKCTRL_CLKEN | GCLK_GEN(DT_INST_PROP(0, clock_generator));

    /* Synchronize GCLK. */
    while (GCLK->STATUS.bit.SYNCBUSY) {
        // pass
    }
#endif

    /* Reset module to hardware defaults. */
    rtc_reset(rtc);

    cfg->irq_config_func(dev);

    /* Configure RTC with 32-bit mode, configured prescaler and MATCHCLR. */
    #if defined(RTC_MODE0_CTRL_MODE)
    uint16_t ctrl = RTC_MODE0_CTRL_MODE(0) | cfg->prescaler;
    #else
    uint16_t ctrl = RTC_MODE0_CTRLA_MODE(0) | cfg->prescaler;
    #endif

    #if defined(RTC_MODE0_CTRLA_COUNTSYNC)
    ctrl |= RTC_MODE0_CTRLA_COUNTSYNC;
    #endif

    rtc_sync(rtc);

    #if defined(RTC_MODE0_CTRL_MODE)
    rtc->CTRL.reg  = ctrl;
    #else
    rtc->CTRLA.reg = ctrl;
    #endif

    /* Lets RTC count continually and ignores overflows. */
    rtc->INTFLAG.reg  = RTC_MODE0_INTFLAG_CMP0;
    rtc->INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
    rtc->COMP[0].reg  = 0xFFFFFFFFU;

    /* Enable RTC module. */
    rtc_sync(rtc);

    #if defined(RTC_MODE0_CTRL_ENABLE)
    rtc->CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
    #else
    rtc->CTRLA.reg |= RTC_MODE0_CTRLA_ENABLE;
    #endif

    return (0);
}

static const struct counter_driver_api counter_sam0_rtc_driver_api = {
    .start           = counter_sam0_rtc_start,
    .stop            = counter_sam0_rtc_stop,
    .get_value       = counter_sam0_rtc_get_value,
    .set_alarm       = counter_sam0_rtc_set_alarm,
    .cancel_alarm    = counter_sam0_rtc_cancel_alarm,
    .set_top_value   = counter_sam0_rtc_set_top_value,
    .get_pending_int = counter_sam0_rtc_get_pending_int,
    .get_top_value   = counter_sam0_rtc_get_top_value,
};


#define SAM0_RTC_PRESCALER(n)                           \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prescaler),    \
            (DT_INST_PROP(n, prescaler)), (1))

#define COUNTER_SAM0_RTC_DEVICE(n)                      \
    static void counter_sam0_rtc_config_##n(const struct device *dev); \
    static struct counter_sam0_rtc_config DT_CONST	\
                                                        \
    counter_sam0_rtc_dev_config_##n = {                 \
        .info = {                                       \
            .max_top_value = UINT32_MAX,                \
            .freq = SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ /    \
                SAM0_RTC_PRESCALER(n),                  \
            .flags = COUNTER_CONFIG_INFO_COUNT_UP,      \
            .channels = 1                               \
        },                                              \
        .regs = (RtcMode0*)DT_INST_REG_ADDR(n),         \
        .prescaler = UTIL_CAT(RTC_MODE2_CTRLA_PRESCALER_DIV,    \
                      SAM0_RTC_PRESCALER(n)),           \
        .irq_config_func = &counter_sam0_rtc_config_##n,\
    };                              \
                                    \
    static struct counter_sam0_rtc_data counter_sam0_rtc_dev_data_##n;\
                                    \
    DEVICE_DT_INST_DEFINE(n,        \
                &counter_sam0_rtc_initialize,       \
                NULL,                               \
                &counter_sam0_rtc_dev_data_##n,     \
                &counter_sam0_rtc_dev_config_##n,   \
                PRE_KERNEL_1,                       \
                CONFIG_COUNTER_INIT_PRIORITY,       \
                &counter_sam0_rtc_driver_api);      \
                                    \
    static void counter_sam0_rtc_config_##n(const struct device *dev) { \
        NVIC_ClearPendingIRQ(DT_INST_IRQN(n));      \
        IRQ_CONNECT(DT_INST_IRQN(n),                \
                DT_INST_IRQ(n, priority),           \
                counter_sam0_rtc_isr,               \
                DEVICE_DT_INST_GET(n), 0);          \
        irq_enable(DT_INST_IRQN(n));                \
    }

DT_INST_FOREACH_STATUS_OKAY(COUNTER_SAM0_RTC_DEVICE)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "samc21_reg_stub.h"

void zephyr_gtest_rtc_sam0(void) {
    counter_sam0_rtc_dev_config_0.regs = (RtcMode0*)ut_mcu_rtc_ptr;
}

#endif


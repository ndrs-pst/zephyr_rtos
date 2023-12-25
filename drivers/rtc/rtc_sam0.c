/*
 * Copyright (c) 2023 Bjarki Arge Andreasen
 * Copyright (c) 2023 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_rtc

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

#include <string.h>
#include <soc.h>

/* MAX/MIN equal to (127 x 10^9) / (4096 x 240) = ± 129,191 ppb*/
/* LSB     equal to (  1 x 10^9) / (4096 x 240) =     1,017 ppb*/
#define RTC_SAM0_CALIBRATE_PPB_MAX          (129191)
#define RTC_SAM0_CALIBRATE_PPB_MIN          (-129191)
#define RTC_SAM0_CALIBRATE_PPB_LSB          (1017)

/* Reference Year */
#define RTC_SAM0_REFERENCE_YEAR             (2020U)

/* Reference Year in tm structure year (C standard) */
#define RTC_SAM0_TM_STRUCT_REFERENCE_YEAR   (1900U)

/* Adjust user year with respect to tm structure year (C Standard) */
#define RTC_SAM0_ADJUST_TM_YEAR(year)       (year + RTC_SAM0_TM_STRUCT_REFERENCE_YEAR)

/* Adjust user month */
#define RTC_SAM0_ADJUST_MONTH(month)        (month + 1U)

/* Adjust to tm structure month */
#define RTC_SAM0_ADJUST_TM_STRUCT_MONTH(mon) (mon - 1U)

/* Convert tm_year into CLOCK/ALARM */
#define RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(tm_year)                \
    (((RTC_SAM0_TM_STRUCT_REFERENCE_YEAR + (uint32_t)(tm_year)) - RTC_SAM0_REFERENCE_YEAR) << RTC_MODE2_CLOCK_YEAR_Pos)

/* Convert CLOCK/ALARM register into tm_year */
#define RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(reg_val)                \
    (((((reg_val) & RTC_MODE2_CLOCK_YEAR_Msk) >> RTC_MODE2_CLOCK_YEAR_Pos) + RTC_SAM0_REFERENCE_YEAR) -   \
     RTC_SAM0_TM_STRUCT_REFERENCE_YEAR)

/* Clock/Calendar (Mode 2) */
typedef void (*rtc_sam0_irq_init_fn_ptr)(void);

struct rtc_sam0_config {
    RtcMode2* regs;
    uint16_t  irq_num;
    rtc_sam0_irq_init_fn_ptr irq_init_fn_ptr;
    uint16_t  prescaler;
};

struct rtc_sam0_data {
    #if defined(CONFIG_RTC_ALARM)
    rtc_alarm_callback alarm_callback;
    void* alarm_user_data;
    #endif /* CONFIG_RTC_ALARM */

    #if defined(CONFIG_RTC_UPDATE)
    rtc_update_callback update_callback;
    void* update_user_data;
    #endif /* CONFIG_RTC_UPDATE */

    struct k_spinlock lock;
};

static bool rtc_sam0_validate_tm(const struct rtc_time* timeptr, uint32_t mask) {
    if ((mask & RTC_ALARM_TIME_MASK_SECOND) &&
        ((timeptr->tm_sec < 0) || (timeptr->tm_sec > 59))) {
        return (false);
    }

    if ((mask & RTC_ALARM_TIME_MASK_MINUTE) &&
        ((timeptr->tm_min < 0) || (timeptr->tm_min > 59))) {
        return (false);
    }

    if ((mask & RTC_ALARM_TIME_MASK_HOUR) &&
        ((timeptr->tm_hour < 0) || (timeptr->tm_hour > 23))) {
        return (false);
    }

    if ((mask & RTC_ALARM_TIME_MASK_MONTH) &&
        ((timeptr->tm_mon < 0) || (timeptr->tm_mon > 11))) {
        return (false);
    }

    if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) &&
        ((timeptr->tm_mday < 1) || (timeptr->tm_mday > 31))) {
        return (false);
    }

    if ((mask & RTC_ALARM_TIME_MASK_YEAR) &&
        ((timeptr->tm_year < 0) || (timeptr->tm_year > 199))) {
        return (false);
    }

    return (true);
}

static int rtc_sam0_set_time(const struct device* dev, const struct rtc_time* timeptr) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    uint32_t  clk_val;
    bool rc;

    rc = rtc_sam0_validate_tm(timeptr, UINT32_MAX);
    if (rc == false) {
        return (-EINVAL);
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    clk_val = (uint32_t)(RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(timeptr->tm_year) |
                        ((RTC_SAM0_ADJUST_MONTH((uint32_t)(timeptr->tm_mon))) << RTC_MODE2_CLOCK_MONTH_Pos) |
                        ((uint32_t)timeptr->tm_mday << RTC_MODE2_CLOCK_DAY_Pos)    |
                        ((uint32_t)timeptr->tm_hour << RTC_MODE2_CLOCK_HOUR_Pos)   |
                        ((uint32_t)timeptr->tm_min  << RTC_MODE2_CLOCK_MINUTE_Pos) |
                        ((uint32_t)timeptr->tm_sec  << RTC_MODE2_CLOCK_SECOND_Pos));

    regs->CLOCK.reg = clk_val;

    while (regs->SYNCBUSY.bit.CLOCK == 1U) {
        /* Wait for Synchronization */
    }

    k_spin_unlock(&data->lock, key);

    return (0);
}

static int rtc_sam0_get_time(const struct device* dev, struct rtc_time* timeptr) {
    const struct rtc_sam0_config* config = dev->config;
    RtcMode2* regs = config->regs;
    uint32_t clk_val;
    uint32_t tm_msk;

    if ((regs->CTRLA.reg & RTC_MODE2_CTRLA_CLOCKSYNC) == 0U) {
        regs->CTRLA.reg |= RTC_MODE2_CTRLA_CLOCKSYNC;

        while ((regs->SYNCBUSY.reg & RTC_MODE2_SYNCBUSY_CLOCKSYNC) ==
               RTC_MODE2_SYNCBUSY_CLOCKSYNC) {
            /* Wait for Synchronization */
        }
    }

    while ((regs->SYNCBUSY.reg & RTC_MODE2_SYNCBUSY_CLOCK) ==
           RTC_MODE2_SYNCBUSY_CLOCK) {
        /* Synchronization before reading value from CLOCK Register */
    }

    clk_val = regs->CLOCK.reg;

    tm_msk = RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(clk_val);
    timeptr->tm_year = (int)tm_msk;
    tm_msk = RTC_SAM0_ADJUST_TM_STRUCT_MONTH(((clk_val & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos));
    timeptr->tm_mon  = (int)tm_msk;
    tm_msk = (clk_val & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos;
    timeptr->tm_mday = (int)tm_msk;

    tm_msk = (clk_val & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos;
    timeptr->tm_hour = (int)tm_msk;
    tm_msk = (clk_val & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos;
    timeptr->tm_min  = (int)tm_msk;
    tm_msk = (clk_val & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos;
    timeptr->tm_sec  = (int)tm_msk;

    timeptr->tm_wday  = -1;
    timeptr->tm_yday  = -1;
    timeptr->tm_isdst = -1;
    timeptr->tm_nsec  = 0;

    return (0);
}

static void rtc_sam0_isr(const struct device* dev) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    uint16_t intf  = regs->INTFLAG.reg;

    /* Clear All Interrupts */
    regs->INTFLAG.reg = RTC_MODE2_INTFLAG_MASK;
    (void) regs->INTFLAG.reg;

    #if defined(CONFIG_RTC_ALARM)
    if ((intf & RTC_MODE2_INTFLAG_ALARM0) != 0U) {
        if (data->alarm_callback != NULL) {
            data->alarm_callback(dev, 0, data->alarm_user_data);
        }
    }
    #endif /* CONFIG_RTC_ALARM */

    #if defined(CONFIG_RTC_UPDATE)
    if ((intf & RTC_MODE2_INTFLAG_PER4) != 0U) {
        if (data->update_callback != NULL) {
            data->update_callback(dev, data->update_user_data);
        }
    }
    #endif /* CONFIG_RTC_UPDATE */
}

#if defined(CONFIG_RTC_ALARM)
static uint16_t rtc_sam0_alarm_get_supported_mask(void) {
    return (RTC_ALARM_TIME_MASK_SECOND   | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |
            RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_MONTH  | RTC_ALARM_TIME_MASK_YEAR);
}

static int rtc_sam0_alarm_get_supported_fields(const struct device* dev, uint16_t id, uint16_t* mask) {
    ARG_UNUSED(dev);
    ARG_UNUSED(id);

    *mask = rtc_sam0_alarm_get_supported_mask();

    return (0);
}

static bool rtc_sam0_validate_mask(uint16_t mask) {
    uint16_t msk_val;
    uint16_t chk_msk_bits;

    /* Ensure bits 8 through 6 are not set */
    if (mask & (RTC_ALARM_TIME_MASK_NSEC | RTC_ALARM_TIME_MASK_YEARDAY | RTC_ALARM_TIME_MASK_WEEKDAY)) {
        return (false);
    }

    msk_val = (mask & 0x3FU);                                   /* Extract the first 6 bits */
    chk_msk_bits = RTC_ALARM_TIME_MASK_SECOND;                  /* Starting with the second */

    /* Loop through the bits and check for consecutiveness */
    for (int i = 0; i < 6; ++i) {
        if (msk_val == chk_msk_bits) {
            return (true);
        }

        /* Add the next consecutive bit to the check_mask */
        chk_msk_bits |= (chk_msk_bits << 1);
    }

    return (false);
}

static int rtc_sam0_alarm_set_time(const struct device* dev, uint16_t id, uint16_t mask,
                                   const struct rtc_time* timeptr) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    uint32_t  alm_val;

    if (id != 0) {
        return (-EINVAL);
    }

    if ((mask > 0) && (timeptr == NULL)) {
        return (-EINVAL);
    }

    if (rtc_sam0_validate_mask(mask) == false) {
        return (-EINVAL);
    }

    if (rtc_sam0_validate_tm(timeptr, mask) == false) {
        return (-EINVAL);
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    irq_disable(config->irq_num);

    /*
     * Add 1900 to the tm_year member and the adjust for the RTC reference year
     * Set YEAR(according to Reference Year), MONTH and DAY
     * Set Hour, Minute and second
     */
    alm_val = (uint32_t)(RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(timeptr->tm_year) |
                         ((RTC_SAM0_ADJUST_MONTH((uint32_t)(timeptr->tm_mon))) << RTC_MODE2_CLOCK_MONTH_Pos) |
                         ((uint32_t)timeptr->tm_mday << RTC_MODE2_CLOCK_DAY_Pos)    |
                         ((uint32_t)timeptr->tm_hour << RTC_MODE2_CLOCK_HOUR_Pos)   |
                         ((uint32_t)timeptr->tm_min  << RTC_MODE2_CLOCK_MINUTE_Pos) |
                         ((uint32_t)timeptr->tm_sec  << RTC_MODE2_CLOCK_SECOND_Pos));

    regs->Mode2Alarm[0].ALARM.reg = alm_val;

    while ((regs->SYNCBUSY.reg & RTC_MODE2_SYNCBUSY_ALARM0) == RTC_MODE2_SYNCBUSY_ALARM0) {
        /* Synchronization after writing to ALARM register */
    }

    regs->Mode2Alarm[0].MASK.reg = (uint8_t)mask;

    while ((regs->SYNCBUSY.reg & RTC_MODE2_SYNCBUSY_MASK0) == RTC_MODE2_SYNCBUSY_MASK0) {
        /* Synchronization after writing value to MASK Register */
    }

    regs->INTENSET.reg = (uint16_t)RTC_MODE2_INTENSET_ALARM0;

    irq_enable(config->irq_num);
    k_spin_unlock(&data->lock, key);

    return (0);
}

static int rtc_sam0_alarm_get_time(const struct device* dev, uint16_t id, uint16_t* mask, struct rtc_time* timeptr) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    uint32_t  alm_val;
    uint16_t  msk_val;
    uint32_t  tm_msk;

    if ((id != 0) || (mask == NULL) || (timeptr == NULL)) {
        return (-EINVAL);
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    alm_val = regs->Mode2Alarm[0].ALARM.reg;
    msk_val = regs->Mode2Alarm[0].MASK.reg;

    k_spin_unlock(&data->lock, key);

    (void) memset(timeptr, 0x00, sizeof(*timeptr));

    if ((msk_val & RTC_ALARM_TIME_MASK_YEAR) != 0) {
        tm_msk = RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(alm_val);
        timeptr->tm_year = (int)tm_msk;
    }

    if ((msk_val & RTC_ALARM_TIME_MASK_MONTH) != 0) {
        tm_msk = RTC_SAM0_ADJUST_TM_STRUCT_MONTH(((alm_val & RTC_MODE2_CLOCK_MONTH_Msk) >> RTC_MODE2_CLOCK_MONTH_Pos));
        timeptr->tm_mon  = (int)tm_msk;
    }

    if ((msk_val & RTC_ALARM_TIME_MASK_MONTHDAY) != 0) {
        tm_msk = (alm_val & RTC_MODE2_CLOCK_DAY_Msk) >> RTC_MODE2_CLOCK_DAY_Pos;
        timeptr->tm_mday = (int)tm_msk;
    }

    if ((msk_val & RTC_ALARM_TIME_MASK_HOUR) != 0) {
        tm_msk = (alm_val & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos;
        timeptr->tm_hour = (int)tm_msk;
    }

    if ((msk_val & RTC_ALARM_TIME_MASK_MINUTE) != 0) {
        tm_msk = (alm_val & RTC_MODE2_CLOCK_MINUTE_Msk) >> RTC_MODE2_CLOCK_MINUTE_Pos;
        timeptr->tm_min  = (int)tm_msk;
    }

    if ((msk_val & RTC_ALARM_TIME_MASK_SECOND) != 0) {
        tm_msk = (alm_val & RTC_MODE2_CLOCK_SECOND_Msk) >> RTC_MODE2_CLOCK_SECOND_Pos;
        timeptr->tm_sec  = (int)tm_msk;
    }

    return (0);
}

static int rtc_sam0_alarm_is_pending(const struct device* dev, uint16_t id) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    int rc;

    if (id != 0) {
        return (-EINVAL);
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    if ((regs->INTFLAG.reg & RTC_MODE2_INTFLAG_ALARM_Msk) ==
        RTC_MODE2_INTFLAG_ALARM_Msk) {
        regs->INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;

        rc = 0;
    }
    else {
        rc = 1;
    }

    k_spin_unlock(&data->lock, key);

    return (rc);
}

static int rtc_sam0_alarm_set_callback(const struct device* dev, uint16_t id, rtc_alarm_callback callback,
                                       void* user_data) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;

    if (id != 0) {
        return (-EINVAL);
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    irq_disable(config->irq_num);
    data->alarm_callback  = callback;
    data->alarm_user_data = user_data;

    if (data->alarm_callback) {
        regs->INTENSET.reg = RTC_MODE2_INTENSET_ALARM0;
    }
    else {
        regs->INTENCLR.reg = RTC_MODE2_INTENCLR_ALARM0;
    }

    irq_enable(config->irq_num);
    k_spin_unlock(&data->lock, key);

    return (0);
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int rtc_sam0_update_set_callback(const struct device* dev,
                                        rtc_update_callback callback, void* user_data) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    irq_disable(config->irq_num);

    data->update_callback  = callback;
    data->update_user_data = user_data;

    if (data->update_callback != NULL) {
        regs->INTENSET.reg = RTC_MODE2_INTENSET_PER4;
    }
    else {
        regs->INTENCLR.reg = RTC_MODE2_INTENCLR_PER4;
    }

    irq_enable(config->irq_num);

    k_spin_unlock(&data->lock, key);

    return (0);
}
#endif /* CONFIG_RTC_UPDATE */

#if defined(CONFIG_RTC_CALIBRATION)
// @see 24.6.8.2 Frequency Correction
// @note Based on API for setting RTC calibration.
// A positive calibration value will increase the frequency of the RTC clock,
// a negative value will decrease the frequency of the RTC clock.
static int rtc_sam0_set_calibration(const struct device* dev, int32_t calibration) {
    const struct rtc_sam0_config* config = dev->config;
    struct rtc_sam0_data* data = dev->data;
    RtcMode2* regs = config->regs;
    bool    negative_calibration;
    uint8_t freqcorr;

    if ((calibration < RTC_SAM0_CALIBRATE_PPB_MIN) ||
        (calibration > RTC_SAM0_CALIBRATE_PPB_MAX)) {
        return (-EINVAL);
    }

    /* The value written to the register is absolute */
    if (calibration < 0) {
        negative_calibration = true;
        calibration = -calibration;
    }
    else {
        negative_calibration = false;
    }

    /*
     * Formula adapted from
     * 24.6.8.2 Frequency Correction
     *                       FREQCORR.VALUE
     * Correction in ppb = ----------------- x 10^9 ppm
     *                        4096 x 240
     */
    freqcorr = (uint8_t)((int32_t)(calibration * 4096L * 240L) / (1000000000L));

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    /*
     * The Sign bit in the Frequency Correction register (FREQCORR.SIGN)
     * determines the direction of the correction.
     * A positive value will add counts and increase the period (reducing the frequency).
     * A negative value will reduce counts per period (speeding up the frequency).
     * Inconclusion RTC_FREQCORR_SIGN will be set when speeding up the frequency (calibration > 0).
     */
    if (negative_calibration == true) {
        regs->FREQCORR.reg = freqcorr;
    }
    else {
        regs->FREQCORR.reg = (RTC_FREQCORR_SIGN | freqcorr);
    }

    k_spin_unlock(&data->lock, key);

    return (0);
}

static int rtc_sam0_get_calibration(const struct device* dev, int32_t* calibration) {
    const struct rtc_sam0_config* config = dev->config;
    RtcMode2* regs = config->regs;

    uint8_t freqcorr;
    int32_t correction;

    if (calibration == NULL) {
        return (-EINVAL);
    }

    freqcorr = regs->FREQCORR.reg;

    /* Formula documented in rtc_sam0_set_calibration() */
    if (freqcorr == 0) {
        *calibration = 0;
    }
    else {
        correction   = (freqcorr & 0x7FU);
        *calibration = (correction * RTC_SAM0_CALIBRATE_PPB_LSB);
    }

    /* SIGN Correction Sign
     * The correction value is positive, i.e., frequency will be decreased. (calibration < 0) */
    if ((freqcorr & RTC_FREQCORR_SIGN) == 0U) {
        *calibration = -*calibration;
    }

    return (0);
}
#endif /* CONFIG_RTC_CALIBRATION */

static struct rtc_driver_api const rtc_sam0_driver_api = {
    .set_time = rtc_sam0_set_time,
    .get_time = rtc_sam0_get_time,

    #if defined(CONFIG_RTC_ALARM)
    .alarm_get_supported_fields = rtc_sam0_alarm_get_supported_fields,
    .alarm_set_time     = rtc_sam0_alarm_set_time,
    .alarm_get_time     = rtc_sam0_alarm_get_time,
    .alarm_is_pending   = rtc_sam0_alarm_is_pending,
    .alarm_set_callback = rtc_sam0_alarm_set_callback,
    #endif /* CONFIG_RTC_ALARM */

    #if defined(CONFIG_RTC_UPDATE)
    .update_set_callback = rtc_sam0_update_set_callback,
    #endif /* CONFIG_RTC_UPDATE */

    #if defined(CONFIG_RTC_CALIBRATION)
    .set_calibration = rtc_sam0_set_calibration,
    .get_calibration = rtc_sam0_get_calibration
    #endif /* CONFIG_RTC_CALIBRATION */
};

static int rtc_sam0_init(const struct device* dev) {
    const struct rtc_sam0_config* config = dev->config;
    RtcMode2* regs = config->regs;

    // Select 1.024 kHz from 32.768 kHz external oscillator
    OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K;

    regs->CTRLA.bit.SWRST = 1U;
    while (regs->SYNCBUSY.bit.SWRST == 1U) {
        /* Wait for synchronization after Software Reset */
    }

    // RTC Mode Register Gregorian calendar, 24-hour mode is selected.
    regs->CTRLA.reg = (uint16_t)(RTC_MODE2_CTRLA_MODE_CLOCK    |
                                 RTC_MODE2_CTRLA_PRESCALER(10) |
                                 RTC_MODE2_CTRLA_CLOCKSYNC     |
                                 RTC_MODE2_CTRLA_ENABLE);
    while (regs->SYNCBUSY.bit.CLOCKSYNC == 1U) {
        /* Wait for Synchronization */
    }

    while (regs->SYNCBUSY.bit.ENABLE == 1U) {
        /* Wait for Synchronization after Enabling RTC */
    }

    /* Debug Control, the RTC continues normal operation when the CPU is halted by an external debugger. */
    regs->DBGCTRL.reg = RTC_DBGCTRL_DBGRUN;

    /* RTC Control Register (clear all of interrupt enable bit) */
    regs->INTENCLR.reg = RTC_MODE2_INTENCLR_MASK;

    config->irq_init_fn_ptr();
    irq_enable(config->irq_num);

    return (0);
}

#define RTC_SAM0_DEVICE(id)                                     \
    static void rtc_sam0_irq_init_##id(void) {                  \
        IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),  \
                    rtc_sam0_isr, DEVICE_DT_INST_GET(id), 0);   \
    }                                                           \
                                                                \
    static struct rtc_sam0_config DT_CONST rtc_sam0_config_##id = { \
        .regs            = (RtcMode2*)DT_INST_REG_ADDR(id),     \
        .irq_num         = DT_INST_IRQN(id),                    \
        .irq_init_fn_ptr = rtc_sam0_irq_init_##id,              \
        .prescaler       = DT_INST_PROP(id, prescaler)          \
    };                                                          \
                                                                \
    static struct rtc_sam0_data rtc_sam0_data_##id;             \
                                                                \
    DEVICE_DT_INST_DEFINE(id, rtc_sam0_init, NULL, &rtc_sam0_data_##id, &rtc_sam0_config_##id, \
                          POST_KERNEL, CONFIG_RTC_INIT_PRIORITY, &rtc_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_SAM0_DEVICE);

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "samc21_reg_stub.h"

void zephyr_gtest_rtc_sam0(void) {
    rtc_sam0_config_0.regs = (RtcMode2*)ut_mcu_rtc_ptr;
}

#endif

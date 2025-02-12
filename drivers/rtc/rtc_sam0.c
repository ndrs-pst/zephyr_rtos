/*
 * Copyright (c) 2024-2025 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_rtc

/** @file
 * @brief RTC driver for Atmel SAM0 MCU family.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/rtc.h>
#include "rtc_utils.h"

#include <stdlib.h>
#include <string.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtc_sam0, CONFIG_RTC_LOG_LEVEL);

/* clang-format off */

#define RTC_SAM0_TIME_MASK		\
	(RTC_ALARM_TIME_MASK_SECOND	\
	| RTC_ALARM_TIME_MASK_MINUTE	\
	| RTC_ALARM_TIME_MASK_HOUR	\
	| RTC_ALARM_TIME_MASK_MONTHDAY	\
	| RTC_ALARM_TIME_MASK_MONTH	\
	| RTC_ALARM_TIME_MASK_YEAR	\
	)

/* 24.6.8.2 Frequency Correction */
#define RTC_SAM0_CORRECTION_MAX             (127)

/* Reference Year */
#define RTC_SAM0_REFERENCE_YEAR             (2020)

/* Reference Year in tm structure year (C standard) */
#define RTC_SAM0_TM_STRUCT_REFERENCE_YEAR   (1900)

/* Convert tm_year into CLOCK/ALARM */
#define RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(tm_year)                \
    RTC_MODE2_CLOCK_YEAR((RTC_SAM0_TM_STRUCT_REFERENCE_YEAR + (uint32_t)(tm_year)) - RTC_SAM0_REFERENCE_YEAR)

/* Convert CLOCK/ALARM register into tm_year */
#define RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(YEAR)                   \
    (((int)(YEAR) + RTC_SAM0_REFERENCE_YEAR) - RTC_SAM0_TM_STRUCT_REFERENCE_YEAR)

enum rtc_sam0_counter_mode {
	COUNTER_MODE_0,
	COUNTER_MODE_1,
	COUNTER_MODE_2,
};

struct rtc_sam0_config {
	Rtc *regs;
	enum rtc_sam0_counter_mode mode;
	uint16_t prescaler;

	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint32_t gclk_gen;
	uint16_t gclk_id;
	bool has_gclk;
	bool has_osc32kctrl;
	uint8_t osc32_src;
	uint32_t evt_ctrl_msk;

#ifdef CONFIG_RTC_ALARM
	uint8_t alarms_count;
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_CALIBRATION
	int32_t cal_constant;
#endif
};

struct rtc_sam0_alarm_cb {
	rtc_alarm_callback cb;
	void *cb_data;
};

struct rtc_sam0_data {
	struct k_spinlock lock;

#ifdef CONFIG_RTC_ALARM
	struct rtc_sam0_alarm_cb *const alarms;
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	rtc_update_callback update_callback;
	void *update_user_data;
#endif /* CONFIG_RTC_UPDATE */
};

static inline void rtc_sam0_sync(const Rtc *rtc)
{
	/* Wait for synchronization */
#ifdef MCLK
	while (rtc->MODE2.SYNCBUSY.reg & RTC_MODE2_SYNCBUSY_MASK_) {
		if (IS_ENABLED(__GTEST)) {
			break;
		}
	}
#else
	while (rtc->MODE2.STATUS.reg & RTC_STATUS_SYNCBUSY) {
		if (IS_ENABLED(__GTEST)) {
			break;
		}
	}
#endif
}

static int rtc_sam0_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;
	uint32_t clk_val;
	bool rc;

	rc = rtc_utils_validate_rtc_time(timeptr, RTC_SAM0_TIME_MASK);
	if (rc == false) {
		return -EINVAL;
	}

	clk_val = (RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(timeptr->tm_year) |
		  ((uint32_t)(timeptr->tm_mon + 1)) << RTC_MODE2_CLOCK_MONTH_Pos) |
		  ((uint32_t)timeptr->tm_mday << RTC_MODE2_CLOCK_DAY_Pos)    |
		  ((uint32_t)timeptr->tm_hour << RTC_MODE2_CLOCK_HOUR_Pos)   |
		  ((uint32_t)timeptr->tm_min  << RTC_MODE2_CLOCK_MINUTE_Pos) |
		  ((uint32_t)timeptr->tm_sec  << RTC_MODE2_CLOCK_SECOND_Pos);

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	regs->CLOCK.reg = clk_val;
	rtc_sam0_sync(cfg->regs);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rtc_sam0_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_sam0_config *cfg = dev->config;
	RtcMode2 *regs = &cfg->regs->MODE2;

#ifdef MCLK
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
#endif

	RTC_MODE2_CLOCK_Type calendar = regs->CLOCK;
	timeptr->tm_sec  = calendar.bit.SECOND;
	timeptr->tm_min  = calendar.bit.MINUTE;
	timeptr->tm_hour = calendar.bit.HOUR;
	timeptr->tm_mday = calendar.bit.DAY;
	timeptr->tm_mon  = calendar.bit.MONTH - 1;
	timeptr->tm_year = RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(calendar.bit.YEAR);
	timeptr->tm_wday = -1;
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec  = 0;

	LOG_DBG("D/M/Y H:M:S %02d/%02d/%02d %02d:%02d:%02d",
		timeptr->tm_mday, timeptr->tm_mon + 1, timeptr->tm_year - 99,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

#ifdef CONFIG_RTC_ALARM
static inline uint32_t rtc_sam0_datetime_from_tm(const struct rtc_time *timeptr,
						 uint32_t mask)
{
	uint32_t datetime = 0;

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		datetime |= RTC_MODE2_CLOCK_SECOND(timeptr->tm_sec);
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		datetime |= RTC_MODE2_CLOCK_MINUTE(timeptr->tm_min);
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		datetime |= RTC_MODE2_CLOCK_HOUR(timeptr->tm_hour);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		datetime |= RTC_MODE2_CLOCK_DAY(timeptr->tm_mday);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		datetime |= RTC_MODE2_CLOCK_MONTH(timeptr->tm_mon + 1);
	}

	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
		datetime |= RTC_SAM0_TM_STRUCT_TO_REGS_YEAR(timeptr->tm_year);
	}

	return datetime;
}

static inline void rtc_sam0_tm_from_datetime(struct rtc_time *timeptr, uint32_t mask,
					     RTC_MODE2_ALARM_Type calendar)
{
	memset(timeptr, 0x00, sizeof(struct rtc_time));

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		timeptr->tm_sec = calendar.bit.SECOND;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		timeptr->tm_min = calendar.bit.MINUTE;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		timeptr->tm_hour = calendar.bit.HOUR;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		timeptr->tm_mday = calendar.bit.DAY;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		timeptr->tm_mon = calendar.bit.MONTH - 1;
	}

	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
		timeptr->tm_year = RTC_SAM0_REGS_YEAR_TO_TM_STRUCT(calendar.bit.YEAR);
	}

	timeptr->tm_wday = -1;
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = 0;
}

static inline uint32_t rtc_sam0_alarm_msk_from_mask(uint32_t mask)
{
	uint32_t alarm_mask = 0;

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		alarm_mask = RTC_MODE2_MASK_SEL_SS_Val;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		alarm_mask = RTC_MODE2_MASK_SEL_MMSS_Val;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		alarm_mask = RTC_MODE2_MASK_SEL_HHMMSS_Val;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		alarm_mask = RTC_MODE2_MASK_SEL_DDHHMMSS_Val;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		alarm_mask = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val;
	}

	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
		alarm_mask = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val;
	}

	return alarm_mask;
}

static inline uint32_t rtc_sam0_mask_from_alarm_msk(uint32_t alarm_mask)
{
	uint32_t mask = 0;

	switch (alarm_mask) {
	case RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val:
		mask |= RTC_ALARM_TIME_MASK_YEAR;
		__fallthrough;
	case RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val:
		mask |= RTC_ALARM_TIME_MASK_MONTH;
		__fallthrough;
	case RTC_MODE2_MASK_SEL_DDHHMMSS_Val:
		mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
		__fallthrough;
	case RTC_MODE2_MASK_SEL_HHMMSS_Val:
		mask |= RTC_ALARM_TIME_MASK_HOUR;
		__fallthrough;
	case RTC_MODE2_MASK_SEL_MMSS_Val:
		mask |= RTC_ALARM_TIME_MASK_MINUTE;
		__fallthrough;
	case RTC_MODE2_MASK_SEL_SS_Val:
		mask |= RTC_ALARM_TIME_MASK_SECOND;
		break;
	default:
		break;
	}

	return mask;
}

static int rtc_sam0_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *mask)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	*mask = RTC_SAM0_TIME_MASK;

	return 0;
}

static int rtc_sam0_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;
	uint32_t mask_supported = RTC_SAM0_TIME_MASK;
	uint32_t datetime;
	uint32_t alarm_msk;

	if (BIT(id) > RTC_MODE2_INTFLAG_ALARM_Msk) {
		return -EINVAL;
	}

	if ((mask > 0) && (timeptr == NULL)) {
		return -EINVAL;
	}

	if (mask & ~mask_supported) {
		return -EINVAL;
	}

	if (rtc_utils_validate_rtc_time(timeptr, mask) == false) {
		return -EINVAL;
	}

	datetime = rtc_sam0_datetime_from_tm(timeptr, mask);
	alarm_msk = rtc_sam0_alarm_msk_from_mask(mask);

	LOG_DBG("S: datetime: %d, mask: %d", datetime, alarm_msk);

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	irq_disable(DT_INST_IRQN(0));

	rtc_sam0_sync(cfg->regs);
	regs->Mode2Alarm[id].ALARM.reg = datetime;
	regs->Mode2Alarm[id].MASK.reg = RTC_MODE2_MASK_SEL(alarm_msk);
	regs->INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM(BIT(id));

	irq_enable(DT_INST_IRQN(0));

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rtc_sam0_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	const RtcMode2 *regs = &cfg->regs->MODE2;
	RTC_MODE2_ALARM_Type datetime;
	uint32_t alarm_msk;

	if (BIT(id) > RTC_MODE2_INTFLAG_ALARM_Msk) {
		return -EINVAL;
	}

	if ((mask == NULL) || (timeptr == NULL)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	rtc_sam0_sync(cfg->regs);

	datetime = regs->Mode2Alarm[id].ALARM;
	alarm_msk = regs->Mode2Alarm[id].MASK.reg;

	LOG_DBG("G: datetime: %d, mask: %d", datetime.reg, alarm_msk);

	k_spin_unlock(&data->lock, key);

	*mask = (uint16_t)rtc_sam0_mask_from_alarm_msk(alarm_msk);

	rtc_sam0_tm_from_datetime(timeptr, *mask, datetime);

	return 0;
}

static int rtc_sam0_alarm_is_pending(const struct device *dev, uint16_t id)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;

	if (BIT(id) > RTC_MODE2_INTFLAG_ALARM_Msk) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if ((regs->INTFLAG.reg & RTC_MODE2_INTFLAG_ALARM(BIT(id))) == 0) {
		k_spin_unlock(&data->lock, key);

		return 0;
	}

	regs->INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM(BIT(id));

	k_spin_unlock(&data->lock, key);

	return 1;
}

static int rtc_sam0_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;

	if (BIT(id) > RTC_MODE2_INTFLAG_ALARM_Msk) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->alarms[id].cb = callback;
	data->alarms[id].cb_data = user_data;

	if (callback) {
		regs->INTENSET.reg = RTC_MODE2_INTENSET_ALARM(BIT(id));
	} else {
		regs->INTENCLR.reg = RTC_MODE2_INTENCLR_ALARM(BIT(id));
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int rtc_sam0_update_set_callback(const struct device *dev,
					rtc_update_callback callback, void *user_data)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	irq_disable(DT_INST_IRQN(0));

	data->update_callback  = callback;
	data->update_user_data = user_data;

	/* @see 24.6.8.1 Periodic Intervals
	 *                    f(CLK_RTC_OSC) (default 1,024 Hz)
	 * f(PERIODIC(n)) = -----------------
	 *                     2^(n + 3)
	 * 
	 * for RTC_MODE2_INTENSET_PER4 will be 2^(4 + 3) = 1,024 / 128 = 8 Hz
	 */
	if (data->update_callback != NULL) {
		regs->INTENSET.reg = RTC_MODE2_INTENSET_PER4;
	} else {
		regs->INTENCLR.reg = RTC_MODE2_INTENCLR_PER4;
	}

	irq_enable(DT_INST_IRQN(0));

	k_spin_unlock(&data->lock, key);

	return (0);
}
#endif /* CONFIG_RTC_UPDATE */

static void rtc_sam0_isr(const struct device *dev)
{
	const struct rtc_sam0_config *cfg = dev->config;
	struct rtc_sam0_data *data = dev->data;
	RtcMode2 *regs = &cfg->regs->MODE2;
	uint16_t int_flags = regs->INTFLAG.reg;

	/* Clear All Interrupts */
	regs->INTFLAG.reg = RTC_MODE2_INTFLAG_MASK;
	(void) regs->INTFLAG.reg;

	#if defined(CONFIG_RTC_ALARM)
	for (int i = 0; i < cfg->alarms_count; ++i) {
		if (int_flags & RTC_MODE2_INTFLAG_ALARM(BIT(i))) {
			if (data->alarms[i].cb != NULL) {
				data->alarms[i].cb(dev, (uint16_t)i, data->alarms[i].cb_data);
			}
		}
	}
	#endif

	#if defined(CONFIG_RTC_UPDATE)
	if ((int_flags & RTC_MODE2_INTFLAG_PER4) != 0U) {
		if (data->update_callback != NULL) {
			data->update_callback(dev, data->update_user_data);
		}
	}
	#endif /* CONFIG_RTC_UPDATE */
}


#ifdef CONFIG_RTC_CALIBRATION
/* @see 24.6.8.2 Frequency Correction
 * @note Based on API for setting RTC calibration.
 * A positive calibration value will increase the frequency of the RTC clock,
 * a negative value will decrease the frequency of the RTC clock.
 */
static int rtc_sam0_set_calibration(const struct device *dev, int32_t calibration)
{
	const struct rtc_sam0_config *cfg = dev->config;
	RtcMode2 *regs = &cfg->regs->MODE2;
	int32_t correction = calibration / (1000000000 / cfg->cal_constant);
	uint32_t abs_correction = abs(correction);

	LOG_DBG("Correction: %d, Absolute: %d, Calibration: %d",
		correction, abs_correction, calibration);

	if (abs_correction == 0) {
		regs->FREQCORR.reg = 0;
		return 0;
	}

	if (abs_correction > RTC_SAM0_CORRECTION_MAX) {
		LOG_ERR("The calibration %d result in an out of range value %d",
			calibration, abs_correction);
		return -EINVAL;
	}

	rtc_sam0_sync(cfg->regs);
	regs->FREQCORR.reg = RTC_FREQCORR_VALUE(abs_correction)
			   | (correction < 0 ? RTC_FREQCORR_SIGN : 0);

	LOG_DBG("W REG: 0x%02x", regs->FREQCORR.reg);

	return 0;
}

static int rtc_sam0_get_calibration(const struct device *dev, int32_t *calibration)
{
	const struct rtc_sam0_config *cfg = dev->config;
	const RtcMode2 *regs = &cfg->regs->MODE2;
	int32_t correction;

	if (calibration == NULL) {
		return -EINVAL;
	}

	correction = regs->FREQCORR.bit.VALUE;

	if (correction == 0) {
		*calibration = 0;
	} else {
		*calibration = (correction * 1000000000) / cfg->cal_constant;
	}

	if (regs->FREQCORR.bit.SIGN) {
		*calibration *= -1;
	}

	LOG_DBG("R REG: 0x%02x", regs->FREQCORR.reg);

	return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

static int rtc_sam0_init(const struct device *dev)
{
	const struct rtc_sam0_config *cfg = dev->config;
	RtcMode0 *regs = &cfg->regs->MODE0;

	LOG_DBG("Counter Mode %d selected", cfg->mode);
	LOG_DBG("gclk_id: %d, gclk_gen: %d, prescaler: %d, osc32k: %d",
		cfg->gclk_id, cfg->gclk_gen, cfg->prescaler, cfg->osc32_src);

	*cfg->mclk |= cfg->mclk_mask;

#ifdef MCLK
	if (cfg->has_gclk) {
		GCLK->PCHCTRL[cfg->gclk_id].reg = GCLK_PCHCTRL_CHEN
						| GCLK_PCHCTRL_GEN(cfg->gclk_gen);
	}
#else
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN
			  | GCLK_CLKCTRL_GEN(cfg->gclk_gen)
			  | GCLK_CLKCTRL_ID(cfg->gclk_id);
#endif
	rtc_sam0_sync(cfg->regs);

#ifdef MCLK
	if (cfg->has_osc32kctrl) {
		OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL(cfg->osc32_src);
	}
#endif

	regs->CTRLA.bit.SWRST = 1U;
	while (regs->SYNCBUSY.bit.SWRST == 1U) {
		/* Wait for synchronization after Software Reset */
	}

	rtc_sam0_sync(cfg->regs);
	regs->EVCTRL.reg = (cfg->evt_ctrl_msk & RTC_MODE0_EVCTRL_MASK);

#ifdef MCLK
	regs->CTRLA.reg = RTC_MODE0_CTRLA_ENABLE
			| RTC_MODE0_CTRLA_COUNTSYNC
			| RTC_MODE0_CTRLA_MODE(cfg->mode)
			| RTC_MODE0_CTRLA_PRESCALER(cfg->prescaler + 1);
#else
	regs->CTRL.reg = RTC_MODE0_CTRL_ENABLE
		       | RTC_MODE0_CTRL_MODE(cfg->mode)
		       | RTC_MODE0_CTRL_PRESCALER(cfg->prescaler);
#endif

	regs->INTFLAG.reg = 0;
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    rtc_sam0_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static DEVICE_API(rtc, rtc_sam0_driver_api) = {
	.set_time = rtc_sam0_set_time,
	.get_time = rtc_sam0_get_time,

#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_sam0_alarm_get_supported_fields,
	.alarm_set_time = rtc_sam0_alarm_set_time,
	.alarm_get_time = rtc_sam0_alarm_get_time,
	.alarm_is_pending = rtc_sam0_alarm_is_pending,
	.alarm_set_callback = rtc_sam0_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_sam0_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_sam0_set_calibration,
	.get_calibration = rtc_sam0_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

#define ASSIGNED_CLOCKS_CELL_BY_NAME							\
	ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME

#define RTC_SAM0_GCLK(n)								\
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, gclk),					\
	(										\
		.has_gclk = true,							\
		.gclk_gen = ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),			\
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id)			\
	),										\
	(										\
		.has_gclk = false,							\
		.gclk_gen = 0,								\
		.gclk_id = 0								\
	))

#define RTC_SAM0_OSC32KCTRL(n)								\
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, osc32kctrl),				\
	(										\
		.has_osc32kctrl = true,							\
		.osc32_src = ASSIGNED_CLOCKS_CELL_BY_NAME(n, osc32kctrl, src)		\
	),										\
	(										\
		.has_osc32kctrl = false,						\
		.osc32_src = 0								\
	))

#define RTC_SAM0_DEVICE(n)								\
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, counter_mode),				\
	"sam0:rtc: Missing counter-mode devicetree property");				\
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, prescaler),				\
	"sam0:rtc: Missing prescaler devicetree property");				\
											\
	static struct rtc_sam0_config DT_CONST rtc_sam0_config_##n = {			\
		.regs = (Rtc *)DT_INST_REG_ADDR(n),					\
		.mode = DT_INST_ENUM_IDX(n, counter_mode),				\
		.prescaler = DT_INST_ENUM_IDX(n, prescaler),				\
		.evt_ctrl_msk = DT_INST_PROP(n, event_control_msk),			\
		RTC_SAM0_GCLK(n),							\
		RTC_SAM0_OSC32KCTRL(n),							\
		.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),			\
		.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),		\
		IF_ENABLED(CONFIG_RTC_ALARM, (						\
			.alarms_count = DT_INST_PROP(n, alarms_count),			\
		))									\
		IF_ENABLED(CONFIG_RTC_CALIBRATION, (					\
			.cal_constant = DT_INST_PROP(n, cal_constant),			\
		))									\
	};										\
											\
	IF_ENABLED(CONFIG_RTC_ALARM, (							\
		static struct rtc_sam0_alarm_cb						\
			rtc_sam0_data_cb_##n[DT_INST_PROP(n, alarms_count)] = {};	\
	))										\
											\
	static struct rtc_sam0_data rtc_sam0_data_##n = {				\
		IF_ENABLED(CONFIG_RTC_ALARM, (						\
			.alarms = rtc_sam0_data_cb_##n,					\
		))									\
	};										\
											\
	DEVICE_DT_INST_DEFINE(n, rtc_sam0_init,						\
			      NULL,							\
			      &rtc_sam0_data_##n,					\
			      &rtc_sam0_config_##n, POST_KERNEL,			\
			      CONFIG_RTC_INIT_PRIORITY,					\
			      &rtc_sam0_driver_api);					\

DT_INST_FOREACH_STATUS_OKAY(RTC_SAM0_DEVICE);

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_rtc_sam0(void) {
    rtc_sam0_config_0.regs = (Rtc*)ut_mcu_rtc_ptr;
}

#endif

/* clang-format on */

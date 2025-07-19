/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2022 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Real-time clock control based on the SAM0 series RTC – Real-Time Counter.
 *
 * The core Zephyr API to this device is as a counter, with the
 * following limitations:
 * * counter_read() and counter_*_alarm() cannot be invoked from
 *   interrupt context, as they require communication with the device
 *   over an I2C bus.
 * * many other counter APIs, such as start/stop/set_top_value are not
 *   supported as the clock is always running.
 * * two alarm channels are supported but are not equally capable:
 *   channel 0 supports alarms at 1 s resolution, while channel 1
 *   supports alarms at sub second resolution.
 *
 * Most applications for this device will need to use the extended
 * functionality exposed by this header to access the real-time-clock
 * features.  The majority of these functions must be invoked from
 * supervisor mode.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_SAM0_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_SAM0_H_

#include <time.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/notify.h>

#ifdef __cplusplus
extern "C" {
#endif

// TBA API specific to register callback for periodic ISR

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTC_SAM0_H_ */

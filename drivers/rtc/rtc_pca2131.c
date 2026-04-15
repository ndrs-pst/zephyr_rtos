/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_pca2131

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pca2131, CONFIG_RTC_LOG_LEVEL);

/* PCA2131 register addresses (I2C auto-increments) */
#define PCA2131_CONTROL_1       0x00U
#define PCA2131_CONTROL_2       0x01U
#define PCA2131_CONTROL_3       0x02U
#define PCA2131_CONTROL_4       0x03U
#define PCA2131_CONTROL_5       0x04U
#define PCA2131_SR_RESET        0x05U
#define PCA2131_100TH_SECONDS   0x06U
#define PCA2131_SECONDS         0x07U
#define PCA2131_MINUTES         0x08U
#define PCA2131_HOURS           0x09U
#define PCA2131_DAYS            0x0AU
#define PCA2131_WEEKDAYS        0x0BU
#define PCA2131_MONTHS          0x0CU
#define PCA2131_YEARS           0x0DU
#define PCA2131_SECOND_ALARM    0x0EU
#define PCA2131_MINUTE_ALARM    0x0FU
#define PCA2131_HOUR_ALARM      0x10U
#define PCA2131_DAY_ALARM       0x11U
#define PCA2131_WEEKDAY_ALARM   0x12U
#define PCA2131_CLKOUT_CTRL     0x13U
/* 0x14 .. 0x2F timestamps omitted */
#define PCA2131_AGING_OFFSET    0x30U
#define PCA2131_INT_A_MASK1     0x31U
#define PCA2131_INT_A_MASK2     0x32U
#define PCA2131_INT_B_MASK1     0x33U
#define PCA2131_INT_B_MASK2     0x34U
#define PCA2131_WATCHDOG_CTL    0x35U
#define PCA2131_WATCHDOG_VAL    0x36U

/* Control register bits */
#define PCA2131_CONTROL_1_EXT_TEST    BIT(7)
#define PCA2131_CONTROL_1_TC_DIS      BIT(6)
#define PCA2131_CONTROL_1_STOP        BIT(5)
#define PCA2131_CONTROL_1_100TH_S_DIS BIT(4)
#define PCA2131_CONTROL_1_POR_OVRD    BIT(3)
#define PCA2131_CONTROL_1_12_24       BIT(2)
#define PCA2131_CONTROL_1_MI          BIT(1)
#define PCA2131_CONTROL_1_SI          BIT(0)

#define PCA2131_CONTROL_2_MSF   BIT(7)
#define PCA2131_CONTROL_2_WDTF  BIT(6)
#define PCA2131_CONTROL_2_AF    BIT(4)
#define PCA2131_CONTROL_2_AIE   BIT(1)

#define PCA2131_CONTROL_3_PWRMNG_MASK GENMASK(7, 5)
#define PCA2131_CONTROL_3_BTSE  BIT(4)
#define PCA2131_CONTROL_3_BF    BIT(3)
#define PCA2131_CONTROL_3_BLF   BIT(2)
#define PCA2131_CONTROL_3_BIE   BIT(1)
#define PCA2131_CONTROL_3_BLIE  BIT(0)

#define PCA2131_CONTROL_4_TSF1  BIT(7)
#define PCA2131_CONTROL_4_TSF2  BIT(6)
#define PCA2131_CONTROL_4_TSF3  BIT(5)
#define PCA2131_CONTROL_4_TSF4  BIT(4)

#define PCA2131_CONTROL_5_TSIE1 BIT(7)
#define PCA2131_CONTROL_5_TSIE2 BIT(6)
#define PCA2131_CONTROL_5_TSIE3 BIT(5)
#define PCA2131_CONTROL_5_TSIE4 BIT(4)

/* Time and date register bits */
#define PCA2131_SECONDS_OS     BIT(7)
#define PCA2131_SECONDS_MASK   GENMASK(6, 0)
#define PCA2131_MINUTES_MASK   GENMASK(6, 0)
#define PCA2131_HOURS_AMPM     BIT(5)
#define PCA2131_HOURS_12H_MASK GENMASK(4, 0)
#define PCA2131_HOURS_24H_MASK GENMASK(5, 0)
#define PCA2131_DAYS_MASK      GENMASK(5, 0)
#define PCA2131_WEEKDAYS_MASK  GENMASK(2, 0)
#define PCA2131_MONTHS_MASK    GENMASK(4, 0)
#define PCA2131_YEARS_MASK     GENMASK(7, 0)

/* Alarm register bits (AE_* = 1 disables compare) */
#define PCA2131_SECOND_ALARM_AEN_S  BIT(7)
#define PCA2131_SECOND_ALARM_MASK   GENMASK(6, 0)
#define PCA2131_MINUTE_ALARM_AEN_M  BIT(7)
#define PCA2131_MINUTE_ALARM_MASK   GENMASK(6, 0)
#define PCA2131_HOUR_ALARM_AEN_H    BIT(7)
#define PCA2131_HOUR_ALARM_AMPM     BIT(5)
#define PCA2131_HOUR_ALARM_12H_MASK GENMASK(4, 0)
#define PCA2131_HOUR_ALARM_24H_MASK GENMASK(5, 0)
#define PCA2131_DAY_ALARM_AEN_D     BIT(7)
#define PCA2131_DAY_ALARM_MASK      GENMASK(5, 0)
#define PCA2131_WEEKDAY_ALARM_AEN_W BIT(7)
#define PCA2131_WEEKDAY_ALARM_MASK  GENMASK(2, 0)

/* CLKOUT control bits */
#define PCA2131_CLKOUT_TCR_MASK GENMASK(7, 6)
#define PCA2131_CLKOUT_OTPR     BIT(5)
#define PCA2131_CLKOUT_COF_MASK GENMASK(2, 0)

/* Aging offset bits */
#define PCA2131_AGING_MASK GENMASK(3, 0)

/* RTC alarm time fields supported by the PCA2131 */
#define PCA2131_RTC_ALARM_TIME_MASK \
    (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR | \
     RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_WEEKDAY)

#define PCA2131_RTC_TIME_MASK   \
    (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
     RTC_ALARM_TIME_MASK_MONTH  | RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_YEAR |     \
     RTC_ALARM_TIME_MASK_WEEKDAY)

/* The PCA2131 only supports two-digit years, calculate offset to use */
#define PCA2131_YEARS_OFFSET (2000 - 1900)

/* The PCA2131 enumerates months 1 to 12, RTC API uses 0 to 11 */
#define PCA2131_MONTHS_OFFSET 1

/* Helper macro to guard int1-gpios related code */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int1_gpios) && \
    (defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE))
#define PCA2131_INT1_GPIOS_IN_USE 1
#endif

#define PCA2131_WRITE_REG_LEN_MAX 8U

struct pca2131_config {
    struct i2c_dt_spec i2c;

    #ifdef PCA2131_INT1_GPIOS_IN_USE
    struct gpio_dt_spec int1;
    #endif /* PCA2131_INT1_GPIOS_IN_USE */

    uint8_t cof;
    uint8_t pm;
    bool wakeup_source;
};

struct pca2131_data {
    struct k_mutex lock;

    #if PCA2131_INT1_GPIOS_IN_USE
    struct gpio_callback int1_callback;
    struct k_thread int1_thread;
    struct k_sem int1_sem;

    K_KERNEL_STACK_MEMBER(int1_stack, CONFIG_RTC_PCA2131_THREAD_STACK_SIZE);

    #ifdef CONFIG_RTC_ALARM
    rtc_alarm_callback alarm_callback;
    void* alarm_user_data;
    #endif /* CONFIG_RTC_ALARM */

    #ifdef CONFIG_RTC_UPDATE
    rtc_update_callback update_callback;
    void* update_user_data;
    #endif /* CONFIG_RTC_UPDATE */
#endif /* PCA2131_INT1_GPIOS_IN_USE */
};

static int pca2131_read_regs(const struct device* dev, uint8_t addr, void* buf, size_t len) {
    const struct pca2131_config* config = dev->config;
    int ret;

    ret = i2c_write_read_dt(&config->i2c, &addr, sizeof(addr), buf, len);
    if (ret != 0) {
        LOG_ERR("failed to read reg addr 0x%02x, len %d (ret %d)", addr, len, ret);
        return (ret);
    }

    return (0);
}

static int pca2131_read_reg8(const struct device* dev, uint8_t addr, uint8_t* val) {
    return pca2131_read_regs(dev, addr, val, sizeof(*val));
}

static int pca2131_write_regs(const struct device* dev, uint8_t addr, void* buf, size_t len) {
    const struct pca2131_config* config = dev->config;
    uint8_t block[sizeof(addr) + PCA2131_WRITE_REG_LEN_MAX];
    int ret;

    block[0] = addr;
    memcpy(&block[1], buf, len);

    ret = i2c_write_dt(&config->i2c, block, sizeof(block));
    if (ret != 0) {
        LOG_ERR("failed to write reg addr 0x%02x, len %d (ret %d)", addr, len, ret);
        return (ret);
    }

    return (0);
}

static int pca2131_write_reg8(const struct device* dev, uint8_t addr, uint8_t val) {
    return pca2131_write_regs(dev, addr, &val, sizeof(val));
}

static int pca2131_write_stop_bit_unlocked(const struct device* dev, bool value) {
    uint8_t control_1;
    int ret;

    ret = pca2131_read_reg8(dev, PCA2131_CONTROL_1, &control_1);
    if (ret != 0) {
        return (ret);
    }

    if (value) {
        control_1 |= PCA2131_CONTROL_1_STOP;
    }
    else {
        control_1 &= ~(PCA2131_CONTROL_1_STOP);
    }

    ret = pca2131_write_reg8(dev, PCA2131_CONTROL_1, control_1);

    return (ret);
}

#if PCA2131_INT1_GPIOS_IN_USE
static int pca2131_int1_enable_unlocked(const struct device* dev, bool enable) {
    const struct pca2131_config* config = dev->config;
    uint8_t clkout_ctrl;
    int ret;

    if (!config->wakeup_source) {
        /* Only change COF if not configured as wakeup-source */
        ret = pca2131_read_reg8(dev, PCA2131_CLKOUT_CTRL, &clkout_ctrl);
        if (ret != 0) {
            return (ret);
        }

        if (enable) {
            /* Disable CLKOUT (High-Z) */
            clkout_ctrl &= ~(PCA2131_CLKOUT_COF_MASK);
            clkout_ctrl |= FIELD_PREP(PCA2131_CLKOUT_COF_MASK, 0x7);
        }
        else {
            /* Enable CLKOUT at configured frequency */
            clkout_ctrl &= ~(PCA2131_CLKOUT_COF_MASK);
            clkout_ctrl |= FIELD_PREP(PCA2131_CLKOUT_COF_MASK, config->cof);
        }

        ret = pca2131_write_reg8(dev, PCA2131_CLKOUT_CTRL, clkout_ctrl);
        if (ret != 0) {
            return (ret);
        }
    }

    /* Use edge interrupts to avoid multiple GPIO IRQs while servicing the IRQ in the thread */
    ret = gpio_pin_interrupt_configure_dt(&config->int1,
                                          enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret != 0) {
        LOG_ERR("failed to %s GPIO IRQ (ret %d)", enable ? "enable" : "disable", ret);
    }

    return (ret);
}

static void pca2131_int1_thread(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device* dev = p1;
    struct pca2131_data* data = dev->data;
    rtc_alarm_callback alarm_callback = NULL;
    void* alarm_user_data = NULL;
    rtc_update_callback update_callback = NULL;
    void* update_user_data = NULL;
    uint8_t control_2;
    int ret;

    while (true) {
        k_sem_take(&data->int1_sem, K_FOREVER);
        k_mutex_lock(&data->lock, K_FOREVER);

        ret = pca2131_read_reg8(dev, PCA2131_CONTROL_2, &control_2);
        if (ret != 0) {
            goto unlock;
        }

        #ifdef CONFIG_RTC_ALARM
        if ((control_2 & PCA2131_CONTROL_2_AF) != 0) {
            control_2 &= ~(PCA2131_CONTROL_2_AF);
            if (data->alarm_callback != NULL) {
                alarm_callback  = data->alarm_callback;
                alarm_user_data = data->alarm_user_data;
            }
        }
        #endif /* CONFIG_RTC_ALARM */

        #ifdef CONFIG_RTC_UPDATE
        if ((control_2 & PCA2131_CONTROL_2_MSF) != 0) {
            control_2 &= ~(PCA2131_CONTROL_2_MSF);
            if (data->update_callback != NULL) {
                update_callback  = data->update_callback;
                update_user_data = data->update_user_data;
            }
        }
        #endif /* CONFIG_RTC_UPDATE */

        ret = pca2131_write_reg8(dev, PCA2131_CONTROL_2, control_2);
        if (ret != 0) {
            goto unlock;
        }

        /* Check if interrupt occurred between CONTROL_2 read/write */
        ret = pca2131_read_reg8(dev, PCA2131_CONTROL_2, &control_2);
        if (ret != 0) {
            goto unlock;
        }

        if (((control_2 & PCA2131_CONTROL_2_AF) != 0U && alarm_callback != NULL) ||
            ((control_2 & PCA2131_CONTROL_2_MSF) != 0U && update_callback != NULL)) {
            /*
             * Another interrupt occurred while servicing this one, process current
             * callback(s) and yield.
             */
            k_sem_give(&data->int1_sem);
        }

unlock :
        k_mutex_unlock(&data->lock);

        if (alarm_callback != NULL) {
            alarm_callback(dev, 0U, alarm_user_data);
            alarm_callback = NULL;
        }

        if (update_callback != NULL) {
            update_callback(dev, update_user_data);
            update_callback = NULL;
        }
    }
}

static void pca2131_int1_callback_handler(const struct device* port, struct gpio_callback* cb,
                                          gpio_port_pins_t pins) {
    struct pca2131_data* data = CONTAINER_OF(cb, struct pca2131_data, int1_callback);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    k_sem_give(&data->int1_sem);
}
#endif /* PCA2131_INT1_GPIOS_IN_USE */

static int pca2131_set_time(const struct device* dev, const struct rtc_time* timeptr) {
    struct pca2131_data* data = dev->data;
    uint8_t regs[7];
    int ret;

    if ((timeptr->tm_year < PCA2131_YEARS_OFFSET) ||
        (timeptr->tm_year > PCA2131_YEARS_OFFSET + 99)) {
        return (-EINVAL);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /* Freeze the time circuits */
    ret = pca2131_write_stop_bit_unlocked(dev, true);
    if (ret != 0) {
        goto unlock;
    }

    LOG_DBG("set time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
            "min = %d, sec = %d",
            timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
            timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

    regs[0] = bin2bcd(timeptr->tm_sec)  & PCA2131_SECONDS_MASK;
    regs[1] = bin2bcd(timeptr->tm_min)  & PCA2131_MINUTES_MASK;
    regs[2] = bin2bcd(timeptr->tm_hour) & PCA2131_HOURS_24H_MASK;
    regs[3] = bin2bcd(timeptr->tm_mday) & PCA2131_DAYS_MASK;
    regs[4] = bin2bcd(timeptr->tm_wday) & PCA2131_WEEKDAYS_MASK;
    regs[5] = bin2bcd(timeptr->tm_mon  + PCA2131_MONTHS_OFFSET) & PCA2131_MONTHS_MASK;
    regs[6] = bin2bcd(timeptr->tm_year - PCA2131_YEARS_OFFSET ) & PCA2131_YEARS_MASK;

    /* Write registers PCA2131_SECONDS through PCA2131_YEARS */
    ret = pca2131_write_regs(dev, PCA2131_SECONDS, &regs, sizeof(regs));
    if (ret != 0) {
        goto unlock;
    }

    /* Unfreeze time circuits */
    ret = pca2131_write_stop_bit_unlocked(dev, false);
    if (ret != 0) {
        goto unlock;
    }

unlock :
    k_mutex_unlock(&data->lock);

    return (ret);
}

static int pca2131_get_time(const struct device* dev, struct rtc_time* timeptr) {
    uint8_t regs[7];
    uint8_t control_1;
    int ret;

    ret = pca2131_read_reg8(dev, PCA2131_CONTROL_1, &control_1);
    if (ret != 0) {
        return (ret);
    }

    if ((control_1 & PCA2131_CONTROL_1_STOP) != 0) {
        LOG_WRN("time circuits frozen");
        return (-ENODATA);
    }

    /* Read registers PCA2131_SECONDS through PCA2131_YEARS */
    ret = pca2131_read_regs(dev, PCA2131_SECONDS, &regs, sizeof(regs));
    if (ret != 0) {
        return (ret);
    }

    if ((regs[0] & PCA2131_SECONDS_OS) != 0) {
        LOG_WRN("oscillator stopped or interrupted");
        return (-ENODATA);
    }

    memset(timeptr, 0U, sizeof(*timeptr));
    timeptr->tm_sec   = bcd2bin(regs[0] & PCA2131_SECONDS_MASK);
    timeptr->tm_min   = bcd2bin(regs[1] & PCA2131_MINUTES_MASK);
    timeptr->tm_hour  = bcd2bin(regs[2] & PCA2131_HOURS_24H_MASK);
    timeptr->tm_mday  = bcd2bin(regs[3] & PCA2131_DAYS_MASK);
    timeptr->tm_wday  = bcd2bin(regs[4] & PCA2131_WEEKDAYS_MASK);
    timeptr->tm_mon   = bcd2bin(regs[5] & PCA2131_MONTHS_MASK) - PCA2131_MONTHS_OFFSET;
    timeptr->tm_year  = bcd2bin(regs[6] & PCA2131_YEARS_MASK ) + PCA2131_YEARS_OFFSET;
    timeptr->tm_yday  = -1;
    timeptr->tm_isdst = -1;

    LOG_DBG("get time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
            "min = %d, sec = %d",
            timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
            timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

    return (0);
}

#ifdef CONFIG_RTC_ALARM
static int pca2131_alarm_get_supported_fields(const struct device* dev, uint16_t id, uint16_t* mask) {
    ARG_UNUSED(dev);

    if (id != 0U) {
        LOG_ERR("invalid ID %d", id);
        return (-EINVAL);
    }

    *mask = PCA2131_RTC_ALARM_TIME_MASK;

    return (0);
}

static int pca2131_alarm_set_time(const struct device* dev, uint16_t id, uint16_t mask,
                                  const struct rtc_time* timeptr) {
    uint8_t regs[5];

    if (id != 0U) {
        LOG_ERR("invalid ID %d", id);
        return (-EINVAL);
    }

    if ((mask & ~(PCA2131_RTC_ALARM_TIME_MASK)) != 0U) {
        LOG_ERR("unsupported alarm field mask 0x%04x", mask);
        return (-EINVAL);
    }

    if ((mask & RTC_ALARM_TIME_MASK_SECOND) != 0U) {
        regs[0] = bin2bcd(timeptr->tm_sec) & PCA2131_SECOND_ALARM_MASK;
    }
    else {
        regs[0] = PCA2131_SECOND_ALARM_AEN_S;
    }

    if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0U) {
        regs[1] = bin2bcd(timeptr->tm_min) & PCA2131_MINUTE_ALARM_MASK;
    }
    else {
        regs[1] = PCA2131_MINUTE_ALARM_AEN_M;
    }

    if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0U) {
        regs[2] = bin2bcd(timeptr->tm_hour) & PCA2131_HOUR_ALARM_24H_MASK;
    }
    else {
        regs[2] = PCA2131_HOUR_ALARM_AEN_H;
    }

    if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0U) {
        regs[3] = bin2bcd(timeptr->tm_mday) & PCA2131_DAY_ALARM_MASK;
    }
    else {
        regs[3] = PCA2131_DAY_ALARM_AEN_D;
    }

    if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) {
        regs[4] = bin2bcd(timeptr->tm_wday) & PCA2131_WEEKDAY_ALARM_MASK;
    }
    else {
        regs[4] = PCA2131_WEEKDAY_ALARM_AEN_W;
    }

    LOG_DBG("set alarm: year = %d, mon = %d, mday = %d, hour = %d, min = %d, mask = 0x%04x",
            timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_hour,
            timeptr->tm_min, mask);

    /* Write registers PCA2131_SECOND_ALARM through PCA2131_WEEKDAY_ALARM */
    return pca2131_write_regs(dev, PCA2131_SECOND_ALARM, &regs, sizeof(regs));
}

static int pca2131_alarm_get_time(const struct device* dev, uint16_t id, uint16_t* mask,
                                  struct rtc_time* timeptr) {
    uint8_t regs[5];
    int ret;

    if (id != 0) {
        LOG_ERR("invalid ID %d", id);

        return (-EINVAL);
    }

    /* Read registers PCA2131_SECOND_ALARM through PCA2131_WEEKDAY_ALARM */
    ret = pca2131_read_regs(dev, PCA2131_SECOND_ALARM, &regs, sizeof(regs));
    if (ret != 0) {
        return (ret);
    }

    memset(timeptr, 0U, sizeof(*timeptr));
    *mask = 0U;

    if ((regs[0] & PCA2131_SECOND_ALARM_AEN_S) == 0) {
        timeptr->tm_sec = bcd2bin(regs[0] & PCA2131_SECOND_ALARM_MASK);
        *mask |= RTC_ALARM_TIME_MASK_SECOND;
    }

    if ((regs[1] & PCA2131_MINUTE_ALARM_AEN_M) == 0) {
        timeptr->tm_min = bcd2bin(regs[1] & PCA2131_MINUTE_ALARM_MASK);
        *mask |= RTC_ALARM_TIME_MASK_MINUTE;
    }

    if ((regs[2] & PCA2131_HOUR_ALARM_AEN_H) == 0) {
        timeptr->tm_hour = bcd2bin(regs[2] & PCA2131_HOUR_ALARM_24H_MASK);
        *mask |= RTC_ALARM_TIME_MASK_HOUR;
    }

    if ((regs[3] & PCA2131_DAY_ALARM_AEN_D) == 0) {
        timeptr->tm_mday = bcd2bin(regs[3] & PCA2131_DAY_ALARM_MASK);
        *mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
    }

    if ((regs[4] & PCA2131_WEEKDAY_ALARM_AEN_W) == 0) {
        timeptr->tm_wday = bcd2bin(regs[4] & PCA2131_WEEKDAY_ALARM_MASK);
        *mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
    }

    LOG_DBG("get alarm: year = %d, mon = %d, mday = %d, hour = %d, min = %d, mask = 0x%04x",
            timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_hour,
            timeptr->tm_min, *mask);

    return (0);
}

static int pca2131_alarm_is_pending(const struct device* dev, uint16_t id) {
    struct pca2131_data* data = dev->data;
    uint8_t control_2;
    int ret;

    if (id != 0U) {
        LOG_ERR("invalid ID %d", id);
        return (-EINVAL);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    ret = pca2131_read_reg8(dev, PCA2131_CONTROL_2, &control_2);
    if (ret != 0) {
        goto unlock;
    }

    if ((control_2 & PCA2131_CONTROL_2_AF) != 0) {
        /* Clear alarm flag */
        control_2 &= ~(PCA2131_CONTROL_2_AF);

        ret = pca2131_write_reg8(dev, PCA2131_CONTROL_2, control_2);
        if (ret != 0) {
            goto unlock;
        }

        /* Alarm pending */
        ret = 1;
    }

unlock:
    k_mutex_unlock(&data->lock);

    return (ret);
}

static int pca2131_alarm_set_callback(const struct device* dev, uint16_t id,
                                      rtc_alarm_callback callback, void* user_data) {
    #ifndef PCA2131_INT1_GPIOS_IN_USE
    ARG_UNUSED(dev);
    ARG_UNUSED(id);
    ARG_UNUSED(callback);
    ARG_UNUSED(user_data);

    return (-ENOTSUP);
    #else
    const struct pca2131_config* config = dev->config;
    struct pca2131_data* data = dev->data;
    uint8_t control_2;
    int ret = 0;

    if (config->int1.port == NULL) {
        return (-ENOTSUP);
    }

    if (id != 0U) {
        LOG_ERR("invalid ID %d", id);
        return (-EINVAL);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    data->alarm_callback  = callback;
    data->alarm_user_data = user_data;

    ret = pca2131_read_reg8(dev, PCA2131_CONTROL_2, &control_2);
    if (ret != 0) {
        goto unlock;
    }

    control_2 &= ~(PCA2131_CONTROL_2_AIE);
    if ((callback != NULL) || config->wakeup_source) {
        control_2 |= PCA2131_CONTROL_2_AIE;
    }

    ret = pca2131_int1_enable_unlocked(dev, callback != NULL || data->update_callback != NULL);
    if (ret != 0) {
        goto unlock;
    }

    ret = pca2131_write_reg8(dev, PCA2131_CONTROL_2, control_2);
    if (ret != 0) {
        goto unlock;
    }

unlock :
    k_mutex_unlock(&data->lock);

    /* Wake up the INT1 thread since the alarm flag may already be set */
    k_sem_give(&data->int1_sem);

    return (ret);
#endif /* PCA2131_INT1_GPIOS_IN_USE */
}
#endif /* CONFIG_RTC_ALARM */

#if PCA2131_INT1_GPIOS_IN_USE && defined(CONFIG_RTC_UPDATE)
static int pca2131_update_set_callback(const struct device* dev, rtc_update_callback callback,
                                       void* user_data) {
    const struct pca2131_config* config = dev->config;
    struct pca2131_data* data = dev->data;
    uint8_t control_1;
    int ret;

    if (config->int1.port == NULL) {
        return (-ENOTSUP);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    data->update_callback  = callback;
    data->update_user_data = user_data;

    ret = pca2131_read_reg8(dev, PCA2131_CONTROL_1, &control_1);
    if (ret != 0) {
        goto unlock;
    }

    if (callback != NULL) {
        control_1 |= PCA2131_CONTROL_1_SI;
    }
    else {
        control_1 &= ~(PCA2131_CONTROL_1_SI);
    }

    ret = pca2131_int1_enable_unlocked(dev, callback != NULL || data->alarm_callback != NULL);
    if (ret != 0) {
        goto unlock;
    }

    ret = pca2131_write_reg8(dev, PCA2131_CONTROL_1, control_1);
    if (ret != 0) {
        goto unlock;
    }

unlock :
    k_mutex_unlock(&data->lock);

    /* Wake up the INT1 thread since the seconds flag may already be set */
    k_sem_give(&data->int1_sem);

    return (ret);
}
#endif /* PCA2131_INT1_GPIOS_IN_USE && defined(CONFIG_RTC_UPDATE) */

#ifdef CONFIG_RTC_CALIBRATION

/* Aging offset: AO[3:0], 0x8 = 0 ppm, steps of ~2 ppm. Positive values speed the clock up. */
#define PCA2131_AGING_STEP_PPM  2
#define PCA2131_AGING_MAX_PPM   16
#define PCA2131_AGING_MIN_PPM   (-14)

static int pca2131_set_calibration(const struct device* dev, int32_t freq_ppb) {
    int32_t freq_ppm = DIV_ROUND_CLOSEST(freq_ppb, 1000);
    int32_t freq_ppm_even;
    int32_t ao;

    if (freq_ppm > PCA2131_AGING_MAX_PPM) {
        freq_ppm = PCA2131_AGING_MAX_PPM;
    }
    else if (freq_ppm < PCA2131_AGING_MIN_PPM) {
        freq_ppm = PCA2131_AGING_MIN_PPM;
    }

    /* Convert to nearest even ppm step (2 ppm granularity) */
    freq_ppm_even = DIV_ROUND_CLOSEST(freq_ppm, PCA2131_AGING_STEP_PPM) * PCA2131_AGING_STEP_PPM;

    /* Map 0 ppm to 0x8; positive ppm => below 0x8; negative => above 0x8 */
    ao = 8 - (freq_ppm_even / PCA2131_AGING_STEP_PPM);
    ao = z_clamp(ao, 0, 15);

    LOG_DBG("freq_ppb=%d, freq_ppm_even=%d, ao=%d", freq_ppb, freq_ppm_even, ao);

    return pca2131_write_reg8(dev, PCA2131_AGING_OFFSET, (uint8_t)ao);
}

static int pca2131_get_calibration(const struct device* dev, int32_t* freq_ppb) {
    uint8_t ao;
    int32_t freq_ppm;
    int ret;

    ret = pca2131_read_reg8(dev, PCA2131_AGING_OFFSET, &ao);
    if (ret != 0) {
        return (ret);
    }

    freq_ppm  = (8 - (ao & PCA2131_AGING_MASK)) * PCA2131_AGING_STEP_PPM;
    *freq_ppb = freq_ppm * 1000;

    LOG_DBG("ao=0x%02x, freq_ppb=%d", ao, *freq_ppb);

    return (0);
}
#endif /* CONFIG_RTC_CALIBRATION */

static int pca2131_init(const struct device* dev) {
    const struct pca2131_config* config = dev->config;
    struct pca2131_data* data = dev->data;
    uint8_t clkout_ctrl;
    uint8_t regs[3];
    int ret;

    k_mutex_init(&data->lock);

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus not ready");
        return (-ENODEV);
    }

    #if PCA2131_INT1_GPIOS_IN_USE
    k_tid_t tid;

    if (config->int1.port != NULL) {
        k_sem_init(&data->int1_sem, 0, INT_MAX);

        if (!gpio_is_ready_dt(&config->int1)) {
            LOG_ERR("GPIO not ready");
            return (-ENODEV);
        }

        ret = gpio_pin_configure_dt(&config->int1, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("failed to configure GPIO (ret %d)", ret);
            return (-ENODEV);
        }

        gpio_init_callback(&data->int1_callback, pca2131_int1_callback_handler,
                           BIT(config->int1.pin));

        ret = gpio_add_callback_dt(&config->int1, &data->int1_callback);
        if (ret != 0) {
            LOG_ERR("failed to add GPIO callback (ret %d)", ret);
            return (-ENODEV);
        }

        tid = k_thread_create(&data->int1_thread, data->int1_stack,
                              K_THREAD_STACK_SIZEOF(data->int1_stack),
                              pca2131_int1_thread, (void*)dev, NULL,
                              NULL, CONFIG_RTC_PCA2131_THREAD_PRIO, 0, K_NO_WAIT);
        k_thread_name_set(tid, "pca2131");

        /*
         * Defer GPIO interrupt configuration due to INT1/CLKOUT pin sharing. This allows
         * using the CLKOUT square-wave signal for RTC calibration when no alarm/update
         * callbacks are enabled (and not configured as a wakeup-source).
         */
    }
    #endif /* PCA2131_INT1_GPIOS_IN_USE */

    /*
     * Manually initialize the required PCA2131 registers as performing a software reset will
     * reset the time circuits.
     */

    /* Read registers PCA2131_CONTROL_1 through PCA2131_CONTROL_3 */
    ret = pca2131_read_regs(dev, PCA2131_CONTROL_1, &regs, sizeof(regs));
    if (ret != 0) {
        return (-ENODEV);
    }

    /* Use 24h time format, disable second/minute interrupts by default */
    regs[0] &= ~(PCA2131_CONTROL_1_12_24 | PCA2131_CONTROL_1_SI | PCA2131_CONTROL_1_MI);

    /* Clear minute/second and alarm flags (write 0 clears, 1 leaves unchanged) */
    regs[1] &= ~(PCA2131_CONTROL_2_MSF | PCA2131_CONTROL_2_AF);

    /* Configure alarm interrupt enable depending on wakeup-source */
    if (config->wakeup_source) {
        regs[1] |= PCA2131_CONTROL_2_AIE;
    }
    else {
        regs[1] &= ~(PCA2131_CONTROL_2_AIE);
    }

    /* Configure battery switch-over function */
    regs[2] &= ~(PCA2131_CONTROL_3_PWRMNG_MASK);
    regs[2] |= FIELD_PREP(PCA2131_CONTROL_3_PWRMNG_MASK, config->pm);

    /* Clear battery status flag, disable battery interrupts */
    regs[2] &= ~(PCA2131_CONTROL_3_BF);
    regs[2] &= ~(PCA2131_CONTROL_3_BIE | PCA2131_CONTROL_3_BLIE);

    /* Write registers PCA2131_CONTROL_1 through PCA2131_CONTROL_3 */
    ret = pca2131_write_regs(dev, PCA2131_CONTROL_1, &regs, sizeof(regs));
    if (ret != 0) {
        return (-ENODEV);
    }

    /* Configure CLKOUT frequency */
    clkout_ctrl = FIELD_PREP(PCA2131_CLKOUT_COF_MASK, config->cof);

    ret = pca2131_write_reg8(dev, PCA2131_CLKOUT_CTRL, clkout_ctrl);
    if (ret != 0) {
        return (-ENODEV);
    }

    return (0);
}

/* Mapping from DT battery-switch-over enum to CONTROL_3 PWRMNG field value */
#define PCA2131_PM_STANDARD 0U
#define PCA2131_PM_DIRECT   3U
#define PCA2131_PM_DISABLED 7U

#ifdef CONFIG_PM_DEVICE
static int pca2131_pm_action(const struct device* dev, enum pm_device_action action) {
    const struct pca2131_config* config = dev->config;
    uint8_t control_3;
    int ret;

    if (config->pm == PCA2131_PM_DISABLED) {
        /* Only one power supply */
        return (-ENOTSUP);
    }

    switch (action) {
        case PM_DEVICE_ACTION_SUSPEND :
            /* Disable battery switch-over function */
            control_3 = FIELD_PREP(PCA2131_CONTROL_3_PWRMNG_MASK, PCA2131_PM_DISABLED);
            break;

        case PM_DEVICE_ACTION_RESUME :
            /* Re-enable battery switch-over function */
            control_3 = FIELD_PREP(PCA2131_CONTROL_3_PWRMNG_MASK, config->pm);
            break;

        default :
            return (-ENOTSUP);
    }

    ret = pca2131_write_reg8(dev, PCA2131_CONTROL_3, control_3);
    if (ret != 0) {
        return -EIO;
    }

    return (0);
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(rtc, pca2131_driver_api) = {
    .set_time = pca2131_set_time,
    .get_time = pca2131_get_time,

    #ifdef CONFIG_RTC_ALARM
    .alarm_get_supported_fields = pca2131_alarm_get_supported_fields,
    .alarm_set_time             = pca2131_alarm_set_time,
    .alarm_get_time             = pca2131_alarm_get_time,
    .alarm_is_pending           = pca2131_alarm_is_pending,
    .alarm_set_callback         = pca2131_alarm_set_callback,
    #endif /* CONFIG_RTC_ALARM */

    #if PCA2131_INT1_GPIOS_IN_USE && defined(CONFIG_RTC_UPDATE)
    .update_set_callback = pca2131_update_set_callback,
    #endif /* PCA2131_INT1_GPIOS_IN_USE && defined(CONFIG_RTC_UPDATE) */

    #ifdef CONFIG_RTC_CALIBRATION
    .set_calibration = pca2131_set_calibration,
    .get_calibration = pca2131_get_calibration,
    #endif /* CONFIG_RTC_CALIBRATION */
};

#define PCA2131_PM_FROM_DT_INST(inst) \
    UTIL_CAT(PCA2131_PM_, DT_INST_STRING_UPPER_TOKEN(inst, battery_switch_over))

#define PCA2131_INIT(inst)          \
    static struct pca2131_config DT_CONST pca2131_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                          \
        .cof = DT_INST_ENUM_IDX(inst, clkout_frequency),            \
        .pm  = PCA2131_PM_FROM_DT_INST(inst),                       \
        .wakeup_source = DT_INST_PROP(inst, wakeup_source),         \
        IF_ENABLED(PCA2131_INT1_GPIOS_IN_USE,                       \
                   (.int1 = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0})))}; \
                                                                    \
    static struct pca2131_data pca2131_data_##inst;                 \
                                                                    \
    PM_DEVICE_DT_INST_DEFINE(inst, pca2131_pm_action);              \
                                                                    \
    DEVICE_DT_INST_DEFINE(inst, &pca2131_init, PM_DEVICE_DT_INST_GET(inst), \
                          &pca2131_data_##inst, &pca2131_config_##inst, POST_KERNEL, \
                          CONFIG_RTC_INIT_PRIORITY, &pca2131_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PCA2131_INIT)

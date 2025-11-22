/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_fs26_wdog

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_nxp_fs26);

#include "wdt_nxp_fs26.h"

#if defined(CONFIG_BIG_ENDIAN)
#define SWAP_ENDIANNESS
#endif

#define FS26_CRC_TABLE_SIZE      256U
#define FS26_CRC_INIT            0xFF
#define FS26_FS_WD_TOKEN_DEFAULT 0x5AB2
#define FS26_INIT_FS_TIMEOUT_MS  1000U

/* Helper macros to set register values from Kconfig options */
#define WD_ERR_LIMIT(x) Z_CONCAT(WD_ERR_LIMIT_, x)
#define WD_RFR_LIMIT(x) Z_CONCAT(WD_RFR_LIMIT_, x)
#define WDW_PERIOD(x)   Z_CONCAT(_CONCAT(WDW_PERIOD_, x), MS)

#define BAD_WD_REFRESH_ERROR_STRING(x)                          \
    ((((x) & BAD_WD_DATA) ? "error in the data" :               \
        (((x) & BAD_WD_TIMING) ? "error in the timing (window)" \
        : "unknown error")))

enum fs26_wd_type {
    FS26_WD_SIMPLE,
    FS26_WD_CHALLENGER
};

struct fs26_spi_rx_frame {
    union {
        struct {
            uint8_t m_aval : 1;
            uint8_t fs_en  : 1;
            uint8_t fs_g   : 1;
            uint8_t com_g  : 1;
            uint8_t wio_g  : 1;
            uint8_t vsup_g : 1;
            uint8_t reg_g  : 1;
            uint8_t tsd_g  : 1;
        };

        uint8_t raw;
    } status;

    uint16_t data;
};

struct fs26_spi_tx_frame {
    bool     write;
    uint8_t  addr;
    uint16_t data;
};

struct wdt_nxp_fs26_config {
    struct spi_dt_spec  spi;
    enum fs26_wd_type   wd_type;
    struct gpio_dt_spec int_gpio;
};

struct wdt_nxp_fs26_data {
    wdt_callback_t callback;
    uint16_t token;             /* local copy of the watchdog token */
    bool timeout_installed;
    uint8_t window_period;
    uint8_t window_duty_cycle;
    uint8_t fs_reaction;
    struct gpio_callback int_gpio_cb;
    struct k_sem int_sem;
    struct k_thread int_thread;

    K_KERNEL_STACK_MEMBER(int_thread_stack, CONFIG_WDT_NXP_FS26_INT_THREAD_STACK_SIZE);
};

/*
 * Allowed values for watchdog period and duty cycle (CLOSED window).
 * The index is the value to write to the register. Keep values in ascending order.
 */
static uint32_t const fs26_period_values[] = {
    0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 64, 128, 256, 512, 1024
};

static double const fs26_dc_closed_values[] = {
    0.3125, 0.375, 0.5, 0.625, 0.6875, 0.75, 0.8125
};

/* CRC lookup table */
static uint8_t const FS26_CRC_TABLE[FS26_CRC_TABLE_SIZE] = {
    0x00U, 0x1DU, 0x3AU, 0x27U, 0x74U, 0x69U, 0x4EU, 0x53U,
    0xE8U, 0xF5U, 0xD2U, 0xCFU, 0x9CU, 0x81U, 0xA6U, 0xBBU,
    0xCDU, 0xD0U, 0xF7U, 0xEAU, 0xB9U, 0xA4U, 0x83U, 0x9EU,
    0x25U, 0x38U, 0x1FU, 0x02U, 0x51U, 0x4CU, 0x6BU, 0x76U,
    0x87U, 0x9AU, 0xBDU, 0xA0U, 0xF3U, 0xEEU, 0xC9U, 0xD4U,
    0x6FU, 0x72U, 0x55U, 0x48U, 0x1BU, 0x06U, 0x21U, 0x3CU,
    0x4AU, 0x57U, 0x70U, 0x6DU, 0x3EU, 0x23U, 0x04U, 0x19U,
    0xA2U, 0xBFU, 0x98U, 0x85U, 0xD6U, 0xCBU, 0xECU, 0xF1U,
    0x13U, 0x0EU, 0x29U, 0x34U, 0x67U, 0x7AU, 0x5DU, 0x40U,
    0xFBU, 0xE6U, 0xC1U, 0xDCU, 0x8FU, 0x92U, 0xB5U, 0xA8U,
    0xDEU, 0xC3U, 0xE4U, 0xF9U, 0xAAU, 0xB7U, 0x90U, 0x8DU,
    0x36U, 0x2BU, 0x0CU, 0x11U, 0x42U, 0x5FU, 0x78U, 0x65U,
    0x94U, 0x89U, 0xAEU, 0xB3U, 0xE0U, 0xFDU, 0xDAU, 0xC7U,
    0x7CU, 0x61U, 0x46U, 0x5BU, 0x08U, 0x15U, 0x32U, 0x2FU,
    0x59U, 0x44U, 0x63U, 0x7EU, 0x2DU, 0x30U, 0x17U, 0x0AU,
    0xB1U, 0xACU, 0x8BU, 0x96U, 0xC5U, 0xD8U, 0xFFU, 0xE2U,
    0x26U, 0x3BU, 0x1CU, 0x01U, 0x52U, 0x4FU, 0x68U, 0x75U,
    0xCEU, 0xD3U, 0xF4U, 0xE9U, 0xBAU, 0xA7U, 0x80U, 0x9DU,
    0xEBU, 0xF6U, 0xD1U, 0xCCU, 0x9FU, 0x82U, 0xA5U, 0xB8U,
    0x03U, 0x1EU, 0x39U, 0x24U, 0x77U, 0x6AU, 0x4DU, 0x50U,
    0xA1U, 0xBCU, 0x9BU, 0x86U, 0xD5U, 0xC8U, 0xEFU, 0xF2U,
    0x49U, 0x54U, 0x73U, 0x6EU, 0x3DU, 0x20U, 0x07U, 0x1AU,
    0x6CU, 0x71U, 0x56U, 0x4BU, 0x18U, 0x05U, 0x22U, 0x3FU,
    0x84U, 0x99U, 0xBEU, 0xA3U, 0xF0U, 0xEDU, 0xCAU, 0xD7U,
    0x35U, 0x28U, 0x0FU, 0x12U, 0x41U, 0x5CU, 0x7BU, 0x66U,
    0xDDU, 0xC0U, 0xE7U, 0xFAU, 0xA9U, 0xB4U, 0x93U, 0x8EU,
    0xF8U, 0xE5U, 0xC2U, 0xDFU, 0x8CU, 0x91U, 0xB6U, 0xABU,
    0x10U, 0x0DU, 0x2AU, 0x37U, 0x64U, 0x79U, 0x5EU, 0x43U,
    0xB2U, 0xAFU, 0x88U, 0x95U, 0xC6U, 0xDBU, 0xFCU, 0xE1U,
    0x5AU, 0x47U, 0x60U, 0x7DU, 0x2EU, 0x33U, 0x14U, 0x09U,
    0x7FU, 0x62U, 0x45U, 0x58U, 0x0BU, 0x16U, 0x31U, 0x2CU,
    0x97U, 0x8AU, 0xADU, 0xB0U, 0xE3U, 0xFEU, 0xD9U, 0xC4U
};

static uint8_t fs26_calcrc(uint8_t const* data, size_t size) {
    uint8_t crc;
    uint8_t tableidx;

    /* Set CRC token value */
    crc = FS26_CRC_INIT;

    for (size_t i = size; i > 0; i--) {
        tableidx = crc ^ data[i];
        crc      = FS26_CRC_TABLE[tableidx];
    }

    return crc;
}

static int fs26_spi_transceive(const struct spi_dt_spec* spi,
                               struct fs26_spi_tx_frame* tx_frame,
                               struct fs26_spi_rx_frame* rx_frame) {
    uint32_t tx_buf;
    uint32_t rx_buf;
    uint8_t  crc;
    int retval;

    struct spi_buf spi_tx_buf = {
        .buf = &tx_buf,
        .len = sizeof(tx_buf)
    };
    struct spi_buf spi_rx_buf = {
        .buf = &rx_buf,
        .len = sizeof(rx_buf)
    };
    struct spi_buf_set spi_tx_set = {
        .buffers = &spi_tx_buf,
        .count = 1U
    };
    struct spi_buf_set spi_rx_set = {
        .buffers = &spi_rx_buf,
        .count = 1U
    };

    /* Create frame to Tx, always for Fail Safe */
    tx_buf = (uint32_t)(FS26_SET_REG_ADDR(tx_frame->addr) |
                        FS26_SET_DATA(tx_frame->data)     |
                        (tx_frame->write ? FS26_RW : 0));

    crc = fs26_calcrc((uint8_t*)&tx_buf, sizeof(tx_buf) - 1);
    tx_buf |= (uint32_t)FS26_SET_CRC(crc);

    #if defined(SWAP_ENDIANNESS)
    tx_buf = __builtin_bswap32(tx_buf);
    #endif

    retval = spi_transceive_dt(spi, &spi_tx_set, &spi_rx_set);
    if (retval) {
        goto error;
    }

    #if defined(SWAP_ENDIANNESS)
    rx_buf = __builtin_bswap32(rx_buf);
    #endif

    /* Verify CRC of Rx frame */
    crc = fs26_calcrc((uint8_t*)&rx_buf, sizeof(rx_buf) - 1);
    if (crc != ((uint8_t)FS26_GET_CRC(rx_buf))) {
        LOG_ERR("Rx invalid CRC");
        retval = -EIO;
        goto error;
    }

    if (rx_frame) {
        rx_frame->status.raw = (uint8_t)FS26_GET_DEV_STATUS(rx_buf);
        rx_frame->data = (uint16_t)FS26_GET_DATA(rx_buf);
    }

error :

    return retval;
}

/**
 * @brief Get value of register with address @p addr
 *
 * @param spi SPI specs for interacting with the device
 * @param addr Register address
 * @param rx_frame SPI frame containing read data and device status flags
 *
 * @return 0 on success, error code otherwise
 */
static int fs26_getreg(const struct spi_dt_spec* spi, uint8_t addr,
                       struct fs26_spi_rx_frame* rx_frame) {
    struct fs26_spi_tx_frame tx_frame = {
        .addr = addr,
        .write = 0,
        .data = 0
    };

    return fs26_spi_transceive(spi, &tx_frame, rx_frame);
}

/**
 * @brief Set @p regval value in register with address @p addr
 *
 * @param spi SPI specs for interacting with the device
 * @param addr Register address
 * @param regval Register value to set
 *
 * @return 0 on success, error code otherwise
 */
static int fs26_setreg(const struct spi_dt_spec* spi, uint8_t addr, uint16_t regval) {
    struct fs26_spi_tx_frame tx_frame = {
        .addr = addr,
        .write = true,
        .data = regval
    };

    return fs26_spi_transceive(spi, &tx_frame, NULL);
}

/**
 * @brief Calculate watchdog answer based on received token
 *
 * @return answer value to write to FS_WD_ANSWER
 */
static inline uint16_t fs26_wd_compute_answer(uint16_t token) {
    uint32_t tmp = token;

    tmp *= 4U;
    tmp += 6U;
    tmp -= 4U;
    tmp = ~tmp;
    tmp /= 4U;

    return (uint16_t)tmp;
}

/**
 * @brief Refresh the watchdog and verify the refresh was good.
 *
 * @return 0 on success, error code otherwise
 */
static int fs26_wd_refresh(const struct device* dev) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct wdt_nxp_fs26_data* data = dev->data;
    int retval = 0;
    int key;
    uint16_t answer;
    struct fs26_spi_rx_frame rx_frame;

    if (config->wd_type == FS26_WD_SIMPLE) {
        if (fs26_setreg(&config->spi, FS26_FS_WD_ANSWER, data->token) == 0) {
            LOG_ERR("Failed to write answer");
            retval = -EIO;
        }
    }
    else if (config->wd_type == FS26_WD_CHALLENGER) {
        key = irq_lock();

        /* Read challenge token generated by the device */
        if (fs26_getreg(&config->spi, FS26_FS_WD_TOKEN, &rx_frame)) {
            LOG_ERR("Failed to obtain watchdog token");
            retval = -EIO;
        }
        else {
            data->token = rx_frame.data;
            LOG_DBG("Watchdog token is %x", data->token);

            answer = fs26_wd_compute_answer(data->token);
            if (fs26_setreg(&config->spi, FS26_FS_WD_ANSWER, answer)) {
                LOG_ERR("Failed to write answer");
                retval = -EIO;
            }
        }

        irq_unlock(key);
    }
    else {
        retval = -EINVAL;
    }

    /* Check if watchdog refresh was successful */
    if (!retval) {
        if (!fs26_getreg(&config->spi, FS26_FS_GRL_FLAGS, &rx_frame)) {
            if ((rx_frame.data & FS_WD_G_MASK) == FS_WD_G) {
                if (!fs26_getreg(&config->spi, FS26_FS_DIAG_SAFETY1, &rx_frame)) {
                    LOG_ERR("Bad watchdog refresh, %s",
                            BAD_WD_REFRESH_ERROR_STRING(rx_frame.data));
                }
                retval = -EIO;
            }
            else {
                LOG_DBG("Refreshed the watchdog");
            }
        }
    }

    return retval;
}

/**
 * @brief Wait for state machine to be at in INIT_FS state
 *
 * @return 0 on success, -ETIMEDOUT if timedout
 */
static int fs26_poll_for_init_fs_state(const struct device* dev) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct fs26_spi_rx_frame rx_frame;
    uint32_t regval = 0;
    int64_t timeout;
    int64_t now;

    timeout = k_uptime_get() + FS26_INIT_FS_TIMEOUT_MS;

    do {
        if (!fs26_getreg(&config->spi, FS26_FS_STATES, &rx_frame)) {
            regval = rx_frame.data;
        }
        k_sleep(K_MSEC(1));
        now = k_uptime_get();
    } while ((now < timeout) && (regval & FS_STATES_MASK) != FS_STATES_INIT_FS);

    if (now >= timeout) {
        LOG_ERR("Timedout waiting for INIT_FS state");
        return -ETIMEDOUT;
    }

    return 0;
}

/**
 * @brief Go to INIT_FS state from any FS state after INIT_FS
 *
 * After INIT_FS closure, it is possible to come back to INIT_FS with the
 * GOTO_INIT bit in FS_SAFE_IOS_1 register from any FS state after INIT_FS.
 *
 * @return 0 on success, error code otherwise
 */
static int fs26_goto_init_fs_state(const struct device* dev) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct fs26_spi_rx_frame rx_frame;
    uint32_t current_state;
    int retval = -EIO;

    if (!fs26_getreg(&config->spi, FS26_FS_STATES, &rx_frame)) {
        current_state = rx_frame.data & FS_STATES_MASK;
        if (current_state < FS_STATES_INIT_FS) {
            LOG_ERR("Cannot go to INIT_FS from current state %x", current_state);
            retval = -EIO;
        }
        else if (current_state == FS_STATES_INIT_FS) {
            retval = 0;
        }
        else {
            fs26_setreg(&config->spi, FS26_FS_SAFE_IOS_1, (uint32_t)FS_GOTO_INIT);
            retval = fs26_poll_for_init_fs_state(dev);
        }
    }

    return retval;
}

/**
 * @brief Close INIT_FS phase with a (good) watchdog refresh.
 *
 * @return 0 on success, error code otherwise
 */
static inline int fs26_exit_init_fs_state(const struct device* dev) {
    return fs26_wd_refresh(dev);
}

static int wdt_nxp_fs26_feed(const struct device* dev, int channel_id) {
    struct wdt_nxp_fs26_data* data = dev->data;

    if (channel_id != 0) {
        LOG_ERR("Invalid channel ID");
        return -EINVAL;
    }

    if (!data->timeout_installed) {
        LOG_ERR("No timeout installed");
        return -EINVAL;
    }

    return fs26_wd_refresh(dev);
}

static int wdt_nxp_fs26_setup(const struct device* dev, uint8_t options) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct wdt_nxp_fs26_data* data = dev->data;
    uint32_t regval;

    if (!data->timeout_installed) {
        LOG_ERR("No timeout installed");
        return -EINVAL;
    }

    if ((options & WDT_OPT_PAUSE_IN_SLEEP) || (options & WDT_OPT_PAUSE_HALTED_BY_DBG)) {
        return -ENOTSUP;
    }

    /*
     * Apply fail-safe reaction configuration on RSTB and/or the safety output(s),
     * configurable during the initialization phase.
     */
    if (fs26_goto_init_fs_state(dev)) {
        LOG_ERR("Failed to go to INIT_FS");
        return -EIO;
    }

    regval = WD_ERR_LIMIT(CONFIG_WDT_NXP_FS26_ERROR_COUNTER_LIMIT)   |
             WD_RFR_LIMIT(CONFIG_WDT_NXP_FS26_REFRESH_COUNTER_LIMIT) |
             ((data->fs_reaction << WD_FS_REACTION_SHIFT) & WD_FS_REACTION_MASK);

    fs26_setreg(&config->spi, FS26_FS_I_WD_CFG, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_WD_CFG, ~regval);

    /* Apply watchdog window configuration, configurable during any FS state */
    regval = ((data->window_period << WDW_PERIOD_SHIFT) & WDW_PERIOD_MASK) |
             ((data->window_duty_cycle << WDW_DC_SHIFT) & WDW_DC_MASK)     |
             WDW_RECOVERY_DISABLE;

    fs26_setreg(&config->spi, FS26_FS_WDW_DURATION, regval);
    fs26_setreg(&config->spi, FS26_FS_NOT_WDW_DURATION, ~regval);

    /*
     * The new watchdog window is effective after the next watchdog refresh,
     * so feed the watchdog once to make it effective after exiting this
     * function. Also it's required to close init phase.
     */
    if (fs26_exit_init_fs_state(dev)) {
        LOG_ERR("Failed to close INIT_FS");
        return -EIO;
    }

    return 0;
}

static int wdt_nxp_fs26_install_timeout(const struct device* dev,
                                        const struct wdt_timeout_cfg* cfg) {
    struct wdt_nxp_fs26_data* data = dev->data;
    uint32_t window_min;
    uint8_t i;

    if (data->timeout_installed) {
        LOG_ERR("No more timeouts can be installed");
        return -ENOMEM;
    }

    if ((cfg->window.max == 0) || (cfg->window.max > 1024) ||
        (cfg->window.max <= cfg->window.min)) {
        LOG_ERR("Invalid timeout value");
        return -EINVAL;
    }

    /* Find nearest period value (rounded up) */
    for (i = 0; i < ARRAY_SIZE(fs26_period_values); i++) {
        if (fs26_period_values[i] >= cfg->window.max) {
            break;
        }
    }
    data->window_period = i;
    LOG_DBG("window.max requested %d ms, using %d ms",
            cfg->window.max, fs26_period_values[data->window_period]);

    /*
     * Find nearest duty cycle value based on new period, that results in a
     * window's minimum near the requested (rounded up)
     */
    for (i = 0; i < ARRAY_SIZE(fs26_dc_closed_values); i++) {
        window_min = (uint32_t)(fs26_dc_closed_values[i] *
                                fs26_period_values[data->window_period]);
        if (window_min >= cfg->window.min) {
            break;
        }
    }

    if (i >= ARRAY_SIZE(fs26_dc_closed_values)) {
        LOG_ERR("Watchdog opened window too small");
        return -EINVAL;
    }
    data->window_duty_cycle = i;

    LOG_DBG("window.min requested %d ms, using %d ms (%.2f%%)",
            cfg->window.min, window_min,
            fs26_dc_closed_values[data->window_duty_cycle] * 100);

    /* Fail-safe reaction configuration */
    switch (cfg->flags) {
        case WDT_FLAG_RESET_SOC :
            __fallthrough;
        case WDT_FLAG_RESET_CPU_CORE :
            data->fs_reaction = WD_FS_REACTION_RSTB_FS0B >> WD_FS_REACTION_SHIFT;
            LOG_DBG("Configuring reset mode");
            break;
        case WDT_FLAG_RESET_NONE :
            data->fs_reaction = WD_FS_REACTION_NO_ACTION >> WD_FS_REACTION_SHIFT;
            LOG_DBG("Configuring non-reset mode");
            break;
        default :
            LOG_ERR("Unsupported watchdog configuration flag");
            return -EINVAL;
    }

    data->callback = cfg->callback;
    data->timeout_installed = true;

    /* Always return channel ID equal to 0 */
    return 0;
}

static int wdt_nxp_fs26_disable(const struct device* dev) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct wdt_nxp_fs26_data* data = dev->data;
    struct fs26_spi_rx_frame rx_frame;
    uint32_t regval;

    if (fs26_getreg(&config->spi, FS26_FS_WDW_DURATION, &rx_frame)) {
        return -EIO;
    }
    if ((rx_frame.data & WDW_PERIOD_MASK) == WDW_PERIOD_DISABLE) {
        LOG_ERR("Watchdog already disabled");
        return -EFAULT;
    }

    /* The watchdog window can be disabled only during the initialization phase */
    if (fs26_goto_init_fs_state(dev)) {
        LOG_ERR("Failed to go to INIT_FS");
        return -EIO;
    }

    regval = WDW_PERIOD_DISABLE | WDW_RECOVERY_DISABLE;
    fs26_setreg(&config->spi, FS26_FS_WDW_DURATION, regval);
    fs26_setreg(&config->spi, FS26_FS_NOT_WDW_DURATION, ~regval);

    /* The watchdog disabling is effective when the initialization phase is closed */
    if (fs26_exit_init_fs_state(dev)) {
        LOG_ERR("Failed to close INIT_FS");
        return -EIO;
    }

    LOG_DBG("Watchdog disabled");
    data->timeout_installed = false;

    return 0;
}

static void wdt_nxp_fs26_int_thread(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device* dev = p1;
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct wdt_nxp_fs26_data* data = dev->data;
    struct fs26_spi_rx_frame rx_frame;
    uint32_t regval;

    while (true) {
        k_sem_take(&data->int_sem, K_FOREVER);

        if ((!fs26_getreg(&config->spi, FS26_FS_GRL_FLAGS, &rx_frame)) &&
            ((rx_frame.data & FS_WD_G_MASK) == FS_WD_G)) {

            if ((!fs26_getreg(&config->spi, FS26_FS_DIAG_SAFETY1, &rx_frame)) &&
                (rx_frame.data & BAD_WD_TIMING)) {

                /* Clear flag */
                regval = BAD_WD_TIMING;
                fs26_setreg(&config->spi, FS26_FS_DIAG_SAFETY1, regval);

                /* Invoke user callback */
                if (data->callback && data->timeout_installed) {
                    data->callback(dev, 0);
                }
            }
        }
    }
}

static void wdt_nxp_fs26_int_callback(const struct device* dev,
                                      struct gpio_callback* cb,
                                      uint32_t pins) {
    struct wdt_nxp_fs26_data* data = CONTAINER_OF(cb, struct wdt_nxp_fs26_data,
                                                  int_gpio_cb);

    ARG_UNUSED(dev);
    ARG_UNUSED(pins);

    k_sem_give(&data->int_sem);
}

static int wdt_nxp_fs26_init(const struct device* dev) {
    const struct wdt_nxp_fs26_config* config = dev->config;
    struct wdt_nxp_fs26_data* data = dev->data;
    struct fs26_spi_rx_frame rx_frame;
    uint32_t regval;

    /* Validate bus is ready */
    if (!spi_is_ready_dt(&config->spi)) {
        return -ENODEV;
    }

    k_sem_init(&data->int_sem, 0, 1);

    /* Configure GPIO used for INTB signal */
    if (!gpio_is_ready_dt(&config->int_gpio)) {
        LOG_ERR("GPIO port %s not ready", config->int_gpio.port->name);
        return -ENODEV;
    }

    if (gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT)) {
        LOG_ERR("Unable to configure GPIO pin %u", config->int_gpio.pin);
        return -EIO;
    }

    gpio_init_callback(&(data->int_gpio_cb), wdt_nxp_fs26_int_callback,
                       BIT(config->int_gpio.pin));

    if (gpio_add_callback(config->int_gpio.port, &(data->int_gpio_cb))) {
        return -EINVAL;
    }

    if (gpio_pin_interrupt_configure_dt(&config->int_gpio,
                                        GPIO_INT_EDGE_FALLING)) {
        return -EINVAL;
    }

    k_thread_create(&data->int_thread, data->int_thread_stack,
                    CONFIG_WDT_NXP_FS26_INT_THREAD_STACK_SIZE,
                    wdt_nxp_fs26_int_thread,
                    (void*)dev, NULL, NULL,
                    K_PRIO_COOP(CONFIG_WDT_NXP_FS26_INT_THREAD_PRIO),
                    0, K_NO_WAIT);

    /* Verify FS BIST before proceeding */
    if (fs26_getreg(&config->spi, FS26_FS_DIAG_SAFETY1, &rx_frame)) {
        return -EIO;
    }

    if ((rx_frame.data & (ABIST1_PASS_MASK | LBIST_STATUS_MASK)) !=
        (ABIST1_PASS | LBIST_STATUS_OK)) {

        LOG_ERR("BIST failed 0x%x", rx_frame.data);
        return -EIO;
    }

    /* Get FS state machine state */
    if (fs26_getreg(&config->spi, FS26_FS_STATES, &rx_frame)) {
        return -EIO;
    }

    /* Verify if in DEBUG mode */
    if ((rx_frame.data & DBG_MODE_MASK) == DBG_MODE) {
        if (IS_ENABLED(CONFIG_WDT_NXP_FS26_EXIT_DEBUG_MODE)) {
            LOG_DBG("Exiting DEBUG mode");
            regval = rx_frame.data | EXIT_DBG_MODE;
            fs26_setreg(&config->spi, FS26_FS_STATES, regval);
        }
        else {
            LOG_ERR("In DEBUG mode, watchdog is disabled");
            return -EIO;
        }
    }

    /* Go to INIT_FS state, if not already there */
    if (fs26_goto_init_fs_state(dev)) {
        LOG_ERR("Failed to go to INIT_FS");
        return -EIO;
    }

    /* Clear pending FS diagnostic flags before initializing */
    regval = BAD_WD_DATA | BAD_WD_TIMING | ABIST2_PASS | ABIST2_DONE |
             SPI_FS_CLK  | SPI_FS_REQ    | SPI_FS_CRC  | FS_OSC_DRIFT;
    fs26_setreg(&config->spi, FS26_FS_DIAG_SAFETY1, regval);

    /*
     * Perform the following sequence for all INIT_FS registers (FS_I_xxxx)
     * - Write the desired data in the FS_I_Register_A (data)
     * - Write the opposite in the FS_I_NOT_Register_A (~data)
     */

    /* OVUV_SAFE_REACTION1 */
    regval = VMON_PRE_OV_FS_REACTION_NO_EFFECT  |
             VMON_PRE_UV_FS_REACTION_NO_EFFECT  |
             VMON_CORE_OV_FS_REACTION_NO_EFFECT |
             VMON_CORE_UV_FS_REACTION_NO_EFFECT |
             VMON_LDO1_OV_FS_REACTION_NO_EFFECT |
             VMON_LDO1_UV_FS_REACTION_NO_EFFECT |
             VMON_LDO2_OV_FS_REACTION_NO_EFFECT |
             VMON_LDO2_UV_FS_REACTION_NO_EFFECT;

    fs26_setreg(&config->spi, FS26_FS_I_OVUV_SAFE_REACTION1, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_OVUV_SAFE_REACTION1, ~regval);

    /* OVUV_SAFE_REACTION2 */
    regval = VMON_EXT_OV_FS_REACTION_NO_EFFECT  |
             VMON_EXT_UV_FS_REACTION_NO_EFFECT  |
             VMON_REF_OV_FS_REACTION_NO_EFFECT  |
             VMON_REF_UV_FS_REACTION_NO_EFFECT  |
             VMON_TRK2_OV_FS_REACTION_NO_EFFECT |
             VMON_TRK2_UV_FS_REACTION_NO_EFFECT |
             VMON_TRK1_OV_FS_REACTION_NO_EFFECT |
             VMON_TRK1_UV_FS_REACTION_NO_EFFECT;

    fs26_setreg(&config->spi, FS26_FS_I_OVUV_SAFE_REACTION2, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_OVUV_SAFE_REACTION2, ~regval);

    /* FS_I_SAFE_INPUTS */
    regval = FCCU_CFG_NO_MONITORING | ERRMON_ACK_TIME_32MS;

    fs26_setreg(&config->spi, FS26_FS_I_SAFE_INPUTS, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_SAFE_INPUTS, ~regval);

    /* FS_I_FSSM */
    regval = FLT_ERR_REACTION_NO_EFFECT | CLK_MON_DIS | DIS8S;

    fs26_setreg(&config->spi, FS26_FS_I_FSSM, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_FSSM, ~regval);

    /* FS_I_WD_CFG */
    regval = WD_ERR_LIMIT(CONFIG_WDT_NXP_FS26_ERROR_COUNTER_LIMIT)   |
             WD_RFR_LIMIT(CONFIG_WDT_NXP_FS26_REFRESH_COUNTER_LIMIT) |
             WD_FS_REACTION_NO_ACTION;

    fs26_setreg(&config->spi, FS26_FS_I_WD_CFG, regval);
    fs26_setreg(&config->spi, FS26_FS_I_NOT_WD_CFG, ~regval);

    /* FS_WDW_DURATION */
    /* Watchdog always disabled at boot */
    regval = WDW_PERIOD_DISABLE | WDW_RECOVERY_DISABLE;

    fs26_setreg(&config->spi, FS26_FS_WDW_DURATION, regval);
    fs26_setreg(&config->spi, FS26_FS_NOT_WDW_DURATION, ~regval);

    /* Set watchdog seed if not using the default */
    if (data->token != FS26_FS_WD_TOKEN_DEFAULT) {
        LOG_DBG("Set seed to %x", data->token);
        fs26_setreg(&config->spi, FS26_FS_WD_TOKEN, data->token);
    }

    /* Mask all Fail-Safe interrupt sources except for watchdog bad refresh */
    regval = ~BAD_WD_M;
    fs26_setreg(&config->spi, FS26_FS_INTB_MASK, regval);

    /* Mask all main interrupt souces */
    regval = 0xffff;
    fs26_setreg(&config->spi, FS26_M_TSD_MSK, regval);
    fs26_setreg(&config->spi, FS26_M_REG_MSK, regval);
    fs26_setreg(&config->spi, FS26_M_VSUP_MSK, regval);
    fs26_setreg(&config->spi, FS26_M_WIO_MSK, regval);
    fs26_setreg(&config->spi, FS26_M_COM_MSK, regval);

    /* INIT_FS must be closed before the 256 ms timeout */
    if (fs26_exit_init_fs_state(dev)) {
        LOG_ERR("Failed to close INIT_FS");
        return -EIO;
    }

    /* After INIT_FS is completed, check for data corruption in init registers */
    if (!fs26_getreg(&config->spi, FS26_FS_STATES, &rx_frame)) {
        if ((rx_frame.data & REG_CORRUPT_MASK) == REG_CORRUPT) {
            LOG_ERR("Data content corruption detected in init registers");
            return -EIO;
        }
    }

    return 0;
}

static DEVICE_API(wdt, wdt_nxp_fs26_api) = {
    .setup = wdt_nxp_fs26_setup,
    .disable = wdt_nxp_fs26_disable,
    .install_timeout = wdt_nxp_fs26_install_timeout,
    .feed = wdt_nxp_fs26_feed,
};

#define FS26_WDT_DEVICE_INIT(n)                                 \
    COND_CODE_1(DT_INST_ENUM_IDX(n, type),                      \
                (BUILD_ASSERT(CONFIG_WDT_NXP_FS26_SEED != 0x0,  \
                              "Seed value 0x0000 is not allowed");), \
                (BUILD_ASSERT((CONFIG_WDT_NXP_FS26_SEED != 0x0) &&   \
                              (CONFIG_WDT_NXP_FS26_SEED != 0xFFFF),  \
                              "Seed values 0x0000 and 0xFFFF are not allowed");)) \
                                                                \
    static struct wdt_nxp_fs26_data wdt_nxp_fs26_data_##n = {   \
        .token = CONFIG_WDT_NXP_FS26_SEED,                      \
    };                                                          \
                                                                \
    static const struct wdt_nxp_fs26_config wdt_nxp_fs26_config_##n = { \
        .spi = SPI_DT_SPEC_INST_GET(n,                          \
                                    SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(32)), \
        .wd_type  = Z_CONCAT(FS26_WD_, DT_INST_STRING_UPPER_TOKEN(n, type)), \
        .int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),        \
    };                                                          \
                                                                \
    DEVICE_DT_INST_DEFINE(n,                                    \
                          wdt_nxp_fs26_init,                    \
                          NULL,                                 \
                          &wdt_nxp_fs26_data_##n,               \
                          &wdt_nxp_fs26_config_##n,             \
                          POST_KERNEL,                          \
                          CONFIG_WDT_NXP_FS26_INIT_PRIORITY,    \
                          &wdt_nxp_fs26_api);

DT_INST_FOREACH_STATUS_OKAY(FS26_WDT_DEVICE_INIT)

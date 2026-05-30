/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan8710ai

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_lan8710ai, CONFIG_PHY_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/gpio.h>

#include "phy_mii.h"

#define LAN8710_PHY_ID1      0x0007U
#define LAN8710_PHY_ID2      0xC0F0U
#define LAN8710_PHY_ID2_MASK 0xFFF0U

/* Maximum retries waiting for BMCR soft-reset self-clear (10 ms each) */
#define LAN8710_RESET_RETRIES 50
#define MII_AUTONEG_POLL_INTERVAL_MS 100

struct phy_lan8710ai_config {
    const struct device* mdio_dev;
    struct gpio_dt_spec  reset_gpio;        /* zeroed if not in DT */
    enum phy_link_speed  default_speeds;
    uint8_t phy_addr;
};

struct phy_lan8710ai_data {
    const struct device* dev;               /* self-pointer for work handler */
    phy_callback_t cb;
    void* cb_data;
    struct phy_link_state state;
    struct k_mutex lock;
    struct k_work_delayable monitor_work;
    bool autoneg_in_progress;
};

/* -------------------------------------------------------------------
 * Low-level MDIO read/write helpers
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_read(const struct device* dev, uint16_t reg_addr, uint32_t* data) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    uint16_t val;
    int ret;

    ret = mdio_read(cfg->mdio_dev, cfg->phy_addr, (uint8_t)reg_addr, &val);
    if (ret == 0) {
        *data = val;
    }

    return (ret);
}

static int phy_lan8710ai_write(const struct device* dev, uint16_t reg_addr, uint32_t data) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    int ret;

    ret = mdio_write(cfg->mdio_dev, cfg->phy_addr, (uint8_t)reg_addr, (uint16_t)data);

    return (ret);
}

/* -------------------------------------------------------------------
 * Invoke registered MAC callback with current state.
 * Must be called while holding data->lock.
 * ------------------------------------------------------------------- */
static void phy_lan8710ai_invoke_cb(const struct device* dev) {
    struct phy_lan8710ai_data* data = dev->data;

    if (data->cb) {
        data->cb(dev, &data->state, data->cb_data);
    }
}

/* -------------------------------------------------------------------
 * Link state update — reads PHY registers and fires callback on change.
 * Called from monitor_work (thread context); lock must NOT be held on entry.
 * ------------------------------------------------------------------- */
static void phy_lan8710ai_update_link_state(const struct device* dev) {
    struct phy_lan8710ai_data* data = dev->data;
    struct phy_link_state new_state = {0};
    uint32_t bmsr;
    uint32_t anlpar;
    bool link_up;

    /* Read BMSR twice: link-status bit is latching-low (cleared on read) */
    phy_lan8710ai_read(dev, MII_BMSR, &bmsr);
    phy_lan8710ai_read(dev, MII_BMSR, &bmsr);
    link_up = (bmsr & MII_BMSR_LINK_STATUS) != 0U;

    if (!link_up) {
        new_state.is_up = false;
        goto check_change;
    }

    if (data->autoneg_in_progress) {
        if (!(bmsr & MII_BMSR_AUTONEG_COMPLETE)) {
            return; /* still negotiating — no state change yet */
        }

        phy_lan8710ai_read(dev, MII_ANLPAR, &anlpar);
        if (anlpar & MII_ADVERTISE_100_FULL) {
            new_state.speed = LINK_FULL_100BASE;
        }
        else if (anlpar & MII_ADVERTISE_100_HALF) {
            new_state.speed = LINK_HALF_100BASE;
        }
        else if (anlpar & MII_ADVERTISE_10_FULL) {
            new_state.speed = LINK_FULL_10BASE;
        }
        else {
            new_state.speed = LINK_HALF_10BASE;
        }

        new_state.is_up = true;
        data->autoneg_in_progress = false;
    }
    else {
        /* Link already up — preserve negotiated speed */
        new_state = data->state;
        new_state.is_up = true;
    }

check_change :
    k_mutex_lock(&data->lock, K_FOREVER);
    if (memcmp(&new_state, &data->state, sizeof(new_state)) != 0) {
        data->state = new_state;
        phy_lan8710ai_invoke_cb(dev);
    }

    k_mutex_unlock(&data->lock);
}

/* -------------------------------------------------------------------
 * Delayed work: periodic link monitor
 * ------------------------------------------------------------------- */
static void phy_lan8710ai_monitor_work_fn(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct phy_lan8710ai_data* data =
        CONTAINER_OF(dwork, struct phy_lan8710ai_data, monitor_work);
    const struct device* dev = data->dev;

    phy_lan8710ai_update_link_state(dev);

    /* Poll faster during autoneg, slower once stable */
    int next_ms = data->autoneg_in_progress ? 100 : 1000;

    k_work_reschedule(&data->monitor_work, K_MSEC(next_ms));
}

/* -------------------------------------------------------------------
 * PHY reset: GPIO (if wired) + BMCR soft-reset
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_reset(const struct device* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    int retries;

    if (cfg->reset_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return (-ENODEV);
        }

        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_sleep(K_MSEC(10));
    }

    phy_lan8710ai_write(dev, MII_BMCR, MII_BMCR_RESET);

    for (retries = 0; retries < LAN8710_RESET_RETRIES; retries++) {
        uint32_t bmcr;

        k_msleep(10);
        if ((phy_lan8710ai_read(dev, MII_BMCR, &bmcr) == 0) &&
            ((bmcr & MII_BMCR_RESET) == 0)) {
            return (0);
        }
    }

    LOG_ERR("PHY soft reset timed out");
    return (-ETIMEDOUT);
}

/* -------------------------------------------------------------------
 * PHY API implementations
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_cfg_link(const struct device* dev,
                                  enum phy_link_speed adv_speeds,
                                  enum phy_cfg_link_flag flags) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    int ret;

    if (cfg->mdio_dev == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
        ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
        if (ret >= 0) {
            data->autoneg_in_progress = false;
            k_work_reschedule(&data->monitor_work, K_NO_WAIT);
        }
    }
    else {
        ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, false);
        if (ret >= 0) {
            LOG_DBG("PHY (%d) Starting MII PHY auto-negotiate sequence", cfg->phy_addr);
            data->autoneg_in_progress = true;
            k_work_reschedule(&data->monitor_work, K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
        }
    }

    if (ret == -EALREADY) {
        LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
    }

    k_mutex_unlock(&data->lock);

    return (ret);
}

static int phy_lan8710ai_get_link(const struct device* dev,
                                  struct phy_link_state* state) {
    struct phy_lan8710ai_data* data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    *state = data->state;
    k_mutex_unlock(&data->lock);

    return (0);
}

static int phy_lan8710ai_link_cb_set(const struct device* dev,
                                     phy_callback_t cb, void* user_data) {
    struct phy_lan8710ai_data* data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    data->cb = cb;
    data->cb_data = user_data; /* caller's opaque pointer (e.g. MAC dev) */
    k_mutex_unlock(&data->lock);

    return (0);
}

static int phy_lan8710ai_init(const struct device* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    uint32_t id1;
    uint32_t id2;
    bool is_ready;
    int ret;

    data->state.is_up = false;
    k_mutex_init(&data->lock);
    data->dev = dev;

    is_ready = device_is_ready(cfg->mdio_dev);
    if (is_ready == false) {
        LOG_ERR_DEVICE_NOT_READY(cfg->mdio_dev);
        return (-ENODEV);
    }

    ret = phy_lan8710ai_reset(dev);
    if (ret < 0) {
        LOG_ERR("Failed to reset PHY (%d): %d", cfg->phy_addr, ret);
        return (ret);
    }

    /* Verify PHY ID */
    phy_lan8710ai_read(dev, MII_PHYID1R, &id1);
    phy_lan8710ai_read(dev, MII_PHYID2R, &id2);

    if ((id1 != LAN8710_PHY_ID1) || ((id2 & LAN8710_PHY_ID2_MASK) != LAN8710_PHY_ID2)) {
        LOG_ERR("No PHY found at address %d (ID 0x%04x:0x%04x)", cfg->phy_addr, id1, id2);
        return (-EINVAL);
    }

    LOG_INF("PHY (%d) ID 0x%04x:0x%04x", cfg->phy_addr, id1, id2);

    k_work_init_delayable(&data->monitor_work, phy_lan8710ai_monitor_work_fn);

    /* Advertise default speeds */
    ret = phy_lan8710ai_cfg_link(dev, cfg->default_speeds, 0);
    if (ret < 0) {
        LOG_ERR("Failed to configure link (%d)", ret);
        return (ret);
    }

    /* Schedule the monitor work, if not already scheduled by phy_lan8710ai_cfg_link(). */
    k_work_schedule(&data->monitor_work, K_NO_WAIT);

    return (0);
}

/* -------------------------------------------------------------------
 * Driver API and instantiation
 * ------------------------------------------------------------------- */
static DEVICE_API(ethphy, phy_lan8710ai_driver_api) = {
    .cfg_link    = phy_lan8710ai_cfg_link,
    .get_link    = phy_lan8710ai_get_link,
    .link_cb_set = phy_lan8710ai_link_cb_set,
    .read        = phy_lan8710ai_read,
    .write       = phy_lan8710ai_write
};

#define LAN8710AI_RESET_GPIO_OR_ZERO(n)                         \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(n, reset_gpios),           \
               (GPIO_DT_SPEC_INST_GET(n, reset_gpios)))

#define LAN8710AI_INIT(n)                                       \
    static struct phy_lan8710ai_data phy_lan8710ai_data_##n;    \
    static const struct phy_lan8710ai_config phy_lan8710ai_cfg_##n = { \
        .phy_addr       = DT_INST_REG_ADDR(n),                  \
        .mdio_dev       = DEVICE_DT_GET(DT_INST_BUS(n)),        \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),  \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, reset_gpios),       \
                   (.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios), )) \
    };                                                          \
    DEVICE_DT_INST_DEFINE(n, phy_lan8710ai_init, NULL,          \
                          &phy_lan8710ai_data_##n,              \
                          &phy_lan8710ai_cfg_##n, POST_KERNEL,  \
                          CONFIG_PHY_INIT_PRIORITY,             \
                          &phy_lan8710ai_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LAN8710AI_INIT)

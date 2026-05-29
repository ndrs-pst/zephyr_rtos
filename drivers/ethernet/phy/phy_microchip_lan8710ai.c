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

#define LAN8710_PHY_ID1      0x0007U
#define LAN8710_PHY_ID2      0xC0F0U
#define LAN8710_PHY_ID2_MASK 0xFFF0U

/* Maximum retries waiting for BMCR soft-reset self-clear (10 ms each) */
#define LAN8710_RESET_RETRIES 50

struct phy_lan8710ai_config {
    const struct device* mdio_dev;
    struct gpio_dt_spec  reset_gpio;        /* zeroed if not in DT */
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
        if (!(bmsr & MII_BMSR_AN_COMPLETE)) {
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
 * PHY API implementations
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_cfg_link(const struct device* dev,
                                  enum phy_link_speed adv_speeds,
                                  enum phy_cfg_link_flag flags) {
    ARG_UNUSED(flags);
    struct phy_lan8710ai_data* data = dev->data;
    uint32_t anar = MII_ADVERTISE_CSMA;
    uint32_t bmcr;

    /* Build ANAR from requested speeds */
    if (adv_speeds & LINK_FULL_100BASE) {
        anar |= MII_ADVERTISE_100_FULL;
    }

    if (adv_speeds & LINK_HALF_100BASE) {
        anar |= MII_ADVERTISE_100_HALF;
    }

    if (adv_speeds & LINK_FULL_10BASE) {
        anar |= MII_ADVERTISE_10_FULL;
    }

    if (adv_speeds & LINK_HALF_10BASE) {
        anar |= MII_ADVERTISE_10_HALF;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    phy_lan8710ai_write(dev, MII_ANAR, anar);

    phy_lan8710ai_read(dev, MII_BMCR, &bmcr);
    bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
    phy_lan8710ai_write(dev, MII_BMCR, bmcr);

    data->autoneg_in_progress = true;
    data->state.is_up         = false;

    k_mutex_unlock(&data->lock);

    return (0);
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
    int retries;

    k_mutex_init(&data->lock);
    data->dev = dev; /* self-pointer for work handler — never changes */

    if (!device_is_ready(cfg->mdio_dev)) {
        LOG_ERR("MDIO device not ready");
        return (-ENODEV);
    }

    /* Hardware reset via GPIO if wired */
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

    /* Soft reset — poll until BMCR reset bit self-clears (up to 500 ms) */
    phy_lan8710ai_write(dev, MII_BMCR, MII_BMCR_RESET);
    for (retries = 0; retries < LAN8710_RESET_RETRIES; retries++) {
        uint32_t bmcr;

        k_msleep(10);
        if ((phy_lan8710ai_read(dev, MII_BMCR, &bmcr) == 0) &&
            ((bmcr & MII_BMCR_RESET) == 0)) {
            break;
        }
    }

    if (retries == LAN8710_RESET_RETRIES) {
        LOG_ERR("PHY soft reset timed out");
        return (-ETIMEDOUT);
    }

    /* Verify PHY ID */
    phy_lan8710ai_read(dev, MII_PHYSID1, &id1);
    phy_lan8710ai_read(dev, MII_PHYSID2, &id2);

    if ((id1 != LAN8710_PHY_ID1) || ((id2 & LAN8710_PHY_ID2_MASK) != LAN8710_PHY_ID2)) {
        LOG_ERR("Unsupported PHY ID: 0x%04x:0x%04x", id1, id2);
        return (-ENODEV);
    }

    LOG_DBG("LAN8710AI PHY ID match: 0x%04x:0x%04x", id1, id2);

    k_work_init_delayable(&data->monitor_work, phy_lan8710ai_monitor_work_fn);

    /* Start autoneg advertising all supported speeds */
    phy_lan8710ai_cfg_link(dev,
                           LINK_FULL_100BASE | LINK_HALF_100BASE |
                           LINK_FULL_10BASE  | LINK_HALF_10BASE, 0);

    k_work_reschedule(&data->monitor_work, K_MSEC(100));

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
        .phy_addr = DT_INST_REG_ADDR(n),                        \
        .mdio_dev = DEVICE_DT_GET(DT_INST_BUS(n)),              \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, reset_gpios),       \
                   (.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios), )) \
    };                                                          \
    DEVICE_DT_INST_DEFINE(n, phy_lan8710ai_init, NULL,          \
                          &phy_lan8710ai_data_##n,              \
                          &phy_lan8710ai_cfg_##n, POST_KERNEL,  \
                          CONFIG_PHY_INIT_PRIORITY,             \
                          &phy_lan8710ai_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LAN8710AI_INIT)

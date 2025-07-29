/*
 * Copyright (c) 2024 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan8742a

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME         phy_mc_lan8742a
#define LOG_LEVEL               CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "phy_mii.h"

#define PHY_MC_LAN8742A_OMSO_REG                0x16
#define PHY_MC_LAN8742A_OMSO_FACTORY_MODE_MASK  BIT(15)
#define PHY_MC_LAN8742A_OMSO_NAND_TREE_MASK     BIT(5)

#define PHY_MC_LAN8742A_CTRL2_REG               0x1F
#define PHY_MC_LAN8742A_CTRL2_REF_CLK_SEL       BIT(7)

#define PHY_MC_LAN8742A_RESET_HOLD_TIME

struct mc_lan8742a_config {
    uint8_t addr;
    struct device const* mdio_dev;
    enum phy_link_speed default_speeds;

    #if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_reset_gpio)
    struct gpio_dt_spec reset_gpio;
    #endif

    #if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_interrupt_gpio)
    struct gpio_dt_spec interrupt_gpio;
    #endif
};

struct mc_lan8742a_data {
    struct device const* dev;
    struct phy_link_state state;
    phy_callback_t cb;
    void* cb_data;
    struct k_mutex mutex;
    struct k_work_delayable phy_monitor_work;
};

static int phy_link_state_cmp(struct phy_link_state const* a_state, struct phy_link_state const* b_state) {
    int ret;

    if ((a_state->is_up == b_state->is_up) &&
        (a_state->speed == b_state->speed)) {
        ret = 0;
    }
    else {
        ret = 1;
    }

    return (ret);
}

static int phy_mc_lan8742a_read(const struct device* dev,
                                uint16_t reg_addr, uint32_t* data) {
    const struct mc_lan8742a_config* cfg = dev->config;
    uint16_t tmp16;
    int ret;

    ret = mdio_read(cfg->mdio_dev, cfg->addr, (uint8_t)reg_addr, &tmp16);
    if (ret == 0) {
        *data = tmp16;
    }

    return (ret);
}

static int phy_mc_lan8742a_write(const struct device* dev,
                                 uint16_t reg_addr, uint32_t data) {
    const struct mc_lan8742a_config* cfg = dev->config;
    int ret;

    ret = mdio_write(cfg->mdio_dev, cfg->addr, (uint8_t)reg_addr, (uint16_t)data);

    return (ret);
}

static int phy_mc_lan8742a_autonegotiate(const struct device* dev) {
    struct mc_lan8742a_config const* cfg = dev->config;
    int ret;
    uint32_t bmcr;
    uint32_t bmsr;
    uint16_t timeout = CONFIG_PHY_AUTONEG_TIMEOUT_MS / 100;

    /* Read control register to write back with autonegotiation bit */
    ret = phy_mc_lan8742a_read(dev, MII_BMCR, &bmcr);
    if (ret != 0) {
        LOG_ERR("Error reading phy (%d) basic control register", cfg->addr);
        return (ret);
    }

    /* (re)start autonegotiation */
    LOG_DBG("PHY (%d) is entering autonegotiation sequence", cfg->addr);
    bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
    bmcr &= ~MII_BMCR_ISOLATE;

    ret = phy_mc_lan8742a_write(dev, MII_BMCR, bmcr);
    if (ret != 0) {
        LOG_ERR("Error writing phy (%d) basic control register", cfg->addr);
        return (ret);
    }

    /* TODO change this to GPIO interrupt driven */
    do {
        if (timeout-- == 0) {
            LOG_DBG("PHY (%d) autonegotiation timed out", cfg->addr);
            /* The value -ETIMEDOUT can be returned by PHY read/write functions, so
             * return -ENETDOWN instead to distinguish link timeout from PHY timeout.
             */
            return (-ENETDOWN);
        }
        k_msleep(100);

        ret = phy_mc_lan8742a_read(dev, MII_BMSR, &bmsr);
        if (ret != 0) {
            LOG_ERR("Error reading phy (%d) basic status register", cfg->addr);
            return (ret);
        }
    } while (!(bmsr & MII_BMSR_AUTONEG_COMPLETE));

    LOG_DBG("PHY (%d) autonegotiation completed", cfg->addr);

    return (0);
}

static int phy_mc_lan8742a_get_link(const struct device* dev,
                                    struct phy_link_state* state) {
    struct mc_lan8742a_config const* cfg = dev->config;
    struct mc_lan8742a_data* ctx = dev->data;
    int ret;
    uint32_t bmsr;                          /* Basic Mode Status Register */
    uint32_t anar;                          /* Auto-Negotiation Advertisement Register */
    uint32_t anlpar;                        /* Auto-Negotiation Link Partner Ability Register */
    struct phy_link_state old_state = ctx->state;

    ret = k_mutex_lock(&ctx->mutex, K_FOREVER);
    if (ret != 0) {
        LOG_ERR("PHY mutex lock error");
        return (ret);
    }

    /* Read link state */
    ret = phy_mc_lan8742a_read(dev, MII_BMSR, &bmsr);
    if (ret != 0) {
        LOG_ERR("Error reading phy (%d) basic status register", cfg->addr);
        goto unlock;
    }

    state->is_up = (bmsr & MII_BMSR_LINK_STATUS) ? true : false;
    state->speed = LINK_UNDEFINED;
    if (state->is_up == false) {
        goto result;
    }

    /* Read currently configured advertising options */
    ret = phy_mc_lan8742a_read(dev, MII_ANAR, &anar);
    if (ret != 0) {
        LOG_ERR("Error reading phy (%d) advertising register", cfg->addr);
        goto unlock;
    }

    /* Read link partner capability */
    ret = phy_mc_lan8742a_read(dev, MII_ANLPAR, &anlpar);
    if (ret != 0) {
        LOG_ERR("Error reading phy (%d) link partner register", cfg->addr);
        goto unlock;
    }

    uint32_t mutual_capabilities = (anar & anlpar);

    if (mutual_capabilities & MII_ADVERTISE_100_FULL) {
        state->speed = LINK_FULL_100BASE;
    }
    else if (mutual_capabilities & MII_ADVERTISE_100_HALF) {
        state->speed = LINK_HALF_100BASE;
    }
    else if (mutual_capabilities & MII_ADVERTISE_10_FULL) {
        state->speed = LINK_FULL_10BASE;
    }
    else if (mutual_capabilities & MII_ADVERTISE_10_HALF) {
        state->speed = LINK_HALF_10BASE;
    }
    else {
        ret = -EIO;
    }

result :
    if (phy_link_state_cmp(&old_state, state) != 0) {
        LOG_DBG("PHY %d is %s", cfg->addr, state->is_up ? "up" : "down");
        if (state->is_up == true) {
            LOG_DBG("PHY (%d) Link speed %s Mb, %s duplex\n", cfg->addr,
                (PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
                PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
        }
    }

unlock :
    k_mutex_unlock(&ctx->mutex);

    return (ret);
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
static int phy_mc_lan8742a_static_cfg(const struct device* dev) {
    const struct mc_lan8742a_config* cfg = dev->config;
    uint32_t omso;
    int ret;

    ARG_UNUSED(cfg);

    /* Force normal operation in the case of factory mode */
    ret = phy_mc_lan8742a_read(dev, PHY_MC_LAN8742A_OMSO_REG, &omso);
    if (ret != 0) {
        return (ret);
    }

    omso &= ~PHY_MC_LAN8742A_OMSO_FACTORY_MODE_MASK &
            ~PHY_MC_LAN8742A_OMSO_NAND_TREE_MASK;

    ret = phy_mc_lan8742a_write(dev, PHY_MC_LAN8742A_OMSO_REG, omso);
    if (ret != 0) {
        return (ret);
    }

    return (0);
}

static int phy_mc_lan8742a_reset(struct device const* dev) {
    struct mc_lan8742a_data* ctx = dev->data;
    int ret;

    ret = k_mutex_lock(&ctx->mutex, K_FOREVER);
    if (ret != 0) {
        LOG_ERR("PHY mutex lock error");
        return (ret);
    }

    #if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_reset_gpio)
    struct mc_lan8742a_config const* cfg = dev->config;
    if (cfg->reset_gpio.port != NULL) {
        /* Start reset */
        ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
        if (ret == 0) {
            /* Wait for 1 ms */
            k_busy_wait(1 * USEC_PER_MSEC);

            /* Reset over */
            ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
        }

        k_mutex_unlock(&ctx->mutex);
        return (ret);
    }
    #endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_reset_gpio) */

    ret = phy_mc_lan8742a_write(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret == 0) {
        /* Wait for 1 ms  */
        k_busy_wait(1 * USEC_PER_MSEC);
    }

    k_mutex_unlock(&ctx->mutex);

    return (ret);
}

static int phy_mc_lan8742a_cfg_link(const struct device* dev, enum phy_link_speed speeds,
                                    enum phy_cfg_link_flag flags) {
    struct mc_lan8742a_config const* cfg = dev->config;
    struct mc_lan8742a_data* ctx = dev->data;
    struct phy_link_state state;
    int ret;

    if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0) {
        LOG_ERR("Disabling auto-negotiation is not supported by this driver");
        return (-ENOTSUP);
    }

    ret = k_mutex_lock(&ctx->mutex, K_FOREVER);
    if (ret != 0) {
        LOG_ERR("PHY mutex lock error");
        goto done;
    }

    /* We are going to reconfigure the phy, don't need to monitor until done */
    k_work_cancel_delayable(&ctx->phy_monitor_work);

    /* Reset PHY */
    ret = phy_mc_lan8742a_reset(dev);
    if (ret != 0) {
        goto done;
    }

    /* DT configurations */
    ret = phy_mc_lan8742a_static_cfg(dev);
    if (ret != 0) {
        goto done;
    }

    ret = phy_mii_set_anar_reg(dev, speeds);
    if ((ret < 0) && (ret != -EALREADY)) {
        LOG_ERR("Error setting ANAR register for phy (%d)", cfg->addr);
        goto done;
    }

    /* (re)do autonegotiation */
    ret = phy_mc_lan8742a_autonegotiate(dev);
    if ((ret != 0) && (ret != -ENETDOWN)) {
        LOG_ERR("Error in autonegotiation");
        goto done;
    }

    /* Get link status */
    ret = phy_mc_lan8742a_get_link(dev, &state);
    if ((ret == 0) &&
        (phy_link_state_cmp(&state, &ctx->state) != 0)) {
        ctx->state = state;
        if (ctx->cb) {
            ctx->cb(dev, &ctx->state, ctx->cb_data);
        }
    }

    /* Log the results of the configuration */
    LOG_INF("PHY %d is %s", cfg->addr, ctx->state.is_up ? "up" : "down");
    if (ctx->state.is_up == true) {
        LOG_INF("PHY (%d) Link speed %s Mb, %s duplex\n", cfg->addr,
                (PHY_LINK_IS_SPEED_100M(ctx->state.speed) ? "100" : "10"),
                PHY_LINK_IS_FULL_DUPLEX(ctx->state.speed) ? "full" : "half");
    }

done :
    k_mutex_unlock(&ctx->mutex);

    /* Start monitoring */
    k_work_reschedule(&ctx->phy_monitor_work,
                      K_MSEC(CONFIG_PHY_MONITOR_PERIOD));

    return (ret);
}

static int phy_mc_lan8742a_link_cb_set(const struct device* dev,
                                       phy_callback_t cb, void* user_data) {
    struct mc_lan8742a_data* ctx = dev->data;

    ctx->cb = cb;
    ctx->cb_data = user_data;

    phy_mc_lan8742a_get_link(dev, &ctx->state);

    ctx->cb(dev, &ctx->state, ctx->cb_data);

    return (0);
}

static void phy_mc_lan8742a_monitor_work_handler(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct mc_lan8742a_data* ctx =
            CONTAINER_OF(dwork, struct mc_lan8742a_data, phy_monitor_work);
    const struct device* dev = ctx->dev;
    struct phy_link_state state;
    int rc;

    rc = phy_mc_lan8742a_get_link(dev, &state);
    if ((rc == 0) &&
        (phy_link_state_cmp(&state, &ctx->state) != 0)) {
        ctx->state = state;
        if (ctx->cb) {
            ctx->cb(dev, &ctx->state, ctx->cb_data);
        }
    }

    /* TODO change this to GPIO interrupt driven */
    k_work_reschedule(&ctx->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_mc_lan8742a_init(const struct device* dev) {
    const struct mc_lan8742a_config* cfg = dev->config;
    struct mc_lan8742a_data* ctx = dev->data;
    int ret;

    ctx->dev = dev;

    ret = k_mutex_init(&ctx->mutex);
    if (ret != 0) {
        return (ret);
    }

    mdio_bus_enable(cfg->mdio_dev);

    #if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_interrupt_gpio)
    if (cfg->interrupt_gpio.port == NULL) {
        goto skip_int_gpio;
    }

    /* Prevent NAND TREE mode */
    ret = gpio_pin_configure_dt(&cfg->interrupt_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        return (ret);
    }

skip_int_gpio :
    #endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_interrupt_gpio) */

    #if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_reset_gpio)
    if (cfg->reset_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret) {
            return ret;
        }
    }
    #endif

    /* Reset PHY */
    ret = phy_mc_lan8742a_reset(dev);
    if (ret) {
        return (ret);
    }

    k_work_init_delayable(&ctx->phy_monitor_work,
                          phy_mc_lan8742a_monitor_work_handler);

    /* Advertise default speeds */
    phy_mc_lan8742a_cfg_link(dev, cfg->default_speeds, 0);

    return (0);
}

static DEVICE_API(ethphy, mc_lan8742a_phy_api) = {
    .get_link    = phy_mc_lan8742a_get_link,
    .cfg_link    = phy_mc_lan8742a_cfg_link,
    .link_cb_set = phy_mc_lan8742a_link_cb_set,
    .read        = phy_mc_lan8742a_read,
    .write       = phy_mc_lan8742a_write
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_reset_gpio)
#define RESET_GPIO(n) \
                .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, mc_reset_gpio, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(mc_interrupt_gpio)
#define INTERRUPT_GPIO(n) \
                .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, mc_interrupt_gpio, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

#define MICROCHIP_LAN8742A_INIT(n)                                      \
    static const struct mc_lan8742a_config mc_lan8742a_##n##_config = { \
        .addr      = DT_INST_REG_ADDR(n),                               \
        .mdio_dev  = DEVICE_DT_GET(DT_INST_PARENT(n)),                  \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),          \
        RESET_GPIO(n)                                                   \
        INTERRUPT_GPIO(n)                                               \
    };                                                                  \
                                                                        \
    static struct mc_lan8742a_data mc_lan8742a_##n##_data;              \
                                                                        \
    DEVICE_DT_INST_DEFINE(n, &phy_mc_lan8742a_init, NULL,               \
                          &mc_lan8742a_##n##_data, &mc_lan8742a_##n##_config, \
                          POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,        \
                          &mc_lan8742a_phy_api);

DT_INST_FOREACH_STATUS_OKAY(MICROCHIP_LAN8742A_INIT)

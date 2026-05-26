/*
 * Copyright (c) 2024 Marvell / Zephyr Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT marvell_88e1510

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_marvell_88e1510, CONFIG_PHY_LOG_LEVEL);

#include "phy_mii.h"

/* PHY OUI + model, ignoring the 4-bit revision field */
#define MARVELL_88E1510_PHY_ID      0x01410DD0U
#define MARVELL_88E1510_PHY_ID_MASK 0xFFFFFFF0U

#define MII_INVALID_PHY_ID          UINT32_MAX
#define MII_AUTONEG_POLL_INTERVAL_MS 100

/* 1000BASE-T Status register offset relative to 1KTCR for capability comparison */
#define MII_1KSTSR_OFFSET 2

#define ANY_RESET_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)

struct phy_88e1510_config {
    const struct device* mdio;
    uint8_t phy_addr;
    enum phy_link_speed default_speeds;

    #if ANY_RESET_GPIO
    struct gpio_dt_spec reset_gpio;
    uint32_t reset_assert_duration_us;
    uint32_t reset_deassertion_timeout_ms;
    #endif
};

struct phy_88e1510_data {
    const struct device* dev;
    phy_callback_t cb;
    void* cb_data;
    struct phy_link_state state;
    struct k_sem sem;
    struct k_work_delayable monitor_work;
    bool gigabit_supported;
    bool autoneg_in_progress;
    k_timepoint_t autoneg_timeout;
};

static void phy_88e1510_invoke_link_cb(const struct device* dev);
static int  phy_88e1510_check_autoneg_completion(const struct device* dev);

static int phy_88e1510_read(const struct device* dev, uint16_t reg_addr, uint32_t* data) {
    const struct phy_88e1510_config* cfg = dev->config;
    uint16_t reg_val;
    int ret;

    ret = mdio_read(cfg->mdio, cfg->phy_addr, (uint8_t)reg_addr, &reg_val);
    *data = reg_val;

    return (ret);
}

static int phy_88e1510_write(const struct device* dev, uint16_t reg_addr, uint32_t data) {
    const struct phy_88e1510_config* cfg = dev->config;
    int ret;

    ret = mdio_write(cfg->mdio, cfg->phy_addr, (uint8_t)reg_addr, (uint16_t)data);

    return (ret);
}

static int phy_88e1510_reset(const struct device* dev) {
    uint32_t timeout = 12U;
    uint32_t value;
    int ret;

    #if ANY_RESET_GPIO
    const struct phy_88e1510_config* cfg = dev->config;

    if (gpio_is_ready_dt(&cfg->reset_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure RST pin (%d)", ret);
            return (ret);
        }

        k_busy_wait(cfg->reset_assert_duration_us);

        ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to de-assert RST pin (%d)", ret);
            return (ret);
        }

        k_msleep(cfg->reset_deassertion_timeout_ms);

        return (0);
    }
    #endif /* ANY_RESET_GPIO */

    /* Soft reset via BMCR bit 15 */
    ret = phy_88e1510_write(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret < 0) {
        return (-EIO);
    }

    /* Poll until reset self-clears, up to ~600 ms (IEEE 802.3 §22.2.4.1.1) */
    do {
        if (timeout-- == 0U) {
            return (-ETIMEDOUT);
        }

        k_msleep(50);

        ret = phy_88e1510_read(dev, MII_BMCR, &value);
        if (ret < 0) {
            return (-EIO);
        }
    } while ((value & MII_BMCR_RESET) != 0U);

    return (0);
}

static int phy_88e1510_get_id(const struct device* dev, uint32_t* phy_id) {
    uint32_t value;
    int ret;

    ret = phy_88e1510_read(dev, MII_PHYID1R, &value);
    if (ret < 0) {
        return (-EIO);
    }
    *phy_id = value << 16;

    ret = phy_88e1510_read(dev, MII_PHYID2R, &value);
    if (ret < 0) {
        return (-EIO);
    }
    *phy_id |= value;

    return (0);
}

static int phy_88e1510_read_gigabit_flag(const struct device* dev, bool* supported) {
    uint32_t bmsr_reg;
    uint32_t estat_reg;
    int ret;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        return (-EIO);
    }

    if ((bmsr_reg & MII_BMSR_EXTEND_STATUS) != 0U) {
        ret = phy_88e1510_read(dev, MII_ESTAT, &estat_reg);
        if (ret < 0) {
            return (-EIO);
        }

        if ((estat_reg & (MII_ESTAT_1000BASE_T_HALF | MII_ESTAT_1000BASE_T_FULL)) != 0U) {
            *supported = true;
            return (0);
        }
    }

    *supported = false;

    return (0);
}

static int phy_88e1510_update_link_state(const struct device* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    uint32_t bmcr_reg;
    uint32_t bmsr_reg;
    bool link_up;
    int ret;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        return (-EIO);
    }

    link_up = ((bmsr_reg & MII_BMSR_LINK_STATUS) != 0U);
    if (link_up == false) {
        data->state.speed = 0;

        if (link_up != data->state.is_up) {
            data->state.is_up = false;
            LOG_INF("PHY (%d) is down", cfg->phy_addr);

            return (0);
        }

        return (-EAGAIN);
    }

    ret = phy_88e1510_read(dev, MII_BMCR, &bmcr_reg);
    if (ret < 0) {
        return (-EIO);
    }

    if ((bmcr_reg & MII_BMCR_AUTONEG_ENABLE) == 0U) {
        enum phy_link_speed new_speed = phy_mii_get_link_speed_bmcr_reg(dev, bmcr_reg);

        if ((data->state.speed != new_speed) ||
            (data->state.is_up == false)) {
            data->state.is_up = true;
            data->state.speed = new_speed;

            LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                    PHY_LINK_IS_SPEED_1000M(data->state.speed) ? "1000" :
                    (PHY_LINK_IS_SPEED_100M(data->state.speed) ?  "100" : "10"),
                    PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");

            return (0);
        }

        return (-EAGAIN);
    }

    if (data->state.is_up) {
        return (-EAGAIN);
    }

    data->state.is_up = true;

    LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);

    data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));

    ret = phy_88e1510_check_autoneg_completion(dev);

    return (ret);
}

static int phy_88e1510_check_autoneg_completion(const struct device* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    uint32_t anar_reg;
    uint32_t anlpar_reg;
    uint32_t bmsr_reg;
    uint32_t c1kt_reg;
    uint32_t s1kt_reg;
    int ret;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        return (-EIO);
    }

    if ((bmsr_reg & MII_BMSR_AUTONEG_COMPLETE) == 0U) {
        if (sys_timepoint_expired(data->autoneg_timeout)) {
            LOG_DBG("PHY (%d) auto-negotiate timeout", cfg->phy_addr);
            return (-ETIMEDOUT);
        }

        return (-EINPROGRESS);
    }

    /* BMSR link-status bit is latched-low; re-read to get current value */
    if (unlikely((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U)) {
        ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
        if (ret < 0) {
            return (-EIO);
        }

        if ((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U) {
            return (-EAGAIN);
        }
    }

    LOG_DBG("PHY (%d) auto-negotiate completed", cfg->phy_addr);

    ret = phy_88e1510_read(dev, MII_ANAR, &anar_reg);
    if (ret < 0) {
        return (-EIO);
    }

    ret = phy_88e1510_read(dev, MII_ANLPAR, &anlpar_reg);
    if (ret < 0) {
        return (-EIO);
    }

    if (data->gigabit_supported) {
        ret = phy_88e1510_read(dev, MII_1KTCR, &c1kt_reg);
        if (ret < 0) {
            return (-EIO);
        }

        ret = phy_88e1510_read(dev, MII_1KSTSR, &s1kt_reg);
        if (ret < 0) {
            return (-EIO);
        }

        s1kt_reg = (s1kt_reg >> MII_1KSTSR_OFFSET);
    }

    if (data->gigabit_supported &&
        ((c1kt_reg & s1kt_reg & MII_ADVERTISE_1000_FULL) != 0U)) {
        data->state.speed = LINK_FULL_1000BASE;
    }
    else if (data->gigabit_supported &&
             ((c1kt_reg & s1kt_reg & MII_ADVERTISE_1000_HALF) != 0U)) {
        data->state.speed = LINK_HALF_1000BASE;
    }
    else if ((anar_reg & anlpar_reg & MII_ADVERTISE_100_FULL) != 0U) {
        data->state.speed = LINK_FULL_100BASE;
    }
    else if ((anar_reg & anlpar_reg & MII_ADVERTISE_100_HALF) != 0U) {
        data->state.speed = LINK_HALF_100BASE;
    }
    else if ((anar_reg & anlpar_reg & MII_ADVERTISE_10_FULL) != 0U) {
        data->state.speed = LINK_FULL_10BASE;
    }
    else {
        data->state.speed = LINK_HALF_10BASE;
    }

    data->state.is_up = true;

    LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
            PHY_LINK_IS_SPEED_1000M(data->state.speed) ? "1000" :
            (PHY_LINK_IS_SPEED_100M(data->state.speed) ?  "100" : "10"),
            PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");

    return (0);
}

static void phy_88e1510_monitor_work_handler(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct phy_88e1510_data* data =
        CONTAINER_OF(dwork, struct phy_88e1510_data, monitor_work);
    const struct device* dev = data->dev;
    int ret;

    if (k_sem_take(&data->sem, K_NO_WAIT) == 0) {
        if (data->autoneg_in_progress) {
            ret = phy_88e1510_check_autoneg_completion(dev);
        }
        else {
            ret = phy_88e1510_update_link_state(dev);
        }

        if (ret == -EINPROGRESS) {
            data->autoneg_in_progress = true;
        }
        else {
            data->autoneg_in_progress = false;
        }

        k_sem_give(&data->sem);

        if (ret == 0) {
            phy_88e1510_invoke_link_cb(dev);
        }
    }

    k_work_reschedule(&data->monitor_work,
                      data->autoneg_in_progress ? K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS)
                                                : K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_88e1510_cfg_link(const struct device* dev, enum phy_link_speed adv_speeds,
                                enum phy_cfg_link_flag flags) {
    const struct phy_88e1510_config* cfg  = dev->config;
    struct phy_88e1510_data* data = dev->data;
    int ret;

    k_sem_take(&data->sem, K_FOREVER);

    if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
        if (!data->gigabit_supported && PHY_LINK_IS_SPEED_1000M(adv_speeds)) {
            LOG_ERR("PHY (%d) Gigabit not supported", cfg->phy_addr);
            ret = -ENOTSUP;
            goto out;
        }

        ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
        if (ret >= 0) {
            data->autoneg_in_progress = false;
            k_work_reschedule(&data->monitor_work, K_NO_WAIT);
        }
    }
    else {
        ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, data->gigabit_supported);
        if (ret >= 0) {
            LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);
            data->autoneg_in_progress = true;
            data->autoneg_timeout =
                sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
            k_work_reschedule(&data->monitor_work,
                              K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
        }
    }

    if (ret == -EALREADY) {
        LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
    }

out :
    k_sem_give(&data->sem);

    return (ret);
}

static int phy_88e1510_get_link_state(const struct device* dev, struct phy_link_state* state) {
    struct phy_88e1510_data* data = dev->data;

    k_sem_take(&data->sem, K_FOREVER);

    memcpy(state, &data->state, sizeof(struct phy_link_state));

    /* Speed == 0 means autoneg is in progress; report link as down */
    if (state->speed == 0) {
        state->is_up = false;
    }

    k_sem_give(&data->sem);

    return (0);
}

static void phy_88e1510_invoke_link_cb(const struct device* dev) {
    struct phy_88e1510_data* data = dev->data;
    struct phy_link_state state;

    if (data->cb != NULL) {
        phy_88e1510_get_link_state(dev, &state);

        data->cb(dev, &state, data->cb_data);
    }
}

static int phy_88e1510_link_cb_set(const struct device* dev, phy_callback_t cb, void* user_data) {
    struct phy_88e1510_data* data = dev->data;

    data->cb = cb;
    data->cb_data = user_data;

    phy_88e1510_invoke_link_cb(dev);

    return (0);
}

static int phy_88e1510_init(const struct device* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    uint32_t phy_id;
    int ret;

    data->state.is_up = false;

    if (!device_is_ready(cfg->mdio)) {
        LOG_ERR("MDIO device not ready");
        return -ENODEV;
    }

    ret = phy_88e1510_reset(dev);
    if (ret < 0) {
        LOG_ERR("PHY (%d) reset failed: %d", cfg->phy_addr, ret);
        return (ret);
    }

    ret = phy_88e1510_get_id(dev, &phy_id);
    if (ret < 0) {
        LOG_ERR("PHY (%d) ID read failed: %d", cfg->phy_addr, ret);
        return (ret);
    }

    if (phy_id == MII_INVALID_PHY_ID) {
        LOG_ERR("No PHY found at address %d", cfg->phy_addr);
        return (-EINVAL);
    }

    if ((phy_id & MARVELL_88E1510_PHY_ID_MASK) != MARVELL_88E1510_PHY_ID) {
        LOG_ERR("PHY (%d) unexpected ID 0x%08X (expected 0x%08X)",
                cfg->phy_addr, phy_id, MARVELL_88E1510_PHY_ID);
        return (-ENODEV);
    }

    LOG_INF("PHY (%d) Marvell 88E1510 ID 0x%08X", cfg->phy_addr, phy_id);

    ret = phy_88e1510_read_gigabit_flag(dev, &data->gigabit_supported);
    if (ret < 0) {
        LOG_ERR("PHY (%d) capability read failed: %d", cfg->phy_addr, ret);
        return (ret);
    }

    k_work_init_delayable(&data->monitor_work, phy_88e1510_monitor_work_handler);

    ret = phy_88e1510_cfg_link(dev, cfg->default_speeds, 0);
    if (ret == -EALREADY) {
        data->autoneg_in_progress = true;
        data->autoneg_timeout =
            sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
    }

    k_work_schedule(&data->monitor_work, K_NO_WAIT);

    return (0);
}

static DEVICE_API(ethphy, phy_88e1510_driver_api) = {
    .get_link    = phy_88e1510_get_link_state,
    .link_cb_set = phy_88e1510_link_cb_set,
    .cfg_link    = phy_88e1510_cfg_link,
    .read        = phy_88e1510_read,
    .write       = phy_88e1510_write,
};

#if ANY_RESET_GPIO
#define RESET_GPIO_FIELDS(n) \
    .reset_gpio                   = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),   \
    .reset_assert_duration_us     = DT_INST_PROP_OR(n, reset_assert_duration_us, 0), \
    .reset_deassertion_timeout_ms = DT_INST_PROP_OR(n, reset_deassertion_timeout_ms, 0),
#else
#define RESET_GPIO_FIELDS(n)
#endif /* ANY_RESET_GPIO */

#define PHY_88E1510_CONFIG(n)                                   \
    BUILD_ASSERT((PHY_INST_GENERATE_DEFAULT_SPEEDS(n) != 0),    \
                 "At least one speed must be configured");      \
    static struct phy_88e1510_config DT_CONST phy_88e1510_config_##n = { \
        .mdio     = DEVICE_DT_GET(DT_INST_BUS(n)),              \
        .phy_addr = DT_INST_REG_ADDR(n),                        \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),  \
        RESET_GPIO_FIELDS(n)                                    \
    }

#define PHY_88E1510_DATA(n)                                     \
    static struct phy_88e1510_data phy_88e1510_data_##n = {     \
        .dev = DEVICE_DT_INST_GET(n),                           \
        .cb  = NULL,                                            \
        .sem = Z_SEM_INITIALIZER(phy_88e1510_data_##n.sem, 1, 1), \
    }

#define PHY_88E1510_DEVICE(n)                                   \
    PHY_88E1510_CONFIG(n);                                      \
    PHY_88E1510_DATA(n);                                        \
    DEVICE_DT_INST_DEFINE(n, phy_88e1510_init, NULL,            \
                          &phy_88e1510_data_##n, &phy_88e1510_config_##n, \
                          POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, \
                          &phy_88e1510_driver_api)

DT_INST_FOREACH_STATUS_OKAY(PHY_88E1510_DEVICE)

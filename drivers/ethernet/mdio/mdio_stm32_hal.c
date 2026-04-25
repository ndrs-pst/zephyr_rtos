/*
 * Copyright (c) 2024 BayLibre, SAS
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>
#include <zephyr/net/mii.h>

#include "../eth_stm32_hal_priv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_stm32_hal, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_mdio

struct mdio_stm32_data {
    struct k_mutex mutex;
};

struct mdio_stm32_config {
    const struct device* ethernet_dev;
    const struct pinctrl_dev_config* pincfg;
};

static int mdio_stm32_read(const struct device* dev, uint8_t prtad,
                           uint8_t regad, uint16_t* data) {
    struct mdio_stm32_data* const ctx = dev->data;
    const struct mdio_stm32_config* const cfg = dev->config;
    struct eth_stm32_hal_dev_data* eth_ctx = cfg->ethernet_dev->data;
    ETH_HandleTypeDef* heth = &eth_ctx->heth;
    HAL_StatusTypeDef ret;
    uint32_t read;

    k_mutex_lock(&ctx->mutex, K_FOREVER);

    #if defined(CONFIG_ETH_STM32_HAL_API_V2)
    ret = HAL_ETH_ReadPHYRegister(heth, prtad, regad, &read);
    #else
    heth->Init.PhyAddress = prtad;

    ret = HAL_ETH_ReadPHYRegister(heth, regad, &read);
    #endif

    k_mutex_unlock(&ctx->mutex);

    if (ret != HAL_OK) {
        return (-EIO);
    }

    *data = (read & GENMASK(15, 0));

    return (0);
}

static int mdio_stm32_write(const struct device* dev, uint8_t prtad,
                            uint8_t regad, uint16_t data) {
    struct mdio_stm32_data* const ctx = dev->data;
    const struct mdio_stm32_config* const cfg = dev->config;
    struct eth_stm32_hal_dev_data* eth_ctx = cfg->ethernet_dev->data;
    ETH_HandleTypeDef* heth = &eth_ctx->heth;
    HAL_StatusTypeDef ret;

    k_mutex_lock(&ctx->mutex, K_FOREVER);

    #if defined(CONFIG_ETH_STM32_HAL_API_V2)
    ret = HAL_ETH_WritePHYRegister(heth, prtad, regad, data);
    #else
    heth->Init.PhyAddress = prtad;

    ret = HAL_ETH_WritePHYRegister(heth, regad, data);
    #endif

    k_mutex_unlock(&ctx->mutex);

    if (ret != HAL_OK) {
        return (-EIO);
    }

    return (0);
}

static int mdio_stm32_c45_setup_dev_reg(const struct device* dev, uint8_t prtad,
                                        uint16_t devad, uint16_t reg) {
    int ret;

    ret = mdio_write(dev, prtad, MII_MMD_ACR, devad);
    if (ret < 0) {
        return (ret);
    }

    ret = mdio_write(dev, prtad, MII_MMD_AADR, reg);
    if (ret < 0) {
        return (ret);
    }

    ret = mdio_write(dev, prtad, MII_MMD_ACR, MII_MMD_ACR_DATA_NO_POS_INC | devad);

    return (ret);
}

/**
 * @brief Read a PHY register using the MDIO bus clause 45
 * @param[in] dev   MDIO device
 * @param[in] prtad PHY port address
 * @param[in] devad PHY device address
 * @param[in] regad PHY register address
 * @param[out] data Pointer to the data to be read
 * @return 0 on success, negative errno code on fail
 */
static int mdio_stm32_read_c45(const struct device* dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t* data) {
    int ret;

    ret = mdio_stm32_c45_setup_dev_reg(dev, prtad, devad, regad);
    if (ret < 0) {
        return (ret);
    }

    ret = mdio_read(dev, prtad, MII_MMD_AADR, data);

    return (ret);
}

/**
 * @brief Write a PHY register using the MDIO bus clause 45
 * @param[in] dev MDIO device
 * @param[in] prtad PHY port address
 * @param[in] devad PHY device address
 * @param[in] regad PHY register address
 * @param[in] data  Data to be written
 * @return 0 on success, negative errno code on fail
 */
static int mdio_stm32_write_c45(const struct device* dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t data) {
    int ret;

    ret = mdio_stm32_c45_setup_dev_reg(dev, prtad, devad, regad);
    if (ret < 0) {
        return (ret);
    }

    ret = mdio_write(dev, prtad, MII_MMD_AADR, data);

    return (ret);
}

static int mdio_stm32_init(const struct device* dev) {
    struct mdio_stm32_data* const ctx = dev->data;
    const struct mdio_stm32_config* const cfg = dev->config;
    int ret;

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return (ret);
    }

    k_mutex_init(&ctx->mutex);

    return (0);
}

static DEVICE_API(mdio, mdio_stm32_api) = {
    .read      = mdio_stm32_read,
    .write     = mdio_stm32_write,
    .read_c45  = mdio_stm32_read_c45,
    .write_c45 = mdio_stm32_write_c45,
};

#define MDIO_STM32_HAL_DEVICE(inst)                             \
    PINCTRL_DT_INST_DEFINE(inst);                               \
                                                                \
    static struct mdio_stm32_data mdio_stm32_data_##inst;       \
                                                                \
    static struct mdio_stm32_config mdio_stm32_config_##inst = {\
        .ethernet_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),    \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),         \
    };                                                          \
    DEVICE_DT_INST_DEFINE(inst, mdio_stm32_init, NULL, &mdio_stm32_data_##inst, \
                          &mdio_stm32_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY, \
                          &mdio_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_STM32_HAL_DEVICE)

#if (__GTEST == 1) /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_mdio_stm32(void) {
    /* pass */
}

void zephyr_gtest_mdio_stm32_init(const struct device* dev) {
    mdio_stm32_init(dev);
}

#endif

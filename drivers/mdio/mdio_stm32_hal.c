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
#include <zephyr/net/mii.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_stm32_hal, CONFIG_MDIO_LOG_LEVEL);

/* #CUSTOM@NDRS */
#define DEVICE_STM32_GET_ETH_HNDL(dev) (ETH_HandleTypeDef*)(&(((struct mdio_stm32_dev_data*)(dev)->data)->heth))

#define DT_DRV_COMPAT st_stm32_mdio

struct ETH_HandlePrivTypeDef {
    ETH_TypeDef* Instance; /*!< Register base address       */
};

struct mdio_stm32_dev_data {
    struct ETH_HandlePrivTypeDef heth;
    struct k_sem sem;
};

struct mdio_stm32_dev_config {
    const struct pinctrl_dev_config* pincfg;
    struct stm32_pclken pclken;
};

static int mdio_stm32_read(const struct device* dev, uint8_t prtad,
                           uint8_t regad, uint16_t* data) {
    struct mdio_stm32_dev_data* const dev_data = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    HAL_StatusTypeDef hal_sts;
    uint32_t read;

    k_sem_take(&dev_data->sem, K_FOREVER);
    hal_sts = HAL_ETH_ReadPHYRegister(heth, prtad, regad, &read);
    k_sem_give(&dev_data->sem);

    if (hal_sts != HAL_OK) {
        return (-EIO);
    }

    *data = (read & GENMASK(15, 0));

    return (0);
}

static int mdio_stm32_write(const struct device* dev, uint8_t prtad,
                            uint8_t regad, uint16_t data) {
    struct mdio_stm32_dev_data* const ctx = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    HAL_StatusTypeDef hal_sts;

    k_sem_take(&ctx->sem, K_FOREVER);
    hal_sts = HAL_ETH_WritePHYRegister(heth, prtad, regad, data);
    k_sem_give(&ctx->sem);

    if (hal_sts != HAL_OK) {
        return (-EIO);
    }

    return (0);
}

static int mdio_stm32_c45_setup_dev_reg(const struct device* dev, uint8_t prtad, uint16_t devad, uint16_t reg) {
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
    const struct mdio_stm32_dev_config* const cfg = dev->config;
    struct mdio_stm32_dev_data* const ctx = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    int ret;

    ret = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                           (clock_control_subsys_t)&cfg->pclken);
    if (ret < 0) {
        LOG_ERR("Failed to enable ethernet clock needed for MDIO (%d)", ret);
        return (ret);
    }

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return (ret);
    }

    HAL_ETH_SetMDIOClockRange(heth);
    k_sem_init(&ctx->sem, 1, 1);

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
    static struct mdio_stm32_dev_data mdio_stm32_dev_data_##inst = {            \
        .heth = {.Instance = (ETH_TypeDef*)DT_REG_ADDR(DT_INST_PARENT(inst))},  \
    };                                                          \
    static struct mdio_stm32_dev_config mdio_stm32_dev_config_##inst = {        \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),         \
        .pclken = STM32_CLOCK_INFO_BY_NAME(DT_INST_PARENT(inst), stm_eth),      \
    };                                                          \
    DEVICE_DT_INST_DEFINE(inst, mdio_stm32_init, NULL, &mdio_stm32_dev_data_##inst, \
                          &mdio_stm32_dev_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY, \
                          &mdio_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_STM32_HAL_DEVICE)

#if (__GTEST == 1U) /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_mdio_stm32(void) {
    mdio_stm32_dev_data_0.heth.Instance = (ETH_TypeDef*)ut_mcu_eth_ptr;
}

void zephyr_gtest_mdio_stm32_init(const struct device* dev) {
    mdio_stm32_init(dev);
}

#endif

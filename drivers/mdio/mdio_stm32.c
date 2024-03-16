/*
 * Copyright (c) 2024 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_mdio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_stm32, CONFIG_MDIO_LOG_LEVEL);

/* #CUSTOM@NDRS */
#define DEVICE_STM32_GET_ETH_HNDL(dev)         (ETH_HandleTypeDef*)(&(((struct mdio_stm32_dev_data*)(dev)->data)->heth))

struct ETH_HandlePrivTypeDef {
    ETH_TypeDef* Instance;                  /*!< Register base address       */
};

struct mdio_stm32_dev_data {
    struct ETH_HandlePrivTypeDef heth;
    struct k_sem sem;
};

struct mdio_stm32_dev_config {
    const struct pinctrl_dev_config* pcfg;
};

/**
 * @brief Read a PHY register using the MDIO bus clause 22
 * @param[in] dev   MDIO device
 * @param[in] prtad PHY port address
 * @param[in] regad PHY register address
 * @param[out] data Pointer to the data to be read
 * @return 0 on success, negative errno code on fail
 */
static int mdio_stm32_read(const struct device* dev, uint8_t prtad, uint8_t regad, uint16_t* data) {
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    HAL_StatusTypeDef hal_sts;
    uint32_t reg_val;
    int ret;

    hal_sts = HAL_ETH_ReadPHYRegister(heth, prtad, regad, &reg_val);
    if (hal_sts != HAL_OK) {
        ret = -EIO;
    }
    else {
        *data = (uint16_t)reg_val;
        ret = 0;
    }

    return (ret);
}

/**
 * @brief Write a PHY register using the MDIO bus clause 22
 * @param[in] dev MDIO device
 * @param[in] prtad PHY port address
 * @param[in] regad PHY register address
 * @param[in] data  Data to be written
 * @return 0 on success, negative errno code on fail
 */
static int mdio_stm32_write(const struct device* dev, uint8_t prtad, uint8_t regad, uint16_t data) {
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    HAL_StatusTypeDef hal_sts;
    int ret;

    hal_sts = HAL_ETH_WritePHYRegister(heth, prtad, regad, data);
    if (hal_sts != HAL_OK) {
        ret = -EIO;
    }
    else {
        ret = 0;
    }

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
    ARG_UNUSED(dev);
    ARG_UNUSED(prtad);
    ARG_UNUSED(devad);
    ARG_UNUSED(regad);
    ARG_UNUSED(data);

    return (-ENOTSUP);
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
    ARG_UNUSED(dev);
    ARG_UNUSED(prtad);
    ARG_UNUSED(devad);
    ARG_UNUSED(regad);
    ARG_UNUSED(data);

    return (-ENOTSUP);
}

static void mdio_stm32_bus_enable(const struct device* dev) {
    ARG_UNUSED(dev);

    /*
     * MDIO bus device is actually part of ethernet device, and
     * does not support ability to disable/enable MDIO bus hardware
     * independently of the ethernet/MAC hardware, so do nothing.
     */
}

static void mdio_stm32_bus_disable(const struct device* dev) {
    ARG_UNUSED(dev);

    /*
     * MDIO bus device is actually part of ethernet device, and
     * does not support ability to disable/enable MDIO bus hardware
     * independently of the ethernet/MAC hardware, so do nothing.
     */
}

/**
 * @brief Initialize the MDIO bus device
 * @param[in] dev MDIO device
 * @return 0 on success, negative errno code on fail
 */
static int mdio_stm32_initialize(const struct device* dev) {
    struct mdio_stm32_dev_config const* cfg = dev->config;
    struct mdio_stm32_dev_data* const data = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    int ret;

    k_sem_init(&data->sem, 1, 1);

    HAL_ETH_SetMDIOClockRange(heth);

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

    return (ret);
}

static struct mdio_driver_api const mdio_stm32_driver_api = {
    .read        = mdio_stm32_read,
    .write       = mdio_stm32_write,
    .read_c45    = mdio_stm32_read_c45,
    .write_c45   = mdio_stm32_write_c45,
    .bus_enable  = mdio_stm32_bus_enable,
    .bus_disable = mdio_stm32_bus_disable
};

#define MDIO_STM32_CONFIG(n)                                    \
    static struct mdio_stm32_dev_config DT_CONST mdio_stm32_dev_config_##n = { \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)               \
    }

#define MDIO_STM32_DEVICE(n)                                    \
    PINCTRL_DT_INST_DEFINE(n);                                  \
    MDIO_STM32_CONFIG(n);                                       \
    static struct mdio_stm32_dev_data mdio_stm32_dev_data_##n = {     \
        .heth = {                                               \
            .Instance = (ETH_TypeDef*)DT_REG_ADDR(DT_INST_PARENT(n)), \
        }                                                       \
    };                                                          \
    DEVICE_DT_INST_DEFINE(n,                                    \
                          &mdio_stm32_initialize,               \
                          NULL,                                 \
                          &mdio_stm32_dev_data_##n,             \
                          &mdio_stm32_dev_config_##n,           \
                          POST_KERNEL,                          \
                          CONFIG_MDIO_INIT_PRIORITY,            \
                          &mdio_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_STM32_DEVICE)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_mdio_stm32(void) {
    mdio_stm32_dev_data_0.heth.Instance = (ETH_TypeDef*)ut_mcu_eth_ptr;
}

void zephyr_gtest_mdio_stm32_init(const struct device* dev) {
    mdio_stm32_initialize(dev);
}

#endif


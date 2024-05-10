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
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_stm32_hal, CONFIG_MDIO_LOG_LEVEL);

/* #CUSTOM@NDRS */
#define DEVICE_STM32_GET_ETH_HNDL(dev)         (ETH_HandleTypeDef*)(&(((struct mdio_stm32_dev_data*)(dev)->data)->heth))

#define DT_DRV_COMPAT st_stm32_mdio

#define ADIN1100_REG_VALUE_MASK GENMASK(15, 0)

struct ETH_HandlePrivTypeDef {
    ETH_TypeDef* Instance;                  /*!< Register base address       */
};

struct mdio_stm32_dev_data {
    struct ETH_HandlePrivTypeDef heth;
    struct k_sem sem;
};

struct mdio_stm32_dev_config {
    const struct pinctrl_dev_config* pincfg;
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
    struct mdio_stm32_dev_data* const dev_data = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    uint32_t read;
    int ret;

    k_sem_take(&dev_data->sem, K_FOREVER);

    ret = HAL_ETH_ReadPHYRegister(heth, prtad, regad, &read);

    k_sem_give(&dev_data->sem);

    if (ret != HAL_OK) {
        return (-EIO);
    }

    *data = read & ADIN1100_REG_VALUE_MASK;

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
    struct mdio_stm32_dev_data* const dev_data = dev->data;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    int ret;

    k_sem_take(&dev_data->sem, K_FOREVER);

    ret = HAL_ETH_WritePHYRegister(heth, prtad, regad, data);

    k_sem_give(&dev_data->sem);

    if (ret != HAL_OK) {
        return (-EIO);
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
static int mdio_stm32_init(const struct device* dev) {
    struct mdio_stm32_dev_data* const dev_data = dev->data;
    const struct mdio_stm32_dev_config* const config = dev->config;
    ETH_HandleTypeDef* heth = DEVICE_STM32_GET_ETH_HNDL(dev);
    int ret;

    ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return (ret);
    }

    k_sem_init(&dev_data->sem, 1, 1);

    HAL_ETH_SetMDIOClockRange(heth);

    return (0);
}

static struct mdio_driver_api const mdio_stm32_driver_api = {
    .read        = mdio_stm32_read,
    .write       = mdio_stm32_write,
    .read_c45    = mdio_stm32_read_c45,
    .write_c45   = mdio_stm32_write_c45,
    .bus_enable  = mdio_stm32_bus_enable,
    .bus_disable = mdio_stm32_bus_disable
};

#define MDIO_STM32_HAL_DEVICE(inst)                                                                                    \
    PINCTRL_DT_INST_DEFINE(inst);                                                                                      \
                                                                                                                       \
    static struct mdio_stm32_dev_data mdio_stm32_dev_data_##inst = {                                                   \
        .heth = {.Instance = (ETH_TypeDef*)DT_REG_ADDR(DT_INST_PARENT(inst))},                                         \
    };                                                                                                                 \
    static struct mdio_stm32_dev_config mdio_stm32_dev_config_##inst = {                                               \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                                                \
    };                                                                                                                 \
    DEVICE_DT_INST_DEFINE(inst, &mdio_stm32_init, NULL, &mdio_stm32_dev_data_##inst, &mdio_stm32_dev_config_##inst,    \
                          POST_KERNEL, CONFIG_ETH_INIT_PRIORITY, &mdio_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_STM32_HAL_DEVICE)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_mdio_stm32(void) {
    mdio_stm32_dev_data_0.heth.Instance = (ETH_TypeDef*)ut_mcu_eth_ptr;
}

void zephyr_gtest_mdio_stm32_init(const struct device* dev) {
    mdio_stm32_init(dev);
}

#endif

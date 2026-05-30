/*
 * Copyright (c) 2026 Infineon
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_mdio_pdl

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(infineon_mdio_pdl, CONFIG_MDIO_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>

#include "cy_ethif.h"

struct mdio_infineon_pdl_config {
    ETH_Type* base;
    const struct pinctrl_dev_config* pincfg;
};

struct mdio_infineon_pdl_data {
    struct k_mutex bus_mutex;
};

static int mdio_infineon_pdl_read(const struct device* dev, uint8_t prtad,
                                  uint8_t regad, uint16_t* regval) {
    const struct mdio_infineon_pdl_config* cfg = dev->config;
    struct mdio_infineon_pdl_data* data = dev->data;
    uint32_t val;

    k_mutex_lock(&data->bus_mutex, K_FOREVER);
    val = Cy_ETHIF_PhyRegRead(cfg->base, regad, prtad);
    k_mutex_unlock(&data->bus_mutex);

    if (val == CY_ETHIF_MDIO_READ_FAILED) {
        return (-EIO);
    }

    *regval = (uint16_t)(val & 0xFFFF);

    return (0);
}

static int mdio_infineon_pdl_write(const struct device* dev, uint8_t prtad,
                                   uint8_t regad, uint16_t regval) {
    const struct mdio_infineon_pdl_config* cfg = dev->config;
    struct mdio_infineon_pdl_data* data = dev->data;
    cy_en_ethif_status_t status;

    k_mutex_lock(&data->bus_mutex, K_FOREVER);
    status = Cy_ETHIF_PhyRegWrite(cfg->base, regad, regval, prtad);
    k_mutex_unlock(&data->bus_mutex);

    return (status == CY_ETHIF_SUCCESS) ? 0 : -EIO;
}

static int mdio_infineon_pdl_init(const struct device* dev) {
    const struct mdio_infineon_pdl_config* cfg = dev->config;
    struct mdio_infineon_pdl_data* data = dev->data;
    int ret;

    k_mutex_init(&data->bus_mutex);

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("MDIO pinctrl apply failed (%d)", ret);
        return (ret);
    }

    /* Cy_ETHIF_MdioInit() is called by the MAC (eth_infineon_pdl) driver,
     * which owns the ETHIF block and its clocks.
     */

    return (0);
}

static DEVICE_API(mdio, mdio_infineon_pdl_driver_api) = {
    .read  = mdio_infineon_pdl_read,
    .write = mdio_infineon_pdl_write,
};

#define INFINEON_MDIO_PDL_DEVICE(n)                                             \
    PINCTRL_DT_INST_DEFINE(n);                                                  \
    static struct mdio_infineon_pdl_data mdio_infineon_pdl_data_##n;            \
    static const struct mdio_infineon_pdl_config mdio_infineon_pdl_cfg_##n = {  \
        .base   = (ETH_Type*)DT_REG_ADDR(DT_INST_PARENT(n)),                    \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                            \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n,                                                    \
                          mdio_infineon_pdl_init,                               \
                          NULL,                                                 \
                          &mdio_infineon_pdl_data_##n,                          \
                          &mdio_infineon_pdl_cfg_##n,                           \
                          POST_KERNEL,                                          \
                          CONFIG_MDIO_INIT_PRIORITY,                            \
                          &mdio_infineon_pdl_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INFINEON_MDIO_PDL_DEVICE)

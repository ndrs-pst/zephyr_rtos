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
    struct mdio_infineon_pdl_data* data = dev->data;

    k_mutex_init(&data->bus_mutex);

    /* Note: Cy_ETHIF_MdioInit() should be called here or by the MAC driver.
     * In this dual-driver design, the MAC driver usually calls Cy_ETHIF_Init()
     * which handles standard clocking setups, or we can assume it's orchestrated
     * via the MAC driver. If MDIO needs separate init without MAC, we would call
     * Cy_ETHIF_MdioInit() here with basic configs.
     * We will rely on MAC init to setup the base clocks for this wrapper.
     */

    return (0);
}

static DEVICE_API(mdio, mdio_infineon_pdl_driver_api) = {
    .read  = mdio_infineon_pdl_read,
    .write = mdio_infineon_pdl_write,
};

#define INFINEON_MDIO_PDL_DEVICE(n)                                             \
    static struct mdio_infineon_pdl_data mdio_infineon_pdl_data_##n;            \
    static const struct mdio_infineon_pdl_config mdio_infineon_pdl_cfg_##n = {  \
        .base = (ETH_Type*)DT_INST_REG_ADDR(n),                                 \
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

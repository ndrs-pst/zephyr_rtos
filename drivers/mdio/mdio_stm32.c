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

struct ETH_HandlePrivTypeDef {
    ETH_TypeDef* Instance;                  /*!< Register base address       */
};

struct mdio_stm32_dev_data {
    struct k_sem sem;
};

struct mdio_stm32_dev_config {
    struct ETH_HandlePrivTypeDef heth;
    const struct pinctrl_dev_config* pcfg;
};

static int mdio_stm32_transfer(const struct device* dev,
                               uint8_t prtad, uint8_t regad,
                               enum mdio_opcode op, bool c45,
                               uint16_t data_in, uint16_t* data_out) {
    const struct mdio_stm32_dev_config* const cfg = dev->config;
    struct mdio_stm32_dev_data* const data = dev->data;
    int timeout = 50;

    k_sem_take(&data->sem, K_FOREVER);

#if (0)
    /* Write mdio transaction */
    cfg->regs->GMAC_MAN = (c45 ? 0U : GMAC_MAN_CLTTO) | GMAC_MAN_OP(op) | GMAC_MAN_WTN(0x02) | GMAC_MAN_PHYA(prtad) |
                          GMAC_MAN_REGA(regad) | GMAC_MAN_DATA(data_in);

    /* Wait until done */
    while (!(cfg->regs->GMAC_NSR & GMAC_NSR_IDLE)) {
        if (timeout-- == 0U) {
            LOG_ERR("transfer timedout %s", dev->name);
            k_sem_give(&data->sem);

            return -ETIMEDOUT;
        }

        k_sleep(K_MSEC(5));
    }

    if (data_out) {
        *data_out = cfg->regs->GMAC_MAN & GMAC_MAN_DATA_Msk;
    }
#endif

    k_sem_give(&data->sem);

    return 0;
}

static int mdio_stm32_read(const struct device* dev, uint8_t prtad, uint8_t regad, uint16_t* data) {
    return mdio_stm32_transfer(dev, prtad, regad, MDIO_OP_C22_READ, false, 0, data);
}

static int mdio_stm32_write(const struct device* dev, uint8_t prtad, uint8_t regad, uint16_t data) {
    return mdio_stm32_transfer(dev, prtad, regad, MDIO_OP_C22_WRITE, false, data, NULL);
}

static int mdio_stm32_read_c45(const struct device* dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t* data) {
    int err;

    err = mdio_stm32_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true, regad, NULL);
    if (!err) {
        err = mdio_stm32_transfer(dev, prtad, devad, MDIO_OP_C45_READ, true, 0, data);
    }

    return err;
}

static int mdio_stm32_write_c45(const struct device* dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t data) {
    int err;

    err = mdio_stm32_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true, regad, NULL);
    if (!err) {
        err = mdio_stm32_transfer(dev, prtad, devad, MDIO_OP_C45_WRITE, true, data, NULL);
    }

    return err;
}

static void mdio_stm32_bus_enable(const struct device* dev) {
    const struct mdio_stm32_dev_config* const cfg = dev->config;

    //cfg->regs->GMAC_NCR |= GMAC_NCR_MPE;
}

static void mdio_stm32_bus_disable(const struct device* dev) {
    const struct mdio_stm32_dev_config* const cfg = dev->config;

    //cfg->regs->GMAC_NCR &= ~GMAC_NCR_MPE;
}

static int mdio_stm32_initialize(const struct device* dev) {
    struct mdio_stm32_dev_config const* cfg  = dev->config;
    struct mdio_stm32_dev_data* const data = dev->data;
    int retval;

    k_sem_init(&data->sem, 1, 1);

    HAL_ETH_SetMDIOClockRange((ETH_HandleTypeDef*)&cfg->heth);

    retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

    return retval;
}

static struct mdio_driver_api const mdio_stm32_driver_api = {
    .read        = mdio_stm32_read,
    .write       = mdio_stm32_write,
    .read_c45    = mdio_stm32_read_c45,
    .write_c45   = mdio_stm32_write_c45,
    .bus_enable  = mdio_stm32_bus_enable,
    .bus_disable = mdio_stm32_bus_disable
};

#define MDIO_SAM_CLOCK(n) COND_CODE_1(CONFIG_SOC_FAMILY_ATMEL_SAM, (.clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(n), ), ())

#define MDIO_STM32_CONFIG(n)                                                \
    static struct mdio_stm32_dev_config DT_CONST mdio_stm32_dev_config_##n = { \
        .heth = {                                                           \
            .Instance = (ETH_TypeDef*)DT_REG_ADDR(DT_INST_PARENT(n)),       \
        },                                                                  \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)                           \
    }

#define MDIO_STM32_DEVICE(n)                                    \
    PINCTRL_DT_INST_DEFINE(n);                                  \
    MDIO_STM32_CONFIG(n);                                       \
    static struct mdio_stm32_dev_data mdio_stm32_dev_data##n;   \
    DEVICE_DT_INST_DEFINE(n,                                    \
                          &mdio_stm32_initialize,               \
                          NULL,                                 \
                          &mdio_stm32_dev_data##n,              \
                          &mdio_stm32_dev_config_##n,           \
                          POST_KERNEL,                          \
                          CONFIG_MDIO_INIT_PRIORITY,            \
                          &mdio_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_STM32_DEVICE)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_mdio_stm32(void) {
    mdio_stm32_dev_config_0.heth.Instance = (ETH_TypeDef*)ut_mcu_eth_ptr;
}

void zephyr_gtest_mdio_stm32_init(const struct device* dev) {
    mdio_stm32_initialize(dev);
}

#endif


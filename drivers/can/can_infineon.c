/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/can.h>
#include "can_mcan.h"
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <infineon_kconfig.h>
#include <zephyr/drivers/clock_control/clock_control_ifx_cat1.h>
#include <zephyr/dt-bindings/clock/ifx_clock_source_common.h>

#include <cy_canfd.h>

LOG_MODULE_REGISTER(can_infineon, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT infineon_can

struct can_infineon_data {
    struct ifx_cat1_clock clock;
};

struct can_infineon_config {
    mm_reg_t base;
    mem_addr_t mrba;
    mem_addr_t mram;
    void (*config_irq)(void);
    const struct pinctrl_dev_config* pcfg;
    const struct device* ctrl_dev;
    en_clk_dst_t clk_dst;
};

/*
 * Wrapper config — defined here (before DT_DRV_COMPAT switch) so the
 * channel driver can access the parent wrapper's config.
 */
struct canfd_ifx_ctrl_config {
    CANFD_Type* base;

    #if defined(CONFIG_CAN_RX_TIMESTAMP)
    bool timestamp_counter;
    #endif /* CONFIG_CAN_RX_TIMESTAMP */

    bool ecc_enabled;
};

static int can_infineon_read_reg(const struct device* dev, uint16_t reg, uint32_t* val) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;

    return can_mcan_sys_read_reg(ifx_cfg->base, reg, val);
}

static int can_infineon_write_reg(const struct device* dev, uint16_t reg, uint32_t val) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;

    return can_mcan_sys_write_reg(ifx_cfg->base, reg, val);
}

static int can_infineon_read_mram(const struct device* dev, uint16_t offset, void* dst, size_t len) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;

    return can_mcan_sys_read_mram(ifx_cfg->mram, offset, dst, len);
}

static int can_infineon_write_mram(const struct device* dev, uint16_t offset, void const* src,
                                   size_t len) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;

    return can_mcan_sys_write_mram(ifx_cfg->mram, offset, src, len);
}

static int can_infineon_clear_mram(const struct device* dev, uint16_t offset, size_t len) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;

    return can_mcan_sys_clear_mram(ifx_cfg->mram, offset, len);
}

static int can_infineon_get_core_clock(const struct device* dev, uint32_t* rate) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;
    struct can_mcan_data* mcan_data = dev->data;
    struct can_infineon_data* data = mcan_data->custom;

    *rate = ifx_cat1_utils_peri_pclk_get_frequency(ifx_cfg->clk_dst, &data->clock);

    return (0);
}

static int can_infineon_init(const struct device* dev) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_infineon_config* ifx_cfg = mcan_cfg->custom;
    const struct can_mcan_data* mcan_data = dev->data;
    const struct can_infineon_data* data = mcan_data->custom;
    cy_rslt_t result;
    int ret;

    /* Ensure the parent controller (MRAM + channel clocks) is ready */
    if (!device_is_ready(ifx_cfg->ctrl_dev)) {
        LOG_ERR_DEVICE_NOT_READY(ifx_cfg->ctrl_dev);
        return (-ENODEV);
    }

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(ifx_cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret != 0) {
        LOG_ERR("CAN pinctrl setup failed (%d)", ret);
        return (ret);
    }

    /* Connect this CAN instance to the peripheral clock divider */
    result = ifx_cat1_utils_peri_pclk_assign_divider(ifx_cfg->clk_dst, &data->clock);
    if (result != CY_RSLT_SUCCESS) {
        LOG_ERR("CAN clock assign failed (%d)", (int)result);
        return (-EIO);
    }

    ret = can_mcan_configure_mram(dev, ifx_cfg->mrba, ifx_cfg->mram);
    if (ret != 0) {
        return (ret);
    }

    ret = can_mcan_init(dev);
    if (ret != 0) {
        LOG_ERR("can_mcan_init failed (%d)", ret);
        return (ret);
    }

    #ifdef CONFIG_CAN_RX_TIMESTAMP
    /*
     * If the parent controller has the shared timestamp counter enabled,
     * select the external timestamp source (TSS = 2) in the M_CAN core.
     * This must be done after can_mcan_init() which sets TSS = 1.
     */
    const struct canfd_ifx_ctrl_config* ctrl_cfg = ifx_cfg->ctrl_dev->config;

    if (ctrl_cfg->timestamp_counter) {
        ret = can_mcan_write_reg(dev, CAN_MCAN_TSCC, FIELD_PREP(CAN_MCAN_TSCC_TSS, 2U));
        if (ret != 0) {
            return (ret);
        }
    }
    #endif /* CONFIG_CAN_RX_TIMESTAMP */

    /* No need null check since config_irq is always provided */
    ifx_cfg->config_irq();

    return (0);
}

static DEVICE_API(can, can_infineon_driver_api) = {
    .get_capabilities = can_mcan_get_capabilities,
    .start            = can_mcan_start,
    .stop             = can_mcan_stop,
    .set_mode         = can_mcan_set_mode,
    .set_timing       = can_mcan_set_timing,
    .send             = can_mcan_send,
    .add_rx_filter    = can_mcan_add_rx_filter,
    .remove_rx_filter = can_mcan_remove_rx_filter,
    .get_state        = can_mcan_get_state,

    #ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
    .recover = can_mcan_recover,
    #endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE*/

    .get_core_clock            = can_infineon_get_core_clock,
    .get_max_filters           = can_mcan_get_max_filters,
    .set_state_change_callback = can_mcan_set_state_change_callback,
    .timing_min                = CAN_MCAN_TIMING_MIN_INITIALIZER,
    .timing_max                = CAN_MCAN_TIMING_MAX_INITIALIZER,

    #ifdef CONFIG_CAN_FD_MODE
    .set_timing_data = can_mcan_set_timing_data,
    .timing_data_min = CAN_MCAN_TIMING_DATA_MIN_INITIALIZER,
    .timing_data_max = CAN_MCAN_TIMING_DATA_MAX_INITIALIZER,
    #endif
};

static const struct can_mcan_ops can_infineon_ops = {
    .read_reg   = can_infineon_read_reg,
    .write_reg  = can_infineon_write_reg,
    .read_mram  = can_infineon_read_mram,
    .write_mram = can_infineon_write_mram,
    .clear_mram = can_infineon_clear_mram,
};

#if defined(CONFIG_SOC_FAMILY_INFINEON_EDGE)
#define CAN_PERI_CLOCK_INIT(n)                                  \
    .clock = {                                                  \
        .block = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(              \
            DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 0), \
            DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
            DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)),      \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel), \
    },
#else
#define CAN_PERI_CLOCK_INIT(n)                                  \
    .clock = {                                                  \
        .block = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(              \
            DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
            DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)),      \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel), \
    },
#endif

#define CAN_INFINEON_MCAN_INIT(n)                               \
    CAN_MCAN_DT_INST_BUILD_ASSERT_MRAM_CFG(n);                  \
    BUILD_ASSERT(CAN_MCAN_DT_INST_MRAM_ELEMENTS_SIZE(n) <= CAN_MCAN_DT_INST_MRAM_SIZE(n), \
                 "Insufficient Message RAM size to hold elements"); \
                                                                \
    PINCTRL_DT_INST_DEFINE(n);                                  \
    CAN_MCAN_DT_INST_CALLBACKS_DEFINE(n, can_infineon_cbs_##n); \
                                                                \
    static void infineon_mcan_irq_config_##n(void) {            \
        IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, int0, irq),          \
                    DT_INST_IRQ_BY_NAME(n, int0, priority), can_mcan_line_0_isr, \
                    DEVICE_DT_INST_GET(n), 0);                  \
        irq_enable(DT_INST_IRQ_BY_NAME(n, int0, irq));          \
        IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, int1, irq),          \
                    DT_INST_IRQ_BY_NAME(n, int1, priority), can_mcan_line_1_isr, \
                    DEVICE_DT_INST_GET(n), 0);                  \
        irq_enable(DT_INST_IRQ_BY_NAME(n, int1, irq));          \
    }                                                           \
                                                                \
    static struct can_infineon_data can_infineon_data_##n = {CAN_PERI_CLOCK_INIT(n)}; \
                                                                \
    static struct can_infineon_config DT_CONST can_infineon_cfg_##n = { \
        .base       = CAN_MCAN_DT_INST_MCAN_ADDR(n),            \
        .mrba       = CAN_MCAN_DT_INST_MRBA(n),                 \
        .mram       = CAN_MCAN_DT_INST_MRAM_ADDR(n),            \
        .config_irq = infineon_mcan_irq_config_##n,             \
        .pcfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(n),        \
        .ctrl_dev   = DEVICE_DT_GET(DT_INST_PARENT(n)),         \
        .clk_dst    = DT_INST_PROP(n, clk_dst),                 \
    };                                                          \
                                                                \
    static struct can_mcan_config DT_CONST can_mcan_cfg_##n = CAN_MCAN_DT_CONFIG_INST_GET( \
        n, &can_infineon_cfg_##n, &can_infineon_ops, &can_infineon_cbs_##n); \
                                                                \
    CAN_MCAN_DATA_DEFINE(can_mcan_data_##n, &can_infineon_data_##n); \
                                                                \
    CAN_DEVICE_DT_INST_DEFINE(n, can_infineon_init, NULL, &can_mcan_data_##n, \
                              &can_mcan_cfg_##n, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
                              &can_infineon_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_INFINEON_MCAN_INIT);

/*
 * Infineon CAN FD controller wrapper.
 *
 * The Infineon CAN implementation wraps the individual BOSCH M_CAN channels in a higher
 * level block.  The wrapper manages powering on the shared Message RAM and configures
 * other optional features, such as timestamp-counter and ECC.
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT infineon_canfd_controller

static int canfd_ifx_ctrl_init(const struct device* dev) {
    const struct canfd_ifx_ctrl_config* cfg = dev->config;

    /* Power on MRAM (clear CTL.MRAM_OFF) */
    cfg->base->CTL &= ~CANFD_CTL_MRAM_OFF_Msk;

    /*
     * Wait for MRAM power-up.  The PDL recommends 150 CPU cycles
     * (6 us at 25 MHz).  Using 10 us to be safe.
     */
    k_busy_wait(10U);

    #ifdef CONFIG_CAN_RX_TIMESTAMP
    /* Enable the shared timestamp counter if configured */
    if (cfg->timestamp_counter) {
        cfg->base->TS_CTL = CANFD_TS_CTL_ENABLED_Msk;
    }
    #endif /* CONFIG_CAN_RX_TIMESTAMP */

    /* Enable ECC if configured */
    if (cfg->ecc_enabled) {
        cfg->base->ECC_CTL = CANFD_ECC_CTL_ECC_EN_Msk;
    }

    return (0);
}

#define CANFD_IFX_CTRL_INIT(n)                                                                     \
    static struct canfd_ifx_ctrl_config DT_CONST canfd_ifx_ctrl_cfg_##n = {                        \
        .base = (CANFD_Type*)DT_INST_REG_ADDR_BY_NAME(n, wrapper),                                 \
        .ecc_enabled = DT_INST_PROP(n, ecc_enabled),                                               \
        IF_ENABLED(CONFIG_CAN_RX_TIMESTAMP,                                                        \
                   (.timestamp_counter = DT_INST_PROP(n, shared_timestamp_counter), ))};           \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, canfd_ifx_ctrl_init, NULL, NULL, &canfd_ifx_ctrl_cfg_##n,             \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(CANFD_IFX_CTRL_INIT)

#if (__GTEST == 1) /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT infineon_can

#define IFX_CAN_CFG_REG_INIT(n) \
    zephyr_gtest_can_ifx_reg_init(DEVICE_DT_GET(DT_DRV_INST(n)), \
                                  &can_infineon_data_##n, \
                                  &can_infineon_cfg_##n);

void zephyr_gtest_can_ifx_reg_init(const struct device* dev,
                                   struct can_infineon_data* data,
                                   struct can_infineon_config* cfg) {
    mem_addr_t base_addr = cfg->base;
    int rc;

    switch (base_addr) {
        case CANFD0_BASE : {
            cfg->base = (mem_addr_t)ut_mcu_canfd0_ch0_ptr;
            cfg->mrba = (mem_addr_t)ut_mcu_canfd0_ch0_mram_area;
            cfg->mram = (mem_addr_t)ut_mcu_canfd0_ch0_mram_area;
            break;
        }

        case (CANFD0_BASE + 0x200) : {
            cfg->base = (mem_addr_t)ut_mcu_canfd0_ch1_ptr;
            cfg->mrba = (mem_addr_t)ut_mcu_canfd0_ch1_mram_area;
            cfg->mram = (mem_addr_t)ut_mcu_canfd0_ch1_mram_area;
            break;
        }

        default: {
            break;
        }
    }

    rc = dev->ops.init(dev);
    if (rc == 0) {
        dev->state->initialized = true;
        dev->state->init_res = 0U;
    }
}

void zephyr_gtest_can_ifx(void) {
    struct device const* dev;
    int rc;

    canfd_ifx_ctrl_cfg_0.base = (CANFD_Type*)ut_mcu_canfd0_ptr;
    dev = DEVICE_DT_GET(DT_NODELABEL(canfd0));
    rc = canfd_ifx_ctrl_init(dev);
    if (rc == 0) {
        dev->state->initialized = true;
        dev->state->init_res = 0U;
    }

    DT_INST_FOREACH_STATUS_OKAY(IFX_CAN_CFG_REG_INIT);
}
#endif

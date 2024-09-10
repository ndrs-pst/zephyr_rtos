/*
 * Copyright (c) 2021 Alexander Wachter
 * Copyright (c) 2022 Kamil Serwus
 * Copyright (c) 2022 Vestas Wind Systems A/S
 * Copyright (c) 2022 NDR Solution (Thailand) Co., Ltd.
 * Copyright (c) 2023 Sebastian Schlupp
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_mcan.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(can_sam0, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT atmel_sam0_can

// ---------- EXTERNAL METHOD --------------------------------------------------------------------------------------- //
extern void can_mcan_state_change_handler(struct device const* dev);
extern void can_mcan_tx_event_handler(struct device const* dev);
extern void can_mcan_get_message(struct device const* dev, uint16_t fifo_offset,
                                 uint16_t fifo_status_reg, uint16_t fifo_ack_reg);

#define CAN_MCAN_IR_ISR_MSK_BIT     (CAN_MCAN_IR_BO   | CAN_MCAN_IR_EW   | CAN_MCAN_IR_EP |     \
                                     CAN_MCAN_IR_TEFL | CAN_MCAN_IR_TEFN |                      \
                                     CAN_MCAN_IR_ARA  | CAN_MCAN_IR_MRAF |                      \
                                     CAN_MCAN_IR_RF0N | CAN_MCAN_IR_RF1N |                      \
                                     CAN_MCAN_IR_RF0L | CAN_MCAN_IR_RF1L |                      \
                                     CAN_MCAN_IR_PEA  | CAN_MCAN_IR_PED)

// ---------- PRIVATE DATA DEFINITION ------------------------------------------------------------------------------- //
struct can_sam0_config {
    mm_reg_t base;
    mem_addr_t mram;
    void (*config_irq)(void);
    const struct pinctrl_dev_config* pcfg;
    volatile uint32_t* mclk;
    uint32_t mclk_mask;
    uint16_t gclk_core_id;
    int divider;
};

// ---------- PRIVATE PROGRAMMING DEFINE / CONSTEXPR ---------------------------------------------------------------- //
static int can_sam0_read_reg(struct device const* dev, uint16_t reg, uint32_t* val) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    ret = can_mcan_sys_read_reg(sam0_cfg->base, reg, val);

    return (ret);
}

static int can_sam0_write_reg(struct device const* dev, uint16_t reg, uint32_t val) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    switch (reg) {
        case CAN_MCAN_ILS :
            /* All interrupts are assigned to MCAN_INT0 */
            val = 0;
            break;

        case CAN_MCAN_ILE :
            /* SAM0 has only one line to handle interrupts */
            val = CAN_MCAN_ILE_EINT0;
            break;

        default :
            /* No field remap needed */
            break;
    }

    ret = can_mcan_sys_write_reg(sam0_cfg->base, reg, val);

    return (ret);
}

static int can_sam0_read_mram(struct device const* dev, uint16_t offset, void* dst, size_t len) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    ret = can_mcan_sys_read_mram(sam0_cfg->mram, offset, dst, len);

    return (ret);
}

static int can_sam0_write_mram(struct device const* dev, uint16_t offset, const void* src, size_t len) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    ret = can_mcan_sys_write_mram(sam0_cfg->mram, offset, src, len);

    return (ret);
}

static int can_sam0_clear_mram(struct device const* dev, uint16_t offset, size_t len) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    ret = can_mcan_sys_clear_mram(sam0_cfg->mram, offset, len);

    return (ret);
}

static void can_sam0_line_x_isr(struct device const* dev) {
    struct can_mcan_data* data = dev->data;
    struct can_mcan_config const* config = dev->config;
    uint32_t ir;
    int err;

    err = can_mcan_read_reg(dev, CAN_MCAN_IR, &ir);
    if (err != 0) {
        return;
    }

    // @see can_mcan_line_0_isr, can_mcan_line_1_isr
    while (true) {
        err = can_mcan_write_reg(dev, CAN_MCAN_IR, (ir & CAN_MCAN_IR_ISR_MSK_BIT));
        if (err != 0) {
            return;
        }

        if ((ir & (CAN_MCAN_IR_BO | CAN_MCAN_IR_EP | CAN_MCAN_IR_EW)) != 0UL) {
            // Bus_Off, Error Passive, Error Warning
            can_mcan_state_change_handler(dev);
        }

        // TX event FIFO new entry
        if ((ir & CAN_MCAN_IR_TEFN) != 0UL) {
            can_mcan_tx_event_handler(dev);
        }

        if ((ir & CAN_MCAN_IR_TEFL) != 0UL) {
            LOG_ERR("TX FIFO element lost");
            k_sem_give(&data->tx_sem);
        }

        if ((ir & CAN_MCAN_IR_ARA) != 0UL) {
            LOG_ERR("Access to reserved address");
        }

        if ((ir & CAN_MCAN_IR_MRAF) != 0UL) {
            LOG_ERR("Message RAM access failure");
        }

        #ifdef CONFIG_CAN_STATS
        if ((ir & (CAN_MCAN_IR_PEA | CAN_MCAN_IR_PED)) != 0U) {
            uint32_t reg;
            /* This function automatically updates protocol error stats */
            can_mcan_read_psr(dev, &reg);
        }
        #endif

        if ((ir & CAN_MCAN_IR_RF0N) != 0UL) {
            LOG_DBG("RX FIFO0 INT");
            can_mcan_get_message(dev, config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO0],
                                 CAN_MCAN_RXF0S, CAN_MCAN_RXF0A);
        }

        if ((ir & CAN_MCAN_IR_RF1N) != 0UL) {
            LOG_DBG("RX FIFO1 INT");
            can_mcan_get_message(dev, config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO1],
                                 CAN_MCAN_RXF1S, CAN_MCAN_RXF1A);
        }

        if ((ir & CAN_MCAN_IR_RF0L) != 0UL) {
            LOG_ERR("Message lost on FIFO0");
        }

        if ((ir & CAN_MCAN_IR_RF1L) != 0UL) {
            LOG_ERR("Message lost on FIFO1");
        }

        err = can_mcan_read_reg(dev, CAN_MCAN_IR, &ir);
        if (err != 0) {
            return;
        }

        if ((ir & CAN_MCAN_IR_ISR_MSK_BIT) == 0U) {
            // All of flag already handle then can break from this loop !!!
            break;
        }
    }
}

static int can_sam0_get_core_clock(struct device const* dev, uint32_t* rate) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;

    #if defined(CONFIG_SOC_SERIES_SAME51) || defined(CONFIG_SOC_SERIES_SAME54)
    /* DFFL has to be used as clock source for the ATSAME51/54 family of SoCs */
    *rate = SOC_ATMEL_SAM0_DFLL48_FREQ_HZ / (sam0_cfg->divider);
    #elif defined(CONFIG_SOC_SERIES_SAMC21)
    /* OSC48M has to be used as clock source for the ATSAMC21 family of SoCs */
    *rate = SOC_ATMEL_SAM0_OSC48M_FREQ_OUT_HZ / (sam0_cfg->divider);
    #endif

    return (0);
}

static void can_sam0_clock_enable(struct can_sam0_config const* cfg) {
    /* Enable the GLCK7 with DIV*/
    #if defined(CONFIG_SOC_SERIES_SAME51) || defined(CONFIG_SOC_SERIES_SAME54)
    /*DFFL has to be used as clock source for the ATSAME51/54 family of SoCs*/
    GCLK->GENCTRL[7].reg = (GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) |
                            GCLK_GENCTRL_DIV(cfg->divider) | GCLK_GENCTRL_GENEN);
    #elif defined(CONFIG_SOC_SERIES_SAMC21)
    /*OSC48M has to be used as clock source for the ATSAMC21 family of SoCs*/
    GCLK->GENCTRL[7].reg = (GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M) |
                            GCLK_GENCTRL_DIV(cfg->divider) | GCLK_GENCTRL_GENEN);
    #endif

    /* Route channel */
    GCLK->PCHCTRL[cfg->gclk_core_id].reg = (GCLK_PCHCTRL_GEN_GCLK7 | GCLK_PCHCTRL_CHEN);

    /* Enable CAN clock in MCLK */
    *cfg->mclk |= cfg->mclk_mask;
}

static int can_sam0_init(struct device const* dev) {
    struct can_mcan_config const* mcan_cfg = dev->config;
    struct can_sam0_config const* sam0_cfg = mcan_cfg->custom;
    int ret;

    can_sam0_clock_enable(sam0_cfg);

    ret = pinctrl_apply_state(sam0_cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("failed to apply pinctrl");
        return (ret);
    }

    ret = can_mcan_configure_mram(dev, 0U, sam0_cfg->mram);
    if (ret != 0) {
        LOG_ERR("failed to configure message ram");
        return (ret);
    }

    ret = can_mcan_init(dev);
    if (ret != 0) {
        LOG_ERR("failed to mcan init");
        return (ret);
    }

    sam0_cfg->config_irq();

    return (ret);
}

static const struct can_driver_api can_sam0_driver_api = {
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
    #endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
    .get_core_clock  = can_sam0_get_core_clock,
    .get_max_filters = can_mcan_get_max_filters,
    .set_state_change_callback = can_mcan_set_state_change_callback,
    .timing_min = CAN_MCAN_TIMING_MIN_INITIALIZER,
    .timing_max = CAN_MCAN_TIMING_MAX_INITIALIZER,
    #ifdef CONFIG_CAN_FD_MODE
    .set_timing_data = can_mcan_set_timing_data,
    .timing_data_min = CAN_MCAN_TIMING_DATA_MIN_INITIALIZER,
    .timing_data_max = CAN_MCAN_TIMING_DATA_MAX_INITIALIZER,
    #endif /* CONFIG_CAN_FD_MODE */
};

static struct can_mcan_ops const can_sam0_ops = {
    .read_reg   = can_sam0_read_reg,
    .write_reg  = can_sam0_write_reg,
    .read_mram  = can_sam0_read_mram,
    .write_mram = can_sam0_write_mram,
    .clear_mram = can_sam0_clear_mram
};

#define CAN_SAM0_IRQ_CFG_FUNCTION(inst)                                             \
static void config_can_##inst##_irq(void) {                                         \
    LOG_DBG("Enable CAN##inst## IRQ");                                              \
    IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, int0, irq),                               \
                DT_INST_IRQ_BY_NAME(inst, int0, priority), can_sam0_line_x_isr,     \
                                    DEVICE_DT_INST_GET(inst), 0);                   \
    irq_enable(DT_INST_IRQ_BY_NAME(inst, int0, irq));                               \
}

#define CAN_SAM0_CFG_INST(inst)                                                     \
    CAN_MCAN_DT_INST_CALLBACKS_DEFINE(inst, can_sam0_cbs_##inst);                   \
    CAN_MCAN_DT_INST_MRAM_DEFINE(inst, can_sam0_mram_##inst);                       \
                                                                                    \
    static struct can_sam0_config DT_CONST can_sam0_cfg_##inst = {                  \
        .base = CAN_MCAN_DT_INST_MCAN_ADDR(inst),                                   \
        .mram = (mem_addr_t)POINTER_TO_UINT(&can_sam0_mram_##inst),                 \
        .mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),               \
        .mclk_mask  = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),            \
        .gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch),         \
        .divider = DT_INST_PROP(inst, divider),                                     \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                               \
        .config_irq = config_can_##inst##_irq,                                      \
    };                                                                              \
                                                                                    \
static struct can_mcan_config DT_CONST can_mcan_cfg_##inst =                        \
    CAN_MCAN_DT_CONFIG_INST_GET(inst, &can_sam0_cfg_##inst, &can_sam0_ops,          \
                                &can_sam0_cbs_##inst);

#define CAN_SAM0_DATA_INST(inst)                                \
    static struct can_mcan_data can_mcan_data_##inst = CAN_MCAN_DATA_INITIALIZER(NULL);

#define CAN_SAM0_DEVICE_INST(inst)                              \
    CAN_DEVICE_DT_INST_DEFINE(inst, can_sam0_init, NULL,        \
                          &can_mcan_data_##inst,                \
                          &can_mcan_cfg_##inst,                 \
                          POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,\
                          &can_sam0_driver_api);

#define CAN_SAM0_INST(inst)                                     \
    CAN_MCAN_DT_INST_BUILD_ASSERT_MRAM_CFG(inst);               \
    PINCTRL_DT_INST_DEFINE(inst);                               \
    CAN_SAM0_IRQ_CFG_FUNCTION(inst)                             \
    CAN_SAM0_CFG_INST(inst)                                     \
    CAN_SAM0_DATA_INST(inst)                                    \
    CAN_SAM0_DEVICE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_SAM0_INST)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "samc21_reg_stub.h"

void zephyr_gtest_can_sam0(void) {
    can_sam0_cfg_0.base = (mm_reg_t)ut_mcu_can0_ptr;
}

#endif

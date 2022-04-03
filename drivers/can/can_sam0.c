/*
 * Copyright (c) 2021 Alexander Wachter
 * Copyright (c) 2022 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_mcan.h"
#include "can_mcan_int.h"

#include <drivers/can.h>
#include <drivers/can/transceiver.h>
#include <drivers/pinctrl.h>
#include <soc.h>
#include <kernel.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(can_sam0, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT atmel_sam0_can

// ---------- EXTERNAL METHOD --------------------------------------------------------------------------------------- //
extern void can_mcan_state_change_handler(const struct can_mcan_config* cfg, struct can_mcan_data* data);
extern void can_mcan_tc_event_handler(struct can_mcan_reg* can,
                                      struct can_mcan_msg_sram* msg_ram,
                                      struct can_mcan_data* data);
extern void can_mcan_get_message(struct can_mcan_data *data,
                                 volatile struct can_mcan_rx_fifo* fifo,
                                 volatile uint32_t* fifo_status_reg,
                                 volatile uint32_t* fifo_ack_reg);

#define CAN_MCAN_IR_ISR_MSK_BIT     (CAN_MCAN_IR_BO   | CAN_MCAN_IR_EW   | CAN_MCAN_IR_EP |     \
                                     CAN_MCAN_IR_TEFL | CAN_MCAN_IR_TEFN |                      \
                                     CAN_MCAN_IR_RF0N | CAN_MCAN_IR_RF1N |                      \
                                     CAN_MCAN_IR_RF0L | CAN_MCAN_IR_RF1L)

// ---------- PRIVATE DATA DEFINITION ------------------------------------------------------------------------------- //
struct can_sam0_config {
    struct can_mcan_config mcan_cfg;
    void (*config_irq)(void);
    const struct pinctrl_dev_config *pcfg;
    volatile uint32_t* mclk;
    uint32_t mclk_mask;
    uint16_t gclk_core_id;
};

struct can_sam0_data {
    struct can_mcan_data mcan_data;
    struct can_mcan_msg_sram msg_ram;
};

// ---------- PRIVATE PROGRAMMING DEFINE / CONSTEXPR ---------------------------------------------------------------- //
#define USE_CAN_SAM0_ISR_OPTIMIZE           1U

static int can_sam0_get_core_clock(const struct device* dev, uint32_t* rate) {
    *rate = SOC_ATMEL_SAM0_MCK_FREQ_HZ / (CONFIG_CAN_SAM0_CKDIV + 1);

    return (0);
}

static void can_sam0_set_state_change_callback(const struct device* dev,
                                              can_state_change_callback_t cb,
                                              void* user_data) {
    struct can_sam0_data* data = dev->data;

    data->mcan_data.state_change_cb_data = user_data;
    data->mcan_data.state_change_cb      = cb;
}

static void can_sam0_clock_enable(const struct can_sam0_config* cfg) {
    /* Enable the GCLK */
    GCLK->PCHCTRL[cfg->gclk_core_id].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

    /* Enable the MCLK */
    *cfg->mclk |= cfg->mclk_mask;
}

static int can_sam0_init(const struct device* dev) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    struct can_sam0_data* data             = dev->data;
    struct can_mcan_data* mcan_data        = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram      = &data->msg_ram;
    int ret;

    can_sam0_clock_enable(cfg);

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }

    ret = can_mcan_init(dev, mcan_cfg, msg_ram, mcan_data);
    if (ret) {
        return ret;
    }

    cfg->config_irq();

    return (ret);
}

static int can_sam0_get_state(const struct device* dev, enum can_state* state, struct can_bus_err_cnt* err_cnt) {
    const struct can_sam0_config* cfg;
    const struct can_mcan_config* mcan_cfg;
    int ret;

    cfg      = dev->config;
    mcan_cfg = &cfg->mcan_cfg;

    ret = can_mcan_get_state(mcan_cfg, state, err_cnt);

    return (ret);
}

static int can_sam0_send(const struct device* dev, const struct zcan_frame* frame,
                         k_timeout_t timeout, can_tx_callback_t callback, void* user_data) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    struct can_sam0_data* data             = dev->data;
    struct can_mcan_data* mcan_data        = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram      = &data->msg_ram;
    int ret;

    ret = can_mcan_send(mcan_cfg, mcan_data, msg_ram, frame, timeout, callback, user_data);

    return (ret);
}

static int can_sam0_add_rx_filter(const struct device* dev, can_rx_callback_t cb, void* cb_arg,
                                  const struct zcan_filter* filter) {
    struct can_sam0_data* data        = dev->data;
    struct can_mcan_data* mcan_data   = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram = &data->msg_ram;
    int ret;

    ret =  can_mcan_add_rx_filter(mcan_data, msg_ram, cb, cb_arg, filter);

    return (ret);
}

static void can_sam0_remove_rx_filter(const struct device* dev, int filter_id) {
    struct can_sam0_data* data        = dev->data;
    struct can_mcan_data* mcan_data   = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram = &data->msg_ram;

    can_mcan_remove_rx_filter(mcan_data, msg_ram, filter_id);
}

static int can_sam0_set_mode(const struct device* dev, enum can_mode mode) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    int ret;

    ret = can_mcan_set_mode(mcan_cfg, mode);

    return (ret);
}

static int can_sam0_set_timing(const struct device* dev, const struct can_timing* timing,
                               const struct can_timing* timing_data) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    int ret;

    ret = can_mcan_set_timing(mcan_cfg, timing, timing_data);

    return (ret);
}

int can_sam0_get_max_bitrate(const struct device* dev, uint32_t* max_bitrate) {
    const struct can_sam0_config* cfg = dev->config;

    *max_bitrate = cfg->mcan_cfg.max_bitrate;

    return (0);
}

#if (USE_CAN_SAM0_ISR_OPTIMIZE == 1U)
static void can_sam0_line_01_isr(const struct device* dev) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    struct can_sam0_data* data             = dev->data;
    struct can_mcan_data* mcan_data        = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram      = &data->msg_ram;
    struct can_mcan_reg* can               = mcan_cfg->can;
    uint32_t ir;

    // @see can_mcan_line_0_isr, can_mcan_line_1_isr
    while (true) {
        // Get interrupt flag and clear pending
        ir = can->ir;
        arch_nop();
        can->ir = ir;

        if ((ir & (CAN_MCAN_IR_BO | CAN_MCAN_IR_EP | CAN_MCAN_IR_EW)) != 0UL) {
            // Bus_Off, Error Passive, Error Warning
            can_mcan_state_change_handler(mcan_cfg, mcan_data);
        }

        // TX event FIFO new entry
        if ((ir & CAN_MCAN_IR_TEFN) != 0UL) {
            can_mcan_tc_event_handler(can, msg_ram, mcan_data);
        }

        if ((ir & CAN_MCAN_IR_TEFL) != 0UL) {
            LOG_ERR("TX FIFO element lost");
            k_sem_give(&mcan_data->tx_sem);
        }

        if ((ir & CAN_MCAN_IR_ARA) != 0UL) {
            LOG_ERR("Access to reserved address");
        }

        if ((ir & CAN_MCAN_IR_MRAF) != 0UL) {
            LOG_ERR("Message RAM access failure");
        }

        if ((ir & CAN_MCAN_IR_RF0N) != 0UL) {
            LOG_DBG("RX FIFO0 INT");
            can_mcan_get_message(mcan_data, msg_ram->rx_fifo0, &can->rxf0s, &can->rxf0a);
        }

        if ((ir & CAN_MCAN_IR_RF1N) != 0UL) {
            LOG_DBG("RX FIFO1 INT");
            can_mcan_get_message(mcan_data, msg_ram->rx_fifo1, &can->rxf1s, &can->rxf1a);
        }

        if ((ir & CAN_MCAN_IR_RF0L) != 0UL) {
            LOG_ERR("Message lost on FIFO0");
        }

        if ((ir & CAN_MCAN_IR_RF1L) != 0UL) {
            LOG_ERR("Message lost on FIFO1");
        }

        if ((can->ir & CAN_MCAN_IR_ISR_MSK_BIT) == 0U) {
            // All of flag already handle then can break from this loop !!!
            break;
        }
    };
}
#else
static void can_sam0_line_01_isr(const struct device* dev) {
    const struct can_sam0_config* cfg      = dev->config;
    const struct can_mcan_config* mcan_cfg = &cfg->mcan_cfg;
    struct can_sam0_data* data             = dev->data;
    struct can_mcan_data* mcan_data        = &data->mcan_data;
    struct can_mcan_msg_sram* msg_ram      = &data->msg_ram;

    can_mcan_line_0_isr(mcan_cfg, msg_ram, mcan_data);
    can_mcan_line_1_isr(mcan_cfg, msg_ram, mcan_data);
}
#endif

static const struct can_driver_api can_api_funcs = {
    .set_mode         = can_sam0_set_mode,
    .set_timing       = can_sam0_set_timing,
    .send             = can_sam0_send,
    .add_rx_filter    = can_sam0_add_rx_filter,
    .remove_rx_filter = can_sam0_remove_rx_filter,
    .get_state        = can_sam0_get_state,
    #ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    .recover          = can_mcan_recover,
    #endif
    .get_core_clock   = can_sam0_get_core_clock,
    .get_max_filters  = can_mcan_get_max_filters,
    .get_max_bitrate  = can_sam0_get_max_bitrate,
    .set_state_change_callback = can_sam0_set_state_change_callback,
    .timing_min = {
        .sjw        = 0x0001U,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0001U,
        .phase_seg2 = 0x0001U,
        .prescaler  = 0x0001U
    },
    .timing_max = {
        .sjw        = 0x007FU,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0100U,
        .phase_seg2 = 0x0080U,
        .prescaler  = 0x0200U
    },

    #ifdef CONFIG_CAN_FD_MODE
    .timing_min_data = {
        .sjw        = 0x0001U,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0001U,
        .phase_seg2 = 0x0001U,
        .prescaler  = 0x0001U
        },
    .timing_max_data = {
        .sjw        = 0x0010U,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0020U,
        .phase_seg2 = 0x0010U,
        .prescaler  = 0x0020U
        }
    #endif
};

#define CAN_SAM0_IRQ_CFG_FUNCTION(inst)                                             \
static void config_can_##inst##_irq(void) {                                         \
    LOG_DBG("Enable CAN##inst## IRQ");                                              \
    IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, line_01, irq),                            \
                DT_INST_IRQ_BY_NAME(inst, line_01, priority), can_sam0_line_01_isr,  \
                DEVICE_DT_INST_GET(inst), 0);                                       \
    irq_enable(DT_INST_IRQ_BY_NAME(inst, line_01, irq));                            \
}

#ifdef CONFIG_CAN_FD_MODE
#define CAN_SAM0_MCAN_CFG(inst) {                                                   \
    .can          = (struct can_mcan_reg *)DT_INST_REG_ADDR(inst),                  \
    .bus_speed    = DT_INST_PROP(inst, bus_speed),                                  \
    .sjw          = DT_INST_PROP(inst, sjw),                                        \
    .sample_point = DT_INST_PROP_OR(inst, sample_point, 0),                         \
    .prop_ts1     = DT_INST_PROP_OR(inst, prop_seg, 0) + DT_INST_PROP_OR(inst, phase_seg1, 0), \
    .ts2          = DT_INST_PROP_OR(inst, phase_seg2, 0),                           \
    .bus_speed_data = DT_INST_PROP(inst, bus_speed_data),                           \
    .sjw_data     = DT_INST_PROP(inst, sjw_data),                                   \
    .sample_point_data = DT_INST_PROP_OR(inst, sample_point_data, 0),               \
    .prop_ts1_data = DT_INST_PROP_OR(inst, prop_seg_data, 0) + DT_INST_PROP_OR(inst, phase_seg1_data, 0),   \
    .ts2_data     = DT_INST_PROP_OR(inst, phase_seg2_data, 0),                      \
    .tx_delay_comp_offset = DT_INST_PROP(inst, tx_delay_comp_offset),               \
    .phy          = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, phys)),             \
    .max_bitrate  = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(inst, 5000000),             \
}
#else /* CONFIG_CAN_FD_MODE */
#define CAN_SAM0_MCAN_CFG(inst) {                                                   \
    .can          = (struct can_mcan_reg *)DT_INST_REG_ADDR(inst),                  \
    .bus_speed    = DT_INST_PROP(inst, bus_speed),                                  \
    .sjw          = DT_INST_PROP(inst, sjw),                                        \
    .sample_point = DT_INST_PROP_OR(inst, sample_point, 0),                         \
    .prop_ts1     = DT_INST_PROP_OR(inst, prop_seg, 0) + DT_INST_PROP_OR(inst, phase_seg1, 0), \
    .ts2          = DT_INST_PROP_OR(inst, phase_seg2, 0),                           \
    .phy          = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, phys)),             \
    .max_bitrate  = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(inst, 1000000),             \
}
#endif /* CONFIG_CAN_FD_MODE */

#define CAN_SAM0_CFG_INST(inst)                                                     \
static const struct can_sam0_config can_sam0_cfg_##inst = {                         \
    .mcan_cfg   = CAN_SAM0_MCAN_CFG(inst),                                          \
    .config_irq = config_can_##inst##_irq,                                          \
    .pcfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                             \
    .mclk       = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),             \
    .mclk_mask  = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),                \
    .gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch)              \
};

#define CAN_SAM0_DATA_INST(inst) static struct can_sam0_data can_sam0_dev_data_##inst;

#define CAN_SAM0_DEVICE_INST(inst)                                                  \
DEVICE_DT_INST_DEFINE(inst, &can_sam0_init, NULL, &can_sam0_dev_data_##inst,        \
                      &can_sam0_cfg_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,  \
                      &can_api_funcs);

#define CAN_SAM0_INST(inst)                                     \
    PINCTRL_DT_INST_DEFINE(inst);                               \
    CAN_SAM0_IRQ_CFG_FUNCTION(inst)                             \
    CAN_SAM0_CFG_INST(inst)                                     \
    CAN_SAM0_DATA_INST(inst)                                    \
    CAN_SAM0_DEVICE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_SAM0_INST)

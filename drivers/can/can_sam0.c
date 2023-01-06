/*
 * Copyright (c) 2022 NDR Solution (Thailand) Co., Ltd.
 * Copyright (c) 2022 Vestas Wind Systems A/S
 * Copyright (c) 2021 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "can_mcan.h"
#include "can_mcan_priv.h"

LOG_MODULE_REGISTER(can_sam0, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT atmel_sam0_can

// ---------- EXTERNAL METHOD --------------------------------------------------------------------------------------- //
extern void can_mcan_state_change_handler(const struct device* dev);
extern void can_mcan_tc_event_handler(const struct device* dev);
extern void can_mcan_get_message(const struct device* dev,
                                 volatile struct can_mcan_rx_fifo* fifo,
                                 volatile uint32_t* fifo_status_reg,
                                 volatile uint32_t* fifo_ack_reg);

#define CAN_MCAN_IR_ISR_MSK_BIT     (CAN_MCAN_IR_BO   | CAN_MCAN_IR_EW   | CAN_MCAN_IR_EP |     \
                                     CAN_MCAN_IR_TEFL | CAN_MCAN_IR_TEFN |                      \
                                     CAN_MCAN_IR_RF0N | CAN_MCAN_IR_RF1N |                      \
                                     CAN_MCAN_IR_RF0L | CAN_MCAN_IR_RF1L)

// ---------- PRIVATE DATA DEFINITION ------------------------------------------------------------------------------- //
struct can_sam0_config {
    void (*config_irq)(void);
    const struct pinctrl_dev_config* pcfg;
    volatile uint32_t* mclk;
    uint32_t mclk_mask;
    uint16_t gclk_core_id;
};

struct can_sam0_data {
    struct can_mcan_msg_sram msg_ram;
};

// ---------- PRIVATE PROGRAMMING DEFINE / CONSTEXPR ---------------------------------------------------------------- //
#define USE_CAN_SAM0_ISR_OPTIMIZE           1U

static int can_sam0_get_core_clock(const struct device* dev, uint32_t* rate) {
    ARG_UNUSED(dev);

    *rate = SOC_ATMEL_SAM0_GCLK1_FREQ_HZ;

    return (0);
}

static void can_sam0_clock_enable(const struct can_sam0_config* cfg) {
    /* Enable the GCLK */
    GCLK->PCHCTRL[cfg->gclk_core_id].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;

    /* Enable the MCLK */
    *cfg->mclk |= cfg->mclk_mask;
}

static int can_sam0_init(const struct device* dev) {
    const struct can_mcan_config* mcan_cfg = dev->config;
    const struct can_sam0_config* sam_cfg  = mcan_cfg->custom;
    int ret;

    can_sam0_clock_enable(sam_cfg);

    ret = pinctrl_apply_state(sam_cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret == 0) {
        ret = can_mcan_init(dev);
        if (ret == 0) {
            sam_cfg->config_irq();
        }
    }

    return (ret);
}

#if (USE_CAN_SAM0_ISR_OPTIMIZE == 1U)
static void can_sam0_line_01_isr(const struct device* dev) {
    const struct can_mcan_config* cfg = dev->config;
    struct can_mcan_data* data        = dev->data;
    struct can_mcan_msg_sram* msg_ram = data->msg_ram;
    struct can_mcan_reg* can          = cfg->can;
    uint32_t ir;

    // @see can_mcan_line_0_isr, can_mcan_line_1_isr
    while (true) {
        // Get interrupt flag and clear pending
        ir = can->ir;
        arch_nop();
        can->ir = ir;

        if ((ir & (CAN_MCAN_IR_BO | CAN_MCAN_IR_EP | CAN_MCAN_IR_EW)) != 0UL) {
            // Bus_Off, Error Passive, Error Warning
            can_mcan_state_change_handler(dev);
        }

        // TX event FIFO new entry
        if ((ir & CAN_MCAN_IR_TEFN) != 0UL) {
            can_mcan_tc_event_handler(dev);
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

        if ((ir & CAN_MCAN_IR_RF0N) != 0UL) {
            LOG_DBG("RX FIFO0 INT");
            can_mcan_get_message(dev, msg_ram->rx_fifo0, &can->rxf0s, &can->rxf0a);
        }

        if ((ir & CAN_MCAN_IR_RF1N) != 0UL) {
            LOG_DBG("RX FIFO1 INT");
            can_mcan_get_message(dev, msg_ram->rx_fifo1, &can->rxf1s, &can->rxf1a);
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
    can_mcan_line_0_isr(dev);
    can_mcan_line_1_isr(dev);
}
#endif

static const struct can_driver_api can_sam0_driver_api = {
    .get_capabilities = can_mcan_get_capabilities,
    .set_mode         = can_mcan_set_mode,
    .start            = can_mcan_start,
    .stop             = can_mcan_stop,
    .set_timing       = can_mcan_set_timing,
    .send             = can_mcan_send,
    .add_rx_filter    = can_mcan_add_rx_filter,
    .remove_rx_filter = can_mcan_remove_rx_filter,
    .get_state        = can_mcan_get_state,
    #ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    .recover          = can_mcan_recover,
    #endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
    .get_core_clock   = can_sam0_get_core_clock,
    .get_max_filters  = can_mcan_get_max_filters,
    .get_max_bitrate  = can_mcan_get_max_bitrate,
    .set_state_change_callback = can_mcan_set_state_change_callback,
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
    .set_timing_data = can_mcan_set_timing_data,
    .timing_data_min = {
        .sjw        = 0x0001U,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0001U,
        .phase_seg2 = 0x0001U,
        .prescaler  = 0x0001U
        },
    .timing_data_max = {
        .sjw        = 0x0010U,
        .prop_seg   = 0x0000U,
        .phase_seg1 = 0x0020U,
        .phase_seg2 = 0x0010U,
        .prescaler  = 0x0020U
        }
    #endif /* CONFIG_CAN_FD_MODE */
};

#define CAN_SAM0_IRQ_CFG_FUNCTION(inst)                                             \
static void config_can_##inst##_irq(void) {                                         \
    LOG_DBG("Enable CAN##inst## IRQ");                                              \
    IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, line_01, irq),                            \
                DT_INST_IRQ_BY_NAME(inst, line_01, priority), can_sam0_line_01_isr, \
                DEVICE_DT_INST_GET(inst), 0);                                       \
    irq_enable(DT_INST_IRQ_BY_NAME(inst, line_01, irq));                            \
}

#define CAN_SAM0_CFG_INST(inst)                                                     \
static struct can_sam0_config DT_CONST can_sam0_cfg_##inst = {                      \
    .config_irq = config_can_##inst##_irq,                                          \
    .pcfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                             \
    .mclk       = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),             \
    .mclk_mask  = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),                \
    .gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch)              \
};                                                                                  \
                                                                                    \
static struct can_mcan_config DT_CONST can_mcan_cfg_##inst =                        \
    CAN_MCAN_DT_CONFIG_INST_GET(inst, &can_sam0_cfg_##inst);

#define CAN_SAM0_DATA_INST(inst)                                \
    static struct can_sam0_data can_sam0_data_##inst;           \
                                                                \
    static struct can_mcan_data can_mcan_data_##inst =          \
        CAN_MCAN_DATA_INITIALIZER(&can_sam0_data_##inst.msg_ram,\
                                  &can_sam0_data_##inst);       \

#define CAN_SAM0_DEVICE_INST(inst)                              \
    DEVICE_DT_INST_DEFINE(inst, &can_sam0_init, NULL,           \
                          &can_mcan_data_##inst,                \
                          &can_mcan_cfg_##inst,                 \
                          POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,\
                          &can_sam0_driver_api);

#define CAN_SAM0_INST(inst)                                     \
    PINCTRL_DT_INST_DEFINE(inst);                               \
    CAN_SAM0_IRQ_CFG_FUNCTION(inst)                             \
    CAN_SAM0_CFG_INST(inst)                                     \
    CAN_SAM0_DATA_INST(inst)                                    \
    CAN_SAM0_DEVICE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_SAM0_INST)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "samc21_reg_stub.h"
void zephyr_gtest_can_sam0(void) {
    can_mcan_cfg_0.can = (struct can_mcan_reg*)ut_mcu_can0_ptr;
}

#endif


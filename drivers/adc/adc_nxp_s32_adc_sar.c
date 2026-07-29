/*
 * Copyright 2023-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <Adc_Sar_Ip.h>
#include <Adc_Sar_Ip_Irq.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define DT_DRV_COMPAT nxp_s32_adc_sar
LOG_MODULE_REGISTER(adc_nxp_s32_adc_sar, CONFIG_ADC_LOG_LEVEL);

/* Define the hardware register size when calculating bit positions */
#define ADC_SAR_IP_HW_REG_SIZE 32

/* Definitions to compute bit positions from channel index */
#define ADC_SAR_IP_CHAN_2_BIT(CHNIDX)       ((CHNIDX) % ADC_SAR_IP_HW_REG_SIZE)

/* Convert channel of group ADC to channel of physical ADC instance */
#define ADC_NXP_S32_GROUPCHAN_2_PHYCHAN(group, channel) \
    ((ADC_SAR_IP_HW_REG_SIZE * group) + channel)

/* Bitmask of channel_ids (== each channel@N node's `reg`) that have a DT node */
#define ADC_NXP_S32_CHAN_BIT_ENTRY(node_id) | BIT(DT_REG_ADDR(node_id))
#define ADC_NXP_S32_CHAN_MASK(n) \
    (0 DT_INST_FOREACH_CHILD_STATUS_OKAY(n, ADC_NXP_S32_CHAN_BIT_ENTRY))

/* Resolved physical ADC channel number for a channel@N node: its own
 * `channel-number` if given, else <parent's group-channel index> * 32 + reg
 * (today's single-group behavior, where reg already doubles as the
 * group-local index). */
#define ADC_NXP_S32_RESOLVE_PHYCHAN(node_id, n) \
    DT_PROP_OR(node_id, channel_number, \
              ADC_NXP_S32_GROUPCHAN_2_PHYCHAN(DT_INST_ENUM_IDX(n, group_channel), \
                                              DT_REG_ADDR(node_id)))

/* channel_id (reg) -> resolved physical channel number */
#define ADC_NXP_S32_CHAN_PHY_ENTRY(node_id, n) \
    [DT_REG_ADDR(node_id)] = ADC_NXP_S32_RESOLVE_PHYCHAN(node_id, n),
#define ADC_NXP_S32_CHAN_PHY_TABLE(n) \
    { DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, ADC_NXP_S32_CHAN_PHY_ENTRY, n) }

/* Zephyr's adc_sequence.channels is a 32-bit bitmask, so each channel@N's
 * reg (channel_id) must stay within 0-31. */
#define ADC_NXP_S32_CHAN_REG_CHECK(node_id) \
    BUILD_ASSERT(DT_REG_ADDR(node_id) < ADC_SAR_IP_HW_REG_SIZE, \
                 "ADC channel reg (channel_id) must be < 32");

/* channel-number (physical) must stay within this instance's channel space,
 * i.e. group count * channels-per-group. */
#define ADC_NXP_S32_CHAN_NUM_CHECK(node_id, n) \
    BUILD_ASSERT(ADC_NXP_S32_RESOLVE_PHYCHAN(node_id, n) < \
                (ADC_SAR_IP_HW_REG_SIZE * ADC_SAR_IP_NUM_GROUP_CHAN), \
                "ADC channel-number is out of range for this instance");

/* The per-channel end-of-conversion callback only receives the physical
 * channel id from the HAL and relies on channel_id == PhysicalChanId % 32
 * plus ascending-physical == ascending-channel_id completion order. Both
 * only hold when no channel decouples channel-number from the node-level
 * group mapping, so such channels require "normal-end-chain", which drains
 * results by channel_id and needs no reverse translation. */
#define ADC_NXP_S32_CHAN_EOC_CHECK(node_id, n) \
    BUILD_ASSERT((DT_INST_ENUM_IDX(n, callback_select) == 1) || \
                 (ADC_NXP_S32_RESOLVE_PHYCHAN(node_id, n) == \
                  ADC_NXP_S32_GROUPCHAN_2_PHYCHAN(DT_INST_ENUM_IDX(n, group_channel), \
                                                  DT_REG_ADDR(node_id))), \
                 "channel-number override requires callback-select \"normal-end-chain\"");

struct adc_nxp_s32_config {
    ADC_Type* base;
    uint8_t instance;
    uint8_t callback_select;
    uint32_t chan_defined_mask;
    const uint8_t* chan_phy_map;
    Adc_Sar_Ip_ConfigType* adc_cfg;
    void (*irq_config_func)(const struct device* dev);
    const struct pinctrl_dev_config* pin_cfg;
};

struct adc_nxp_s32_data {
    const struct device* dev;
    struct adc_context ctx;
    uint16_t* buffer;
    uint16_t* buf_end;
    uint16_t* repeat_buffer;
    uint32_t mask_channels;
};

static int adc_nxp_s32_init(const struct device* dev) {
    const struct adc_nxp_s32_config* config = dev->config;
    struct adc_nxp_s32_data* data = dev->data;
    Adc_Sar_Ip_StatusType status;

    if (config->pin_cfg) {
        if (pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT)) {
            return (-EIO);
        }
    }

    status = Adc_Sar_Ip_Init(config->instance, config->adc_cfg);
    if (status) {
        return (-EIO);
    }

    #if FEATURE_ADC_HAS_CALIBRATION
    status = Adc_Sar_Ip_DoCalibration(config->instance);
    if (status) {
        return (-EIO);
    }
    #endif

    Adc_Sar_Ip_EnableNotifications(config->instance,
                                   config->callback_select ? ADC_SAR_IP_NOTIF_FLAG_NORMAL_ENDCHAIN
                                                           : ADC_SAR_IP_NOTIF_FLAG_NORMAL_EOC);

    data->dev = dev;
    config->irq_config_func(dev);

    adc_context_unlock_unconditionally(&data->ctx);

    return (0);
}

static int adc_nxp_s32_channel_setup(const struct device* dev,
                                     const struct adc_channel_cfg* channel_cfg) {
    const struct adc_nxp_s32_config* config = dev->config;
    uint32_t chan_mask_bit;

    chan_mask_bit = config->chan_defined_mask & BIT(channel_cfg->channel_id);
    if (chan_mask_bit == 0U) {
        LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
        return (-EINVAL);
    }

    if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
        LOG_ERR("Unsupported channel acquisition time");
        return (-ENOTSUP);
    }

    if (channel_cfg->differential) {
        LOG_ERR("Differential channels are not supported");
        return (-ENOTSUP);
    }

    if (channel_cfg->gain != ADC_GAIN_1) {
        LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
        return (-ENOTSUP);
    }

    if (channel_cfg->reference != ADC_REF_INTERNAL) {
        LOG_ERR("Unsupported channel reference");
        return (-ENOTSUP);
    }

    return (0);
}

static int adc_nxp_s32_validate_buffer_size(const struct device* dev,
                                            const struct adc_sequence* sequence) {
    size_t active_channels;
    size_t needed_size;

    active_channels = POPCOUNT(sequence->channels);
    needed_size = active_channels * sizeof(uint16_t);
    if (sequence->options) {
        needed_size *= (1 + sequence->options->extra_samplings);
    }

    if (sequence->buffer_size < needed_size) {
        return (-ENOMEM);
    }

    return (0);
}

#if FEATURE_ADC_HAS_AVERAGING
static int adc_nxp_s32_set_averaging(const struct device* dev, uint8_t oversampling) {
    const struct adc_nxp_s32_config* config = dev->config;
    Adc_Sar_Ip_AvgSelectType avg_sel = ADC_SAR_IP_AVG_4_CONV;
    bool avg_en = true;

    switch (oversampling) {
        case 0 :
            avg_en = false;
            break;

        case 2 :
            avg_sel = ADC_SAR_IP_AVG_4_CONV;
            break;

        case 3 :
            avg_sel = ADC_SAR_IP_AVG_8_CONV;
            break;

        case 4 :
            avg_sel = ADC_SAR_IP_AVG_16_CONV;
            break;

        case 5 :
            avg_sel = ADC_SAR_IP_AVG_32_CONV;
            break;

        default :
            LOG_ERR("Unsupported oversampling value");
            return (-ENOTSUP);
    }
    Adc_Sar_Ip_SetAveraging(config->instance, avg_en, avg_sel);

    return (0);
}
#endif

#if (ADC_SAR_IP_SET_RESOLUTION == STD_ON)
static int adc_nxp_s32_set_resolution(const struct device* dev, uint8_t adc_resol) {
    const struct adc_nxp_s32_config* config = dev->config;
    Adc_Sar_Ip_Resolution resolution;

    switch (adc_resol) {
        case 8 :
            resolution = ADC_SAR_IP_RESOLUTION_8;
            break;

        case 10 :
            resolution = ADC_SAR_IP_RESOLUTION_10;
            break;

        case 12 :
            resolution = ADC_SAR_IP_RESOLUTION_12;
            break;

        case 14 :
            resolution = ADC_SAR_IP_RESOLUTION_14;
            break;

        default :
            LOG_ERR("Unsupported resolution");
            return (-ENOTSUP);
    }
    Adc_Sar_Ip_SetResolution(config->instance, resolution);

    return (0);
}
#endif

static int adc_nxp_s32_start_read_async(const struct device* dev,
                                        const struct adc_sequence* sequence) {
    const struct adc_nxp_s32_config* config = dev->config;
    struct adc_nxp_s32_data* data = dev->data;
    int ret;
    uint32_t mask;

    mask = sequence->channels & ~config->chan_defined_mask;
    if (mask != 0U) {
        LOG_ERR("Channels out of bit map");
        return (-EINVAL);
    }

    ret = adc_nxp_s32_validate_buffer_size(dev, sequence);
    if (ret) {
        LOG_ERR("Buffer size isn't enough");
        return (-EINVAL);
    }

    #if FEATURE_ADC_HAS_AVERAGING
    ret = adc_nxp_s32_set_averaging(dev, sequence->oversampling);
    if (ret) {
        return (-ENOTSUP);
    }
    #else
    if (sequence->oversampling) {
        LOG_ERR("Oversampling can't be changed");
        return (-ENOTSUP);
    }
    #endif

    #if (ADC_SAR_IP_SET_RESOLUTION == STD_ON)
    ret = adc_nxp_s32_set_resolution(dev, sequence->resolution);
    if (ret) {
        return (-ENOTSUP);
    }
    #else
    if (sequence->resolution != ADC_SAR_IP_MAX_RESOLUTION) {
        LOG_ERR("Resolution can't be changed");
        return (-ENOTSUP);
    }
    #endif

    if (sequence->calibrate) {
        #if FEATURE_ADC_HAS_CALIBRATION
        ret = Adc_Sar_Ip_DoCalibration(config->instance);
        if (ret) {
            LOG_ERR("Error during calibration");
            return (-EIO);
        }
        #else
        LOG_ERR("Unsupported calibration");
        return (-ENOTSUP);
        #endif
    }

    uint32_t chan_mask_bit = config->chan_defined_mask;
    while (chan_mask_bit != 0U) {
        size_t chan_id = find_lsb_set(chan_mask_bit) - 1;
        size_t channel = config->chan_phy_map[chan_id];

        chan_mask_bit &= ~BIT(chan_id);
        if (sequence->channels & BIT(chan_id)) {
            Adc_Sar_Ip_EnableChannelNotifications(config->instance,
                                                  channel, ADC_SAR_IP_CHAN_NOTIF_EOC);
            Adc_Sar_Ip_EnableChannel(config->instance,
                                     ADC_SAR_IP_CONV_CHAIN_NORMAL, channel);
        }
        else {
            Adc_Sar_Ip_DisableChannelNotifications(config->instance,
                                                   channel, ADC_SAR_IP_CHAN_NOTIF_EOC);
            Adc_Sar_Ip_DisableChannel(config->instance,
                                      ADC_SAR_IP_CONV_CHAIN_NORMAL, channel);
        }
    }

    /* Save ADC sequence sampling buffer and its end pointer address */
    data->buffer = sequence->buffer;
    if (config->callback_select) {
        data->buf_end = data->buffer + (sequence->buffer_size / sizeof(uint16_t));
    }

    adc_context_start_read(&data->ctx, sequence);
    ret = adc_context_wait_for_completion(&data->ctx);

    return (ret);
}

static void adc_context_start_sampling(struct adc_context* ctx) {
    struct adc_nxp_s32_data* data = CONTAINER_OF(ctx, struct adc_nxp_s32_data, ctx);
    const struct adc_nxp_s32_config* config = data->dev->config;

    data->mask_channels = ctx->sequence.channels;
    data->repeat_buffer = data->buffer;

    Adc_Sar_Ip_StartConversion(config->instance, ADC_SAR_IP_CONV_CHAIN_NORMAL);
}

static void adc_context_update_buffer_pointer(struct adc_context* ctx,
                                              bool repeat_sampling) {
    struct adc_nxp_s32_data* const data =
        CONTAINER_OF(ctx, struct adc_nxp_s32_data, ctx);

    if (repeat_sampling) {
        data->buffer = data->repeat_buffer;
    }
}

static int adc_nxp_s32_read_async(const struct device* dev,
                                  const struct adc_sequence* sequence,
                                  struct k_poll_signal* async) {
    struct adc_nxp_s32_data* data = dev->data;
    int ret;

    adc_context_lock(&data->ctx, async ? true : false, async);
    ret = adc_nxp_s32_start_read_async(dev, sequence);
    adc_context_release(&data->ctx, ret);

    return (ret);
}

static int adc_nxp_s32_read(const struct device* dev,
                            const struct adc_sequence* sequence) {
    return adc_nxp_s32_read_async(dev, sequence, NULL);
}

static void adc_nxp_s32_isr(const struct device* dev) {
    const struct adc_nxp_s32_config* config = dev->config;

    Adc_Sar_Ip_IRQHandler(config->instance);
}

#define ADC_NXP_S32_DRIVER_API(n)                               \
    static DEVICE_API(adc, adc_nxp_s32_driver_api_##n) = {      \
        .channel_setup = adc_nxp_s32_channel_setup,             \
        .read          = adc_nxp_s32_read,                      \
        IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_nxp_s32_read_async, )) \
        .ref_internal = DT_INST_PROP(n, vref_mv),               \
    };

#define ADC_NXP_S32_IRQ_CONFIG(n)                               \
    static void adc_nxp_s32_adc_sar_config_func_##n(const struct device* dev) { \
        IRQ_CONNECT(DT_INST_IRQN(n),                            \
                    DT_INST_IRQ(n, priority),                   \
                    adc_nxp_s32_isr, DEVICE_DT_INST_GET(n), 0); \
        irq_enable(DT_INST_IRQN(n));                            \
    };

#define ADC_NXP_S32_CALLBACK_DEFINE(n)                          \
    void adc_nxp_s32_normal_end_conversion_callback##n(const uint16 PhysicalChanId) { \
        const struct device* dev = DEVICE_DT_INST_GET(n);       \
        struct adc_nxp_s32_data* data = dev->data;              \
        uint16_t result;                                        \
                                                                \
        result = Adc_Sar_Ip_GetConvData(n, PhysicalChanId);     \
        LOG_DBG("End conversion, channel %d, group %d, result = %d", \
                ADC_SAR_IP_CHAN_2_BIT(PhysicalChanId),          \
                PhysicalChanId / ADC_SAR_IP_HW_REG_SIZE, result); \
                                                                \
        *data->buffer++ = result;                               \
        data->mask_channels &= ~BIT(ADC_SAR_IP_CHAN_2_BIT(PhysicalChanId)); \
                                                                \
        if (!data->mask_channels) {                             \
            adc_context_on_sampling_done(&data->ctx, dev);      \
        }                                                       \
    };                                                          \
                                                                \
    void adc_nxp_s32_normal_endchain_callback##n(void) {        \
        const struct device* dev = DEVICE_DT_INST_GET(n);       \
        const struct adc_nxp_s32_config* config = dev->config;  \
        struct adc_nxp_s32_data* data = dev->data;              \
                                                                \
        while (data->mask_channels) {                           \
            size_t chan_id = (find_lsb_set(data->mask_channels) - 1); \
            size_t channel = config->chan_phy_map[chan_id];     \
            uint16_t result = Adc_Sar_Ip_GetConvData(n, channel); \
                                                                \
            LOG_DBG("End chain, channel %zu, group %zu, result = %d", \
                    chan_id, (channel / ADC_SAR_IP_HW_REG_SIZE), result); \
                                                                \
            if (data->buffer < data->buf_end) {                 \
                *data->buffer++ = result;                       \
            }                                                   \
            data->mask_channels &= ~BIT(chan_id);               \
        }                                                       \
                                                                \
        adc_context_on_sampling_done(&data->ctx, dev);          \
    };

#define ADC_NXP_S32_INSTANCE_CHECK(indx, n) \
    ((DT_INST_REG_ADDR(n) == IP_ADC_##indx##_BASE) ? indx : 0)

#if defined(_MSC_VER) /* #CUSTOM@NDRS */
#define ADC_NXP_S32_GET_INSTANCE(n) (n)
#else
#define ADC_NXP_S32_GET_INSTANCE(n) \
    LISTIFY(__DEBRACKET ADC_INSTANCE_COUNT, ADC_NXP_S32_INSTANCE_CHECK, (|), n)
#endif

#if (FEATURE_ADC_HAS_HIGH_SPEED_ENABLE == 1U)
#define ADC_NXP_S32_HIGH_SPEED_CFG(n) .HighSpeedConvEn = DT_INST_PROP(n, high_speed),
#else
#define ADC_NXP_S32_HIGH_SPEED_CFG(n)
#endif

#if (ADC_SAR_IP_SET_RESOLUTION == STD_ON)
#define ADC_NXP_S32_RESOLUTION_CFG(n) .AdcResolution = ADC_SAR_IP_RESOLUTION_14,
#else
#define ADC_NXP_S32_RESOLUTION_CFG(n)
#endif

#define ADC_NXP_S32_INIT_DEVICE(n)                                              \
    ADC_NXP_S32_DRIVER_API(n)                                                   \
    ADC_NXP_S32_CALLBACK_DEFINE(n)                                              \
    ADC_NXP_S32_IRQ_CONFIG(n)                                                   \
    COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n),                                  \
                (PINCTRL_DT_INST_DEFINE(n);), (EMPTY))                          \
                                                                                \
    DT_INST_FOREACH_CHILD_STATUS_OKAY(n, ADC_NXP_S32_CHAN_REG_CHECK)            \
    DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, ADC_NXP_S32_CHAN_NUM_CHECK, n)   \
    DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, ADC_NXP_S32_CHAN_EOC_CHECK, n)   \
                                                                                \
    static const uint8_t adc_nxp_s32_chan_phy_map_##n[ADC_SAR_IP_HW_REG_SIZE] = \
        ADC_NXP_S32_CHAN_PHY_TABLE(n);                                          \
                                                                                \
    static Adc_Sar_Ip_ConfigType const adc_nxp_s32_default_config##n = {        \
        .ConvMode = ADC_SAR_IP_CONV_MODE_ONESHOT,                               \
        ADC_NXP_S32_RESOLUTION_CFG(n)                                           \
        ADC_NXP_S32_HIGH_SPEED_CFG(n)                                           \
        .EndOfNormalChainNotification =                                         \
            adc_nxp_s32_normal_endchain_callback##n,                            \
        .EndOfConvNotification =                                                \
            adc_nxp_s32_normal_end_conversion_callback##n,                      \
        .ClkSelect = ADC_SAR_IP_CLK_HALF_BUS,                                   \
        .CalibrationClkSelect = ADC_SAR_IP_CLK_QUARTER_BUS,                     \
    };                                                                          \
                                                                                \
    static struct adc_nxp_s32_data adc_nxp_s32_data_##n = {                     \
        ADC_CONTEXT_INIT_TIMER(adc_nxp_s32_data_##n, ctx),                      \
        ADC_CONTEXT_INIT_LOCK(adc_nxp_s32_data_##n, ctx),                       \
        ADC_CONTEXT_INIT_SYNC(adc_nxp_s32_data_##n, ctx),                       \
    };                                                                          \
                                                                                \
    static struct adc_nxp_s32_config DT_CONST adc_nxp_s32_config_##n = {        \
        .base              = (ADC_Type*)DT_INST_REG_ADDR(n),                    \
        .instance          = ADC_NXP_S32_GET_INSTANCE(n),                       \
        .chan_defined_mask = ADC_NXP_S32_CHAN_MASK(n),                          \
        .chan_phy_map      = adc_nxp_s32_chan_phy_map_##n,                      \
        .callback_select   = DT_INST_ENUM_IDX(n, callback_select),              \
        .adc_cfg           = (Adc_Sar_Ip_ConfigType*)&adc_nxp_s32_default_config##n, \
        .irq_config_func   = adc_nxp_s32_adc_sar_config_func_##n,               \
        .pin_cfg           = COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n),         \
                                       (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL)), \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n,                                                    \
                          adc_nxp_s32_init,                                     \
                          NULL,                                                 \
                          &adc_nxp_s32_data_##n,                                \
                          &adc_nxp_s32_config_##n,                              \
                          POST_KERNEL,                                          \
                          CONFIG_ADC_INIT_PRIORITY,                             \
                          &adc_nxp_s32_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_NXP_S32_INIT_DEVICE)

#if (__GTEST == 1) /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

#define S32_ADC_CFG_REG_INIT(n)                                                 \
    zephyr_gtest_adc_s32k3_reg_init(DEVICE_DT_GET(DT_DRV_INST(n)), &adc_nxp_s32_data_##n,\
                                    &adc_nxp_s32_config_##n);

static void zephyr_gtest_adc_s32k3_reg_init(const struct device* dev, struct adc_nxp_s32_data* data,
                                            struct adc_nxp_s32_config* cfg) {
    ARG_UNUSED(data);
    uintptr_t base_addr = (uintptr_t)cfg->base;
    int rc;

    switch (base_addr) {
        case 0x400A0000UL : /* IP_ADC_0_BASE */ {
            cfg->base = (ADC_Type*)ut_mcu_adc_0_area;
            cfg->instance = 0U;
            break;
        }

        case 0x400A4000UL : /* IP_ADC_1_BASE */ {
            cfg->base = (ADC_Type*)ut_mcu_adc_1_area;
            cfg->instance = 1U;
            break;
        }

        default: { /* IP_ADC_2_BASE */
            cfg->base = (ADC_Type *)ut_mcu_adc_2_area;
            cfg->instance = 2U;
            break;
        }
    }

    rc = dev->ops.init(dev);
    if (rc == 0) {
        dev->state->initialized = true;
        dev->state->init_res = 0U;
    }
}

void zephyr_gtest_adc_s32k3(void) {
    DT_INST_FOREACH_STATUS_OKAY(S32_ADC_CFG_REG_INIT)
}
#endif

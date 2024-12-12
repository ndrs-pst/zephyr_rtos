/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_sdhc

#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

/* STM32 includes */
#include <stm32_ll_rcc.h>
#include <stm32_ll_sdmmc.h>

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

static const uint32_t sdhc_stm32_dma_tbl_pri[] = {
	DMA_PRIORITY_LOW,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH
};

struct sdmmc_dma_stream {
	const struct device *dev;
	uint32_t channel;
	uint32_t channel_nb;
	DMA_TypeDef *reg;
	struct dma_config cfg;
};

struct sdhc_stm32_config {
	SDIO_TypeDef *sdio;
	void (*irq_config)(const struct device *port);
	struct k_sem thread_lock;
	struct k_sem sync;
	int status;
	struct k_work work;
	struct gpio_callback cd_cb;
	struct gpio_dt_spec cd;
	struct gpio_dt_spec pe;
	struct gpio_dt_spec pwr_gpio;
	struct stm32_pclken *pclken;
	const struct pinctrl_dev_config *pcfg;
	const struct reset_dt_spec reset;

	struct sdhc_host_props props;
};

struct sdhc_stm32_data {
	uint8_t bus_width;
	uint32_t bus_clock;

	enum sdhc_power power_mode;
	enum sdhc_timing_mode timing;

	struct k_mutex s_request_mutex;

#if STM32_SDMMC_USE_DMA
	struct sdmmc_dma_stream dma_rx;
	struct sdmmc_dma_stream dma_tx;
#endif
};

static int sdhc_stm32_reset(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;

	/* Reset carried out successfully */
	return 0;
}

/*
 * Set SDHC io properties
 */
static int sdhc_stm32_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	uint8_t bus_width;
	int ret = 0;

	LOG_INF("SDHC I/O: slot: %d, bus width %d, clock %dHz, card power %s, voltage %s",
		cfg->slot, ios->bus_width, ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (ios->clock > 0) {
		/* pass */
	}

	if (ios->bus_width > 0) {
		/* Set bus width */
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			bus_width = 1;
			break;
		case SDHC_BUS_WIDTH4BIT:
			bus_width = 4;
			break;
		default:
			return -ENOTSUP;
		}

		if (ctx->bus_width != bus_width) {
			ctx->bus_width = bus_width;
		}
	}

	/* Toggle card power supply */
	if ((ctx->power_mode != ios->power_mode) && (cfg->pwr_gpio.port)) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			gpio_pin_set_dt(&cfg->pwr_gpio, 0);
		} else if (ios->power_mode == SDHC_POWER_ON) {
			gpio_pin_set_dt(&cfg->pwr_gpio, 1);
		}

		ctx->power_mode = ios->power_mode;
	}

	if (ios->timing > 0) {
		/* Set I/O timing */
		if (ctx->timing != ios->timing) {
			switch (ios->timing) {
			case SDHC_TIMING_LEGACY:
			case SDHC_TIMING_HS:

				break;

			case SDHC_TIMING_DDR50:
			case SDHC_TIMING_DDR52:
				/* Enable DDR mode */

				LOG_INF("DDR mode enabled");
				break;

			case SDHC_TIMING_SDR12:
			case SDHC_TIMING_SDR25:

				break;

			case SDHC_TIMING_SDR50:
			case SDHC_TIMING_HS400:
			case SDHC_TIMING_SDR104:
			case SDHC_TIMING_HS200:
			default:
				LOG_ERR("Timing mode not supported for this device");
				ret = -ENOTSUP;
				break;
			}

			LOG_INF("Bus timing successfully changed to %s", timingStr[ios->timing]);
			ctx->timing = ios->timing;
		}
	}

	return ret;
}

/*
 * Return 0 if card is not busy, 1 if it is
 */
static int sdhc_stm32_card_busy(const struct device *dev)
{
	return 0;
}

/*
 * Send CMD or CMD/DATA via SDHC
 */
static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *data)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	unsigned int retries = (cmd->retries + 1);
	int ret;

	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
		break;

	case SD_APP_CMD:
	case SD_SEND_STATUS:
	case SD_SET_BLOCK_SIZE:
		break;

	case SD_SEND_IF_COND:
		break;

	case SD_APP_SEND_OP_COND:
		break;

	case SDIO_RW_DIRECT:
		break;

	case SDIO_SEND_OP_COND:
		break;

	case SD_ALL_SEND_CID:
		break;

	case SD_SEND_RELATIVE_ADDR:
		break;

	case SD_SEND_CSD:
		break;

	case SD_SELECT_CARD:
		break;

	case SD_APP_SEND_SCR:
	case SD_SWITCH:
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
	case SD_APP_SEND_NUM_WRITTEN_BLK:
		break;

	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		break;

	default:
		LOG_INF("SDHC driver: command %u not supported", cmd->opcode);
		return -ENOTSUP;
	}

	while (retries > 0) {
		/* do_transaction */
	}

	/* Interpret response */
	ret = 0;

	return (ret);
}

/*
 * Get card presence
 */
static int sdhc_stm32_get_card_present(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	int ret;

	if (!cfg->cd.port) {
		return 1;
	}

	ret = gpio_pin_get_dt(&cfg->cd);
	if (ret < 0) {
		LOG_WRN("reading card detect failed %d", ret);
		return -EIO;
	}

	return ret;
}

/*
 * Get host properties
 */
static int sdhc_stm32_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_stm32_config *cfg = dev->config;

	memcpy(props, &cfg->props, sizeof(struct sdhc_host_props));

	return 0;
}

/*
 * Perform early system init for SDHC
 */
static int sdhc_stm32_init(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure SDHC pins");
		return -EINVAL;
	}

	/* Reset controller */
	sdhc_stm32_reset(dev);

	return 0;
}

static DEVICE_API(sdhc, sdhc_stm32_api) = {
	.reset = sdhc_stm32_reset,
	.request = sdhc_stm32_request,
	.set_io = sdhc_stm32_set_io,
	.get_card_present = sdhc_stm32_get_card_present,
	.card_busy = sdhc_stm32_card_busy,
	.get_host_props = sdhc_stm32_get_host_props
};

#define SDHC_STM32_INIT(n)                                                                         \
                                                                                                   \
	PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                         \
	K_MSGQ_DEFINE(sdhc##n##_queue, sizeof(struct sdmmc_event), SDMMC_EVENT_QUEUE_LENGTH, 1);   \
                                                                                                   \
	static const struct sdhc_stm32_config sdhc_stm32_##n##_config = {                          \
		.sdmmc = (const sdmmc_dev_t *)DT_REG_ADDR(DT_INST_PARENT(n)),                      \
		.irq_source = DT_IRQN(DT_INST_PARENT(n)),                                          \
		.slot = DT_REG_ADDR(DT_DRV_INST(n)),                                               \
		.bus_width_cfg = DT_INST_PROP(n, bus_width),                                       \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),                                 \
		.pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_gpios, {0}),                           \
		.props = {.is_spi = false,                                                         \
			  .f_max = DT_INST_PROP(n, max_bus_freq),                                  \
			  .f_min = DT_INST_PROP(n, min_bus_freq),                                  \
			  .max_current_330 = DT_INST_PROP(n, max_current_330),                     \
			  .max_current_180 = DT_INST_PROP(n, max_current_180),                     \
			  .power_delay = DT_INST_PROP_OR(n, power_delay_ms, 0),                    \
			  .host_caps = {.vol_180_support = false,                                  \
					.vol_300_support = false,                                  \
					.vol_330_support = true,                                   \
					.suspend_res_support = false,                              \
					.sdma_support = true,                                      \
					.high_spd_support =                                        \
						(DT_INST_PROP(n, bus_width) == 4) ? true : false,  \
					.adma_2_support = false,                                   \
					.max_blk_len = 0,                                          \
					.ddr50_support = false,                                    \
					.sdr104_support = false,                                   \
					.sdr50_support = false,                                    \
					.bus_8_bit_support = false,                                \
					.bus_4_bit_support =                                       \
						(DT_INST_PROP(n, bus_width) == 4) ? true : false,  \
					.hs200_support = false,                                    \
					.hs400_support = false}}};                                 \
                                                                                                   \
	static struct sdhc_stm32_data sdhc_stm32_##n##_data = {                                    \
		.bus_width = SDMMC_SLOT_WIDTH_DEFAULT,                                             \
		.bus_clock = (SDMMC_FREQ_PROBING * 1000),                                          \
		.power_mode = SDHC_POWER_ON,                                                       \
		.timing = SDHC_TIMING_LEGACY,                                                      \
		.s_host_ctx = {.event_queue = &sdhc##n##_queue}};                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sdhc_stm32_init, NULL, &sdhc_stm32_##n##_data,                    \
			      &sdhc_stm32_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_STM32_INIT)

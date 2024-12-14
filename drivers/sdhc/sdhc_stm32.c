/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_sdhc

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <soc.h>

/* STM32 includes */
#include <stm32_ll_rcc.h>
#include <stm32_ll_sdmmc.h>

LOG_MODULE_REGISTER(stm32_sdhc, CONFIG_SDHC_LOG_LEVEL);

// #define STM32_SDHC_USE_DMA DT_NODE_HAS_PROP(DT_DRV_INST(0), dmas)
#define STM32_SDHC_USE_DMA      1

#if STM32_SDHC_USE_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <stm32_ll_dma.h>
#endif

#if STM32_SDHC_USE_DMA

static const uint32_t sdhc_stm32_dma_tbl_pri[] = {
	DMA_PRIORITY_LOW,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH
};

struct sdhc_dma_stream {
	const struct device *dev;
	uint32_t channel;
	uint32_t channel_nb;
	DMA_TypeDef *reg;
	struct dma_config cfg;
};
#endif

struct sdhc_stm32_config {
	void (*irq_config)(const struct device *dev);
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
	SDIO_TypeDef *sdio;
	SDIO_HandleTypeDef hsdio;
	struct k_sem thread_lock;
	struct k_sem sync;
	int status;
	struct k_work work;

	enum sdhc_power power_mode;
	enum sdhc_timing_mode timing;

	uint32_t ercd; /*!< SDIO Card Error codes */
	HAL_SDIO_StateTypeDef state;
	uint32_t xfer_ctx;
	bool use_dma;

#if STM32_SDHC_USE_DMA
	struct sdhc_dma_stream dma_rx;
	struct sdhc_dma_stream dma_tx;
	DMA_HandleTypeDef dma_tx_handle;
	DMA_HandleTypeDef dma_rx_handle;
#endif
};

#ifdef CONFIG_SDHC_STM32_HWFC
static void stm32_sdmmc_fc_enable(struct stm32_sdmmc_priv *priv)
{
	MMC_TypeDef *sdmmcx = priv->hsd.Instance;

	sdmmcx->CLKCR |= SDMMC_CLKCR_HWFC_EN;
}
#endif

static void stm32_sdmmc_isr(const struct device *dev)
{
	struct sdhc_stm32_data *data = dev->data;

	HAL_SDIO_IRQHandler(&data->hsdio);
}

#define DEFINE_HAL_CALLBACK(name)                                                                  \
	void name(SDIO_HandleTypeDef *hsdio)                                                       \
	{                                                                                          \
		struct sdhc_stm32_data *data = CONTAINER_OF(hsdio, struct sdhc_stm32_data, hsdio); \
                                                                                                   \
		data->status = hsdio->ErrorCode;                                                   \
                                                                                                   \
		k_sem_give(&data->sync);                                                           \
	}

DEFINE_HAL_CALLBACK(HAL_SDIO_TxCpltCallback);
DEFINE_HAL_CALLBACK(HAL_SDIO_RxCpltCallback);
DEFINE_HAL_CALLBACK(HAL_SDIO_ErrorCallback);

static int sdhc_stm32_clock_enable(struct sdhc_stm32_data *data)
{
	const struct device *clock;

	/* HSI48 Clock is enabled through using the device tree */
	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (DT_INST_NUM_CLOCKS(0) > 1) {
		if (clock_control_configure(clock,
					    (clock_control_subsys_t)&data->pclken[1],
					    NULL) != 0) {
			LOG_ERR("Failed to enable SDHC domain clock");
			return -EIO;
		}
	}

	if (IS_ENABLED(CONFIG_SDHC_STM32_CLOCK_CHECK)) {
		uint32_t sdhc_clock_rate;

		if (clock_control_get_rate(clock,
					   (clock_control_subsys_t)&data->pclken[1],
					   &sdhc_clock_rate) != 0) {
			LOG_ERR("Failed to get SDHC domain clock rate");
			return -EIO;
		}

		if (sdhc_clock_rate != MHZ(48)) {
			LOG_ERR("SDHC Clock is not 48MHz (%d)", sdhc_clock_rate);
			return -ENOTSUP;
		}
	}

	/* Enable the APB clock for stm32_sdhc */
	return clock_control_on(clock, (clock_control_subsys_t)&data->pclken[0]);
}

static int sdhc_stm32_clock_disable(struct sdhc_stm32_data *data)
{
	const struct device *clock;

	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	return clock_control_off(clock,
				 (clock_control_subsys_t)&data->pclken);
}

#if STM32_SDHC_USE_DMA

static void sdhc_stm32_dma_cb(const struct device *dev, void *arg,
			      uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	}

	HAL_DMA_IRQHandler(hdma);
}

static int sdhc_stm32_configure_dma(DMA_HandleTypeDef *handle, struct sdhc_dma_stream *dma)
{
	int ret;

	if (!device_is_ready(dma->dev)) {
		LOG_ERR("Failed to get dma dev");
		return -ENODEV;
	}

	dma->cfg.user_data = handle;

	ret = dma_config(dma->dev, dma->channel, &dma->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to conig");
		return ret;
	}

	handle->Instance                 = __LL_DMA_GET_STREAM_INSTANCE(dma->reg, dma->channel_nb);
	handle->Init.Channel             = dma->cfg.dma_slot * DMA_CHANNEL_1;
	handle->Init.PeriphInc           = DMA_PINC_DISABLE;
	handle->Init.MemInc              = DMA_MINC_ENABLE;
	handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	handle->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	handle->Init.Mode                = DMA_PFCTRL;
	handle->Init.Priority            = table_priority[dma->cfg.channel_priority],
	handle->Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	handle->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	handle->Init.MemBurst            = DMA_MBURST_INC4;
	handle->Init.PeriphBurst         = DMA_PBURST_INC4;

	return ret;
}

static int sdhc_stm32_dma_init(struct sdhc_stm32_data *data)
{
	int err;

	LOG_DBG("using dma");

	err = sdhc_stm32_configure_dma(&data->dma_tx_handle, &data->dma_tx);
	if (err) {
		LOG_ERR("failed to init tx dma");
		return err;
	}
	__HAL_LINKDMA(&data->hsd, hdmatx, data->dma_tx_handle);
	HAL_DMA_Init(&data->dma_tx_handle);

	err = sdhc_stm32_configure_dma(&data->dma_rx_handle, &data->dma_rx);
	if (err) {
		LOG_ERR("failed to init rx dma");
		return err;
	}
	__HAL_LINKDMA(&data->hsd, hdmarx, data->dma_rx_handle);
	HAL_DMA_Init(&data->dma_rx_handle);

	return err;
}

static int sdhc_stm32_dma_deinit(struct sdhc_stm32_data *data)
{
	int ret;
	struct sdmmc_dma_stream *dma_tx = &data->dma_tx;
	struct sdmmc_dma_stream *dma_rx = &data->dma_rx;

	ret = dma_stop(dma_tx->dev, dma_tx->channel);
	HAL_DMA_DeInit(&data->dma_tx_handle);
	if (ret != 0) {
		LOG_ERR("Failed to stop tx DMA transmission");
		return ret;
	}
	ret = dma_stop(dma_rx->dev, dma_rx->channel);
	HAL_DMA_DeInit(&data->dma_rx_handle);
	if (ret != 0) {
		LOG_ERR("Failed to stop rx DMA transmission");
		return ret;
	}
	return ret;
}
#endif

static int sdhc_stm32_snd_cmd(SDIO_TypeDef *sdio, struct sdhc_command *cmd, uint32_t response)
{
	SDMMC_CmdInitTypeDef cmd_init int ret;

	cmd_init.Argument = cmd->arg;
	cmd_init.CmdIndex = cmd->opcode;
	cmd_init.Response = response;
	cmd_init.WaitForInterrupt = SDMMC_WAIT_NO;
	cmd_init.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(sdio, &cmd_init);

	ret = sdhc_stm32_get_resp(sdio, cmd);
}

static uint8_t sdhc_stm32_cnv_blk_sz(uint32_t blk_sz)
{
	uint8_t most_bit = (uint8_t)__CLZ(__RBIT(blk_sz));

	/*(1 << most_bit) - 1) is the mask used for blocksize*/
	if (((uint8_t)blk_sz & ((1U << most_bit) - 1U)) != 0U) {
		return (uint8_t)SDMMC_DATABLOCK_SIZE_4B;
	}

	return most_bit << SDMMC_DCTRL_DBLOCKSIZE_Pos;
}

static void sdhc_stm32_cfg_data(SDIO_TypeDef *sdio, uint32_t dat_len, uint32_t blk_sz, uint32_t dir)
{
	SDMMC_DataInitTypeDef config;

	/* Configure the SD DPSM (Data Path State Machine) */
	config.DataTimeOut   = SDMMC_DATATIMEOUT;
	config.DataLength    = dat_len;
	config.DataBlockSize = blk_sz;
	config.TransferDir   = dir;
	config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
	config.DPSM          = SDMMC_DPSM_DISABLE;
	(void)SDMMC_ConfigData(sdio, &config);

	__SDMMC_CMDTRANS_ENABLE(sdio);
}

static int sdhc_stm32_get_resp(SDIO_TypeDef *sdio, struct sdhc_command *cmd)
{
	int timeout_ms = cmd->timeout_ms;

	while (true) {
		uint32_t sta_reg = sdio->STA;

		if (((sta_reg & (SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT |
				 SDMMC_FLAG_BUSYD0END)) != 0U) &&
		    ((sta_reg & SDMMC_FLAG_CMDACT) == 0U)) {
			break;
		}

		k_msleep(1);
		if (timeout_ms-- == 0) {
			return -ETIMEDOUT;
		}
	}

	if (__SDMMC_GET_FLAG(sdio, SDMMC_FLAG_CTIMEOUT)) {
		__SDMMC_CLEAR_FLAG(sdio, SDMMC_FLAG_CTIMEOUT);
		return -ETIMEDOUT;
	} else if (__SDMMC_GET_FLAG(sdio, SDMMC_FLAG_CCRCFAIL)) {
		__SDMMC_CLEAR_FLAG(sdio, SDMMC_FLAG_CCRCFAIL);
		return -EIO;
	} else {
		/* Nothing to do */
	}

	/* Clear all the static flags */
	__SDMMC_CLEAR_FLAG(sdio, SDMMC_STATIC_CMD_FLAGS);

	/* Check response received is of desired command */
	if (SDMMC_GetCommandResponse(sdio) != (uint8_t)cmd->opcode) {
		return -EIO;
	}

	/* We have received response, retrieve it for analysis  */
	cmd->response[0] = SDMMC_GetResponse(sdio, SDMMC_RESP1);
	if (cmd->response_type == SD_SPI_RSP_TYPE_R2) {
		cmd->response[1] = SDMMC_GetResponse(sdio, SDMMC_RESP2);
		cmd->response[2] = SDMMC_GetResponse(sdio, SDMMC_RESP3);
		cmd->response[3] = SDMMC_GetResponse(sdio, SDMMC_RESP4);
	}

	return 0;
}

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

static int sdhc_stm32_req_ll(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *data)
{
	struct sdhc_stm32_data *ctx = dev->data;
	SDMMC_CmdInitTypeDef cmd_init;
	uint32_t err_state;
	int ret;

	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
		err_state = SDMMC_CmdGoIdleState(ctx->sdio);
		if (err_state != HAL_SD_ERROR_NONE) {
			return -EIO;
		}
		break;

	case SDIO_SEND_OP_COND:
		err_state = SDMMC_CmdSendOperationcondition(ctx->sdio, cmd->arg, cmd->response);
		if (err_state != HAL_SD_ERROR_NONE) {
			return -EIO;
		}
		break;

	case SD_ALL_SEND_CID: /* SDMMC_CmdSendCID */
	case SD_SEND_CSD:     /* SDMMC_CmdSendCSD */
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_LONG);
		break;

	case SD_SEND_RELATIVE_ADDR: /* SDMMC_CmdSetRelAdd */
	case SD_SELECT_CARD:        /* SDMMC_CmdSelDesel */
	case SD_SEND_IF_COND:       /* SDMMC_CmdSendEXTCSD */
	case SD_VOL_SWITCH:         /* SDMMC_CmdVoltageSwitch */
	case SD_SEND_STATUS:        /* SDMMC_CmdSendStatus */
	case SD_SET_BLOCK_SIZE:     /* SDMMC_CmdBlockLength */
	case SD_APP_SEND_OP_COND:   /* SDMMC_CmdAppOperCommand */
	case SD_APP_CMD:            /* SDMMC_CmdAppCommand */
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_SHORT);
		break;

	case SD_APP_SEND_NUM_WRITTEN_BLK:
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_SHORT);
		// TBA
		break;

	case SDIO_RW_DIRECT: { /* HAL_SDIO_ReadDirect */
		uint8_t resp5;

		err_state = SDMMC_SDIO_CmdReadWriteDirect(ctx->sdio, cmd->arg, &resp5);
		if (err_state != HAL_SD_ERROR_NONE) {
			ctx->ercd |= err_state;
			if (err_state !=
			    (SDMMC_ERROR_ADDR_OUT_OF_RANGE | SDMMC_ERROR_ILLEGAL_CMD |
			     SDMMC_ERROR_COM_CRC_FAILED | SDMMC_ERROR_GENERAL_UNKNOWN_ERR)) {
				/* Clear all the static flags */
				__SDMMC_CLEAR_FLAG(ctx->sdio, SDMMC_STATIC_DATA_FLAGS);
				ctx->state = HAL_SDIO_STATE_READY;
				ctx->xfer_ctx = SDIO_CONTEXT_NONE;
				return -EIO;
			}
			return -EIO;
		}

		cmd->response[0] = resp5;
		__SDMMC_CMDTRANS_DISABLE(ctx->sdio);

		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(ctx->sdio, SDMMC_STATIC_DATA_FLAGS);
		break;
	}

	case SDIO_RW_EXTENDED: {
		SDMMC_DataInitTypeDef config;
		uint32_t dir;
		uint32_t arg;
		uint32_t mode;
		uint32_t dat_len;
		uint32_t blk_sz;

		arg = cmd->arg;

		if (arg & BIT(SDIO_EXTEND_CMD_ARG_BLK_SHIFT)) {
			dat_len = (uint32_t)(data->blocks * data->block_size);
			blk_sz = sdhc_stm32_cnv_blk_sz(data->block_size);
			mode = SDMMC_TRANSFER_MODE_BLOCK;
		} else {
			dat_len = data->block_size;
			blk_sz = SDMMC_DATABLOCK_SIZE_1B;
			mode = SDMMC_TRANSFER_MODE_SDIO;
		}

		if (arg & BIT(SDIO_CMD_ARG_RW_SHIFT)) {
			dir = SDMMC_TRANSFER_DIR_TO_CARD;
		} else {
			dir = SDMMC_TRANSFER_DIR_TO_SDMMC;
		}

		/* Configure the SD DPSM (Data Path State Machine) */
		config.DataTimeOut   = SDMMC_DATATIMEOUT;
		config.DataLength    = dat_len;
		config.DataBlockSize = blk_sz;
		config.TransferDir   = dir;
		config.TransferMode  = mode;
		config.DPSM          = SDMMC_DPSM_DISABLE;
		(void)SDMMC_ConfigData(ctx->sdio, &config);

		__SDMMC_CMDTRANS_ENABLE(ctx->sdio);

		err_state = SDMMC_SDIO_CmdReadWriteExtended(ctx->sdio, arg);
		if (err_state != HAL_SD_ERROR_NONE) {
			ctx->ercd |= err_state;
			if (err_state !=
			    (SDMMC_ERROR_ADDR_OUT_OF_RANGE | SDMMC_ERROR_ILLEGAL_CMD |
			     SDMMC_ERROR_COM_CRC_FAILED | SDMMC_ERROR_GENERAL_UNKNOWN_ERR)) {
				/* Clear all the static flags */
				__SDMMC_CLEAR_FLAG(ctx->sdio, SDMMC_STATIC_DATA_FLAGS);
				ctx->state = HAL_SDIO_STATE_READY;
				ctx->xfer_ctx = SDIO_CONTEXT_NONE;
				return -EIO;
			}
			return -EIO;
		}
		__SDMMC_CMDTRANS_DISABLE(ctx->sdio);

		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(ctx->sdio, SDMMC_STATIC_DATA_FLAGS);
		break;
	}

	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK: {
		if (cmd->opcode == SD_READ_SINGLE_BLOCK) {
			ctx->xfer_ctx = SD_CONTEXT_READ_SINGLE_BLOCK;
		} else {
			ctx->xfer_ctx = SD_CONTEXT_READ_MULTIPLE_BLOCK;
		}

		sdhc_stm32_cfg_data(ctx->sdio, data->blocks * BLOCKSIZE, SDMMC_DATABLOCK_SIZE_512B,
				    SDMMC_TRANSFER_DIR_TO_SDMMC);
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			__SDMMC_ENABLE_IT(ctx->sdio, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						      SDMMC_IT_RXOVERR | SDMMC_IT_DATAEND |
						      SDMMC_FLAG_RXFIFOHF));
		}
		break;
	}

	case SD_APP_SEND_SCR: /* sdmmc_read_scr */
	case SD_SWITCH:       /* sdmmc_switch */
		// TBA
		break;

	case SD_APP_SEND_NUM_WRITTEN_BLK: {
		sdhc_stm32_cfg_data(ctx->sdio, 4, SDMMC_DATABLOCK_SIZE_4B,
				    SDMMC_TRANSFER_DIR_TO_SDMMC);
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			__SDMMC_ENABLE_IT(ctx->sdio, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						      SDMMC_IT_RXOVERR | SDMMC_IT_DATAEND |
						      SDMMC_FLAG_RXFIFOHF));
		}
		break;
	}

	case SD_WRITE_SINGLE_BLOCK:   /* SDMMC_CmdWriteSingleBlock */
	case SD_WRITE_MULTIPLE_BLOCK: /* SDMMC_CMD_WRITE_MULT_BLOCK */
		if (cmd->opcode == SD_WRITE_SINGLE_BLOCK) {
			ctx->xfer_ctx = SD_CONTEXT_WRITE_SINGLE_BLOCK;
		} else {
			ctx->xfer_ctx = SD_CONTEXT_WRITE_MULTIPLE_BLOCK;
		}

		sdhc_stm32_cfg_data(ctx->sdio, data->blocks * BLOCKSIZE, SDMMC_DATABLOCK_SIZE_512B,
				    SDMMC_TRANSFER_DIR_TO_CARD);
		ret = sdhc_stm32_snd_cmd(ctx->sdio, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			if (ctx->use_dma) {
				ctx->xfer_ctx |= SD_CONTEXT_DMA;
				__SDMMC_ENABLE_IT(ctx->sdio,
						  (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						   SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND));
			} else {
				ctx->xfer_ctx |= SD_CONTEXT_IT;
				__SDMMC_ENABLE_IT(ctx->sdio,
						  (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						   SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND |
						   SDMMC_FLAG_TXFIFOHE));
			}
		}
		break;

	case SD_STOP_TRANSMISSION: /* SDMMC_CmdStopTransfer */
		// TBA
		break;

	default:
		LOG_INF("SDHC driver: command %u not supported", cmd->opcode);
		return -ENOTSUP;
	}

	/* Interpret response */
	ret = 0;

	return (ret);
}

/*
 * Send CMD or CMD/DATA via SDHC
 */
static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *data)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	int busy_timeout = 5000;
	uint32_t err_state;
	int ret;

	do {
		ret = sdhc_stm32_req_ll(dev, cmd, data);
		if (data && (ret || data->blocks > 1)) {
			// FIXME sdhc_stm32_abort(dev);
			while (busy_timeout > 0) {
				if (!sdhc_stm32_card_busy(dev)) {
					break;
				}
				k_busy_wait(125);
				busy_timeout -= 125;
			}

			if (busy_timeout <= 0) {
				LOG_ERR("Card did not idle after CMD12");
				ret = -ETIMEDOUT;
			}
		}
	} while ((ret != 0) && (cmd->retries-- > 0));

	return ret;
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

	err = sdhc_stm32_clock_enable(ctx);
	if (err) {
		LOG_ERR("failed to init clocks");
		return err;
	}

	/* init carrier detect (if set) */
	if (cfg->cd.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->cd)) {
			LOG_ERR("GPIO port for cd pin is not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->cd, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Couldn't configure cd pin: (%d)", ret);
			return ret;
		}
	}

#if STM32_SDHC_USE_DMA
	err = sdhc_stm32_dma_init(ctx);
	if (err) {
		LOG_ERR("DMA init failed");
		return err;
	}
#endif

	err = reset_line_toggle_dt(&ctx->reset);
	if (err) {
		LOG_ERR("failed to reset peripheral");
		return err;
	}

	err = HAL_SDIO_Init(&ctx->hsdio);
	if (err != HAL_OK) {
		LOG_ERR("failed to init sdhc_stm32 (ErrorCode 0x%X)", ctx->hsdio.ErrorCode);
		return -EIO;
	}

#ifdef CONFIG_SDHC_STM32_HWFC
	sdhc_stm32_fc_enable(ctx);
#endif

	/* Reset controller */
	sdhc_stm32_reset(dev);

	return 0;
}

#define STM32_SDHC_IRQ_CONNECT(id)                  \
	COND_CODE_1(DT_INST_PROP(id, stpm3x_isr),   \
		(IRQ_CONNECT(DT_INST_IRQN(id),      \
				DT_INST_IRQ(id, priority),     \
                     spi_stpm3x_isr,                \
                     DEVICE_DT_INST_GET(id), 0)),   \
        (IRQ_CONNECT(DT_INST_IRQN(id),              \
                     DT_INST_IRQ(id, priority),     \
                     spi_stm32_isr,                 \
                     DEVICE_DT_INST_GET(id), 0))    \
    )

#define STM32_SDHC_IRQ_HANDLER_DECL(id)      \
	static void spi_stm32_irq_config_func_##id(const struct device *dev)

#define STM32_SDHC_IRQ_HANDLER_FUNC(id)      \
	.irq_config = spi_stm32_irq_config_func_##id,

#define STM32_SDHC_IRQ_HANDLER(id)           \
	static void spi_stm32_irq_config_func_##id(const struct device *dev) {  \
		IRQ_CONNECT(DT_INST_IRQN(id),              \
			    DT_INST_IRQ(id, priority),     \
			    spi_stpm3x_isr,                \
			    DEVICE_DT_INST_GET(id), 0));
		irq_enable(DT_INST_IRQN(id));       \
	}

#define STM32_SDHC_IRQ_NUM(id)   .irq = DT_INST_IRQN(id),

static void sdhc_stm32_irq_config_func_##n(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(n),
		DT_INST_IRQ(n, priority),
		sdhc_stm32_isr, DEVICE_DT_INST_GET(n),
		0);
	irq_enable(DT_INST_IRQN(n));
}

#if DT_INST_PROP(n, bus_width) == 1
#define SDMMC_BUS_WIDTH SDMMC_BUS_WIDE_1B
#elif DT_INST_PROP(n, bus_width) == 4
#define SDMMC_BUS_WIDTH SDMMC_BUS_WIDE_4B
#elif DT_INST_PROP(n, bus_width) == 8
#define SDMMC_BUS_WIDTH SDMMC_BUS_WIDE_8B
#endif /* DT_INST_PROP(n, bus_width) */

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
		.irq_config = sdhc_stm32_irq_config_func_##n,                                      \
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
		.hsdio = {                                                                         \
			.Instance = (SDIO_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                \
			.Init.BusWide = SDMMC_BUS_WIDTH,                                           \
#if DT_INST_NODE_HAS_PROP(n, clk_div)                                                              \
			.Init.ClockDiv = DT_INST_PROP(n, clk_div),                                 \
#endif                                                                                             \
		},                                                                                 \
#if DT_INST_NODE_HAS_PROP(n, cd_gpios)                                                             \
		.cd = GPIO_DT_SPEC_INST_GET(n, cd_gpios),                                          \
#endif                                                                                             \
#if DT_INST_NODE_HAS_PROP(n, pwr_gpios)                                                            \
		.pe = GPIO_DT_SPEC_INST_GET(n, pwr_gpios),                                         \
#endif                                                                                             \
		.pclken = pclken_sdmmc,                                                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.reset = RESET_DT_SPEC_INST_GET(n),                                                \
		SDMMC_DMA_CHANNEL(rx, RX)                                                          \
		SDMMC_DMA_CHANNEL(tx, TX)                                                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sdhc_stm32_init, NULL, &sdhc_stm32_##n##_data,                    \
			      &sdhc_stm32_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_STM32_INIT)

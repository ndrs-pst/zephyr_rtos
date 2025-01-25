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

#include <zephyr/drivers/gpio.h>
extern struct gpio_dt_spec const g_dbg_pin_gpio_dt[];

// #define STM32_SDHC_USE_DMA DT_NODE_HAS_PROP(DT_DRV_INST(0), dmas)
#define STM32_SDHC_USE_DMA      0

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
	struct gpio_dt_spec cd_gpio;
	struct gpio_dt_spec pwr_gpio;
	const struct stm32_pclken *pclken; /* clock subsystem driving this peripheral */
	size_t pclk_len;                   /* number of clock subsystems */
	const struct pinctrl_dev_config *pcfg;
	const struct reset_dt_spec reset;

	bool detect_dat3;
	bool detect_cd;

	struct sdhc_host_props props;
};

struct sdhc_stm32_data {
	SDMMC_TypeDef *sdmmc;
	struct sdhc_io host_io;
	struct k_sem sync;
	int status;
	struct k_work work;

	uint32_t ercd; /*!< SDIO Card Error codes */
	HAL_SDIO_StateTypeDef state;
	uint32_t xfer_ctx;

	bool use_dma;
	bool card_present;

	uint8_t *tx_buffer;
	size_t tx_xfer_sz;

	uint8_t *rx_buffer;
	size_t rx_xfer_sz;

	uint8_t io_int_num;
	uint8_t io_func_msk;
	void (*io_func_cb[SDIO_MAX_IO_NUMBER])(struct sdhc_stm32_data *data, uint32_t func);
};

#ifdef CONFIG_SDHC_STM32_HWFC
static void stm32_sdmmc_fc_enable(struct stm32_sdmmc_priv *priv)
{
	MMC_TypeDef *sdmmcx = priv->hsd.Instance;

	sdmmcx->CLKCR |= SDMMC_CLKCR_HWFC_EN;
}
#endif

static int sdhc_stm32_clock_enable(const struct sdhc_stm32_config *cfg)
{
	const struct device *clock;

	/* HSI48 Clock is enabled through using the device tree */
	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (cfg->pclk_len > 1) {
		if (clock_control_configure(clock,
					    (clock_control_subsys_t)&cfg->pclken[1],
					    NULL) != 0) {
			LOG_ERR("Failed to enable SDHC domain clock");
			return -EIO;
		}
	}

	if (IS_ENABLED(CONFIG_SDHC_STM32_CLOCK_CHECK)) {
		uint32_t sdhc_clock_rate;

		if (clock_control_get_rate(clock,
					   (clock_control_subsys_t)&cfg->pclken[1],
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
	return clock_control_on(clock, (clock_control_subsys_t)&cfg->pclken[0]);
}

static int sdhc_stm32_clock_disable(const struct sdhc_stm32_config *config)
{
	const struct device *clock;

	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	return clock_control_off(clock,
				 (clock_control_subsys_t)&config->pclken);
}

#if STM32_SDHC_USE_DMA

static void sdhc_stm32_dma_cb(const struct device *dev, void *arg, uint32_t channel, int status)
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

	handle->Instance = __LL_DMA_GET_STREAM_INSTANCE(dma->reg, dma->channel_nb);
	// FIXME handle->Init.Channel             = dma->cfg.dma_slot * DMA_CHANNEL_1;
	handle->Init.PeriphInc = DMA_PINC_DISABLE;
	handle->Init.MemInc = DMA_MINC_ENABLE;
	handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	handle->Init.Mode = DMA_PFCTRL;
	// FIXME handle->Init.Priority            = table_priority[dma->cfg.channel_priority],
	handle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	handle->Init.MemBurst = DMA_MBURST_INC4;
	handle->Init.PeriphBurst = DMA_PBURST_INC4;

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

static uint8_t sdhc_stm32_cnv_blk_sz(uint32_t blk_sz)
{
	uint8_t most_bit = (uint8_t)__CLZ(__RBIT(blk_sz));

	/* (1 << most_bit) - 1) is the mask used for blocksize */
	if (((uint8_t)blk_sz & ((1U << most_bit) - 1U)) != 0U) {
		return (uint8_t)SDMMC_DATABLOCK_SIZE_4B;
	}

	return most_bit << SDMMC_DCTRL_DBLOCKSIZE_Pos;
}

static int sdhc_stm32_get_resp_ll(SDMMC_TypeDef *sdmmc, int timeout_ms, uint32_t opcode,
				  enum sd_rsp_type rsp_type)
{
	while (true) {
		uint32_t sta_reg = sdmmc->STA;

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

	if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_CTIMEOUT)) {
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_FLAG_CTIMEOUT);
		return -ETIMEDOUT;
	} else if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_CCRCFAIL) &&
		   (rsp_type == SD_RSP_TYPE_R1)) {
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_FLAG_CCRCFAIL);
		return -EIO;
	} else {
		/* Nothing to do */
	}

	/* Check response received is of desired command (only when SD_RSP_TYPE_R1) */
	if ((rsp_type == SD_RSP_TYPE_R1) &&
	    (SDMMC_GetCommandResponse(sdmmc) != (uint8_t)opcode)) {
		return -EIO;
	}

	/* Clear all the static flags */
	__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_CMD_FLAGS);

	return 0;
}

static int sdhc_stm32_get_resp(SDMMC_TypeDef *sdmmc, struct sdhc_command *cmd)
{
	int ret;
	enum sd_rsp_type rsp_type;

	rsp_type = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
	ret = sdhc_stm32_get_resp_ll(sdmmc, cmd->timeout_ms, cmd->opcode, rsp_type);
	if (ret) {
		return ret;
	}

	/* We have received response, retrieve it for analysis  */
	cmd->response[0] = SDMMC_GetResponse(sdmmc, SDMMC_RESP1);
	if (rsp_type == SD_RSP_TYPE_R2) {
		cmd->response[1] = SDMMC_GetResponse(sdmmc, SDMMC_RESP2);
		cmd->response[2] = SDMMC_GetResponse(sdmmc, SDMMC_RESP3);
		cmd->response[3] = SDMMC_GetResponse(sdmmc, SDMMC_RESP4);
	}

	return 0;
}

static int sdhc_stm32_snd_cmd(SDMMC_TypeDef *sdmmc, struct sdhc_command *cmd, uint32_t response)
{
	SDMMC_CmdInitTypeDef cmd_init;
	int ret;

	cmd_init.Argument = cmd->arg;
	cmd_init.CmdIndex = cmd->opcode;
	cmd_init.Response = response;
	cmd_init.WaitForInterrupt = SDMMC_WAIT_NO;
	cmd_init.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(sdmmc, &cmd_init);

	ret = sdhc_stm32_get_resp(sdmmc, cmd);

	return ret;
}

static int sdhc_stm32_sdio_rd_direct(SDMMC_TypeDef *sdmmc, uint32_t func_num, uint32_t reg_addr,
				     uint8_t *data_out)
{
	SDMMC_CmdInitTypeDef cmd_init;
	int ret;
	uint32_t arg;

	arg = ((func_num << 28U) | ((reg_addr & 0x1FFFFU) << 9U));

	cmd_init.Argument = arg;
	cmd_init.CmdIndex = SDMMC_CMD_SDMMC_RW_DIRECT;
	cmd_init.Response = SDMMC_RESPONSE_SHORT;
	cmd_init.WaitForInterrupt = SDMMC_WAIT_NO;
	cmd_init.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(sdmmc, &cmd_init);

	ret = sdhc_stm32_get_resp_ll(sdmmc, 1, SDMMC_CMD_SDMMC_RW_DIRECT, SD_RSP_TYPE_R1);
	if (ret == 0) {
		__SDMMC_CMDTRANS_DISABLE(sdmmc);
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_DATA_FLAGS);

		*data_out = (uint8_t)SDMMC_GetResponse(sdmmc, SDMMC_RESP1);
	}

	return ret;
}

static int sdhc_stm32_sdio_wr_direct(SDMMC_TypeDef *sdmmc, uint32_t func_num, uint32_t raw,
				     uint32_t reg_addr, uint8_t data_in)
{
	SDMMC_CmdInitTypeDef cmd_init;
	int ret;
	uint32_t arg;

	arg = (BIT(31) | (func_num << 28U) | (raw << 27U) | ((reg_addr & 0x1FFFFU) << 9U) |
	       ((uint32_t)data_in & 0x000000FFU));

	cmd_init.Argument = arg;
	cmd_init.CmdIndex = SDMMC_CMD_SDMMC_RW_DIRECT;
	cmd_init.Response = SDMMC_RESPONSE_SHORT;
	cmd_init.WaitForInterrupt = SDMMC_WAIT_NO;
	cmd_init.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(sdmmc, &cmd_init);

	ret = sdhc_stm32_get_resp_ll(sdmmc, 1, SDMMC_CMD_SDMMC_RW_DIRECT, SD_RSP_TYPE_R1);
	if (ret == 0) {
		__SDMMC_CMDTRANS_DISABLE(sdmmc);
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_DATA_FLAGS);
	}

	return ret;
}

static int sdhc_stm32_sdio_rw_direct(SDMMC_TypeDef *sdmmc, struct sdhc_command *cmd)
{
	SDMMC_CmdInitTypeDef cmd_init;
	int ret;

	cmd_init.Argument = cmd->arg;
	cmd_init.CmdIndex = SDMMC_CMD_SDMMC_RW_DIRECT;
	cmd_init.Response = SDMMC_RESPONSE_SHORT;
	cmd_init.WaitForInterrupt = SDMMC_WAIT_NO;
	cmd_init.CPSM = SDMMC_CPSM_ENABLE;
	(void)SDMMC_SendCommand(sdmmc, &cmd_init);

	ret = sdhc_stm32_get_resp_ll(sdmmc, 1, SDMMC_CMD_SDMMC_RW_DIRECT, SD_RSP_TYPE_R1);
	if (ret == 0) {
		__SDMMC_CMDTRANS_DISABLE(sdmmc);
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_DATA_FLAGS);

		cmd->response[0] = SDMMC_GetResponse(sdmmc, SDMMC_RESP1);
	}

	return ret;
}

static int sdhc_stm32_sdio_rd_fifo(SDMMC_TypeDef *sdmmc, void *data, uint32_t dat_len,
				   int timeout_ms)
{
	uint32_t *rx_buf32 = (uint32_t *)data;
	uint8_t *rx_buf8 = data;
	uint32_t remain_len = dat_len;
	uint32_t chk_flg = (SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
			    SDMMC_FLAG_DATAEND);

	/* Fast path for 32-bit aligned data */
	if (((uintptr_t)data & 3) == 0) {
		while (!__SDMMC_GET_FLAG(sdmmc, chk_flg)) {
			if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_RXFIFOHF) && (remain_len >= 32U)) {
				/* Direct 32-bit reads for aligned data */
				for (int i = 0; i < 8; i++) {
					*rx_buf32++ = sdmmc->FIFO;
				}
				remain_len -= 32U;
			} else if ((remain_len < 32U) && (remain_len > 0U)) {
				while (!__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_RXFIFOE) &&
				       remain_len >= 4U) {
					*rx_buf32++ = sdmmc->FIFO;
					remain_len -= 4U;
				}

				/* Handle remaining bytes */
				if (remain_len > 0U) {
					uint32_t temp = sdmmc->FIFO;
					uint8_t *last_bytes = (uint8_t *)&temp;
					for (uint32_t i = 0; i < remain_len; i++) {
						*rx_buf8++ = last_bytes[i];
					}
					remain_len = 0U;
				}
			} else {
				/* Nothing to do */
			}

			k_msleep(1);
			if (timeout_ms-- == 0) {
				return -ETIMEDOUT;
			}
		}
	} else {
		while (!__SDMMC_GET_FLAG(sdmmc, chk_flg)) {
			if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_RXFIFOHF) && (remain_len >= 32U)) {
				for (size_t cnt = 0U; cnt < 8U; cnt++) {
					uint32_t rd_fifo = sdmmc->FIFO;
					*rx_buf8 = (uint8_t)(rd_fifo & 0xFFU);
					rx_buf8++;
					*rx_buf8 = (uint8_t)((rd_fifo >> 8U) & 0xFFU);
					rx_buf8++;
					*rx_buf8 = (uint8_t)((rd_fifo >> 16U) & 0xFFU);
					rx_buf8++;
					*rx_buf8 = (uint8_t)((rd_fifo >> 24U) & 0xFFU);
					rx_buf8++;
				}
				remain_len -= 32U;
			} else if (remain_len < 32U) {
				while ((remain_len > 0U) &&
				       !(__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_RXFIFOE))) {
					uint32_t rd_fifo = sdmmc->FIFO;
					for (size_t cnt = 0U; cnt < 4U; cnt++) {
						if (remain_len > 0U) {
							*rx_buf8 =
								(uint8_t)((rd_fifo >> (cnt * 8U)) &
									  0xFFU);
							rx_buf8++;
							remain_len--;
						}
					}
				}
			} else {
				/* Nothing to do */
			}

			k_msleep(1);
			if (timeout_ms-- == 0) {
				return -ETIMEDOUT;
			}
		}
	}

	return 0;
}

static int sdhc_stm32_sdio_wr_fifo(SDMMC_TypeDef *sdmmc, void *data, uint32_t dat_len,
				   int timeout_ms)
{
	uint32_t *tx_buf32 = (uint32_t *)data;
	uint32_t remain_len = dat_len;
	uint32_t chk_flg = (SDMMC_FLAG_TXUNDERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
			    SDMMC_FLAG_DATAEND);

	/* Fast path for 32-bit aligned data */
	if (((uintptr_t)data & 3) == 0) {
		while (!__SDMMC_GET_FLAG(sdmmc, chk_flg)) {
			if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_TXFIFOHE) && (remain_len >= 32U)) {
				for (size_t cnt = 0; cnt < 8; cnt++) {
					sdmmc->FIFO = *tx_buf32++;
				}
				remain_len -= 32U;
			} else if ((remain_len < 32U) && (remain_len > 0U)) {
				while (!__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_TXFIFOHE |
							 SDMMC_FLAG_TXFIFOE) &&
				       remain_len >= 4U) {
					sdmmc->FIFO = *tx_buf32++;
					remain_len -= 4U;
				}

				/* Handle remaining bytes */
				if (remain_len > 0U) {
					uint8_t *tx_buf8 = (uint8_t *)tx_buf32;
					uint32_t dat32 = 0U;
					for (size_t cnt = 0U; cnt < remain_len; cnt++) {
						dat32 |= ((uint32_t)(*tx_buf8) << (cnt << 3U));
						tx_buf8++;
					}
					sdmmc->FIFO = dat32;
				}
			}

			k_msleep(1);
			if (timeout_ms-- == 0) {
				return -ETIMEDOUT;
			}
		}
	} else {
		/* FIXME not yet support */
	}

	return 0;
}

static int sdhc_stm32_sdio_rw_extend(struct sdhc_stm32_data *ctx, struct sdhc_command *cmd,
				     const struct sdhc_data *data)
{
	SDMMC_TypeDef *sdmmc = ctx->sdmmc;
	SDMMC_DataInitTypeDef config;
	uint32_t dir;
	uint32_t arg;
	uint32_t mode;
	uint32_t dat_len;
	uint32_t blk_sz;
	int ret;

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

	if ((sdmmc->DCTRL & SDMMC_DCTRL_SDIOEN) != 0U) {
		sdmmc->DCTRL = SDMMC_DCTRL_SDIOEN;
	} else {
		sdmmc->DCTRL = 0U;
	}

	/* Configure the SD DPSM (Data Path State Machine) */
	config.DataTimeOut   = SDMMC_DATATIMEOUT;
	config.DataLength    = dat_len;
	config.DataBlockSize = blk_sz;
	config.TransferDir   = dir;
	config.TransferMode  = mode;
	config.DPSM          = SDMMC_DPSM_DISABLE;
	(void)SDMMC_ConfigData(ctx->sdmmc, &config);

	__SDMMC_CMDTRANS_ENABLE(sdmmc);

	ret = sdhc_stm32_snd_cmd(sdmmc, cmd, SDMMC_RESPONSE_SHORT);
	if (ret) {
		return -EIO;
	}

	if (ctx->use_dma) {
		/* @todo */
	} else {
		if (arg & BIT(SDIO_CMD_ARG_RW_SHIFT)) {
			ret = sdhc_stm32_sdio_wr_fifo(sdmmc, data->data, dat_len, data->timeout_ms);
		} else {
			ret = sdhc_stm32_sdio_rd_fifo(sdmmc, data->data, dat_len, data->timeout_ms);
		}

		__SDMMC_CMDTRANS_DISABLE(sdmmc);

		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_DATA_FLAGS);
	}

	return ret;
}

static void sdhc_stm32_cfg_data(SDMMC_TypeDef *sdmmc, uint32_t dat_len, uint32_t blk_sz,
				uint32_t dir)
{
	SDMMC_DataInitTypeDef config;

	/* Initialize data control register */
	sdmmc->DCTRL = 0U;

	/* Configure the SD DPSM (Data Path State Machine) */
	config.DataTimeOut = SDMMC_DATATIMEOUT;
	config.DataLength = dat_len;
	config.DataBlockSize = blk_sz;
	config.TransferDir = dir;
	config.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
	config.DPSM = SDMMC_DPSM_DISABLE;
	(void)SDMMC_ConfigData(sdmmc, &config);

	__SDMMC_CMDTRANS_ENABLE(sdmmc);
}

static int sdhc_stm32_reset(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;

	/* Switch to default I/O voltage of 3.3V */

	return 0;
}

#if CONFIG_SDHC_STM32_HWFC
static sdhc_stm32_fc_enable(SDMMC_TypeDef *sdmmc)
{
	sdmmc->CLKCR |= SDMMC_CLKCR_HWFC_EN;
}
#endif

static void SD_Read_IT(struct sdhc_stm32_data *ctx, const SDMMC_TypeDef *sdmmc)
{
	uint32_t count;
	uint32_t data;
	uint8_t *tmp;

	tmp = ctx->rx_buffer;

	if (ctx->rx_xfer_sz >= SDMMC_FIFO_SIZE) {
		/* Read data from SDMMC Rx FIFO */
		for (count = 0U; count < (SDMMC_FIFO_SIZE / 4U); count++) {
			data = SDMMC_ReadFIFO(sdmmc);
			*tmp = (uint8_t)(data & 0xFFU);
			tmp++;
			*tmp = (uint8_t)((data >> 8U) & 0xFFU);
			tmp++;
			*tmp = (uint8_t)((data >> 16U) & 0xFFU);
			tmp++;
			*tmp = (uint8_t)((data >> 24U) & 0xFFU);
			tmp++;
		}

    		ctx->rx_buffer = tmp;
    		ctx->rx_xfer_sz -= SDMMC_FIFO_SIZE;
  	}
}

static void SD_Write_IT(struct sdhc_stm32_data *ctx, SDMMC_TypeDef *sdmmc) {
	uint32_t count;
	uint32_t data;
	uint8_t *tmp;

	tmp = ctx->tx_buffer;

	if (ctx->tx_xfer_sz >= SDMMC_FIFO_SIZE) {
		/* Write data to SDMMC Tx FIFO */
		for (count = 0U; count < (SDMMC_FIFO_SIZE / 4U); count++) {
			data = (uint32_t)(*tmp);
			tmp++;
			data |= ((uint32_t)(*tmp) << 8U);
			tmp++;
			data |= ((uint32_t)(*tmp) << 16U);
			tmp++;
			data |= ((uint32_t)(*tmp) << 24U);
			tmp++;
			(void)SDMMC_WriteFIFO(sdmmc, &data);
		}

		ctx->tx_buffer = tmp;
		ctx->tx_xfer_sz -= SDMMC_FIFO_SIZE;
	}
}

static void sdhc_stm32_isr(const struct device *dev)
{
	struct sdhc_stm32_data *ctx = dev->data;
	SDMMC_TypeDef *sdmmc = ctx->sdmmc;
	uint32_t flags;

	gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[0], 1);          /* DBG_PIN0_HIGH */

	/* @see HAL_SDIO_IRQHandler */
	flags = sdmmc->STA;
	if ((flags & SDMMC_FLAG_SDIOIT) != 0U) {
		(void)k_work_submit(&ctx->work);
	}

	if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_RXFIFOHF)) {
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_FLAG_RXFIFOHF);

		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[1], 1);  /* DBG_PIN1_HIGH */
		SD_Read_IT(ctx, sdmmc);
		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[1], 0);  /* DBG_PIN1_LOW */
	} else if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_DATAEND)) {
		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[2], 1);  /* DBG_PIN2_HIGH */
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_FLAG_DATAEND);

		__SDMMC_DISABLE_IT(sdmmc, SDMMC_IT_DATAEND  | SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | \
				   SDMMC_IT_TXUNDERR | SDMMC_IT_RXOVERR  | SDMMC_IT_TXFIFOHE | \
				   SDMMC_IT_RXFIFOHF);

		__SDMMC_DISABLE_IT(sdmmc, SDMMC_IT_IDMABTC);
		__SDMMC_CMDTRANS_DISABLE(sdmmc);

		/* Clear all the static flags */
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_STATIC_DATA_FLAGS);
		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[2], 0);  /* DBG_PIN2_LOW */
	} else if (__SDMMC_GET_FLAG(sdmmc, SDMMC_FLAG_TXFIFOHE)) {
		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[3], 1);  /* DBG_PIN3_HIGH */
		__SDMMC_CLEAR_FLAG(sdmmc, SDMMC_FLAG_TXFIFOHE);
		SD_Write_IT(ctx, sdmmc);
		gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[3], 0);  /* DBG_PIN3_LOW */
	}

	gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[0], 0);          /* DBG_PIN0_LOW */
}

/*
 * Set SDHC io properties @see sdhc_esp32_set_io, imx_usdhc_set_io
 */
static int sdhc_stm32_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	SDMMC_TypeDef *sdmmc = ctx->sdmmc;
	struct sdhc_io *host_io = &ctx->host_io;

	LOG_INF("SDHC I/O: bus width %d, clock %dHz, card power %s, voltage %s",
		ios->bus_width, ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (host_io->clock != ios->clock) {
		uint32_t sdmmc_ker_ck;
		uint32_t clk_div;
		const struct device *clock;

		clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
		if (clock_control_get_rate(clock, (clock_control_subsys_t)&cfg->pclken[1],
					   &sdmmc_ker_ck) != 0) {
			LOG_ERR("Failed to get SDMMC domain clock rate");
			return -EIO;
		}

		if (((unsigned int)ios->clock > cfg->props.f_max) ||
		    ((unsigned int)ios->clock < cfg->props.f_min)) {
			return -EINVAL;
		}

		/* clk_div is equal to [sdmmc_ker_ck / (2 * SDMMC_CK)] */
		clk_div = sdmmc_ker_ck / (2U * (uint32_t)ios->clock);
		if (clk_div > 0) {
			MODIFY_REG(sdmmc->CLKCR, SDMMC_CLKCR_CLKDIV, clk_div);
		} else {
			return -EINVAL;
		}

		host_io->clock = ios->clock;
	}

	if (host_io->bus_width != ios->bus_width) {
		uint32_t bus_wide;

		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			bus_wide = SDMMC_BUS_WIDE_1B;
			break;

		case SDHC_BUS_WIDTH4BIT:
			bus_wide = SDMMC_BUS_WIDE_4B;
			break;

		default: /* SDHC_BUS_WIDTH8BIT */
			bus_wide = SDMMC_BUS_WIDE_8B;
			break;
		}

		MODIFY_REG(sdmmc->CLKCR, SDMMC_CLKCR_WIDBUS, bus_wide);
		host_io->bus_width = ios->bus_width;
	}

	if (ios->signal_voltage != host_io->signal_voltage) {
		/* Switch to 1.8V */
		if (ios->signal_voltage == SD_VOL_1_8_V) {
			/* Enable 1.8V signal voltage */
			/* @todo */
		} else {
			/* Switch to 3.3V */
			/* Disable 1.8V signal voltage */
			/* @todo */
		}

		host_io->signal_voltage = ios->signal_voltage;
	}

	/* Toggle card power supply */
	if ((host_io->power_mode != ios->power_mode) && cfg->pwr_gpio.port) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			gpio_pin_set_dt(&cfg->pwr_gpio, 0);
		} else { /* SDHC_POWER_ON */
			gpio_pin_set_dt(&cfg->pwr_gpio, 1);
		}

		host_io->power_mode = ios->power_mode;
	}

	/* Set I/O timing */
	if (host_io->timing != ios->timing) {
		switch (ios->timing) { /* HAL_SDIO_SetSpeedMode */
		case SDHC_TIMING_LEGACY:
		case SDHC_TIMING_HS:
		case SDHC_TIMING_SDR12:
		case SDHC_TIMING_SDR25:
			MODIFY_REG(sdmmc->CLKCR, (SDMMC_CLKCR_DDR | SDMMC_CLKCR_BUSSPEED), 0U);
			break;

		case SDHC_TIMING_SDR50:
		case SDHC_TIMING_SDR104:
		case SDHC_TIMING_HS200:
			MODIFY_REG(sdmmc->CLKCR, (SDMMC_CLKCR_DDR | SDMMC_CLKCR_BUSSPEED),
				   SDMMC_CLKCR_BUSSPEED);
			break;

		case SDHC_TIMING_DDR50:
			MODIFY_REG(sdmmc->CLKCR, (SDMMC_CLKCR_DDR | SDMMC_CLKCR_BUSSPEED),
				   (SDMMC_CLKCR_DDR | SDMMC_CLKCR_BUSSPEED));
			/* Enable DDR mode */
			LOG_INF("DDR mode enabled");
			break;

		default:
			LOG_ERR("Timing mode not supported for this device");
			return -ENOTSUP;
		}

		host_io->timing = ios->timing;
	}

	return 0;
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
	uint32_t err_state;
	int ret = 0;

	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
		err_state = SDMMC_CmdGoIdleState(ctx->sdmmc);
		if (err_state != HAL_SD_ERROR_NONE) {
			return -EIO;
		}
		break;

	case SDIO_SEND_OP_COND:
		err_state = SDMMC_CmdSendOperationcondition(ctx->sdmmc, cmd->arg, cmd->response);
		if (err_state != HAL_SD_ERROR_NONE) {
			return -EIO;
		}
		break;

	case SD_ALL_SEND_CID: /* SDMMC_CmdSendCID */
	case SD_SEND_CSD:     /* SDMMC_CmdSendCSD */
		ret = sdhc_stm32_snd_cmd(ctx->sdmmc, cmd, SDMMC_RESPONSE_LONG);
		break;

	case MMC_SEND_OP_COND:         /* SDMMC_CmdOpCondition */
	case SDMMC_SEND_RELATIVE_ADDR: /* SDMMC_CmdSetRelAdd */
	case SD_SWITCH:                /* SDMMC_CmdSwitch, share with SD_APP_SET_BUS_WIDTH */
	case SD_SELECT_CARD:           /* SDMMC_CmdSelDesel */
	case SD_SEND_IF_COND:          /* SDMMC_CmdSendEXTCSD */
	case SD_VOL_SWITCH:            /* SDMMC_CmdVoltageSwitch */
	case SD_SEND_STATUS:           /* SDMMC_CmdSendStatus */
	case SD_SET_BLOCK_SIZE:        /* SDMMC_CmdBlockLength */
	case SD_APP_SEND_OP_COND:      /* SDMMC_CmdAppOperCommand */
	case SD_APP_CMD:               /* SDMMC_CmdAppCommand */
		ret = sdhc_stm32_snd_cmd(ctx->sdmmc, cmd, SDMMC_RESPONSE_SHORT);
		break;

	case SDIO_RW_DIRECT: { /* HAL_SDIO_ReadDirect */
		ret = sdhc_stm32_sdio_rw_direct(ctx->sdmmc, cmd);
		break;
	}

	case SDIO_RW_EXTENDED: {
		ret = sdhc_stm32_sdio_rw_extend(ctx, cmd, data);
		break;
	}

	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK: {
		if (cmd->opcode == SD_READ_SINGLE_BLOCK) {
			ctx->xfer_ctx = SD_CONTEXT_READ_SINGLE_BLOCK;
		} else {
			ctx->xfer_ctx = SD_CONTEXT_READ_MULTIPLE_BLOCK;
		}

		ctx->rx_buffer  = data->data;
		ctx->rx_xfer_sz = data->blocks * BLOCKSIZE;
		sdhc_stm32_cfg_data(ctx->sdmmc, ctx->rx_xfer_sz, SDMMC_DATABLOCK_SIZE_512B,
				    SDMMC_TRANSFER_DIR_TO_SDMMC);
		ret = sdhc_stm32_snd_cmd(ctx->sdmmc, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			__SDMMC_ENABLE_IT(ctx->sdmmc, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						       SDMMC_IT_RXOVERR  | SDMMC_IT_DATAEND  |
						       SDMMC_FLAG_RXFIFOHF));
		}
		break;
	}

	case SD_APP_SEND_SCR: /* sdmmc_read_scr */
		break;

	case SD_APP_SEND_NUM_WRITTEN_BLK: {
		sdhc_stm32_cfg_data(ctx->sdmmc, 4, SDMMC_DATABLOCK_SIZE_4B,
				    SDMMC_TRANSFER_DIR_TO_SDMMC);
		ret = sdhc_stm32_snd_cmd(ctx->sdmmc, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			__SDMMC_ENABLE_IT(ctx->sdmmc, (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
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

		ctx->tx_buffer  = data->data;
		ctx->tx_xfer_sz = data->blocks * BLOCKSIZE;
		sdhc_stm32_cfg_data(ctx->sdmmc, ctx->tx_xfer_sz, SDMMC_DATABLOCK_SIZE_512B,
				    SDMMC_TRANSFER_DIR_TO_CARD);
		ret = sdhc_stm32_snd_cmd(ctx->sdmmc, cmd, SDMMC_RESPONSE_SHORT);
		if (ret == 0) {
			if (ctx->use_dma) {
				ctx->xfer_ctx |= SD_CONTEXT_DMA;
				__SDMMC_ENABLE_IT(ctx->sdmmc,
						  (SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT |
						   SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND));
			} else {
				ctx->xfer_ctx |= SD_CONTEXT_IT;
				__SDMMC_ENABLE_IT(ctx->sdmmc,
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

	return (ret);
}

/*
 * Send CMD or CMD/DATA via SDHC
 */
static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *data)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	unsigned int retries = cmd->retries + 1;
	int busy_timeout = 5000;
	int ret;

	gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[4], 1);  /* DBG_PIN4_HIGH */

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

		retries--;
	} while ((ret != 0) && (retries > 0));

	gpio_pin_set_raw_dt(&g_dbg_pin_gpio_dt[4], 0);  /* DBG_PIN4_LOW */

	return ret;
}

/*
 * Get card presence @see imx_usdhc_get_card_present
 */
static int sdhc_stm32_get_card_present(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;

	if (cfg->detect_dat3) {
		if (!ctx->card_present) {
			/* Detect card presence with DAT3 line pull */

			/* Delay to ensure host has time to detect card */
			k_busy_wait(1000);

			/* Clear card detection and pull */
		}
	} else if (cfg->detect_cd) {
		/* TBA */
	} else if (cfg->cd_gpio.port) {
		ctx->card_present = gpio_pin_get_dt(&cfg->cd_gpio) > 0;
	} else {
		LOG_WRN("No card detection method configured, assuming card "
			"is present");
		ctx->card_present = true;
	}

	return ((int)ctx->card_present);
}

/*
 * Get host properties
 */
static int sdhc_stm32_get_host_props(const struct device *dev,
				     struct sdhc_host_props *props) /* SD_INIT_SEQ02 */
{
	const struct sdhc_stm32_config *cfg = dev->config;

	memcpy(props, &cfg->props, sizeof(struct sdhc_host_props));

	return 0;
}

static void sdhc_stm32_work_handler(struct k_work *work)
{
	struct sdhc_stm32_data *ctx = CONTAINER_OF(work, struct sdhc_stm32_data, work);
	SDMMC_TypeDef *sdmmc = ctx->sdmmc;
	uint8_t int_pending;

	if (ctx->io_int_num == 1U) { /* HAL_SDIO_RegisterIOFunctionCallback */
		if ((ctx->io_func_cb[ctx->io_func_msk - 1U]) != NULL) {
			(ctx->io_func_cb[ctx->io_func_msk - 1U])(ctx, ctx->io_func_msk - 1U);
		}
	} else if ((ctx->io_int_num > 1U) && (ctx->io_func_msk != 0U)) {
		/* Get pending int firstly */
		int ret;

		ret = sdhc_stm32_sdio_rd_direct(sdmmc, SDIO_FUNC_NUM_0, SDIO_CCCR_INT_P,
						&int_pending);
		if (ret) {
			return;
		}

		if ((int_pending != 0U) && (ctx->io_func_msk != 0U)) {
			for (size_t count = 1; count <= SDIO_MAX_IO_NUMBER; count++) {
				if (((int_pending & (1U << count)) != 0U) &&
				    (((1U << count) & ctx->io_func_msk) != 0U)) {
					if ((ctx->io_func_cb[count - 1U]) != NULL) {
						(ctx->io_func_cb[count - 1U])(ctx, count);
					}
				}
			}
		}
	} else {
		/* Nothing to do */
	}
}

static int sdhc_stm32_registers_configure(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data const *ctx = dev->data;
	SDMMC_TypeDef *sdmmc = ctx->sdmmc;
	const struct device *clock;
	SDMMC_InitTypeDef init;
	uint32_t sdmmc_ker_ck;
	int ret;

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("reset controller not ready");
		return -ENODEV;
	}

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("failed to reset peripheral");
		return ret;
	}

	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	if (clock_control_get_rate(clock, (clock_control_subsys_t)&cfg->pclken[1],
	    &sdmmc_ker_ck) != 0) {
		LOG_ERR("Failed to get SDMMC domain clock rate");
		return -EIO;
	}

	init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
	init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	init.BusWide             = SDMMC_BUS_WIDE_1B;
	init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	init.ClockDiv            = sdmmc_ker_ck / (2U * SDMMC_CLOCK_400KHZ);
	(void)SDMMC_Init(sdmmc, init);

	(void)SDMMC_PowerState_ON(sdmmc);

	return ret;
}

/*
 * Perform early system init for SDHC
 */
static int sdhc_stm32_init(const struct device *dev)
{
	const struct sdhc_stm32_config *cfg = dev->config;
	struct sdhc_stm32_data *ctx = dev->data;
	int ret;

	ret = sdhc_stm32_clock_enable(cfg);
	if (ret) {
		LOG_ERR("failed to init clocks");
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure SDHC pins");
		return -EINVAL;
	}

	ret = sdhc_stm32_registers_configure(dev);
	if (ret < 0) {
		return ret;
	}

	/* init carrier detect (if set) */
	if (cfg->cd_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->cd_gpio)) {
			LOG_ERR("GPIO port for cd pin is not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
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

#ifdef CONFIG_SDHC_STM32_HWFC
	sdhc_stm32_fc_enable(ctx->sdmmc);
#endif

	/* Set all host IO values to zeroes */
	memset(&ctx->host_io, 0, sizeof(struct sdhc_io));

	k_work_init(&ctx->work, sdhc_stm32_work_handler);

	cfg->irq_config(dev);

	return 0;
}

#define STM32_SDHC_IRQ_HANDLER(n)                                                                  \
	static void sdhc_stm32_irq_config_func_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), sdhc_stm32_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
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
	STM32_SDHC_IRQ_HANDLER(n);                                                                 \
	static const struct stm32_pclken pclken_##n[] = STM32_DT_INST_CLOCKS(n);                   \
	PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                         \
                                                                                                   \
	static const struct sdhc_stm32_config sdhc_stm32_##n##_config = {                          \
		.irq_config = sdhc_stm32_irq_config_func_##n,                                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.pclken = pclken_##n,                                                              \
		.pclk_len = DT_INST_NUM_CLOCKS(n),                                                 \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cd_gpios, {0}),                             \
		.pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_gpios, {0}),                           \
		.reset = RESET_DT_SPEC_INST_GET(n),                                                \
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
					.ddr50_support = true,                                     \
					.sdr104_support = true,                                    \
					.sdr50_support = true,                                     \
					.bus_8_bit_support = false,                                \
					.bus_4_bit_support =                                       \
						(DT_INST_PROP(n, bus_width) == 4) ? true : false,  \
					.hs200_support = true,                                     \
					.hs400_support = false}}};                                 \
                                                                                                   \
	static struct sdhc_stm32_data sdhc_stm32_##n##_data = {                                    \
		.sdmmc = (SDMMC_TypeDef *)DT_REG_ADDR(DT_DRV_INST(n)),                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sdhc_stm32_init, NULL, &sdhc_stm32_##n##_data,                    \
			      &sdhc_stm32_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &sdhc_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_STM32_INIT)

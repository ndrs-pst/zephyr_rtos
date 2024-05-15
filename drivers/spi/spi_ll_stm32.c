/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ll_stm32);

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32_ll_spi.h>
#include <errno.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/toolchain.h>
#ifdef CONFIG_SPI_STM32_DMA
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/mem_mgmt/mem_attr.h>

#ifdef CONFIG_SOC_SERIES_STM32H7X
#include <zephyr/dt-bindings/memory-attr/memory-attr-arm.h>
#endif

#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#elif defined(CONFIG_CACHE_MANAGEMENT)
#include <zephyr/arch/cache.h>
#endif /* CONFIG_NOCACHE_MEMORY */

#include "spi_ll_stm32.h"

/*
 * Check defined(CONFIG_DCACHE) because some platforms disable it in the tests
 * e.g. nucleo_f746zg
 */
#if (defined(CONFIG_CPU_HAS_DCACHE) &&      \
     defined(CONFIG_DCACHE) &&              \
     !defined(CONFIG_NOCACHE_MEMORY))
#define SPI_STM32_MANUAL_CACHE_COHERENCY_REQUIRED   1
#else
#define SPI_STM32_MANUAL_CACHE_COHERENCY_REQUIRED   0
#endif /* defined(CONFIG_CPU_HAS_DCACHE) && !defined(CONFIG_NOCACHE_MEMORY) */

#define WAIT_1US        1U

/*
 * Check for SPI_SR_FRE to determine support for TI mode frame format
 * error flag, because STM32F1 SoCs do not support it and  STM32CUBE
 * for F1 family defines an unused LL_SPI_SR_FRE.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCE | LL_SPI_SR_MODF | \
                           LL_SPI_SR_OVR | LL_SPI_SR_TIFRE)
#else
#if defined(LL_SPI_SR_UDR)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
                           LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#elif defined(SPI_SR_FRE)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
                           LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#else
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | LL_SPI_SR_OVR)
#endif
#endif /* CONFIG_SOC_SERIES_STM32MP1X */

#if (__GTEST == 1U)
extern void bsp_hal_spi_tx_rx_cplt_callback(SPI_TypeDef* spi);
extern void bsp_hal_spi_err_callback(SPI_TypeDef* spi, uint32_t ercd);
#else
__weak void bsp_hal_spi_tx_rx_cplt_callback(SPI_TypeDef* spi) {
    /* pass */
}

__weak void bsp_hal_spi_err_callback(SPI_TypeDef* spi, uint32_t ercd) {
    /* pass */
}
#endif

#ifdef CONFIG_SPI_STM32_DMA
static uint32_t bits2bytes(uint32_t bits) {
    return (bits / 8);
}

/* dummy value used for transferring NOP when tx buf is null
 * and use as dummy sink for when rx buf is null.
 */
#ifdef CONFIG_NOCACHE_MEMORY
/*
 * If a nocache area is available, place it there to avoid potential DMA
 * cache-coherency problems.
 */
static __aligned(32) uint32_t dummy_rx_tx_buffer
       __attribute__((__section__(".nocache")));

#else /* CONFIG_NOCACHE_MEMORY */

/*
 * If nocache areas are not available, cache coherency might need to be kept
 * manually. See SPI_STM32_MANUAL_CACHE_COHERENCY_REQUIRED.
 */
static __aligned(32) uint32_t dummy_rx_tx_buffer;
#endif /* CONFIG_NOCACHE_MEMORY */

/* #CUSTOM@NDRS */
#define DEVICE_STM32_GET_SPI(dev)         (((const struct spi_stm32_config*)(dev)->config)->spi)

/* This function is executed in the interrupt context */
__maybe_unused static void spi_stm32_dma_callback(const struct device* dev, void* arg,
                                                  uint32_t channel, int status) {
    /* arg directly holds the spi device */
    struct spi_stm32_data* data = arg;

    if (status < 0) {
        LOG_ERR("DMA callback error with channel %d.", channel);
        data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
    }
    else {
        /* identify the origin of this callback */
        if (channel == data->dma_tx.channel) {
            /* this part of the transfer ends */
            data->status_flags |= SPI_STM32_DMA_TX_DONE_FLAG;
        }
        else if (channel == data->dma_rx.channel) {
            /* this part of the transfer ends */
            data->status_flags |= SPI_STM32_DMA_RX_DONE_FLAG;
        }
        else {
            LOG_ERR("DMA callback channel %d is not valid.",
                    channel);
            data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
        }
    }

    k_sem_give(&data->status_sem);
}

static int spi_stm32_dma_tx_load(const struct device* dev, uint8_t const* buf,
                                 size_t len) {
    SPI_TypeDef* spi = DEVICE_STM32_GET_SPI(dev);
    struct spi_stm32_data* data = dev->data;
    struct dma_block_config* blk_cfg;
    int ret;

    /* remember active TX DMA channel (used in callback) */
    struct stream* stream = &data->dma_tx;

    blk_cfg = &stream->dma_blk_cfg;

    /* prepare the block for this TX DMA channel */
    memset(blk_cfg, 0, sizeof(struct dma_block_config));
    blk_cfg->block_size = len;

    /* tx direction has memory as source and periph as dest. */
    if (buf == NULL) {
        /* if tx buff is null, then sends NOP on the line. */
        dummy_rx_tx_buffer = 0;
        #if SPI_STM32_MANUAL_CACHE_COHERENCY_REQUIRED
        arch_dcache_flush_range((void*)&dummy_rx_tx_buffer, sizeof(uint32_t));
        #endif /* CONFIG_CPU_HAS_DCACHE && !defined(CONFIG_NOCACHE_MEMORY) */
        blk_cfg->source_address  = (uint32_t)&dummy_rx_tx_buffer;
        blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }
    else {
        blk_cfg->source_address = (uint32_t)buf;
        if (data->dma_tx.src_addr_increment) {
            blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        }
        else {
            blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        }
    }

    blk_cfg->dest_address = ll_func_dma_get_reg_addr(spi, SPI_STM32_DMA_TX);
    /* fifo mode NOT USED there */
    if (data->dma_tx.dst_addr_increment) {
        blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    }
    else {
        blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }

    /* give the fifo mode from the DT */
    blk_cfg->fifo_mode_control = (uint16_t)data->dma_tx.fifo_threshold;

    /* direction is given by the DT */
    stream->dma_cfg.head_block = blk_cfg;
    /* give the client dev as arg, as the callback comes from the dma */
    stream->dma_cfg.user_data = data;
    /* pass our client origin to the dma: data->dma_tx.dma_channel */
    ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.channel,
                     &stream->dma_cfg);
    /* the channel is the actual stream from 0 */
    if (ret != 0) {
        return (ret);
    }

    /* gives the request ID to the dma mux */
    ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);

    return (ret);
}

static int spi_stm32_dma_rx_load(const struct device* dev, uint8_t* buf,
                                 size_t len) {
    SPI_TypeDef* spi = DEVICE_STM32_GET_SPI(dev);
    struct spi_stm32_data* data = dev->data;
    struct dma_block_config* blk_cfg;
    int ret;

    /* retrieve active RX DMA channel (used in callback) */
    struct stream* stream = &data->dma_rx;

    blk_cfg = &stream->dma_blk_cfg;

    /* prepare the block for this RX DMA channel */
    (void) memset(blk_cfg, 0, sizeof(struct dma_block_config));
    blk_cfg->block_size = len;

    /* rx direction has periph as source and mem as dest. */
    if (buf == NULL) {
        /* if rx buff is null, then write data to dummy address. */
        blk_cfg->dest_address  = (uint32_t)&dummy_rx_tx_buffer;
        blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }
    else {
        blk_cfg->dest_address = (uint32_t)buf;
        if (data->dma_rx.dst_addr_increment) {
            blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        }
        else {
            blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        }
    }

    blk_cfg->source_address = ll_func_dma_get_reg_addr(spi, SPI_STM32_DMA_RX);
    if (data->dma_rx.src_addr_increment) {
        blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    }
    else {
        blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }

    /* give the fifo mode from the DT */
    blk_cfg->fifo_mode_control = (uint16_t)data->dma_rx.fifo_threshold;

    /* direction is given by the DT */
    stream->dma_cfg.head_block = blk_cfg;
    stream->dma_cfg.user_data  = data;

    /* pass our client origin to the dma: data->dma_rx.channel */
    ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.channel,
                     &stream->dma_cfg);
    /* the channel is the actual stream from 0 */
    if (ret != 0) {
        return (ret);
    }

    /* gives the request ID to the dma mux */
    ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);

    return (ret);
}

static int spi_dma_move_buffers(const struct device* dev, size_t len) {
    struct spi_stm32_data* data = dev->data;
    int ret;
    size_t dma_segment_len;

    dma_segment_len = len * data->dma_rx.dma_cfg.dest_data_size;
    ret = spi_stm32_dma_rx_load(dev, data->ctx.rx_buf, dma_segment_len);
    if (ret != 0) {
        return (ret);
    }

    dma_segment_len = len * data->dma_tx.dma_cfg.source_data_size;
    ret = spi_stm32_dma_tx_load(dev, data->ctx.tx_buf, dma_segment_len);

    return (ret);
}

#endif /* CONFIG_SPI_STM32_DMA */

/* Value to shift out when no application data needs transmitting. */
#define SPI_STM32_TX_NOP 0x00

static void spi_stm32_send_next_frame(SPI_TypeDef* spi,
                                      struct spi_stm32_data *data) {
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    uint32_t tx_frame = SPI_STM32_TX_NOP;

    if (frame_size == 8) {
        if (spi_context_tx_buf_on(&data->ctx)) {
            tx_frame = UNALIGNED_GET((uint8_t*)(data->ctx.tx_buf));
        }
        LL_SPI_TransmitData8(spi, (uint8_t)tx_frame);
        spi_context_update_tx(&data->ctx, 1, 1);
    }
    else {
        if (spi_context_tx_buf_on(&data->ctx)) {
            tx_frame = UNALIGNED_GET((uint16_t*)(data->ctx.tx_buf));
        }
        LL_SPI_TransmitData16(spi, (uint16_t)tx_frame);
        spi_context_update_tx(&data->ctx, 2, 1);
    }
}

static void spi_stm32_read_next_frame(SPI_TypeDef* spi,
                                      struct spi_stm32_data* data) {
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    uint32_t rx_frame = 0;

    if (frame_size == 8) {
        rx_frame = LL_SPI_ReceiveData8(spi);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT(rx_frame, (uint8_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 1, 1);
    }
    else {
        rx_frame = LL_SPI_ReceiveData16(spi);
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT(rx_frame, (uint16_t*)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 2, 1);
    }
}

static bool spi_stm32_transfer_ongoing(struct spi_stm32_data* data) {
    return (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx));
}

static int spi_stm32_get_err(SPI_TypeDef* spi) {
    uint32_t sr = LL_SPI_ReadReg(spi, SR);

    if (sr & SPI_STM32_ERR_MSK) {
        LOG_ERR("%s: err=%d", __func__,
                sr & (uint32_t)SPI_STM32_ERR_MSK);

        /* OVR error must be explicitly cleared */
        if (LL_SPI_IsActiveFlag_OVR(spi)) {
            LL_SPI_ClearFlag_OVR(spi);
        }

        return (-EIO);
    }

    return (0);
}

static void spi_stm32_shift_fifo(SPI_TypeDef* spi, struct spi_stm32_data* data) {
    if (ll_func_rx_is_not_empty(spi)) {
        spi_stm32_read_next_frame(spi, data);
    }

    if (ll_func_tx_is_not_full(spi)) {
        spi_stm32_send_next_frame(spi, data);
    }
}

/* Shift a SPI frame as master. */
static void spi_stm32_shift_m(const struct spi_stm32_config* cfg,
                              struct spi_stm32_data* data) {
    SPI_TypeDef* spi = cfg->spi;

    if (cfg->fifo_enabled) {
        spi_stm32_shift_fifo(spi, data);
    }
    else {
        while (!ll_func_tx_is_not_full(spi)) {
            /* NOP */
        }

        spi_stm32_send_next_frame(spi, data);

        while (!ll_func_rx_is_not_empty(spi)) {
            /* NOP */
        }

        spi_stm32_read_next_frame(spi, data);
    }
}

/* Shift a SPI frame as slave. */
static void spi_stm32_shift_s(SPI_TypeDef* spi, struct spi_stm32_data* data) {
    if (ll_func_tx_is_not_full(spi) && spi_context_tx_on(&data->ctx)) {
        uint16_t tx_frame;

        if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
            tx_frame = UNALIGNED_GET((uint8_t*)(data->ctx.tx_buf));
            LL_SPI_TransmitData8(spi, (uint8_t)tx_frame);
            spi_context_update_tx(&data->ctx, 1, 1);
        }
        else {
            tx_frame = UNALIGNED_GET((uint16_t*)(data->ctx.tx_buf));
            LL_SPI_TransmitData16(spi, tx_frame);
            spi_context_update_tx(&data->ctx, 2, 1);
        }
    }
    else {
        ll_func_disable_int_tx_empty(spi);
    }

    if (ll_func_rx_is_not_empty(spi) &&
        spi_context_rx_buf_on(&data->ctx)) {
        uint16_t rx_frame;

        if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
            rx_frame = LL_SPI_ReceiveData8(spi);
            UNALIGNED_PUT((uint8_t)rx_frame, (uint8_t*)data->ctx.rx_buf);
            spi_context_update_rx(&data->ctx, 1, 1);
        }
        else {
            rx_frame = LL_SPI_ReceiveData16(spi);
            UNALIGNED_PUT(rx_frame, (uint16_t*)data->ctx.rx_buf);
            spi_context_update_rx(&data->ctx, 2, 1);
        }
    }
}

/*
 * Without a FIFO, we can only shift out one frame's worth of SPI
 * data, and read the response back.
 *
 * TODO: support 16-bit data frames.
 */
static int spi_stm32_shift_frames(const struct spi_stm32_config* cfg,
                                  struct spi_stm32_data* data) {
    SPI_TypeDef* spi = cfg->spi;
    uint16_t operation = (uint16_t)data->ctx.config->operation;

    if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER) {
        spi_stm32_shift_m(cfg, data);
    }
    else {
        spi_stm32_shift_s(spi, data);
    }

    return spi_stm32_get_err(spi);
}

static void spi_stm32_cs_control(const struct device* dev, bool on) {
    struct spi_stm32_data* data = dev->data;

    spi_context_cs_control(&data->ctx, on);

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
    const struct spi_stm32_config* cfg = dev->config;

    if (cfg->use_subghzspi_nss) {
        if (on) {
            LL_PWR_SelectSUBGHZSPI_NSS();
        }
        else {
            LL_PWR_UnselectSUBGHZSPI_NSS();
        }
    }
    #endif
}

static void spi_stm32_complete(const struct device* dev, int status) {
    const struct spi_stm32_config* cfg = dev->config;
    SPI_TypeDef* spi = cfg->spi;
    struct spi_stm32_data* data = dev->data;

    #ifdef CONFIG_SPI_STM32_INTERRUPT
    ll_func_disable_int_tx_empty(spi);
    ll_func_disable_int_rx_not_empty(spi);
    ll_func_disable_int_errors(spi);

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (cfg->fifo_enabled) {
        LL_SPI_DisableIT_EOT(spi);
    }
    #endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

    #endif

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
    /* Flush RX buffer */
    while (ll_func_rx_is_not_empty(spi)) {
        (void) LL_SPI_ReceiveData8(spi);

        if (IS_ENABLED(__GTEST)) {
            break;
        }
    }
    #endif

    if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
        while (ll_func_spi_is_busy(spi)) {
            if (IS_ENABLED(__GTEST)) {
                break;
            }
        }

        spi_stm32_cs_control(dev, false);
    }

    /* BSY flag is cleared when MODF flag is raised */
    if (LL_SPI_IsActiveFlag_MODF(spi)) {
        LL_SPI_ClearFlag_MODF(spi);
    }

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (cfg->fifo_enabled) {
        LL_SPI_ClearFlag(spi, SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_EOTC);
        LL_SPI_SetTransferSize(spi, 0);
    }
    #endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

    if (!(data->ctx.config->operation & SPI_HOLD_ON_CS)) {
        ll_func_disable_spi(spi);
    }

    #ifdef CONFIG_SPI_STM32_INTERRUPT
    spi_context_complete(&data->ctx, dev, status);
    #endif
}

#ifdef CONFIG_SPI_STM32_INTERRUPT
__maybe_unused static void /**/spi_stm32_isr(const struct device* dev) {
    const struct spi_stm32_config* cfg = dev->config;
    struct spi_stm32_data* data = dev->data;
    SPI_TypeDef* spi = cfg->spi;
    int err;

    /* Some spurious interrupts are triggered when SPI is not enabled; ignore them.
     * Do it only when fifo is enabled to leave non-fifo functionality untouched for now
     */
    if (cfg->fifo_enabled) {
        if (!LL_SPI_IsEnabled(spi)) {
            return;
        }
    }

    err = spi_stm32_get_err(spi);
    if (err != 0) {
        spi_stm32_complete(dev, err);
        return;
    }

    if (spi_stm32_transfer_ongoing(data) == true) {
        err = spi_stm32_shift_frames(cfg, data);
    }

    if ((err != 0) ||
        (spi_stm32_transfer_ongoing(data) == false)) {
        spi_stm32_complete(dev, err);
    }
}


static inline void spi_rx_isr_stpm3x(SPI_TypeDef* spi, struct spi_stm32_data* data) {
    // Receive data in 32 Bit mode when
    // Init.FifoThreshold is SPI_FIFO_THRESHOLD_04DATA
    uint32_t rx_frame;

    rx_frame = LL_SPI_ReceiveData32(spi);
    UNALIGNED_PUT(rx_frame, (uint32_t*)data->ctx.rx_buf);
    LL_SPI_DisableIT_RXP(spi);
}


#define SPI_EVENT_ERROR       (1 << 1)
#define SPI_EVENT_COMPLETE    (1 << 2)
#define SPI_EVENT_RX_OVERFLOW (1 << 3)
#define SPI_EVENT_ALL         (SPI_EVENT_ERROR | SPI_EVENT_COMPLETE | SPI_EVENT_RX_OVERFLOW)
#define SPI_EVENT_INTERNAL_TRANSFER_COMPLETE (1 << 30) // Internal flag to report that an event occurred


static void stpm3x_hal_spi_cls_xfer(SPI_TypeDef* spi,
                                    struct spi_stm32_data* data) {
    uint32_t itflag = spi->SR;

    LL_SPI_ClearFlag_EOT(spi);
    LL_SPI_ClearFlag_TXTF(spi);

    /* Disable SPI peripheral */
    LL_SPI_Disable(spi);

    /* Disable ITs */
    LL_SPI_DisableIT(spi, SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
                          SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF);

    /* Disable Tx DMA Request */
    CLEAR_BIT(spi->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    /* Report UnderRun error for non RX Only communication */
    if (data->State != HAL_SPI_STATE_BUSY_RX) {
        if ((itflag & SPI_FLAG_UDR) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_UDR);
            LL_SPI_ClearFlag_UDR(spi);
        }
    }

    /* Report OverRun error for non TX Only communication */
    if (data->State != HAL_SPI_STATE_BUSY_TX) {
        if ((itflag & SPI_FLAG_OVR) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_OVR);
            LL_SPI_ClearFlag_OVR(spi);
        }

        /* Check if CRC error occurred */
        if ((itflag & SPI_FLAG_CRCERR) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_CRC);
            LL_SPI_ClearFlag_CRCERR(spi);
        }
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if ((itflag & SPI_FLAG_MODF) != 0UL) {
        SET_BIT(data->ErrorCode, HAL_SPI_ERROR_MODF);
        LL_SPI_ClearFlag_MODF(spi);
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if ((itflag & SPI_FLAG_FRE) != 0UL) {
        SET_BIT(data->ErrorCode, HAL_SPI_ERROR_FRE);
        LL_SPI_ClearFlag_FRE(spi);
    }
}


void stpm3x_hal_spi_irq_hndl(SPI_TypeDef* spi,
                             struct spi_stm32_data* data) {
    /* @see STPM3X_SPI::irq_hndl, HAL_SPI_IRQHandler */
    uint32_t itsource = spi->IER;
    uint32_t itflag   = spi->SR;
    uint32_t trigger  = itsource & itflag;
    uint32_t cfg1     = spi->CFG1;
    uint32_t handled  = 0UL;
    HAL_SPI_StateTypeDef State = data->State;

    /* SPI in mode Transmitter and Receiver ------------------------------------*/
    if (HAL_IS_BIT_CLR(trigger, SPI_FLAG_OVR) &&
        HAL_IS_BIT_CLR(trigger, SPI_FLAG_UDR) &&
        HAL_IS_BIT_SET(trigger, SPI_FLAG_DXP)) {
        spi_rx_isr_stpm3x(spi, data);

        handled = 1UL;
    }

    if (handled != 0UL) {
        return;
    }

    /* SPI End Of Transfer: DMA or IT based transfer */
    if (HAL_IS_BIT_SET(trigger, SPI_FLAG_EOT)) {
        /* Clear EOT/TXTF/SUSP flag */
        spi->IFCR = (SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC);

        /* Disable EOT interrupt */
        LL_SPI_DisableIT_EOT(spi);

        /* DMA Normal Mode */
        if (HAL_IS_BIT_CLR(cfg1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)) {
            /* Call SPI Standard close procedure */
            stpm3x_hal_spi_cls_xfer(spi, data);

            data->State = HAL_SPI_STATE_READY;
            if (data->ErrorCode != HAL_SPI_ERROR_NONE) {
                bsp_hal_spi_err_callback(spi, data->ErrorCode);
                return;
            }
        }

        if (State == HAL_SPI_STATE_BUSY_TX_RX) {
            bsp_hal_spi_tx_rx_cplt_callback(spi);
        }
        return;
    }

    if (HAL_IS_BIT_SET(itflag, SPI_FLAG_SUSP) &&
        HAL_IS_BIT_SET(itsource, SPI_FLAG_EOT)) {
        /* Abort on going, clear SUSP flag to avoid infinite looping */
        LL_SPI_ClearFlag_SUSP(spi);

        return;
    }

    /* SPI in Error Treatment --------------------------------------------------*/
    if ((trigger & (SPI_FLAG_MODF | SPI_FLAG_OVR | SPI_FLAG_FRE | SPI_FLAG_UDR)) != 0UL) {
        /* SPI Overrun error interrupt occurred ----------------------------------*/
        if ((trigger & SPI_FLAG_OVR) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_OVR);
            LL_SPI_ClearFlag_OVR(spi);
        }

        /* SPI Mode Fault error interrupt occurred -------------------------------*/
        if ((trigger & SPI_FLAG_MODF) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_MODF);
            LL_SPI_ClearFlag_MODF(spi);
        }

        /* SPI Frame error interrupt occurred ------------------------------------*/
        if ((trigger & SPI_FLAG_FRE) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_FRE);
            LL_SPI_ClearFlag_FRE(spi);
        }

        /* SPI Underrun error interrupt occurred ------------------------------------*/
        if ((trigger & SPI_FLAG_UDR) != 0UL) {
            SET_BIT(data->ErrorCode, HAL_SPI_ERROR_UDR);
            LL_SPI_ClearFlag_UDR(spi);
        }

        if (data->ErrorCode != HAL_SPI_ERROR_NONE) {
            /* Disable SPI peripheral */
            LL_SPI_Disable(spi);

            /* Disable all interrupts */
            LL_SPI_DisableIT(spi, (SPI_IT_EOT | SPI_IT_RXP | SPI_IT_TXP | SPI_IT_MODF |
                                   SPI_IT_OVR | SPI_IT_FRE | SPI_IT_UDR));

            /* Disable the SPI DMA requests if enabled */
            if (HAL_IS_BIT_SET(cfg1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)) {
                /* Disable the SPI DMA requests */
                CLEAR_BIT(spi->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
            }
            else {
                /* Restore hspi->State to Ready */
                data->State = HAL_SPI_STATE_READY;

                bsp_hal_spi_err_callback(spi, data->ErrorCode);
            }
        }
        return;
    }
}


uint32_t spi_irq_handler_asynch(SPI_TypeDef* spi,
                                struct spi_stm32_data* data,
                                IRQn_Type irq_n) {
    int event;

    stpm3x_hal_spi_irq_hndl(spi, data);

    event = 0;
    if (data->State == HAL_SPI_STATE_READY) {
        // When HAL SPI is back to READY state, check if there was an error
        int error = data->ErrorCode;
        if (error != HAL_SPI_ERROR_NONE) {
            // something went wrong and the transfer has definitely completed
            event = (SPI_EVENT_ERROR | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE);

            if (error & HAL_SPI_ERROR_OVR) {
                // buffer overrun
                event |= SPI_EVENT_RX_OVERFLOW;
            }
        }
        else {
            // else we're done
            event = (SPI_EVENT_COMPLETE | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE);
        }

        // disable the interrupt
        NVIC_DisableIRQ(irq_n);
        NVIC_ClearPendingIRQ(irq_n);

        // reset transfer size
        LL_SPI_SetTransferSize(spi, 0);

        // HAL_SPI_TransmitReceive_IT/HAL_SPI_Transmit_IT/HAL_SPI_Receive_IT
        // function disable SPI after transfer. So we need enabled it back,
        // otherwise spi_master_block_write/spi_master_write won't work in 4-wire mode.
        LL_SPI_Enable(spi);
    }

    return (event & (SPI_EVENT_ALL | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE));
}


/**
 * @brief SPI interrupt handler for STPM3x devices
 * 
 * @param[in] dev SPI device struct
 */
__maybe_unused static void /**/spi_stpm3x_isr(const struct device* dev) {
    const struct spi_stm32_config* cfg = dev->config;
    struct spi_stm32_data* data = dev->data;
    SPI_TypeDef* spi = cfg->spi;
    uint32_t event;

    /* Some spurious interrupts are triggered when SPI is not enabled; ignore them.
     * Do it only when fifo is enabled to leave non-fifo functionality untouched for now
     */
    if (cfg->fifo_enabled) {
        if (!LL_SPI_IsEnabled(spi)) {
            return;
        }
    }

    event = spi_irq_handler_asynch(spi, data, cfg->irq);
    if (data->ctx.callback && (event & SPI_EVENT_ALL)) {
        data->ctx.callback(dev, event, data->ctx.callback_data);
    }
}
#endif

static int spi_stm32_configure(const struct device* dev,
                               const struct spi_config* config) {
    const struct spi_stm32_config* cfg = dev->config;
    struct spi_stm32_data* data = dev->data;
    const uint32_t scaler[] = {
        LL_SPI_BAUDRATEPRESCALER_DIV2,
        LL_SPI_BAUDRATEPRESCALER_DIV4,
        LL_SPI_BAUDRATEPRESCALER_DIV8,
        LL_SPI_BAUDRATEPRESCALER_DIV16,
        LL_SPI_BAUDRATEPRESCALER_DIV32,
        LL_SPI_BAUDRATEPRESCALER_DIV64,
        LL_SPI_BAUDRATEPRESCALER_DIV128,
        LL_SPI_BAUDRATEPRESCALER_DIV256
    };
    SPI_TypeDef* spi = cfg->spi;
    uint32_t clock;
    int br;

    if (spi_context_configured(&data->ctx, config)) {
        /* Nothing to do */
        return (0);
    }

    if ((SPI_WORD_SIZE_GET(config->operation) != 8) &&
        (SPI_WORD_SIZE_GET(config->operation) != 16)) {
        return (-ENOTSUP);
    }

    if ((config->operation & SPI_HALF_DUPLEX) == SPI_HALF_DUPLEX) {
        /* TODO : customized to support 3-wire SPI */
    }

    /* configure the frame format Motorola (default) or TI */
    if ((config->operation & SPI_FRAME_FORMAT_TI) == SPI_FRAME_FORMAT_TI) {
        #ifdef LL_SPI_PROTOCOL_TI
        LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_TI);
        #else
        LOG_ERR("Frame Format TI not supported");
        /* on stm32F1 or some stm32L1 (cat1,2) without SPI_CR2_FRF */
        return (-ENOTSUP);
        #endif
    #if defined(LL_SPI_PROTOCOL_MOTOROLA) && defined(SPI_CR2_FRF)
    }
    else {
        LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_MOTOROLA);
    #endif
    }

    if (IS_ENABLED(STM32_SPI_DOMAIN_CLOCK_SUPPORT) && (cfg->pclk_len > 1)) {
        if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                                   (clock_control_subsys_t)&cfg->pclken[1], &clock) < 0) {
            LOG_ERR("Failed call clock_control_get_rate(pclk[1])");
            return (-EIO);
        }
    }
    else {
        if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                                   (clock_control_subsys_t)&cfg->pclken[0], &clock) < 0) {
            LOG_ERR("Failed call clock_control_get_rate(pclk[0])");
            return (-EIO);
        }
    }

    for (br = 1; br <= ARRAY_SIZE(scaler); ++br) {
        uint32_t clk = clock >> br;

        if (clk <= config->frequency) {
            break;
        }
    }

    if (br > ARRAY_SIZE(scaler)) {
        LOG_ERR("Unsupported frequency %uHz, max %uHz, min %uHz",
                config->frequency,
                clock >> 1,
                clock >> ARRAY_SIZE(scaler));
        return (-EINVAL);
    }

    LL_SPI_Disable(spi);
    LL_SPI_SetBaudRatePrescaler(spi, scaler[br - 1]);

    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
    }
    else {
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
    }

    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
    }
    else {
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);
    }

    LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);

    if (config->operation & SPI_TRANSFER_LSB) {
        LL_SPI_SetTransferBitOrder(spi, LL_SPI_LSB_FIRST);
    }
    else {
        LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);
    }

    if (config->operation & SPI_CRC_ENABLE) {
        LL_SPI_EnableCRC(spi);
    }
    else {
        LL_SPI_DisableCRC(spi);
    }

    if (spi_cs_is_gpio(config) || !IS_ENABLED(CONFIG_SPI_STM32_USE_HW_SS)) {
        #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
        if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
            if (LL_SPI_GetNSSPolarity(spi) == LL_SPI_NSS_POLARITY_LOW) {
                LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
            }
        }
        #endif
        LL_SPI_SetNSSMode(spi, LL_SPI_NSS_SOFT);
    }
    else {
        if (config->operation & SPI_OP_MODE_SLAVE) {
            LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_INPUT);
        }
        else {
            LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_OUTPUT);
        }
    }

    if (config->operation & SPI_OP_MODE_SLAVE) {
        LL_SPI_SetMode(spi, LL_SPI_MODE_SLAVE);
    }
    else {
        LL_SPI_SetMode(spi, LL_SPI_MODE_MASTER);
    }

    if (SPI_WORD_SIZE_GET(config->operation) ==  8) {
        LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_8BIT);
    }
    else {
        LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_16BIT);
    }

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    LL_SPI_SetMasterSSIdleness(spi, cfg->mssi_clocks);
    LL_SPI_SetInterDataIdleness(spi, (cfg->midi_clocks << SPI_CFG2_MIDI_Pos));
    #endif

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
    ll_func_set_fifo_threshold_8bit(spi);
    #endif

    /* At this point, it's mandatory to set this on the context! */
    data->ctx.config = config;

    LOG_DBG("Installed config %p: freq %uHz (div = %u),"
            " mode %u/%u/%u, slave %u",
            config, clock >> br, 1 << br,
            (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) ? 1 : 0,
            (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0,
            (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) ? 1 : 0,
            config->slave);

    return (0);
}

static int spi_stm32_release(const struct device* dev,
                             const struct spi_config* config) {
    struct spi_stm32_data* data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;

    spi_context_unlock_unconditionally(&data->ctx);
	ll_func_disable_spi(cfg->spi);

    return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
static int32_t spi_stm32_count_bufset_frames(const struct spi_config* config,
                                             const struct spi_buf_set* bufs) {
    if (bufs == NULL) {
        return (0);
    }

    uint32_t num_bytes = 0;

    for (size_t i = 0; i < bufs->count; i++) {
        num_bytes += bufs->buffers[i].len;
    }

    uint8_t bytes_per_frame = SPI_WORD_SIZE_GET(config->operation) / 8;

    if ((num_bytes % bytes_per_frame) != 0) {
        return -EINVAL;
    }

    return (num_bytes / bytes_per_frame);
}

static int32_t spi_stm32_count_total_frames(const struct spi_config* config,
                                            const struct spi_buf_set* tx_bufs,
                                            const struct spi_buf_set* rx_bufs) {
    int tx_frames = spi_stm32_count_bufset_frames(config, tx_bufs);

    if (tx_frames < 0) {
        return (tx_frames);
    }

    int rx_frames = spi_stm32_count_bufset_frames(config, rx_bufs);

    if (rx_frames < 0) {
        return (rx_frames);
    }

    if (tx_frames > UINT16_MAX || rx_frames > UINT16_MAX) {
        return (-EMSGSIZE);
    }

    return MAX(rx_frames, tx_frames);
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

static int transceive(const struct device* dev,
                      const struct spi_config* config,
                      const struct spi_buf_set* tx_bufs,
                      const struct spi_buf_set* rx_bufs,
                      bool asynchronous,
                      spi_callback_t cb,
                      void* userdata) {
    const struct spi_stm32_config* cfg = dev->config;
    struct spi_stm32_data* data = dev->data;
    SPI_TypeDef* spi = cfg->spi;
    int ret;

    if (!tx_bufs && !rx_bufs) {
        return (0);
    }

    #ifndef CONFIG_SPI_STM32_INTERRUPT
    if (asynchronous) {
        return (-ENOTSUP);
    }
    #endif

    spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

    ret = spi_stm32_configure(dev, config);
    if (ret) {
        goto end;
    }

    /* Set buffers info */
    if (SPI_WORD_SIZE_GET(config->operation) == 8) {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
    }
    else {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 2);
    }

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (cfg->fifo_enabled && SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
        int total_frames = spi_stm32_count_total_frames(
            config, tx_bufs, rx_bufs);
        if (total_frames < 0) {
            ret = total_frames;
            goto end;
        }
        LL_SPI_SetTransferSize(spi, (uint32_t)total_frames);
    }

    #endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
    /* Flush RX buffer */
    while (ll_func_rx_is_not_empty(spi)) {
        (void) LL_SPI_ReceiveData8(spi);

        if (IS_ENABLED(__GTEST)) {
            break;
        }
    }
    #endif

    LL_SPI_Enable(spi);

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    /* With the STM32MP1, STM32U5 and the STM32H7,
     * if the device is the SPI master,
     * we need to enable the start of the transfer with
     * LL_SPI_StartMasterTransfer(spi)
     */
    if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
        LL_SPI_StartMasterTransfer(spi);
        while (!LL_SPI_IsActiveMasterTransfer(spi)) {
            /* NOP */
        }
    }
    #endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

    #if CONFIG_SOC_SERIES_STM32H7X
    /*
     * Add a small delay after enabling to prevent transfer stalling at high
     * system clock frequency (see errata sheet ES0392).
     */
    k_busy_wait(WAIT_1US);
    #endif

    /* This is turned off in spi_stm32_complete(). */
    spi_stm32_cs_control(dev, true);

    #ifdef CONFIG_SPI_STM32_INTERRUPT

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    if (cfg->fifo_enabled) {
        LL_SPI_EnableIT_EOT(spi);
    }
    #endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi) */

    ll_func_enable_int_errors(spi);

    if (rx_bufs) {
        ll_func_enable_int_rx_not_empty(spi);
    }

    ll_func_enable_int_tx_empty(spi);

    ret = spi_context_wait_for_completion(&data->ctx);
    #else
    do {
        ret = spi_stm32_shift_frames(cfg, data);
    } while (!ret && spi_stm32_transfer_ongoing(data));

    spi_stm32_complete(dev, ret);

    #ifdef CONFIG_SPI_SLAVE
    if (spi_context_is_slave(&data->ctx) && !ret) {
        ret = data->ctx.recv_frames;
    }
    #endif /* CONFIG_SPI_SLAVE */

    #endif

end :
    spi_context_release(&data->ctx, ret);

    return (ret);
}

bool spi_stpm3x_is_active(struct spi_dt_spec const* spec) {
    struct spi_stm32_config const* cfg = spec->bus->config;
    SPI_TypeDef* spi = cfg->spi;
    bool is_active;

    is_active = (bool)LL_SPI_IsActiveMasterTransfer(spi);

    return (is_active);
}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stpm3x_spi)
int spi_stpm3x_transceive_dt(struct spi_dt_spec const* spec,
                             uint8_t const tx[],
                             uint8_t rx[]) {
    struct spi_stm32_config const* cfg = spec->bus->config;
    struct spi_stm32_data* data = spec->bus->data;
    SPI_TypeDef* spi = cfg->spi;

    // Register the thunking handler
    IRQn_Type irq_n = cfg->irq;

    // Enable the interrupt
    NVIC_DisableIRQ(irq_n);
    NVIC_ClearPendingIRQ(irq_n);
    NVIC_EnableIRQ(irq_n);

    // Flush FIFO
    #if defined(SPI_FLAG_FRLVL)             // STM32F0 STM32F3 STM32F7 STM32L4
    LL_SPIEx_FlushRxFifo(spi);
    #endif

    // Set the function for IT treatment
    // @note already set in _spi_init_direct with HAL_SPI_Set_TxISR_Rx_ISR_8BIT
    data->ctx.rx_buf = rx;
    data->ctx.rx_len = 0U;
    data->ctx.tx_len = 0U;

    // Set the number of data at current transfer
    MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, 4U);
    MODIFY_REG(spi->CFG1, SPI_CFG1_FTHLV, SPI_FIFO_THRESHOLD_04DATA);

    /* Enable SPI peripheral */
    LL_SPI_Enable(spi);

    __IO uint8_t* ptxdr_8bits = (__IO uint8_t*)(&(spi->TXDR));

    // Transmit data in 8 Bit mode and put in TxFIFO
    *ptxdr_8bits = tx[0];
    *ptxdr_8bits = tx[1];
    *ptxdr_8bits = tx[2];
    *ptxdr_8bits = tx[3];

    // Enable EOT, RXP, DXP, UDR, OVR, FRE, MODF and TSERF interrupts
    // No TXP since we already put all of data into TxFIFO
    LL_SPI_EnableIT(spi, (SPI_IT_EOT | SPI_IT_RXP  |
                          SPI_IT_DXP | SPI_IT_UDR  | SPI_IT_OVR |
                          SPI_IT_FRE | SPI_IT_MODF | SPI_IT_TSERF));

    // Master transfer start
    SET_BIT(spi->CR1, SPI_CR1_CSTART);

    return (0);
}

int spi_stpm3x_init(struct spi_dt_spec const* spec,
                    spi_callback_t cb,
                    void* userdata) {
    struct spi_stm32_data* data = spec->bus->data;
    int ret;

    ret = spi_stm32_configure(spec->bus, &spec->config);
    if (ret == 0) {
        data->ctx.asynchronous = true;
        data->ctx.callback = cb;
        data->ctx.callback_data = userdata;
    }

    return (ret);
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stpm3x_spi) */

#ifdef CONFIG_SPI_STM32_DMA
static int wait_dma_rx_tx_done(const struct device* dev) {
    struct spi_stm32_data* data = dev->data;
    int res;
    k_timeout_t timeout;

    /*
     * In slave mode we do not know when the transaction will start. Hence,
     * it doesn't make sense to have timeout in this case.
     */
    if (IS_ENABLED(CONFIG_SPI_SLAVE) &&
        spi_context_is_slave(&data->ctx)) {
        timeout = K_FOREVER;
    }
    else {
        timeout = K_MSEC(1000);
    }

    while (1) {
        res = k_sem_take(&data->status_sem, timeout);
        if (res != 0) {
            return (res);
        }

        if (data->status_flags & SPI_STM32_DMA_ERROR_FLAG) {
            return (-EIO);
        }

        if (data->status_flags & SPI_STM32_DMA_DONE_FLAG) {
            return (0);
        }
    }

    return (res);
}

#ifdef CONFIG_SOC_SERIES_STM32H7X
static bool buf_in_nocache(uintptr_t buf, size_t len_bytes) {
    bool buf_within_nocache = false;

    #ifdef CONFIG_NOCACHE_MEMORY
    buf_within_nocache = ((buf >= ((uintptr_t)_nocache_ram_start)) &&
                          ((buf + len_bytes - 1) <= ((uintptr_t)_nocache_ram_end)));
    if (buf_within_nocache) {
        return (true);
    }
    #endif /* CONFIG_NOCACHE_MEMORY */

    buf_within_nocache = (mem_attr_check_buf(
            (void*)buf, len_bytes, DT_MEM_ARM(ATTR_MPU_RAM_NOCACHE)) == 0);

    return (buf_within_nocache);
}

static bool is_dummy_buffer(const struct spi_buf* buf) {
    return (buf->buf == NULL);
}

static bool spi_buf_set_in_nocache(const struct spi_buf_set* bufs) {
    for (size_t i = 0; i < bufs->count; i++) {
        const struct spi_buf* buf = &bufs->buffers[i];

        if (!is_dummy_buffer(buf) &&
            !buf_in_nocache((uintptr_t)buf->buf, buf->len)) {
            return (false);
        }
    }
    return (true);
}
#endif /* CONFIG_SOC_SERIES_STM32H7X */

static int transceive_dma(const struct device* dev,
                          const struct spi_config* config,
                          const struct spi_buf_set* tx_bufs,
                          const struct spi_buf_set* rx_bufs,
                          bool asynchronous,
                          spi_callback_t cb,
                          void* userdata) {
    const struct spi_stm32_config* cfg = dev->config;
    struct spi_stm32_data* data = dev->data;
    SPI_TypeDef* spi = cfg->spi;
    int ret;

    if (!tx_bufs && !rx_bufs) {
        return (0);
    }

    if (asynchronous) {
        return (-ENOTSUP);
    }

    #ifdef CONFIG_SOC_SERIES_STM32H7X
    if (((tx_bufs != NULL) && !spi_buf_set_in_nocache(tx_bufs)) ||
        ((rx_bufs != NULL) && !spi_buf_set_in_nocache(rx_bufs))) {
        return (-EFAULT);
    }
    #endif /* CONFIG_SOC_SERIES_STM32H7X */

    spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

    k_sem_reset(&data->status_sem);

    ret = spi_stm32_configure(dev, config);
    if (ret) {
        goto end;
    }

    /* Set buffers info */
    if (SPI_WORD_SIZE_GET(config->operation) == 8) {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
    }
    else {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 2);
    }

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
    /* set request before enabling (else SPI CFG1 reg is write protected) */
    LL_SPI_EnableDMAReq_RX(spi);
    LL_SPI_EnableDMAReq_TX(spi);

    LL_SPI_Enable(spi);
    if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
        LL_SPI_StartMasterTransfer(spi);
    }
    #else
    LL_SPI_Enable(spi);
    #endif /* st_stm32h7_spi */

    /* This is turned off in spi_stm32_complete(). */
    spi_stm32_cs_control(dev, true);

    while ((data->ctx.rx_len > 0) || (data->ctx.tx_len > 0)) {
        size_t dma_len;

        if (data->ctx.rx_len == 0) {
            dma_len = data->ctx.tx_len;
        }
        else if (data->ctx.tx_len == 0) {
            dma_len = data->ctx.rx_len;
        }
        else {
            dma_len = MIN(data->ctx.tx_len, data->ctx.rx_len);
        }

        data->status_flags = 0;

        ret = spi_dma_move_buffers(dev, dma_len);
        if (ret != 0) {
            break;
        }

        #if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
        /* toggle the DMA request to restart the transfer */
        LL_SPI_EnableDMAReq_RX(spi);
        LL_SPI_EnableDMAReq_TX(spi);
        #endif /* ! st_stm32h7_spi */

        ret = wait_dma_rx_tx_done(dev);
        if (ret != 0) {
            break;
        }

        #ifdef SPI_SR_FTLVL
        while (LL_SPI_GetTxFIFOLevel(spi) > 0) {
            /* pass */
        }
        #endif

        #ifdef CONFIG_SPI_STM32_ERRATA_BUSY
        WAIT_FOR(ll_func_spi_dma_busy(spi) != 0,
                 CONFIG_SPI_STM32_BUSY_FLAG_TIMEOUT,
                 k_yield());
        #else
        /* wait until spi is no more busy (spi TX fifo is really empty) */
        while (ll_func_spi_dma_busy(spi) == 0) {
            if (IS_ENABLED(__GTEST)) {
                break;
            }
        }
        #endif

        #if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
        /* toggle the DMA transfer request */
        LL_SPI_DisableDMAReq_TX(spi);
        LL_SPI_DisableDMAReq_RX(spi);
        #endif /* ! st_stm32h7_spi */

        uint8_t frame_size_bytes = (uint8_t)bits2bytes(
                SPI_WORD_SIZE_GET(config->operation));

        spi_context_update_tx(&data->ctx, frame_size_bytes, dma_len);
        spi_context_update_rx(&data->ctx, frame_size_bytes, dma_len);
    }

    /* spi complete relies on SPI Status Reg which cannot be disabled */
    spi_stm32_complete(dev, ret);
    /* disable spi instance after completion */
    LL_SPI_Disable(spi);
    /* The Config. Reg. on some mcus is write un-protected when SPI is disabled */
    LL_SPI_DisableDMAReq_TX(spi);
    LL_SPI_DisableDMAReq_RX(spi);

    dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
    dma_stop(data->dma_tx.dma_dev, data->dma_tx.channel);

    #ifdef CONFIG_SPI_SLAVE
    if (spi_context_is_slave(&data->ctx) && !ret) {
        ret = data->ctx.recv_frames;
    }
    #endif /* CONFIG_SPI_SLAVE */

end:
    spi_context_release(&data->ctx, ret);

    return (ret);
}
#endif /* CONFIG_SPI_STM32_DMA */

static int spi_stm32_transceive(const struct device* dev,
                                const struct spi_config* config,
                                const struct spi_buf_set* tx_bufs,
                                const struct spi_buf_set* rx_bufs) {
    #ifdef CONFIG_SPI_STM32_DMA
    struct spi_stm32_data const* data = dev->data;

    if ((data->dma_tx.dma_dev != NULL) &&
        (data->dma_rx.dma_dev != NULL)) {
        return transceive_dma(dev, config, tx_bufs, rx_bufs,
                              false, NULL, NULL);
    }
    #endif /* CONFIG_SPI_STM32_DMA */
    return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_stm32_transceive_async(const struct device* dev,
                                      const struct spi_config* config,
                                      const struct spi_buf_set* tx_bufs,
                                      const struct spi_buf_set* rx_bufs,
                                      spi_callback_t cb,
                                      void* userdata) {
    return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static struct spi_driver_api DT_CONST api_funcs = {
    .transceive = spi_stm32_transceive,
    #ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_stm32_transceive_async,
    #endif
    .release = spi_stm32_release,
};

static inline bool spi_stm32_is_subghzspi(const struct device* dev) {
    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
    const struct spi_stm32_config* cfg = dev->config;

    return (cfg->use_subghzspi_nss);
    #else
    ARG_UNUSED(dev);
    return (false);
    #endif
}

static int spi_stm32_init(const struct device* dev) {
    struct spi_stm32_data* data __attribute__((unused)) = dev->data;
    const struct spi_stm32_config* cfg = dev->config;
    int err;

    if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
        LOG_ERR("clock control device not ready");
        return (-ENODEV);
    }

    err = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                           (clock_control_subsys_t)&cfg->pclken[0]);
    if (err < 0) {
        LOG_ERR("Could not enable SPI clock");
        return (err);
    }

    if (IS_ENABLED(STM32_SPI_DOMAIN_CLOCK_SUPPORT) && (cfg->pclk_len > 1)) {
        err = clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                                      (clock_control_subsys_t)&cfg->pclken[1],
                                      NULL);
        if (err < 0) {
            LOG_ERR("Could not select SPI domain clock");
            return (err);
        }
    }

    if (!spi_stm32_is_subghzspi(dev)) {
        /* Configure dt provided device signals when available */
        err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0) {
            LOG_ERR("SPI pinctrl setup failed (%d)", err);
            return (err);
        }
    }

    #ifdef CONFIG_SPI_STM32_INTERRUPT
    cfg->irq_config(dev);
    #endif

    LL_SPI_EnableGPIOControl(cfg->spi);

    #ifdef CONFIG_SPI_STM32_DMA
    if ((data->dma_rx.dma_dev != NULL) &&
        !device_is_ready(data->dma_rx.dma_dev)) {
        LOG_ERR("%s device not ready", data->dma_rx.dma_dev->name);
        return (-ENODEV);
    }

    if ((data->dma_tx.dma_dev != NULL) &&
        !device_is_ready(data->dma_tx.dma_dev)) {
        LOG_ERR("%s device not ready", data->dma_tx.dma_dev->name);
        return (-ENODEV);
    }

    LOG_DBG("SPI with DMA transfer");

    #endif /* CONFIG_SPI_STM32_DMA */

    err = spi_context_cs_configure_all(&data->ctx);
    if (err < 0) {
        return (err);
    }

    spi_context_unlock_unconditionally(&data->ctx);

    return (0);
}

#ifdef CONFIG_SPI_STM32_INTERRUPT
#define STM32_SPI_IRQ_CONNECT(id)                   \
    COND_CODE_1(DT_INST_PROP(id, stpm3x_isr),       \
        (IRQ_CONNECT(DT_INST_IRQN(id),              \
                     DT_INST_IRQ(id, priority),     \
                     spi_stpm3x_isr,                \
                     DEVICE_DT_INST_GET(id), 0)),   \
        (IRQ_CONNECT(DT_INST_IRQN(id),              \
                     DT_INST_IRQ(id, priority),     \
                     spi_stm32_isr,                 \
                     DEVICE_DT_INST_GET(id), 0))    \
    )
#define STM32_SPI_IRQ_HANDLER_DECL(id)      \
        static void spi_stm32_irq_config_func_##id(const struct device* dev)
#define STM32_SPI_IRQ_HANDLER_FUNC(id)      \
        .irq_config = spi_stm32_irq_config_func_##id,
#define STM32_SPI_IRQ_HANDLER(id)           \
static void spi_stm32_irq_config_func_##id(const struct device* dev) {  \
    STM32_SPI_IRQ_CONNECT(id);              \
    irq_enable(DT_INST_IRQN(id));           \
}
#define STM32_SPI_IRQ_NUM(id)   .irq = DT_INST_IRQN(id),
#else
#define STM32_SPI_IRQ_CONNECT(id)
#define STM32_SPI_IRQ_HANDLER_DECL(id)
#define STM32_SPI_IRQ_HANDLER_FUNC(id)
#define STM32_SPI_IRQ_HANDLER(id)
#define STM32_SPI_IRQ_NUM(id)
#endif

#define SPI_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)    \
    .dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(index, dir)),       \
    .channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),  \
    .dma_cfg = {                            \
        .dma_slot            = STM32_DMA_SLOT(index, dir, slot),\
        .channel_direction   = STM32_DMA_CONFIG_DIRECTION(      \
                                  STM32_DMA_CHANNEL_CONFIG(index, dir)),\
        .source_data_size    = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(  \
                                  STM32_DMA_CHANNEL_CONFIG(index, dir)),\
        .dest_data_size      = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE( \
                                  STM32_DMA_CHANNEL_CONFIG(index, dir)),\
        .source_burst_length = 1, /* SINGLE transfer */         \
        .dest_burst_length   = 1, /* SINGLE transfer */         \
        .channel_priority    = STM32_DMA_CONFIG_PRIORITY(       \
                                  STM32_DMA_CHANNEL_CONFIG(index, dir)),\
        .dma_callback        = spi_stm32_dma_callback,\
        .block_count         = 2,           \
    },                                      \
    .src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(\
                             STM32_DMA_CHANNEL_CONFIG(index, dir)),     \
    .dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(       \
                             STM32_DMA_CHANNEL_CONFIG(index, dir)),     \
    .fifo_threshold     = STM32_DMA_FEATURES_FIFO_THRESHOLD(    \
                             STM32_DMA_FEATURES(index, dir)),   \

#if CONFIG_SPI_STM32_DMA
#define SPI_DMA_CHANNEL(id, dir, DIR, src, dest)            \
    .dma_##dir = {                          \
        COND_CODE_1(DT_INST_DMAS_HAS_NAME(id, dir),         \
                    (SPI_DMA_CHANNEL_INIT(id, dir, DIR, src, dest)), \
                    (NULL))                 \
    },

#define SPI_DMA_STATUS_SEM(id)              \
    .status_sem = Z_SEM_INITIALIZER(        \
            spi_stm32_dev_data_##id.status_sem, 0, 1),
#else
#define SPI_DMA_CHANNEL(id, dir, DIR, src, dest)
#define SPI_DMA_STATUS_SEM(id)
#endif

#define SPI_SUPPORTS_FIFO(id)   DT_INST_NODE_HAS_PROP(id, fifo_enable)
#define SPI_GET_FIFO_PROP(id)   DT_INST_PROP(id, fifo_enable)
#define SPI_FIFO_ENABLED(id)    COND_CODE_1(SPI_SUPPORTS_FIFO(id), (SPI_GET_FIFO_PROP(id)), (0))

#define STM32_SPI_INIT(id)                                      \
STM32_SPI_IRQ_HANDLER_DECL(id);                                 \
                                                                \
PINCTRL_DT_INST_DEFINE(id);                                     \
                                                                \
static const struct stm32_pclken pclken_##id[] =                \
                          STM32_DT_INST_CLOCKS(id);             \
                                                                \
static struct spi_stm32_config DT_CONST spi_stm32_cfg_##id = {  \
    .spi      = (SPI_TypeDef*)DT_INST_REG_ADDR(id),             \
    .pclken   = pclken_##id,                                    \
    .pclk_len = DT_INST_NUM_CLOCKS(id),                         \
    .pcfg     = PINCTRL_DT_INST_DEV_CONFIG_GET(id),             \
    .fifo_enabled = SPI_FIFO_ENABLED(id),                       \
    STM32_SPI_IRQ_HANDLER_FUNC(id)                              \
    STM32_SPI_IRQ_NUM(id)                                       \
    IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz),  \
        (.use_subghzspi_nss =                                   \
            DT_INST_PROP_OR(id, use_subghzspi_nss, false),))    \
    IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi),       \
        (.midi_clocks =                                         \
            DT_INST_PROP(id, midi_clock),))                     \
    IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi),       \
        (.mssi_clocks =                                         \
            DT_INST_PROP(id, mssi_clock),))                     \
};                                                              \
                                                                \
static struct spi_stm32_data spi_stm32_dev_data_##id = {        \
    SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_##id, ctx),        \
    SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_##id, ctx),        \
    SPI_DMA_CHANNEL(id, rx, RX, PERIPHERAL, MEMORY)             \
    SPI_DMA_CHANNEL(id, tx, TX, MEMORY, PERIPHERAL)             \
    SPI_DMA_STATUS_SEM(id)                                      \
    SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(id), ctx)       \
};                                                              \
                                                                \
DEVICE_DT_INST_DEFINE(id, &spi_stm32_init, NULL,                \
                      &spi_stm32_dev_data_##id, &spi_stm32_cfg_##id, \
                      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
                      &api_funcs);                              \
                                                                \
STM32_SPI_IRQ_HANDLER(id)

DT_INST_FOREACH_STATUS_OKAY(STM32_SPI_INIT)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

static void zephyr_gtest_spi_stm32_cfg_init(struct spi_stm32_config* cfg) {
    uintptr_t base_addr = (uintptr_t)cfg->spi;

    switch (base_addr) {
        case SPI1_BASE : {
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi1_ptr;
            break;
        }

        case SPI2_BASE : {
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi2_ptr;
            break;
        }

        case SPI3_BASE : {
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi3_ptr;
            break;
        }

        case SPI4_BASE : {
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi4_ptr;
            break;
        }

        case SPI5_BASE : {
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi5_ptr;
            break;
        }

        default : { // SPI6_BASE
            cfg->spi = (SPI_TypeDef*)ut_mcu_spi6_ptr;
            break;
        }
    }
}

void zephyr_gtest_spi_stm32(void) {
    zephyr_gtest_spi_stm32_cfg_init(&spi_stm32_cfg_0);
    zephyr_gtest_spi_stm32_cfg_init(&spi_stm32_cfg_1);
    zephyr_gtest_spi_stm32_cfg_init(&spi_stm32_cfg_2);
}

#endif


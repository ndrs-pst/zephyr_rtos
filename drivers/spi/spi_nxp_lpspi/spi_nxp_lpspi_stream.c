/*
 * Copyright 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Continuous SPI slave RX streaming using LPSPI + eDMA circular buffer.
 *
 * Design summary
 * --------------
 * spi_read_stream_async() configures:
 *   1. LPSPI in slave mode with TCR.TXMSK=1 (RX-only, no TX output).
 *   2. FCR.RXWATER=0 (DMA request per word).
 *   3. IER.FCIE=1  (interrupt on Frame Complete Flag = CS deassertion).
 *   4. eDMA channel (dma_config.cyclic=1) from LPSPI_RDR into ring_buf.
 *      DLAST = -(int32_t)ring_buf_size wraps destination automatically.
 *   5. DER.RDDE=1  (RX DMA request enable).
 *
 * On each CS deassertion lpspi_isr() fires FCIE → calls
 * lpspi_stream_isr_fcf_handler() which:
 *   - Records the frame start/length into a pool descriptor (no alloc).
 *   - Posts descriptor to cfg->frame_fifo (k_fifo_put, ISR-safe).
 *   - Advances write_pos by frame_size (software ring pointer).
 *
 * The application thread drains cfg->frame_fifo with k_fifo_get() and
 * reads directly from the ring buffer — zero CPU copy.
 *
 * Restrictions
 * ------------
 *   - Peripheral mode only.
 *   - Fixed frame_size per streaming session.
 *   - ring_buf_size must be a multiple of frame_size, >= 2×frame_size.
 *   - LPSPI DTS node must have "dmas" property with an "rx" entry.
 *   - CONFIG_SPI_NXP_LPSPI_DMA must be enabled (DMA driver wired).
 *   - Concurrent spi_transceive() on the same device is rejected with EBUSY.
 */

/* (01) LOCAL PREDEFINE */
#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/sys/slist.h>
#include "spi_nxp_lpspi_priv.h"

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */
static inline LPSPI_Type* lpspi_base(const struct device* dev) {
    return (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
}

/* -------------------------------------------------------------------------
 * DMA configuration
 * ------------------------------------------------------------------------- */
/**
 * No-op DMA callback for the streaming RX channel.
 *
 * dma_mcux_edma.c wires nxp_edma_callback unconditionally onto every channel
 * and calls data->dma_callback() after each major loop (INTMAJOR fires when
 * EDMA_PrepareTransfer sets enabledInterruptMask=kEDMA_MajorInterruptEnable).
 * A NULL dma_callback pointer would cause a null-pointer crash on that call.
 *
 * For the streaming path, frame notification is driven entirely by the FCF
 * interrupt — the DMA major-loop interrupt is not needed.  This callback
 * simply absorbs the periodic major-loop interrupt (one per full ring buffer
 * traversal) without taking any action.
 */
static void lpspi_stream_dma_callback(const struct device* dma_dev,
                                      void* user_data,
                                      uint32_t channel, int status) {
    ARG_UNUSED(dma_dev);
    ARG_UNUSED(user_data);
    ARG_UNUSED(channel);
    ARG_UNUSED(status);
    /* Intentionally empty: FCF ISR owns frame notification */
}

/**
 * Configure the RX DMA channel for cyclic (circular) operation.
 *
 * Source : fixed at &lpspi->RDR (peripheral, no address increment).
 * Dest   : ring_buf base with automatic wrap (DLAST = -ring_buf_size).
 * Trigger: LPSPI RDDE DMA request (one minor loop per received word).
 *
 * Circular wrap is implemented by EDMA_PrepareTransfer() for
 * kEDMA_PeripheralToMemory: it sets DLAST_SGA = -block_size and SLAST_SDA = 0,
 * so the destination address wraps to ring_buf[0] after each major loop while
 * the source (RDR) remains fixed.  DREQ is left 0 (don't auto-stop) for
 * peripheral transfers, so the channel restarts immediately.
 *
 * INTMAJOR is set by EDMA_PrepareTransfer (kEDMA_MajorInterruptEnable).
 * lpspi_stream_dma_callback() is registered to absorb those interrupts safely
 * without performing any work — FCF/FCIE is the actual frame notification path.
 *
 * source_data_size and source_burst_length are derived from the SPI word size
 * (SPI_WORD_SIZE_GET) rather than inherited from the normal RX DMA config.
 * This ensures the TCD is valid even before any spi_transceive() has run,
 * and guarantees NBYTES == SSIZE (eDMA requirement).
 */
static int lpspi_stream_dma_configure(const struct device* dev) {
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream_data;
    const struct spi_stream_cfg* cfg = stream->cfg;
    struct spi_nxp_dma_data* dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    int rc;

    /* ---- Block config ---- */
    struct dma_block_config* dma_blk = &stream->dma_blk;
    memset(dma_blk, 0, sizeof(struct dma_block_config));
    dma_blk->source_address   = (uint32_t)(uintptr_t)&lpspi->RDR;
    dma_blk->dest_address     = (uint32_t)(uintptr_t)cfg->ring_buf;
    dma_blk->block_size       = cfg->ring_buf_size;
    dma_blk->source_addr_adj  = DMA_ADDR_ADJ_NO_CHANGE;         /* RDR stays fixed */
    dma_blk->dest_addr_adj    = DMA_ADDR_ADJ_INCREMENT;         /* Advance through ring */
    dma_blk->dest_reload_en   = 1U;                             /* Belt-and-suspenders: DLAST already set by FSL SDK */
    dma_blk->source_reload_en = 0U;

    /*
     * Derive word width from SPI configuration so this is independent of
     * whether a normal spi_transceive() has ever run on this device.
     * dma_data->dma_rx.dma_cfg fields are populated only at transceive time
     * and would be zero on first use — invalid for dma_mcux_edma_validate_cfg().
     */
    uint8_t word_sz_bits = (uint8_t)SPI_WORD_SIZE_GET(cfg->spi_cfg->operation);
    uint8_t word_sz;

    if (word_sz_bits <= 8U) {
        word_sz = 1U;
    }
    else if (word_sz_bits <= 16U) {
        word_sz = 2U;
    }
    else {
        word_sz = 4U;
    }

    /* ---- Channel config ---- */
    struct dma_config* dma_cfg = &stream->dma_cfg;
    memset(dma_cfg, 0, sizeof(struct dma_config));
    dma_cfg->channel_direction = PERIPHERAL_TO_MEMORY;
    dma_cfg->dma_slot          = dma_data->dma_rx.dma_cfg.dma_slot;
    dma_cfg->source_data_size  = word_sz;
    dma_cfg->dest_data_size    = word_sz;

    /* NBYTES must equal SSIZE — both derived from word_sz. */
    dma_cfg->source_burst_length  = word_sz;
    dma_cfg->dest_burst_length    = word_sz;
    dma_cfg->cyclic               = 1U;
    dma_cfg->complete_callback_en = 1U;
    dma_cfg->error_callback_dis   = 0U;
    dma_cfg->block_count          = 1U;
    dma_cfg->head_block           = &stream->dma_blk;

    /* Must be non-NULL: nxp_edma_callback calls dma_callback unconditionally. */
    dma_cfg->dma_callback = lpspi_stream_dma_callback;
    dma_cfg->user_data    = (void*)dev;

    rc = dma_config(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel, dma_cfg);

    return rc;
}

/* -------------------------------------------------------------------------
 * ISR-context FCF handler (called from lpspi_isr in spi_nxp_lpspi_dma.c)
 * ------------------------------------------------------------------------- */
/**
 * lpspi_stream_isr_fcf_handler() - Handle Frame Complete interrupt.
 *
 * Runs at interrupt priority.  Must not block, allocate, or call any
 * sleeping Zephyr API.  Execution time is O(1).
 *
 * Sequence:
 *   1. Record frame_start = current write_pos.
 *   2. Advance write_pos by frame_size (wraps modulo ring_buf_size).
 *   3. Acquire next descriptor from round-robin pool (no malloc).
 *   4. Fill descriptor fields.
 *   5. k_fifo_put() to notify consumer thread.
 */
void lpspi_stream_isr_fcf_handler(const struct device* dev) {
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream_data;
    const struct spi_stream_cfg* cfg;
    uint32_t frame_start;
    uint32_t pool_idx;
    struct spi_stream_frame* desc;

    /* Guard order matters: check stream before dereferencing it for cfg */
    if (stream == NULL) {
        return;
    }

    cfg = stream->cfg;
    if ((cfg == NULL) || (atomic_get(&stream->active) == 0)) {
        return;
    }

    /* Step 1: snapshot current write position (= start of just-received frame) */
    frame_start = stream->write_pos;

    /* Step 2: advance write pointer for next frame */
    stream->write_pos = (frame_start + (uint32_t)cfg->frame_size) % (uint32_t)cfg->ring_buf_size;

    /* Step 3: get next descriptor slot (round-robin, no allocation) */
    pool_idx = stream->desc_pool_head % (uint32_t)cfg->frame_pool_count;
    stream->desc_pool_head++;

    desc = &cfg->frame_pool[pool_idx];

    /*
     * Overrun guard: sys_snode_peek_next returns non-NULL while the node is
     * still linked inside the k_fifo (k_fifo_get sets next=NULL on removal).
     * If the consumer has not yet drained this slot, posting it again would
     * corrupt the fifo linked list.  Drop the frame, record the overrun, and
     * let the consumer detect the gap via frame_idx discontinuity.
     */
    if (sys_slist_peek_next(&desc->node) != NULL) {
        stream->overrun_count++;
        return;
    }

    /* Step 4: populate descriptor */
    desc->data = (cfg->ring_buf + frame_start);
    desc->len  = cfg->frame_size;
    desc->frame_idx = (uint32_t)atomic_inc(&stream->frame_idx);

    /* Step 5: notify consumer thread (ISR-safe, non-blocking) */
    k_fifo_put(cfg->frame_fifo, desc);
}

/* -------------------------------------------------------------------------
 * Public API implementation
 * ------------------------------------------------------------------------- */
int spi_read_stream_async(const struct device* dev, const struct spi_stream_cfg* cfg) {
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream_data;
    struct spi_nxp_dma_data* dma_data;
    int ret;

    if (stream == NULL) {
        LOG_ERR("stream: CONFIG_SPI_NXP_LPSPI_STREAM not enabled or stream_data NULL");
        return (-ENODEV);
    }

    /* Validate configuration */
    if ((cfg == NULL)             || (cfg->spi_cfg == NULL)     ||
        (cfg->ring_buf == NULL)   || (cfg->frame_pool  == NULL) ||
        (cfg->frame_fifo == NULL) || (cfg->ring_buf_size == 0U) ||
        (cfg->frame_size == 0U)   || (cfg->frame_pool_count == 0U)) {
        return (-EINVAL);
    }

    if ((cfg->ring_buf_size % cfg->frame_size) != 0U) {
        LOG_ERR("stream: ring_buf_size must be a multiple of frame_size");
        return (-EINVAL);
    }

    if (cfg->ring_buf_size < (2U * cfg->frame_size)) {
        LOG_ERR("stream: ring_buf_size must be >= 2 × frame_size");
        return (-EINVAL);
    }

    if (cfg->frame_pool_count < (cfg->ring_buf_size / cfg->frame_size)) {
        LOG_ERR("stream: frame_pool_count must be >= ring_buf_size / frame_size");
        return (-EINVAL);
    }

    /* Only slave mode is supported */
    if ((cfg->spi_cfg->operation & SPI_OP_MODE_SLAVE) == 0U) {
        LOG_ERR("stream: slave mode (SPI_OP_MODE_SLAVE) required");
        return (-EINVAL);
    }

    /* Prevent concurrent streaming */
    if (atomic_cas(&stream->active, 0, 1) == false) {
        return (-EBUSY);
    }

    dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    if (dma_data == NULL) {
        LOG_ERR("stream: DMA data not initialised (CONFIG_SPI_NXP_LPSPI_DMA required)");
        atomic_set(&stream->active, 0);
        return (-ENODEV);
    }

    /* Initialise stream state */
    stream->cfg            = cfg;
    stream->write_pos      = 0U;
    stream->desc_pool_head = 0U;
    stream->overrun_count  = 0U;
    atomic_set(&stream->frame_idx, 0);

    /* Configure LPSPI hardware for slave RX-only streaming */
    ret = lpspi_configure(dev, cfg->spi_cfg);
    if (ret != 0) {
        LOG_ERR("stream: lpspi_configure failed (%d)", ret);
        goto err_clear;
    }

    /* 3-state MISO — slave does not drive output in RX-only stream mode.
     * TXMSK auto-clear only applies in Controller mode; in Peripheral mode
     * the bit stays set until software clears it (see RM TXMSK description).
     */
    lpspi->TCR |= LPSPI_TCR_TXMSK_MASK;

    /* RXWATER=0: DMA request fires per received word (maximum granularity) */
    lpspi->FCR = LPSPI_FCR_RXWATER(0U);

    /* Configure cyclic DMA channel */
    ret = lpspi_stream_dma_configure(dev);
    if (ret != 0) {
        LOG_ERR("stream: DMA configure failed (%d)", ret);
        goto err_clear;
    }

    /* Start DMA — runs continuously; never stopped between frames */
    ret = dma_start(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
    if (ret != 0) {
        LOG_ERR("stream: dma_start failed (%d)", ret);
        goto err_clear;
    }

    /* Enable LPSPI module */
    lpspi->CR |= LPSPI_CR_MEN_MASK;

    /* Enable RX DMA request and Frame Complete interrupt (order matters) */
    lpspi->DER |= LPSPI_DER_RDDE_MASK;
    lpspi->SR   = LPSPI_SR_FCF_MASK;        /* Clear any stale FCF before enabling IRQ */
    lpspi->IER |= LPSPI_IER_FCIE_MASK;

    LOG_INF("stream: started on %s — ring=%u B, frame=%u B, pool=%u entries",
            dev->name, (unsigned)cfg->ring_buf_size,
            (unsigned)cfg->frame_size, (unsigned)cfg->frame_pool_count);
    return (0);

err_clear :
    stream->cfg = NULL;
    atomic_set(&stream->active, 0);

    return (ret);
}

int spi_stream_stop(const struct device* dev) {
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream_data;
    struct spi_nxp_dma_data* dma_data;
    int ret;

    if (stream == NULL) {
        return (-ENODEV);
    }

    if (atomic_cas(&stream->active, 1, 0) == false) {
        return (-EALREADY);
    }

    /* Disable interrupt and DMA request first to avoid stray callbacks */
    lpspi->IER &= ~LPSPI_IER_FCIE_MASK;
    lpspi->DER &= ~LPSPI_DER_RDDE_MASK;

    /* Clear TXMSK — it does not auto-clear in Peripheral mode */
    lpspi->TCR &= ~LPSPI_TCR_TXMSK_MASK;

    dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    if (dma_data != NULL) {
        ret = dma_stop(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
        if (ret != 0) {
            LOG_WRN("stream: dma_stop returned %d", ret);
        }
    }

    stream->cfg = NULL;

    LOG_INF("stream: stopped on %s (overruns=%u)",
            dev->name, stream->overrun_count);
    return (0);
}

uint32_t spi_stream_overrun_count(const struct device* dev) {
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream_data;

    if (stream == NULL) {
        return (0U);
    }

    return (stream->overrun_count);
}

/* END OF FILE */

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

#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/sys/slist.h>
#include "spi_nxp_lpspi_priv.h"

/*
 * S32K358_DMA_TCD.h (included transitively via S32K358.h) defines:
 *   IP_TCD_BASE = 0x40210000  — base of the eDMA TCD register region
 * Each TCD channel occupies 0x4000 bytes; TCD_DADDR resides at offset +0x30.
 * These constants are used once, at stream-start, to cache the DADDR register
 * pointer used by the spurious-FCF guard in lpspi_stream_isr_fcf_handler().
 * @see 15.6.2.13 TCD Destination Address (TCD0_DADDR - TCD31_DADDR)
 */
#define LPSPI_STREAM_TCD_STRIDE     (0x4000U)  /* bytes between adjacent TCD channels  */
#define LPSPI_STREAM_TCD_DADDR_OFF  (0x030U)   /* offset of DADDR within one TCD channel */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */
static inline LPSPI_Type* lpspi_base(const struct device* dev) {
    return (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
}

static int lpspi_stream_validate_args(struct spi_nxp_stream_data* stream,
                                      struct spi_dt_spec const* spec,
                                      const struct spi_stream_config* cfg) {
    if (stream == NULL) {
        LOG_ERR("stream: CONFIG_SPI_NXP_LPSPI_STREAM not enabled or stream_data NULL");
        return (-ENODEV);
    }

    if ((cfg == NULL) ||
        (cfg->ring_buf == (uintptr_t)NULL)   || (cfg->frame_pool  == NULL) ||
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

    if ((spec->config.operation & SPI_OP_MODE_SLAVE) == 0U) {
        LOG_ERR("stream: slave mode (SPI_OP_MODE_SLAVE) required");
        return (-EINVAL);
    }

    return (0);
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
static int lpspi_stream_dma_configure(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    const struct spi_stream_config* cfg = stream->cfg;
    struct spi_nxp_dma_data* dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    int rc;

    /* ---- Block config ---- */
    struct spi_dma_stream* dma_rx = &dma_data->dma_rx;
    struct dma_block_config* blk_cfg = &dma_rx->dma_blk_cfg;

    memset(blk_cfg, 0, sizeof(struct dma_block_config));
    blk_cfg->source_address   = (uint32_t)(uintptr_t)&lpspi->RDR;
    blk_cfg->dest_address     = (uint32_t)cfg->ring_buf;
    blk_cfg->block_size       = cfg->ring_buf_size;
    blk_cfg->dest_reload_en   = 1U;
    blk_cfg->source_addr_adj  = DMA_ADDR_ADJ_NO_CHANGE;
    blk_cfg->dest_addr_adj    = DMA_ADDR_ADJ_INCREMENT;
    blk_cfg->dest_reload_en   = 1U;
    blk_cfg->source_reload_en = 0U;

    /* ---- Channel config: streaming-specific overrides ---- */
    struct dma_config* dma_cfg = &dma_rx->dma_cfg;
    dma_cfg->channel_direction    = PERIPHERAL_TO_MEMORY;
    dma_cfg->cyclic               = 1U;
    dma_cfg->complete_callback_en = 0U;
    dma_cfg->head_block           = blk_cfg;

    /* Must be non-NULL: nxp_edma_callback calls dma_callback unconditionally. */
    dma_cfg->dma_callback = lpspi_stream_dma_callback;
    dma_cfg->user_data    = (void*)dev;

    rc = dma_config(dma_rx->dma_dev, dma_rx->channel, dma_cfg);
    if (rc != 0) {
        return (rc);
    }

    /*
     * Cache the eDMA TCD_DADDR register address for ISR-context spurious-FCF detection.
     *
     * S32K358 eDMA TCD layout (from S32K358_DMA_TCD.h, IP_TCD_BASE = 0x40210000):
     *   channel N:  base_addr = IP_TCD_BASE + N * LPSPI_STREAM_TCD_STRIDE
     *   TCD_DADDR:  base_addr + LPSPI_STREAM_TCD_DADDR_OFF
     *
     * This assumes channel < 12 (no channel-gap adjustment required on S32K358).
     * Channels used for LPSPI streaming (ch4-7) satisfy this constraint.
     * Reading DADDR in the ISR is safe — MMIO register, no locking required.
     */
    stream->dma_daddr_reg = (volatile uint32_t *)(uintptr_t)(
        IP_TCD_BASE +
        (dma_data->dma_rx.channel * LPSPI_STREAM_TCD_STRIDE) +
        LPSPI_STREAM_TCD_DADDR_OFF);

    return (0);
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
    struct spi_nxp_stream_data* stream = data->stream;
    const struct spi_stream_config* cfg = stream->cfg;
    uint32_t frame_start;
    uint32_t dma_pos;
    uint32_t pool_idx;
    struct spi_stream_frame* desc;

    /*
     * Spurious-FCF guard: the master deasserted CS without clocking any data.
     *
     * The eDMA TCD_DADDR register reflects the live destination address
     * i.e. where the DMA will write the NEXT byte.  Normalised to a ring-buffer offset
     * it equals the byte position up to which data has been written so far.
     * If it matches write_pos, no new data arrived and we must not advance
     * write_pos or publish a descriptor.
     *
     * dma_daddr_reg is an MMIO register pointer cached at stream-start time
     * (lpspi_stream_dma_configure).  Reading it here is ISR-safe.
     *
     * Note: on the very first FCF after stream start, DADDR == ring_buf base
     * and write_pos == 0, so (dma_pos == write_pos) correctly suppresses the
     * spurious event without any special first-call logic.
     */
    dma_pos = (*stream->dma_daddr_reg - (uint32_t)cfg->ring_buf) %
              (uint32_t)cfg->ring_buf_size;

    if (dma_pos != stream->write_pos) {
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
        * corrupt the fifo linked list. Drop the frame, record the overrun, and
        * let the consumer detect the gap via frame_idx discontinuity.
        */
        if (sys_slist_peek_next(&desc->node) != NULL) {
            stream->overrun_count++;
        }
        else {
            /* Step 4: populate descriptor */
            desc->data = (cfg->ring_buf + frame_start);
            desc->len  = cfg->frame_size;
            desc->frame_idx += 1U;

            /* Step 5: notify consumer thread (ISR-safe, non-blocking) */
            k_fifo_put(cfg->frame_fifo, desc);
        }
    }
    else {
        stream->spurious_count++;
    }
}

/* -------------------------------------------------------------------------
 * Public API implementation
 * ------------------------------------------------------------------------- */
int spi_read_stream_async_dt(struct spi_dt_spec const* spec, const struct spi_stream_config* cfg) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    struct spi_nxp_dma_data* dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    int ret;

    ret = lpspi_stream_validate_args(stream, spec, cfg);
    if (ret != 0) {
        return (ret);
    }

    /* Initialise stream state */
    stream->cfg            = cfg;
    stream->write_pos      = 0U;
    stream->desc_pool_head = 0U;
    stream->overrun_count  = 0U;
    stream->frame_idx      = 0U;
    stream->spurious_count = 0U;
    /* stream->dma_daddr_reg is set by lpspi_stream_dma_configure() below */

    /* Configure LPSPI hardware for slave RX-only streaming */
    ret = lpspi_configure(dev, &spec->config);
    if (ret == 0) {
        /* 3-state MISO — slave does not drive output in RX-only stream mode.
        * TXMSK auto-clear only applies in Controller mode; in Peripheral mode
        * the bit stays set until software clears it (see RM TXMSK description).
        */
        lpspi->TCR |= LPSPI_TCR_TXMSK_MASK;

        /* RXWATER=0: DMA request fires per received word (maximum granularity) */
        lpspi->FCR = LPSPI_FCR_RXWATER(0U);

        /* Configure cyclic DMA channel */
        ret = lpspi_stream_dma_configure(spec);
        if (ret == 0) {
            /* Start DMA — runs continuously; never stopped between frames */
            ret = dma_start(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
            if (ret == 0) {
                /* Enable LPSPI module */
                lpspi->CR |= LPSPI_CR_MEN_MASK;

                /* Enable RX DMA request and Frame Complete interrupt (order matters) */
                lpspi->DER |= LPSPI_DER_RDDE_MASK;
                lpspi->SR   = LPSPI_SR_FCF_MASK;        /* Clear any stale FCF before enabling IRQ */
                lpspi->IER |= LPSPI_IER_FCIE_MASK;

                LOG_INF("stream: started on %s — ring=%u B, frame=%u B, pool=%u entries",
                        dev->name, (unsigned)cfg->ring_buf_size,
                        (unsigned)cfg->frame_size, (unsigned)cfg->frame_pool_count);
            }
        }
    }

    if (ret != 0) {
        LOG_ERR("stream: init failed (%d)", ret);
    }

    return (ret);
}

int spi_stream_stop_dt(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi = lpspi_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    struct spi_nxp_dma_data* dma_data;
    int ret;

    if (stream == NULL) {
        return (-ENODEV);
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

uint32_t spi_stream_overrun_count_dt(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;

    if (stream == NULL) {
        return (0U);
    }

    return (stream->overrun_count);
}

/* END OF FILE */

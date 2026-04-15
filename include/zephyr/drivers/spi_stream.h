/*
 * Copyright 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Continuous SPI peripheral RX streaming API using eDMA circular buffer.
 *
 * Architecture overview:
 *
 *   SPI controller ─[MOSI]─► LPSPI (peripheral) ─ RX FIFO ─ eDMA (cyclic P→M)
 *                                                          │
 *                                              Ring buffer (static, N×frame_sz)
 *                                                          │
 *                                         FCF interrupt (CS deasserts = frame end)
 *                                                          │
 *                                              k_fifo ← spi_stream_frame (no-alloc)
 *                                                          │
 *                                              Zephyr thread processes frame
 *
 * Constraints:
 *   - Peripheral mode only.
 *   - Fixed frame size per stream session.
 *   - All buffers (ring_buf, frame_pool) must be statically allocated by caller.
 *   - DMA runs continuously — never stopped between frames.
 *   - ISR does NOT copy data; it only records frame boundaries.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Frame descriptor posted by ISR to the thread notification k_fifo.
 *
 * The node field must remain first — it is used as the k_fifo link.
 * Descriptors are reused round-robin from the pool; never freed.
 * After processing a frame obtained via k_fifo_get(), the consumer MUST call
 * spi_stream_frame_release() to allow the ISR to reuse this slot.
 */
struct spi_stream_frame {
    sys_snode_t node;           /**< k_fifo linkage — MUST be first field */
    uintptr_t data;             /**< Pointer into ring buffer (zero-copy) */
    size_t   len;               /**< Frame length in bytes */
    atomic_t in_fifo;           /**< 1 while queued/processing; 0 after spi_stream_frame_release() */
};

/**
 * @brief Release a frame descriptor back to the ISR pool.
 *
 * Must be called after the consumer has finished accessing frame->data.
 * Clears in_fifo so the ISR can reuse this descriptor slot.
 * Failure to call this causes permanent overrun on this slot.
 *
 *   struct spi_stream_frame *f = k_fifo_get(&fifo, K_FOREVER);
 *   process((uint8_t *)f->data, f->len);
 *   spi_stream_frame_release(f);   // <-- must call before next use
 */
static inline void spi_stream_frame_release(struct spi_stream_frame* frame) {
    atomic_set(&frame->in_fifo, 0);
}

/**
 * @brief Configuration for a continuous SPI peripheral RX stream.
 *
 * All pointed-to memory must remain valid for the lifetime of the stream.
 * Typically these are static or global objects.
 */
struct spi_stream_config {
    /**
     * Statically allocated ring buffer.
     * Size must be a multiple of frame_size and at least 2×frame_size.
     * Alignment: 4-byte minimum (recommend cache-line for performance).
     */
    uintptr_t ring_buf;
    size_t ring_buf_size;

    /**
     * Expected bytes per SPI frame.
     * Must divide ring_buf_size evenly.
     * If a received frame differs from this length, it is still delivered
     * but with its actual length recorded in spi_stream_frame.len.
     */
    size_t frame_size;

    /**
     * Statically allocated pool of frame descriptors.
     * Pool depth must be >= ring_buf_size / frame_size.
     * Descriptors are reused round-robin; overflow increments a counter
     * in the driver (detectable via spi_stream_overrun_count()).
     */
    struct spi_stream_frame* frame_pool;
    size_t frame_pool_count;

    /**
     * k_fifo for ISR-to-thread frame notification.
     * Must be initialised by caller (K_FIFO_DEFINE or k_fifo_init).
     * ISR calls k_fifo_put() with pool descriptors — no heap allocation.
     */
    struct k_fifo* frame_fifo;
};

/**
 * @brief Start continuous DMA-based SPI peripheral RX streaming.
 *
 * Configures LPSPI in peripheral mode with TXMSK (RX-only), sets up
 * circular eDMA (cyclic=1) into ring_buf, enables the Frame Complete
 * interrupt (FCF/FCIE), and starts DMA.  Returns immediately;
 * the caller thread reads frames via cfg->frame_fifo.
 *
 * @param dev  SPI device node (must have DMA channels in DTS).
 * @param cfg  Streaming configuration.  All fields must be initialised.
 * @return 0 on success, -EBUSY if already streaming, -ENODEV if DMA not
 *         configured, or negative errno on other error.
 */
int spi_read_stream_async_dt(struct spi_dt_spec const* spec,
                             struct spi_stream_config const* cfg);

/**
 * @brief Stop continuous SPI peripheral RX streaming.
 *
 * Disables RDDE and FCIE, stops the DMA channel, and clears internal
 * state.  Any pending descriptors in frame_fifo remain valid until
 * drained by the application.
 *
 * @param spec SPI device specification.
 * @return 0 on success, -EALREADY if not streaming.
 */
int spi_stream_stop_dt(struct spi_dt_spec const* spec);

/**
 * @brief Return the number of frames dropped due to descriptor pool exhaustion.
 *
 * Resets to zero on each call to spi_read_stream_async_dt().
 *
 * @param spec SPI device specification.
 * @return Overrun count since last stream start.
 */
uint32_t spi_stream_overrun_count_dt(struct spi_dt_spec const* spec);

#ifdef __cplusplus
}
#endif

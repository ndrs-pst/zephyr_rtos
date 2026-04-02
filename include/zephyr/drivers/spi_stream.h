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
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Frame descriptor posted by ISR to the thread notification k_fifo.
 *
 * The node field must remain first — it is used as the k_fifo link.
 * The caller must NOT free these descriptors; they are reused from the pool.
 */
struct spi_stream_frame {
    sys_snode_t node;           /**< k_fifo linkage — MUST be first field */
    uintptr_t data;             /**< Pointer into ring buffer (zero-copy) */
    size_t   len;               /**< Frame length in bytes */
    uint32_t frame_idx;         /**< Monotonic frame counter (wraps at UINT32_MAX) */
};

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

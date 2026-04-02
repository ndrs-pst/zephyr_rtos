/*
 * Copyright 2018, 2024-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_NXP_LPSPI_PRIV_H_
#define ZEPHYR_DRIVERS_SPI_SPI_NXP_LPSPI_PRIV_H_

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>

#ifdef CONFIG_SPI_NXP_LPSPI_DMA
#include <zephyr/drivers/dma.h>
#endif

#ifdef CONFIG_SPI_NXP_LPSPI_STREAM
#include <zephyr/drivers/spi_stream.h>
#include <zephyr/sys/atomic.h>
#endif

#include "../spi_context.h"

#if CONFIG_NXP_LP_FLEXCOMM
#include <zephyr/drivers/mfd/nxp_lp_flexcomm.h>
#endif

/* If any hardware revisions change this, make it into a DT property.
 * DON'T make #ifdefs here by platform.
 */
#define LPSPI_CHIP_SELECT_COUNT   4
#define LPSPI_MIN_FRAME_SIZE_BITS 8

#define LPSPI_INTERRUPT_BITS GENMASK(13, 8)

/* Required by DEVICE_MMIO_NAMED_* macros */
#define DEV_CFG(_dev)  ((struct lpspi_config DT_CONST*)(_dev)->config)
#define DEV_DATA(_dev) ((struct lpspi_data*)(_dev)->data)

struct lpspi_config {
    DEVICE_MMIO_NAMED_ROM(reg_base);
    const struct device* clock_dev;
    clock_control_subsys_t clock_subsys;
    void (*irq_config_func)(const struct device* dev);
    uint32_t pcs_sck_delay;
    uint32_t sck_pcs_delay;
    uint32_t transfer_delay;
    const struct pinctrl_dev_config* pincfg;
    uint8_t data_pin_config;
    bool tristate_output;
    uint8_t tx_fifo_size;
    uint8_t rx_fifo_size;
    uint8_t irqn;
};

struct lpspi_data {
    DEVICE_MMIO_NAMED_RAM(reg_base);
    struct spi_context ctx;
    void* driver_data;
    size_t transfer_len;
    uint32_t clock_freq;
    uint8_t major_version;
#ifdef CONFIG_SPI_NXP_LPSPI_STREAM
    /** Streaming state — NULL when streaming is not configured. */
    struct spi_nxp_stream_data *stream_data;
#endif
};

#ifdef CONFIG_SPI_NXP_LPSPI_DMA
/* Transfer-state machine used by the DMA callback. */
typedef enum {
    LPSPI_TRANSFER_STATE_NULL,
    LPSPI_TRANSFER_STATE_ONGOING,
    LPSPI_TRANSFER_STATE_NEXT_DMA_SIZE_UPDATED,
    LPSPI_TRANSFER_STATE_TX_DONE,
    LPSPI_TRANSFER_STATE_RX_DONE,
    LPSPI_TRANSFER_STATE_RX_TX_DONE,
    LPSPI_TRANSFER_STATE_INVALID = 0xFFFFFFFFUL,
} lpspi_transfer_state_t;

struct spi_dma_stream {
    const struct device *dma_dev;
    uint32_t channel;
    struct dma_config dma_cfg;
    struct dma_block_config dma_blk_cfg;
};

struct spi_nxp_dma_data {
    struct spi_dma_stream dma_rx;
    struct spi_dma_stream dma_tx;
    lpspi_transfer_state_t state;
    /* Synchronized DMA size for RX/TX interleaving on v1 LPSPI. */
    size_t synchronize_dma_size;
};
#endif /* CONFIG_SPI_NXP_LPSPI_DMA */

#ifdef CONFIG_SPI_NXP_LPSPI_STREAM
/**
 * Streaming state for CONFIG_SPI_NXP_LPSPI_STREAM.
 *
 * Allocated statically per device instance when CONFIG_SPI_NXP_LPSPI_STREAM=y.
 * Hung off lpspi_data.stream_data (pointer, NULL when stream not configured).
 *
 * Lifetime: zero-initialised at compile time; populated by spi_read_stream_async()
 * and cleared by spi_stream_stop().
 */
struct spi_nxp_stream_data {
    /* ------------------------------------------------------------------ */
    /* Streaming configuration (set at stream_start, NULLed at stop)      */
    /* ------------------------------------------------------------------ */
    const struct spi_stream_cfg *cfg;

    /* ------------------------------------------------------------------ */
    /* RX-only DMA channel                                                 */
    /* ------------------------------------------------------------------ */
    struct dma_block_config dma_blk;    /**< eDMA block config (cyclic, single block) */
    struct dma_config dma_cfg;          /**< eDMA channel config */

    /* ------------------------------------------------------------------ */
    /* Ring buffer write pointer (software-tracked per FCF event)          */
    /* Advanced by frame_size bytes on every FCF interrupt.               */
    /* ------------------------------------------------------------------ */
    uint32_t write_pos; /**< Byte offset of next frame start in ring_buf */

    /* ------------------------------------------------------------------ */
    /* Frame descriptor pool state                                         */
    /* ------------------------------------------------------------------ */
    uint32_t desc_pool_head; /**< Next pool slot to use (mod frame_pool_count) */
    uint32_t overrun_count;  /**< Descriptors dropped due to pool exhaustion */

    /* ------------------------------------------------------------------ */
    /* Monotonic frame counter                                             */
    /* ------------------------------------------------------------------ */
    atomic_t frame_idx;

    /* ------------------------------------------------------------------ */
    /* Lifecycle guard: 1 = streaming active, 0 = idle                    */
    /* ------------------------------------------------------------------ */
    atomic_t active;
};
#endif /* CONFIG_SPI_NXP_LPSPI_STREAM */

/* Common helper functions used to interact with LPSPI FIFOs (TX and RX) when dealing
 * with cpu-based implementation.
 */
static inline size_t rx_fifo_cur_len(LPSPI_Type* base) {
    return (size_t)((base->FSR & LPSPI_FSR_RXCOUNT_MASK) >> LPSPI_FSR_RXCOUNT_SHIFT);
}

static inline size_t tx_fifo_cur_len(LPSPI_Type* base) {
    return (size_t)((base->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT);
}

/* Verifies spi_cfg validity and set up configuration of hardware for xfer
 * Unsets interrupt and watermark options, specific implementation should configure that.
 * Sets bits in the TCR ONLY *directly* relating to what is in the spi_config struct.
 */
int lpspi_configure(const struct device* dev, const struct spi_config* spi_cfg);

/* Does these things:
 * Set data.dev
 * Check clocks device is ready
 * Configure cs gpio pin if needed
 * Mux pinctrl to lpspi
 * Enable LPSPI IRQ at system level
 */
int spi_nxp_init_common(const struct device* dev);

/* common api function for now */
int spi_lpspi_release(const struct device* dev, const struct spi_config* spi_cfg);

int lpspi_wait_tx_fifo_empty(const struct device* dev, spi_operation_t operation);

#define SPI_LPSPI_IRQ_FUNC_LP_FLEXCOMM(n) \
    nxp_lp_flexcomm_setirqhandler(DEVICE_DT_GET(DT_INST_PARENT(n)), DEVICE_DT_INST_GET(n), \
                                  LP_FLEXCOMM_PERIPH_LPSPI, lpspi_isr);

#define SPI_LPSPI_IRQ_FUNC_DISTINCT(n) \
    IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), lpspi_isr, DEVICE_DT_INST_GET(n), \
                0); \
    irq_enable(DT_INST_IRQN(n));

#define SPI_LPSPI_IRQ_FUNC(n)                                   \
    COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PARENT(n),           \
                                   nxp_lp_flexcomm),            \
                (SPI_LPSPI_IRQ_FUNC_LP_FLEXCOMM(n)),            \
                (SPI_LPSPI_IRQ_FUNC_DISTINCT(n)))

#define LPSPI_IRQN(n) \
    COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_PARENT(n), nxp_lp_flexcomm), \
                (DT_IRQN(DT_INST_PARENT(n))), (DT_INST_IRQN(n)))

#define SPI_LPSPI_CONFIG_INIT(n)                                                \
    static struct lpspi_config DT_CONST lpspi_config_##n = {                    \
        DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                   \
        .clock_dev       = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),               \
        .clock_subsys    = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
        .irq_config_func = lpspi_config_func_##n,                               \
        .pcs_sck_delay   = DT_INST_PROP_OR(n, pcs_sck_delay, 0),                \
        .sck_pcs_delay   = DT_INST_PROP_OR(n, sck_pcs_delay, 0),                \
        .transfer_delay  = DT_INST_PROP_OR(n, transfer_delay, 0),               \
        .pincfg          = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                   \
        .data_pin_config = (uint8_t)DT_INST_ENUM_IDX(n, data_pin_config),       \
        .tristate_output = DT_INST_PROP(n, tristate_output),                    \
        .rx_fifo_size    = (uint8_t)DT_INST_PROP(n, rx_fifo_size),              \
        .tx_fifo_size    = (uint8_t)DT_INST_PROP(n, tx_fifo_size),              \
        .irqn            = (uint8_t)LPSPI_IRQN(n),                              \
    };

#define SPI_NXP_LPSPI_COMMON_INIT(n)                            \
    PINCTRL_DT_INST_DEFINE(n);                                  \
                                                                \
    static void lpspi_config_func_##n(const struct device* dev) { \
        SPI_LPSPI_IRQ_FUNC(n)                                   \
    }

#define SPI_NXP_LPSPI_COMMON_DATA_INIT(n)                       \
    SPI_CONTEXT_INIT_LOCK(lpspi_data_##n, ctx),                 \
        SPI_CONTEXT_INIT_SYNC(lpspi_data_##n, ctx),             \
        SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)

#define SPI_NXP_LPSPI_HAS_DMAS(n) \
    UTIL_AND(DT_INST_DMAS_HAS_NAME(n, tx), DT_INST_DMAS_HAS_NAME(n, rx))

/* -----------------------------------------------------------------------
 * Streaming API internals (spi_nxp_lpspi_stream.c)
 * ----------------------------------------------------------------------- */
#ifdef CONFIG_SPI_NXP_LPSPI_STREAM

/**
 * Called from lpspi_isr() in spi_nxp_lpspi_dma.c when FCF (Frame Complete
 * Flag) is asserted.  Runs at interrupt priority — must not block.
 */
void lpspi_stream_isr_fcf_handler(const struct device *dev);

/**
 * Per-instance static storage declaration — used inside LPSPI_DMA_INIT(n)
 * in spi_nxp_lpspi_dma.c before the lpspi_data_##n struct initialiser.
 */
#define SPI_NXP_LPSPI_STREAM_DATA_DECL(n) \
    static struct spi_nxp_stream_data lpspi_stream_data_##n;

/**
 * Per-instance stream_data pointer initialiser — used inside the
 * lpspi_data_##n compound literal in LPSPI_DMA_INIT(n).
 */
#define SPI_NXP_LPSPI_STREAM_DATA_INIT(n) \
    .stream_data = &lpspi_stream_data_##n,

#endif /* CONFIG_SPI_NXP_LPSPI_STREAM */

#endif /* ZEPHYR_DRIVERS_SPI_SPI_NXP_LPSPI_PRIV_H_ */

/*
 * Copyright (c) 2017 Google LLC.
 * Copyright (c) 2024 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT atmel_sam0_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_sam0);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#ifndef SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val
#define SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val (0x3)
#endif

/* #CUSTOM@NDRS : Use GCLK1 as clock source instead of GCLK0 */

/* Device constant configuration parameters */
struct spi_sam0_config {
    SercomSpi* regs;
    uint32_t pads;
    const struct pinctrl_dev_config* pcfg;

    volatile uint32_t* mclk;
    uint32_t mclk_mask;
    uint32_t gclk_gen;
    uint16_t gclk_id;

    #ifdef CONFIG_SPI_ASYNC
    const struct device* dma_dev;
    uint8_t tx_dma_request;
    uint8_t tx_dma_channel;
    uint8_t rx_dma_request;
    uint8_t rx_dma_channel;
    #endif
};

/* Device run time data */
struct spi_sam0_data {
    struct spi_context ctx;
    #ifdef CONFIG_SPI_ASYNC
    const struct device* dev;
    uint32_t dma_segment_len;
    #endif
};

static void wait_synchronization(SercomSpi* regs) {
    #if defined(SERCOM_SPI_SYNCBUSY_MASK)
    /* SYNCBUSY is a register */
    while ((regs->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_MASK) != 0U) {
        /* pass */
    }
    #elif defined(SERCOM_SPI_STATUS_SYNCBUSY)
    /* SYNCBUSY is a bit */
    while ((regs->STATUS.reg & SERCOM_SPI_STATUS_SYNCBUSY) != 0U) {
        /* pass */
    }
    #else
    #error Unsupported device
    #endif
}

static int spi_sam0_configure(const struct device* dev,
                              const struct spi_config* config) {
    const struct spi_sam0_config* cfg;
    struct spi_sam0_data* data;
    SercomSpi* regs;
    SERCOM_SPI_CTRLA_Type ctrla;
    SERCOM_SPI_CTRLB_Type ctrlb;
    #ifdef SERCOM_SPI_CTRLC_MASK
    SERCOM_SPI_CTRLC_Type ctrlc = {.reg = 0};
    SERCOM_SPI_LENGTH_Type length = {.reg = 0};
    #endif
    int div;
    int ret;
    bool rc;

    data = dev->data;
    rc = spi_context_configured(&data->ctx, config);
    if (rc == false) {
        ret = -ENOTSUP;

        if ((config->operation & SPI_HALF_DUPLEX) == 0U) {
            if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
                if (SPI_WORD_SIZE_GET(config->operation) == 8) {
                    ret = 0;
                }
                else {
                    /* Support only 8-bit word size */
                }
            }
            else {
                /* Slave mode is not implemented. */
            }
        }
        else {
            LOG_ERR("Half-duplex not supported");
        }

        if (ret == 0) {
            /* CTRLA: SPI host operation, pads, Enable beforehand */
            cfg  = dev->config;
            ctrla.reg = (SERCOM_SPI_CTRLA_MODE(SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val) | cfg->pads | SERCOM_SPI_CTRLA_ENABLE);
            regs = cfg->regs;

            if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
                ctrla.bit.DORD = 1;
            }

            if ((config->operation & SPI_MODE_CPOL) != 0U) {
                ctrla.bit.CPOL = 1;
            }

            if ((config->operation & SPI_MODE_CPHA) != 0U) {
                ctrla.bit.CPHA = 1;
            }

            if ((config->operation & SPI_MODE_LOOP) != 0U) {
                /* Put MISO and MOSI on the same pad */
                ctrla.bit.DOPO = 0;
                ctrla.bit.DIPO = 0;
            }

            /* CTRLB: RXEN, 8 bits per transfer */
            ctrlb.reg = (SERCOM_SPI_CTRLB_RXEN | SERCOM_USART_CTRLB_CHSIZE(0));

            /* Use the requested or next highest possible frequency */
            div = ((SOC_ATMEL_SAM0_GCLK1_FREQ_HZ / config->frequency) / 2U) - 1U;
            div = CLAMP(div, 0, UINT8_MAX);

            #ifdef SERCOM_SPI_CTRLC_MASK
            /* LENGTH.LEN must only be enabled when CTRLC.bit.DATA32B is enabled.
             * Since we are about to explicitly disable it, we need to clear the LENGTH register.
             */
            length.reg = SERCOM_SPI_LENGTH_RESETVALUE;

            /* Disable inter-character spacing and the 32-bit read/write extension */
            ctrlc.reg = SERCOM_SPI_CTRLC_RESETVALUE;
            #endif

            /* Update the configuration only if it has changed */
            if ((regs->CTRLA.reg != ctrla.reg) ||
                (regs->CTRLB.reg != ctrlb.reg) ||
                (regs->BAUD.reg  != div)
                #ifdef SERCOM_SPI_CTRLC_MASK
                || (regs->LENGTH.reg != length.reg) || (regs->CTRLC.reg != ctrlc.reg)
                #endif
            ) {
                regs->CTRLA.bit.ENABLE = 0;
                wait_synchronization(regs);
                regs->CTRLB.reg = ctrlb.reg;
                wait_synchronization(regs);
                regs->BAUD.reg = div;
                wait_synchronization(regs);
                regs->CTRLA.reg = ctrla.reg;
                wait_synchronization(regs);

                #ifdef SERCOM_SPI_CTRLC_MASK
                regs->LENGTH = length;
                wait_synchronization(regs);

                /* Although CTRLC is not write-synchronized, it is enabled-protected */
                regs->CTRLC = ctrlc;
                #endif
            }

            data->ctx.config = config;
        }
    }
    else {
        ret = 0;
    }

    return (ret);
}

static bool spi_sam0_transfer_ongoing(struct spi_sam0_data* data) {
    return (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx));
}

static void spi_sam0_shift_master(SercomSpi* regs, struct spi_sam0_data* data) {
    uint8_t tx;
    uint8_t rx;
    bool rc;

    rc = spi_context_tx_buf_on(&data->ctx);
    if (rc == true) {
        tx = *(uint8_t*)(data->ctx.tx_buf);
    }
    else {
        tx = 0U;
    }

    while (regs->INTFLAG.bit.DRE == 0U) {
        /* pass */
    }

    regs->DATA.reg = tx;
    spi_context_update_tx(&data->ctx, 1, 1);

    while (regs->INTFLAG.bit.RXC == 0U) {
        /* pass */
    }

    rx = regs->DATA.reg;

    rc = spi_context_rx_buf_on(&data->ctx);
    if (rc == true) {
        *data->ctx.rx_buf = rx;
    }
    spi_context_update_rx(&data->ctx, 1, 1);
}

/* Finish any ongoing writes and drop any remaining read data */
static void spi_sam0_finish(SercomSpi* regs) {
    while (regs->INTFLAG.bit.TXC == 0U) {
        /* pass */
    }

    while (regs->INTFLAG.bit.RXC == 1U) {
        (void)regs->DATA.reg;
    }
}

/* Fast path that transmits a buf */
static void spi_sam0_fast_tx(SercomSpi* regs, const struct spi_buf* tx_buf) {
    const uint8_t* p;
    const uint8_t* pend;
    uint8_t ch;

    p    = tx_buf->buf;
    pend = (uint8_t*)tx_buf->buf + tx_buf->len;
    while (p != pend) {
        ch = *p++;

        while (regs->INTFLAG.bit.DRE == 0U) {
            /* pass */
        }

        regs->DATA.reg = ch;
    }

    spi_sam0_finish(regs);
}

/* Fast path that reads into a buf */
static void spi_sam0_fast_rx(SercomSpi* regs, const struct spi_buf* rx_buf) {
    uint8_t* rx;
    int len;

    rx  = rx_buf->buf;
    len = rx_buf->len;
    if (len > 0) {
        while (len > 0) {
            /* Send the next byte */
            regs->DATA.reg = 0U;
            len--;

            /* Wait for completion, and read */
            while (regs->INTFLAG.bit.RXC == 0U) {
                /* pass */
            }

            *rx++ = regs->DATA.reg;
        }

        spi_sam0_finish(regs);
    }
}

/* Fast path that writes and reads bufs of the same length */
static void spi_sam0_fast_txrx(SercomSpi* regs,
                               const struct spi_buf* tx_buf,
                               const struct spi_buf* rx_buf) {
    const uint8_t* tx;
    const uint8_t* txend;
    uint8_t* rx;
    size_t len;

    rx  = rx_buf->buf;
    len = rx_buf->len;
    if (len > 0) {
        tx    = tx_buf->buf;
        txend = (uint8_t*)tx_buf->buf + tx_buf->len;

        while (tx != txend) {
            /* Send the next byte */
            regs->DATA.reg = *tx++;

            /* Wait for completion, and read */
            while (regs->INTFLAG.bit.RXC == 0U) {
                /* pass */
            }
            *rx++ = (uint8_t)regs->DATA.reg;
        }

        spi_sam0_finish(regs);
    }
}

/* Fast path where every overlapping tx and rx buffer is the same length */
static void spi_sam0_fast_transceive(const struct device* dev,
                                     const struct spi_config* config,
                                     const struct spi_buf_set* tx_bufs,
                                     const struct spi_buf_set* rx_bufs) {
    const struct spi_sam0_config* cfg;
    size_t tx_count;
    size_t rx_count;
    SercomSpi* regs;
    const struct spi_buf* tx;
    const struct spi_buf* rx;

    if (tx_bufs != NULL) {
        tx = tx_bufs->buffers;
        tx_count = tx_bufs->count;
    }
    else {
        tx = NULL;
        tx_count = 0U;
    }

    if (rx_bufs != NULL) {
        rx = rx_bufs->buffers;
        rx_count = rx_bufs->count;
    }
    else {
        rx = NULL;
        rx_count = 0U;
    }

    cfg  = dev->config;
    regs = cfg->regs;
    while ((tx_count > 0U) && (rx_count > 0U)) {
        if (tx->buf == NULL) {
            spi_sam0_fast_rx(regs, rx);
        }
        else if (rx->buf == NULL) {
            spi_sam0_fast_tx(regs, tx);
        }
        else {
            spi_sam0_fast_txrx(regs, tx, rx);
        }

        ++tx;
        --tx_count;
        ++rx;
        --rx_count;
    }

    for (; tx_count > 0U; tx_count--) {
        spi_sam0_fast_tx(regs, tx++);
    }

    for (; rx_count > 0U; rx_count--) {
        spi_sam0_fast_rx(regs, rx++);
    }
}

/* Returns true if the request is suitable for the fast
 * path. Specifically, the bufs are a sequence of:
 *
 * - Zero or more RX and TX buf pairs where each is the same length.
 * - Zero or more trailing RX only bufs
 * - Zero or more trailing TX only bufs
 */
static bool spi_sam0_is_regular(const struct spi_buf_set* tx_bufs,
                                const struct spi_buf_set* rx_bufs) {
    const struct spi_buf* tx;
    const struct spi_buf* rx;
    size_t tx_count;
    size_t rx_count;
    bool rc;

    if (tx_bufs != NULL) {
        tx = tx_bufs->buffers;
        tx_count = tx_bufs->count;
    }
    else {
        tx = NULL;
        tx_count = 0U;
    }

    if (rx_bufs != NULL) {
        rx = rx_bufs->buffers;
        rx_count = rx_bufs->count;
    }
    else {
        rx = NULL;
        rx_count = 0U;
    }

    rc = true;
    while ((tx_count > 0U) && (rx_count > 0U)) {
        if (tx->len != rx->len) {
            rc = false;
            break;
        }

        tx++;
        tx_count--;
        rx++;
        rx_count--;
    }

    return (rc);
}

static int spi_sam0_transceive(const struct device* dev,
                               const struct spi_config* config,
                               const struct spi_buf_set* tx_bufs,
                               const struct spi_buf_set* rx_bufs) {
    const struct spi_sam0_config* cfg;
    struct spi_sam0_data* data;
    SercomSpi* regs;
    bool rc;
    int  ret;

    cfg  = dev->config;
    data = dev->data;
    regs = cfg->regs;

    spi_context_lock(&data->ctx, false, NULL, NULL, config);

    ret = spi_sam0_configure(dev, config);
    if (ret == 0) {
        spi_context_cs_control(&data->ctx, true);

        /* This driver special cases the common send only, receive
         * only, and transmit then receive operations.  This special
         * casing is 4x faster than the spi_context() routines
         * and allows the transmit and receive to be interleaved.
         */
        rc = spi_sam0_is_regular(tx_bufs, rx_bufs);
        if (rc == true) {
            spi_sam0_fast_transceive(dev, config, tx_bufs, rx_bufs);
        }
        else {
            spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

            do {
                spi_sam0_shift_master(regs, data);
            } while (spi_sam0_transfer_ongoing(data));
        }

        spi_context_cs_control(&data->ctx, false);
    }

    spi_context_release(&data->ctx, ret);

    return (ret);
}

static int spi_sam0_transceive_sync(const struct device* dev,
                                    const struct spi_config* config,
                                    const struct spi_buf_set* tx_bufs,
                                    const struct spi_buf_set* rx_bufs) {
    return spi_sam0_transceive(dev, config, tx_bufs, rx_bufs);
}

#ifdef CONFIG_SPI_ASYNC

static void spi_sam0_dma_rx_done(const struct device* dma_dev, void* arg,
                                 uint32_t id, int error_code);

static int spi_sam0_dma_rx_load(const struct device* dev, uint8_t* buf, size_t len) {
    const struct spi_sam0_config* cfg;
    struct spi_sam0_data* data;
    SercomSpi* regs;
    struct dma_config dma_cfg = {0};
    struct dma_block_config dma_blk = {0};
    int ret;

    cfg  = dev->config;
    data = dev->data;
    regs = cfg->regs;

    dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    dma_cfg.source_data_size  = 1;
    dma_cfg.dest_data_size    = 1;
    dma_cfg.user_data         = data;
    dma_cfg.dma_callback      = spi_sam0_dma_rx_done;
    dma_cfg.block_count       = 1;
    dma_cfg.head_block        = &dma_blk;
    dma_cfg.dma_slot          = cfg->rx_dma_request;

    dma_blk.block_size = len;

    if (buf != NULL) {
        dma_blk.dest_address = (uint32_t)buf;
    }
    else {
        static uint8_t dummy;

        dma_blk.dest_address  = (uint32_t)&dummy;
        dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }

    dma_blk.source_address  = (uint32_t)(&(regs->DATA.reg));
    dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

    ret = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
    if (ret == 0) {
        ret = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
    }

    return (ret);
}

static int spi_sam0_dma_tx_load(const struct device* dev, const uint8_t* buf, size_t len) {
    const struct spi_sam0_config* cfg;
    SercomSpi* regs;
    struct dma_config dma_cfg = {0};
    struct dma_block_config dma_blk = {0};
    int ret;

    cfg  = dev->config;
    regs = cfg->regs;

    dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    dma_cfg.source_data_size  = 1;
    dma_cfg.dest_data_size    = 1;
    dma_cfg.block_count       = 1;
    dma_cfg.head_block        = &dma_blk;
    dma_cfg.dma_slot          = cfg->tx_dma_request;

    dma_blk.block_size = len;

    if (buf != NULL) {
        dma_blk.source_address = (uint32_t)buf;
    }
    else {
        static const uint8_t dummy;

        dma_blk.source_address  = (uint32_t)&dummy;
        dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    }

    dma_blk.dest_address  = (uint32_t)(&(regs->DATA.reg));
    dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

    ret = dma_config(cfg->dma_dev, cfg->tx_dma_channel, &dma_cfg);
    if (ret == 0) {
        ret = dma_start(cfg->dma_dev, cfg->tx_dma_channel);
    }

    return (ret);
}

static bool spi_sam0_dma_advance_segment(const struct device* dev) {
    struct spi_sam0_data* data;
    uint32_t segment_len;
    bool rc;

    data = dev->data;
    /* Pick the shorter buffer of ones that have an actual length */
    if (data->ctx.rx_len != 0) {
        segment_len = data->ctx.rx_len;
        if (data->ctx.tx_len != 0) {
            segment_len = MIN(segment_len, data->ctx.tx_len);
        }
    }
    else {
        segment_len = data->ctx.tx_len;
    }

    if (segment_len > 0) {
        segment_len = MIN(segment_len, 65535);

        data->dma_segment_len = segment_len;

        rc = true;
    }
    else {
        rc = false;
    }

    return (rc);
}

static int spi_sam0_dma_advance_buffers(const struct device* dev) {
    struct spi_sam0_data* data = dev->data;
    int ret;

    if (data->dma_segment_len > 0U) {
        /* Load receive first, so it can accept transmit data */
        if (data->ctx.rx_len > 0U) {
            ret = spi_sam0_dma_rx_load(dev, data->ctx.rx_buf, data->dma_segment_len);
        }
        else {
            ret = spi_sam0_dma_rx_load(dev, NULL, data->dma_segment_len);
        }

        if (ret == 0) {
            /* Now load the transmit, which starts the actual bus clocking */
            if (data->ctx.tx_len > 0U) {
                ret = spi_sam0_dma_tx_load(dev, data->ctx.tx_buf, data->dma_segment_len);
            }
            else {
                ret = spi_sam0_dma_tx_load(dev, NULL, data->dma_segment_len);
            }
        }
    }
    else {
        ret = -EINVAL;
    }

    return (ret);
}

static void spi_sam0_dma_rx_done(const struct device* dma_dev, void* arg,
                                 uint32_t id, int error_code) {
    struct spi_sam0_data* data;
    const struct device* dev;
    const struct spi_sam0_config* cfg;
    int ret;

    data = arg;
    dev  = data->dev;
    cfg  = dev->config;

    ARG_UNUSED(id);
    ARG_UNUSED(error_code);

    spi_context_update_tx(&data->ctx, 1, data->dma_segment_len);
    spi_context_update_rx(&data->ctx, 1, data->dma_segment_len);

    if (spi_sam0_dma_advance_segment(dev) == false) {
        /* Done */
        spi_context_cs_control(&data->ctx, false);
        spi_context_complete(&data->ctx, dev, 0);
    }
    else {
        ret = spi_sam0_dma_advance_buffers(dev);
        if (ret != 0) {
            dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
            dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
            spi_context_cs_control(&data->ctx, false);
            spi_context_complete(&data->ctx, dev, ret);
        }
    }
}


static int spi_sam0_transceive_async(const struct device* dev,
                                     const struct spi_config* config,
                                     const struct spi_buf_set* tx_bufs,
                                     const struct spi_buf_set* rx_bufs,
                                     spi_callback_t cb,
                                     void* userdata) {
    const struct spi_sam0_config* cfg;
    struct spi_sam0_data* data;
    int ret;

    cfg  = dev->config;
    data = dev->data;

    /*
     * Transmit clocks the output and we use receive to determine when
     * the transmit is done, so we always need both
     */
    if ((cfg->tx_dma_channel != 0xFFU) && (cfg->rx_dma_channel != 0xFF)) {
        spi_context_lock(&data->ctx, true, cb, userdata, config);

        ret = spi_sam0_configure(dev, config);
        if (ret == 0) {
            spi_context_cs_control(&data->ctx, true);

            spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

            spi_sam0_dma_advance_segment(dev);
            ret = spi_sam0_dma_advance_buffers(dev);
            if (ret != 0) {
                dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
                dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
                spi_context_cs_control(&data->ctx, false);
            }
        }

        spi_context_release(&data->ctx, ret);
    }
    else {
        ret = -ENOTSUP;
    }

    return (ret);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_sam0_release(const struct device* dev, const struct spi_config* config) {
    struct spi_sam0_data* data = dev->data;

    spi_context_unlock_unconditionally(&data->ctx);

    return (0);
}

static int spi_sam0_init(const struct device* dev) {
    const struct spi_sam0_config* cfg;
    struct spi_sam0_data* data;
    SercomSpi* regs;
    int ret;

    cfg  = dev->config;
    data = dev->data;
    regs = cfg->regs;

    *cfg->mclk |= cfg->mclk_mask;

    #if defined(MCLK)
    GCLK->PCHCTRL[cfg->gclk_id].reg = GCLK_PCHCTRL_CHEN |
                                      GCLK_PCHCTRL_GEN(cfg->gclk_gen);
    #else
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN(cfg->gclk_gen) |
                        GCLK_CLKCTRL_ID(cfg->gclk_id);
    #endif

    /* Ensure all registers are at their default values */
    regs->CTRLA.bit.SWRST = 1;
    wait_synchronization(regs);

    /* Disable all SPI interrupts */
    regs->INTENCLR.reg = SERCOM_SPI_INTENCLR_MASK;
    wait_synchronization(regs);

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret == 0) {
        #ifdef CONFIG_SPI_ASYNC
        if (device_is_ready(cfg->dma_dev) == true) {
            data->dev = dev;
        }
        else {
            ret = -ENODEV;
        }
        #endif

        if (ret == 0) {
            ret = spi_context_cs_configure_all(&data->ctx);
            if (ret == 0) {
                spi_context_unlock_unconditionally(&data->ctx);

                /* The device will be configured and enabled when transceive
                 * is called.
                 */
            }
        }
    }

    return (ret);
}

static DEVICE_API(spi, spi_sam0_driver_api) = {
    .transceive = spi_sam0_transceive_sync,
    #ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_sam0_transceive_async,
    #endif
    #ifdef CONFIG_SPI_RTIO
    .iodev_submit = spi_rtio_iodev_default_submit,
    #endif
    .release = spi_sam0_release,
};

#if CONFIG_SPI_ASYNC
#define SPI_SAM0_DMA_CHANNELS(n)                    \
    .dma_dev = DEVICE_DT_GET(ATMEL_SAM0_DT_INST_DMA_CTLR(n, tx)),   \
    .tx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),    \
    .tx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, tx),    \
    .rx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),    \
    .rx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),
#else
#define SPI_SAM0_DMA_CHANNELS(n)
#endif

#define SPI_SAM0_SERCOM_PADS(n) \
    SERCOM_SPI_CTRLA_DIPO(DT_INST_PROP(n, dipo)) | \
    SERCOM_SPI_CTRLA_DOPO(DT_INST_PROP(n, dopo))

#define ASSIGNED_CLOCKS_CELL_BY_NAME    \
    ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME

#ifdef MCLK
#define SPI_SAM0_DEFINE_CONFIG(n)                               \
static struct spi_sam0_config DT_CONST spi_sam0_config_##n = {  \
    .regs = (SercomSpi *)DT_INST_REG_ADDR(n),                   \
    .gclk_gen = ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),     \
    .gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),        \
    .mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),      \
    .mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),\
    .pads = SPI_SAM0_SERCOM_PADS(n),                            \
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                  \
    SPI_SAM0_DMA_CHANNELS(n)                                    \
}
#else
#define SPI_SAM0_DEFINE_CONFIG(n)                               \
static struct spi_sam0_config DT_CONST spi_sam0_config_##n = {  \
    .regs = (SercomSpi*)DT_INST_REG_ADDR(n),                    \
    .gclk_gen = ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),     \
    .gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),        \
    .mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),      \
    .mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),\
    .pads = SPI_SAM0_SERCOM_PADS(n),                            \
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                  \
    SPI_SAM0_DMA_CHANNELS(n)                                    \
}
#endif /* MCLK */

#define SPI_SAM0_DEVICE_INIT(n)                 \
    PINCTRL_DT_INST_DEFINE(n);                  \
    SPI_SAM0_DEFINE_CONFIG(n);                  \
    static struct spi_sam0_data spi_sam0_dev_data_##n = {   \
        SPI_CONTEXT_INIT_LOCK(spi_sam0_dev_data_##n, ctx),  \
        SPI_CONTEXT_INIT_SYNC(spi_sam0_dev_data_##n, ctx),  \
        SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)\
    };                                          \
    SPI_DEVICE_DT_INST_DEFINE(n, spi_sam0_init, NULL,   \
                              &spi_sam0_dev_data_##n,   \
                              &spi_sam0_config_##n, POST_KERNEL, \
                              CONFIG_SPI_INIT_PRIORITY, \
                              &spi_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SAM0_DEVICE_INIT)

#if (__GTEST == 1U)                         /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_spi_sam0(void) {
    if (spi_sam0_config_0.regs == (SercomSpi*)0x42001800) {
        /* SERCOM5 */
        spi_sam0_config_0.regs = (SercomSpi*)ut_mcu_sercom_ptr[5];
    }
}
#endif

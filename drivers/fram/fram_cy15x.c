/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for Infineon CY15xJ I2C and CY15xQ SPI FRAMs.
 */

#include <zephyr/drivers/fram.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_FRAM_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fram_cy15x);

/* CY15Q instruction set */
#define FRAM_CY15Q_WRSR     0x01U       /* Write STATUS register        */
#define FRAM_CY15Q_WRITE    0x02U       /* Write data to memory array   */
#define FRAM_CY15Q_READ     0x03U       /* Read data from memory array  */
#define FRAM_CY15Q_WRDI     0x04U       /* Reset the write enable latch */
#define FRAM_CY15Q_RDSR     0x05U       /* Read STATUS register         */
#define FRAM_CY15Q_WREN     0x06U       /* Set the write enable latch   */
#define FRAM_CY15Q_A8_BIT   BIT(3)      /* High-byte address for 4 kbit variant */

/* CY15Q status register bits */
#define FRAM_CY15Q_STATUS_WEL BIT(1) /* Write Enable Latch (RO) */
#define FRAM_CY15Q_STATUS_BP0 BIT(2) /* Block Protection 0 (RW) */
#define FRAM_CY15Q_STATUS_BP1 BIT(3) /* Block Protection 1 (RW) */

#define HAS_WP_OR(id)   DT_NODE_HAS_PROP(id, wp_gpios) ||
#define ANY_INST_HAS_WP_GPIOS (DT_FOREACH_STATUS_OKAY(infineon_cy15xj, HAS_WP_OR) \
                               DT_FOREACH_STATUS_OKAY(infineon_cy15xq, HAS_WP_OR) 0)

struct fram_cy15x_config {
    union {
#ifdef CONFIG_FRAM_CY15XJ
        struct i2c_dt_spec i2c;
#endif /* CONFIG_FRAM_CY15XJ */
#if (defined(CONFIG_FRAM_CY15XQ) || defined(_MSC_VER)) /* #CUSTOM@NDRS */
        struct spi_dt_spec spi;
#endif /* CONFIG_FRAM_CY15XQ */
    } bus;
#if ANY_INST_HAS_WP_GPIOS
    struct gpio_dt_spec wp_gpio;
#endif /* ANY_INST_HAS_WP_GPIOS */
    size_t   size;
    uint8_t  addr_width;
    bool     readonly;
    uint16_t timeout;
    bool (*bus_is_ready)(const struct device *dev);
    fram_api_read  read_fn;
    fram_api_write write_fn;
};

struct fram_cy15x_data {
    struct k_mutex lock;
};

#if ANY_INST_HAS_WP_GPIOS
static inline int fram_cy15x_write_protect(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;

    if (config->wp_gpio.port == NULL) {
        return (0);
    }

    return gpio_pin_set_dt(&config->wp_gpio, 1);
}

static inline int fram_cy15x_write_enable(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;

    if (config->wp_gpio.port == NULL) {
        return (0);
    }

    return gpio_pin_set_dt(&config->wp_gpio, 0);
}
#else
static inline int fram_cy15x_write_protect(const struct device* dev) {
    return (0);
}

static inline int fram_cy15x_write_enable(const struct device* dev) {
    return (0);
}
#endif /* ANY_INST_HAS_WP_GPIOS */

static int fram_cy15x_read(const struct device* dev, off_t offset, void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    struct fram_cy15x_data* data = dev->data;
    uint8_t* pbuf = buf;
    int ret;

    if (len == 0U) {
        return 0U;
    }

    if ((offset + len) > config->size) {
        LOG_WRN("attempt to read past device boundary");
        return (-EINVAL);
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    while (len) {
        ret = config->read_fn(dev, offset, pbuf, len);
        if (ret < 0) {
            LOG_ERR("failed to read FRAM (err %d)", ret);
            k_mutex_unlock(&data->lock);
            return ret;
        }

        pbuf   += ret;
        offset += ret;
        len    -= ret;
    }

    k_mutex_unlock(&data->lock);

    return (0);
}

static int fram_cy15x_write(const struct device* dev, off_t offset,
                            const void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    struct fram_cy15x_data* data = dev->data;
    const uint8_t* pbuf = buf;
    int ret;

    if (config->readonly == true) {
        LOG_WRN("attempt to write to read-only device");
        return (-EACCES);
    }

    if (len == 0U) {
        return (0);
    }

    if ((offset + len) > config->size) {
        LOG_WRN("attempt to write past device boundary");
        return (-EINVAL);
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    if (ANY_INST_HAS_WP_GPIOS == 1) {
        // Write process which have write-protect pin
        ret = fram_cy15x_write_enable(dev);
        if (ret == 0) {
            while (len) {
                ret = config->write_fn(dev, offset, pbuf, len);
                if (ret < 0) {
                    LOG_ERR("failed to write to FRAM (err %d)", ret);
                    break;
                }

                pbuf   += ret;
                offset += ret;
                len    -= ret;
            }

            int rc = fram_cy15x_write_protect(dev);
            if (rc != 0) {
                LOG_ERR("failed to write-protect FRAM (err %d)", rc);
                ret = rc;
            }
        }
        else {
            LOG_ERR("failed to write-enable FRAM (err %d)", ret);
        }
    }
    else {
        ret = 0;
        while (len) {
            ret = config->write_fn(dev, offset, pbuf, len);
            if (ret < 0) {
                LOG_ERR("failed to write to FRAM (err %d)", ret);
                break;
            }

            pbuf   += ret;
            offset += ret;
            len    -= ret;
        }
    }

    if (ret >= 0) {
        ret = 0;
    }

    k_mutex_unlock(&data->lock);

    return (ret);
}

static size_t fram_cy15x_size(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;

    return config->size;
}

#ifdef CONFIG_FRAM_CY15XJ

static bool fram_cy15xj_bus_is_ready(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;

    return device_is_ready(config->bus.i2c.bus);
}

/**
 * @brief translate an offset to a device address / offset pair
 *
 * It allows to address several devices as a continuous memory region
 * but also to address higher part of fram for chips
 * with more than 2^(addr_width) adressable word.
 */
static uint16_t fram_cy15xj_translate_offset(const struct device* dev, off_t* offset) {
    const struct fram_cy15x_config* config = dev->config;

    const uint16_t addr_incr = *offset >> config->addr_width;
    *offset &= BIT_MASK(config->addr_width);

    return (config->bus.i2c.addr + addr_incr);
}

static size_t fram_cy15xj_adjust_read_count(const struct device* dev, off_t offset, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    const size_t remainder = BIT(config->addr_width) - offset;

    if (len > remainder) {
        len = remainder;
    }

    return (len);
}

static int fram_cy15xj_read(const struct device* dev, off_t offset, void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    int64_t  timeout;
    uint8_t  addr[2];
    uint16_t bus_addr;
    int err;

    bus_addr = fram_cy15xj_translate_offset(dev, &offset);
    if (config->addr_width == 16) {
        sys_put_be16(offset, addr);
    }
    else {
        addr[0] = offset & BIT_MASK(8);
    }

    len = fram_cy15xj_adjust_read_count(dev, offset, len);      // Adjust read count to not wrap around

    /*
     * A write cycle may be in progress so reads must be attempted
     * until the current write cycle should be completed.
     */
    timeout = k_uptime_get() + config->timeout;
    while (1) {
        int64_t now = k_uptime_get();
        err = i2c_write_read(config->bus.i2c.bus, bus_addr,
                             addr, (config->addr_width / 8), buf, len);
        if (!err || (now > timeout)) {
            break;
        }

        k_sleep(K_MSEC(1));
    }

    if (err < 0) {
        return (err);
    }

    return (len);
}

static int fram_cy15xj_write(const struct device* dev, off_t offset,
                             const void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    uint8_t  block[(config->addr_width / 8U) + len];
    int64_t  timeout;
    uint16_t bus_addr;
    int i = 0;
    int err;

    bus_addr = fram_cy15xj_translate_offset(dev, &offset);

    /*
     * Not all I2C FRAMs support repeated start so the the
     * address (offset) and data (buf) must be provided in one
     * write transaction (block).
     */
    if (config->addr_width == 16) {
        block[i++] = offset >> 8;
    }
    block[i++] = offset;
    (void) memcpy(&block[i], buf, len);

    /*
     * A write cycle may already be in progress so writes must be
     * attempted until the previous write cycle should be
     * completed.
     */
    timeout = k_uptime_get() + config->timeout;
    while (1) {
        int64_t now = k_uptime_get();
        err = i2c_write(config->bus.i2c.bus, block, sizeof(block),
                        bus_addr);
        if (!err || now > timeout) {
            break;
        }
        k_sleep(K_MSEC(1));
    }

    if (err < 0) {
        return (err);
    }

    return (len);
}
#endif /* CONFIG_FRAM_CY15XJ */

#if (defined(CONFIG_FRAM_CY15XQ) || defined(_MSC_VER)) /* #CUSTOM@NDRS */

static bool fram_cy15xq_bus_is_ready(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;

    return spi_is_ready_dt(&config->bus.spi);
}

static int fram_cy15xq_read(const struct device* dev, off_t offset, void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    size_t   cmd_len = 1U + (config->addr_width / 8U);
    uint8_t  cmd[4]  = {FRAM_CY15Q_READ, 0, 0, 0};
    uint8_t* paddr;
    int err;
    const struct spi_buf tx_buf = {
        .buf = cmd,
        .len = cmd_len
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count   = 1
    };
    const struct spi_buf rx_bufs[2] = {
        {
            .buf = NULL,
            .len = cmd_len,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count   = ARRAY_SIZE(rx_bufs),
    };

    if (len == 0U) {
        return (0);
    }

    if ((offset + len) > config->size) {
        LOG_WRN("attempt to read past device boundary");
        return (-EINVAL);
    }

    if ((config->size <= 512U) && (offset >= 256U)) {
        cmd[0] |= FRAM_CY15Q_A8_BIT;
    }

    paddr = &cmd[1];
    switch (config->addr_width) {
        case 24 :
            *paddr++ = (uint8_t)(offset >> 16);
            __fallthrough;

        case 16 :
            *paddr++ = (uint8_t)(offset >> 8);
            __fallthrough;

        case 8 :
            *paddr = (uint8_t)offset;
            break;

        default :
            __ASSERT(0, "invalid address width");
    }

    err = spi_transceive_dt(&config->bus.spi, &tx, &rx);
    if (err < 0) {
        return err;
    }

    return (len);
}

static int fram_cy15xq_wren(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;
    uint8_t cmd = FRAM_CY15Q_WREN;
    const struct spi_buf tx_buf = {
        .buf = &cmd,
        .len = 1
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    int ret;

    ret = spi_write_dt(&config->bus.spi, &tx);

    return (ret);
}

static int fram_cy15xq_write(const struct device* dev, off_t offset, const void* buf, size_t len) {
    const struct fram_cy15x_config* config = dev->config;
    uint8_t  cmd[4]  = {FRAM_CY15Q_WRITE, 0, 0, 0};
    size_t   cmd_len = (1U + (config->addr_width / 8U));
    uint8_t* paddr;
    int err;
    const struct spi_buf tx_bufs[2] = {
        {
            .buf = cmd,
            .len = cmd_len,
        },
        {
            .buf = (void*)buf,
            .len = len
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs),
    };

    if ((config->size <= 512U) && (offset >= 256U)) {
        cmd[0] |= FRAM_CY15Q_A8_BIT;
    }

    paddr = &cmd[1];
    switch (config->addr_width) {
        case 24 :
            *paddr++ = (uint8_t)(offset >> 16);
        __fallthrough;

        case 16 :
            *paddr++ = (uint8_t)(offset >> 8);
        __fallthrough;

        case 8 :
            *paddr = (uint8_t)(offset);
            break;

        default :
            __ASSERT(0, "invalid address width");
    }

    err = fram_cy15xq_wren(dev);
    if (err) {
        LOG_ERR("failed to disable write protection (err %d)", err);
        return err;
    }

    err = spi_transceive_dt(&config->bus.spi, &tx, NULL);
    if (err) {
        return (err);
    }

    return (len);
}
#endif /* CONFIG_FRAM_CY15XQ */

static int fram_cy15x_init(const struct device* dev) {
    const struct fram_cy15x_config* config = dev->config;
    struct fram_cy15x_data* data = dev->data;

    k_mutex_init(&data->lock);

    if (!config->bus_is_ready(dev)) {
        LOG_ERR("parent bus device not ready");
        return -EINVAL;
    }

#if ANY_INST_HAS_WP_GPIOS
    if (config->wp_gpio.port) {
        int err;
        if (!device_is_ready(config->wp_gpio.port)) {
            LOG_ERR("wp gpio device not ready");
            return -EINVAL;
        }

        err = gpio_pin_configure_dt(&config->wp_gpio, GPIO_OUTPUT_ACTIVE);
        if (err) {
            LOG_ERR("failed to configure WP GPIO pin (err %d)", err);
            return err;
        }
    }
#endif /* ANY_INST_HAS_WP_GPIOS */

    return (0);
}

static const struct fram_driver_api fram_cy15x_api = {
    .read  = fram_cy15x_read,
    .write = fram_cy15x_write,
    .size  = fram_cy15x_size
};

#define ASSERT_CY15j_ADDR_W_VALID(w)    \
    BUILD_ASSERT((w == 8U) || (w == 16U), "Unsupported address width")

#define ASSERT_CY15q_ADDR_W_VALID(w)    \
    BUILD_ASSERT((w == 8U) || (w == 16U) || (w == 24U), "Unsupported address width")

#define INST_DT_CY15X(inst, t) DT_INST(inst, infineon_cy15x##t)

/// @note Required "j" to be small letter
#define fram_cy15j_bus(n, t)    \
    { .i2c = I2C_DT_SPEC_GET(INST_DT_CY15X(n, t)) }

/// @note Required "q" to be small letter
#define fram_cy15q_bus(n, t)    \
    { .spi = SPI_DT_SPEC_GET(INST_DT_CY15X(n, t),       \
             SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |    \
             SPI_WORD_SET(8), 0) }

#define FRAM_CY15X_WP_GPIOS(id) \
    IF_ENABLED(DT_NODE_HAS_PROP(id, wp_gpios),          \
           (.wp_gpio = GPIO_DT_SPEC_GET(id, wp_gpios),))

#define FRAM_CY15X_DEVICE(n, t) \
    ASSERT_CY15##t##_ADDR_W_VALID(DT_PROP(INST_DT_CY15X(n, t), \
                                          address_width)); \
    static const struct fram_cy15x_config fram_cy15x##t##_config_##n = { \
        .bus        = fram_cy15##t##_bus(n, t),     \
        FRAM_CY15X_WP_GPIOS(INST_DT_CY15X(n, t))    \
        .size       = DT_PROP(INST_DT_CY15X(n, t), size),       \
        .addr_width = DT_PROP(INST_DT_CY15X(n, t), address_width),  \
        .readonly   = DT_PROP(INST_DT_CY15X(n, t), read_only),  \
        .timeout    = DT_PROP(INST_DT_CY15X(n, t), timeout),    \
        .bus_is_ready = fram_cy15x##t##_bus_is_ready,           \
        .read_fn  = fram_cy15x##t##_read,  \
        .write_fn = fram_cy15x##t##_write, \
    }; \
    static struct fram_cy15x_data fram_cy15x##t##_data_##n;     \
    DEVICE_DT_DEFINE(INST_DT_CY15X(n, t), &fram_cy15x_init,     \
                     NULL, &fram_cy15x##t##_data_##n,           \
                     &fram_cy15x##t##_config_##n, POST_KERNEL,  \
                     CONFIG_FRAM_INIT_PRIORITY,                 \
                     &fram_cy15x_api)

#define FRAM_CY15XJ_DEVICE(n) FRAM_CY15X_DEVICE(n, j)
#define FRAM_CY15XQ_DEVICE(n) FRAM_CY15X_DEVICE(n, q)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_CY15X_FOREACH(t, inst_expr) \
    LISTIFY(DT_NUM_INST_STATUS_OKAY(infineon_cy15x##t),   \
            CALL_WITH_ARG, (), inst_expr)

#ifdef CONFIG_FRAM_CY15XJ
INST_DT_CY15X_FOREACH(j, FRAM_CY15XJ_DEVICE);
#endif

#if (defined(CONFIG_FRAM_CY15XQ) || defined(_MSC_VER)) /* #CUSTOM@NDRS */
INST_DT_CY15X_FOREACH(q, FRAM_CY15XQ_DEVICE);
#endif

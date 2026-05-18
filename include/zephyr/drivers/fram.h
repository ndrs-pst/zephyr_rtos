/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2023 NDR Solution (Thailand) Co., Ltd.
 *
 * Heavily based on drivers/flash.h which is:
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for FRAM drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FRAM_H_
#define ZEPHYR_INCLUDE_DRIVERS_FRAM_H_

/**
 * @brief Interfaces for Ferroelectric Random Access Memory (FRAM).
 * @defgroup fram_interface FRAM
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*fram_api_read)(struct device const* dev, off_t offset, void* data, size_t len);
typedef int (*fram_api_write)(struct device const* dev, off_t offset, void const* data, size_t len);
typedef size_t (*fram_api_size)(struct device const* dev);

__subsystem struct fram_driver_api {
    fram_api_read read;
    fram_api_write write;
    fram_api_size size;
};

/**
 *  @brief Read data from FRAM
 *
 *  @param dev FRAM device
 *  @param offset Address offset to read from.
 *  @param data Buffer to store read data.
 *  @param len Number of bytes to read.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int fram_read(struct device const* dev, off_t offset, void* data, size_t len);

static inline int z_impl_fram_read(struct device const* dev,
                                   off_t offset, void* data, size_t len) {
    return DEVICE_API_GET(fram, dev)->read(dev, offset, data, len);
}

/**
 *  @brief Write data to FRAM
 *
 *  @param dev FRAM device
 *  @param offset Address offset to write data to.
 *  @param data Buffer with data to write.
 *  @param len Number of bytes to write.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int fram_write(struct device const* dev, off_t offset, void const* data, size_t len);

static inline int z_impl_fram_write(struct device const* dev, off_t offset,
                                    void const* data, size_t len) {
    return DEVICE_API_GET(fram, dev)->write(dev, offset, data, len);
}

/**
 *  @brief Get the size of the FRAM in bytes
 *
 *  @param dev FRAM device.
 *
 *  @return FRAM size in bytes.
 */
__syscall size_t fram_get_size(struct device const* dev);

static inline size_t z_impl_fram_get_size(struct device const* dev) {
    return DEVICE_API_GET(fram, dev)->size(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <zephyr/syscalls/fram.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_FRAM_H_ */

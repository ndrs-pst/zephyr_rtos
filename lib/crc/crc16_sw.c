/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>

static const uint16_t crc16_modbus_small_table[16] = {
    0x0000U, 0xCC01U, 0xD801U, 0x1400U, 0xF001U, 0x3C00U, 0x2800U, 0xE401U,
    0xA001U, 0x6C00U, 0x7800U, 0xB401U, 0x5000U, 0x9C01U, 0x8801U, 0x4400U
};

uint16_t crc16(uint16_t poly, uint16_t seed, uint8_t const* src, size_t len) {
    uint16_t crc = seed;
    size_t i;
    size_t j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)(src[i] << 8U);

        for (j = 0; j < 8; j++) {
            if (crc & 0x8000UL) {
                crc = (crc << 1U) ^ poly;
            }
            else {
                crc = crc << 1U;
            }
        }
    }

    return (crc);
}

uint16_t crc16_reflect(uint16_t poly, uint16_t seed, uint8_t const* src, size_t len) {
    uint16_t crc = seed;
    size_t i;
    size_t j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)src[i];

        for (j = 0; j < 8; j++) {
            if (crc & 0x0001UL) {
                crc = (crc >> 1U) ^ poly;
            }
            else {
                crc = crc >> 1U;
            }
        }
    }

    return (crc);
}

uint16_t crc16_ccitt(uint16_t seed, uint8_t const* src, size_t len) {
    for (; len > 0; len--) {
        uint8_t e, f;

        e    = seed ^ *src++;
        f    = e ^ (e << 4);
        seed = (seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }

    return (seed);
}

uint16_t crc16_itu_t(uint16_t seed, uint8_t const* src, size_t len) {
    for (; len > 0; len--) {
        seed = BSWAP_16(seed);
        seed ^= *src++;
        seed ^= (seed & 0xffU) >> 4U;
        seed ^= seed << 12U;
        seed ^= (seed & 0xffU) << 5U;
    }

    return (seed);
}

uint16_t crc16_modbus(uint16_t seed, uint8_t const* src, size_t len) {
    uint8_t const* p = src;

    for (size_t i = 0U; i < len; i++) {
        seed ^= p[i];
        seed = (seed >> 4) ^ crc16_modbus_small_table[seed & 0x0F];
        seed = (seed >> 4) ^ crc16_modbus_small_table[seed & 0x0F];
    }

    return (seed);
}

/*
 * Copyright (c) 2020 Intel Corporation
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015 Runtime Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/crc.h>

static const uint8_t crc8_ccitt_small_table[16] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D
};

static const uint8_t crc8_rohc_small_table[16] = {
    0x00, 0x1C, 0x38, 0x24, 0x70, 0x6C, 0x48, 0x54,
    0xE0, 0xFC, 0xD8, 0xC4, 0x90, 0x8C, 0xA8, 0xB4
};

uint8_t crc8_ccitt(uint8_t val, void const* buf, size_t cnt) {
    uint8_t const* p = buf;

    for (size_t i = 0U; i < cnt; i++) {
		val ^= p[i];
        val = (uint8_t)((val << 4) ^ crc8_ccitt_small_table[val >> 4]);
        val = (uint8_t)((val << 4) ^ crc8_ccitt_small_table[val >> 4]);
	}

    return (val);
}

uint8_t crc8_rohc(uint8_t val, void const* buf, size_t cnt) {
    uint8_t const* p = buf;

    for (size_t i = 0U; i < cnt; i++) {
        val ^= p[i];
        val = (uint8_t)((val >> 4) ^ crc8_rohc_small_table[val & 0x0F]);
        val = (uint8_t)((val >> 4) ^ crc8_rohc_small_table[val & 0x0F]);
    }

    return (val);
}

uint8_t crc8(uint8_t const* src, size_t len,
             uint8_t polynomial, uint8_t initial_value, bool reversed) {
	uint8_t crc = initial_value;

    for (size_t i = 0U; i < len; i++) {
		crc ^= src[i];

        for (size_t j = 0U; j < 8U; j++) {
			if (reversed) {
                if (crc & 0x01U) {
                    crc = (uint8_t)((crc >> 1U) ^ polynomial);
                }
                else {
                    crc >>= 1;
                }
            }
            else {
                if ((crc & 0x80U) != 0U) {
                    crc = (uint8_t)((crc << 1U) ^ polynomial);
                }
                else {
                    crc <<= 1;
                }
            }
        }
    }

    return (crc);
}

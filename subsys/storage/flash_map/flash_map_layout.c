/*
 * Copyright (c) 2017-2021 Nordic Semiconductor ASA
 * Copyright (c) 2015 Runtime Inc
 * Copyright (c) 2017 Linaro Ltd
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>

struct layout_data {
    uint32_t area_idx;
    uint32_t area_off;
    uint32_t area_len;
    void*    ret;        /* struct flash_area* or struct flash_sector* */
    uint32_t ret_idx;
    uint32_t ret_len;
    int      status;
};

/*
 * Check if a flash_page_foreach() callback should exit early, due to
 * one of the following conditions:
 *
 * - The flash page described by "info" is before the area of interest
 *   described in "data"
 * - The flash page is after the end of the area
 * - There are too many flash pages on the device to fit in the array
 *   held in data->ret. In this case, data->status is set to -ENOMEM.
 *
 * The value to return to flash_page_foreach() is stored in
 * "bail_value" if the callback should exit early.
 */
static bool should_bail(const struct flash_pages_info* info,
                        struct layout_data* data,
                        bool* bail_value) {
    if ((uint32_t)info->start_offset < data->area_off) {
        *bail_value = true;
        return (true);
    }
    else if ((uint32_t)info->start_offset >= (data->area_off + data->area_len)) {
        *bail_value = false;
        return (true);
    }
    else if (data->ret_idx >= data->ret_len) {
        data->status = -ENOMEM;
        *bail_value  = false;
        return (true);
    }

    return (false);
}

static bool get_sectors_cb(const struct flash_pages_info* info, void* datav) {
    struct layout_data*  data = datav;
    struct flash_sector* ret  = data->ret;
    bool bail;

    if (should_bail(info, data, &bail)) {
        return (bail);
    }

    ret[data->ret_idx].fs_off  = (info->start_offset - data->area_off);
    ret[data->ret_idx].fs_size = info->size;
    data->ret_idx++;

    return (true);
}

int flash_area_get_sectors(int fa_id, uint32_t* count, struct flash_sector* sectors) {
    struct layout_data data;
    int rc;
    const struct device* flash_dev;
    const struct flash_area *fa;

    rc = flash_area_open(fa_id, &fa);
    if ((rc < 0) || (fa == NULL)) {
        return (-EINVAL);
    }

    data.area_idx = fa_id;
    data.area_off = fa->fa_off;
    data.area_len = fa->fa_size;

    data.ret     = sectors;
    data.ret_idx = 0U;
    data.ret_len = *count;
    data.status  = 0;

    flash_dev = fa->fa_dev;
    flash_area_close(fa);
    if (flash_dev == NULL) {
        return (-ENODEV);
    }

    flash_page_foreach(flash_dev, get_sectors_cb, &data);

    if (data.status == 0) {
        *count = data.ret_idx;
    }

    return (data.status);
}

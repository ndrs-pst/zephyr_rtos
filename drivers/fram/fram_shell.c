/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2021 Lemonbeat GmbH
 * Copyright (c) 2023 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief FRAM shell commands.
 */

#include <zephyr/shell/shell.h>
#include <zephyr/drivers/fram.h>
#include <stdlib.h>

struct args_index {
    uint8_t device;
    uint8_t offset;
    uint8_t length;
    uint8_t data;
    uint8_t pattern;
};

static const struct args_index args_indx = {
    .device  = 1,
    .offset  = 2,
    .length  = 3,
    .data    = 3,
    .pattern = 4
};

static int cmd_read(const struct shell* sh, size_t argc, char** argv) {
    const struct device* dev;
    size_t addr;
    size_t len;
    size_t pending;
    int ret;
    int err = 0;

    ret  = -EINVAL;
    addr = shell_strtoul(argv[args_indx.offset], 0, &err);
    len  = shell_strtoul(argv[args_indx.length], 0, &err);

    dev = device_get_binding(argv[args_indx.device]);
    if ((dev != NULL) && (err == 0)) {
        shell_print(sh, "Reading %zu bytes from FRAM, offset %zu...", len, addr);

        for (size_t upto = 0U; upto < len; upto += pending) {
            uint8_t data[SHELL_HEXDUMP_BYTES_IN_LINE];

            pending = z_min((len - upto), SHELL_HEXDUMP_BYTES_IN_LINE);
            ret = fram_read(dev, addr, data, pending);
            if (ret == 0) {
                shell_hexdump_line(sh, addr, data, pending);
                addr += pending;
            }
            else {
                shell_error(sh, "FRAM read failed (err %d)", ret);
                break;
            }
        }

        shell_print(sh, "");
    }
    else {
        shell_error(sh, "FRAM device not found");
    }

    return (ret);
}

static int cmd_write(const struct shell* sh, size_t argc, char** argv) {
    uint8_t wr_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
    uint8_t rd_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
    const struct device* dev;
    unsigned long byte;
    off_t  ofs;
    size_t len;
    int ret;
    int err = 0;

    ret = -EINVAL;
    ofs = shell_strtoul(argv[args_indx.offset], 0, &err);
    len = (argc - args_indx.data);
    if ((len <= sizeof(wr_buf)) && (err == 0)) {
        ret = 0;

        for (size_t i = 0; i < len; i++) {
            byte = shell_strtoul(argv[args_indx.data + i], 16, &err);
            if ((err != 0) || (byte > UINT8_MAX)) {
                shell_error(sh, "Error parsing data byte %zu", i);
                ret = -EINVAL;
                break;
            }
            wr_buf[i] = byte;
        }

        if (ret == 0) {
            dev = device_get_binding(argv[args_indx.device]);
            if (dev != NULL) {
                shell_print(sh, "Writing %zu bytes to FRAM...", len);

                ret = fram_write(dev, ofs, wr_buf, len);
                if (ret == 0) {
                    shell_print(sh, "Verifying...");

                    ret = fram_read(dev, ofs, rd_buf, len);
                    if (ret == 0) {
                        if (memcmp(wr_buf, rd_buf, len) == 0) {
                            shell_print(sh, "Verify OK");
                        }
                        else {
                            shell_error(sh, "Verify failed");
                            ret = -EIO;
                        }
                    }
                    else {
                        shell_error(sh, "FRAM read failed (err %d)", ret);
                    }
                }
                else {
                    shell_error(sh, "FRAM write failed (err %d)", ret);
                }
            }
            else {
                shell_error(sh, "FRAM device not found");
            }
        }
    }
    else {
        shell_error(sh, "Write buffer size (%zu bytes) exceeded", sizeof(wr_buf));
    }

    return (ret);
}

static int cmd_size(const struct shell* sh, size_t argc, char** argv) {
    const struct device* dev;
    int ret;

    dev = device_get_binding(argv[args_indx.device]);
    if (dev != NULL) {
        shell_print(sh, "%zu bytes", fram_get_size(dev));
        ret = 0;
    }
    else {
        shell_error(sh, "FRAM device not found");
        ret = -EINVAL;
    }

    return (ret);
}

static int cmd_fill(const struct shell* sh, size_t argc, char** argv) {
    uint8_t wr_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
    uint8_t rd_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
    const struct device *dev;
    unsigned long ptn;
    size_t addr;
    size_t init_ofs;
    size_t len;
    size_t pending;
    int ret;
    int err = 0;

    ret = -EINVAL;
    init_ofs = shell_strtoul(argv[args_indx.offset], 0, &err);
    len = shell_strtoul(argv[args_indx.length], 0, &err);

    ptn = shell_strtoul(argv[args_indx.pattern], 0, &err);
    if ((ptn <= UINT8_MAX) && (err == 0)) {
        (void) memset(wr_buf, ptn, z_min(len, CONFIG_FRAM_SHELL_BUFFER_SIZE));

        dev = device_get_binding(argv[args_indx.device]);
        if (dev != NULL) {
            shell_print(sh, "Writing %zu bytes of 0x%02lx to FRAM...", len, ptn);

            addr = init_ofs;
            for (size_t upto = 0; upto < len; upto += pending) {
                pending = z_min(len - upto, CONFIG_FRAM_SHELL_BUFFER_SIZE);
                ret = fram_write(dev, addr, wr_buf, pending);
                if (ret == 0) {
                    addr += pending;
                }
                else {
                    shell_error(sh, "FRAM write failed (err %d)", ret);
                    break;
                }
            }

            if (ret == 0) {
                addr = init_ofs;

                shell_print(sh, "Verifying...");

                for (size_t upto = 0; upto < len; upto += pending) {
                    pending = z_min(len - upto, CONFIG_FRAM_SHELL_BUFFER_SIZE);
                    ret = fram_read(dev, addr, rd_buf, pending);
                    if (ret == 0) {
                        if (memcmp(wr_buf, rd_buf, pending) == 0) {
                            addr += pending;
                        }
                        else {
                            shell_error(sh, "Verify failed");
                            ret = -EIO;
                            break;
                        }
                    }
                    else {
                        shell_error(sh, "FRAM read failed (err %d)", ret);
                        break;
                    }
                }

                if (ret == 0) {
                    shell_print(sh, "Verify OK");
                }
            }
        }
        else {
            shell_error(sh, "FRAM device not found");
        }
    }
    else {
        shell_error(sh, "Error parsing pattern byte");
    }

    return (ret);
}

/* Device name autocompletion support */
static void device_name_get(size_t idx, struct shell_static_entry* entry) {
    const struct device* dev = shell_device_lookup(idx, NULL);

    if (dev != NULL) {
        entry->syntax = dev->name;
    }
    else {
        entry->syntax = NULL;
    }

    entry->handler = NULL;
    entry->help    = NULL;
    entry->subcmd  = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(fram_cmds,
    SHELL_CMD_ARG(read, &dsub_device_name,
        SHELL_HELP("Read from FRAM",
                   "<device> <offset> <length>"),
                   cmd_read, 4, 0),
    SHELL_CMD_ARG(write, &dsub_device_name,
        SHELL_HELP("Write to FRAM",
                   "<device> <offset> [byte0] <byte1> .. <byteN>"),
                   cmd_write, 4, CONFIG_FRAM_SHELL_BUFFER_SIZE - 1),
    SHELL_CMD_ARG(size, &dsub_device_name,
        SHELL_HELP("Get FRAM size",
                   "<device>"),
                   cmd_size, 2, 0),
    SHELL_CMD_ARG(fill, &dsub_device_name,
        SHELL_HELP("Fill FRAM with a pattern",
                   "<device> <offset> <length> <pattern>"),
                   cmd_fill, 5, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fram, &fram_cmds, "FRAM shell commands", NULL);

/*
 * Copyright (c) 2020 Intel Corporation
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 * Copyright (c) 2022 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <stdlib.h>
#ifdef CONFIG_NATIVE_LIBC
#include <unistd.h>
#else
#include <zephyr/posix/unistd.h>
#endif
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>

#ifndef CONFIG_NATIVE_LIBC
extern void getopt_init(void);
#endif

static inline bool is_ascii(uint8_t data) {
    return (((data >= 0x30) && (data <= 0x39)) ||
            ((data >= 0x61) && (data <= 0x66)) ||
            ((data >= 0x41) && (data <= 0x46)));
}

struct devmem_service_context {
    unsigned char* bytes;
    uint32_t* data;
    int sum;
    int chunk_element;
    char chunk[2];
    bool littleendian;
};

static struct devmem_service_context devmem_context;

#define CHAR_CAN 0x18
#define CHAR_DC1 0x11

#ifndef BITS_PER_BYTE
#define BITS_PER_BYTE 8
#endif

static int memory_dump(const struct shell* sh, mem_addr_t phys_addr, size_t size, uint8_t width) {
    uint32_t value;
    size_t data_offset;
    mm_reg_t addr;
    const size_t vsize = width / BITS_PER_BYTE;
    uint8_t hex_data[SHELL_HEXDUMP_BYTES_IN_LINE];

    #if defined(CONFIG_MMU) || defined(CONFIG_PCIE)
    device_map((mm_reg_t*)&addr, phys_addr, size, K_MEM_CACHE_NONE);

    shell_print(sh, "Mapped 0x%lx to 0x%lx", phys_addr, addr);
    #else
    addr = phys_addr;
    #endif /* defined(CONFIG_MMU) || defined(CONFIG_PCIE) */

    for (; size > 0;
        addr += SHELL_HEXDUMP_BYTES_IN_LINE, size -= MIN(size, SHELL_HEXDUMP_BYTES_IN_LINE)) {
        for (data_offset = 0;
             size >= vsize && data_offset + vsize <= SHELL_HEXDUMP_BYTES_IN_LINE;
             data_offset += vsize) {
            switch (width) {
                case 8 :
                    value = sys_read8(addr + data_offset);
                    hex_data[data_offset] = value;
                    break;

                case 16 :
                    value = sys_le16_to_cpu(sys_read16(addr + data_offset));
                    sys_put_le16(value, &hex_data[data_offset]);
                    break;

                case 32 :
                    value = sys_le32_to_cpu(sys_read32(addr + data_offset));
                    sys_put_le32(value, &hex_data[data_offset]);
                    break;

                default :
                    shell_print(sh, "Incorrect data width");
                    return (-EINVAL);
            }
        }

        shell_hexdump_line(sh, addr, hex_data, MIN(size, SHELL_HEXDUMP_BYTES_IN_LINE));
    }

    return (0);
}

static int cmd_dump(const struct shell* sh, size_t argc, char** argv) {
    int rv;
    size_t size = -1;
    size_t width = 32;
    mem_addr_t addr = -1;

    optind = 1;
    #ifndef CONFIG_NATIVE_LIBC
    getopt_init();
    #endif
    while ((rv = getopt(argc, argv, "a:s:w:")) != -1) {
        switch (rv) {
            case 'a' :
                addr = (mem_addr_t)strtoul(optarg, NULL, 16);
                if (addr == 0 && errno == EINVAL) {
                    shell_error(sh, "invalid addr '%s'", optarg);
                    return (-EINVAL);
                }
                break;

            case 's' :
                size = (size_t)strtoul(optarg, NULL, 0);
                if (size == 0 && errno == EINVAL) {
                    shell_error(sh, "invalid size '%s'", optarg);
                    return (-EINVAL);
                }
                break;

            case 'w' :
                width = (size_t)strtoul(optarg, NULL, 0);
                if (width == 0 && errno == EINVAL) {
                    shell_error(sh, "invalid width '%s'", optarg);
                    return (-EINVAL);
                }
                break;

            case '?' :
            default :
                return (-EINVAL);
        }
    }

    if (addr == -1) {
        shell_error(sh, "'-a <address>' is mandatory");
        return (-EINVAL);
    }

    if (size == -1) {
        shell_error(sh, "'-s <size>' is mandatory");
        return (-EINVAL);
    }

    return memory_dump(sh, addr, size, width);
}

static int set_bypass(const struct shell* sh, shell_bypass_cb_t bypass) {
    static bool in_use;

    if (bypass && in_use) {
        shell_error(sh, "devmem load supports setting bypass on a single instance.");

        return -EBUSY;
    }

    in_use = !in_use;
    if (in_use) {
        shell_print(sh, "Loading...\npress ctrl-x ctrl-q to escape");
        in_use = true;
    }

    shell_set_bypass(sh, bypass);

    return (0);
}

static void bypass_cb(const struct shell* sh, uint8_t* recv, size_t len) {
    bool escape = false;
    static uint8_t tail;
    uint8_t byte;
    struct devmem_service_context* context = &devmem_context;

    if (tail == CHAR_CAN && recv[0] == CHAR_DC1) {
        escape = true;
    }
    else {
        for (int i = 0; i < (len - 1); i++) {
            if (recv[i] == CHAR_CAN && recv[i + 1] == CHAR_DC1) {
                escape = true;
                break;
            }
        }
    }

    if (escape) {
        shell_print(sh, "Number of bytes read: %d", context->sum);
        set_bypass(sh, NULL);

        if (!context->littleendian) {
            while (context->sum > 4) {
                *context->data = BSWAP_32(*context->data);
                context->data++;
                context->sum = context->sum - 4;
            }

            if (context->sum % 4 == 0) {
                *context->data = BSWAP_32(*context->data);
            }
            else if (context->sum % 4 == 2) {
                *context->data = BSWAP_16(*context->data);
            }
            else if (context->sum % 4 == 3) {
                *context->data = BSWAP_24(*context->data);
            }
        }
        return;
    }

    tail = recv[len - 1];

    if (is_ascii(*recv)) {
        context->chunk[context->chunk_element] = *recv;
        context->chunk_element++;
    }

    if (context->chunk_element == 2) {
        byte = (uint8_t)strtoul(context->chunk, NULL, 16);
        *context->bytes = byte;
        context->bytes++;
        context->sum++;
        context->chunk_element = 0;
    }
}

static int cmd_load(const struct shell* sh, size_t argc, char** argv) {
    struct devmem_service_context* context = &devmem_context;
    char* arg;

    context->littleendian = false;
    context->chunk_element = 0;
    context->sum = 0;

    while (argc >= 2) {
        arg = argv[1] + (!strncmp(argv[1], "--", 2) && argv[1][2]);
        if (!strncmp(arg, "-e", 2)) {
            context->littleendian = true;
        }
        else if (!strcmp(arg, "--")) {
            argv++;
            argc--;
            break;
        }
        else if (arg[0] == '-' && arg[1]) {
            shell_print(sh, "Unknown option \"%s\"", arg);
        }
        else {
            break;
        }
        argv++;
        argc--;
    }

    context->bytes = (unsigned char*)strtol(argv[1], NULL, 0);
    context->data  = (uint32_t*)strtol(argv[1], NULL, 0);

    set_bypass(sh, bypass_cb);

    return (0);
}

static int memory_read(const struct shell* sh, mem_addr_t addr, uint8_t width) {
    uint32_t value;
    int      err = 0;

    switch (width) {
        case 8 :
            value = sys_read8(addr);
            break;

        case 16 :
            value = sys_read16(addr);
            break;

        case 32 :
            value = sys_read32(addr);
            break;

        default :
            shell_print(sh, "Incorrect data width");
            err = -EINVAL;
            break;
    }

    if (err == 0) {
        shell_print(sh, "Read value 0x%x", value);
    }

    return (err);
}

static int memory_write(const struct shell* sh, mem_addr_t addr, uint8_t width, uint64_t value) {
    int err = 0;

    switch (width) {
        case 8 :
            sys_write8(value, addr);
            break;

        case 16 :
            sys_write16(value, addr);
            break;

        case 32 :
            sys_write32(value, addr);
            break;

        default :
            shell_print(sh, "Incorrect data width");
            err = -EINVAL;
            break;
    }

    return (err);
}

/* The syntax of the command is similar to busybox's devmem */
static int cmd_devmem(const struct shell* sh, size_t argc, char** argv) {
    mem_addr_t phys_addr;
    mem_addr_t addr;
    uint32_t value;
    uint8_t width;
    char* endptr;

    if ((argc < 2) || (argc > 4)) {
        return (-EINVAL);
    }

    phys_addr = strtoul(argv[1], &endptr, 16);
    if (*endptr != '\0') {
        shell_print(sh, "Invalid physical address format");
        return (-EINVAL);
    }

    #if defined(CONFIG_MMU) || defined(CONFIG_PCIE)
    device_map((mm_reg_t*)&addr, phys_addr, 0x100, K_MEM_CACHE_NONE);

    shell_print(sh, "Mapped 0x%lx to 0x%lx", phys_addr, addr);
    #else
    addr = phys_addr;
    #endif /* defined(CONFIG_MMU) || defined(CONFIG_PCIE) */

    if (argc < 3) {
        width = 32;
    }
    else {
        width = strtoul(argv[2], NULL, 10);
    }

    shell_print(sh, "Using data width %d", width);

    if (argc <= 3) {
        return memory_read(sh, addr, width);
    }

    /* If there are more then 3 arguments, that means we are going to write
     * this value at the address provided
     */

    value = strtoul(argv[3], &endptr, 16);
    if (*endptr != '\0') {
        shell_print(sh, "Invalid value format");
        return (-EINVAL);
    }

    shell_print(sh, "Writing value 0x%x", value);

    return memory_write(sh, addr, width, value);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_devmem,
                               SHELL_CMD_ARG(dump, NULL,
                                             "Usage:\n"
                                             "devmem dump -a <address> -s <size> [-w <width>]\n",
                                             cmd_dump, 5, 2),
                               SHELL_CMD_ARG(load, NULL,
                                             "Usage:\n"
                                             "devmem load [options] [address]\n"
                                             "Options:\n"
                                             "-e\tlittle-endian parse",
                                             cmd_load, 2, 1),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(devmem, &sub_devmem,
                   "Read/write physical memory\n"
                   "Usage:\n"
                   "Read memory at address with optional width:\n"
                   "devmem address [width]\n"
                   "Write memory at address with mandatory width and value:\n"
                   "devmem address <width> <value>",
                   cmd_devmem);

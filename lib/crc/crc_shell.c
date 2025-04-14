/*
 * Copyright (c) 2022 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef CONFIG_ARCH_POSIX
#include <unistd.h>
#else
#include <zephyr/posix/unistd.h>
#endif

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/crc.h>

static char const* const crc_types[] = {
    [CRC4]        = "4",
    [CRC4_TI]     = "4_ti",
    [CRC7_BE]     = "7_be",
    [CRC8]        = "8",
    [CRC8_CCITT]  = "8_ccitt",
    [CRC8_ROHC]   = "8_rohc",
    [CRC16]       = "16",
    [CRC16_ANSI]  = "16_ansi",
    [CRC16_CCITT] = "16_ccitt",
    [CRC16_ITU_T] = "16_itu_t",
    [CRC24_PGP]   = "24_pgp",
    [CRC32_C]     = "32_c",
    [CRC32_IEEE]  = "32_ieee",
    [CRC32_K_4_2] = "32_k_4_2",
};

static int string_to_crc_type(char const* s) {
    int i;

    for (i = 0; i < ARRAY_SIZE(crc_types); ++i) {
        if (strcmp(s, crc_types[i]) == 0) {
            return (i);
        }
    }

    return (-1);
}

static void usage(const struct shell* sh) {
    size_t i;

    shell_print(sh, "crc [options..] <address> <size>");
    shell_print(sh, "options:");
    shell_print(sh, "-f         This is the first packet");
    shell_print(sh, "-l         This is the last packet");
    shell_print(sh, "-p <poly>  Use polynomial 'poly'");
    shell_print(sh, "-r         Reflect");
    shell_print(sh, "-s <seed>  Use 'seed' as the initial value");
    shell_print(sh, "-t <type>  Compute the CRC described by 'type'");
    shell_print(sh, "Note: some options are only useful for certain CRCs");
    shell_print(sh, "CRC Types:");

    for (i = 0; i < ARRAY_SIZE(crc_types); ++i) {
        shell_print(sh, "%s", crc_types[i]);
    }
}

static int cmd_crc(const struct shell* sh, size_t argc, char** argv) {
    int rv;
    bool last     = false;
    uint32_t poly = 0;
    bool first    = false;
    uint32_t seed = 0;
    bool reflect  = false;
    void* addr;
    size_t size;
    enum crc_type type = CRC32_IEEE;

    z_optind = 1;

    while ((rv = z_getopt(argc, argv, "fhlp:rs:t:")) != -1) {
        switch (rv) {
            case 'f' :
                first = true;
                break;

            case 'h' :
                usage(sh);
                return (0);

            case 'l' :
                last = true;
                break;

            case 'p' :
                poly = (size_t)strtoul(z_optarg, NULL, 16);
                if ((poly == 0) && (errno == EINVAL)) {
                    shell_error(sh, "invalid seed '%s'", z_optarg);
                    return (-EINVAL);
                }
                break;

            case 'r' :
                reflect = true;
                break;

            case 's' :
                seed = (size_t)strtoul(z_optarg, NULL, 16);
                if ((seed == 0) && (errno == EINVAL)) {
                    shell_error(sh, "invalid seed '%s'", z_optarg);
                    return (-EINVAL);
                }
                break;

            case 't' :
                type = string_to_crc_type(z_optarg);
                if (type == -1) {
                    shell_error(sh, "invalid type '%s'", z_optarg);
                    return (-EINVAL);
                }
                break;

            case '?' :
            default :
                usage(sh);
                return (-EINVAL);
        }
    }

    if ((size_t)(z_optind + 2) > argc) {
        shell_error(sh, "'address' and 'size' arguments are mandatory");
        usage(sh);
        return (-EINVAL);
    }

    addr = (void*)strtoul(argv[z_optind], NULL, 16);
    if ((addr == 0) && (errno == EINVAL)) {
        shell_error(sh, "invalid address '%s'", argv[z_optind]);
        return (-EINVAL);
    }

    size = (size_t)strtoul(argv[z_optind + 1], NULL, 0);
    if ((size == 0) && (errno == EINVAL)) {
        shell_error(sh, "invalid size '%s'", argv[z_optind + 1]);
        return (-EINVAL);
    }

    shell_print(sh, "0x%x", crc_by_type(type, addr, size, seed, poly, reflect, first, last));

    return (0);
}

SHELL_CMD_ARG_REGISTER(crc, NULL, "CRC calculation", cmd_crc, 0, 12);

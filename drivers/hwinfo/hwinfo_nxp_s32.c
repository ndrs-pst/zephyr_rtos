/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * hwinfo driver for NXP S32K358
 *
 * Reset cause  — MC_RGM @ 0x4028C000 (IP_MC_RGM)
 *                DES: Destructive Event Status  (POR, clocks, security faults)
 *                FES: Functional/External Reset Status (watchdog, pin, JTAG, SW)
 *                Both registers are W1C (write-1-to-clear, safe to write only known bits).
 *
 * Device ID    — SIUL2 @ 0x40290000 (IP_SIUL2)
 *                MIDR1-MIDR4: part number, variant, flash/RAM config, feature set.
 *                NOTE: S32K358 has NO unique per-chip serial number.
 *                      All chips of the same SKU return identical MIDR values.
 *
 * Reset-cause flag mapping:
 *   RESET_POR      <- DES.F_POR
 *   RESET_HARDWARE <- DES.FCCU_FTR | DES.STCU_URF  | DES.MC_RGM_FRE |
 *                     FES.FCCU_RST | FES.ST_DONE    | FES.HSE_BOOT_RST
 *   RESET_CLOCK    <- DES.FXOSC_FAIL | DES.PLL_LOL  | DES.CORE_CLK_FAIL |
 *                     DES.AIPS_PLAT_CLK_FAIL | DES.HSE_CLK_FAIL | DES.SYS_DIV_FAIL |
 *                     FES.PLL_AUX
 *   RESET_SECURITY <- DES.HSE_TMPR_RST | DES.HSE_SNVS_RST
 *   RESET_SOFTWARE <- DES.SW_DEST  | FES.HSE_SWT_RST | FES.SW_FUNC
 *   RESET_DEBUG    <- DES.DEBUG_DEST | FES.JTAG_RST  | FES.DEBUG_FUNC
 *   RESET_PIN      <- FES.F_EXR
 *   RESET_WATCHDOG <- FES.SWT0_RST | FES.SWT1_RST | FES.SWT2_RST
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <errno.h>

#include "S32K358_MC_RGM.h"
#include "S32K358_SIUL2.h"

/* All W1C-capable bits in MC_RGM DES — derived from S32K358_MC_RGM.h masks.
 * Use this mask when reading and when writing back to clear (W1C).
 * Writing only known bits avoids inadvertently toggling reserved fields.
 */
#define MC_RGM_DES_RW_BITS                                      \
    (MC_RGM_DES_F_POR_MASK         | MC_RGM_DES_FCCU_FTR_MASK   | MC_RGM_DES_STCU_URF_MASK | \
     MC_RGM_DES_MC_RGM_FRE_MASK    | MC_RGM_DES_FXOSC_FAIL_MASK | MC_RGM_DES_PLL_LOL_MASK  | \
     MC_RGM_DES_CORE_CLK_FAIL_MASK | MC_RGM_DES_AIPS_PLAT_CLK_FAIL_MASK | MC_RGM_DES_HSE_CLK_FAIL_MASK | \
     MC_RGM_DES_SYS_DIV_FAIL_MASK  | MC_RGM_DES_HSE_TMPR_RST_MASK | MC_RGM_DES_HSE_SNVS_RST_MASK | \
     MC_RGM_DES_SW_DEST_MASK       | MC_RGM_DES_DEBUG_DEST_MASK)

/* All W1C-capable bits in MC_RGM FES. */
#define MC_RGM_FES_RW_BITS                                      \
    (MC_RGM_FES_F_EXR_MASK        | MC_RGM_FES_FCCU_RST_MASK | MC_RGM_FES_ST_DONE_MASK     | \
     MC_RGM_FES_SWT0_RST_MASK     | MC_RGM_FES_SWT1_RST_MASK | MC_RGM_FES_SWT2_RST_MASK    | \
     MC_RGM_FES_JTAG_RST_MASK     | MC_RGM_FES_PLL_AUX_MASK  | MC_RGM_FES_HSE_SWT_RST_MASK | \
     MC_RGM_FES_HSE_BOOT_RST_MASK | MC_RGM_FES_SW_FUNC_MASK  | MC_RGM_FES_DEBUG_FUNC_MASK)

/* Device ID: 4 x 32-bit MIDR registers = 16 bytes. */
#define HWINFO_DEVICE_ID_LEN (4U * sizeof(uint32_t))

/* Grouped masks: all DES/FES bits that map to the same Zephyr flag. */
#define DES_HARDWARE_MASK   (MC_RGM_DES_FCCU_FTR_MASK  | MC_RGM_DES_STCU_URF_MASK   | \
                             MC_RGM_DES_MC_RGM_FRE_MASK)
#define DES_CLOCK_MASK      (MC_RGM_DES_FXOSC_FAIL_MASK | MC_RGM_DES_PLL_LOL_MASK    | \
                             MC_RGM_DES_CORE_CLK_FAIL_MASK | MC_RGM_DES_AIPS_PLAT_CLK_FAIL_MASK | \
                             MC_RGM_DES_HSE_CLK_FAIL_MASK  | MC_RGM_DES_SYS_DIV_FAIL_MASK)
#define DES_SECURITY_MASK   (MC_RGM_DES_HSE_TMPR_RST_MASK | MC_RGM_DES_HSE_SNVS_RST_MASK)
#define FES_HARDWARE_MASK   (MC_RGM_FES_FCCU_RST_MASK  | MC_RGM_FES_ST_DONE_MASK     | \
                             MC_RGM_FES_HSE_BOOT_RST_MASK)
#define FES_WATCHDOG_MASK   (MC_RGM_FES_SWT0_RST_MASK  | MC_RGM_FES_SWT1_RST_MASK    | \
                             MC_RGM_FES_SWT2_RST_MASK)
#define FES_SOFTWARE_MASK   (MC_RGM_FES_HSE_SWT_RST_MASK | MC_RGM_FES_SW_FUNC_MASK)
#define FES_DEBUG_MASK      (MC_RGM_FES_JTAG_RST_MASK  | MC_RGM_FES_DEBUG_FUNC_MASK)

ssize_t z_impl_hwinfo_get_device_id(uint8_t* buffer, size_t length) {
    uint32_t id[4U];

    id[0U] = sys_cpu_to_be32(IP_SIUL2->MIDR1);
    id[1U] = sys_cpu_to_be32(IP_SIUL2->MIDR2);
    id[2U] = sys_cpu_to_be32(IP_SIUL2->MIDR3);
    id[3U] = sys_cpu_to_be32(IP_SIUL2->MIDR4);

    if (length > HWINFO_DEVICE_ID_LEN) {
        length = HWINFO_DEVICE_ID_LEN;
    }

    memcpy(buffer, id, length);

    return ((ssize_t)length);
}

int z_impl_hwinfo_get_reset_cause(uint32_t* cause) {
    uint32_t flags = 0U;
    uint32_t des = (IP_MC_RGM->DES & MC_RGM_DES_RW_BITS);
    uint32_t fes = (IP_MC_RGM->FES & MC_RGM_FES_RW_BITS);

    if ((des & MC_RGM_DES_F_POR_MASK) != 0U) {
        flags |= RESET_POR;
    }

    if ((des & DES_HARDWARE_MASK) != 0U) {
        flags |= RESET_HARDWARE;
    }

    if ((des & DES_CLOCK_MASK) != 0U) {
        flags |= RESET_CLOCK;
    }

    if ((des & DES_SECURITY_MASK) != 0U) {
        flags |= RESET_SECURITY;
    }

    if ((des & MC_RGM_DES_SW_DEST_MASK) != 0U) {
        flags |= RESET_SOFTWARE;
    }

    if ((des & MC_RGM_DES_DEBUG_DEST_MASK) != 0U) {
        flags |= RESET_DEBUG;
    }

    if ((fes & MC_RGM_FES_F_EXR_MASK) != 0U) {
        flags |= RESET_PIN;
    }

    if ((fes & FES_HARDWARE_MASK) != 0U) {
        flags |= RESET_HARDWARE;
    }

    if ((fes & FES_WATCHDOG_MASK) != 0U) {
        flags |= RESET_WATCHDOG;
    }

    if ((fes & MC_RGM_FES_PLL_AUX_MASK) != 0U) {
        flags |= RESET_CLOCK;
    }

    if ((fes & FES_SOFTWARE_MASK) != 0U) {
        flags |= RESET_SOFTWARE;
    }

    if ((fes & FES_DEBUG_MASK) != 0U) {
        flags |= RESET_DEBUG;
    }

    *cause = flags;

    return (0);
}

int z_impl_hwinfo_clear_reset_cause(void) {
    /* W1C: write the bits we own back to clear them. Reading first ensures we
     * only clear bits that are actually set (safe for reserved fields). */
    uint32_t des = (IP_MC_RGM->DES & MC_RGM_DES_RW_BITS);
    uint32_t fes = (IP_MC_RGM->FES & MC_RGM_FES_RW_BITS);

    IP_MC_RGM->DES = des;
    IP_MC_RGM->FES = fes;

    return (0);
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t* supported) {
    *supported = (RESET_POR   |      RESET_PIN | RESET_WATCHDOG | RESET_SOFTWARE |
                  RESET_DEBUG | RESET_HARDWARE | RESET_SECURITY | RESET_CLOCK);

    return (0);
}

int z_impl_hwinfo_get_device_eui64(uint8_t* buffer) {
    /* S32K358 has no EUI-64 — not a wireless-capable MCU. */
    (void)buffer;

    return (-ENOSYS);
}

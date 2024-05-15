/*
 * Copyright (c) 2017 Google LLC.
 * Copyright (c) 2022 Kamil Serwus
 * Copyright (c) 2023 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAMC MCU series initialization code
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <cmsis_core.h>
#include <soc.h>

#define SOC_ATMEL_SAMC_XOSC_FREQ_HZ         16000000UL

#if ((SOC_ATMEL_SAM0_MCK_FREQ_HZ == 48000000UL) || (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 64000000UL))
/* pass */
#else
#error "Expect system to run at 48 MHz or 64 MHz !!!"
#endif

/* Adjust GAIN[2:0] value according to Recommended Max Frequency [MHz] of XOSC */
#if (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 8000000UL)
#define OSCCTRL_XOSCCTRL_GAIN_VAL           OSCCTRL_XOSCCTRL_GAIN(2UL)
#elif (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 16000000UL)
#define OSCCTRL_XOSCCTRL_GAIN_VAL           OSCCTRL_XOSCCTRL_GAIN(3UL)
#else
#error "Expect XOSC frequency at 8 MHz or 16 MHz !!!"
#endif

/*
 *  XOSC32K(32.768 kHz)                     -> FDPLL96M -> GCLK0 (48 MHz)
 *  OSC48M(OUT 16 MHz) / 16 = GCLK3(1 MHz)  -> FDPLL96M -> GCLK0 (48 MHz)
 *  XOSC( 8 MHz) /  8        = 1 MHz        -> FDPLL96M -> GCLK0 (48 MHz)
 *  XOSC(16 MHz) / 16        = 1 MHz        -> FDPLL96M -> GCLK0 (48 MHz)
 */
#define DPLLRATIO_LDR_VAL   \
    (((SOC_ATMEL_SAM0_MCK_FREQ_HZ + (SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ / 2)) / SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ) - 1U)

/**
 * @brief Initialize NVM read wait state
 * @see Table 45-41. NVM Max Speed Characteristics when operating upto +85 °C
 */
static void flash_waitstates_init(void) {
    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 48000000UL) {
        /* One wait state at 48 MHz. */
        NVMCTRL->CTRLB.bit.RWS = 1U;
    }

    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 64000000UL) {
        /* Two wait state at 64 MHz. */
        NVMCTRL->CTRLB.bit.RWS = 2U;
    }
}

/**
 * @brief Initialize 48MHz RC Oscillator (OSC48M)
 * @note
 * CFDPRESC: Since we're using CFD feature, care should be taken to set prescaler value large enough
 * so that CFD clock is less than XOSC clock. With the 2^4 = (16) prescaler,
 * CFD clock is 1 MHz (OSC48M which smaller than XOSC clock of 8 and 16 MHz).
 */
static void osc48m_init(void) {
    uint32_t cal_val;

    /****************** XOSC Initialization ********************************/
    /* 20.8.6 Clock Failure Detector Prescaler */
    /* The OSC48M oscillator is used to clock the CFD prescaler.
       The CFD safe clock frequency is the OSC48M frequency divided by 2^CFDPRESC. */
    OSCCTRL->CFDPRESC.reg = OSCCTRL_CFDPRESC_CFDPRESC(4U);      /* @warning shall set CFDPRESC before set CFDEN */

    cal_val = (uint32_t)(((*(uint64_t*)0x806020UL) >> 41) & 0x3FFFFFUL);
    OSCCTRL->CAL48M.reg = cal_val;

    /* Selection of the Division Value (48 MHz / 3 = 16 MHz) when OSC48M_DIV is 3 */
    OSCCTRL->OSC48MDIV.reg = (uint8_t)OSCCTRL_OSC48MDIV_DIV(SOC_ATMEL_SAM0_OSC48M_DIV - 1U);

    while (OSCCTRL->OSC48MSYNCBUSY.bit.OSC48MDIV == 1) {
        /* Waiting for the synchronization */
    }

    while (OSCCTRL->STATUS.bit.OSC48MRDY == 0) {
        /* Waiting for the OSC48M Ready state */
    }

    OSCCTRL->OSC48MCTRL.bit.ONDEMAND = 0;
}

/**
 * @brief Initialize 32.768 kHz Internal Oscillator (OSC32K)
 * @see 47.6.3 32.768 kHz Internal Oscillator (OSC32K) Characteristics
 */
static void osc32k_init(void) {
    uint32_t cal_val;

    cal_val = (((*(uint32_t*)0x806020UL) >> 12) & 0x7FUL);
    OSC32KCTRL->OSC32K.reg = (OSC32KCTRL_OSC32K_CALIB(cal_val) | OSC32KCTRL_OSC32K_STARTUP(1U) |
                              OSC32KCTRL_OSC32K_ENABLE         | OSC32KCTRL_OSC32K_RUNSTDBY    |
                              OSC32KCTRL_OSC32K_EN32K          | OSC32KCTRL_OSC32K_EN1K);

    /* Wait for the oscillator to stabilize. */
    while (!OSC32KCTRL->STATUS.bit.OSC32KRDY) {
        /* pass */
    }
}

/**
 * @brief Initialize External Multipurpose Crystal Oscillator (XOSC) Control
 * @note 
 * CFDPRESC: Since we're using CFD feature, care should be taken to set prescaler value large enough
 * so that CFD clock is less than XOSC clock. With the 2^4 = (16) prescaler,
 * CFD clock is 1 MHz (OSC48M which smaller than XOSC clock of 8 and 16 MHz.
 * 
 * STARTUP[3:0]: @see Table 20-5. Start-Up Time for External Multipurpose Crystal Oscillator
 * With the current setting value of 6, the start-up time is 1,953 μs.
 */
static void xosc_init(void) {
    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC)
    /* Configure External Oscillator */
    OSCCTRL->XOSCCTRL.reg = (uint16_t)(OSCCTRL_XOSCCTRL_STARTUP(6UL) | OSCCTRL_XOSCCTRL_GAIN_VAL |
                                       OSCCTRL_XOSCCTRL_RUNSTDBY     | OSCCTRL_XOSCCTRL_CFDEN    |
                                       OSCCTRL_XOSCCTRL_XTALEN       | OSCCTRL_XOSCCTRL_ENABLE);
    while (OSCCTRL->STATUS.bit.XOSCRDY == 0) {
        /* Waiting for the XOSC Ready state */
    }

    /* Setting the Automatic Gain Control */
    OSCCTRL->XOSCCTRL.bit.AMPGC = 1U;
    #endif
}

static void xosc32k_init(void) {
    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K)
    OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP(1UL) | OSC32KCTRL_XOSC32K_ENABLE |
                               OSC32KCTRL_XOSC32K_RUNSTDBY     | OSC32KCTRL_XOSC32K_EN32K  |
                               OSC32KCTRL_XOSC32K_EN1K         | OSC32KCTRL_XOSC32K_XTALEN);

    /* The CFD safe clock frequency is the OSCULP32K frequency divided by 2 */
    OSC32KCTRL->CFDCTRL.bit.CFDPRESC = 1U;

    /* Enable clock failure detection */
    OSC32KCTRL->CFDCTRL.bit.CFDEN = 1U;

    /* Wait for the crystal to stabilize. */
    while (OSC32KCTRL->STATUS.bit.XOSC32KRDY == 0U) {
        /* pass */
    }
    #endif
}

/// @note Errata 1.25.1 FDPLL Unlock
/// When using FDPLL at temperature below 25°C, spurious DPLL unlocks (OSCCTRL.DPLLSTATUS.LOCK = 0)
/// may be detected while the FDPLL still adheres to the metrics described in the related electrical characteristics
/// chapters of the data sheet. During these unlock periods, the DPLL output clock is halted and then restarts.
/// Workaround :
/// When using FDPLL at temperature below 25°C, enable the lock bypass (OSCCTRL.DPLLCTRLB.LBYPASS = 1)
/// to avoid losing FDPLL clock output during a false unlock status.
static void fdpll_init(void) {
    /* Disable DPLL beforehand */
    OSCCTRL->DPLLCTRLA.bit.ENABLE = 0U;

    /****************** DPLL Initialization *********************************/
    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K_AS_MAIN)
    /* Configure DPLL */
    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 48000000UL) {
        /* XOSC32k(32.768 kHz) = 32,768 x ((2,928 + 1) + 11/16) = 96 MHz */
        OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL) | OSCCTRL_DPLLCTRLB_LBYPASS |
                                  OSCCTRL_DPLLCTRLB_REFCLK(0UL));
        OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(11UL) | OSCCTRL_DPLLRATIO_LDR(2928UL);
    }

    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 64000000UL) {
        /* XOSC32k(32.768 kHz) = 32,768 x ((1,952 + 1) + 2/16) = 64 MHz */
        OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL) | OSCCTRL_DPLLCTRLB_LBYPASS |
                                  OSCCTRL_DPLLCTRLB_REFCLK(0UL));
        OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(2UL) | OSCCTRL_DPLLRATIO_LDR(1952UL);
    }
    #elif defined(CONFIG_SOC_ATMEL_SAMC_OSC48M_AS_MAIN)
    /* Route OSC48M(16 MHz) / 16 = GCLK3 (1 MHz) */
    GCLK->GENCTRL[3].reg = (GCLK_GENCTRL_DIV(16UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M_Val) |
                            GCLK_GENCTRL_RUNSTDBY  | GCLK_GENCTRL_GENEN);
    while (GCLK->SYNCBUSY.bit.GENCTRL3 == 1) {
        /* wait for the Generator 3 synchronization */
    }

    /* Enable GCLK3 as GCLK_DPLL : FDPLL96M input clock source for reference */
    GCLK->PCHCTRL[0].reg = GCLK_PCHCTRL_GEN(0x3UL) | GCLK_PCHCTRL_CHEN;
    while (GCLK->PCHCTRL[0].bit.CHEN == 1U) {
        /* Wait for synchronization */
    }

    /* GCLK3 (1 MHz) x 48 = 48 MHz / 1 = 48 MHz */
    OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL) | OSCCTRL_DPLLCTRLB_LBYPASS |
                              OSCCTRL_DPLLCTRLB_REFCLK(2UL));
    #elif defined(CONFIG_SOC_ATMEL_SAMC_XOSC_AS_MAIN)
    /* Configure DPLL */
    if (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 8000000UL) {
        /* (XOSC(8M) / 8) = 1 MHz */
        /* OSCCTRL_DPLLCTRLB_DIV(3) : fDIV = fXOSC / 2 * (DIV + 1) */
        OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL) | OSCCTRL_DPLLCTRLB_LBYPASS |
                                  OSCCTRL_DPLLCTRLB_REFCLK(1UL) | OSCCTRL_DPLLCTRLB_DIV(3));
    }

    if (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 16000000UL) {
        /* (XOSC(16M) / 16) = 1 MHz */
        /* OSCCTRL_DPLLCTRLB_DIV(7) : fDIV = fXOSC / 2 * (DIV + 1) */
        OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL) | OSCCTRL_DPLLCTRLB_LBYPASS |
                                  OSCCTRL_DPLLCTRLB_REFCLK(1UL) | OSCCTRL_DPLLCTRLB_DIV(7));
    }
    #else
    #error Unsupported main clock source.
    #endif

    #if !defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K_AS_MAIN)
    OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0UL) | OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR_VAL);
    #endif

    while (OSCCTRL->DPLLSYNCBUSY.bit.DPLLRATIO == 1U) {
        /* Waiting for the synchronization */
    }

    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K_AS_MAIN)
    /* Selection of the DPLL Pre-Scalar */
    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 48000000UL) {
        /* XOSC32k(32.768 kHz) = 32,768 x ((2,928 + 1) + 11/16) = 96 MHz / 2U = 48 MHz */
        OSCCTRL->DPLLPRESC.reg = (uint8_t)OSCCTRL_DPLLPRESC_PRESC(1UL);
    }

    if (SOC_ATMEL_SAM0_MCK_FREQ_HZ == 64000000UL) {
        /* XOSC32k(32.768 kHz) = 32,768 x ((1,952 + 1) + 2/16) = 64 MHz / 1U = 64 MHz */
        OSCCTRL->DPLLPRESC.reg = (uint8_t)OSCCTRL_DPLLPRESC_PRESC(0UL);
    }

    while (OSCCTRL->DPLLSYNCBUSY.bit.DPLLPRESC == 1U) {
        /* Waiting for the synchronization */
    }
    #endif

    /* Selection of the DPLL Enable */
    OSCCTRL->DPLLCTRLA.reg = (uint8_t)(OSCCTRL_DPLLCTRLA_ENABLE | OSCCTRL_DPLLCTRLA_RUNSTDBY);

    while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE == 1U) {
        /* Waiting for the DPLL enable synchronization */
    }

    while ((OSCCTRL->DPLLSTATUS.reg & (OSCCTRL_DPLLSTATUS_LOCK | OSCCTRL_DPLLSTATUS_CLKRDY)) !=
           (OSCCTRL_DPLLSTATUS_LOCK | OSCCTRL_DPLLSTATUS_CLKRDY)) {
        /* Waiting for the Ready state */
    }
}

static void mclk_init(void) {
    MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV_DIV1_Val;
}

static void wait_gclk_sync(uint32_t sync_bit) {
    while ((GCLK->SYNCBUSY.reg & sync_bit) == sync_bit) {
        /* pass */
    }
}

static void gclks_deinit(void) {
    // The reset value is 0x00000106 for GCLK0
    GCLK->GENCTRL[0].reg = (GCLK_GENCTRL_DIV(0UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M_Val) |
                            GCLK_GENCTRL_GENEN);
}

static void gclks_init(void) {
    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC_AS_MAIN)
    if (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 8000000UL) {
        /* XOSC(8M)/1 -> GCLK2 (8 MHz) */
        GCLK->GENCTRL[2].reg = (GCLK_GENCTRL_DIV(1UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC_Val) |
                                GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
        wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK2);
    }

    if (SOC_ATMEL_SAMC_XOSC_FREQ_HZ == 16000000UL) {
        /* XOSC(16M)/2 -> GCLK2 (8 MHz) */
        GCLK->GENCTRL[2].reg = (GCLK_GENCTRL_DIV(2UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC_Val) |
                                GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
        wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK2);
    }
    #endif

    /* FDPLL/2 -> GCLK1 */
    GCLK->GENCTRL[1].reg = (GCLK_GENCTRL_DIV(2UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL96M_Val) |
                            GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
    wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK1);

    /* FDPLL/1 -> GCLK0 */
    GCLK->GENCTRL[0].reg = (GCLK_GENCTRL_DIV(1UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL96M_Val) |
                            GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
    wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK0);
}

void z_arm_platform_init(void) {
    flash_waitstates_init();
    osc48m_init();
    osc32k_init();
    xosc_init();
    xosc32k_init();
    gclks_deinit();
    fdpll_init();
    mclk_init();
    gclks_init();

    /* Configure the AHB Bridge Clocks */
    MCLK->AHBMASK.reg = 0x1DFFU;
}

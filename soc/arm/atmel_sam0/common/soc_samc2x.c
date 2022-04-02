/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAMC MCU series initialization code
 */

#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

#if (SOC_ATMEL_SAM0_MCK_FREQ_HZ != 48000000)
#error "Expect system to run at 48 MHz !!!"
#endif

static void flash_waitstates_init(void) {
    /* One wait state at 48 MHz. */
    NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;
}

/*
 *  XOSC32K(32.768 kHz)                 -> DFLL48M -> GCLK0 (48 MHz)
 *  XOSC48M(48 MHz) / 48 = GCLK1(1 MHz) -> DFLL48M -> GCLK0 (48 MHz)
 *  XOSC(8 MHz) / 8      = 1 MHz        -> DFLL48M -> GCLK0 (48 MHz)
 */

static void xosc_init(void) {
    #if defined(CONFIG_SOC_ATMEL_SAMC_XOSC)
    /* Configure External Oscillator */
    OSCCTRL->XOSCCTRL.reg = (uint16_t)(OSCCTRL_XOSCCTRL_STARTUP(3UL) | OSCCTRL_XOSCCTRL_GAIN(2UL) |
                                       OSCCTRL_XOSCCTRL_RUNSTDBY     | OSCCTRL_XOSCCTRL_CFDEN     |
                                       OSCCTRL_XOSCCTRL_XTALEN       | OSCCTRL_XOSCCTRL_ENABLE);
    while (OSCCTRL->STATUS.bit.XOSCRDY == 0) {
        /* Waiting for the XOSC Ready state */
    }

    /* Setting the Automatic Gain Control */
    OSCCTRL->XOSCCTRL.reg |= (uint16_t)OSCCTRL_XOSCCTRL_AMPGC;
    #endif
}

static void wait_gclk_sync(uint32_t sync_bit) {
    while ((GCLK->SYNCBUSY.reg & sync_bit) == sync_bit) {
        /* pass */
    }
}

static void xosc32k_init(void) {
    #ifdef CONFIG_SOC_ATMEL_SAMC_XOSC32K
    OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP(1UL) | OSC32KCTRL_XOSC32K_ENABLE |
                               OSC32KCTRL_XOSC32K_RUNSTDBY     | OSC32KCTRL_XOSC32K_EN32K  | OSC32KCTRL_XOSC32K_XTALEN);

    /* Enable clock failure detection */
    OSC32KCTRL->CFDCTRL.bit.CFDEN = 1U;

    /* Wait for the crystal to stabilize. */
    while (!OSC32KCTRL->STATUS.bit.XOSC32KRDY) {
        /* pass */
    }

    OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL(OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val);
    #endif
}

static void osc32k_init(void) {
    uint32_t cal_val;

    cal_val = (((*(uint32_t*)0x806020UL) >> 12) & 0x7FUL);
    OSC32KCTRL->OSC32K.reg = (OSC32KCTRL_OSC32K_CALIB(cal_val) | OSC32KCTRL_OSC32K_STARTUP(1U) |
                              OSC32KCTRL_OSC32K_ENABLE         | OSC32KCTRL_OSC32K_RUNSTDBY    |
                              OSC32KCTRL_OSC32K_EN32K);

	/* Wait for the oscillator to stabilize. */
    while (!OSC32KCTRL->STATUS.bit.OSC32KRDY) {
        /* pass */
    }

    OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL(OSC32KCTRL_RTCCTRL_RTCSEL_OSC32K_Val);
}

#define DPLLRATIO_LDR_VAL   (((SOC_ATMEL_SAM0_MCK_FREQ_HZ + (SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ / 2)) / SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ) - 1U)

static void fdpll_init(void) {
    /****************** DPLL Initialization  *********************************/
#if defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K_AS_MAIN)
    /* Configure DPLL */
    /* XOSC32k(32.768 kHz) = 32,768 x (2928 + 11/16) = 96 MHz */
    OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL)  | OSCCTRL_DPLLCTRLB_LTIME(0UL)| OSCCTRL_DPLLCTRLB_REFCLK(0UL));
    OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(11UL) | OSCCTRL_DPLLRATIO_LDR(2928UL);
#elif defined(CONFIG_SOC_ATMEL_SAMC_OSC48M_AS_MAIN)
	// Enable GCLK_DPLL : FDPLL96M input clock source for reference
    GCLK->PCHCTRL[0].reg = GCLK_PCHCTRL_GEN(0x1UL) | GCLK_PCHCTRL_CHEN;
    while (GCLK->PCHCTRL[0].bit.CHEN == 1U) {
        /* Wait for synchronization */
    }

	/* Route OSC48M(48 MHz) / 48 = GCLK1 (1 MHz) */
    GCLK->GENCTRL[1].reg = (GCLK_GENCTRL_DIV(48UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M_Val) |
                            GCLK_GENCTRL_RUNSTDBY  | GCLK_GENCTRL_GENEN);
    while (GCLK->SYNCBUSY.bit.GENCTRL1 == 1) {
        /* wait for the Generator 1 synchronization */
    }

    /* GCLK1 (1 MHz) x 48 = 48 MHz / 1 = 48 MHz */
    OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL)| OSCCTRL_DPLLCTRLB_REFCLK(2UL));
#elif defined(CONFIG_SOC_ATMEL_SAMC_XOSC_AS_MAIN)
    /* Configure DPLL */
	/* (XOSC(8M) / 8) = 1 MHz x 48 = 48 MHz / 1 = 48 MHz */
	/* OSCCTRL_DPLLCTRLB_DIV(3) : fDIV = fXOSC / 2 * (DIV + 1) */
    OSCCTRL->DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_FILTER(0UL) | OSCCTRL_DPLLCTRLB_LTIME(0UL)|
                              OSCCTRL_DPLLCTRLB_REFCLK(1UL) | OSCCTRL_DPLLCTRLB_DIV(3));
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
    /* XOSC32k(32.768 kHz) = 32,768 x (2928 + 11/16) = 96 MHz / 2U = 48 MHz */
    OSCCTRL->DPLLPRESC.reg = (uint8_t)OSCCTRL_DPLLPRESC_PRESC(1UL);

    while (OSCCTRL->DPLLSYNCBUSY.bit.DPLLPRESC == 1U) {
        /* Waiting for the synchronization */
    }
    #endif

    /* Selection of the DPLL Enable */
    OSCCTRL->DPLLCTRLA.reg = (uint8_t)(OSCCTRL_DPLLCTRLA_ENABLE);

    while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE == 1U) {
        /* Waiting for the DPLL enable synchronization */
    }

    while ((OSCCTRL->DPLLSTATUS.reg & (OSCCTRL_DPLLSTATUS_LOCK | OSCCTRL_DPLLSTATUS_CLKRDY)) !=
                    (OSCCTRL_DPLLSTATUS_LOCK | OSCCTRL_DPLLSTATUS_CLKRDY)) {
        /* Waiting for the Ready state */
    }
}

static void osc48m_init(void) {
    uint32_t cal_val;

    /****************** XOSC Initialization   ********************************/
    /* Selection of the Clock failure detection (CFD) pre scalar */
    OSCCTRL->CFDPRESC.bit.CFDPRESC = OSCCTRL_CFDPRESC_CFDPRESC(0);

    cal_val = (uint32_t)(((*(uint64_t*)0x806020UL) >> 41 ) & 0x3fffffUL);
    OSCCTRL->CAL48M.reg = cal_val;

    /* Selection of the Division Value */
    OSCCTRL->OSC48MDIV.reg = (uint8_t)OSCCTRL_OSC48MDIV_DIV(0UL);

    while (OSCCTRL->OSC48MSYNCBUSY.bit.OSC48MDIV == 1) {
        /* Waiting for the synchronization */
    }

    while (OSCCTRL->STATUS.bit.OSC48MRDY == 0) {
        /* Waiting for the OSC48M Ready state */
    }


    OSCCTRL->OSC48MCTRL.bit.ONDEMAND = 0;
}

static void gclks_init(void) {
    /* OSC48M/2 -> GCLK3 */
    GCLK->GENCTRL[3].reg = (GCLK_GENCTRL_DIV(2UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M_Val) |
                            GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
    wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK3);

    /* OSCULP32K/32 -> GCLK2 */
    GCLK->GENCTRL[2].reg = (GCLK_GENCTRL_DIV(32UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K_Val) |
                            GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
    wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK2);

    /* FDPLL/1 -> GCLK0 */
    GCLK->GENCTRL[0].reg = (GCLK_GENCTRL_DIV(1UL) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL96M_Val) |
                            GCLK_GENCTRL_IDC      | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN);
    wait_gclk_sync(GCLK_SYNCBUSY_GENCTRL_GCLK0);
}

static int atmel_samc_init(const struct device* arg) {
    uint32_t key;

    ARG_UNUSED(arg);

    key = irq_lock();

    flash_waitstates_init();
    osc48m_init();
    osc32k_init();
    xosc_init();
    xosc32k_init();
    fdpll_init();
    gclks_init();

    /* Configure the AHB Bridge Clocks */
    MCLK->AHBMASK.reg = 0x1DFFU;

    /* Install default handler that simply resets the CPU
     * if configured in the kernel, NOP otherwise
     */
    NMI_INIT();

    irq_unlock(key);

    return 0;
}

SYS_INIT(atmel_samc_init, PRE_KERNEL_1, 0);

/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ATMEL_SAMC_SOC_H_
#define _ATMEL_SAMC_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>


#if defined(CONFIG_SOC_PART_NUMBER_SAMC21E15A)
#include <samc21e15a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21E16A)
#include <samc21e16a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21E17A)
#include <samc21e17a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21E18A)
#include <samc21e18a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G15A)
#include <samc21g15a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G16A)
#include <samc21g16a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G17A)
#include <samc21g17a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G17AU)
#include <samc21g17au.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G18A)
#include <samc21g18a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21G18AU)
#include <samc21g18au.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21J15A)
#include <samc21j15a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21J16A)
#include <samc21j16a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21J17A)
#include <samc21j17a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMC21J18A)
#include <samc21j18a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#include "adc_fixup_sam0.h"
#include "../common/soc_port.h"
#include "../common/atmel_sam0_dt.h"

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM0_HCLK_FREQ_HZ         ATMEL_SAM0_DT_CPU_CLK_FREQ_HZ

/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAM0_MCK_FREQ_HZ          SOC_ATMEL_SAM0_HCLK_FREQ_HZ
#define SOC_ATMEL_SAM0_OSC32K_FREQ_HZ       32768
#define SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ      32768
#define SOC_ATMEL_SAM0_FDPLL_OUT_FREQ_HZ    SOC_ATMEL_SAM0_HCLK_FREQ_HZ
#define SOC_ATMEL_SAM0_OSC48M_FREQ_HZ       48000000
#define SOC_ATMEL_SAM0_GCLK0_FREQ_HZ        SOC_ATMEL_SAM0_FDPLL_OUT_FREQ_HZ

#if defined(CONFIG_SOC_ATMEL_SAMC_XOSC32K_AS_MAIN)
#define SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ     SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ
#elif defined(CONFIG_SOC_ATMEL_SAMC_OSC48M_AS_MAIN)
#define SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ     1000000
#elif defined(CONFIG_SOC_ATMEL_SAMC_XOSC_AS_MAIN)
#define SOC_ATMEL_SAM0_FDPLL_IN_FREQ_HZ     1000000
#else
#error Unsupported GCLK3 clock source.
#endif

#define SOC_ATMEL_SAM0_GCLK1_FREQ_HZ        (SOC_ATMEL_SAM0_FDPLL_OUT_FREQ_HZ / 2U)
#define SOC_ATMEL_SAM0_APBA_FREQ_HZ         SOC_ATMEL_SAM0_MCK_FREQ_HZ
#define SOC_ATMEL_SAM0_APBB_FREQ_HZ         SOC_ATMEL_SAM0_MCK_FREQ_HZ
#define SOC_ATMEL_SAM0_APBC_FREQ_HZ         SOC_ATMEL_SAM0_MCK_FREQ_HZ
#define SOC_ATMEL_SAM0_APBD_FREQ_HZ         SOC_ATMEL_SAM0_MCK_FREQ_HZ

#endif /* _ATMEL_SAMC_SOC_H_ */

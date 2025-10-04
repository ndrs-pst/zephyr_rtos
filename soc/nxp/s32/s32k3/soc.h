/*
 * Copyright 2023, 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NXP_S32_S32K_SOC_H_
#define _NXP_S32_S32K_SOC_H_

#if defined(CONFIG_SOC_S32K344)
#include <S32K344.h>
#elif defined(CONFIG_SOC_S32K358)
/* S32K358 temporarily uses S32K344 headers until S32K358.h is added to the NXP HAL */
#include <S32K344.h>
#else
#error "Unknown S32K3 SoC"
#endif

#include <core_cm7.h>

#if defined(CONFIG_CMSIS_RTOS_V2)
#include <cmsis_rtos_v2_adapt.h>
#include <soc_common.h>
#endif

#endif /* _NXP_S32_S32K_SOC_H_ */

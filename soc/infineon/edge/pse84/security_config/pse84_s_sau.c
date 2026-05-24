/*
 * SPDX-FileCopyrightText: <text>Copyright (c) 2026 Infineon Technologies AG,
 * or an affiliate of Infineon Technologies AG. All rights reserved.</text>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pse84_s_sau.h"

const cy_sau_config_t sau_config[CY_SAU_REGION_CNT] = {
	{
		.reg_num = 0U,
		.base_addr = 0x00000000U,
		.size = 0x10000000U,
		.end_addr = 0x0FFFFFFFU,
		.nsc = false,
	},
	{
		.reg_num = 1U,
		.base_addr = 0x20000000U,
		.size = 0x10000000U,
		.end_addr = 0x2FFFFFFFU,
		.nsc = false,
	},
	{
		.reg_num = 2U,
		.base_addr = 0x40000000U,
		.size = 0xC0000000U,
		.end_addr = 0xFFFFFFFFU,
		.nsc = false,
	},
};

void cy_sau_init(void)
{
	SAU->CTRL |= SAU_CTRL_ENABLE_Msk;
	for (size_t i = 0U; i < CY_SAU_REGION_CNT; i++) {
		cy_sau_config_t const* config = &sau_config[i];

		SAU->RNR  = config->reg_num;
		SAU->RBAR = (config->base_addr & SAU_RBAR_BADDR_Msk);
		SAU->RLAR = ((config->end_addr & SAU_RLAR_LADDR_Msk) |
			     (config->nsc ? SAU_RLAR_NSC_Msk : 0U) | SAU_RLAR_ENABLE_Msk);
	}
}

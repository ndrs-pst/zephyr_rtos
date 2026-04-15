/*
 * Copyright 2022, 2024-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_SIUL2_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_SIUL2_PINCTRL_H_

#include <zephyr/dt-bindings/dt-util.h>

/*
 * The NXP S32 pinmux configuration is encoded in a 64-bit field value as follows:
 *
 * - 0..3:   Output mux Source Signal Selection (MSCR.SSS)
 * - 4..8:   Input mux Source Signal Selection (IMCR.SSS)
 * - 9..17:  Input Multiplexed Signal Configuration Register (IMCR) index
 * - 18..26: Multiplexed Signal Configuration Register (MSCR) index
 * - 27..29: MSCR SIUL2 instance index (0..7)
 * - 30..32: IMCR SIUL2 instance index (0..7)
 * - 33..63: Reserved for future use
 */
#define NXP_SIUL2_MSCR_SSS_SHIFT       0U
#define NXP_SIUL2_MSCR_SSS_MASK        BIT64_MASK(4)
#define NXP_SIUL2_IMCR_SSS_SHIFT       4U
#define NXP_SIUL2_IMCR_SSS_MASK        BIT64_MASK(4)
#define NXP_SIUL2_IMCR_IDX_SHIFT       8U
#define NXP_SIUL2_IMCR_IDX_MASK        BIT64_MASK(9)
#define NXP_SIUL2_MSCR_IDX_SHIFT       17U
#define NXP_SIUL2_MSCR_IDX_MASK        BIT64_MASK(9)
#define NXP_SIUL2_MSCR_SIUL2_IDX_SHIFT 26U
#define NXP_SIUL2_MSCR_SIUL2_IDX_MASK  BIT64_MASK(3)
#define NXP_SIUL2_IMCR_SIUL2_IDX_SHIFT 29U
#define NXP_SIUL2_IMCR_SIUL2_IDX_MASK  BIT64_MASK(3)

#define NXP_SIUL2_PINMUX_MSCR_SSS(cfg)                                                             \
	(((cfg) & NXP_SIUL2_MSCR_SSS_MASK) << NXP_SIUL2_MSCR_SSS_SHIFT)

#define NXP_SIUL2_PINMUX_IMCR_SSS(cfg)                                                             \
	(((cfg) & NXP_SIUL2_IMCR_SSS_MASK) << NXP_SIUL2_IMCR_SSS_SHIFT)

#define NXP_SIUL2_PINMUX_IMCR_IDX(cfg)                                                             \
	(((cfg) & NXP_SIUL2_IMCR_IDX_MASK) << NXP_SIUL2_IMCR_IDX_SHIFT)

#define NXP_SIUL2_PINMUX_MSCR_IDX(cfg)                                                             \
	(((cfg) & NXP_SIUL2_MSCR_IDX_MASK) << NXP_SIUL2_MSCR_IDX_SHIFT)

#define NXP_SIUL2_PINMUX_MSCR_SIUL2_IDX(cfg)                                                       \
	(((cfg) & NXP_SIUL2_MSCR_SIUL2_IDX_MASK) << NXP_SIUL2_MSCR_SIUL2_IDX_SHIFT)

#define NXP_SIUL2_PINMUX_IMCR_SIUL2_IDX(cfg)                                                       \
	(((cfg) & NXP_SIUL2_IMCR_SIUL2_IDX_MASK) << NXP_SIUL2_IMCR_SIUL2_IDX_SHIFT)

#define NXP_SIUL2_PINMUX_GET_MSCR_SSS(cfg)                                                         \
	(((cfg) >> NXP_SIUL2_MSCR_SSS_SHIFT) & NXP_SIUL2_MSCR_SSS_MASK)

#define NXP_SIUL2_PINMUX_GET_IMCR_SSS(cfg)                                                         \
	(((cfg) >> NXP_SIUL2_IMCR_SSS_SHIFT) & NXP_SIUL2_IMCR_SSS_MASK)

#define NXP_SIUL2_PINMUX_GET_IMCR_IDX(cfg)                                                         \
	(((cfg) >> NXP_SIUL2_IMCR_IDX_SHIFT) & NXP_SIUL2_IMCR_IDX_MASK)

#define NXP_SIUL2_PINMUX_GET_MSCR_IDX(cfg)                                                         \
	(((cfg) >> NXP_SIUL2_MSCR_IDX_SHIFT) & NXP_SIUL2_MSCR_IDX_MASK)

#define NXP_SIUL2_PINMUX_GET_MSCR_SIUL2_IDX(cfg)                                                   \
	(((cfg) >> NXP_SIUL2_MSCR_SIUL2_IDX_SHIFT) & NXP_SIUL2_MSCR_SIUL2_IDX_MASK)

#define NXP_SIUL2_PINMUX_GET_IMCR_SIUL2_IDX(cfg)                                                   \
	(((cfg) >> NXP_SIUL2_IMCR_SIUL2_IDX_SHIFT) & NXP_SIUL2_IMCR_SIUL2_IDX_MASK)

/**
 * @brief Utility macro to build NXP S32 pinmux property for pinctrl nodes.
 *
 * @param mscr_siul2_idx MSCR SIUL2 instance index
 * @param imcr_siul2_idx IMCR SIUL2 instance index
 * @param mscr_idx Multiplexed Signal Configuration Register (MSCR) index
 * @param mscr_sss Output mux Source Signal Selection (MSCR.SSS)
 * @param imcr_idx Input Multiplexed Signal Configuration Register (IMCR) index
 * @param imcr_sss Input mux Source Signal Selection (IMCR.SSS)
 */
#define NXP_SIUL2_PINMUX(mscr_siul2_idx, imcr_siul2_idx, mscr_idx, mscr_sss, imcr_idx, imcr_sss)   \
	(NXP_SIUL2_PINMUX_MSCR_SIUL2_IDX(mscr_siul2_idx) |                                         \
	 NXP_SIUL2_PINMUX_IMCR_SIUL2_IDX(imcr_siul2_idx) |                                         \
	 NXP_SIUL2_PINMUX_MSCR_IDX(mscr_idx) |                                                     \
	 NXP_SIUL2_PINMUX_MSCR_SSS(mscr_sss) |                                                     \
	 NXP_SIUL2_PINMUX_IMCR_IDX(imcr_idx) |                                                     \
	 NXP_SIUL2_PINMUX_IMCR_SSS(imcr_sss))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NXP_SIUL2_PINCTRL_H_ */

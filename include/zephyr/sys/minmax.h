/*
 * Copyright (c) 2011-2014, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Lowercase min/max/clamp helpers.
 *
 * These short-named macros are kept out of @ref util.h so they are not pulled
 * in transitively by broad headers such as <pthread.h>. Source files that need
 * @ref min, @ref max, or @ref clamp should include this header explicitly.
 */

#ifndef ZEPHYR_INCLUDE_SYS_MINMAX_H_
#define ZEPHYR_INCLUDE_SYS_MINMAX_H_

#include <zephyr/sys/util.h>

#ifndef _ASMLANGUAGE

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @cond INTERNAL_HIDDEN
 */
#define _minmax_unique(op, a, b, ua, ub) ({ \
		__typeof__(a) ua = (a);     \
		__typeof__(b) ub = (b);     \
		op(ua, ub);                 \
	})

#define _minmax_cnt(op, a, b, cnt) \
	_minmax_unique(op, a, b, UTIL_CAT(_value_a_, cnt), UTIL_CAT(_value_b_, cnt))

#define _minmax3_unique(op, a, b, c, ua, ub, uc) ({ \
		__typeof__(a) ua = (a);             \
		__typeof__(b) ub = (b);             \
		__typeof__(c) uc = (c);             \
		op(ua, op(ub, uc));                 \
	})

#define _minmax3_cnt(op, a, b, c, cnt)            \
	_minmax3_unique(op, a, b, c,              \
			UTIL_CAT(_value_a_, cnt), \
			UTIL_CAT(_value_b_, cnt), \
			UTIL_CAT(_value_c_, cnt))
/**
 * @endcond
 */

#if !defined(__cplusplus) && !defined(_MSC_VER) /* #CUSTOM@NDRS */
/** @brief Return larger value of two provided expressions.
 *
 * Macro ensures that expressions are evaluated only once.
 *
 * @note Macro has limited usage compared to the standard macro as it cannot be
 *     used:
 *     - to generate constant integer, e.g. __aligned(max(4,5))
 *     - static variable, e.g. array like static uint8_t array[max(...)];
 */
#define z_max(a, b) _minmax_cnt(Z_INTERNAL_MAX, a, b, __COUNTER__)
#elif defined(_MSC_VER) /* #CUSTOM@NDRS */
#define z_max(a, b) Z_INTERNAL_MAX(a, b)
#endif

/** @brief Return larger value of three provided expressions.
 *
 * Macro ensures that expressions are evaluated only once. See @ref max for
 * macro limitations.
 */
#define max3(a, b, c) _minmax3_cnt(Z_INTERNAL_MAX, a, b, c, __COUNTER__)

#if !defined(__cplusplus) && !defined(_MSC_VER) /* #CUSTOM@NDRS */
/** @brief Return smaller value of two provided expressions.
 *
 * Macro ensures that expressions are evaluated only once. See @ref max for
 * macro limitations.
 */
#define z_min(a, b) _minmax_cnt(Z_INTERNAL_MIN, a, b, __COUNTER__)
#elif defined(_MSC_VER) /* #CUSTOM@NDRS */
#define z_min(a, b) Z_INTERNAL_MIN(a, b)
#endif

/** @brief Return smaller value of three provided expressions.
 *
 * Macro ensures that expressions are evaluated only once. See @ref max for
 * macro limitations.
 */
#define min3(a, b, c) _minmax3_cnt(Z_INTERNAL_MIN, a, b, c, __COUNTER__)

#if !defined(__cplusplus) && !defined(_MSC_VER) /* #CUSTOM@NDRS */
/** @brief Return a value clamped to a given range.
 *
 * Macro ensures that expressions are evaluated only once. See @ref max for
 * macro limitations.
 */
#define z_clamp(val, low, high) ({                                             \
		/* random suffix to avoid naming conflict */                   \
		__typeof__(val) _value_val_ = (val);                           \
		__typeof__(low) _value_low_ = (low);                           \
		__typeof__(high) _value_high_ = (high);                        \
		(_value_val_ < _value_low_)  ? _value_low_ :                   \
		(_value_val_ > _value_high_) ? _value_high_ :                  \
					       _value_val_;                    \
	})
#elif defined(_MSC_VER) /* #CUSTOM@NDRS */
/* Fall back to evaluating arguments multiple time due to MSVC limitations */
#define z_clamp(val, low, high) (((val) <= (low)) ? (low) : Z_INTERNAL_MIN(val, high))
#endif

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_SYS_MINMAX_H_ */

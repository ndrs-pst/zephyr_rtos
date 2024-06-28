/* ARM AArch32 inline assembler functions and macros for public functions */

/*
 * Copyright (c) 2015, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_H_

/*
 * The file must not be included directly
 * Include kernel.h instead
 */

#if defined(__GNUC__)
#include <zephyr/arch/arm/asm_inline_gcc.h>
#elif defined(_MSC_VER)                     /* #CUSTOM@NDRS */
/* @note Dummy function for __GTEST */
static ALWAYS_INLINE void arch_irq_unlock(unsigned int key) {
    (void) key;
}

/* @note Dummy function for __GTEST */
static ALWAYS_INLINE unsigned int arch_irq_lock(void) {
    return (0U);
}

#else
#include <arch/arm/asm_inline_other.h>
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_H_ */

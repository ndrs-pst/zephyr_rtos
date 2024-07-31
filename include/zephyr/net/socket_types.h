/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief socket types definitionis
 */

#ifndef ZEPHYR_INCLUDE_NET_SOCKET_TYPES_H_
#define ZEPHYR_INCLUDE_NET_SOCKET_TYPES_H_

/**
 * @brief BSD Sockets compatible API
 * @defgroup bsd_sockets BSD Sockets compatible API
 * @ingroup networking
 * @{
 */

#include <zephyr/types.h>

/** @cond INTERNAL_HIDDEN */

#ifdef CONFIG_NEWLIB_LIBC

#if defined(_MSC_VER)   /* #CUSTOM@NDRS */
typedef int32_t suseconds_t;
#else
#include <newlib.h>
#endif

#ifdef __NEWLIB__
#include <sys/_timeval.h>
#else /* __NEWLIB__ */
#include <sys/types.h>
#if !defined(_MSC_VER)  /* #CUSTOM@NDRS */
/* workaround for older Newlib 2.x, as it lacks sys/_timeval.h */
struct timeval {
	time_t tv_sec;
	suseconds_t tv_usec;
};
#endif
#endif /* __NEWLIB__ */

#else /* CONFIG_NEWLIB_LIBC */

#if defined(CONFIG_ARCH_POSIX) && defined(CONFIG_EXTERNAL_LIBC)
#include <bits/types/struct_timeval.h>
#else
#include <sys/_timeval.h>
#endif

#endif /* CONFIG_NEWLIB_LIBC */

#ifdef __cplusplus
extern "C" {
#endif

/* @see struct timeval */
struct zsock_timeval {
	time_t tv_sec;
	suseconds_t tv_usec;
};

#ifdef __cplusplus
}
#endif

/** @endcond */

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_SOCKET_TYPES_H_ */

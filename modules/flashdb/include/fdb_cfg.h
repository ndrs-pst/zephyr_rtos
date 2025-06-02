/*
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief configuration file
 */

#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

/* using KVDB feature */
#if defined(CONFIG_FLASHDB_KEYVALUE)
#define FDB_USING_KVDB

#ifdef FDB_USING_KVDB
/* Auto update KV to latest default when current KVDB version number is changed.
 * @see fdb_kvdb.ver_num */
/* #define FDB_KV_AUTO_UPDATE */
#endif

#endif /* CONFIG_FLASHDB_KEYVALUE */

#if defined(CONFIG_FLASHDB_TIMESERIES)

/* using TSDB (Time series database) feature */
#define FDB_USING_TSDB

#endif /* CONFIG_FLASHDB_TIMESERIES */

#define FDB_USING_ZEPHYR_FLASH_MAP          1

#if defined(CONFIG_FLASHDB_DEBUG_ENABLE)
/* log print macro. default EF_PRINT macro is printf() */
#if defined(_MSC_VER)
#define FDB_PRINT           printf
#else
#define FDB_PRINT           LOG_INF
#endif

/**
 * Flash write granularity in bits.
 *
 * Defines the minimum writable unit (1 → bit, 8 → byte, 32 → word, etc.).
 * Must match flash hardware capabilities; incorrect value may cause data errors.
 * Check the flash datasheet for proper setting.
 */
#define FDB_WRITE_GRAN      8               // Example: 32-bit granularity for MCU internal flash

/* print debug information */
#define FDB_DEBUG_ENABLE

#endif /* CONFIG_FLASHDB_DEBUG_ENABLE */

#endif /* _FDB_CFG_H_ */

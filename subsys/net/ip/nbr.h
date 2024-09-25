/** @file
 *  @brief IPv6 neighbor management.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __NET_NBR_H
#define __NET_NBR_H

#include <stddef.h>
#include <zephyr/types.h>
#include <stdbool.h>

#include <zephyr/net/net_if.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NET_NBR_LLADDR_UNKNOWN 0xff

/* The neighbors are tracked by link layer address. This is not part
 * of struct net_nbr because this data can be shared between different
 * neighboring tables.
 */
struct net_nbr_lladdr {
	/** Link layer address */
	struct net_linkaddr_storage lladdr;

	/** Reference count. */
	uint8_t ref;
};

#define NET_NBR_LLADDR_INIT(_name, _count)	\
	struct net_nbr_lladdr _name[_count] = { }

/* Alignment needed for various parts of the neighbor definition */
#define __net_nbr_align __aligned(sizeof(int))

/* The neighbor node generic data. There can be sub-system specific
 * data at the end of the node.
 */
struct net_nbr {
	/** Reference count. */
	uint8_t ref;

	/** Link to ll address. This is the index into lladdr array.
	 * The value NET_NBR_LLADDR_UNKNOWN tells that this neighbor
	 * does not yet have lladdr linked to it.
	 */
	uint8_t idx;

	/** Amount of data that this neighbor buffer can store. */
	#if defined(_MSC_VER) /* #CUSTOM@NDRS */
	uint16_t size;
	#else
	const uint16_t size;
	#endif

	/** Interface this neighbor is found */
	struct net_if *iface;

	/** Pointer to the start of data in the neighbor table. */
	uint8_t *data;

	/** Function to be called when the neighbor is removed. */
	#if defined(_MSC_VER) /* #CUSTOM@NDRS */
	void (* remove)(struct net_nbr *nbr);
	#else
	void (*const remove)(struct net_nbr *nbr);
	#endif

	/** Start of the data storage. Not to be accessed directly
	 *  (the data pointer should be used instead).
	 */
	#if defined(_MSC_VER) /* #CUSTOM@NDRS */
	uint8_t __nbr[1] __net_nbr_align;
	#else
	uint8_t __nbr[0] __net_nbr_align;
	#endif
};

/* This is an array of struct net_nbr + some additional data */
#if defined(_MSC_VER) /* #CUSTOM@NDRS */
#define NET_NBR_POOL_INIT(_name, _count, _size, _remove)		\
	struct _name##_t {						\
		struct net_nbr nbr;					\
		uint8_t data[ROUND_UP(_size, 4)] __net_nbr_align;	\
	} _name[_count];						\
	void _name##_init(struct _name##_t* obj, uint16_t count, uint16_t sz, void (* fp)(struct net_nbr *nbr)) { \
		for (int i = 0; i < count; i++) {			\
			obj[i].nbr.idx    = NET_NBR_LLADDR_UNKNOWN;	\
			obj[i].nbr.remove = fp;				\
			obj[i].nbr.size   = ROUND_UP(sz, 4);		\
		}							\
	}
#else
#define NET_NBR_POOL_INIT(_name, _count, _size, _remove)		\
	struct {							\
		struct net_nbr nbr;					\
		uint8_t data[ROUND_UP(_size, 4)] __net_nbr_align;	\
	} _name[_count] = {						\
		[0 ... (_count - 1)] = { .nbr = {			\
			.idx = NET_NBR_LLADDR_UNKNOWN,			\
			.remove = _remove,				\
			.size = ROUND_UP(_size, 4) } },			\
	}
#endif

struct net_nbr_table {
	/** Link to a neighbor pool */
	struct net_nbr *nbr;

	/** Function to be called when the table is cleared. */
	#if defined(_MSC_VER) /* #CUSTOM@NDRS */
	void (* clear)(struct net_nbr_table *table);
	#else
	void (*const clear)(struct net_nbr_table *table);
	#endif

	/** Max number of neighbors in the pool */
	#if defined(_MSC_VER) /* #CUSTOM@NDRS */
	uint16_t nbr_count;
	#else
	const uint16_t nbr_count;
	#endif
};

#define NET_NBR_LOCAL static
#define NET_NBR_GLOBAL

/* Type of the table can be NET_NBR_LOCAL or NET_NBR_GLOBAL
 */
#define NET_NBR_TABLE_INIT(_type, _name, _pool, _clear)			\
	_type struct net_nbr_table_##_name {				\
		struct net_nbr_table table;				\
	} net_##_name __used = {					\
		.table = {						\
			.clear = _clear,				\
			.nbr = (struct net_nbr *)_pool,			\
			.nbr_count = ARRAY_SIZE(_pool),			\
		}							\
	}

/**
 * @brief Decrement the reference count. If count goes to 0, the neighbor
 * is released and returned to free list.
 * @param nbr Pointer to neighbor
 */
#if defined(CONFIG_NET_IPV6_NBR_CACHE_LOG_LEVEL_DBG)
void net_nbr_unref_debug(struct net_nbr *nbr, const char *caller, int line);
#define net_nbr_unref(nbr) net_nbr_unref_debug(nbr, __func__, __LINE__)
#else
void net_nbr_unref(struct net_nbr *nbr);
#endif

/**
 * @brief Increment the reference count.
 * @param nbr Pointer to neighbor
 * @return Pointer to neighbor
 */
#if defined(CONFIG_NET_IPV6_NBR_CACHE_LOG_LEVEL_DBG)
struct net_nbr *net_nbr_ref_debug(struct net_nbr *nbr, const char *caller,
				  int line);
#define net_nbr_ref(nbr) net_nbr_ref_debug(nbr, __func__, __LINE__)
#else
struct net_nbr *net_nbr_ref(struct net_nbr *nbr);
#endif

/**
 * @brief Get a free neighbor from specific table.
 * @param table Neighbor table
 * @return Pointer to neighbor, NULL if no free neighbors
 */
struct net_nbr *net_nbr_get(struct net_nbr_table *table);

/**
 * @brief Find a neighbor from specific table.
 * @param table Neighbor table
 * @param iface Network interface to use
 * @param lladdr Neighbor link layer address
 * @return Pointer to neighbor, NULL if not found
 */
struct net_nbr *net_nbr_lookup(struct net_nbr_table *table,
			       struct net_if *iface,
			       struct net_linkaddr *lladdr);

/**
 * @brief Link a neighbor to specific link layer address.
 * @param table Neighbor table
 * @param iface Network interface to use
 * @param lladdr Neighbor link layer address
 * @return 0 if ok, <0 if linking failed
 */
int net_nbr_link(struct net_nbr *nbr, struct net_if *iface,
		 const struct net_linkaddr *lladdr);

/**
 * @brief Unlink a neighbor from specific link layer address.
 * @param table Neighbor table
 * @param lladdr Neighbor link layer address
 * @return 0 if ok, <0 if unlinking failed
 */
int net_nbr_unlink(struct net_nbr *nbr, struct net_linkaddr *lladdr);

/**
 * @brief Return link address for a specific lladdr table index
 * @param idx Link layer address index in ll table.
 * @return Pointer to link layer address storage, NULL if not found
 */
#if defined(CONFIG_NET_NATIVE)
struct net_linkaddr_storage *net_nbr_get_lladdr(uint8_t idx);
#else
static inline struct net_linkaddr_storage *net_nbr_get_lladdr(uint8_t idx)
{
	ARG_UNUSED(idx);

	return NULL;
}
#endif

/**
 * @brief Clear table from all neighbors. After this the linking between
 * lladdr and neighbor is removed.
 * @param table Neighbor table
 */
void net_nbr_clear_table(struct net_nbr_table *table);

/**
 * @brief Debug helper to print out the neighbor information.
 * @param table Neighbor table
 */
void net_nbr_print(struct net_nbr_table *table);

#ifdef __cplusplus
}
#endif

#endif /* __NET_NBR_H */

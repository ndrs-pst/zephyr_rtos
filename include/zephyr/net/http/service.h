/*
 * Copyright (c) 2022 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_HTTP_SERVICE_H_
#define ZEPHYR_INCLUDE_NET_HTTP_SERVICE_H_

/**
 * @file service.h
 *
 * @brief HTTP service API
 *
 * @defgroup http_service HTTP service API
 * @since 3.4
 * @version 0.1.0
 * @ingroup networking
 * @{
 */

#include "zephyr/net/http/server.h"
#include <stdint.h>
#include <stddef.h>

#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/net/tls_credentials.h>

#ifdef __cplusplus
extern "C" {
#endif

/** HTTP resource description */
struct http_resource_desc {
	/** Resource name */
	const char *resource;
	/** Detail associated with this resource */
	void *detail;
};

/**
 * @brief Define a static HTTP resource
 *
 * A static HTTP resource is one that is known prior to system initialization. In contrast,
 * dynamic resources may be discovered upon system initialization. Dynamic resources may also be
 * inserted, or removed by events originating internally or externally to the system at runtime.
 *
 * @note The @p _resource is the URL without the associated protocol, host, or URL parameters. E.g.
 * the resource for `http://www.foo.com/bar/baz.html#param1=value1` would be `/bar/baz.html`. It
 * is often referred to as the "path" of the URL. Every `(service, resource)` pair should be
 * unique. The @p _resource must be non-NULL.
 *
 * @param _name Name of the resource.
 * @param _service Name of the associated service.
 * @param _resource Pathname-like string identifying the resource.
 * @param _detail Implementation-specific detail associated with the resource.
 */
#define HTTP_RESOURCE_DEFINE(_name, _service, _resource, _detail)                                  \
	const STRUCT_SECTION_ITERABLE_ALTERNATE(http_resource_desc_##_service, http_resource_desc, \
						_name) = {                                         \
		.resource = _resource,                                                             \
		.detail = (void *)(_detail),                                                       \
	}

/** @cond INTERNAL_HIDDEN */

struct http_service_runtime_data {
	int num_clients;
};

struct http_service_desc;

/** Custom socket creation function type */
typedef int (*http_socket_create_fn)(const struct http_service_desc *svc, int af, int proto);

/** HTTP service configuration */
struct http_service_config {
	/** Custom socket creation for the service if needed */
	http_socket_create_fn socket_create;
	/* If any more service-specific configuration is needed, it can be added here. */
};

struct http_service_desc {
	const char *host;
	uint16_t *port;
	int *fd;
	void *detail;
	size_t concurrent;
	size_t backlog;
	struct http_service_runtime_data *data;
	struct http_resource_desc *res_begin;
	struct http_resource_desc *res_end;
	struct http_resource_detail *res_fallback;
	const struct http_service_config *config;
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	const sec_tag_t *sec_tag_list;
	size_t sec_tag_list_size;
#endif
};

#define __z_http_service_define(_name, _host, _port, _concurrent, _backlog, _detail,               \
				_res_fallback, _res_begin, _res_end, _config, ...)                 \
	BUILD_ASSERT(_concurrent <= CONFIG_HTTP_SERVER_MAX_CLIENTS,                                \
		     "can't accept more then MAX_CLIENTS");                                        \
	BUILD_ASSERT(_backlog > 0, "backlog can't be 0");                                          \
	static int _name##_fd = -1;                                                                \
	static struct http_service_runtime_data _name##_data = {0};                                \
	const STRUCT_SECTION_ITERABLE(http_service_desc, _name) = {                                \
		.host = _host,                                                                     \
		.port = (uint16_t *)(_port),                                                       \
		.fd = &_name##_fd,                                                                 \
		.detail = (void *)(_detail),                                                       \
		.concurrent = (_concurrent),                                                       \
		.backlog = (_backlog),                                                             \
		.data = &_name##_data,                                                             \
		.res_begin = (_res_begin),                                                         \
		.res_end = (_res_end),                                                             \
		.res_fallback = (_res_fallback),                                                   \
		.config = (_config),                                                               \
		COND_CODE_1(CONFIG_NET_SOCKETS_SOCKOPT_TLS,                                        \
			    (.sec_tag_list = COND_CODE_0(NUM_VA_ARGS_LESS_1(__VA_ARGS__), (NULL),  \
							 (GET_ARG_N(1, __VA_ARGS__))),), ())       \
		COND_CODE_1(CONFIG_NET_SOCKETS_SOCKOPT_TLS,                                        \
			    (.sec_tag_list_size = COND_CODE_0(NUM_VA_ARGS_LESS_1(__VA_ARGS__), (0),\
					     (GET_ARG_N(1, GET_ARGS_LESS_N(1, __VA_ARGS__))))), ())\
	}

/** @endcond */

/**
 * @brief Define an HTTP service without static resources.
 *
 * @note The @p _host parameter is used to specify an IP address either in
 * IPv4 or IPv6 format a fully-qualified hostname or a virtual host. If left NULL, the listening
 * port will listen on all addresses.
 *
 * @note The @p _port parameter must be non-`NULL`. It points to a location that specifies the port
 * number to use for the service. If the specified port number is zero, then an ephemeral port
 * number will be used and the actual port number assigned will be written back to memory. For
 * ephemeral port numbers, the memory pointed to by @p _port must be writeable.
 *
 * @param _name Name of the service.
 * @param _host IP address or hostname associated with the service.
 * @param[inout] _port Pointer to port associated with the service.
 * @param _concurrent Maximum number of concurrent clients. (max. CONFIG_HTTP_SERVER_MAX_CLIENTS)
 * @param _backlog Maximum number of queued connections. (min. 1)
 * @param _detail User-defined detail associated with the service.
 * @param _res_fallback Fallback resource to be served if no other resource matches path
 * @param _config Pointer to http_service_config structure (can be NULL for default behavior)
 */
#define HTTP_SERVICE_DEFINE_EMPTY(_name, _host, _port, _concurrent, _backlog, _detail,             \
				  _res_fallback, _config)                                          \
	__z_http_service_define(_name, _host, _port, _concurrent, _backlog, _detail,               \
				_res_fallback, NULL, NULL, _config)

/**
 * @brief Define an HTTPS service without static resources.
 *
 * @note The @p _host parameter is used to specify an IP address either in
 * IPv4 or IPv6 format a fully-qualified hostname or a virtual host. If left NULL, the listening
 * port will listen on all addresses.
 *
 * @note The @p _port parameter must be non-`NULL`. It points to a location that specifies the port
 * number to use for the service. If the specified port number is zero, then an ephemeral port
 * number will be used and the actual port number assigned will be written back to memory. For
 * ephemeral port numbers, the memory pointed to by @p _port must be writeable.
 *
 * @param _name Name of the service.
 * @param _host IP address or hostname associated with the service.
 * @param[inout] _port Pointer to port associated with the service.
 * @param _concurrent Maximum number of concurrent clients. (max. CONFIG_HTTP_SERVER_MAX_CLIENTS)
 * @param _backlog Maximum number of queued connections. (min. 1)
 * @param _detail User-defined detail associated with the service.
 * @param _res_fallback Fallback resource to be served if no other resource matches path
 * @param _config Pointer to http_service_config structure (can be NULL for default behavior)
 * @param _sec_tag_list TLS security tag list used to setup a HTTPS socket.
 * @param _sec_tag_list_size TLS security tag list size used to setup a HTTPS socket.
 */
#define HTTPS_SERVICE_DEFINE_EMPTY(_name, _host, _port, _concurrent, _backlog, _detail,          \
				   _res_fallback, _config, _sec_tag_list, _sec_tag_list_size)    \
	__z_http_service_define(_name, _host, _port, _concurrent, _backlog, _detail,             \
				_res_fallback, NULL, NULL, _config,                              \
				_sec_tag_list, _sec_tag_list_size);				 \
	BUILD_ASSERT(IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS),				 \
		     "TLS is required for HTTP secure (CONFIG_NET_SOCKETS_SOCKOPT_TLS)")

/**
 * @brief Define an HTTP service with static resources.
 *
 * @note The @p _host parameter is used to specify an IP address either in
 * IPv4 or IPv6 format a fully-qualified hostname or a virtual host. If left NULL, the listening
 * port will listen on all addresses.
 *
 * @note The @p _port parameter must be non-`NULL`. It points to a location that specifies the port
 * number to use for the service. If the specified port number is zero, then an ephemeral port
 * number will be used and the actual port number assigned will be written back to memory. For
 * ephemeral port numbers, the memory pointed to by @p _port must be writeable.
 *
 * @param _name Name of the service.
 * @param _host IP address or hostname associated with the service.
 * @param[inout] _port Pointer to port associated with the service.
 * @param _concurrent Maximum number of concurrent clients. (max. CONFIG_HTTP_SERVER_MAX_CLIENTS)
 * @param _backlog Maximum number of queued connections. (min. 1)
 * @param _detail User-defined detail associated with the service.
 * @param _res_fallback Fallback resource to be served if no other resource matches path
 * @param _config Pointer to http_service_config structure (can be NULL for default behavior)
 */
#define HTTP_SERVICE_DEFINE(_name, _host, _port, _concurrent, _backlog, _detail, _res_fallback,    \
			    _config)                                                               \
	extern struct http_resource_desc _CONCAT(_http_resource_desc_##_name, _list_start)[];      \
	extern struct http_resource_desc _CONCAT(_http_resource_desc_##_name, _list_end)[];        \
	__z_http_service_define(_name, _host, _port, _concurrent, _backlog, _detail,               \
				_res_fallback,                                                     \
				&_CONCAT(_http_resource_desc_##_name, _list_start)[0],             \
				&_CONCAT(_http_resource_desc_##_name, _list_end)[0], _config);

/**
 * @brief Define an HTTPS service with static resources.
 *
 * @note The @p _host parameter is used to specify an IP address either in
 * IPv4 or IPv6 format a fully-qualified hostname or a virtual host. If left NULL, the listening
 * port will listen on all addresses.
 *
 * @note The @p _port parameter must be non-`NULL`. It points to a location that specifies the port
 * number to use for the service. If the specified port number is zero, then an ephemeral port
 * number will be used and the actual port number assigned will be written back to memory. For
 * ephemeral port numbers, the memory pointed to by @p _port must be writeable.
 *
 * @param _name Name of the service.
 * @param _host IP address or hostname associated with the service.
 * @param[inout] _port Pointer to port associated with the service.
 * @param _concurrent Maximum number of concurrent clients. (max. CONFIG_HTTP_SERVER_MAX_CLIENTS)
 * @param _backlog Maximum number of queued connections. (min. 1)
 * @param _detail User-defined detail associated with the service.
 * @param _res_fallback Fallback resource to be served if no other resource matches path
 * @param _config Pointer to http_service_config structure (can be NULL for default behavior)
 * @param _sec_tag_list TLS security tag list used to setup a HTTPS socket.
 * @param _sec_tag_list_size TLS security tag list size used to setup a HTTPS socket.
 */
#define HTTPS_SERVICE_DEFINE(_name, _host, _port, _concurrent, _backlog, _detail,              \
			     _res_fallback, _config, _sec_tag_list, _sec_tag_list_size)        \
	extern struct http_resource_desc _CONCAT(_http_resource_desc_##_name, _list_start)[];  \
	extern struct http_resource_desc _CONCAT(_http_resource_desc_##_name, _list_end)[];    \
	__z_http_service_define(_name, _host, _port, _concurrent, _backlog, _detail,           \
				_res_fallback,                                                 \
				&_CONCAT(_http_resource_desc_##_name, _list_start)[0],         \
				&_CONCAT(_http_resource_desc_##_name, _list_end)[0], _config,  \
				_sec_tag_list, _sec_tag_list_size);                            \
	BUILD_ASSERT(IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS),                               \
		     "TLS is required for HTTP secure (CONFIG_NET_SOCKETS_SOCKOPT_TLS)")

/**
 * @brief Count the number of HTTP services.
 *
 * @param[out] _dst Pointer to location where result is written.
 */
#define HTTP_SERVICE_COUNT(_dst) STRUCT_SECTION_COUNT(http_service_desc, _dst)

/**
 * @brief Count HTTP service static resources.
 *
 * @param _service Pointer to a service.
 */
#define HTTP_SERVICE_RESOURCE_COUNT(_service) ((_service)->res_end - (_service)->res_begin)

/**
 * @brief Iterate over all HTTP services.
 *
 * @param _it Name of http_service_desc iterator
 */
#define HTTP_SERVICE_FOREACH(_it) STRUCT_SECTION_FOREACH(http_service_desc, _it)

/**
 * @brief Iterate over static HTTP resources associated with a given @p _service.
 *
 * @note This macro requires that @p _service is defined with @ref HTTP_SERVICE_DEFINE.
 *
 * @param _service Name of HTTP service
 * @param _it Name of iterator (of type @ref http_resource_desc)
 */
#define HTTP_RESOURCE_FOREACH(_service, _it)                                                       \
	STRUCT_SECTION_FOREACH_ALTERNATE(http_resource_desc_##_service, http_resource_desc, _it)

/**
 * @brief Iterate over all static resources associated with @p _service .
 *
 * @note This macro is suitable for a @p _service defined with either @ref HTTP_SERVICE_DEFINE
 * or @ref HTTP_SERVICE_DEFINE_EMPTY.
 *
 * @param _service Pointer to HTTP service
 * @param _it Name of iterator (of type @ref http_resource_desc)
 */
#define HTTP_SERVICE_FOREACH_RESOURCE(_service, _it)                                               \
	for (struct http_resource_desc *_it = (_service)->res_begin; ({                            \
		     __ASSERT(_it <= (_service)->res_end, "unexpected list end location");         \
		     _it < (_service)->res_end;                                                    \
	     });                                                                                   \
	     _it++)

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_HTTP_SERVICE_H_ */

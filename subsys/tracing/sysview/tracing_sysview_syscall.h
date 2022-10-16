/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_
#define ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_

#include <SEGGER_SYSVIEW.h>
#include <tracing_sysview_ids.h>

/* #CUSTOM@NDRS : provide function prototype for SEGGER_SYSVIEW_RecordU32(), SEGGER_SYSVIEW_RecordEndCallU32() */
extern void SEGGER_SYSVIEW_RecordU32(unsigned int EventId, U32 Para0);
extern void SEGGER_SYSVIEW_RecordEndCallU32(unsigned int EventID, U32 Para0);

#define sys_port_trace_syscall_enter(id, name, ...)		\
	SEGGER_SYSVIEW_RecordU32(TID_SYSCALL, (uint32_t)id)

#define sys_port_trace_syscall_exit(id, name, ...) \
	SEGGER_SYSVIEW_RecordEndCallU32(TID_SYSCALL, (uint32_t)id)

#endif /* ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_ */

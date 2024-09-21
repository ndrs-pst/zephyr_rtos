/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_
#define ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_

#include <SEGGER_SYSVIEW.h>
#include <tracing_sysview_ids.h>

/* #CUSTOM@NDRS : provide function prototype for SEGGER_SYSVIEW_RecordString(), SEGGER_SYSVIEW_RecordEndCall() */
extern void SEGGER_SYSVIEW_RecordString(unsigned int EventID, const char* pString);
extern void SEGGER_SYSVIEW_RecordEndCall(unsigned int EventID);

#define sys_port_trace_syscall_enter(id, name, ...) \
    SEGGER_SYSVIEW_RecordString(TID_SYSCALL, (const char*)#name)

#define sys_port_trace_syscall_exit(id, name, ...)  \
    SEGGER_SYSVIEW_RecordEndCall(TID_SYSCALL)

#endif /* ZEPHYR_TRACING_SYSVIEW_SYSCALL_H_ */

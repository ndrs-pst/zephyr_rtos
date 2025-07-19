/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#if CONFIG_SHELL_GETOPT
#include <zephyr/sys/iterable_sections.h>
#endif

#include "getopt.h"

/* Referring  below variables is not thread safe. They reflects getopt state
 * only when 1 thread is using getopt.
 * When more threads are using getopt please call getopt_state_get to know
 * getopt state for the current thread.
 */
int z_opterr = 1;           /* if error message should be printed */
int z_optind = 1;           /* index into parent argv vector */
int z_optopt;               /* character checked for validity */
int z_optreset;             /* reset getopt */
char *z_optarg;             /* argument associated with option */

/* Common state for all threads that did not have own getopt state. */
static struct z_getopt_state m_getopt_common_state = {
	.opterr = 1,
	.optind = 1,
	.optopt = 0,
	.optreset = 0,
	.optarg = NULL,

	.place = "", /* EMSG */

#if CONFIG_GETOPT_LONG
	.nonopt_start = -1, /* first non option argument (for permute) */
	.nonopt_end = -1, /* first option after non options (for permute) */
#endif
};

/* This function is not thread safe. All threads using getopt are calling
 * this function.
 */
void z_getopt_global_state_update(struct z_getopt_state *state)
{
	z_opterr = state->opterr;
	z_optind = state->optind;
	z_optopt = state->optopt;
	z_optreset = state->optreset;
	z_optarg = state->optarg;
}

/* It is internal getopt API function, it shall not be called by the user. */
struct z_getopt_state* z_getopt_state_get(void)
{
#if CONFIG_SHELL_GETOPT
	k_tid_t tid;

	tid = k_current_get();
	STRUCT_SECTION_FOREACH(shell, sh) {
		if (tid == sh->ctx->tid) {
			return &sh->ctx->getopt;
		}
	}
#endif
	/* If not a shell thread return a common pointer */
	return &m_getopt_common_state;
}

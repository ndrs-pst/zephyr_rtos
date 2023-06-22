# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_FRAM_SHELL fram_shell.c)

zephyr_library_sources_ifdef(CONFIG_FRAM_CY15X fram_cy15x.c)

# Copyright (c) 2021 Linaro Limited
# SPDX-License-Identifier: Apache-2.0

# Suppress "unique_unit_address_if_enabled" to handle the following overlaps:
# - /soc/pinmux@41000000 & /soc/gpio@41000000
# - /soc/pinmux@41000080 & /soc/gpio@41000080
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")

/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <Port_Ci_Port_Ip.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg) {
    (void) Port_Ci_Port_Ip_Init(pin_cnt, pins);

    return (0);
}


/*
 * Copyright (c) 2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Pin control driver for Infineon CAT1 MCU family.
 */

#include <zephyr/drivers/pinctrl.h>

#include <infineon_kconfig.h>
#include <cy_gpio.h>

#define GPIO_PORT_OR_NULL(node_id) \
    COND_CODE_1(DT_NODE_EXISTS(node_id), ((GPIO_PRT_Type*)DT_REG_ADDR(node_id)), (NULL))

/* On PSE84, the SMIF flash data/clock pins are routed through dedicated
 * GPIO ports inside the SMIF blocks rather than regular IOSS GPIO ports.
 * PDL's Cy_GPIO_Pin_*FastInit detects these base
 * addresses (CY_GPIO_IS_SMIF_GPIO) and dispatches HSIOM/CFG writes to the
 * SMIF GPIO sub-block, so they can be driven by the standard pinctrl
 * path.
 *
 * Slots IFX_SMIF{0,1}_PORT{0,1,2} in gpio_ports[] hold the SMIF port
 * bases on PSE84 and are absent on other SoCs (slots are unreferenced
 * there).
 */
#if defined(CONFIG_SOC_SERIES_PSE84)
#include <zephyr/dt-bindings/pinctrl/ifx_cat1-pinctrl.h>

#define IFX_PINCTRL_SMIF_PORT_ENTRIES                           \
    [IFX_SMIF0_PORT0] = (GPIO_PRT_Type*)SMIF_INST0_PRT0,        \
    [IFX_SMIF0_PORT1] = (GPIO_PRT_Type*)SMIF_INST0_PRT1,        \
    [IFX_SMIF0_PORT2] = (GPIO_PRT_Type*)SMIF_INST0_PRT2,        \
    [IFX_SMIF1_PORT0] = (GPIO_PRT_Type*)SMIF_INST1_PRT0,        \
    [IFX_SMIF1_PORT1] = (GPIO_PRT_Type*)SMIF_INST1_PRT1,        \
    [IFX_SMIF1_PORT2] = (GPIO_PRT_Type*)SMIF_INST1_PRT2,
#else
#define IFX_PINCTRL_SMIF_PORT_ENTRIES
#endif

/* @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
#if (__GTEST == 0) /* #CUSTOM@NDRS */
static GPIO_PRT_Type* const gpio_ports[] = {
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt0)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt1)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt2)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt3)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt4)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt5)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt6)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt7)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt8)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt9)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt10)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt11)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt12)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt13)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt14)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt15)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt16)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt17)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt18)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt19)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt20)),
    GPIO_PORT_OR_NULL(DT_NODELABEL(gpio_prt21)),
    IFX_PINCTRL_SMIF_PORT_ENTRIES
};
#else
static GPIO_PRT_Type* gpio_ports[22];
#endif

/* @brief This function returns gpio drive mode, according to.
 * bias and drive mode params defined in pinctrl node.
 *
 * @param flags - bias and drive mode flags from pinctrl node.
 */
static uint32_t soc_gpio_get_drv_mode(uint32_t gpio_flags) {
    bool input_enable = ((gpio_flags & SOC_GPIO_INPUTENABLE) != 0U);
    uint32_t drv_mode;

    #if defined(CY_IP_MXS22IOSS)
    /* CFGOUT3: peripheral-controlled OE with Hi-Z pull-up; takes priority over bias flags. */
    if ((gpio_flags & SOC_GPIO_CFGOUT3) != 0U) {
        return (CY_GPIO_DM_CFGOUT3_STRONG_PULLUP_HIGHZ);
    }
    #endif /* CY_IP_MXS22IOSS */

    if ((gpio_flags & SOC_GPIO_OPENDRAIN) != 0U) {
        drv_mode = CY_GPIO_DM_OD_DRIVESLOW_IN_OFF;
    }
    else if ((gpio_flags & SOC_GPIO_OPENSOURCE) != 0U) {
        drv_mode = CY_GPIO_DM_OD_DRIVESHIGH_IN_OFF;
    }
    else if ((gpio_flags & SOC_GPIO_PUSHPULL) != 0U) {
        drv_mode = CY_GPIO_DM_STRONG_IN_OFF;
    }
    else if ((gpio_flags & (SOC_GPIO_PULLUP | SOC_GPIO_PULLDOWN)) ==
                           (SOC_GPIO_PULLUP | SOC_GPIO_PULLDOWN)) {
        drv_mode = CY_GPIO_DM_PULLUP_DOWN_IN_OFF;
    }
    else if ((gpio_flags & SOC_GPIO_PULLUP) != 0U) {
        drv_mode = CY_GPIO_DM_PULLUP_IN_OFF;
    }
    else if ((gpio_flags & SOC_GPIO_PULLDOWN) != 0U) {
        drv_mode = CY_GPIO_DM_PULLDOWN_IN_OFF;
    }
    else {
        /* ANALOG (0x00) or HIGHZ (0x08): SOC_GPIO_HIGHZ forces input buffer on */
        input_enable = (input_enable || ((gpio_flags & SOC_GPIO_HIGHZ) != 0U));
        drv_mode = CY_GPIO_DM_ANALOG;
    }

    if (input_enable) {
        drv_mode |= 0x08U;
    }

    return (drv_mode);
}

#if defined(CY_IP_MXS22IOSS)
static uint32_t soc_gpio_get_drv_strength(uint32_t gpio_flags) {
    uint32_t drv_strength_idx = (gpio_flags & SOC_GPIO_DRIVESTRENGTH) >> SOC_GPIO_DRIVESTRENGTH_POS;

    /* DT enum index 0..7 (one-eighth -> full) maps linearly to SEL_0..SEL_7 */
    return (drv_strength_idx);
}
#endif

int pinctrl_configure_pins(const pinctrl_soc_pin_t* pins, uint8_t pin_cnt, uintptr_t reg) {
    ARG_UNUSED(reg);

    for (size_t i = 0U; i < pin_cnt; i++) {
        uint32_t gpio_flags = (pins[i].pincfg & SOC_GPIO_FLAGS_MASK);
        uint32_t drv_mode = soc_gpio_get_drv_mode(gpio_flags);
        uint32_t hsiom    = CAT1_PINMUX_GET_HSIOM_FUNC(pins[i].pinmux);
        uint32_t port_num = CAT1_PINMUX_GET_PORT_NUM(pins[i].pinmux);
        uint32_t pin_num  = CAT1_PINMUX_GET_PIN_NUM(pins[i].pinmux);

        /* Initialize pin */
        #if defined(CY_PDL_TZ_ENABLED)
        Cy_GPIO_Pin_SecFastInit(gpio_ports[port_num], pin_num, drv_mode, 1, hsiom);
        #else
        Cy_GPIO_Pin_FastInit(gpio_ports[port_num], pin_num, drv_mode, 1, hsiom);
        #endif /* defined(CY_PDL_TZ_ENABLED) */

        #if defined(CY_IP_MXS22IOSS)
        if (drv_mode == CY_GPIO_DM_CFGOUT3_STRONG_PULLUP_HIGHZ) {
            uint32_t pin_loc = pin_num << CY_GPIO_DRIVE_MODE_OFFSET;

            GPIO_PRT_CFG(gpio_ports[port_num]) |= (GPIO_PRT_CFG_IN_EN0_Msk << pin_loc);
        }
        #endif /* CY_IP_MXS22IOSS */

        /* Force output to enable pulls */
        switch (drv_mode) {
            case CY_GPIO_DM_PULLUP :
                Cy_GPIO_Write(gpio_ports[port_num], pin_num, 1);
                break;

            case CY_GPIO_DM_PULLDOWN :
                Cy_GPIO_Write(gpio_ports[port_num], pin_num, 0);
                break;

            default :
                /* Do nothing */
                break;
        }

        #if defined(CY_IP_MXS22IOSS)
        Cy_GPIO_SetDriveSel(gpio_ports[port_num], pin_num, soc_gpio_get_drv_strength(gpio_flags));

        /* CFGOUT3 internal pull-up: value from DT, 0 = disabled. */
        if (drv_mode == CY_GPIO_DM_CFGOUT3_STRONG_PULLUP_HIGHZ) {
            uint32_t pullup_val = (gpio_flags & SOC_GPIO_CFGOUT3_PULLUP_MASK) >> SOC_GPIO_CFGOUT3_PULLUP_POS;

            if (pullup_val != 0U) {
                Cy_GPIO_SetPullupResistance(gpio_ports[port_num], pin_num, pullup_val);
            }
        }
        #endif
    }

    return (0);
}

#if (__GTEST == 1) /* #CUSTOM@NDRS */
#include "mcu_reg_stub.h"

void zephyr_gtest_pinctrl_ifx(void) {
    for (size_t i = 0U; i < ARRAY_SIZE(gpio_ports); i++) {
        gpio_ports[i] = (GPIO_PRT_Type*)ut_mcu_gpio_ptr[i];
    }
}
#endif

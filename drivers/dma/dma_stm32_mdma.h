/*
 * Copyright (c) 2023 Jeroen van Dooren, Nobleo Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MDMA_STM32_H_
#define MDMA_STM32_H_

#include <soc.h>
#include <stm32_ll_mdma.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma/dma_stm32.h>

/* Maximum data sent in single transfer (Bytes) */
#define MDMA_STM32_MAX_DATA_ITEMS   0xFFFF

struct mdma_stm32_channel {
    uint32_t direction;
    bool source_periph;
    bool hal_override;
    volatile bool busy;
    uint32_t src_size;
    uint32_t dst_size;
    void* user_data;            /* holds the client data */
    dma_callback_t dma_callback;
    bool cyclic;
};

struct mdma_stm32_data {
    struct dma_context dma_ctx;
};

struct mdma_stm32_config {
    struct stm32_pclken pclken;
    void (*config_irq)(const struct device* dev);
    void* base;
    uint32_t max_channels;
    struct mdma_stm32_channel* channels;
};

uint32_t mdma_stm32_id_to_channel(uint32_t id);

typedef void (*mdma_stm32_clear_flag_func)(MDMA_TypeDef* DMAx);
typedef uint32_t (*mdma_stm32_check_flag_func)(MDMA_TypeDef* DMAx);

bool mdma_stm32_is_gi_active(MDMA_TypeDef* DMAx, uint32_t id);

void mdma_stm32_clear_tc(MDMA_TypeDef* DMAx, uint32_t id);
bool mdma_stm32_is_te_active(MDMA_TypeDef* DMAx, uint32_t id);
void mdma_stm32_clear_te(MDMA_TypeDef* DMAx, uint32_t id);

bool stm32_mdma_is_irq_active(MDMA_TypeDef* dma, uint32_t id);
bool stm32_mdma_is_tc_irq_active(MDMA_TypeDef* dma, uint32_t id);

void stm32_mdma_dump_channel_irq(MDMA_TypeDef* dma, uint32_t id);
void stm32_mdma_clear_channel_irq(MDMA_TypeDef* dma, uint32_t id);
void stm32_mdma_enable_channel(MDMA_TypeDef* dma, uint32_t id);
int  stm32_mdma_disable_channel(MDMA_TypeDef* dma, uint32_t id);

#endif /* MDMA_STM32_H_*/

/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This file is based on mbm_core.c from uC/Modbus Stack.
 *
 *                                uC/Modbus
 *                         The Embedded Modbus Stack
 *
 *      Copyright 2003-2020 Silicon Laboratories Inc. www.silabs.com
 *
 *                   SPDX-License-Identifier: Apache-2.0
 *
 * This software is subject to an open source license and is distributed by
 *  Silicon Laboratories Inc. pursuant to the terms of the Apache License,
 *      Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
 */

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include "modbus_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modbus_c, CONFIG_MODBUS_LOG_LEVEL);

static int mbc_validate_response_fc(struct modbus_context const* ctx,
                                    uint8_t const unit_id,
                                    uint8_t fc) {
    uint8_t resp_fc = ctx->rx_adu.fc;
    uint8_t excep_code = ctx->rx_adu.data[0];
    uint8_t const excep_bit  = BIT(7);
    uint8_t const excep_mask = BIT_MASK(7);

    if (unit_id != ctx->rx_adu.unit_id) {
        return (-EIO);
    }

    if (fc != (resp_fc & excep_mask)) {
        return (-EIO);
    }

    if (resp_fc & excep_bit) {
        if (excep_code > MODBUS_EXC_NONE) {
            return (excep_code);
        }

        return (-EIO);
    }

    return (0);
}

static int mbc_validate_fc03fp_response(struct modbus_context const* ctx, float* ptbl) {
    size_t resp_byte_cnt;
    size_t req_byte_cnt;
    uint16_t req_qty;
    uint8_t const* resp_data;

    resp_byte_cnt = ctx->rx_adu.data[0];
    resp_data     = &ctx->rx_adu.data[1];
    req_qty       = sys_get_be16(&ctx->tx_adu.data[2]);
    req_byte_cnt  = (req_qty * sizeof(uint16_t));

    if (req_byte_cnt != resp_byte_cnt) {
        LOG_ERR("Mismatch in the number of registers");
        return (-EINVAL);
    }

    for (uint16_t i = 0U; i < (req_qty / 2U); i++) {
        uint32_t reg_val = sys_get_be32(resp_data);

        memcpy(&ptbl[i], &reg_val, sizeof(float));
        resp_data += sizeof(uint32_t);
    }

    return (0);
}

static int mbc_validate_rd_response(struct modbus_context const* ctx,
                                    uint8_t const unit_id,
                                    uint8_t fc,
                                    uint8_t* data) {
    int err;
    size_t resp_byte_cnt;
    size_t req_byte_cnt;
    uint16_t req_qty;
    uint16_t req_addr;
    uint8_t const* resp_data;
    uint16_t* data_p16 = (uint16_t*)data;

    ARG_UNUSED(unit_id);

    if (data == NULL) {
        return (-EINVAL);
    }

    resp_byte_cnt = ctx->rx_adu.data[0];
    resp_data     = &ctx->rx_adu.data[1];
    req_qty       = sys_get_be16(&ctx->tx_adu.data[2]);
    req_addr      = sys_get_be16(&ctx->tx_adu.data[0]);

    if ((resp_byte_cnt + 1U) > sizeof(ctx->rx_adu.data)) {
        LOG_ERR("Byte count exceeds buffer length");
        return (-EINVAL);
    }

    switch (fc) {
        case MODBUS_FC01_COIL_RD :
        case MODBUS_FC02_DI_RD :
            req_byte_cnt = ((req_qty - 1U) / 8U) + 1U;
            if (req_byte_cnt != resp_byte_cnt) {
                LOG_ERR("Mismatch in the number of coils or inputs");
                err = -EINVAL;
            }
            else {
                for (uint8_t i = 0; i < resp_byte_cnt; i++) {
                    data[i] = resp_data[i];
                }

                err = 0;
            }
            break;

        case MODBUS_FC03_HOLDING_REG_RD :
            if (IS_ENABLED(CONFIG_MODBUS_FP_EXTENSIONS) &&
                (req_addr >= MODBUS_FP_EXTENSIONS_ADDR)) {
                err = mbc_validate_fc03fp_response(ctx, (float*)data);
                break;
            }
            __fallthrough;
        case MODBUS_FC04_IN_REG_RD :
            req_byte_cnt = req_qty * sizeof(uint16_t);
            if (req_byte_cnt != resp_byte_cnt) {
                LOG_ERR("Mismatch in the number of registers");
                err = -EINVAL;
            }
            else {
                for (uint16_t i = 0; i < req_qty; i++) {
                    data_p16[i] = sys_get_be16(resp_data);
                    resp_data += sizeof(uint16_t);
                }

                err = 0;
            }
            break;

        default :
            LOG_ERR("Validation not implemented for FC 0x%02x", fc);
            err = -ENOTSUP;
            break;
    }

    return (err);
}

static int mbc_validate_fc08_response(struct modbus_context const* ctx,
                                      uint8_t const unit_id,
                                      uint16_t* data) {
    int err;
    uint16_t resp_sfunc;
    uint16_t resp_data;
    uint16_t req_sfunc;
    uint16_t req_data;

    ARG_UNUSED(unit_id);

    if (data == NULL) {
        return (-EINVAL);
    }

    req_sfunc  = sys_get_be16(&ctx->tx_adu.data[0]);
    req_data   = sys_get_be16(&ctx->tx_adu.data[2]);
    resp_sfunc = sys_get_be16(&ctx->rx_adu.data[0]);
    resp_data  = sys_get_be16(&ctx->rx_adu.data[2]);

    if (req_sfunc != resp_sfunc) {
        LOG_ERR("Mismatch in the sub-function code");
        return (-EINVAL);
    }

    switch (resp_sfunc) {
        case MODBUS_FC08_SUBF_QUERY :
        case MODBUS_FC08_SUBF_CLR_CTR :
            if (req_data != resp_data) {
                LOG_ERR("Request and response data are different");
                err = -EINVAL;
            }
            else {
                *data = resp_data;
                err   = 0;
            }
            break;

        case MODBUS_FC08_SUBF_BUS_MSG_CTR :
        case MODBUS_FC08_SUBF_BUS_CRC_CTR :
        case MODBUS_FC08_SUBF_BUS_EXCEPT_CTR :
        case MODBUS_FC08_SUBF_SERVER_MSG_CTR :
        case MODBUS_FC08_SUBF_SERVER_NO_RESP_CTR :
            *data = resp_data;
            err   = 0;
            break;

        default :
            err = -EINVAL;
            break;
    }

    return (err);
}

static int mbc_validate_wr_response(struct modbus_context const* ctx,
                                    uint8_t const unit_id,
                                    uint8_t fc) {
    int err;
    uint16_t req_addr;
    uint16_t req_value;
    uint16_t resp_addr;
    uint16_t resp_value;

    ARG_UNUSED(unit_id);

    req_addr   = sys_get_be16(&ctx->tx_adu.data[0]);
    req_value  = sys_get_be16(&ctx->tx_adu.data[2]);
    resp_addr  = sys_get_be16(&ctx->rx_adu.data[0]);
    resp_value = sys_get_be16(&ctx->rx_adu.data[2]);

    switch (fc) {
        case MODBUS_FC05_COIL_WR :
        case MODBUS_FC06_HOLDING_REG_WR :
        case MODBUS_FC15_COILS_WR :
        case MODBUS_FC16_HOLDING_REGS_WR :
            if (req_addr != resp_addr || req_value != resp_value) {
                err = ENXIO;
            }
            else {
                err = 0;
            }
            break;

        default :
            LOG_ERR("Validation not implemented for FC 0x%02x", fc);
            err = -ENOTSUP;
            break;
    }

    return (err);
}

static int mbc_send_cmd(struct modbus_context* ctx, uint8_t const unit_id,
                        uint8_t fc, void* data) {
    int err;

    ctx->tx_adu.unit_id = unit_id;
    ctx->tx_adu.fc = fc;

    err = modbus_tx_wait_rx_adu(ctx);
    if (err != 0) {
        return (err);
    }

    err = mbc_validate_response_fc(ctx, unit_id, fc);
    if (err < 0) {
        LOG_ERR("Failed to validate unit ID or function code");
        return (err);
    }
    else if (err > 0) {
        LOG_INF("Modbus FC %u, error code %u", fc, err);
        return (err);
    }

    switch (fc) {
        case MODBUS_FC01_COIL_RD :
        case MODBUS_FC02_DI_RD :
        case MODBUS_FC03_HOLDING_REG_RD :
        case MODBUS_FC04_IN_REG_RD :
            err = mbc_validate_rd_response(ctx, unit_id, fc, data);
            break;

        case MODBUS_FC08_DIAGNOSTICS :
            err = mbc_validate_fc08_response(ctx, unit_id, data);
            break;

        case MODBUS_FC05_COIL_WR :
        case MODBUS_FC06_HOLDING_REG_WR :
        case MODBUS_FC15_COILS_WR :
        case MODBUS_FC16_HOLDING_REGS_WR :
            err = mbc_validate_wr_response(ctx, unit_id, fc);
            break;

        default :
            LOG_ERR("FC 0x%02x not implemented", fc);
            err = -ENOTSUP;
            break;
    }

    return (err);
}

int modbus_read_coils(int const iface,
                      uint8_t const unit_id,
                      uint16_t const start_addr,
                      uint8_t* const coil_tbl,
                      uint16_t const num_coils) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(num_coils, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC01_COIL_RD, coil_tbl);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_read_dinputs(int const iface,
                        uint8_t const unit_id,
                        uint16_t const start_addr,
                        uint8_t* const di_tbl,
                        uint16_t const num_di) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(num_di, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC02_DI_RD, di_tbl);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_read_holding_regs(int const iface,
                             uint8_t const unit_id,
                             uint16_t const start_addr,
                             uint16_t* const reg_buf,
                             uint16_t const num_regs) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(num_regs, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC03_HOLDING_REG_RD, reg_buf);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

#ifdef CONFIG_MODBUS_FP_EXTENSIONS
int modbus_read_holding_regs_fp(int const iface,
                                uint8_t const unit_id,
                                uint16_t const start_addr,
                                float* const reg_buf,
                                uint16_t const num_regs) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    /* A 32-bit float is mapped to two 16-bit registers */
    sys_put_be16(num_regs * 2, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC03_HOLDING_REG_RD, reg_buf);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}
#endif

int modbus_read_input_regs(int const iface,
                           uint8_t const unit_id,
                           uint16_t const start_addr,
                           uint16_t* const reg_buf,
                           uint16_t const num_regs) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(num_regs, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC04_IN_REG_RD, reg_buf);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_write_coil(int const iface,
                      uint8_t const unit_id,
                      uint16_t const coil_addr,
                      bool const coil_state) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;
    uint16_t coil_val;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    if (coil_state == false) {
        coil_val = MODBUS_COIL_OFF_CODE;
    }
    else {
        coil_val = MODBUS_COIL_ON_CODE;
    }

    ctx->tx_adu.length = 4U;
    sys_put_be16(coil_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(coil_val, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC05_COIL_WR, NULL);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_write_holding_reg(int const iface,
                             uint8_t const unit_id,
                             uint16_t const start_addr,
                             uint16_t const reg_val) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    sys_put_be16(reg_val, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC06_HOLDING_REG_WR, NULL);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_request_diagnostic(int const iface,
                              uint8_t const unit_id,
                              uint16_t const sfunc,
                              uint16_t const data,
                              uint16_t* const data_out) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    ctx->tx_adu.length = 4U;
    sys_put_be16(sfunc, &ctx->tx_adu.data[0]);
    sys_put_be16(data, &ctx->tx_adu.data[2]);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC08_DIAGNOSTICS, data_out);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_write_coils(int const iface,
                       uint8_t const unit_id,
                       uint16_t const start_addr,
                       uint8_t const* const coil_tbl,
                       uint16_t const num_coils) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    size_t length = 0U;
    uint8_t* data_ptr;
    size_t num_bytes;
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    length += sizeof(start_addr);
    sys_put_be16(num_coils, &ctx->tx_adu.data[2]);
    length += sizeof(num_coils);

    num_bytes = (uint8_t)(((num_coils - 1U) / 8U) + 1U);
    ctx->tx_adu.data[4] = (uint8_t)num_bytes;
    length += (num_bytes + 1U);

    if (length > sizeof(ctx->tx_adu.data)) {
        LOG_ERR("Length of data buffer is not sufficient");
        k_mutex_unlock(&ctx->iface_lock);
        return (-ENOBUFS);
    }

    ctx->tx_adu.length = (uint16_t)length;
    data_ptr = &ctx->tx_adu.data[5];

    memcpy(data_ptr, coil_tbl, num_bytes);

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC15_COILS_WR, NULL);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

int modbus_write_holding_regs(int const iface,
                              uint8_t const unit_id,
                              uint16_t const start_addr,
                              uint16_t const* const reg_buf,
                              uint16_t const num_regs) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    size_t length = 0U;
    uint8_t* data_ptr;
    size_t num_bytes;
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    length += sizeof(start_addr);
    sys_put_be16(num_regs, &ctx->tx_adu.data[2]);
    length += sizeof(num_regs);

    num_bytes = (num_regs * sizeof(uint16_t));
    ctx->tx_adu.data[4] = (uint8_t)num_bytes;
    length += (num_bytes + 1U);

    if (length > sizeof(ctx->tx_adu.data)) {
        LOG_ERR("Length of data buffer is not sufficient");
        k_mutex_unlock(&ctx->iface_lock);
        return (-ENOBUFS);
    }

    ctx->tx_adu.length = (uint16_t)length;
    data_ptr = &ctx->tx_adu.data[5];

    for (uint16_t i = 0U; i < num_regs; i++) {
        sys_put_be16(reg_buf[i], data_ptr);
        data_ptr += sizeof(uint16_t);
    }

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC16_HOLDING_REGS_WR, NULL);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}

#ifdef CONFIG_MODBUS_FP_EXTENSIONS
int modbus_write_holding_regs_fp(int const iface,
                                 uint8_t const unit_id,
                                 uint16_t const start_addr,
                                 float const* const reg_buf,
                                 uint16_t const num_regs) {
    struct modbus_context* ctx = modbus_get_context((uint8_t const)iface);
    size_t length = 0U;
    uint8_t* data_ptr;
    size_t num_bytes;
    int err;

    if (ctx == NULL) {
        return (-ENODEV);
    }

    k_mutex_lock(&ctx->iface_lock, K_FOREVER);

    sys_put_be16(start_addr, &ctx->tx_adu.data[0]);
    length += sizeof(start_addr);
    /* A 32-bit float is mapped to two 16-bit registers */
    sys_put_be16(num_regs * 2, &ctx->tx_adu.data[2]);
    length += sizeof(num_regs);

    num_bytes = (num_regs * sizeof(float));
    ctx->tx_adu.data[4] = (uint8_t)num_bytes;
    length += (num_bytes + 1U);

    if (length > sizeof(ctx->tx_adu.data)) {
        LOG_ERR("Length of data buffer is not sufficient");
        k_mutex_unlock(&ctx->iface_lock);
        return (-ENOBUFS);
    }

    ctx->tx_adu.length = (uint8_t)length;
    data_ptr = &ctx->tx_adu.data[5];

    for (uint16_t i = 0U; i < num_regs; i++) {
        uint32_t reg_val;

        memcpy(&reg_val, &reg_buf[i], sizeof(reg_val));
        sys_put_be32(reg_val, data_ptr);
        data_ptr += sizeof(uint32_t);
    }

    err = mbc_send_cmd(ctx, unit_id, MODBUS_FC16_HOLDING_REGS_WR, NULL);
    k_mutex_unlock(&ctx->iface_lock);

    return (err);
}
#endif

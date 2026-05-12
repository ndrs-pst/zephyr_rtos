/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include "shell_ops.h"

#define CMD_CURSOR_LEN 8

void z_shell_op_cursor_vert_move(struct shell const* sh, int32_t delta) {
    char dir;

    dir = (delta > 0) ? 'A' : 'B';
    if (delta == 0) {
        return;
    }

    if (delta < 0) {
        delta = -delta;
    }

    Z_SHELL_VT100_CMD(sh, "\e[%d%c", delta, dir);
}

void z_shell_op_cursor_horiz_move(struct shell const* sh, int32_t delta) {
    char dir;

    dir = (delta > 0) ? 'C' : 'D';
    if (delta == 0) {
        return;
    }

    if (delta < 0) {
        delta = -delta;
    }

    Z_SHELL_VT100_CMD(sh, "\e[%d%c", delta, dir);
}

/* Function returns true if command length is equal to multiplicity of terminal
 * width.
 */
static inline bool full_line_cmd(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;
    size_t line_length = ctx->cmd_buff_len + z_shell_strlen(ctx->prompt);

    if (line_length == 0) {
        return (false);
    }

    return ((line_length % ctx->vt100_ctx.cons.terminal_wid) == 0U);
}

/* Function returns true if cursor is at beginning of an empty line. */
bool z_shell_cursor_in_empty_line(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;

    // Calculate the total length of the input and prompt, considering if echo is enabled
    size_t total_length = (ctx->cmd_buff_pos * ctx->cfg.flags.echo) + z_shell_strlen(ctx->prompt);

    // Determine if the total length modulo the terminal width equals zero
    bool is_in_empty_line = ((total_length % ctx->vt100_ctx.cons.terminal_wid) == 0U);

    return (is_in_empty_line);
}

void z_shell_op_cond_next_line(struct shell const* sh) {
    if (z_shell_cursor_in_empty_line(sh) || full_line_cmd(sh)) {
        z_cursor_next_line_move(sh);
    }
}

void z_shell_op_cursor_position_synchronize(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;
    struct shell_multiline_cons* cons = &ctx->vt100_ctx.cons;
    bool last_line;

    z_shell_multiline_data_calc(cons, ctx->cmd_buff_pos,
                                ctx->cmd_buff_len);
    last_line = (cons->cur_y == cons->cur_y_end);

    /* In case cursor reaches the bottom line of a terminal, it will
     * be moved to the next line.
     */
    if (full_line_cmd(sh)) {
        z_cursor_next_line_move(sh);
    }

    if (last_line) {
        z_shell_op_cursor_horiz_move(sh, cons->cur_x - cons->cur_x_end);
    }
    else {
        z_shell_op_cursor_vert_move(sh, cons->cur_y_end - cons->cur_y);
        z_shell_op_cursor_horiz_move(sh, cons->cur_x - cons->cur_x_end);
    }
}

void z_shell_op_cursor_move(struct shell const* sh, int val) {
    struct shell_ctx* ctx = sh->ctx;
    struct shell_multiline_cons* cons = &ctx->vt100_ctx.cons;
    uint16_t new_pos = (ctx->cmd_buff_pos + val);
    int32_t row_span;
    int32_t col_span;

    z_shell_multiline_data_calc(cons, ctx->cmd_buff_pos,
                                ctx->cmd_buff_len);

    /* Calculate the new cursor. */
    row_span = z_row_span_with_buffer_offsets_get(&ctx->vt100_ctx.cons,
                                                  ctx->cmd_buff_pos,
                                                  new_pos);
    col_span = z_column_span_with_buffer_offsets_get(&ctx->vt100_ctx.cons,
                                                     ctx->cmd_buff_pos,
                                                     new_pos);

    z_shell_op_cursor_vert_move(sh, -row_span);
    z_shell_op_cursor_horiz_move(sh, col_span);
    ctx->cmd_buff_pos = new_pos;
}

static int shift_calc(char const* str, uint16_t pos, uint16_t len, int16_t sign) {
    bool found = false;
    int ret = 0;
    int idx;

    while (true) {
        idx = pos + (ret * sign);
        if (((idx == 0) && (sign < 0)) ||
            ((idx == len) && (sign > 0))) {
            break;
        }

        if (isalnum((int)str[idx]) != 0) {
            found = true;
        }
        else {
            if (found) {
                break;
            }
        }
        ret++;
    }

    return (ret);
}

void z_shell_op_cursor_word_move(struct shell const* sh, int val) {
    int shift;
    int sign;

    if (val < 0) {
        val  = -val;
        sign = -1;
    }
    else {
        sign = 1;
    }

    struct shell_ctx* ctx = sh->ctx;
    while (val--) {
        shift = shift_calc(ctx->cmd_buff,
                           ctx->cmd_buff_pos,
                           ctx->cmd_buff_len, sign);
        z_shell_op_cursor_move(sh, (sign * shift));
    }
}

void z_shell_op_word_remove(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;

    /* Line must not be empty and cursor must not be at 0 to continue. */
    if ((ctx->cmd_buff_len == 0) ||
        (ctx->cmd_buff_pos == 0)) {
        return;
    }

    char* str = &ctx->cmd_buff[ctx->cmd_buff_pos - 1];
    char* str_start = &ctx->cmd_buff[0];
    int chars_to_delete;

    /* Start at the current position. */
    chars_to_delete = 0;

    /* Look back for all spaces then for non-spaces. */
    while ((str >= str_start) && (*str == ' ')) {
        ++chars_to_delete;
        --str;
    }

    while ((str >= str_start) && (*str != ' ')) {
        ++chars_to_delete;
        --str;
    }

    /* Manage the buffer. */
    memmove((str + 1), (str + 1 + chars_to_delete),
            ctx->cmd_buff_len - chars_to_delete);
    ctx->cmd_buff_len -= chars_to_delete;
    ctx->cmd_buff[ctx->cmd_buff_len] = '\0';

    /* Update display. */
    z_shell_op_cursor_move(sh, -chars_to_delete);
    z_cursor_save(sh);
    z_shell_fprintf(sh, SHELL_NORMAL, "%s", str + 1);
    z_clear_eos(sh);
    z_cursor_restore(sh);
}

void z_shell_op_cursor_home_move(struct shell const* sh) {
    z_shell_op_cursor_move(sh, (-1 * sh->ctx->cmd_buff_pos));
}

void z_shell_op_cursor_end_move(struct shell const* sh) {
    z_shell_op_cursor_move(sh, (int)(sh->ctx->cmd_buff_len - sh->ctx->cmd_buff_pos));
}

void z_shell_op_left_arrow(struct shell const* sh) {
    if (sh->ctx->cmd_buff_pos > 0) {
        z_shell_op_cursor_move(sh, -1);
    }
}

void z_shell_op_right_arrow(struct shell const* sh) {
    if (sh->ctx->cmd_buff_pos < sh->ctx->cmd_buff_len) {
        z_shell_op_cursor_move(sh, 1);
    }
}

static void reprint_from_cursor(struct shell const* sh, int diff,
                                bool data_removed) {
    struct shell_ctx* ctx = sh->ctx;

    /* Clear eos is needed only when newly printed command is shorter than
     * previously printed command. This can happen when delete or backspace
     * was called.
     *
     * Such condition is useful for Bluetooth devices to save number of
     * bytes transmitted between terminal and device.
     */
    if (data_removed) {
        z_clear_eos(sh);
    }

    if (z_flag_obscure_get(sh)) {
        int len = strlen(&ctx->cmd_buff[ctx->cmd_buff_pos]);

        while (len--) {
            z_shell_raw_fprintf(sh->fprintf_ctx, "*");
        }
    }
    else {
        /* Check if the reprint will cross a line boundary */
        int line_len = ctx->cmd_buff_len + z_shell_strlen(ctx->prompt);
        int buff_pos = ctx->cmd_buff_pos + z_shell_strlen(ctx->prompt);

        if ((buff_pos / ctx->vt100_ctx.cons.terminal_wid) !=
            (line_len / ctx->vt100_ctx.cons.terminal_wid)) {
            /*
             * Reprint will take multiple lines.
             * Print each character directly.
             */
            int pos = ctx->cmd_buff_pos;

            while (buff_pos < line_len) {
                if ((buff_pos % ctx->vt100_ctx.cons.terminal_wid) == 0U) {
                    z_cursor_next_line_move(sh);
                }
                buff_pos++;

                z_shell_fprintf(sh, SHELL_NORMAL, "%c",
                                ctx->cmd_buff[pos++]);
            }
        }
        else {
            z_shell_fprintf(sh, SHELL_NORMAL, "%s",
                            &ctx->cmd_buff[ctx->cmd_buff_pos]);
        }
    }
    ctx->cmd_buff_pos = ctx->cmd_buff_len;

    if (full_line_cmd(sh)) {
        if (((data_removed) && (diff > 0)) || (!data_removed)) {
            z_cursor_next_line_move(sh);
        }
    }

    z_shell_op_cursor_move(sh, -diff);
}

static void data_insert(struct shell const* sh, char const* data, size_t len) {
    struct shell_ctx* ctx = sh->ctx;
    int after = (int)(ctx->cmd_buff_len - ctx->cmd_buff_pos);
    char* curr_pos = &ctx->cmd_buff[ctx->cmd_buff_pos];

    if ((ctx->cmd_buff_len + len) >= CONFIG_SHELL_CMD_BUFF_SIZE) {
        return;
    }

    memmove(curr_pos + len, curr_pos, after);
    memcpy(curr_pos, data, len);
    ctx->cmd_buff_len += len;
    ctx->cmd_buff[ctx->cmd_buff_len] = '\0';

    if (!z_flag_echo_get(sh)) {
        ctx->cmd_buff_pos += len;
        return;
    }

    reprint_from_cursor(sh, after, false);
}

static void char_replace(struct shell const* sh, char data) {
    struct shell_ctx* ctx = sh->ctx;

    ctx->cmd_buff[ctx->cmd_buff_pos++] = data;

    if (!z_flag_echo_get(sh)) {
        return;
    }

    if (z_flag_obscure_get(sh)) {
        data = '*';
    }

    z_shell_raw_fprintf(sh->fprintf_ctx, "%c", data);
    if (z_shell_cursor_in_empty_line(sh)) {
        z_cursor_next_line_move(sh);
    }
}

void z_shell_op_char_insert(struct shell const* sh, char data) {
    if (z_flag_insert_mode_get(sh) &&
        (sh->ctx->cmd_buff_len != sh->ctx->cmd_buff_pos)) {
        char_replace(sh, data);
    }
    else {
        data_insert(sh, &data, 1);
    }
}

void z_shell_op_char_backspace(struct shell const* sh) {
    if ((sh->ctx->cmd_buff_len == 0) ||
        (sh->ctx->cmd_buff_pos == 0)) {
        return;
    }

    z_shell_op_cursor_move(sh, -1);
    z_shell_op_char_delete(sh);
}

void z_shell_op_char_delete(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;
    int diff = (int)(ctx->cmd_buff_len - ctx->cmd_buff_pos);
    char* str = &ctx->cmd_buff[ctx->cmd_buff_pos];

    if (diff == 0) {
        return;
    }

    memmove(str, str + 1, diff);
    --ctx->cmd_buff_len;
    reprint_from_cursor(sh, --diff, true);
}

void z_shell_op_delete_from_cursor(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;

    ctx->cmd_buff_len = ctx->cmd_buff_pos;
    ctx->cmd_buff[ctx->cmd_buff_pos] = '\0';

    z_clear_eos(sh);
}

void z_shell_op_completion_insert(struct shell const* sh,
                                  char const* compl,
                                  size_t compl_len) {
    data_insert(sh, compl, compl_len);
}

void z_shell_cmd_line_erase(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;

    z_shell_multiline_data_calc(&ctx->vt100_ctx.cons,
                                ctx->cmd_buff_pos,
                                ctx->cmd_buff_len);
    z_shell_op_cursor_horiz_move(sh,
                                 -(ctx->vt100_ctx.cons.cur_x - 1));
    z_shell_op_cursor_vert_move(sh, (ctx->vt100_ctx.cons.cur_y - 1));

    z_clear_eos(sh);
}

static void print_prompt(struct shell const* sh) {
    if (sh->ctx->readline_state != SHELL_READLINE_INACTIVE) {
        return;
    }

    z_shell_fprintf(sh, SHELL_INFO, "%s", sh->ctx->prompt);
}

void z_shell_print_cmd(struct shell const* sh) {
    struct shell_ctx* ctx = sh->ctx;
    int beg_offset = 0;
    int end_offset = 0;
    int cmd_width  = z_shell_strlen(ctx->cmd_buff);
    int adjust = ctx->vt100_ctx.cons.name_len;
    char ch;

    while (cmd_width > ctx->vt100_ctx.cons.terminal_wid - adjust) {
        end_offset += ctx->vt100_ctx.cons.terminal_wid - adjust;
        ch = ctx->cmd_buff[end_offset];
        ctx->cmd_buff[end_offset] = '\0';

        z_shell_raw_fprintf(sh->fprintf_ctx, "%s\n",
                            &ctx->cmd_buff[beg_offset]);

        ctx->cmd_buff[end_offset] = ch;
        cmd_width -= (ctx->vt100_ctx.cons.terminal_wid - adjust);
        beg_offset = end_offset;
        adjust     = 0;
    }

    if (cmd_width > 0) {
        z_shell_raw_fprintf(sh->fprintf_ctx, "%s",
                            &ctx->cmd_buff[beg_offset]);
    }
}

void z_shell_print_prompt_and_cmd(struct shell const* sh) {
    print_prompt(sh);

    if (z_flag_echo_get(sh)) {
        z_shell_print_cmd(sh);
        z_shell_op_cursor_position_synchronize(sh);
    }
}

static void shell_pend_on_txdone(struct shell const* sh) {
    if (IS_ENABLED(CONFIG_MULTITHREADING) &&
        (sh->ctx->state < SHELL_STATE_PANIC_MODE_ACTIVE)) {
        k_event_wait(&sh->ctx->signal_event, SHELL_SIGNAL_TXDONE, false, K_FOREVER);
        k_event_clear(&sh->ctx->signal_event, SHELL_SIGNAL_TXDONE);
    }
    else {
        /* Blocking wait in case of bare metal. */
        while (!z_flag_tx_rdy_get(sh)) {
        }
        z_flag_tx_rdy_set(sh, false);
    }
}

void z_shell_write(struct shell const* sh, void const* data,
                   size_t length) {
    __ASSERT_NO_MSG(sh && data);

    size_t offset = 0;
    size_t tmp_cnt;

    while (length) {
        int err = sh->iface->api->write(sh->iface,
                        &((uint8_t const*)data)[offset], length,
                        &tmp_cnt);
        (void)err;
        __ASSERT_NO_MSG(err == 0);
        __ASSERT_NO_MSG(length >= tmp_cnt);
        if (err < 0) {
            break;
        }

        offset += tmp_cnt;
        length -= tmp_cnt;
        if ((tmp_cnt == 0) &&
            (sh->ctx->state != SHELL_STATE_PANIC_MODE_ACTIVE)) {
            shell_pend_on_txdone(sh);
        }
    }
}

/* Function shall be only used by the fprintf module. */
void z_shell_print_stream(void const* user_ctx, char const* data, size_t len) {
    z_shell_write((struct shell const*)user_ctx, data, len);
}

static void vt100_bgcolor_set(struct shell const* sh,
                              enum shell_vt100_color bgcolor) {
    if (!IS_ENABLED(CONFIG_SHELL_VT100_COLORS)) {
        return;
    }

    if (bgcolor >= VT100_COLOR_END) {
        return;
    }

    if ((bgcolor == SHELL_NORMAL) ||
        (sh->ctx->vt100_ctx.col.bgcol == bgcolor)) {
        return;
    }

    sh->ctx->vt100_ctx.col.bgcol = bgcolor;
    Z_SHELL_VT100_CMD(sh, "\e[403%dm", bgcolor);
}

void z_shell_vt100_color_set(struct shell const* sh,
                             enum shell_vt100_color color) {
    if (!IS_ENABLED(CONFIG_SHELL_VT100_COLORS)) {
        return;
    }

    if (color >= VT100_COLOR_END) {
        return;
    }

    if (sh->ctx->vt100_ctx.col.col == color) {
        return;
    }

    sh->ctx->vt100_ctx.col.col = color;

    if (color != SHELL_NORMAL) {
        Z_SHELL_VT100_CMD(sh, "\e[1;3%dm", color);
    }
    else {
        Z_SHELL_VT100_CMD(sh, SHELL_VT100_MODESOFF);
    }
}

void z_shell_vt100_colors_restore(struct shell const* sh,
                                  const struct shell_vt100_colors* color) {
    if (!IS_ENABLED(CONFIG_SHELL_VT100_COLORS)) {
        return;
    }

    z_shell_vt100_color_set(sh, color->col);
    vt100_bgcolor_set(sh, color->bgcol);
}

static void z_shell_print(struct shell const* sh, enum shell_vt100_color color, bool do_cbprintf,
                          void* ptr, va_list args) {
    if (IS_ENABLED(CONFIG_SHELL_VT100_COLORS) &&
        z_flag_use_colors_get(sh) &&
        (color != sh->ctx->vt100_ctx.col.col)) {
        struct shell_vt100_colors col;

        z_shell_vt100_colors_store(sh, &col);
        z_shell_vt100_color_set(sh, color);

        if (do_cbprintf) {
            z_shell_cbpprintf_fmt(sh->fprintf_ctx, ptr);
        }
        else {
            z_shell_fprintf_fmt(sh->fprintf_ctx, (const char*)ptr, args);
        }

        z_shell_vt100_colors_restore(sh, &col);
    }
    else {
        if (do_cbprintf) {
            z_shell_cbpprintf_fmt(sh->fprintf_ctx, ptr);
        }
        else {
            z_shell_fprintf_fmt(sh->fprintf_ctx, (const char*)ptr, args);
        }
    }
}

void z_shell_cbpprintf(struct shell const* sh, enum shell_vt100_color color, void* package) {
    va_list no_used = {0};

    z_shell_print(sh, color, true, package, no_used);
}

void z_shell_vfprintf(struct shell const* sh, enum shell_vt100_color color,
                      const char* fmt, va_list args) {
    z_shell_print(sh, color, false, (void*)fmt, args);
}

void z_shell_fprintf(struct shell const* sh,
                     enum shell_vt100_color color,
                     char const* fmt, ...) {
    __ASSERT_NO_MSG(sh);
    __ASSERT_NO_MSG(sh->ctx);
    __ASSERT_NO_MSG(sh->fprintf_ctx);
    __ASSERT_NO_MSG(fmt);
    __ASSERT(z_flag_sync_mode_get(sh) || !k_is_in_isr(),
             "Thread context required.");

    va_list args;

    va_start(args, fmt);
    z_shell_vfprintf(sh, color, fmt, args);
    va_end(args);
}

void z_shell_backend_rx_buffer_flush(const struct shell *sh) {
    __ASSERT_NO_MSG(sh);

    int32_t max_iterations = 1000;
    uint8_t buf[64];
    size_t count = 0;
    int err;

    do {
        err = sh->iface->api->read(sh->iface, buf, sizeof(buf), &count);
    } while (count != 0 && err == 0 && --max_iterations > 0);
}

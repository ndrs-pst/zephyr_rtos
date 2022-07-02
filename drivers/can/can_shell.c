/*
 * Copyright (c) 2019 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/can.h>
#include <zephyr/types.h>
#include <stdlib.h>

CAN_MSGQ_DEFINE(msgq, 4);
const struct shell* msgq_shell;
static struct k_work_poll  msgq_work;
static struct k_poll_event msgq_events[1] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &msgq, 0)
};

#define USE_MISRA_COMPLIANCE        1U

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_config_options(const struct shell* sh, int pos, char** argv,
                                      bool* listenonly, bool* loopback) {
    char* arg = argv[pos];
    int ret;

    if (arg[0] == '-') {
        ret = ++pos;

        for (arg = &arg[1]; *arg; arg++) {
            switch (*arg) {
                case 's' :
                    if (listenonly == NULL) {
                        shell_error(sh, "Unknown option %c", *arg);
                    }
                    else {
                        *listenonly = true;
                    }
                    break;

                case 'l' :
                    if (loopback == NULL) {
                        shell_error(sh, "Unknown option %c", *arg);
                    }
                    else {
                        *loopback = true;
                    }
                    break;

                default :
                    shell_error(sh, "Unknown option %c", *arg);
                    ret = -EINVAL;
                    break;
            }
        }
    }
    else {
        ret = pos;
    }

    return (ret);
}
#else
static inline int read_config_options(const struct shell* sh, int pos, char** argv,
                                      bool* listenonly, bool* loopback) {
    char* arg = argv[pos];

    if (arg[0] != '-') {
        return (pos);
    }

    for (arg = &arg[1]; *arg; arg++) {
        switch (*arg) {
            case 's' :
                if (listenonly == NULL) {
                    shell_error(sh, "Unknown option %c", *arg);
                }
                else {
                    *listenonly = true;
                }
                break;

            case 'l' :
                if (loopback == NULL) {
                    shell_error(sh, "Unknown option %c", *arg);
                }
                else {
                    *loopback = true;
                }
                break;

            default :
                shell_error(sh, "Unknown option %c", *arg);
                return (-EINVAL);
        }
    }

    return (++pos);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_frame_options(const struct shell* sh, int pos, char** argv,
                                     bool* rtr, bool* ext) {
    char* arg = argv[pos];
    int ret;

    if (arg[0] == '-') {
        ret = ++pos;

        for (arg = &arg[1]; *arg; arg++) {
            switch (*arg) {
                case 'r' :
                    if (rtr == NULL) {
                        shell_error(sh, "Unknown option %c", *arg);
                    }
                    else {
                        *rtr = true;
                    }
                    break;

                case 'e' :
                    if (ext == NULL) {
                        shell_error(sh, "Unknown option %c", *arg);
                    }
                    else {
                        *ext = true;
                    }
                    break;

                default :
                    shell_error(sh, "Unknown option %c", *arg);
                    ret = -EINVAL;
                    break;
            }
        }
    }
    else {
        ret = pos;
    }

    return (ret);
}
#else
static inline int read_frame_options(const struct shell* sh, int pos, char** argv,
                                     bool* rtr, bool* ext) {
    char* arg = argv[pos];

    if (arg[0] != '-') {
        return (pos);
    }

    for (arg = &arg[1]; *arg; arg++) {
        switch (*arg) {
            case 'r' :
                if (rtr == NULL) {
                    shell_error(sh, "Unknown option %c", *arg);
                }
                else {
                    *rtr = true;
                }
                break;

            case 'e' :
                if (ext == NULL) {
                    shell_error(sh, "Unknown option %c", *arg);
                }
                else {
                    *ext = true;
                }
                break;

            default :
                shell_error(sh, "Unknown option %c", *arg);
                return (-EINVAL);
        }
    }

    return (++pos);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_bitrate(const struct shell* sh, int pos, char** argv, uint32_t* bitrate) {
    char* end_ptr;
    long  val;
    int   ret;

    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr == '\0') {
        *bitrate = (uint32_t)val;

        ret = ++pos;
    }
    else {
        shell_error(sh, "Bitrate is not a number");
        ret = -EINVAL;
    }

    return (ret);
}
#else
static inline int read_bitrate(const struct shell* sh, int pos, char** argv, uint32_t* bitrate) {
    char* end_ptr;
    long val;

    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr != '\0') {
        shell_error(sh, "Bitrate is not a number");
        return (-EINVAL);
    }

    *bitrate = (uint32_t)val;

    return (++pos);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_id(const struct shell* sh, int pos, char** argv,
                          bool ext, uint32_t* id) {
    char* end_ptr;
    long  val;
    int   ret;

    ret = -EINVAL;
    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr == '\0') {
        if ((val < 0)               ||
            (val > CAN_EXT_ID_MASK) ||
            ((ext == false) && (val > CAN_MAX_STD_ID))) {
            shell_error(sh, "ID invalid. %sid must not be negative or " "bigger than 0x%x", ext ? "ext " : "",
                        ext ? CAN_EXT_ID_MASK : CAN_MAX_STD_ID);
        }
        else {
            *id = (uint32_t)val;
            ret = ++pos;                    // OK path
        }
    }
    else {
        shell_error(sh, "ID is not a number");
    }

    return (ret);
}
#else
static inline int read_id(const struct shell* sh, int pos, char** argv,
                          bool ext, uint32_t* id) {
    char* end_ptr;
    long val;

    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr != '\0') {
        shell_error(sh, "ID is not a number");
        return (-EINVAL);
    }

    if ((val < 0)               ||
        (val > CAN_EXT_ID_MASK) ||
        ((ext == false) && (val > CAN_MAX_STD_ID))) {
        shell_error(sh, "ID invalid. %sid must not be negative or " "bigger than 0x%x", ext ? "ext " : "",
                    ext ? CAN_EXT_ID_MASK : CAN_MAX_STD_ID);
        return (-EINVAL);
    }

    *id = (uint32_t)val;

    return (++pos);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_mask(const struct shell* sh, int pos, char** argv,
                            bool ext, uint32_t* mask) {
    char* end_ptr;
    long  val;
    int   ret;

    ret = -EINVAL;
    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr == '\0') {
        if ((val < 0)               ||
            (val > CAN_EXT_ID_MASK) ||
            ((ext == false) && (val > CAN_MAX_STD_ID))) {
            shell_error(sh, "Mask invalid. %smask must not be negative " "or bigger than 0x%x", ext ? "ext " : "",
                        ext ? CAN_EXT_ID_MASK : CAN_MAX_STD_ID);
        }
        else {
            *mask = (uint32_t)val;
            ret   = ++pos;                  // OK path
        }
    }
    else {
        shell_error(sh, "Mask is not a number");
    }

    return (ret);
}
#else
static inline int read_mask(const struct shell* sh, int pos, char** argv,
                            bool ext, uint32_t* mask) {
    char* end_ptr;
    long val;

    val = strtol(argv[pos], &end_ptr, 0);
    if (*end_ptr != '\0') {
        shell_error(sh, "Mask is not a number");
        return (-EINVAL);
    }

    if ((val < 0)               ||
        (val > CAN_EXT_ID_MASK) ||
        ((ext == false) && (val > CAN_MAX_STD_ID))) {
        shell_error(sh, "Mask invalid. %smask must not be negative " "or bigger than 0x%x", ext ? "ext " : "",
                    ext ? CAN_EXT_ID_MASK : CAN_MAX_STD_ID);
        return (-EINVAL);
    }

    *mask = (uint32_t)val;

    return ++pos;
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static inline int read_data(const struct shell* sh, int pos, char** argv, size_t argc,
                            uint8_t* data, uint8_t* dlc) {
    int i;
    int ret;
    uint8_t* data_ptr = data;

    ret = 0;
    if ((argc - pos) <= CAN_MAX_DLC) {
        for (i = pos; i < argc; i++) {
            char* end_ptr;
            long  val;

            val = strtol(argv[i], &end_ptr, 0);
            if (*end_ptr != '\0') {
                shell_error(sh, "Data bytes must be numbers");
                ret = -EINVAL;
                break;
            }

            if (val & ~0xFFL) {
                shell_error(sh, "A data bytes must not be > 0xFF");
                ret = -EINVAL;
                break;
            }

            *data_ptr = val;
            data_ptr++;
        }

        if (ret == 0) {                     // OK path
            *dlc = i - pos;

            ret = i;
        }
    }
    else {
        shell_error(sh, "Too many databytes. Max is %d", CAN_MAX_DLC);
        ret = -EINVAL;
    }

    return (ret);
}
#else
static inline int read_data(const struct shell* sh, int pos, char** argv, size_t argc,
                            uint8_t* data, uint8_t* dlc) {
    int i;
    uint8_t* data_ptr = data;

    if ((argc - pos) > CAN_MAX_DLC) {
        shell_error(sh, "Too many databytes. Max is %d", CAN_MAX_DLC);
        return (-EINVAL);
    }

    for (i = pos; i < argc; i++) {
        char* end_ptr;
        long val;

        val = strtol(argv[i], &end_ptr, 0);
        if (*end_ptr != '\0') {
            shell_error(sh, "Data bytes must be numbers");
            return (-EINVAL);
        }

        if (val & ~0xFFL) {
            shell_error(sh, "A data bytes must not be > 0xFF");
            return (-EINVAL);
        }

        *data_ptr = val;
        data_ptr++;
    }

    *dlc = i - pos;

    return i;
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static void print_frame(struct zcan_frame* frame, const struct shell* sh) {
    char const* frame_id;
    char const* frame_rtr;

    if (frame->id_type == CAN_STANDARD_IDENTIFIER) {
        frame_id = "std";
    }
    else {
        frame_id = "ext";
    }

    if (frame->rtr) {
        frame_rtr = "RTR";
    }
    else {
        frame_rtr = "   ";
    }

    shell_fprintf(sh, SHELL_NORMAL, "|0x%-8x|%s|%s|%d|", frame->id, frame_id, frame_rtr, frame->dlc);

    for (int i = 0; i < CAN_MAX_DLEN; i++) {
        if (i < frame->dlc) {
            shell_fprintf(sh, SHELL_NORMAL, " 0x%02x", frame->data[i]);
        }
        else {
            shell_fprintf(sh, SHELL_NORMAL, "     ");
        }
    }

    shell_fprintf(sh, SHELL_NORMAL, "|\n");
}
#else
static void print_frame(struct zcan_frame* frame, const struct shell* sh) {
    shell_fprintf(sh, SHELL_NORMAL, "|0x%-8x|%s|%s|%d|", frame->id,
                  frame->id_type == CAN_STANDARD_IDENTIFIER ? "std" : "ext", frame->rtr ? "RTR" : "   ", frame->dlc);

    for (int i = 0; i < CAN_MAX_DLEN; i++) {
        if (i < frame->dlc) {
            shell_fprintf(sh, SHELL_NORMAL, " 0x%02x", frame->data[i]);
        }
        else {
            shell_fprintf(sh, SHELL_NORMAL, "     ");
        }
    }

    shell_fprintf(sh, SHELL_NORMAL, "|\n");
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static void msgq_triggered_work_handler(struct k_work* work) {
    struct zcan_frame frame;
    int ret;

    while (true) {
        ret = k_msgq_get(&msgq, &frame, K_NO_WAIT);
        if (ret == 0) {
            print_frame(&frame, msgq_shell);
        }
        else {
            break;
        }
    }

    ret = k_work_poll_submit(&msgq_work, msgq_events, ARRAY_SIZE(msgq_events), K_FOREVER);
    if (ret != 0) {
        shell_error(msgq_shell, "Failed to resubmit msgq polling [%d]", ret);
    }
}
#else
static void msgq_triggered_work_handler(struct k_work* work) {
    struct zcan_frame frame;
    int ret;

    while (k_msgq_get(&msgq, &frame, K_NO_WAIT) == 0) {
        print_frame(&frame, msgq_shell);
    }

    ret = k_work_poll_submit(&msgq_work, msgq_events, ARRAY_SIZE(msgq_events), K_FOREVER);
    if (ret != 0) {
        shell_error(msgq_shell, "Failed to resubmit msgq polling [%d]", ret);
    }
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static int cmd_config(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int  pos;
    bool listenonly;
    bool loopback;
    can_mode_t mode;
    uint32_t bitrate;
    int ret;

    pos      = 1;
    listenonly = false;
    loopback = false;
    ret      = -EINVAL;

    can_dev = device_get_binding(argv[pos]);
    if (can_dev != NULL) {
        pos++;

        pos = read_config_options(sh, pos, argv, &listenonly, &loopback);
        if (pos >= 0) {
            mode = CAN_MODE_NORMAL;
            if (listenonly == true) {
                mode |= CAN_MODE_LISTENONLY;
            }

            if (loopback == true) {
                mode |= CAN_MODE_LOOPBACK;
            }

            ret = can_set_mode(can_dev, mode);
            if (ret == 0) {
                pos = read_bitrate(sh, pos, argv, &bitrate);
                if (pos >= 0) {
                    ret = can_set_bitrate(can_dev, bitrate);
                    if (ret == 0) {
                        // OK path
                    }
                    else {
                        shell_error(sh, "Failed to set bitrate [%d]", ret);
                    }
                }
            }
            else {
                shell_error(sh, "Failed to set mode [%d]", ret);
            }
        }
    }
    else {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
    }

    return (ret);
}
#else
static int cmd_config(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int pos = 1;
    bool listenonly = false, loopback = false;
    can_mode_t mode;
    uint32_t bitrate;
    int ret;

    can_dev = device_get_binding(argv[pos]);
    if (!can_dev) {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
        return (-EINVAL);
    }

    pos++;

    pos = read_config_options(sh, pos, argv, &listenonly, &loopback);
    if (pos < 0) {
        return (-EINVAL);
    }

    mode = CAN_MODE_NORMAL;
    if (listenonly == true) {
        mode |= CAN_MODE_LISTENONLY;
    }

    if (loopback == true) {
        mode |= CAN_MODE_LOOPBACK;
    }

    ret = can_set_mode(can_dev, mode);
    if (ret) {
        shell_error(sh, "Failed to set mode [%d]", ret);
        return ret;
    }

    pos = read_bitrate(sh, pos, argv, &bitrate);
    if (pos < 0) {
        return (-EINVAL);
    }

    ret = can_set_bitrate(can_dev, bitrate, 0);
    if (ret) {
        shell_error(sh, "Failed to set bitrate [%d]", ret);
        return ret;
    }

    return (0);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static int cmd_send(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int  pos;
    bool rtr;
    bool ext;
    struct zcan_frame frame;
    int ret;
    uint32_t id;

    pos = 1;
    rtr = false;
    ext = false;
    ret = -EINVAL;

    can_dev = device_get_binding(argv[pos]);
    if (can_dev != NULL) {
        pos++;

        pos = read_frame_options(sh, pos, argv, &rtr, &ext);
        if (pos >= 0) {
            if (ext == true) {
                frame.id_type = CAN_EXTENDED_IDENTIFIER;
            }
            else {
                frame.id_type = CAN_STANDARD_IDENTIFIER;
            }

            if (rtr == true) {
                frame.rtr = CAN_REMOTEREQUEST;
            }
            else {
                frame.rtr = CAN_DATAFRAME;
            }

            pos = read_id(sh, pos, argv, ext, &id);
            if (pos >= 0) {
                frame.id = id;

                pos = read_data(sh, pos, argv, argc, frame.data, &frame.dlc);
                if (pos >= 0) {
                    shell_print(sh, "Send frame with ID 0x%x (%s ID) and %d data bytes", frame.id, ext ? "extended" : "standard",
                                frame.dlc);

                    ret = can_send(can_dev, &frame, K_FOREVER, NULL, NULL);
                    if (ret == 0) {
                        // OK path
                    }
                    else {
                        shell_error(sh, "Failed to send frame [%d]", ret);
                        ret = -EIO;
                    }
                }
            }
        }
    }
    else {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
    }

    return (ret);
}
#else
static int cmd_send(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int pos = 1;
    bool rtr = false, ext = false;
    struct zcan_frame frame;
    int ret;
    uint32_t id;

    can_dev = device_get_binding(argv[pos]);
    if (!can_dev) {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
        return (-EINVAL);
    }

    pos++;

    pos = read_frame_options(sh, pos, argv, &rtr, &ext);
    if (pos < 0) {
        return (-EINVAL);
    }

    frame.id_type = ext ? CAN_EXTENDED_IDENTIFIER : CAN_STANDARD_IDENTIFIER;
    frame.rtr = rtr ? CAN_REMOTEREQUEST : CAN_DATAFRAME;

    pos = read_id(sh, pos, argv, ext, &id);
    if (pos < 0) {
        return (-EINVAL);
    }

    frame.id = id;

    pos = read_data(sh, pos, argv, argc, frame.data, &frame.dlc);
    if (pos < 0) {
        return (-EINVAL);
    }

    shell_print(sh, "Send frame with ID 0x%x (%s ID) and %d data bytes", frame.id, ext ? "extended" : "standard",
                frame.dlc);

    ret = can_send(can_dev, &frame, K_FOREVER, NULL, NULL);
    if (ret) {
        shell_error(sh, "Failed to send frame [%d]", ret);
        return -EIO;
    }

    return (0);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static int cmd_add_rx_filter(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int pos;
    bool rtr;
    bool ext;
    bool rtr_mask;
    struct zcan_filter filter;
    int ret;
    uint32_t id;
    uint32_t mask;

    pos = 1;
    rtr = false;
    ext = false;
    rtr_mask = false;
    ret = -EINVAL;

    can_dev = device_get_binding(argv[pos]);
    if (can_dev != NULL) {
        pos++;

        pos = read_frame_options(sh, pos, argv, &rtr, &ext);
        if (pos >= 0) {
            if (ext == true) {
                filter.id_type = CAN_EXTENDED_IDENTIFIER;
            }
            else {
                filter.id_type = CAN_STANDARD_IDENTIFIER;
            }

            if (rtr == true) {
                filter.rtr = CAN_REMOTEREQUEST;
            }
            else {
                filter.rtr = CAN_DATAFRAME;
            }

            pos = read_id(sh, pos, argv, ext, &id);
            if (pos >= 0) {
                filter.id = id;

                if (pos != argc) {
                    pos = read_mask(sh, pos, argv, ext, &mask);
                    filter.id_mask = mask;
                }
                else {
                    if (ext == true) {
                        filter.id_mask = CAN_EXT_ID_MASK;
                    }
                    else {
                        filter.id_mask = CAN_STD_ID_MASK;
                    }
                }

                if ((pos != argc) && (pos >= 0)) {
                    pos = read_frame_options(sh, pos, argv, &rtr_mask, NULL);
                }

                if (pos >= 0) {
                    filter.rtr_mask = rtr_mask;

                    shell_print(sh, "Add RX filter with ID 0x%x (%s ID), mask 0x%x, RTR %d", filter.id, ext ? "extended" : "standard",
                                filter.id_mask, filter.rtr_mask);

                    ret = can_add_rx_filter_msgq(can_dev, &msgq, &filter);
                    if (ret >= 0) {
                        shell_print(sh, "Filter ID: %d", ret);

                        if (msgq_shell == NULL) {
                            msgq_shell = sh;
                            k_work_poll_init(&msgq_work, msgq_triggered_work_handler);
                        }

                        ret = k_work_poll_submit(&msgq_work, msgq_events, ARRAY_SIZE(msgq_events), K_FOREVER);
                        if (ret == 0) {
                            // OK path
                        }
                        else {
                            shell_error(sh, "Failed to submit msgq polling [%d]", ret);
                        }
                    }
                    else {
                        if (ret == -ENOSPC) {
                            shell_error(sh, "Failed to add RX filter, no free filter left");
                        }
                        else {
                            shell_error(sh, "Failed to add RX filter [%d]", ret);
                        }

                        ret = -EIO;
                    }
                }
            }
        }
    }
    else {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
    }

    return (ret);
}
#else
static int cmd_add_rx_filter(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    int pos = 1;
    bool rtr = false, ext = false, rtr_mask = false;
    struct zcan_filter filter;
    int ret;
    uint32_t id, mask;

    can_dev = device_get_binding(argv[pos]);
    if (!can_dev) {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[pos]);
        return (-EINVAL);
    }

    pos++;

    pos = read_frame_options(sh, pos, argv, &rtr, &ext);
    if (pos < 0) {
        return (-EINVAL);
    }

    filter.id_type = ext ? CAN_EXTENDED_IDENTIFIER : CAN_STANDARD_IDENTIFIER;
    filter.rtr = rtr ? CAN_REMOTEREQUEST : CAN_DATAFRAME;

    pos = read_id(sh, pos, argv, ext, &id);
    if (pos < 0) {
        return (-EINVAL);
    }

    filter.id = id;

    if (pos != argc) {
        pos = read_mask(sh, pos, argv, ext, &mask);
        if (pos < 0) {
            return (-EINVAL);
        }
        filter.id_mask = mask;
    }
    else {
        filter.id_mask = ext ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK;
    }

    if (pos != argc) {
        pos = read_frame_options(sh, pos, argv, &rtr_mask, NULL);
        if (pos < 0) {
            return (-EINVAL);
        }
    }

    filter.rtr_mask = rtr_mask;

    shell_print(sh, "Add RX filter with ID 0x%x (%s ID), mask 0x%x, RTR %d", filter.id, ext ? "extended" : "standard",
                filter.id_mask, filter.rtr_mask);

    ret = can_add_rx_filter_msgq(can_dev, &msgq, &filter);
    if (ret < 0) {
        if (ret == -ENOSPC) {
            shell_error(sh, "Failed to add RX filter, no free filter left");
        }
        else {
            shell_error(sh, "Failed to add RX filter [%d]", ret);
        }

        return -EIO;
    }

    shell_print(sh, "Filter ID: %d", ret);

    if (msgq_shell == NULL) {
        msgq_shell = sh;
        k_work_poll_init(&msgq_work, msgq_triggered_work_handler);
    }

    ret = k_work_poll_submit(&msgq_work, msgq_events, ARRAY_SIZE(msgq_events), K_FOREVER);
    if (ret != 0) {
        shell_error(sh, "Failed to submit msgq polling [%d]", ret);
    }

    return (0);
}
#endif

#if (USE_MISRA_COMPLIANCE == 1U)
static int cmd_remove_rx_filter(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    char* end_ptr;
    long id;
    int  ret;

    ret = -EINVAL;
    can_dev = device_get_binding(argv[1]);
    if (can_dev != NULL) {

        id = strtol(argv[2], &end_ptr, 0);
        if (*end_ptr == '\0') {
            if (id >= 0) {
                can_remove_rx_filter(can_dev, (int)id);

                ret = 0;                    // OK path
            }
            else {
                shell_error(sh, "filter_id must not be negative");
            }
        }
        else {
            shell_error(sh, "filter_id is not a number");
        }
    }
    else {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[1]);
    }

    return (ret);
}
#else
static int cmd_remove_rx_filter(const struct shell* sh, size_t argc, char** argv) {
    const struct device* can_dev;
    char* end_ptr;
    long id;

    can_dev = device_get_binding(argv[1]);
    if (!can_dev) {
        shell_error(sh, "Can't get binding to device \"%s\"", argv[1]);
        return (-EINVAL);
    }

    id = strtol(argv[2], &end_ptr, 0);
    if (*end_ptr != '\0') {
        shell_error(sh, "filter_id is not a number");
        return (-EINVAL);
    }

    if (id < 0) {
        shell_error(sh, "filter_id must not be negative");
    }

    can_remove_rx_filter(can_dev, (int)id);

    return (0);
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(sub_can,
    SHELL_CMD_ARG(config, NULL,
                  "Configure CAN controller.\n"
                  " Usage: config device_name [-sl] bitrate\n"
                  " -s Listen-only mode\n"
                  " -l Loopback mode",
                  cmd_config, 3, 1),
    SHELL_CMD_ARG(send, NULL,
                  "Send a CAN frame.\n"
                  " Usage: send device_name [-re] id [byte_1 byte_2 ...]\n"
                  " -r Remote transmission request\n"
                  " -e Extended address",
                  cmd_send, 3, 12),
    SHELL_CMD_ARG(add_rx_filter, NULL,
                  "Add a RX filter and print matching frames.\n"
                  " Usage: add_rx_filter device_name [-re] id [mask [-r]]\n"
                  " -r Remote transmission request\n"
                  " -e Extended address",
                  cmd_add_rx_filter, 3, 3),
    SHELL_CMD_ARG(remove_rx_filter, NULL,
                  "Remove a RX filter and stop printing matching frames\n"
                  " Usage: remove_rx_filter device_name filter_id",
                  cmd_remove_rx_filter, 3, 0),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_ARG_REGISTER(canbus, &sub_can, "CAN commands", NULL, 2, 0);

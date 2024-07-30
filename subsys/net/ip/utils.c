/** @file
 * @brief Misc network utility functions
 *
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_utils, CONFIG_NET_UTILS_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/socketcan.h>

char* net_sprint_addr(sa_family_t af, void const* addr) {
#define NBUFS 3

    static char buf[NBUFS][NET_IPV6_ADDR_LEN];
    static int i;
    char* s = buf[++i % NBUFS];

    return net_addr_ntop(af, addr, s, NET_IPV6_ADDR_LEN);
}

char const* net_verdict2str(enum net_verdict verdict) {
    if (verdict == NET_OK) {
        return "NET_OK";
    }
    else if (verdict == NET_CONTINUE) {
        return "NET_CONTINUE";
    }
    else if (verdict == NET_DROP) {
        return "NET_DROP";
    }

    return "<unknown>";
}

char const* net_proto2str(int family, int proto) {
    if ((family == NET_AF_INET) || (family == NET_AF_INET6)) {
        switch (proto) {
            case NET_IPPROTO_ICMP :
                return "ICMPv4";

            case NET_IPPROTO_TCP :
                return "TCP";

            case NET_IPPROTO_UDP :
                return "UDP";

            case NET_IPPROTO_ICMPV6 :
                return "ICMPv6";

            default :
                break;
        }
    }
    else if (family == NET_AF_CAN) {
        switch (proto) {
            case CAN_RAW :
                return "CAN_RAW";

            default :
                break;
        }
    }

    return "UNK_PROTO";
}

char* net_byte_to_hex(char* ptr, uint8_t byte, char base, bool pad) {
    uint8_t high = (byte >> 4) & 0x0f;
    uint8_t low = byte & 0x0f;

    if (pad || high > 0) {
        *ptr++ = (high < 10) ? (char)(high + '0') : (char)(high - 10 + base);
    }

    *ptr++ = (low < 10) ? (char)(low + '0') : (char)(low - 10 + base);

    *ptr = '\0';

    return ptr;
}

char* net_sprint_ll_addr_buf(uint8_t const* ll, uint8_t ll_len,
                             char* buf, int buflen) {
    uint8_t i;
    uint8_t len;
    uint8_t blen;
    char* ptr = buf;

    if (ll == NULL) {
        return "<unknown>";
    }

    switch (ll_len) {
        case 8 :
            len = 8U;
            break;

        case 6 :
            len = 6U;
            break;

        case 2 :
            len = 2U;
            break;

        default :
            len = 6U;
            break;
    }

    for (i = 0U, blen = (uint8_t)buflen; i < len && blen > 0; i++) {
        ptr = net_byte_to_hex(ptr, (char)ll[i], 'A', true);
        *ptr++ = ':';
        blen -= 3U;
    }

    if (!(ptr - buf)) {
        return NULL;
    }

    *(ptr - 1) = '\0';
    return buf;
}

static int net_value_to_udec(char* buf, uint32_t value, int precision) {
    uint32_t divisor;
    int temp;
    char const* start = buf;

    divisor = 1000000000U;
    if (precision < 0) {
        precision = 1;
    }

    for (int i = 9; i >= 0; i--, divisor /= 10U) {
        temp  = value / divisor;
        value = value % divisor;
        if ((precision > i) || (temp != 0)) {
            precision = i;
            *buf++    = (char)(temp + '0');
        }
    }
    *buf = 0;

    return (buf - start);
}

char* z_impl_net_addr_ntop(sa_family_t family, void const* src,
                           char* dst, size_t size) {
    struct net_in_addr const* addr = NULL;
    struct net_in6_addr const* addr6 = NULL;
    uint16_t const* w = NULL;
    int i;
    uint8_t longest = 1U;
    int pos = -1;
    char delim = ':';
    uint8_t zeros[8] = {0};
    char* ptr = dst;
    int len = -1;
    uint32_t value;
    bool needcolon = false;
    bool mapped = false;

    if (family == NET_AF_INET6) {
        addr6 = src;
        w = addr6->s6_addr16;
        len = 8;

        if (net_ipv6_addr_is_v4_mapped(addr6)) {
            mapped = true;
        }

        for (i = 0; i < 8; i++) {
            for (int j = i; j < 8; j++) {
                if (UNALIGNED_GET(&w[j]) != 0) {
                    break;
                }

                zeros[i]++;
            }
        }

        for (i = 0; i < 8; i++) {
            if (zeros[i] > longest) {
                longest = zeros[i];
                pos = i;
            }
        }

        if (longest == 1U) {
            pos = -1;
        }
    }
    else if (family == NET_AF_INET) {
        addr  = src;
        len   = 4;
        delim = '.';
    }
    else {
        return NULL;
    }

print_mapped :
    for (i = 0; i < len; i++) {
        /* IPv4 address a.b.c.d */
        if (len == 4) {
            int l;

            value = (uint32_t)addr->s4_addr[i];

            /* net_byte_to_udec() eats 0 */
            if (value == 0U) {
                *ptr++ = '0';
                *ptr++ = delim;
                continue;
            }

            l = net_value_to_udec(ptr, value, 0);

            ptr += l;
            *ptr++ = delim;

            continue;
        }

        if (mapped && (i > 5)) {
            delim  = '.';
            len    = 4;
            addr   = (struct net_in_addr const*)(&addr6->s6_addr32[3]);
            *ptr++ = ':';
            family = NET_AF_INET;

            goto print_mapped;
        }

        /* IPv6 address */
        if (i == pos) {
            if (needcolon || i == 0U) {
                *ptr++ = ':';
            }

            *ptr++ = ':';
            needcolon = false;
            i += (int)longest - 1;

            continue;
        }

        if (needcolon) {
            *ptr++ = ':';
        }

        value = (uint32_t)sys_be16_to_cpu(UNALIGNED_GET(&w[i]));
        uint8_t bh = (uint8_t)(value >> 8);
        uint8_t bl = (uint8_t)(value & 0xff);

        if (bh) {
            /* Convert high byte to hex without padding */
            ptr = net_byte_to_hex(ptr, bh, 'a', false);

            /* Always pad the low byte if high byte is non - zero */
            ptr = net_byte_to_hex(ptr, bl, 'a', true);
        }
        else {
            /* For the case where the high byte is zero, only process the low byte
             * Do not pad the low byte if high byte is zero
             */
            ptr = net_byte_to_hex(ptr, bl, 'a', false);
        }

        needcolon = true;
    }

    if (!(ptr - dst)) {
        return NULL;
    }

    if (family == NET_AF_INET) {
        *(ptr - 1) = '\0';
    }
    else {
        *ptr = '\0';
    }

    return dst;
}

#if defined(CONFIG_USERSPACE)
char* z_vrfy_net_addr_ntop(sa_family_t family, void const* src,
                           char* dst, size_t size) {
    char str[INET6_ADDRSTRLEN];
    struct net_in6_addr addr6;
    struct net_in_addr  addr4;
    char* out;
    void const* addr;

    K_OOPS(K_SYSCALL_MEMORY_WRITE(dst, size));

    if (family == NET_AF_INET) {
        K_OOPS(k_usermode_from_copy(&addr4, (void const*)src,
                                    sizeof(addr4)));
        addr = &addr4;
    }
    else if (family == NET_AF_INET6) {
        K_OOPS(k_usermode_from_copy(&addr6, (void const*)src,
                                    sizeof(addr6)));
        addr = &addr6;
    }
    else {
        return 0;
    }

    out = z_impl_net_addr_ntop(family, addr, str, sizeof(str));
    if (!out) {
        return 0;
    }

    K_OOPS(k_usermode_to_copy((void*)dst, str, MIN(size, sizeof(str))));

    return dst;
}

#include <zephyr/syscalls/net_addr_ntop_mrsh.c>
#endif /* CONFIG_USERSPACE */

int z_impl_net_addr_pton(sa_family_t family, char const* src,
                         void* dst) {
    if (family == NET_AF_INET) {
        struct net_in_addr* addr = (struct net_in_addr*)dst;
        size_t i;
        size_t len;

        len = strlen(src);
        for (i = 0; i < len; i++) {
            if (!(src[i] >= '0' && src[i] <= '9') &&
                src[i] != '.') {
                return -EINVAL;
            }
        }

        (void)memset(addr, 0, sizeof(struct net_in_addr));

        for (i = 0; i < sizeof(struct net_in_addr); i++) {
            char* endptr;

            addr->s4_addr[i] = (uint8_t)strtol(src, &endptr, 10);

            src = ++endptr;
        }
    }
    else if (family == NET_AF_INET6) {
        /* If the string contains a '.', it means it's of the form
         * X:X:X:X:X:X:x.x.x.x, and contains only 6 16-bit pieces
         */
        int expected_groups = strchr(src, '.') ? 6 : 8;
        struct net_in6_addr* addr = (struct net_in6_addr*)dst;
        int i;
        int len;

        if (*src == ':') {
            /* Ignore a leading colon, makes parsing neater */
            src++;
        }

        len = strlen(src);
        for (i = 0; i < len; i++) {
            if (!(src[i] >= '0' && src[i] <= '9') &&
                !(src[i] >= 'A' && src[i] <= 'F') &&
                !(src[i] >= 'a' && src[i] <= 'f') &&
                src[i] != '.' && src[i] != ':') {
                return -EINVAL;
            }
        }

        for (i = 0; i < expected_groups; i++) {
            char const* tmp;

            if (!src || *src == '\0') {
                return -EINVAL;
            }

            if (*src != ':') {
                /* Normal IPv6 16-bit piece */
                UNALIGNED_PUT(net_htons(strtol(src, NULL, 16)),
                              &addr->s6_addr16[i]);
                src = strchr(src, ':');
                if (src) {
                    src++;
                }
                else {
                    if (i < expected_groups - 1) {
                        return -EINVAL;
                    }
                }

                continue;
            }

            /* Two colons in a row */

            for (; i < expected_groups; i++) {
                UNALIGNED_PUT(0, &addr->s6_addr16[i]);
            }

            tmp = strrchr(src, ':');
            if (src == tmp && (expected_groups == 6 || !src[1])) {
                src++;
                break;
            }

            if (expected_groups == 6) {
                /* we need to drop the trailing
                 * colon since it's between the
                 * ipv6 and ipv4 addresses, rather than being
                 * a part of the ipv6 address
                 */
                tmp--;
            }

            /* Calculate the amount of skipped zeros */
            i = expected_groups - 1;
            do {
                if (*tmp == ':') {
                    i--;
                }

                if (i < 0) {
                    return -EINVAL;
                }
            } while (tmp-- != src);

            src++;
        }

        if (expected_groups == 6) {
            /* Parse the IPv4 part */
            for (i = 0; i < 4; i++) {
                if (!src || !*src) {
                    return -EINVAL;
                }

                addr->s6_addr[12 + i] = (uint8_t)strtol(src, NULL, 10);

                src = strchr(src, '.');
                if (src) {
                    src++;
                }
                else {
                    if (i < 3) {
                        return -EINVAL;
                    }
                }
            }
        }
    }
    else {
        return -EINVAL;
    }

    return 0;
}

#if defined(CONFIG_USERSPACE)
int z_vrfy_net_addr_pton(sa_family_t family, char const* src,
                         void* dst) {
    char str[MAX(INET_ADDRSTRLEN, INET6_ADDRSTRLEN)] = {};
    struct net_in6_addr addr6;
    struct net_in_addr  addr4;
    void* addr;
    size_t size;
    int err;

    if (family == NET_AF_INET) {
        size = sizeof(struct net_in_addr);
        addr = &addr4;
    }
    else if (family == NET_AF_INET6) {
        size = sizeof(struct net_in6_addr);
        addr = &addr6;
    }
    else {
        return -EINVAL;
    }

    if (k_usermode_string_copy(str, (char*)src, sizeof(str)) != 0) {
        return -EINVAL;
    }

    K_OOPS(K_SYSCALL_MEMORY_WRITE(dst, size));

    err = z_impl_net_addr_pton(family, str, addr);
    if (err) {
        return err;
    }

    K_OOPS(k_usermode_to_copy((void*)dst, addr, size));

    return 0;
}

#include <zephyr/syscalls/net_addr_pton_mrsh.c>
#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_LITTLE_ENDIAN
#define CHECKSUM_BIG_ENDIAN 0
#else
#define CHECKSUM_BIG_ENDIAN 1
#endif

static uint16_t offset_based_swap8(uint8_t const* data) {
    uint16_t data16 = (uint16_t)*data;

    if (((uintptr_t)(data)&1) == CHECKSUM_BIG_ENDIAN) {
        return data16;
    }
    else {
        return data16 << 8;
    }
}

/* Word based checksum calculation based on:
 * https://blogs.igalia.com/dpino/2018/06/14/fast-checksum-computation/
 * It’s not necessary to add octets as 16-bit words. Due to the associative property of addition,
 * it is possible to do parallel addition using larger word sizes such as 32-bit or 64-bit words.
 * In those cases the variable that stores the accumulative sum has to be bigger too.
 * Once the sum is computed a final step folds the sum to a 16-bit word (adding carry if any).
 */
uint16_t calc_chksum(uint16_t sum_in, uint8_t const* data, size_t len) {
    uint64_t  sum;
    uint32_t* p;
    size_t i = 0;
    size_t pending = len;
    int odd_start = ((uintptr_t)data & 0x01);

    /* Sum in is in host endianness, working order endianness is both dependent on endianness
     * and the offset of starting
     */
    if (odd_start == CHECKSUM_BIG_ENDIAN) {
        sum = BSWAP_16(sum_in);
    }
    else {
        sum = sum_in;
    }

    /* Process up to 3 data elements up front, so the data is aligned further down the line */
    if ((((uintptr_t)data & 0x01) != 0) && (pending >= 1)) {
        sum += offset_based_swap8(data);
        data++;
        pending--;
    }
    if ((((uintptr_t)data & 0x02) != 0) && (pending >= sizeof(uint16_t))) {
        pending -= sizeof(uint16_t);
        sum = sum + *((uint16_t*)data);
        data += sizeof(uint16_t);
    }
    p = (uint32_t*)data;

    /* Do loop unrolling for the very large data sets */
    while (pending >= sizeof(uint32_t) * 4) {
        uint64_t sum_a = p[i];
        uint64_t sum_b = p[i + 1];

        pending -= sizeof(uint32_t) * 4;
        sum_a += p[i + 2];
        sum_b += p[i + 3];
        i += 4;
        sum += sum_a + sum_b;
    }
    while (pending >= sizeof(uint32_t)) {
        pending -= sizeof(uint32_t);
        sum = sum + p[i++];
    }
    data = (uint8_t*)(p + i);
    if (pending >= 2) {
        pending -= sizeof(uint16_t);
        sum = sum + *((uint16_t*)data);
        data += sizeof(uint16_t);
    }
    if (pending == 1) {
        sum += offset_based_swap8(data);
    }

    /* Fold sum into 16-bit word. */
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }

    /* Sum in is in host endianness, working order endianness is both dependent on endianness
     * and the offset of starting
     */
    if (odd_start == CHECKSUM_BIG_ENDIAN) {
        return BSWAP_16((uint16_t)sum);
    }
    else {
        return (uint16_t)sum;
    }
}

static inline uint16_t pkt_calc_chksum(struct net_pkt* pkt, uint16_t sum) {
    struct net_pkt_cursor* cur = &pkt->cursor;
    size_t                 len;

    if (!cur->buf || !cur->pos) {
        return sum;
    }

    len = cur->buf->len - (cur->pos - cur->buf->data);

    while (cur->buf) {
        sum = calc_chksum(sum, cur->pos, len);

        cur->buf = cur->buf->frags;
        if (!cur->buf || !cur->buf->len) {
            break;
        }

        cur->pos = cur->buf->data;

        if (len % 2) {
            sum += *cur->pos;
            if (sum < *cur->pos) {
                sum++;
            }

            cur->pos++;
            len = cur->buf->len - 1;
        }
        else {
            len = cur->buf->len;
        }
    }

    return sum;
}

#if defined(CONFIG_NET_IP)
uint16_t net_calc_chksum(struct net_pkt* pkt, uint8_t proto) {
    size_t len = 0U;
    uint16_t sum = 0U;
    struct net_pkt_cursor backup;
    bool ow;

    if (IS_ENABLED(CONFIG_NET_IPV4) &&
        net_pkt_family(pkt) == NET_AF_INET) {
        if (proto != NET_IPPROTO_ICMP && proto != NET_IPPROTO_IGMP) {
            len = 2 * sizeof(struct net_in_addr);
            sum = (uint16_t)(net_pkt_get_len(pkt) -
                             net_pkt_ip_hdr_len(pkt) -
                             net_pkt_ipv4_opts_len(pkt) + proto);
        }
    }
    else if (IS_ENABLED(CONFIG_NET_IPV6) &&
             net_pkt_family(pkt) == NET_AF_INET6) {
        len = 2 * sizeof(struct net_in6_addr);
        sum = (uint16_t)(net_pkt_get_len(pkt) -
                         net_pkt_ip_hdr_len(pkt) -
                         net_pkt_ipv6_ext_len(pkt) + proto);
    }
    else {
        NET_DBG("Unknown protocol family %d", net_pkt_family(pkt));
        return 0;
    }

    net_pkt_cursor_backup(pkt, &backup);
    net_pkt_cursor_init(pkt);

    ow = net_pkt_is_being_overwritten(pkt);
    net_pkt_set_overwrite(pkt, true);

    net_pkt_skip(pkt, net_pkt_ip_hdr_len(pkt) - len);

    sum = calc_chksum(sum, pkt->cursor.pos, len);
    net_pkt_skip(pkt, len + net_pkt_ip_opts_len(pkt));

    sum = pkt_calc_chksum(pkt, sum);

    sum = (sum == 0U) ? 0xffff : net_htons(sum);

    net_pkt_cursor_restore(pkt, &backup);

    net_pkt_set_overwrite(pkt, ow);

    return ~sum;
}
#endif

#if defined(CONFIG_NET_IPV4)
uint16_t net_calc_chksum_ipv4(struct net_pkt* pkt) {
    uint16_t sum;

    sum = calc_chksum(0, pkt->buffer->data,
                      net_pkt_ip_hdr_len(pkt) +
                      net_pkt_ipv4_opts_len(pkt));

    sum = (sum == 0U) ? 0xffff : net_htons(sum);

    return ~sum;
}
#endif /* CONFIG_NET_IPV4 */

#if defined(CONFIG_NET_IPV4_IGMP)
uint16_t net_calc_chksum_igmp(struct net_pkt* pkt) {
    return net_calc_chksum(pkt, NET_IPPROTO_IGMP);
}
#endif /* CONFIG_NET_IPV4_IGMP */

#if defined(CONFIG_NET_IP)
static bool convert_port(char const* buf, uint16_t* port) {
    unsigned long tmp;
    char*         endptr;

    tmp = strtoul(buf, &endptr, 10);
    if ((endptr == buf && tmp == 0) ||
        !(*buf != '\0' && *endptr == '\0') ||
        ((unsigned long)(unsigned short)tmp != tmp)) {
        return false;
    }

    *port = (uint16_t)tmp;

    return true;
}
#endif /* CONFIG_NET_IP */

#if defined(CONFIG_NET_IPV6)
static bool parse_ipv6(char const* str, size_t str_len,
                       struct net_sockaddr* addr, bool has_port) {
    char* ptr = NULL;
    struct net_in6_addr* addr6;
    char ipaddr[INET6_ADDRSTRLEN + 1];
    int end, len, ret, i;
    uint16_t port;

    len = MIN(INET6_ADDRSTRLEN, str_len);

    for (i = 0; i < len; i++) {
        if (!str[i]) {
            len = i;
            break;
        }
    }

    if (has_port) {
        /* IPv6 address with port number */
        ptr = memchr(str, ']', len);
        if (!ptr) {
            return false;
        }

        end = MIN(len, ptr - (str + 1));
        memcpy(ipaddr, str + 1, end);
    }
    else {
        end = len;
        memcpy(ipaddr, str, end);
    }

    ipaddr[end] = '\0';

    addr6 = &net_sin6(addr)->sin6_addr;

    ret = net_addr_pton(NET_AF_INET6, ipaddr, addr6);
    if (ret < 0) {
        return false;
    }

    net_sin6(addr)->sin6_family = NET_AF_INET6;

    if (!has_port) {
        return true;
    }

    if ((ptr + 1) < (str + str_len) && *(ptr + 1) == ':') {
        /* -1 as end does not contain first [
         * -2 as pointer is advanced by 2, skipping ]:
         */
        len = str_len - end - 1 - 2;

        ptr += 2;

        for (i = 0; i < len; i++) {
            if (!ptr[i]) {
                len = i;
                break;
            }
        }

        /* Re-use the ipaddr buf for port conversion */
        memcpy(ipaddr, ptr, len);
        ipaddr[len] = '\0';

        ret = convert_port(ipaddr, &port);
        if (!ret) {
            return false;
        }

        net_sin6(addr)->sin6_port = net_htons(port);

        NET_DBG("IPv6 host %s port %d",
                net_addr_ntop(NET_AF_INET6, addr6, ipaddr, sizeof(ipaddr) - 1),
                port);
    }
    else {
        NET_DBG("IPv6 host %s",
                net_addr_ntop(NET_AF_INET6, addr6, ipaddr, sizeof(ipaddr) - 1));
    }

    return true;
}
#else
static inline bool parse_ipv6(char const* str, size_t str_len,
                              struct net_sockaddr* addr, bool has_port) {
    return false;
}
#endif /* CONFIG_NET_IPV6 */

#if defined(CONFIG_NET_IPV4)
static bool parse_ipv4(char const* str, size_t str_len,
                       struct net_sockaddr* addr, bool has_port) {
    char* ptr = NULL;
    char ipaddr[NET_IPV4_ADDR_LEN + 1];
    struct net_in_addr* addr4;
    int end, len, ret, i;
    uint16_t port;

    len = MIN(NET_IPV4_ADDR_LEN, str_len);

    for (i = 0; i < len; i++) {
        if (!str[i]) {
            len = i;
            break;
        }
    }

    if (has_port) {
        /* IPv4 address with port number */
        ptr = memchr(str, ':', len);
        if (!ptr) {
            return false;
        }

        end = MIN(len, ptr - str);
    }
    else {
        end = len;
    }

    memcpy(ipaddr, str, end);
    ipaddr[end] = '\0';

    addr4 = &net_sin(addr)->sin_addr;

    ret = net_addr_pton(NET_AF_INET, ipaddr, addr4);
    if (ret < 0) {
        return false;
    }

    net_sin(addr)->sin_family = NET_AF_INET;

    if (!has_port) {
        return true;
    }

    memcpy(ipaddr, ptr + 1, str_len - end);
    ipaddr[str_len - end] = '\0';

    ret = convert_port(ipaddr, &port);
    if (!ret) {
        return false;
    }

    net_sin(addr)->sin_port = net_htons(port);

    NET_DBG("IPv4 host %s port %d",
            net_addr_ntop(NET_AF_INET, addr4, ipaddr, sizeof(ipaddr) - 1),
            port);
    return true;
}
#else
static inline bool parse_ipv4(char const* str, size_t str_len,
                              struct net_sockaddr* addr, bool has_port) {
    return false;
}
#endif /* CONFIG_NET_IPV4 */

bool net_ipaddr_parse(char const* str, size_t str_len, struct net_sockaddr* addr) {
    size_t i;
    size_t count;

    if (!str || str_len == 0) {
        return false;
    }

    /* We cannot accept empty string here */
    if (*str == '\0') {
        return false;
    }

    if (*str == '[') {
        return parse_ipv6(str, str_len, addr, true);
    }

    for (count = i = 0; i < str_len && str[i]; i++) {
        if (str[i] == ':') {
            count++;
        }
    }

    if (count == 1) {
        return parse_ipv4(str, str_len, addr, true);
    }

    #if defined(CONFIG_NET_IPV4) && defined(CONFIG_NET_IPV6)
    if (!parse_ipv4(str, str_len, addr, false)) {
        return parse_ipv6(str, str_len, addr, false);
    }

    return true;
    #endif

    #if defined(CONFIG_NET_IPV4) && !defined(CONFIG_NET_IPV6)
    return parse_ipv4(str, str_len, addr, false);
    #endif

    #if defined(CONFIG_NET_IPV6) && !defined(CONFIG_NET_IPV4)
    return parse_ipv6(str, str_len, addr, false);
    #endif

    return false;
}

int net_port_set_default(struct net_sockaddr* addr, uint16_t default_port) {
    if (IS_ENABLED(CONFIG_NET_IPV4) && (addr->sa_family == NET_AF_INET) &&
        (net_sin(addr)->sin_port == 0)) {
        net_sin(addr)->sin_port = net_htons(default_port);
    }
    else if (IS_ENABLED(CONFIG_NET_IPV6) && (addr->sa_family == NET_AF_INET6) &&
             (net_sin6(addr)->sin6_port == 0)) {
        net_sin6(addr)->sin6_port = net_htons(default_port);
    }
    else if ((IS_ENABLED(CONFIG_NET_IPV4) && (addr->sa_family == NET_AF_INET)) ||
             (IS_ENABLED(CONFIG_NET_IPV6) && (addr->sa_family == NET_AF_INET6))) {
        /* Port is already set */
    }
    else {
        LOG_ERR("Unknown address family");
        return -EINVAL;
    }

    return 0;
}

int net_bytes_from_str(uint8_t* buf, int buf_len, char const* src) {
    size_t i;
    size_t src_len = strlen(src);
    char*  endptr;

    for (i = 0U; i < src_len; i++) {
        if (!isxdigit((unsigned char)src[i]) &&
            src[i] != ':') {
            return -EINVAL;
        }
    }

    (void) memset(buf, 0, buf_len);

    for (i = 0U; i < (size_t)buf_len; i++) {
        buf[i] = (uint8_t)strtol(src, &endptr, 16);
        src    = ++endptr;
    }

    return (0);
}

char const* net_family2str(sa_family_t family) {
    switch (family) {
        case NET_AF_UNSPEC :
            return "AF_UNSPEC";

        case NET_AF_INET :
            return "AF_INET";

        case NET_AF_INET6 :
            return "AF_INET6";

        case NET_AF_PACKET :
            return "AF_PACKET";

        case NET_AF_CAN :
            return "AF_CAN";
    }

    return NULL;
}

const struct net_in_addr* net_ipv4_unspecified_address(void) {
    static const struct net_in_addr addr;

    return (&addr);
}

const struct net_in_addr* net_ipv4_broadcast_address(void) {
    static const struct net_in_addr addr = {{{255, 255, 255, 255}}};

    return (&addr);
}

/* IPv6 wildcard and loopback address defined by RFC2553 */
const struct net_in6_addr in6addr_any      = IN6ADDR_ANY_INIT;
const struct net_in6_addr in6addr_loopback = IN6ADDR_LOOPBACK_INIT;

const struct net_in6_addr* net_ipv6_unspecified_address(void) {
    return (&in6addr_any);
}

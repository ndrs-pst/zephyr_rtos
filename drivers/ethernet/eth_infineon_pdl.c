/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_eth_pdl

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(infineon_eth_pdl, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/phy.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/clock_control_ifx_cat1.h>
#include <ethernet/eth_stats.h>

#include "cy_ethif.h"

#define INFINEON_ETH_MAX_FRAME_SIZE (NET_ETH_MTU + 18U) /* MTU + Ethernet header (14) + CRC (4) */
#define INFINEON_ETH_TX_BUF_SIZE    INFINEON_ETH_MAX_FRAME_SIZE
#define INFINEON_ETH_RX_BUF_SIZE    INFINEON_ETH_MAX_FRAME_SIZE
#define INFINEON_ETH_NUM_RX_BUFS    4U
#define INFINEON_ETH_NUM_TX_BUFS    4U

static cy_stc_ethif_mac_config_t const stc_enet_cfg_dfl = {
    .bintrEnable         = 1,                           /** Interrupt enable  */
    .dmaDataBurstLen     = CY_ETHIF_DMA_DBUR_LEN_4,
    .u8dmaCfgFlags       = CY_ETHIF_CFG_DMA_FRCE_TX_BRST,
    #if defined(CONFIG_SOC_FAMILY_INFINEON_EDGE)
    .mdcPclkDiv          = CY_ETHIF_MDC_DIV_BY_96,      /** source clock is 200 MHz and MDC must be less than 2.5MHz   */
    #else
    .mdcPclkDiv          = CY_ETHIF_MDC_DIV_BY_48,      /** source clock is 100 MHz and MDC must be less than 2.5MHz   */
    #endif
    .u8rxLenErrDisc      = 0,                           /** Length error frame not discarded  */
    .u8disCopyPause      = 0,
    .u8chkSumOffEn       = 0,                           /** Checksum for both Tx and Rx disabled    */
    .u8rx1536ByteEn      = 1,                           /** Enable receive frame up to 1536    */
    .u8rxJumboFrEn       = 0,
    .u8enRxBadPreamble   = 1,
    .u8ignoreIpgRxEr     = 0,
    .u8storeUdpTcpOffset = 0,
    .u8aw2wMaxPipeline   = 2,                           /** Value must be > 0   */
    .u8ar2rMaxPipeline   = 2,                           /** Value must be > 0   */
    .u8pfcMultiQuantum   = 0,
    .pstcWrapperConfig   = NULL,
    .pstcTSUConfig       = NULL,
    .btxq0enable         = 1,                           /** Tx Q0 Enabled   */
    .btxq1enable         = 0,                           /** Tx Q1 Disabled  */
    .btxq2enable         = 0,                           /** Tx Q2 Disabled  */
    .brxq0enable         = 1,                           /** Rx Q0 Enabled   */
    .brxq1enable         = 0,                           /** Rx Q1 Disabled  */
    .brxq2enable         = 0,                           /** Rx Q2 Disabled  */
};

/* DT phy-connection-type enum index -> PDL interface selector.
 * Enum order from ethernet-controller.yaml: mii=0, rmii=1, gmii=2, rgmii=3.
 */
static cy_en_ethif_speed_sel_t const k_conn_map[4] = {
    CY_ETHIF_CTL_MII_10,
    CY_ETHIF_CTL_RMII_100,
    CY_ETHIF_CTL_GMII_1000,
    CY_ETHIF_CTL_RGMII_1000
};

struct eth_infineon_pdl_config {
    ETH_Type* base;
    uint8_t mac_addr[6];
    uint8_t phy_conn_type;
    void (*irq_config_func)(const struct device* dev);
    const struct pinctrl_dev_config* pincfg;
    en_clk_dst_t clk_dst;

    /* Per-instance PDL callbacks — populated by macro, passed to
     * Cy_ETHIF_RegisterCallbacks() at init time.
     */
    cy_stc_ethif_cb_t pdl_cb;
};

struct eth_rx_pending {
    uint8_t* buf;
    uint32_t len;
};

struct eth_infineon_pdl_data {
    struct net_if* iface;
    const struct device* dev;   /* self-pointer used by rx_work handler */
    uint8_t mac_addr[6];
    struct ifx_cat1_clock clock;

    struct k_sem tx_sem;
    struct k_mutex tx_mutex;

    const struct device*  phy_dev;
    struct phy_link_state phy_state;

    struct k_work rx_work;

    /* Pending RX frame ring: written in ISR trampoline, read in work handler */
    struct eth_rx_pending rx_pending[INFINEON_ETH_NUM_RX_BUFS];
    uint8_t rx_enq;             /* ISR write index */
    uint8_t rx_deq;             /* work handler read index */
    atomic_t rx_pending_count;

    /* Cycling index used by rxgetbuff to provide replacement buffers */
    atomic_t rx_getbuff_idx;

    /* 32-byte aligned DMA buffers required by Cadence GEM */
    uint8_t __aligned(32) rx_buffers[INFINEON_ETH_NUM_RX_BUFS][INFINEON_ETH_RX_BUF_SIZE];
    uint8_t __aligned(32) tx_buffers[INFINEON_ETH_NUM_TX_BUFS][INFINEON_ETH_TX_BUF_SIZE];

    /* Pointer array passed to PDL as RX queue buffer pool */
    uint8_t* rx_buf_ptrs[INFINEON_ETH_NUM_RX_BUFS];

    uint8_t tx_idx;
};

/* -------------------------------------------------------------------
 * Shared callback bodies — operate on data * directly.
 * Called by per-instance trampolines generated in ETH_INFINEON_PDL_DEVICE(n).
 * These are ISR-context for rx/tx; only ISR-safe ops permitted.
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_on_rx(ETH_Type* base,
                                   struct eth_infineon_pdl_data* data,
                                   uint8_t* buf, uint32_t len) {
    ARG_UNUSED(base);

    if ((buf == NULL) || (len == 0U)) {
        return;
    }

    if (atomic_get(&data->rx_pending_count) <
        (atomic_val_t)INFINEON_ETH_NUM_RX_BUFS) {
        size_t rx_enq = data->rx_enq;

        data->rx_pending[rx_enq].buf = buf;
        data->rx_pending[rx_enq].len = len;
        data->rx_enq = (rx_enq + 1U) % INFINEON_ETH_NUM_RX_BUFS;
        atomic_inc(&data->rx_pending_count);
    }

    k_work_submit(&data->rx_work);
}

static void eth_infineon_pdl_on_tx_complete(struct eth_infineon_pdl_data* data) {
    k_sem_give(&data->tx_sem);
}

static void eth_infineon_pdl_on_tx_error(struct eth_infineon_pdl_data* data) {
    k_sem_give(&data->tx_sem);
}

/* -------------------------------------------------------------------
 * rxgetbuff body — cycles through rx_buffers[] to provide replacement
 * buffers to the PDL descriptor ring. Called from ISR (DecodeEvent).
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_on_rxgetbuff(struct eth_infineon_pdl_data* data,
                                          uint8_t** buf_out, uint32_t* len_out) {
    uint32_t idx = (uint32_t)atomic_inc(&data->rx_getbuff_idx) %
                   INFINEON_ETH_NUM_RX_BUFS;

    *buf_out = data->rx_buffers[idx];
    *len_out = INFINEON_ETH_RX_BUF_SIZE;
}

/* -------------------------------------------------------------------
 * RX work handler — system work queue context.
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_rx_work_handler(struct k_work* work) {
    struct eth_infineon_pdl_data* data =
        CONTAINER_OF(work, struct eth_infineon_pdl_data, rx_work);
    struct net_pkt* pkt;

    while (atomic_get(&data->rx_pending_count) > 0) {
        size_t rx_deq = data->rx_deq;
        uint8_t* buf = data->rx_pending[rx_deq].buf;
        uint32_t len = data->rx_pending[rx_deq].len;

        /* Invalidate D-cache so CPU sees what DMA wrote */
        #if defined(CONFIG_DCACHE)
        sys_cache_data_invd_range(buf, (size_t)len);
        #endif

        pkt = net_pkt_rx_alloc_with_buffer(data->iface, (size_t)len,
                                           NET_AF_UNSPEC, 0, K_MSEC(50));
        if ((pkt != NULL) && (net_pkt_write(pkt, buf, len) == 0)) {
            if (net_recv_data(data->iface, pkt) < 0) {
                net_pkt_unref(pkt);
                eth_stats_update_errors_rx(data->iface);
            }
            else {
                eth_stats_update_bytes_rx(data->iface, len);
                eth_stats_update_pkts_rx(data->iface);
            }
        }
        else {
            net_pkt_unref(pkt);             /* net_pkt_unref(NULL) is a no-op */
            eth_stats_update_errors_rx(data->iface);
        }

        data->rx_deq = (rx_deq + 1U) % INFINEON_ETH_NUM_RX_BUFS;
        atomic_dec(&data->rx_pending_count);
    }
}

/* -------------------------------------------------------------------
 * ISR — calls DecodeEvent which dispatches into the per-instance
 * trampolines registered via Cy_ETHIF_RegisterCallbacks().
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_isr(const struct device* dev) {
    const struct eth_infineon_pdl_config* cfg = dev->config;

    Cy_ETHIF_DecodeEvent(cfg->base);
}

static int eth_infineon_pdl_tx(const struct device* dev, struct net_pkt* pkt) {
    struct eth_infineon_pdl_data* data = dev->data;
    const struct eth_infineon_pdl_config* cfg  = dev->config;
    size_t len = net_pkt_get_len(pkt);
    cy_en_ethif_status_t status;
    int ret = 0;

    if (len > INFINEON_ETH_TX_BUF_SIZE) {
        return (-EINVAL);
    }

    k_mutex_lock(&data->tx_mutex, K_FOREVER);

    if (net_pkt_read(pkt, data->tx_buffers[data->tx_idx], len) < 0) {
        ret = -EIO;
        goto out;
    }

    /* Flush D-cache so DMA reads what CPU wrote */
    #if defined(CONFIG_DCACHE)
    sys_cache_data_flush_range(data->tx_buffers[data->tx_idx], (size_t)len);
    #endif

    status = Cy_ETHIF_TransmitFrame(cfg->base,
                                    data->tx_buffers[data->tx_idx],
                                    len, 0U, true);
    if (status != CY_ETHIF_SUCCESS) {
        eth_stats_update_errors_tx(data->iface);
        ret = -EIO;
        goto out;
    }

    /* Block until on_tx_complete or on_tx_error gives the semaphore */
    if (k_sem_take(&data->tx_sem, K_MSEC(100)) != 0) {
        eth_stats_update_errors_tx(data->iface);
        ret = -ETIMEDOUT;
    }
    else {
        eth_stats_update_bytes_tx(data->iface, len);
        eth_stats_update_pkts_tx(data->iface);
    }

out :
    data->tx_idx = (data->tx_idx + 1U) % INFINEON_ETH_NUM_TX_BUFS;
    k_mutex_unlock(&data->tx_mutex);

    return (ret);
}

/* -------------------------------------------------------------------
 * PHY link-state callback
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_phy_cb(const struct device* phy_dev,
                                    struct phy_link_state* state,
                                    void* user_data) {
    ARG_UNUSED(phy_dev);
    const struct device* dev  = user_data;
    struct eth_infineon_pdl_data* data = dev->data;

    data->phy_state = *state;

    if (state->is_up == true) {
        net_eth_carrier_on(data->iface);
        LOG_INF("Link UP: %d Mbps %s-duplex",
                (state->speed & (LINK_FULL_100BASE | LINK_HALF_100BASE)) ? 100 : 10,
                PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
    }
    else {
        net_eth_carrier_off(data->iface);
        LOG_INF("Link DOWN");
    }
}

/* -------------------------------------------------------------------
 * Ethernet API
 * ------------------------------------------------------------------- */
static void eth_infineon_pdl_iface_init(struct net_if* iface) {
    const struct device* dev = net_if_get_device(iface);
    struct eth_infineon_pdl_data* data = dev->data;

    data->iface = iface;
    net_if_set_link_addr(iface, data->mac_addr,
                         sizeof(data->mac_addr), NET_LINK_ETHERNET);
    ethernet_init(iface);

    if (data->phy_dev) {
        phy_link_callback_set(data->phy_dev,
                              eth_infineon_pdl_phy_cb, (void*)dev);
    }
}

static enum ethernet_hw_caps eth_infineon_pdl_get_capabilities(const struct device* dev,
                                                               struct net_if* iface) {
    ARG_UNUSED(dev);
    ARG_UNUSED(iface);
    return (ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_PROMISC_MODE);
}

static const struct device* eth_infineon_pdl_get_phy(const struct device* dev,
                                                     struct net_if* iface) {
    ARG_UNUSED(iface);
    struct eth_infineon_pdl_data* data = dev->data;

    return (data->phy_dev);
}

static int eth_infineon_pdl_set_config(const struct device* dev,
                                       struct net_if* iface,
                                       enum ethernet_config_type type,
                                       const struct ethernet_config* config) {
    ARG_UNUSED(iface);
    struct eth_infineon_pdl_data* data = dev->data;
    const struct eth_infineon_pdl_config* cfg  = dev->config;

    switch (type) {
        case ETHERNET_CONFIG_TYPE_MAC_ADDRESS : {
            cy_stc_ethif_filter_config_t filter_cfg;

            memcpy(data->mac_addr, config->mac_address.addr,
                   sizeof(data->mac_addr));

            filter_cfg.typeFilter = CY_ETHIF_FILTER_TYPE_DESTINATION;
            memcpy(&filter_cfg.filterAddr.byte[0], data->mac_addr, sizeof(data->mac_addr));
            filter_cfg.ignoreBytes = 0x00;

            Cy_ETHIF_SetFilterAddress(cfg->base, CY_ETHIF_FILTER_NUM_1, &filter_cfg);
            return (0);
        }

        default :
            return (-ENOTSUP);
    }
}

static struct ethernet_api const eth_infineon_pdl_api = {
    .iface_api.init   = eth_infineon_pdl_iface_init,
    .get_capabilities = eth_infineon_pdl_get_capabilities,
    .get_phy          = eth_infineon_pdl_get_phy,
    .set_config       = eth_infineon_pdl_set_config,
    .send             = eth_infineon_pdl_tx
};

/* -------------------------------------------------------------------
 * Driver init — generic across all instances.
 * ------------------------------------------------------------------- */
static int eth_infineon_pdl_init(const struct device* dev) {
    struct eth_infineon_pdl_data* data = dev->data;
    const struct eth_infineon_pdl_config* cfg = dev->config;
    cy_stc_ethif_wrapper_config_t wrapper_cfg = {0};
    cy_stc_ethif_mac_config_t mac_cfg;
    cy_stc_ethif_intr_config_t int_cfg = {0};
    cy_en_ethif_status_t eth_status;
    cy_rslt_t clk_status;
    int ret;

    k_sem_init(&data->tx_sem, 0, 1);
    k_mutex_init(&data->tx_mutex);
    k_work_init(&data->rx_work, eth_infineon_pdl_rx_work_handler);

    data->dev    = dev;
    data->tx_idx = 0U;
    atomic_set(&data->rx_getbuff_idx, 0);
    memcpy(data->mac_addr, cfg->mac_addr, sizeof(data->mac_addr));

    for (int i = 0; i < (int)INFINEON_ETH_NUM_RX_BUFS; i++) {
        data->rx_buf_ptrs[i] = data->rx_buffers[i];
    }

    wrapper_cfg.stcInterfaceSel = k_conn_map[cfg->phy_conn_type];
    wrapper_cfg.bRefClockSource = CY_ETHIF_EXTERNAL_HSIO;
    wrapper_cfg.u8RefClkDiv     = 0U;

    mac_cfg = stc_enet_cfg_dfl;
    mac_cfg.pstcWrapperConfig   = &wrapper_cfg;
    mac_cfg.pRxQbuffPool[0]     = (cy_ethif_buffpool_t*)data->rx_buf_ptrs;
    int_cfg.btx_complete         = true;
    int_cfg.brx_complete         = true;

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("ETH pinctrl apply failed (%d)", ret);
        return (ret);
    }

    clk_status = ifx_cat1_utils_peri_pclk_assign_divider(cfg->clk_dst, &data->clock);
    if (clk_status != CY_RSLT_SUCCESS) {
        LOG_ERR("ETH clock assign failed (%d)", (int)clk_status);
        return (-EIO);
    }

    eth_status = Cy_ETHIF_MdioInit(cfg->base, &mac_cfg);
    if (eth_status != CY_ETHIF_SUCCESS) {
        LOG_ERR("Cy_ETHIF_MdioInit failed");
        return (-EIO);
    }

    eth_status = Cy_ETHIF_Init(cfg->base, &mac_cfg, &int_cfg);
    if (eth_status != CY_ETHIF_SUCCESS) {
        LOG_ERR("Cy_ETHIF_Init failed");
        return (-EIO);
    }

    /* Stack-copy cfg->pdl_cb to satisfy the non-const API signature */
    cy_stc_ethif_filter_config_t filter_cfg;
    cy_stc_ethif_cb_t cb = cfg->pdl_cb;

    Cy_ETHIF_RegisterCallbacks(cfg->base, &cb);

    filter_cfg.typeFilter = CY_ETHIF_FILTER_TYPE_DESTINATION;
    memcpy(&filter_cfg.filterAddr.byte[0], data->mac_addr, sizeof(data->mac_addr));
    filter_cfg.ignoreBytes = 0x00;

    Cy_ETHIF_SetFilterAddress(cfg->base, CY_ETHIF_FILTER_NUM_1, &filter_cfg);

    cfg->irq_config_func(dev);

    if ((data->phy_dev != NULL) && !device_is_ready(data->phy_dev)) {
        LOG_ERR("PHY device not ready");
        return (-ENODEV);
    }

    return (0);
}

/* -------------------------------------------------------------------
 * Per-instance peripheral clock struct initialiser (mirrors CAN driver pattern).
 * ------------------------------------------------------------------- */
#if defined(CONFIG_SOC_FAMILY_INFINEON_EDGE)
#define ETH_PERI_CLOCK_INIT(n)                                              \
    .clock = {                                                              \
        .block   = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(                        \
                       DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 0), \
                       DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
                       DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)),       \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel),             \
    },
#else
#define ETH_PERI_CLOCK_INIT(n)                                              \
    .clock = {                                                              \
        .block   = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(                        \
                       DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
                       DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)),       \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel),             \
    },
#endif

/* -------------------------------------------------------------------
 * Per-instance macro: generates trampolines + device definition.
 *
 * Each trampoline closes over its DT instance index so DEVICE_DT_INST_GET(n)
 * resolves the correct device at compile time — no global table, no loop.
 * ------------------------------------------------------------------- */
#define ETH_INFINEON_PDL_DEVICE(n)                                              \
    PINCTRL_DT_INST_DEFINE(n);                                                  \
                                                                                \
    static void eth_pdl_rx_cb_##n(ETH_Type* base, uint8_t* buf, uint32_t len) { \
        eth_infineon_pdl_on_rx(base, DEVICE_DT_INST_GET(n)->data, buf, len);    \
    }                                                                           \
                                                                                \
    static void eth_pdl_tx_complete_cb_##n(ETH_Type* base, uint8_t q) {         \
        ARG_UNUSED(base);                                                       \
        ARG_UNUSED(q);                                                          \
        eth_infineon_pdl_on_tx_complete(DEVICE_DT_INST_GET(n)->data);           \
    }                                                                           \
                                                                                \
    static void eth_pdl_tx_error_cb_##n(ETH_Type* base, uint8_t q) {            \
        ARG_UNUSED(base);                                                       \
        ARG_UNUSED(q);                                                          \
        eth_infineon_pdl_on_tx_error(DEVICE_DT_INST_GET(n)->data);              \
    }                                                                           \
                                                                                \
    static void eth_pdl_rxgetbuff_cb_##n(ETH_Type* base,                        \
                                         uint8_t** buf_out, uint32_t* len_out) { \
        ARG_UNUSED(base);                                                       \
        eth_infineon_pdl_on_rxgetbuff(DEVICE_DT_INST_GET(n)->data,              \
                                      buf_out, len_out);                        \
    }                                                                           \
                                                                                \
    static void eth_infineon_pdl_irq_config_##n(const struct device* dev) {     \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                  \
                    eth_infineon_pdl_isr, DEVICE_DT_INST_GET(n), 0);            \
        irq_enable(DT_INST_IRQN(n));                                            \
    }                                                                           \
                                                                                \
    static struct eth_infineon_pdl_data eth_infineon_pdl_data_##n = {           \
        .phy_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, phy_handle)),       \
        ETH_PERI_CLOCK_INIT(n)                                                  \
    };                                                                          \
                                                                                \
    static struct eth_infineon_pdl_config DT_CONST eth_infineon_pdl_cfg_##n = { \
        .base            = (ETH_Type*)DT_INST_REG_ADDR(n),                      \
        .mac_addr        = DT_INST_PROP(n, local_mac_address),                  \
        .phy_conn_type   = DT_INST_ENUM_IDX(n, phy_connection_type),            \
        .irq_config_func = eth_infineon_pdl_irq_config_##n,                     \
        .pincfg          = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                   \
        .clk_dst         = DT_INST_PROP(n, clk_dst),                            \
        .pdl_cb = {                                                             \
            .txcompletecb = eth_pdl_tx_complete_cb_##n,                         \
            .txerrorcb    = eth_pdl_tx_error_cb_##n,                            \
            .rxframecb    = eth_pdl_rx_cb_##n,                                  \
            .rxgetbuff    = eth_pdl_rxgetbuff_cb_##n,                           \
        },                                                                      \
    };                                                                          \
                                                                                \
    ETH_NET_DEVICE_DT_INST_DEFINE(n,                                            \
                                  eth_infineon_pdl_init, NULL,                  \
                                  &eth_infineon_pdl_data_##n,                   \
                                  &eth_infineon_pdl_cfg_##n,                    \
                                  CONFIG_ETH_INIT_PRIORITY,                     \
                                  &eth_infineon_pdl_api,                        \
                                  INFINEON_ETH_MAX_FRAME_SIZE);

DT_INST_FOREACH_STATUS_OKAY(ETH_INFINEON_PDL_DEVICE)

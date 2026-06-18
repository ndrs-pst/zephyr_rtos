/*
 * Copyright (c) 2018 Linaro Limited
 * Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arm_pl011
#define SBSA_COMPAT arm_sbsa_uart

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/irq.h>
#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif
#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>
#endif
#if defined(CONFIG_RESET)
#include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
#include <zephyr/drivers/clock_control.h>
#endif

#ifdef CONFIG_CPU_CORTEX_M
#include <cmsis_compiler.h>
#endif

#include "uart_pl011_registers.h"

#define PL011_USE_IRQ                                                                              \
	(CONFIG_UART_INTERRUPT_DRIVEN &&                                                           \
	 (DT_ANY_COMPAT_HAS_PROP_STATUS_OKAY(arm_pl011, interrupts) ||                             \
	  DT_ANY_COMPAT_HAS_PROP_STATUS_OKAY(arm_sbsa_uart, interrupts)))

struct pl011_config {
	DEVICE_MMIO_ROM;
#if defined(CONFIG_PINCTRL)
	const struct pinctrl_dev_config *pincfg;
#endif
#if defined(CONFIG_RESET)
	const struct reset_dt_spec reset;
#endif
#if defined(CONFIG_CLOCK_CONTROL)
	const struct device *clock_dev;
	clock_control_subsys_t clock_id;
#endif
#if PL011_USE_IRQ
	uart_irq_config_func_t irq_config_func;
#endif
	bool fifo_disable;
	int (*clk_enable_func)(const struct device *dev, uint32_t clk);
	int (*pwr_on_func)(const struct device *dev);
};

#ifdef CONFIG_UART_ASYNC_API
struct pl011_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	volatile size_t counter;
	size_t offset;
	int32_t timeout_us;
	struct k_work_delayable timeout_work;
	bool enabled;
};
#endif

/* Device data structure */
struct pl011_data {
	DEVICE_MMIO_RAM;
	struct uart_config uart_cfg;
	bool sbsa; /* SBSA mode */
	uint32_t clk_freq;

#if PL011_USE_IRQ
	volatile bool sw_call_txdrdy;
	uart_irq_callback_user_data_t irq_cb;
	struct k_spinlock irq_cb_lock;
	void *irq_cb_data;
#endif

#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t async_cb;
	void *async_cb_data;
	const struct device *dev;
	struct pl011_dma_stream tx_dma;
	struct pl011_dma_stream rx_dma;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
	struct k_spinlock async_lock;
#endif
};

/*
 * Include headers based on the presence of each specific compatible.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_pl011_uart)
#include "uart_pl011_ambiq.h"
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(raspberrypi_pico_uart)
#include "uart_pl011_raspberrypi_pico.h"
#endif

/*
 * Define generic helper functions only if the generic "arm,pl011"
 * compatible is found.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(arm_pl011)
static inline int pwr_on_arm_pl011(const struct device *dev)
{
	return 0;
}

static inline int clk_enable_arm_pl011(const struct device *dev, uint32_t clk)
{
	return 0;
}
#endif

/*
 * Conditionally define power management (PM) macros.
 */
#if defined(CONFIG_SOC_SERIES_APOLLO3X) || defined(CONFIG_SOC_SERIES_APOLLO5X)

/* For Apollo 3x and 5x, enable PM by defining macros that create and retrieve the PM device */
#define PM_INST_DEFINE(n) PM_DEVICE_DT_INST_DEFINE(n, uart_ambiq_pm_action);
#define PM_INST_GET(n)    PM_DEVICE_DT_INST_GET(n)

#else

/* For all others, define these macros to be empty and NULL, as there is no PM support for them*/
#define PM_INST_DEFINE(n)
#define PM_INST_GET(n) NULL
#endif

static inline void pl011_enable(const struct device *dev)
{
	get_uart(dev)->cr |= PL011_CR_UARTEN;
}

static inline void pl011_disable(const struct device *dev)
{
	get_uart(dev)->cr &= ~PL011_CR_UARTEN;
}

static inline void pl011_enable_fifo(const struct device *dev)
{
	get_uart(dev)->lcr_h |= PL011_LCRH_FEN;
}

static inline void pl011_disable_fifo(const struct device *dev)
{
	get_uart(dev)->lcr_h &= ~PL011_LCRH_FEN;
}

static void pl011_set_flow_control(const struct device *dev, bool rts, bool cts)
{
	volatile struct pl011_regs *uart = get_uart(dev);
	uint32_t cr = uart->cr;

	if (rts) {
		cr |= PL011_CR_RTSEn;
	} else {
		cr &= ~PL011_CR_RTSEn;
	}

	if (cts) {
		cr |= PL011_CR_CTSEn;
	} else {
		cr &= ~PL011_CR_CTSEn;
	}

	uart->cr = cr;
}

static int pl011_set_baudrate(const struct device *dev,
			      uint32_t clk, uint32_t baudrate)
{
	/* Avoiding float calculations, bauddiv is left shifted by 6 */
	uint64_t bauddiv = (((uint64_t)clk) << PL011_FBRD_WIDTH) / (baudrate * 16U);
	volatile struct pl011_regs *uart = get_uart(dev);

	/* Valid bauddiv value
	 * uart_clk (min) >= 16 x baud_rate (max)
	 * uart_clk (max) <= 16 x 65535 x baud_rate (min)
	 */
	if ((bauddiv < (1u << PL011_FBRD_WIDTH)) ||
	    (bauddiv > (65535u << PL011_FBRD_WIDTH))) {
		return -EINVAL;
	}

	uart->ibrd = bauddiv >> PL011_FBRD_WIDTH;
	uart->fbrd = bauddiv & ((1u << PL011_FBRD_WIDTH) - 1u);

	barrier_dmem_fence_full();

	/* In order to internally update the contents of ibrd or fbrd, a
	 * lcr_h write must always be performed at the end
	 * ARM DDI 0183F, Pg 3-13
	 */
	uart->lcr_h = uart->lcr_h;

	return 0;
}

static bool pl011_is_readable(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);
	uint32_t cr = uart->cr;

	if (!data->sbsa &&
	    (!(cr & PL011_CR_UARTEN) || !(cr & PL011_CR_RXE))) {
		return false;
	}

	return (uart->fr & PL011_FR_RXFE) == 0U;
}

static int pl011_poll_in(const struct device *dev, unsigned char *c)
{
	volatile struct pl011_regs *uart = get_uart(dev);

	if (!pl011_is_readable(dev)) {
		return -1;
	}

	/* got a character */
	*c = (unsigned char)uart->dr;

	return 0;
}

static void pl011_poll_out(const struct device *dev,
			   unsigned char c)
{
	volatile struct pl011_regs *uart = get_uart(dev);

	/* Wait for space in FIFO */
	while (uart->fr & PL011_FR_TXFF) {
		; /* Wait */
	}

	/* Send a character */
	uart->dr = (uint32_t)c;
}

static int pl011_err_check(const struct device *dev)
{
	int errors = 0;
	uint32_t rsr;

	/* Clear the latched error status first, then re-read.
	 * RSR latches errors from the most recent DR read.
	 * Writing any value to ECR (same address, write side) clears
	 * all error flags.  We clear first so that after this call
	 * returns, RSR is clean and ready to latch errors from the
	 * next DR read.
	 */
	rsr = get_uart(dev)->rsr;
	get_uart(dev)->rsr = 0;

	if (rsr & PL011_RSR_ECR_OE) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (rsr & PL011_RSR_ECR_BE) {
		errors |= UART_BREAK;
	}

	if (rsr & PL011_RSR_ECR_PE) {
		errors |= UART_ERROR_PARITY;
	}

	if (rsr & PL011_RSR_ECR_FE) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int pl011_runtime_configure_internal(const struct device *dev,
					    const struct uart_config *cfg,
					    bool disable)
{
	const struct pl011_config *config = dev->config;
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);
	uint32_t lcrh;
	int ret = -ENOTSUP;

	if (data->sbsa) {
		goto out;
	}

	if (disable) {
		pl011_disable(dev);
		pl011_disable_fifo(dev);
	}

	lcrh = uart->lcr_h & ~(PL011_LCRH_FORMAT_MASK | PL011_LCRH_STP2);

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		lcrh &= ~(BIT(1) | BIT(2));
		break;
	case UART_CFG_PARITY_ODD:
		lcrh |= PL011_LCRH_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		lcrh |= PL011_LCRH_PARTIY_EVEN;
		break;
	default:
		goto enable;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		lcrh &= ~(PL011_LCRH_STP2);
		break;
	case UART_CFG_STOP_BITS_2:
		lcrh |= PL011_LCRH_STP2;
		break;
	default:
		goto enable;
	}

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		lcrh |= PL011_LCRH_WLEN_SIZE(5) << PL011_LCRH_WLEN_SHIFT;
		break;
	case UART_CFG_DATA_BITS_6:
		lcrh |= PL011_LCRH_WLEN_SIZE(6) << PL011_LCRH_WLEN_SHIFT;
		break;
	case UART_CFG_DATA_BITS_7:
		lcrh |= PL011_LCRH_WLEN_SIZE(7) << PL011_LCRH_WLEN_SHIFT;
		break;
	case UART_CFG_DATA_BITS_8:
		lcrh |= PL011_LCRH_WLEN_SIZE(8) << PL011_LCRH_WLEN_SHIFT;
		break;
	default:
		goto enable;
	}

	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		pl011_set_flow_control(dev, false, false);
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		pl011_set_flow_control(dev, true, true);
		break;
	default:
		goto enable;
	}

	/* Set baud rate */
	ret = pl011_set_baudrate(dev, data->clk_freq, cfg->baudrate);
	if (ret != 0) {
		goto enable;
	}

	/* Update settings */
	uart->lcr_h = lcrh;

	memcpy(&data->uart_cfg, cfg, sizeof(data->uart_cfg));

enable:
	if (disable) {
		if (!config->fifo_disable) {
			pl011_enable_fifo(dev);
		}
		pl011_enable(dev);
	}

out:
	return ret;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

static int pl011_runtime_configure(const struct device *dev,
				   const struct uart_config *cfg)
{
	return pl011_runtime_configure_internal(dev, cfg, true);
}

static int pl011_runtime_config_get(const struct device *dev,
				    struct uart_config *cfg)
{
	struct pl011_data *data = dev->data;

	*cfg = data->uart_cfg;
	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#if PL011_USE_IRQ
static int pl011_fifo_fill(const struct device *dev,
			   const uint8_t *tx_data, int len)
{
	volatile struct pl011_regs *uart = get_uart(dev);
	int num_tx = 0U;

	while (!(uart->fr & PL011_FR_TXFF) && (len - num_tx > 0)) {
		uart->dr = tx_data[num_tx++];
	}
	return num_tx;
}

static int pl011_fifo_read(const struct device *dev,
			   uint8_t *rx_data, const int len)
{
	volatile struct pl011_regs *uart = get_uart(dev);
	int num_rx = 0U;

	while ((len - num_rx > 0) && !(uart->fr & PL011_FR_RXFE)) {
		rx_data[num_rx++] = uart->dr;
	}

	return num_rx;
}

static void pl011_irq_tx_enable(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);

	uart->imsc |= PL011_IMSC_TXIM;
	if (!data->sw_call_txdrdy) {
		return;
	}
	data->sw_call_txdrdy = false;

	/*
	 * Verify if the callback has been registered. Due to HW limitation, the
	 * first TX interrupt should be triggered by the software.
	 *
	 * PL011 TX interrupt is based on a transition through a level, rather
	 * than on the level itself[1]. So that, enable TX interrupt can not
	 * trigger TX interrupt if no data was filled to TX FIFO at the
	 * beginning.
	 *
	 * [1]: PrimeCell UART (PL011) Technical Reference Manual
	 *      functional-overview/interrupts
	 */
	if (!data->irq_cb) {
		return;
	}

	/*
	 * Execute callback while TX interrupt remains enabled. If
	 * uart_fifo_fill() is called with small amounts of data, the 1/8 TX
	 * FIFO threshold may never be reached, and the hardware TX interrupt
	 * will never trigger.
	 *
	 * Exit loop if CTS flow control is enabled and CTS is blocking
	 * transmission. CTS bit low means remote is not ready.
	 */
	while (uart->imsc & PL011_IMSC_TXIM) {
		/* If CTS flow control is enabled and CTS is blocking, exit loop */
		if ((uart->cr & PL011_CR_CTSEn) && !(uart->fr & PL011_FR_CTS)) {
			/* clear software flag to allow TX enable to be called again */
			data->sw_call_txdrdy = true;
			/* Enable CTS interrupt to resume when CTS clears */
			uart->imsc |= PL011_IMSC_CTSMIM;
			break;
		}
		K_SPINLOCK(&data->irq_cb_lock) {
			data->irq_cb(dev, data->irq_cb_data);
		}
	}
}

static void pl011_irq_tx_disable(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);

	data->sw_call_txdrdy = true;
	/* Clear TX and CTS interrupts */
	uart->imsc &= ~(PL011_IMSC_TXIM | PL011_IMSC_CTSMIM);
}

static int pl011_irq_tx_complete(const struct device *dev)
{
	/* Check for UART is busy transmitting data. */
	return ((get_uart(dev)->fr & PL011_FR_BUSY) == 0);
}

static int pl011_irq_tx_ready(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);

	if (!data->sbsa && !(uart->cr & PL011_CR_TXE)) {
		return false;
	}

	return ((uart->imsc & PL011_IMSC_TXIM) &&
		/* Check for TX interrupt status is set or TX FIFO is empty. */
		(uart->ris & PL011_RIS_TXRIS || uart->fr & PL011_FR_TXFE));
}

static void pl011_irq_rx_enable(const struct device *dev)
{
	get_uart(dev)->imsc |= PL011_IMSC_RXIM | PL011_IMSC_RTIM;
}

static void pl011_irq_rx_disable(const struct device *dev)
{
	get_uart(dev)->imsc &= ~(PL011_IMSC_RXIM | PL011_IMSC_RTIM);
}

static int pl011_irq_rx_ready(const struct device *dev)
{
	volatile struct pl011_regs *uart = get_uart(dev);
	struct pl011_data *data = dev->data;

	if (!data->sbsa && !(uart->cr & PL011_CR_RXE)) {
		return false;
	}

	return ((uart->imsc & PL011_IMSC_RXIM) &&
		(!(uart->fr & PL011_FR_RXFE)));
}

static void pl011_irq_err_enable(const struct device *dev)
{
	/* enable framing, parity, break, and overrun */
	get_uart(dev)->imsc |= PL011_IMSC_ERROR_MASK;
}

static void pl011_irq_err_disable(const struct device *dev)
{
	get_uart(dev)->imsc &= ~PL011_IMSC_ERROR_MASK;
}

static int pl011_irq_is_pending(const struct device *dev)
{
	volatile struct pl011_regs *uart = get_uart(dev);

	return pl011_irq_rx_ready(dev) || pl011_irq_tx_ready(dev) ||
	       (uart->mis & PL011_IMSC_ERROR_MASK);
}

static void pl011_irq_callback_set(const struct device *dev,
				   uart_irq_callback_user_data_t cb,
				   void *cb_data)
{
	struct pl011_data *data = dev->data;

	data->irq_cb = cb;
	data->irq_cb_data = cb_data;
}
#endif /* PL011_USE_IRQ */

#ifdef CONFIG_UART_ASYNC_API
static void pl011_async_user_callback(struct pl011_data *data, struct uart_event *evt)
{
	if (data->async_cb) {
		data->async_cb(data->dev, evt, data->async_cb_data);
	}
}

static void pl011_async_evt_tx_done(struct pl011_data *data)
{
	k_spinlock_key_t key;
	struct uart_event evt;

	key = k_spin_lock(&data->async_lock);
	if (data->tx_dma.buffer_length == 0U) {
		k_spin_unlock(&data->async_lock, key);
		return;
	}

	evt.type = UART_TX_DONE;
	evt.data.tx.buf = data->tx_dma.buffer;
	evt.data.tx.len = data->tx_dma.counter;

	data->tx_dma.buffer_length = 0;
	data->tx_dma.counter = 0;
	k_spin_unlock(&data->async_lock, key);
	pl011_async_user_callback(data, &evt);
}

static void pl011_async_evt_tx_abort(struct pl011_data *data)
{
	k_spinlock_key_t key;
	struct uart_event evt;

	key = k_spin_lock(&data->async_lock);
	if (data->tx_dma.buffer_length == 0U) {
		k_spin_unlock(&data->async_lock, key);
		return;
	}

	evt.type = UART_TX_ABORTED;
	evt.data.tx.buf = data->tx_dma.buffer;
	evt.data.tx.len = data->tx_dma.counter;

	data->tx_dma.buffer_length = 0;
	data->tx_dma.counter = 0;
	k_spin_unlock(&data->async_lock, key);
	pl011_async_user_callback(data, &evt);
}

static void pl011_async_evt_rx_rdy(struct pl011_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = data->rx_dma.buffer,
		.data.rx.len = data->rx_dma.counter - data->rx_dma.offset,
		.data.rx.offset = data->rx_dma.offset,
	};

	if (evt.data.rx.len > 0) {
		data->rx_dma.offset = data->rx_dma.counter;
		pl011_async_user_callback(data, &evt);
	}
}

static void pl011_async_evt_rx_buf_req(struct pl011_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	pl011_async_user_callback(data, &evt);
}

static void pl011_async_evt_rx_buf_rel(struct pl011_data *data, uint8_t *buf)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = buf,
	};

	pl011_async_user_callback(data, &evt);
}

static void pl011_async_evt_rx_disabled(struct pl011_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_DISABLED,
	};

	pl011_async_user_callback(data, &evt);
}

static void pl011_async_evt_rx_stopped(struct pl011_data *data, int reason)
{
	struct uart_event evt = {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = reason,
		.data.rx_stop.data.buf = data->rx_dma.buffer,
		.data.rx_stop.data.offset = data->rx_dma.offset,
		.data.rx_stop.data.len = data->rx_dma.counter - data->rx_dma.offset,
	};

	pl011_async_user_callback(data, &evt);
}

static void pl011_async_timer_start(struct k_work_delayable *work, int32_t timeout_us)
{
	if ((timeout_us != SYS_FOREVER_US) && (timeout_us != 0)) {
		k_work_reschedule(work, K_USEC(timeout_us));
	}
}

static inline void pl011_dma_tx_req_enable(const struct device *dev)
{
	get_uart(dev)->dmacr |= PL011_DMACR_TXDMAE;
}

static inline void pl011_dma_tx_req_disable(const struct device *dev)
{
	get_uart(dev)->dmacr &= ~PL011_DMACR_TXDMAE;
}

static inline void pl011_dma_rx_req_enable(const struct device *dev)
{
	struct pl011_data *data = dev->data;

	get_uart(dev)->dmacr |= PL011_DMACR_RXDMAE;
	data->rx_dma.enabled = true;
}

static inline void pl011_dma_rx_req_disable(const struct device *dev)
{
	struct pl011_data *data = dev->data;

	get_uart(dev)->dmacr &= ~PL011_DMACR_RXDMAE;
	data->rx_dma.enabled = false;
}

static void pl011_dma_rx_flush(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	struct dma_status stat;

	if ((data->rx_dma.dma_dev == NULL) || (data->rx_dma.buffer == NULL)) {
		return;
	}

	if (dma_get_status(data->rx_dma.dma_dev, data->rx_dma.dma_channel, &stat) == 0) {
		data->rx_dma.counter = data->rx_dma.buffer_length - stat.pending_length;
		if (data->rx_dma.counter > data->rx_dma.offset) {
			pl011_async_evt_rx_rdy(data);
		}
	}
}

static int pl011_async_tx_abort(const struct device *dev);
static int pl011_async_rx_disable(const struct device *dev);
static void pl011_async_rx_disable_finalize(const struct device *dev);

static void pl011_async_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct pl011_dma_stream *rx_dma = CONTAINER_OF(dwork, struct pl011_dma_stream, timeout_work);
	struct pl011_data *data = CONTAINER_OF(rx_dma, struct pl011_data, rx_dma);
	unsigned int key;

	key = irq_lock();
	if (data->rx_dma.enabled &&
	    (data->rx_dma.counter >= data->rx_dma.buffer_length) &&
	    (data->rx_next_buffer == NULL)) {
		irq_unlock(key);
		(void)pl011_async_rx_disable(data->dev);
		return;
	}
	pl011_dma_rx_flush(data->dev);
	irq_unlock(key);
}

static void pl011_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct pl011_dma_stream *tx_dma = CONTAINER_OF(dwork, struct pl011_dma_stream, timeout_work);
	struct pl011_data *data = CONTAINER_OF(tx_dma, struct pl011_data, tx_dma);

	(void)pl011_async_tx_abort(data->dev);
}

static void pl011_dma_tx_cb(const struct device *dma_dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct pl011_data *data = dev->data;
	struct dma_status dma_stat;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	(void)k_work_cancel_delayable(&data->tx_dma.timeout_work);
	pl011_dma_tx_req_disable(dev);

	if (status < 0) {
		pl011_async_evt_tx_abort(data);
		return;
	}

	if (dma_get_status(data->tx_dma.dma_dev, data->tx_dma.dma_channel, &dma_stat) == 0) {
		data->tx_dma.counter = data->tx_dma.buffer_length - dma_stat.pending_length;
	} else {
		data->tx_dma.counter = data->tx_dma.buffer_length;
	}

	pl011_async_evt_tx_done(data);
}

static void pl011_dma_rx_reload(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	struct pl011_dma_stream *rx_dma = &data->rx_dma;
	uint8_t *released;
	unsigned int key;
	int ret;

	key = irq_lock();
	released = rx_dma->buffer;
	rx_dma->buffer = data->rx_next_buffer;
	rx_dma->buffer_length = data->rx_next_buffer_len;
	rx_dma->counter = 0;
	rx_dma->offset = 0;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;
	irq_unlock(key);

	pl011_async_evt_rx_buf_rel(data, released);

	rx_dma->blk_cfg.dest_address = (uint32_t)rx_dma->buffer;
	rx_dma->blk_cfg.block_size = rx_dma->buffer_length;

	ret = dma_reload(rx_dma->dma_dev, rx_dma->dma_channel,
			 rx_dma->blk_cfg.source_address,
			 rx_dma->blk_cfg.dest_address,
			 rx_dma->blk_cfg.block_size);
	if (ret == 0) {
		ret = dma_start(rx_dma->dma_dev, rx_dma->dma_channel);
	}

	if (ret != 0) {
		pl011_async_evt_rx_stopped(data, ret);
		pl011_async_rx_disable_finalize(dev);
		return;
	}

	pl011_async_evt_rx_buf_req(data);
}

static void pl011_dma_rx_cb(const struct device *dma_dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct pl011_data *data = dev->data;
	bool has_next;
	unsigned int key;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	(void)k_work_cancel_delayable(&data->rx_dma.timeout_work);

	if (status < 0) {
		pl011_async_evt_rx_stopped(data, status);
		pl011_async_rx_disable_finalize(dev);
		return;
	}

	data->rx_dma.counter = data->rx_dma.buffer_length;
	pl011_async_evt_rx_rdy(data);

	key = irq_lock();
	has_next = (data->rx_next_buffer != NULL);
	irq_unlock(key);

	if (has_next) {
		pl011_dma_rx_reload(dev);
	} else {
		/* Buffer full, no next buffer — defer disable to avoid calling
		 * pl011_async_rx_disable from DMA ISR context. */
		(void)k_work_reschedule(&data->rx_dma.timeout_work, K_TICKS(1));
	}
}

static int pl011_async_callback_set(const struct device *dev,
				   uart_callback_t callback,
				   void *user_data)
{
	struct pl011_data *data = dev->data;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	data->async_cb = callback;
	data->async_cb_data = user_data;
#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS) && PL011_USE_IRQ
	data->irq_cb = NULL;
	data->irq_cb_data = NULL;
#endif

	return 0;
}

static int pl011_async_tx(const struct device *dev, const uint8_t *buf,
			  size_t len, int32_t timeout)
{
	struct pl011_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	if (data->tx_dma.dma_dev == NULL) {
		return -ENODEV;
	}

	if ((buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->async_lock);
	if (data->tx_dma.buffer_length != 0U) {
		k_spin_unlock(&data->async_lock, key);
		return -EBUSY;
	}

	data->tx_dma.buffer = (uint8_t *)buf;
	data->tx_dma.buffer_length = len;
	data->tx_dma.counter = 0U;
	data->tx_dma.timeout_us = timeout;
	k_spin_unlock(&data->async_lock, key);

	data->tx_dma.blk_cfg.source_address = (uint32_t)data->tx_dma.buffer;
	data->tx_dma.blk_cfg.block_size = len;

	ret = dma_config(data->tx_dma.dma_dev, data->tx_dma.dma_channel,
			 &data->tx_dma.dma_cfg);
	if (ret != 0) {
		key = k_spin_lock(&data->async_lock);
		data->tx_dma.buffer_length = 0U;
		k_spin_unlock(&data->async_lock, key);
		return -EINVAL;
	}

	ret = dma_start(data->tx_dma.dma_dev, data->tx_dma.dma_channel);
	if (ret != 0) {
		key = k_spin_lock(&data->async_lock);
		data->tx_dma.buffer_length = 0U;
		k_spin_unlock(&data->async_lock, key);
		return -EIO;
	}

	pl011_dma_tx_req_enable(dev);
	pl011_async_timer_start(&data->tx_dma.timeout_work, timeout);

	return 0;
}

static int pl011_async_tx_abort(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	struct dma_status stat;
	k_spinlock_key_t key;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->async_lock);
	if ((data->tx_dma.dma_dev == NULL) || (data->tx_dma.buffer_length == 0U)) {
		k_spin_unlock(&data->async_lock, key);
		return -EFAULT;
	}
	k_spin_unlock(&data->async_lock, key);

	(void)k_work_cancel_delayable(&data->tx_dma.timeout_work);
	if (dma_get_status(data->tx_dma.dma_dev, data->tx_dma.dma_channel, &stat) == 0) {
		data->tx_dma.counter = data->tx_dma.buffer_length - stat.pending_length;
	}

	(void)dma_stop(data->tx_dma.dma_dev, data->tx_dma.dma_channel);
	pl011_dma_tx_req_disable(dev);
	pl011_async_evt_tx_abort(data);

	return 0;
}

static int pl011_async_rx_enable(const struct device *dev, uint8_t *buf,
				 size_t len, int32_t timeout)
{
	struct pl011_data *data = dev->data;
	int ret;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	if (data->rx_dma.dma_dev == NULL) {
		return -ENODEV;
	}

	if ((buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	if (data->rx_dma.enabled) {
		return -EBUSY;
	}

	data->rx_dma.buffer = buf;
	data->rx_dma.buffer_length = len;
	data->rx_dma.counter = 0U;
	data->rx_dma.offset = 0U;
	data->rx_dma.timeout_us = timeout;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;

	data->rx_dma.blk_cfg.dest_address = (uint32_t)buf;
	data->rx_dma.blk_cfg.block_size = len;

	ret = dma_config(data->rx_dma.dma_dev, data->rx_dma.dma_channel,
			 &data->rx_dma.dma_cfg);
	if (ret != 0) {
		return -EINVAL;
	}

	ret = dma_start(data->rx_dma.dma_dev, data->rx_dma.dma_channel);
	if (ret != 0) {
		return -EIO;
	}

	pl011_dma_rx_req_enable(dev);
	get_uart(dev)->imsc |= (PL011_IMSC_RTIM | PL011_IMSC_ERROR_MASK);
	pl011_async_timer_start(&data->rx_dma.timeout_work, timeout);
	pl011_async_evt_rx_buf_req(data);

	return 0;
}

static int pl011_async_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct pl011_data *data = dev->data;
	int ret;
	unsigned int key;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	key = irq_lock();

	if (data->rx_next_buffer != NULL) {
		ret = -EBUSY;
	} else if (!data->rx_dma.enabled) {
		ret = -EACCES;
	} else if ((buf == NULL) || (len == 0U)) {
		ret = -EINVAL;
	} else {
		data->rx_next_buffer = buf;
		data->rx_next_buffer_len = len;
		ret = 0;
	}

	irq_unlock(key);

	return ret;
}

/*
 * Shared cleanup path called after UART_RX_STOPPED (DMA error, UART
 * error interrupt) or as the tail of pl011_async_rx_disable.  Must NOT be
 * called when rx_dma.enabled is already false — callers are responsible for
 * the enabled guard.  Safe from DMA-callback and UART-ISR contexts.
 */
static void pl011_async_rx_disable_finalize(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	uint8_t *next;
	unsigned int key;

	(void)k_work_cancel_delayable(&data->rx_dma.timeout_work);
	get_uart(dev)->imsc &= ~(PL011_IMSC_RTIM | PL011_IMSC_ERROR_MASK);

	/* Clear the enabled flag and DMACR bit atomically */
	key = irq_lock();
	pl011_dma_rx_req_disable(dev);
	next = data->rx_next_buffer;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
	irq_unlock(key);

	(void)dma_stop(data->rx_dma.dma_dev, data->rx_dma.dma_channel);

	pl011_async_evt_rx_buf_rel(data, data->rx_dma.buffer);
	data->rx_dma.buffer = NULL; /* Drop stale pointer immediately */

	if (next != NULL) {
		pl011_async_evt_rx_buf_rel(data, next);
	}

	pl011_async_evt_rx_disabled(data);
}

static int pl011_async_rx_disable(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	unsigned int key;

	if (data->sbsa) {
		return -ENOTSUP;
	}

	key = irq_lock();
	if (!data->rx_dma.enabled) {
		irq_unlock(key);
		pl011_async_evt_rx_disabled(data);
		return -EFAULT;
	}
	pl011_dma_rx_req_disable(dev); /* Clears DMACR.RXDMAE + sets enabled=false */
	irq_unlock(key);

	(void)k_work_cancel_delayable(&data->rx_dma.timeout_work);

	/* Flush while DMA channel is still alive (DMACR cleared, not yet stopped) */
	pl011_dma_rx_flush(dev);
	get_uart(dev)->imsc &= ~(PL011_IMSC_RTIM | PL011_IMSC_ERROR_MASK);
	(void)dma_stop(data->rx_dma.dma_dev, data->rx_dma.dma_channel);

	pl011_async_evt_rx_buf_rel(data, data->rx_dma.buffer);
	data->rx_dma.buffer = NULL; /* Drop stale pointer immediately */

	key = irq_lock();
	if (data->rx_next_buffer != NULL) {
		uint8_t *next = data->rx_next_buffer;

		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0U;
		irq_unlock(key);
		pl011_async_evt_rx_buf_rel(data, next);
	} else {
		data->rx_next_buffer_len = 0U;
		irq_unlock(key);
	}

	pl011_async_evt_rx_disabled(data);

	return 0;
}

static int pl011_async_init(const struct device *dev)
{
	volatile struct pl011_regs *uart = get_uart(dev);
	struct pl011_data *data = dev->data;

	BUILD_ASSERT(sizeof(uintptr_t) <= sizeof(uint32_t),
		     "PL011 async DMA requires a 32-bit address space");

	data->dev = dev;

	if ((data->rx_dma.dma_dev != NULL) && !device_is_ready(data->rx_dma.dma_dev)) {
		return -ENODEV;
	}

	if ((data->tx_dma.dma_dev != NULL) && !device_is_ready(data->tx_dma.dma_dev)) {
		return -ENODEV;
	}

	uart->dmacr &= ~(PL011_DMACR_TXDMAE | PL011_DMACR_RXDMAE);

	k_work_init_delayable(&data->rx_dma.timeout_work, pl011_async_rx_timeout);
	k_work_init_delayable(&data->tx_dma.timeout_work, pl011_async_tx_timeout);

	if (data->rx_dma.dma_dev != NULL) {
		memset(&data->rx_dma.blk_cfg, 0, sizeof(data->rx_dma.blk_cfg));
		data->rx_dma.blk_cfg.source_address = (uint32_t)&uart->dr;
		data->rx_dma.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		data->rx_dma.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		data->rx_dma.dma_cfg.head_block = &data->rx_dma.blk_cfg;
		data->rx_dma.dma_cfg.user_data = (void *)dev;
		data->rx_dma.dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		data->rx_dma.dma_cfg.source_data_size = 1;
		data->rx_dma.dma_cfg.dest_data_size = 1;
		data->rx_dma.dma_cfg.source_burst_length = 1;
		data->rx_dma.dma_cfg.dest_burst_length = 1;
		data->rx_dma.dma_cfg.block_count = 1;
		data->rx_dma.dma_cfg.dma_callback = pl011_dma_rx_cb;
	}

	if (data->tx_dma.dma_dev != NULL) {
		memset(&data->tx_dma.blk_cfg, 0, sizeof(data->tx_dma.blk_cfg));
		data->tx_dma.blk_cfg.dest_address = (uint32_t)&uart->dr;
		data->tx_dma.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		data->tx_dma.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		data->tx_dma.dma_cfg.head_block = &data->tx_dma.blk_cfg;
		data->tx_dma.dma_cfg.user_data = (void *)dev;
		data->tx_dma.dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		data->tx_dma.dma_cfg.source_data_size = 1;
		data->tx_dma.dma_cfg.dest_data_size = 1;
		data->tx_dma.dma_cfg.source_burst_length = 1;
		data->tx_dma.dma_cfg.dest_burst_length = 1;
		data->tx_dma.dma_cfg.block_count = 1;
		data->tx_dma.dma_cfg.dma_callback = pl011_dma_tx_cb;
	}

	return 0;
}
#endif /* CONFIG_UART_ASYNC_API */

static __maybe_unused DEVICE_API(uart, pl011_driver_api_noirq) = {
	.poll_in = pl011_poll_in,
	.poll_out = pl011_poll_out,
	.err_check = pl011_err_check,

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = pl011_runtime_configure,
	.config_get = pl011_runtime_config_get,
#endif

#ifdef CONFIG_UART_ASYNC_API
	.callback_set = pl011_async_callback_set,
	.tx = pl011_async_tx,
	.tx_abort = pl011_async_tx_abort,
	.rx_enable = pl011_async_rx_enable,
	.rx_buf_rsp = pl011_async_rx_buf_rsp,
	.rx_disable = pl011_async_rx_disable,
#endif
};

#if PL011_USE_IRQ
static DEVICE_API(uart, pl011_driver_api) = {
	.poll_in = pl011_poll_in,
	.poll_out = pl011_poll_out,
	.err_check = pl011_err_check,

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = pl011_runtime_configure,
	.config_get = pl011_runtime_config_get,
#endif

	.fifo_fill = pl011_fifo_fill,
	.fifo_read = pl011_fifo_read,
	.irq_tx_enable = pl011_irq_tx_enable,
	.irq_tx_disable = pl011_irq_tx_disable,
	.irq_tx_ready = pl011_irq_tx_ready,
	.irq_rx_enable = pl011_irq_rx_enable,
	.irq_rx_disable = pl011_irq_rx_disable,
	.irq_tx_complete = pl011_irq_tx_complete,
	.irq_rx_ready = pl011_irq_rx_ready,
	.irq_err_enable = pl011_irq_err_enable,
	.irq_err_disable = pl011_irq_err_disable,
	.irq_is_pending = pl011_irq_is_pending,
	.irq_callback_set = pl011_irq_callback_set,

#ifdef CONFIG_UART_ASYNC_API
	.callback_set = pl011_async_callback_set,
	.tx = pl011_async_tx,
	.tx_abort = pl011_async_tx_abort,
	.rx_enable = pl011_async_rx_enable,
	.rx_buf_rsp = pl011_async_rx_buf_rsp,
	.rx_disable = pl011_async_rx_disable,
#endif
};
#endif /* PL011_USE_IRQ */

static int pl011_init(const struct device *dev)
{
	const struct pl011_config *config = dev->config;
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* Must be placed after DEVICE_MMIO_MAP */
	uart = get_uart(dev);

#if defined(CONFIG_RESET)
	if (config->reset.dev) {
		ret = reset_line_toggle_dt(&config->reset);
		if (ret) {
			return ret;
		}
	}
#endif

#if defined(CONFIG_CLOCK_CONTROL)
	if (config->clock_dev) {
		clock_control_on(config->clock_dev, config->clock_id);
		clock_control_get_rate(config->clock_dev, config->clock_id, &data->clk_freq);
	}
#endif

	/*
	 * If working in SBSA mode, we assume that UART is already configured,
	 * or does not require configuration at all (if UART is emulated by
	 * virtualization software).
	 */
	if (!data->sbsa) {
#if defined(CONFIG_PINCTRL)
		ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret) {
			return ret;
		}
#endif
		/* Call vendor-specific function to power on the peripheral */
		if (config->pwr_on_func != NULL) {
			ret = config->pwr_on_func(dev);
		}

		/* disable the uart */
		pl011_disable(dev);
		pl011_disable_fifo(dev);

		/* Call vendor-specific function to enable clock for the peripheral */
		if (config->clk_enable_func != NULL) {
			ret = config->clk_enable_func(dev, data->clk_freq);
			if (ret) {
				return ret;
			}
		}

		pl011_runtime_configure_internal(dev, &data->uart_cfg, false);

		/* Setting transmit and receive interrupt FIFO level */
		uart->ifls = FIELD_PREP(PL011_IFLS_TXIFLSEL_M, TXIFLSEL_1_8_FULL) |
			     FIELD_PREP(PL011_IFLS_RXIFLSEL_M, RXIFLSEL_1_2_FULL);

		/* Enabling the FIFOs */
		if (!config->fifo_disable) {
			pl011_enable_fifo(dev);
		}
	}
	/* initialize all IRQs as masked */
	uart->imsc = 0U;
	uart->icr = PL011_IMSC_MASK_ALL;

	if (!data->sbsa) {
		uart->dmacr = 0U;
		barrier_isync_fence_full();
		uart->cr &= ~PL011_CR_SIREN;
		uart->cr |= PL011_CR_RXE | PL011_CR_TXE;
		barrier_isync_fence_full();
	}

#if PL011_USE_IRQ
	if (config->irq_config_func) {
		config->irq_config_func(dev);
		data->sw_call_txdrdy = true;
	}
#endif

#ifdef CONFIG_UART_ASYNC_API
	if (!data->sbsa) {
		ret = pl011_async_init(dev);
		if (ret != 0) {
			return ret;
		}
	}
#endif

	if (!data->sbsa) {
		pl011_enable(dev);
	}

	return 0;
}

#define COMPAT_SPECIFIC_FUNC_NAME(prefix, name) _CONCAT(prefix, name)

/*
 * The first element of compatible is used to determine the type.
 * When compatible defines as "ambiq,pl011-uart", "arm,pl011",
 * this macro expands to pwr_on_ambiq_pl011_uart.
 */
#define COMPAT_SPECIFIC_PWR_ON_FUNC(n)                                                             \
	COMPAT_SPECIFIC_FUNC_NAME(pwr_on_, DT_INST_STRING_TOKEN_BY_IDX(n, compatible, 0))

/*
 * The first element of compatible is used to determine the type.
 * When compatible defines as "ambiq,pl011-uart", "arm,pl011",
 * this macro expands to clk_enable_ambiq_pl011_uart.
 */
#define COMPAT_SPECIFIC_CLK_ENABLE_FUNC(n)                                                         \
	COMPAT_SPECIFIC_FUNC_NAME(clk_enable_, DT_INST_STRING_TOKEN_BY_IDX(n, compatible, 0))

#define COMPAT_SPECIFIC_CLOCK_CTLR_SUBSYS_CELL(n)                                                  \
	_CONCAT(DT_INST_STRING_UPPER_TOKEN_BY_IDX(n, compatible, 0), _CLOCK_CTLR_SUBSYS_CELL)

#if defined(CONFIG_PINCTRL)
#define PINCTRL_DEFINE(n) PINCTRL_DT_INST_DEFINE(n);
#define PINCTRL_INIT(n)   .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),
#else
#define PINCTRL_DEFINE(n)
#define PINCTRL_INIT(n)
#endif /* CONFIG_PINCTRL */

#if defined(CONFIG_RESET)
#define RESET_INIT(n)                                                                              \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n),))
#else
#define RESET_INIT(n)
#endif

#define CLOCK_INIT(n)                                                                              \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(n), fixed_clock), (),                   \
		    (.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                           \
		     .clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n,                    \
				  COMPAT_SPECIFIC_CLOCK_CTLR_SUBSYS_CELL(n)),))

#define IRQ_CONFIG_FUNC_INIT(n)                                                                    \
	IF_ENABLED(PL011_NODE_USE_IRQ(n), (.irq_config_func = pl011_irq_config_func_##n,))

#ifdef CONFIG_UART_ASYNC_API
#define PL011_DMA_CHANNEL_INIT(n, name, direction)                                                 \
	.name##_dma = {                                                                            \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, name),                                        \
			(.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name)),             \
			 .dma_channel = DT_DMAS_CELL_BY_NAME_OR(DT_DRV_INST(n), name, channel, 0), \
			 .dma_cfg = {.channel_direction = direction, .complete_callback_en = 1}),  \
			(.dma_dev = NULL))                                                         \
	},
#else
#define PL011_DMA_CHANNEL_INIT(n, name, direction)
#endif

#if PL011_USE_IRQ
void pl011_isr(const struct device *dev)
{
	struct pl011_data *data = dev->data;
	volatile struct pl011_regs *uart = get_uart(dev);

#ifdef CONFIG_UART_ASYNC_API
	if (data->async_cb &&
	    (data->irq_cb == NULL) &&
	    data->rx_dma.enabled) {
		if (uart->mis & PL011_IMSC_RTIM) {
			uart->icr = PL011_IMSC_RTIM;
			pl011_dma_rx_flush(dev);
			pl011_async_timer_start(&data->rx_dma.timeout_work,
						data->rx_dma.timeout_us);
		}

		if (uart->mis & PL011_IMSC_ERROR_MASK) {
			int err = pl011_err_check(dev);

			uart->icr = PL011_IMSC_ERROR_MASK;
			if (err != 0) {
				pl011_async_evt_rx_stopped(data, err);
				pl011_async_rx_disable_finalize(dev);
			}
		}
	}
#endif

	/* Clear CTS modem status interrupt and disable it */
	if (uart->mis & PL011_IMSC_CTSMIM) {
		uart->icr = PL011_IMSC_CTSMIM;
		uart->imsc &= ~PL011_IMSC_CTSMIM;
	}

	/* Clear error interrupts (OE, BE, PE, FE) so they don't
	 * re-fire endlessly.  The error status is still available
	 * via uart_err_check() which reads RSR.
	 */
	if (uart->mis & PL011_IMSC_ERROR_MASK) {
		uart->icr = uart->mis & PL011_IMSC_ERROR_MASK;
	}

	/* Verify if the callback has been registered */
	if (data->irq_cb) {
		K_SPINLOCK(&data->irq_cb_lock) {
			data->irq_cb(dev, data->irq_cb_data);
		}
	}
}
#endif /* PL011_USE_IRQ */

#define PL011_IRQ_CONFIG_FUNC_BODY(n, prop, i)                                                     \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQ_BY_IDX(n, i, irq),                                              \
			    DT_IRQ_BY_IDX(n, i, priority),                                         \
			    pl011_isr,                                                             \
			    DEVICE_DT_GET(n),                                                      \
			    0);                                                                    \
		irq_enable(DT_IRQ_BY_IDX(n, i, irq));                                              \
	}

#define PL011_NODE_USE_IRQ(n)                                                                      \
	COND_CODE_1(CONFIG_UART_INTERRUPT_DRIVEN, (DT_INST_NODE_HAS_PROP(n, interrupts)), (0))

#define PL011_DEVICE_API(n)                                                                        \
	COND_CODE_1(CONFIG_UART_INTERRUPT_DRIVEN,                                                  \
		    (COND_CODE_1(DT_INST_NODE_HAS_PROP(n, interrupts),                             \
				 (&pl011_driver_api), (&pl011_driver_api_noirq))),                 \
		    (&pl011_driver_api_noirq))

#define PL011_CONFIG_PORT(n)                                                                       \
	IF_ENABLED(PL011_NODE_USE_IRQ(n), (                                                        \
		static void pl011_irq_config_func_##n(const struct device *dev)                    \
		{                                                                                  \
			DT_INST_FOREACH_PROP_ELEM(n, interrupt_names,                              \
			PL011_IRQ_CONFIG_FUNC_BODY)                                                \
		};                                                                                 \
	))                                                                                         \
                                                                                                   \
	static struct pl011_config pl011_cfg_port_##n = {                                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		CLOCK_INIT(n)                                                                      \
		PINCTRL_INIT(n)                                                                    \
		IRQ_CONFIG_FUNC_INIT(n)                                                            \
		.fifo_disable = DT_INST_PROP(n, fifo_disable),                                     \
		.clk_enable_func = COMPAT_SPECIFIC_CLK_ENABLE_FUNC(n),                             \
		.pwr_on_func = COMPAT_SPECIFIC_PWR_ON_FUNC(n),                                     \
	};

#define PL011_INIT(n)                                                                              \
	PINCTRL_DEFINE(n)                                                                          \
	PL011_CONFIG_PORT(n)                                                                       \
	PM_INST_DEFINE(n)                                                                          \
                                                                                                   \
	static struct pl011_data pl011_data_port_##n = {                                           \
		.uart_cfg =                                                                        \
			{                                                                          \
				.baudrate = DT_INST_PROP(n, current_speed),                        \
				.parity = UART_CFG_PARITY_NONE,                                    \
				.stop_bits = UART_CFG_STOP_BITS_1,                                 \
				.data_bits = UART_CFG_DATA_BITS_8,                                 \
				.flow_ctrl = DT_INST_PROP(n, hw_flow_control)                      \
						     ? UART_CFG_FLOW_CTRL_RTS_CTS                  \
						     : UART_CFG_FLOW_CTRL_NONE,                    \
			},                                                                         \
		.clk_freq = COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(n), fixed_clock),   \
				    (DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency)), (0)),   \
			 PL011_DMA_CHANNEL_INIT(n, rx, PERIPHERAL_TO_MEMORY)                       \
				 PL011_DMA_CHANNEL_INIT(n, tx, MEMORY_TO_PERIPHERAL)               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pl011_init, PM_INST_GET(n), &pl011_data_port_##n,                 \
			      &pl011_cfg_port_##n, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,      \
			      PL011_DEVICE_API(n));

DT_INST_FOREACH_STATUS_OKAY(PL011_INIT)

#ifdef CONFIG_UART_PL011_SBSA

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT SBSA_COMPAT

#define PL011_SBSA_CONFIG_PORT(n)                                                                  \
	IF_ENABLED(PL011_NODE_USE_IRQ(n), (                                                        \
		static void pl011_irq_config_func_sbsa_##n(const struct device *dev)               \
		{                                                                                  \
			DT_INST_FOREACH_PROP_ELEM(n, interrupt_names, PL011_IRQ_CONFIG_FUNC_BODY)  \
		};                                                                                 \
	))                                                                                         \
                                                                                                   \
	static struct pl011_config pl011_cfg_sbsa_##n = {                                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		IF_ENABLED(PL011_NODE_USE_IRQ(n),                                                  \
			   (.irq_config_func = pl011_irq_config_func_sbsa_##n,))                   \
	};

#define PL011_SBSA_INIT(n)                                                                         \
	PL011_SBSA_CONFIG_PORT(n)                                                                  \
                                                                                                   \
	static struct pl011_data pl011_data_sbsa_##n = {                                           \
		.sbsa = true,                                                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pl011_init,                                                       \
			      NULL,                                                                \
			      &pl011_data_sbsa_##n,                                                \
			      &pl011_cfg_sbsa_##n,                                                 \
			      PRE_KERNEL_1,                                                        \
			      CONFIG_SERIAL_INIT_PRIORITY,                                         \
			      PL011_DEVICE_API(n));

DT_INST_FOREACH_STATUS_OKAY(PL011_SBSA_INIT)

#endif /* CONFIG_UART_PL011_SBSA */

// SPDX-License-Identifier: GPL-2.0
//
// Apple SoC SPI device driver
//
// Copyright The Asahi Linux Contributors
//
// Based on spi-sifive.c, Copyright 2018 SiFive, Inc.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define APPLE_SPI_CTRL			0x000
#define APPLE_SPI_CTRL_RUN		BIT(0)
#define APPLE_SPI_CTRL_TX_RESET		BIT(2)
#define APPLE_SPI_CTRL_RX_RESET		BIT(3)
#define APPLE_SPI_CTRL_T8010_ENABLE \
	(APPLE_SPI_CTRL_RUN | APPLE_SPI_CTRL_TX_RESET | APPLE_SPI_CTRL_RX_RESET)

#define APPLE_SPI_CFG			0x004
#define APPLE_SPI_CFG_CPHA		BIT(1)
#define APPLE_SPI_CFG_CPOL		BIT(2)
#define APPLE_SPI_CFG_MODE		GENMASK(6, 5)
#define APPLE_SPI_CFG_MODE_IRQ		1
#define APPLE_SPI_CFG_MODE_DMA		2
#define APPLE_SPI_CFG_LSB_FIRST		BIT(13)
#define APPLE_SPI_CFG_WORD_SIZE		GENMASK(16, 15)
#define APPLE_SPI_CFG_WORD_SIZE_8B	0
#define APPLE_SPI_CFG_FIFO_THRESH	GENMASK(18, 17)
#define APPLE_SPI_CFG_FIFO_THRESH_8B	0

#define APPLE_SPI_CFG_T8010_BASE	0x0010401e
#define APPLE_SPI_CFG_T8010_PIOEN	BIT(5)
#define APPLE_SPI_CFG_T8010_16BIT	BIT(15)

#define APPLE_SPI_T8010_DESC_CFG_BASE		0x4000
#define APPLE_SPI_T8010_DESC_CFG_DEFAULT	0x2000
#define APPLE_SPI_T8010_DESC_WORD_BITS		0x0f
#define APPLE_SPI_T8010_DESC_STATUS_CFG		0x40000f
#define APPLE_SPI_T8010_DESC_CPOL_LOW		0x18
#define APPLE_SPI_T8010_DESC_CPOL_HIGH		0x1c
#define APPLE_SPI_T8010_DESC_CPOL_EXTRA		BIT(20)

#define APPLE_SPI_STATUS		0x008
#define APPLE_SPI_STATUS_T8010_RXRDY	BIT(0)
#define APPLE_SPI_STATUS_T8010_TXEMPTY	BIT(1)
#define APPLE_SPI_STATUS_T8010_TXFIFO	GENMASK(10, 6)
#define APPLE_SPI_STATUS_T8010_RXFIFO	GENMASK(15, 11)
#define APPLE_SPI_STATUS_T8010_COMPL	BIT(22)

#define APPLE_SPI_PIN			0x00c
#define APPLE_SPI_PIN_CS		BIT(1)

#define APPLE_SPI_TXDATA		0x010
#define APPLE_SPI_RXDATA		0x020
#define APPLE_SPI_CLKDIV		0x030
#define APPLE_SPI_CLKDIV_MAX		0x7ff
#define APPLE_SPI_RXCNT			0x034
#define APPLE_SPI_WORD_DELAY		0x038
#define APPLE_SPI_TXCNT			0x04c

#define APPLE_SPI_FIFOSTAT		0x10c
#define APPLE_SPI_FIFOSTAT_LEVEL_TX	GENMASK(15, 8)
#define APPLE_SPI_FIFOSTAT_LEVEL_RX	GENMASK(31, 24)

#define APPLE_SPI_IE_XFER		0x130
#define APPLE_SPI_IF_XFER		0x134
#define APPLE_SPI_XFER_RXCOMPLETE	BIT(0)
#define APPLE_SPI_XFER_TXCOMPLETE	BIT(1)

#define APPLE_SPI_IE_FIFO		0x138
#define APPLE_SPI_IF_FIFO		0x13c
#define APPLE_SPI_FIFO_RXTHRESH		BIT(4)
#define APPLE_SPI_FIFO_TXTHRESH		BIT(5)
#define APPLE_SPI_FIFO_RXUNDERRUN	BIT(16)
#define APPLE_SPI_FIFO_TXOVERFLOW	BIT(17)

#define APPLE_SPI_SHIFTCFG		0x150
#define APPLE_SPI_SHIFTCFG_BITS		GENMASK(21, 16)
#define APPLE_SPI_SHIFTCFG_OVERRIDE_CS	BIT(24)

#define APPLE_SPI_PINCFG		0x154
#define APPLE_SPI_PINCFG_KEEP_CS	BIT(1)
#define APPLE_SPI_PINCFG_CS_IDLE_VAL	BIT(9)

#define APPLE_SPI_DELAY_PRE		0x160
#define APPLE_SPI_DELAY_POST		0x168

#define APPLE_SPI_FIFO_DEPTH		16

/*
 * The slowest refclock available is 24MHz, the highest divider is 0x7ff,
 * the largest word size is 32 bits, the FIFO depth is 16, the maximum
 * intra-word delay is 0xffff refclocks. So the maximum time a transfer
 * cycle can take is:
 *
 * (0x7ff * 32 + 0xffff) * 16 / 24e6 Hz ~= 87ms
 *
 * Double it and round it up to 200ms for good measure.
 */
#define APPLE_SPI_TIMEOUT_MS		200
#define APPLE_SPI_T8010_TIMEOUT_MS	1000

struct apple_spi_hw {
	bool legacy_status;
	unsigned int min_clk_div;
	u32 default_word_delay;
};

struct apple_spi {
	void __iomem      *regs;        /* MMIO register address */
	struct clk        *clk;         /* bus clock */
	struct completion done;         /* wake-up from interrupt */
	struct completion dma_done;     /* wake-up from DMA completion */
	struct dmaengine_result dma_result;
	const struct apple_spi_hw *hw;
	u32 word_delay;
};

static inline void reg_write(struct apple_spi *spi, int offset, u32 value)
{
	writel_relaxed(value, spi->regs + offset);
}

static inline u32 reg_read(struct apple_spi *spi, int offset)
{
	return readl_relaxed(spi->regs + offset);
}

static inline void reg_mask(struct apple_spi *spi, int offset, u32 clear, u32 set)
{
	u32 val = reg_read(spi, offset);

	val &= ~clear;
	val |= set;
	reg_write(spi, offset, val);
}

static bool apple_spi_t8010_uses_descriptor(struct spi_transfer *t,
					    unsigned int words)
{
	if (t->tx_buf && t->rx_buf)
		return words <= APPLE_SPI_FIFO_DEPTH;

	/* Short transmit-only commands use the descriptor register format. */
	return t->tx_buf && !t->rx_buf &&
	       (words == 2 || words == 8 || words == APPLE_SPI_FIFO_DEPTH);
}

static void apple_spi_t8010_write_status_config(struct apple_spi *spi,
						bool use_descriptor)
{
	if (use_descriptor)
		reg_write(spi, APPLE_SPI_STATUS,
			  APPLE_SPI_T8010_DESC_STATUS_CFG);
}

static void apple_spi_init(struct apple_spi *spi)
{
	if (spi->hw->legacy_status) {
		reg_write(spi, APPLE_SPI_CTRL, 0);
		reg_write(spi, APPLE_SPI_PIN, 0);
		reg_write(spi, APPLE_SPI_STATUS, ~0);
		reg_write(spi, APPLE_SPI_IF_XFER, ~0);
		reg_write(spi, APPLE_SPI_IF_FIFO, ~0);
		reg_write(spi, APPLE_SPI_WORD_DELAY, spi->word_delay);
		reg_write(spi, APPLE_SPI_CFG, APPLE_SPI_CFG_T8010_BASE);
		return;
	}

	/* Set CS high (inactive) and disable override and auto-CS. */
	reg_write(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS);
	reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_OVERRIDE_CS, 0);
	reg_mask(spi, APPLE_SPI_PINCFG, APPLE_SPI_PINCFG_CS_IDLE_VAL,
		 APPLE_SPI_PINCFG_KEEP_CS);

	reg_write(spi, APPLE_SPI_CTRL,
		  APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);
	reg_write(spi, APPLE_SPI_CFG,
		  FIELD_PREP(APPLE_SPI_CFG_FIFO_THRESH,
			     APPLE_SPI_CFG_FIFO_THRESH_8B) |
		  FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_IRQ) |
		  FIELD_PREP(APPLE_SPI_CFG_WORD_SIZE,
			     APPLE_SPI_CFG_WORD_SIZE_8B));
	reg_write(spi, APPLE_SPI_IE_FIFO, 0);
	reg_write(spi, APPLE_SPI_IE_XFER, 0);
	reg_write(spi, APPLE_SPI_DELAY_PRE, 0);
	reg_write(spi, APPLE_SPI_DELAY_POST, 0);
}

static int apple_spi_prepare_message(struct spi_controller *ctlr,
				     struct spi_message *msg)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	struct spi_device *device = msg->spi;
	u32 cfg;

	cfg = (device->mode & SPI_CPHA ? APPLE_SPI_CFG_CPHA : 0) |
	      (device->mode & SPI_CPOL ? APPLE_SPI_CFG_CPOL : 0) |
	      (device->mode & SPI_LSB_FIRST ? APPLE_SPI_CFG_LSB_FIRST : 0);
	reg_mask(spi, APPLE_SPI_CFG,
		 APPLE_SPI_CFG_CPHA | APPLE_SPI_CFG_CPOL |
		 APPLE_SPI_CFG_LSB_FIRST, cfg);

	return 0;
}

static void apple_spi_set_cs(struct spi_device *device, bool is_high)
{
	struct apple_spi *spi = spi_controller_get_devdata(device->controller);

	/* The SPI core controls descriptor-backed chip selects itself. */
	if (spi_get_csgpiod(device, 0))
		return;

	reg_mask(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS,
		 is_high ? APPLE_SPI_PIN_CS : 0);
}

static u32 apple_spi_t8010_word_delay(struct apple_spi *spi,
				      struct spi_device *device,
				      bool use_descriptor)
{
	u32 word_delay = spi->word_delay;

	if (use_descriptor && (device->mode & SPI_CPOL) &&
	    word_delay <= 1)
		word_delay = 2;

	return word_delay;
}

static u32 apple_spi_t8010_tx_level(struct apple_spi *spi, u32 status)
{
	return FIELD_GET(APPLE_SPI_STATUS_T8010_TXFIFO, status);
}

static u32 apple_spi_t8010_rx_level(struct apple_spi *spi, u32 status)
{
	return FIELD_GET(APPLE_SPI_STATUS_T8010_RXFIFO, status);
}

static bool apple_spi_prep_transfer(struct apple_spi *spi, struct spi_transfer *t)
{
	u32 cr, fifo_threshold;

	/* Calculate and program the clock rate */
	cr = DIV_ROUND_UP(clk_get_rate(spi->clk), t->speed_hz);
	reg_write(spi, APPLE_SPI_CLKDIV, min_t(u32, cr, APPLE_SPI_CLKDIV_MAX));

	/* Update bits per word */
	reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_BITS,
		 FIELD_PREP(APPLE_SPI_SHIFTCFG_BITS, t->bits_per_word));

	/* We will want to poll if the time we need to wait is
	 * less than the context switching time.
	 * Let's call that threshold 5us. The operation will take:
	 *    bits_per_word * fifo_threshold / hz <= 5 * 10^-6
	 *    200000 * bits_per_word * fifo_threshold <= hz
	 */
	fifo_threshold = APPLE_SPI_FIFO_DEPTH / 2;
	return (200000 * t->bits_per_word * fifo_threshold) <= t->speed_hz;
}

static irqreturn_t apple_spi_irq(int irq, void *dev_id)
{
	struct apple_spi *spi = dev_id;
	u32 fifo = reg_read(spi, APPLE_SPI_IF_FIFO) & reg_read(spi, APPLE_SPI_IE_FIFO);
	u32 xfer = reg_read(spi, APPLE_SPI_IF_XFER) & reg_read(spi, APPLE_SPI_IE_XFER);

	if (fifo || xfer) {
		/* Disable interrupts until next transfer */
		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
		complete(&spi->done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int apple_spi_wait(struct apple_spi *spi, u32 fifo_bit, u32 xfer_bit, int poll)
{
	int ret = 0;

	if (poll) {
		u32 fifo, xfer;
		unsigned long timeout = jiffies + APPLE_SPI_TIMEOUT_MS * HZ / 1000;

		do {
			fifo = reg_read(spi, APPLE_SPI_IF_FIFO);
			xfer = reg_read(spi, APPLE_SPI_IF_XFER);
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				break;
			}
		} while (!((fifo & fifo_bit) || (xfer & xfer_bit)));
	} else {
		reinit_completion(&spi->done);
		reg_write(spi, APPLE_SPI_IE_XFER, xfer_bit);
		reg_write(spi, APPLE_SPI_IE_FIFO, fifo_bit);

		if (!wait_for_completion_timeout(&spi->done,
						 msecs_to_jiffies(APPLE_SPI_TIMEOUT_MS)))
			ret = -ETIMEDOUT;

		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
	}

	return ret;
}

static u32 apple_spi_t8010_config(struct spi_device *device,
				  unsigned int bits_per_word,
				  bool use_descriptor)
{
	u32 cfg = APPLE_SPI_CFG_T8010_BASE;

	if (use_descriptor) {
		unsigned int word_index = bits_per_word == 16;

		cfg = APPLE_SPI_T8010_DESC_CFG_BASE |
		      APPLE_SPI_T8010_DESC_CFG_DEFAULT;
		if (device->mode & SPI_CPOL)
			cfg |= APPLE_SPI_T8010_DESC_CPOL_HIGH |
			       APPLE_SPI_T8010_DESC_CPOL_EXTRA;
		else
			cfg |= APPLE_SPI_T8010_DESC_CPOL_LOW;
		if (device->mode & SPI_CPHA)
			cfg |= APPLE_SPI_CFG_CPHA;
		cfg |= APPLE_SPI_T8010_DESC_WORD_BITS << word_index;
		return cfg;
	}

	if (device->mode & SPI_CPHA)
		cfg |= APPLE_SPI_CFG_CPHA;
	else
		cfg &= ~APPLE_SPI_CFG_CPHA;

	if (device->mode & SPI_CPOL)
		cfg |= APPLE_SPI_CFG_CPOL;
	else
		cfg &= ~APPLE_SPI_CFG_CPOL;

	if (bits_per_word == 16)
		cfg |= APPLE_SPI_CFG_T8010_16BIT;

	return cfg;
}

static int apple_spi_t8010_calc_div(struct apple_spi *spi,
				    unsigned int speed_hz, u32 *div)
{
	if (!speed_hz)
		speed_hz = clk_get_rate(spi->clk) / spi->hw->min_clk_div;

	*div = DIV_ROUND_UP(clk_get_rate(spi->clk), speed_hz);
	if (*div > APPLE_SPI_CLKDIV_MAX)
		return -EINVAL;
	*div = max_t(u32, *div, spi->hw->min_clk_div);

	return 0;
}

static u32 apple_spi_t8010_dma_config(struct spi_device *device,
				      unsigned int bits_per_word)
{
	u32 cfg = apple_spi_t8010_config(device, bits_per_word, true);

	return (cfg & ~APPLE_SPI_T8010_DESC_CFG_DEFAULT) |
	       FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_DMA);
}

static bool apple_spi_t8010_can_dma(struct spi_controller *ctlr,
				    struct spi_device *device,
				    struct spi_transfer *t)
{
	unsigned int bits_per_word = t->bits_per_word ?: device->bits_per_word;

	if (!bits_per_word)
		bits_per_word = 8;

	return ctlr->dma_tx && (bits_per_word == 8 || bits_per_word == 16) &&
	       t->len >= 17 && !(t->len % (bits_per_word / 8)) &&
	       t->tx_buf && !t->rx_buf;
}

static void apple_spi_t8010_dma_complete(void *data,
					 const struct dmaengine_result *result)
{
	struct apple_spi *spi = data;

	spi->dma_result = *result;
	complete(&spi->dma_done);
}

static int apple_spi_transfer_one_t8010_dma(struct spi_controller *ctlr,
					    struct spi_device *device,
					    struct spi_transfer *t)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	struct dma_async_tx_descriptor *descriptor;
	struct dma_slave_config dma_config = { .direction = DMA_MEM_TO_DEV };
	unsigned int bits_per_word = t->bits_per_word ?: device->bits_per_word;
	unsigned long deadline;
	unsigned long timeout;
	dma_cookie_t cookie;
	u32 div;
	u32 cfg;
	u32 status;
	int ret;

	if (!t->tx_sg_mapped || !t->tx_sg.nents)
		return -EINVAL;
	if (!bits_per_word)
		bits_per_word = 8;
	dma_config.dst_addr_width = bits_per_word == 16 ?
		DMA_SLAVE_BUSWIDTH_2_BYTES : DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(ctlr->dma_tx, &dma_config);
	if (ret)
		return ret;

	ret = apple_spi_t8010_calc_div(spi,
				       t->speed_hz ?: device->max_speed_hz, &div);
	if (ret)
		return ret;

	reg_write(spi, APPLE_SPI_CLKDIV, div);
	reg_write(spi, APPLE_SPI_WORD_DELAY,
		  apple_spi_t8010_word_delay(spi, device, true));
	reg_write(spi, APPLE_SPI_STATUS, APPLE_SPI_T8010_DESC_STATUS_CFG);
	cfg = apple_spi_t8010_dma_config(device, bits_per_word);
	reg_write(spi, APPLE_SPI_CFG,
		  cfg & ~FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_DMA));
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_T8010_ENABLE);

	descriptor = dmaengine_prep_slave_sg(ctlr->dma_tx, t->tx_sg.sgl,
					     t->tx_sg.nents, DMA_MEM_TO_DEV,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!descriptor)
		return -ENOMEM;

	reinit_completion(&spi->dma_done);
	spi->dma_result.result = DMA_TRANS_ABORTED;
	spi->dma_result.residue = t->len;
	descriptor->callback_result = apple_spi_t8010_dma_complete;
	descriptor->callback_param = spi;
	cookie = dmaengine_submit(descriptor);
	if (dma_submit_error(cookie)) {
		dmaengine_desc_free(descriptor);
		return cookie;
	}

	dma_async_issue_pending(ctlr->dma_tx);
	if (try_wait_for_completion(&spi->dma_done)) {
		ret = -EIO;
		goto out_terminate;
	}

	reg_write(spi, APPLE_SPI_RXCNT, 0);
	reg_write(spi, APPLE_SPI_TXCNT, t->len);
	reg_write(spi, APPLE_SPI_CFG, cfg);

	timeout = msecs_to_jiffies(APPLE_SPI_T8010_TIMEOUT_MS);
	if (!wait_for_completion_timeout(&spi->dma_done, timeout)) {
		ret = -ETIMEDOUT;
		goto out_disable;
	}
	if (spi->dma_result.result != DMA_TRANS_NOERROR) {
		ret = -EIO;
		goto out_disable;
	}

	deadline = jiffies + timeout;
	do {
		status = reg_read(spi, APPLE_SPI_STATUS);
		if (!FIELD_GET(APPLE_SPI_STATUS_T8010_TXFIFO, status))
			break;
		if (time_after(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			goto out_disable;
		}
		udelay(1);
	} while (true);

	ret = 0;

out_disable:
	reg_write(spi, APPLE_SPI_CFG,
		  cfg & ~FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_DMA));
	if (ret)
		goto out_terminate;

	reg_write(spi, APPLE_SPI_CTRL, 0);
	reg_write(spi, APPLE_SPI_CTRL, 0);
	reg_write(spi, APPLE_SPI_PIN, 0);
	reg_write(spi, APPLE_SPI_STATUS, APPLE_SPI_T8010_DESC_STATUS_CFG);
	return 0;

out_terminate:
	dmaengine_terminate_sync(ctlr->dma_tx);
	return ret;
}

static void apple_spi_t8010_dma_release(void *data)
{
	dma_release_channel(data);
}

static void apple_spi_t8010_reset_fifos(struct apple_spi *spi)
{
	reg_write(spi, APPLE_SPI_CTRL, 0);
	reg_read(spi, APPLE_SPI_CTRL);
	reg_write(spi, APPLE_SPI_CTRL,
		  APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);
	reg_read(spi, APPLE_SPI_CTRL);
	reg_write(spi, APPLE_SPI_CTRL, 0);
	reg_read(spi, APPLE_SPI_CTRL);
	reg_write(spi, APPLE_SPI_STATUS, ~0);
	reg_write(spi, APPLE_SPI_IF_XFER, ~0);
	reg_write(spi, APPLE_SPI_IF_FIFO, ~0);
}

static void apple_spi_t8010_prepare_xfer(struct apple_spi *spi,
					 bool use_descriptor)
{
	apple_spi_t8010_reset_fifos(spi);
	apple_spi_t8010_write_status_config(spi, use_descriptor);
}

static void apple_spi_t8010_write_word(struct apple_spi *spi,
				       const void *tx_buf,
				       unsigned int index,
				       unsigned int bytes_per_word)
{
	u32 data = 0;

	if (tx_buf) {
		if (bytes_per_word == 2)
			data = get_unaligned_le16((const u8 *)tx_buf + index * 2);
		else
			data = *((const u8 *)tx_buf + index);
	}

	reg_write(spi, APPLE_SPI_TXDATA, data);
}

static void apple_spi_t8010_read_word(struct apple_spi *spi, void *rx_buf,
				      unsigned int index,
				      unsigned int bytes_per_word)
{
	u32 data = reg_read(spi, APPLE_SPI_RXDATA);

	if (!rx_buf)
		return;

	if (bytes_per_word == 2)
		put_unaligned_le16(data, (u8 *)rx_buf + index * 2);
	else
		*((u8 *)rx_buf + index) = data;
}

static unsigned int apple_spi_t8010_fill_tx_fifo(struct apple_spi *spi,
						 const void *tx_buf,
						 unsigned int tx_done,
						 unsigned int words,
						 unsigned int bytes_per_word,
						 unsigned int tx_level)
{
	while (tx_done < words && tx_level < APPLE_SPI_FIFO_DEPTH) {
		apple_spi_t8010_write_word(spi, tx_buf, tx_done,
					   bytes_per_word);
		tx_done++;
		tx_level++;
	}

	return tx_done;
}

static unsigned int apple_spi_t8010_tx_space(struct apple_spi *spi, u32 status)
{
	u32 tx_level = apple_spi_t8010_tx_level(spi, status);

	if (tx_level >= APPLE_SPI_FIFO_DEPTH)
		return 0;

	return APPLE_SPI_FIFO_DEPTH - tx_level;
}

static unsigned int apple_spi_t8010_transfer_time_us(unsigned int bits_per_word,
						     unsigned int words,
						     unsigned int speed_hz)
{
	u64 bit_times;

	if (!bits_per_word || !words || !speed_hz)
		return 0;

	bit_times = (u64)bits_per_word * words * USEC_PER_SEC;
	return max_t(unsigned int, DIV_ROUND_UP_ULL(bit_times, speed_hz), 1);
}

static void apple_spi_t8010_finish_xfer(struct apple_spi *spi, u32 cfg,
					bool use_descriptor)
{
	reg_write(spi, APPLE_SPI_CFG, cfg);
	reg_write(spi, APPLE_SPI_CTRL, 0);
	if (!use_descriptor)
		return;

	reg_write(spi, APPLE_SPI_CTRL, 0);
	reg_write(spi, APPLE_SPI_PIN, 0);
	apple_spi_t8010_write_status_config(spi, true);
}

static int apple_spi_transfer_one_t8010_pio(struct spi_controller *ctlr,
					    struct spi_device *device,
					    struct spi_transfer *t)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	unsigned int bits_per_word = t->bits_per_word ?: device->bits_per_word;
	unsigned long deadline = jiffies +
		msecs_to_jiffies(APPLE_SPI_T8010_TIMEOUT_MS);
	unsigned int bytes_per_word, words, tx_done = 0, rx_done = 0;
	unsigned int delay_us, speed_hz, tx_space, rx_level;
	bool use_descriptor;
	u32 cfg, div, status;
	int ret;

	if (!bits_per_word)
		bits_per_word = 8;
	if (bits_per_word != 8 && bits_per_word != 16)
		return -EINVAL;

	bytes_per_word = bits_per_word == 16 ? 2 : 1;
	if (t->len % bytes_per_word)
		return -EINVAL;
	words = t->len / bytes_per_word;
	if (!words)
		return 0;

	speed_hz = t->speed_hz ?: device->max_speed_hz;
	ret = apple_spi_t8010_calc_div(spi, speed_hz, &div);
	if (ret)
		return ret;
	speed_hz = clk_get_rate(spi->clk) / div;

	use_descriptor = apple_spi_t8010_uses_descriptor(t, words);
	reg_write(spi, APPLE_SPI_CLKDIV, div);
	reg_write(spi, APPLE_SPI_WORD_DELAY,
		  apple_spi_t8010_word_delay(spi, device, use_descriptor));
	apple_spi_t8010_prepare_xfer(spi, use_descriptor);

	cfg = apple_spi_t8010_config(device, bits_per_word, use_descriptor);
	if (use_descriptor)
		cfg &= ~(APPLE_SPI_T8010_DESC_CFG_DEFAULT | BIT(0));

	reg_write(spi, APPLE_SPI_RXCNT, words);
	reg_write(spi, APPLE_SPI_TXCNT, words);
	reg_write(spi, APPLE_SPI_CFG, cfg);
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_T8010_ENABLE);
	reg_write(spi, APPLE_SPI_CFG, cfg | APPLE_SPI_CFG_T8010_PIOEN);

	tx_done = apple_spi_t8010_fill_tx_fifo(spi, t->tx_buf, 0, words,
					       bytes_per_word, 0);
	if (use_descriptor)
		reg_write(spi, APPLE_SPI_CFG,
			  cfg | APPLE_SPI_CFG_T8010_PIOEN);

	delay_us = apple_spi_t8010_transfer_time_us(bits_per_word, words,
						    speed_hz);
	if (delay_us)
		fsleep(delay_us);

	while (rx_done < words) {
		status = reg_read(spi, APPLE_SPI_STATUS);
		if (time_after(jiffies, deadline)) {
			dev_err(&ctlr->dev,
				"T8010 PIO timed out: tx %u/%u rx %u/%u status=%#x fifo=%#x\n",
				tx_done, words, rx_done, words, status,
				reg_read(spi, APPLE_SPI_FIFOSTAT));
			apple_spi_t8010_finish_xfer(spi, cfg, use_descriptor);
			return -ETIMEDOUT;
		}
		rx_level = apple_spi_t8010_rx_level(spi, status);
		tx_space = apple_spi_t8010_tx_space(spi, status);
		if (!rx_level && !tx_space) {
			udelay(1);
			continue;
		}

		reg_write(spi, APPLE_SPI_STATUS, status);
		while (rx_done < words && rx_level) {
			apple_spi_t8010_read_word(spi, t->rx_buf, rx_done,
						  bytes_per_word);
			rx_done++;
			rx_level--;
		}

		tx_done = apple_spi_t8010_fill_tx_fifo(spi, t->tx_buf, tx_done,
						       words, bytes_per_word,
						       APPLE_SPI_FIFO_DEPTH -
						       tx_space);
	}

	apple_spi_t8010_finish_xfer(spi, cfg, use_descriptor);
	return 0;
}

static int apple_spi_transfer_one_t8010(struct spi_controller *ctlr,
					struct spi_device *device,
					struct spi_transfer *t)
{
	if (apple_spi_t8010_can_dma(ctlr, device, t) && t->tx_sg_mapped)
		return apple_spi_transfer_one_t8010_dma(ctlr, device, t);

	return apple_spi_transfer_one_t8010_pio(ctlr, device, t);
}

static void apple_spi_tx(struct apple_spi *spi, const void **tx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 inuse, words, wrote;

	if (!*tx_ptr)
		return;

	inuse = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_TX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	words = wrote = min_t(u32, *left, APPLE_SPI_FIFO_DEPTH - inuse);

	if (!words)
		return;

	*left -= words;

	switch (bytes_per_word) {
	case 1: {
		const u8 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 2: {
		const u16 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 4: {
		const u32 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	default:
		WARN_ON(1);
	}

	*tx_ptr = ((u8 *)*tx_ptr) + bytes_per_word * wrote;
}

static void apple_spi_rx(struct apple_spi *spi, void **rx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 words, read;

	if (!*rx_ptr)
		return;

	words = read = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_RX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	WARN_ON(words > *left);

	if (!words)
		return;

	*left -= min_t(u32, *left, words);

	switch (bytes_per_word) {
	case 1: {
		u8 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 2: {
		u16 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 4: {
		u32 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	default:
		WARN_ON(1);
	}

	*rx_ptr = ((u8 *)*rx_ptr) + bytes_per_word * read;
}

static int apple_spi_transfer_one(struct spi_controller *ctlr,
				  struct spi_device *device,
				  struct spi_transfer *t)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	bool poll = apple_spi_prep_transfer(spi, t);
	const void *tx_ptr = t->tx_buf;
	void *rx_ptr = t->rx_buf;
	unsigned int bytes_per_word;
	u32 words, remaining_tx, remaining_rx;
	u32 xfer_flags = 0;
	u32 fifo_flags;
	int retries = 100;
	int ret = 0;

	if (t->bits_per_word > 16)
		bytes_per_word = 4;
	else if (t->bits_per_word > 8)
		bytes_per_word = 2;
	else
		bytes_per_word = 1;

	words = t->len / bytes_per_word;
	remaining_tx = tx_ptr ? words : 0;
	remaining_rx = rx_ptr ? words : 0;

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	/* Clear IRQ flags */
	reg_write(spi, APPLE_SPI_IF_XFER, ~0);
	reg_write(spi, APPLE_SPI_IF_FIFO, ~0);

	/* Determine transfer completion flags we wait for */
	if (tx_ptr)
		xfer_flags |= APPLE_SPI_XFER_TXCOMPLETE;
	if (rx_ptr)
		xfer_flags |= APPLE_SPI_XFER_RXCOMPLETE;

	/* Set transfer length */
	reg_write(spi, APPLE_SPI_TXCNT, remaining_tx);
	reg_write(spi, APPLE_SPI_RXCNT, remaining_rx);

	/* Prime transmit FIFO */
	apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);

	/* Start transfer */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RUN);

	/* TX again since a few words get popped off immediately */
	apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);

	while (xfer_flags) {
		fifo_flags = 0;

		if (remaining_tx)
			fifo_flags |= APPLE_SPI_FIFO_TXTHRESH;
		if (remaining_rx)
			fifo_flags |= APPLE_SPI_FIFO_RXTHRESH;

		/* Wait for anything to happen */
		ret = apple_spi_wait(spi, fifo_flags, xfer_flags, poll);
		if (ret) {
			dev_err(&ctlr->dev, "transfer timed out (remaining %d tx, %d rx)\n",
				remaining_tx, remaining_rx);
			goto err;
		}

		/* Stop waiting on transfer halves once they complete */
		xfer_flags &= ~reg_read(spi, APPLE_SPI_IF_XFER);

		/* Transmit and receive everything we can */
		apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);
		apple_spi_rx(spi, &rx_ptr, &remaining_rx, bytes_per_word);
	}

	/*
	 * Sometimes the transfer completes before the last word is in the RX FIFO.
	 * Normally one retry is all it takes to get the last word out.
	 */
	while (remaining_rx && retries--)
		apple_spi_rx(spi, &rx_ptr, &remaining_rx, bytes_per_word);

	if (remaining_tx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to transmit\n",
			remaining_tx);
	if (remaining_rx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to receive\n",
			remaining_rx);

err:
	fifo_flags = reg_read(spi, APPLE_SPI_IF_FIFO);
	WARN_ON(fifo_flags & APPLE_SPI_FIFO_TXOVERFLOW);
	WARN_ON(fifo_flags & APPLE_SPI_FIFO_RXUNDERRUN);

	/* Stop transfer */
	reg_write(spi, APPLE_SPI_CTRL, 0);

	return ret;
}

static int apple_spi_probe(struct platform_device *pdev)
{
	struct apple_spi *spi;
	int ret, irq;
	struct spi_controller *ctlr;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(struct apple_spi));
	if (!ctlr)
		return -ENOMEM;

	spi = spi_controller_get_devdata(ctlr);
	platform_set_drvdata(pdev, spi);
	init_completion(&spi->done);
	init_completion(&spi->dma_done);
	spi->hw = of_device_get_match_data(&pdev->dev);
	if (!spi->hw)
		return -EINVAL;
	spi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi->regs))
		return PTR_ERR(spi->regs);

	spi->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(spi->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(spi->clk),
				     "Unable to find or enable bus clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, apple_spi_irq, 0,
			       dev_name(&pdev->dev), spi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Unable to bind to interrupt\n");

	spi->word_delay = spi->hw->default_word_delay;
	of_property_read_u32(pdev->dev.of_node, "apple,t8010-word-delay",
			     &spi->word_delay);

	ctlr->bus_num = pdev->id;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL;
	if (!spi->hw->legacy_status)
		ctlr->mode_bits |= SPI_LSB_FIRST;
	if (spi->hw->legacy_status)
		ctlr->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	else
		ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	ctlr->prepare_message = apple_spi_prepare_message;
	ctlr->set_cs = apple_spi_set_cs;
	if (spi->hw->legacy_status)
		ctlr->transfer_one = apple_spi_transfer_one_t8010;
	else
		ctlr->transfer_one = apple_spi_transfer_one;
	ctlr->use_gpio_descriptors = true;
	ctlr->flags |= SPI_CONTROLLER_GPIO_SS;
	ctlr->auto_runtime_pm = true;

	if (spi->hw->legacy_status) {
		ctlr->dma_tx = dma_request_chan(&pdev->dev, "tx");
		if (IS_ERR(ctlr->dma_tx)) {
			ret = PTR_ERR(ctlr->dma_tx);
			ctlr->dma_tx = NULL;
			if (ret != -ENODEV && ret != -ENXIO)
				return dev_err_probe(&pdev->dev, ret,
						     "failed to request TX DMA\n");
		} else {
			ret = devm_add_action_or_reset(&pdev->dev,
						       apple_spi_t8010_dma_release,
						       ctlr->dma_tx);
			if (ret)
				return ret;
			ctlr->can_dma = apple_spi_t8010_can_dma;
		}
	}

	pm_runtime_set_active(&pdev->dev);
	ret = devm_pm_runtime_enable(&pdev->dev);
	if (ret < 0)
		return ret;

	apple_spi_init(spi);

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register SPI controller\n");

	return 0;
}

static const struct apple_spi_hw apple_spi_hw_t8010 = {
	.legacy_status = true,
	.min_clk_div = 2,
};

static const struct apple_spi_hw apple_spi_hw_default = {
	.min_clk_div = 1,
};

static const struct of_device_id apple_spi_of_match[] = {
	{ .compatible = "apple,t8010-spi", .data = &apple_spi_hw_t8010, },
	{ .compatible = "apple,t8103-spi", .data = &apple_spi_hw_default, },
	{ .compatible = "apple,spi", .data = &apple_spi_hw_default, },
	{}
};
MODULE_DEVICE_TABLE(of, apple_spi_of_match);

static struct platform_driver apple_spi_driver = {
	.probe = apple_spi_probe,
	.driver = {
		.name = "apple-spi",
		.of_match_table = apple_spi_of_match,
	},
};
module_platform_driver(apple_spi_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("Apple SoC SPI driver");
MODULE_LICENSE("GPL");

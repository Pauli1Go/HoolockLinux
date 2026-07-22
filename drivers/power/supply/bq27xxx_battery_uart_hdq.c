// SPDX-License-Identifier: GPL-2.0-only
/*
 * BQ27xxx HDQ-over-UART battery monitor transport
 *
 * The UART transmits one HDQ bit per serial byte and receives the resulting
 * line level through its RX input. This is used by Apple T8010 devices and
 * was first documented by the Project Sandcastle implementation.
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/power/bq27xxx_battery.h>

#define BQ27XXX_UART_HDQ_BAUD		57600
#define BQ27XXX_UART_HDQ_BREAK_MIN_US	250
#define BQ27XXX_UART_HDQ_BREAK_MAX_US	500
#define BQ27XXX_UART_HDQ_RECOVERY_MIN_US	150
#define BQ27XXX_UART_HDQ_RECOVERY_MAX_US	500
#define BQ27XXX_UART_HDQ_TIMEOUT_MS	500
#define BQ27XXX_UART_HDQ_WORD_RETRIES	3
#define BQ27XXX_UART_HDQ_CACHE_REFRESH_MS	30000

#define BQ27XXX_UART_HDQ_BITS_PER_BYTE	8
#define BQ27XXX_UART_HDQ_RX_BITS		16
#define BQ27XXX_UART_HDQ_ONE		0xfe
#define BQ27XXX_UART_HDQ_ZERO		0xc0
#define BQ27XXX_UART_HDQ_RX_ONE_MIN	0xf0

struct bq27xxx_uart_hdq {
	struct serdev_device *serdev;
	struct bq27xxx_device_info battery;
	struct completion done;
	struct mutex xfer_lock; /* Serializes complete HDQ transactions. */
	spinlock_t rx_lock; /* Protects receive state from the UART callback. */
	u8 command_symbols[BQ27XXX_UART_HDQ_BITS_PER_BYTE];
	u8 rx_byte;
	u8 rx_bit;
	bool receiving;
};

static void bq27xxx_uart_hdq_cancel_receive(struct bq27xxx_uart_hdq *hdq)
{
	unsigned long flags;

	spin_lock_irqsave(&hdq->rx_lock, flags);
	hdq->receiving = false;
	spin_unlock_irqrestore(&hdq->rx_lock, flags);
}

static size_t bq27xxx_uart_hdq_rx(struct serdev_device *serdev,
				  const u8 *buf, size_t count)
{
	struct bq27xxx_device_info *di = serdev_device_get_drvdata(serdev);
	struct bq27xxx_uart_hdq *hdq =
		container_of(di, struct bq27xxx_uart_hdq, battery);
	unsigned long flags;
	size_t i;

	spin_lock_irqsave(&hdq->rx_lock, flags);
	for (i = 0; i < count && hdq->receiving; i++) {
		if (hdq->rx_bit < BQ27XXX_UART_HDQ_BITS_PER_BYTE &&
		    buf[i] != hdq->command_symbols[hdq->rx_bit])
			continue;

		if (hdq->rx_bit >= BQ27XXX_UART_HDQ_BITS_PER_BYTE &&
		    buf[i] >= BQ27XXX_UART_HDQ_RX_ONE_MIN)
			hdq->rx_byte |= BIT(hdq->rx_bit -
					       BQ27XXX_UART_HDQ_BITS_PER_BYTE);

		hdq->rx_bit++;
		if (hdq->rx_bit == BQ27XXX_UART_HDQ_RX_BITS) {
			hdq->receiving = false;
			complete(&hdq->done);
		}
	}
	spin_unlock_irqrestore(&hdq->rx_lock, flags);

	return count;
}

static const struct serdev_device_ops bq27xxx_uart_hdq_serdev_ops = {
	.receive_buf = bq27xxx_uart_hdq_rx,
	.write_wakeup = serdev_device_write_wakeup,
};

static int bq27xxx_uart_hdq_read_byte(struct bq27xxx_uart_hdq *hdq, u8 reg)
{
	unsigned long flags;
	unsigned long timeout;
	ssize_t written;
	unsigned int bit;
	int ret;
	u8 value;

	for (bit = 0; bit < BQ27XXX_UART_HDQ_BITS_PER_BYTE; bit++)
		hdq->command_symbols[bit] = reg & BIT(bit) ?
			BQ27XXX_UART_HDQ_ONE : BQ27XXX_UART_HDQ_ZERO;

	ret = serdev_device_break_ctl(hdq->serdev, -1);
	if (ret)
		return ret;

	usleep_range(BQ27XXX_UART_HDQ_BREAK_MIN_US,
		     BQ27XXX_UART_HDQ_BREAK_MAX_US);
	ret = serdev_device_break_ctl(hdq->serdev, 0);
	if (ret)
		return ret;

	usleep_range(BQ27XXX_UART_HDQ_RECOVERY_MIN_US,
		     BQ27XXX_UART_HDQ_RECOVERY_MAX_US);

	reinit_completion(&hdq->done);
	spin_lock_irqsave(&hdq->rx_lock, flags);
	hdq->rx_byte = 0;
	hdq->rx_bit = 0;
	hdq->receiving = true;
	spin_unlock_irqrestore(&hdq->rx_lock, flags);

	written = serdev_device_write(hdq->serdev, hdq->command_symbols,
				      sizeof(hdq->command_symbols), HZ);
	if (written < 0) {
		ret = written;
		goto cancel_receive;
	}
	if (written != sizeof(hdq->command_symbols)) {
		ret = -EIO;
		goto cancel_receive;
	}

	serdev_device_wait_until_sent(hdq->serdev, HZ);
	timeout = wait_for_completion_timeout(&hdq->done,
					      msecs_to_jiffies(BQ27XXX_UART_HDQ_TIMEOUT_MS));
	if (!timeout) {
		ret = -ETIMEDOUT;
		goto cancel_receive;
	}

	spin_lock_irqsave(&hdq->rx_lock, flags);
	value = hdq->rx_byte;
	spin_unlock_irqrestore(&hdq->rx_lock, flags);

	return value;

cancel_receive:
	bq27xxx_uart_hdq_cancel_receive(hdq);
	return ret;
}

static int bq27xxx_uart_hdq_read(struct bq27xxx_device_info *di, u8 reg,
				 bool single)
{
	struct bq27xxx_uart_hdq *hdq =
		container_of(di, struct bq27xxx_uart_hdq, battery);
	int high_first;
	int high_second;
	int low;
	unsigned int attempt;
	int ret;

	mutex_lock(&hdq->xfer_lock);
	if (single) {
		ret = bq27xxx_uart_hdq_read_byte(hdq, reg);
		goto out_unlock;
	}

	for (attempt = 0; attempt < BQ27XXX_UART_HDQ_WORD_RETRIES; attempt++) {
		high_first = bq27xxx_uart_hdq_read_byte(hdq, reg + 1);
		if (high_first < 0) {
			ret = high_first;
			goto out_unlock;
		}

		low = bq27xxx_uart_hdq_read_byte(hdq, reg);
		if (low < 0) {
			ret = low;
			goto out_unlock;
		}

		high_second = bq27xxx_uart_hdq_read_byte(hdq, reg + 1);
		if (high_second < 0) {
			ret = high_second;
			goto out_unlock;
		}

		if (high_first == high_second) {
			ret = low | high_first << 8;
			goto out_unlock;
		}
	}

	ret = -EAGAIN;

out_unlock:
	mutex_unlock(&hdq->xfer_lock);
	return ret;
}

static int bq27xxx_uart_hdq_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct bq27xxx_uart_hdq *hdq;
	unsigned int baud;
	int ret;

	hdq = devm_kzalloc(dev, sizeof(*hdq), GFP_KERNEL);
	if (!hdq)
		return -ENOMEM;

	hdq->serdev = serdev;
	hdq->battery.dev = dev;
	hdq->battery.chip = BQ27545;
	hdq->battery.name = "battery";
	hdq->battery.bus.read = bq27xxx_uart_hdq_read;
	hdq->battery.cache_only = true;
	hdq->battery.cache_refresh_ms = BQ27XXX_UART_HDQ_CACHE_REFRESH_MS;
	serdev_device_set_drvdata(serdev, &hdq->battery);

	init_completion(&hdq->done);
	spin_lock_init(&hdq->rx_lock);
	ret = devm_mutex_init(dev, &hdq->xfer_lock);
	if (ret)
		return ret;

	serdev_device_set_client_ops(serdev, &bq27xxx_uart_hdq_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open UART\n");

	baud = serdev_device_set_baudrate(serdev, BQ27XXX_UART_HDQ_BAUD);
	if (baud != BQ27XXX_UART_HDQ_BAUD)
		return dev_err_probe(dev, -EINVAL,
				     "failed to set 57600 baud (got %u)\n", baud);

	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to disable parity\n");

	ret = serdev_device_set_stopbits(serdev, SERDEV_STOPBITS_2);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set two stop bits\n");

	ret = bq27xxx_battery_setup(&hdq->battery);
	if (ret)
		return ret;

	return 0;
}

static void bq27xxx_uart_hdq_remove(struct serdev_device *serdev)
{
	struct bq27xxx_device_info *di = serdev_device_get_drvdata(serdev);
	struct bq27xxx_uart_hdq *hdq =
		container_of(di, struct bq27xxx_uart_hdq, battery);

	bq27xxx_uart_hdq_cancel_receive(hdq);
	bq27xxx_battery_teardown(di);
}

static const struct of_device_id bq27xxx_uart_hdq_of_match[] = {
	{ .compatible = "ti,bq27545-hdq-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, bq27xxx_uart_hdq_of_match);

static struct serdev_device_driver bq27xxx_uart_hdq_driver = {
	.probe = bq27xxx_uart_hdq_probe,
	.remove = bq27xxx_uart_hdq_remove,
	.driver = {
		.name = "bq27xxx-uart-hdq",
		.of_match_table = bq27xxx_uart_hdq_of_match,
		.pm = &bq27xxx_battery_battery_pm_ops,
	},
};
module_serdev_device_driver(bq27xxx_uart_hdq_driver);

MODULE_AUTHOR("Paul Praschl");
MODULE_DESCRIPTION("BQ27xxx HDQ-over-UART battery monitor transport");
MODULE_LICENSE("GPL");

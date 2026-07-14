// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8010 SmartIO coprocessor driver
 *
 * Copyright (C) 2026 Paul Praschl
 */

#include <clocksource/arm_arch_timer.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/sio.h>
#include <linux/xarray.h>

#define APPLE_SIO_REMAP_AP_LO		0x0008
#define APPLE_SIO_REMAP_AP_HI		0x0010
#define APPLE_SIO_REMAP_IOP_LO		0x0018
#define APPLE_SIO_REMAP_IOP_HI		0x0020
#define APPLE_SIO_REMAP_SIZE_LO		0x0028
#define APPLE_SIO_REMAP_SIZE_HI		0x0030
#define APPLE_SIO_REMAP_ENABLE		0x0038
#define APPLE_SIO_CPU_CONTROL		0x0044
#define APPLE_SIO_CPU_RUN		BIT(4)
#define APPLE_SIO_TIME_CONTROL		0x080c
#define APPLE_SIO_IRQ_STATUS_LEGACY	0x0b84
#define APPLE_SIO_IRQ_STATUS		0x0b88
#define APPLE_SIO_QUEUE_INIT_B		0x0c00
#define APPLE_SIO_QUEUE_START_B		0x0c04
#define APPLE_SIO_QUEUE_INIT_A		0x4000
#define APPLE_SIO_QUEUE_START_A		0x4004
#define APPLE_SIO_TX_STATUS		0x4008
#define APPLE_SIO_TX_MESSAGE		0x4010
#define APPLE_SIO_RX_STATUS		0x4020
#define APPLE_SIO_RX_MESSAGE		0x4038
#define APPLE_SIO_TIME_LO		0xc030
#define APPLE_SIO_TIME_HI		0xc038

#define APPLE_SIO_MBOX_ENABLE		BIT(0)
#define APPLE_SIO_MBOX_EMPTY		BIT(17)
#define APPLE_SIO_MBOX_ERROR		GENMASK(19, 18)

#define APPLE_SIO_MSG_ENDPOINT		GENMASK_ULL(7, 0)
#define APPLE_SIO_MSG_TAG		GENMASK_ULL(15, 8)
#define APPLE_SIO_MSG_OPCODE		GENMASK_ULL(23, 16)
#define APPLE_SIO_MSG_PARAMETER		GENMASK_ULL(31, 24)
#define APPLE_SIO_MSG_DATA		GENMASK_ULL(63, 32)

#define APPLE_SIO_OP_ERROR		0x6a
#define APPLE_SIO_OP_READY		0x6d
#define APPLE_SIO_READY_PARAMETER	0x03
#define APPLE_SIO_RX_IRQ_INDEX		2
#define APPLE_SIO_MAX_TAG		0xfe
#define APPLE_SIO_READY_TIMEOUT_MS	5000
#define APPLE_SIO_TX_TIMEOUT_US		200000
#define APPLE_SIO_RX_DRAIN_LIMIT	256
#define APPLE_SIO_REQUEST_TIMEOUT_MS	1000
#define APPLE_SIO_DESC0_SIZE		0x7ff8
#define APPLE_SIO_DESC0_COUNT		0x0aaa
#define APPLE_SIO_DESC0_SLOT_SIZE	(3 * sizeof(u32))
#define APPLE_SIO_DESC1_SIZE		0x10400
#define APPLE_SIO_DESC1_COUNT		0x1040
#define APPLE_SIO_SETUP_COMMANDS	10

struct apple_sio_region {
	void *buffer;
	dma_addr_t dma;
	size_t size;
};

struct apple_sio_async {
	struct apple_sio *sio;
	struct list_head shutdown_node;
	struct completion done;
	refcount_t refs;
	apple_sio_async_callback_t callback;
	void *callback_data;
	unsigned long key;
	u8 tag;
	u8 expected_opcode;
	int status;
	u64 response;
	bool active;
};

struct apple_sio {
	struct device *dev;
	void __iomem *base;
	void *firmware_buffer;
	dma_addr_t firmware_dma;
	size_t firmware_size;
	struct completion ready;
	struct completion setup_done;
	/* Serialize access to the single-entry transmit queue. */
	struct mutex tx_lock;
	/* Protect the final transmit status check and message write. */
	spinlock_t tx_hw_lock;
	struct ida tags;
	struct xarray pending;
	struct apple_sio_region ep0_general;
	struct apple_sio_region ep0_spi;
	struct apple_sio_region ep3_aux;
	struct apple_sio_region desc0;
	struct apple_sio_region desc1;
	unsigned long *desc0_bitmap;
	/* Protect contiguous Descriptor-0 slot allocation. */
	spinlock_t desc0_lock;
	struct platform_device *dma_pdev;
	unsigned int setup_index;
	unsigned int irq_count;
	unsigned int unmatched_count;
	u32 last_irq_legacy;
	u32 last_irq;
	u32 last_rx_status;
	u32 last_tx_status;
	u64 last_rx_message;
	u64 last_tx_message;
	int setup_status;
	bool running;
	bool stopping;
};

static struct apple_sio_async *
apple_sio_request_async_common(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data, bool atomic, int fixed_tag);

static void apple_sio_submit_setup(struct apple_sio *sio);

static unsigned long apple_sio_request_key(u8 endpoint, u8 tag)
{
	return ((unsigned long)endpoint << 8) | tag;
}

void *apple_sio_desc0_alloc(struct apple_sio *sio, unsigned int slots,
			    u32 *first_slot)
{
	unsigned long flags;
	unsigned long slot;
	void *buffer;

	if (!first_slot || !slots || slots > APPLE_SIO_DESC0_COUNT)
		return NULL;

	spin_lock_irqsave(&sio->desc0_lock, flags);
	slot = bitmap_find_next_zero_area(sio->desc0_bitmap,
					  APPLE_SIO_DESC0_COUNT, 0,
					  slots, 0);
	if (slot < APPLE_SIO_DESC0_COUNT)
		bitmap_set(sio->desc0_bitmap, slot, slots);
	spin_unlock_irqrestore(&sio->desc0_lock, flags);

	if (slot >= APPLE_SIO_DESC0_COUNT)
		return NULL;

	buffer = sio->desc0.buffer + slot * APPLE_SIO_DESC0_SLOT_SIZE;
	memset(buffer, 0, slots * APPLE_SIO_DESC0_SLOT_SIZE);
	*first_slot = slot;
	return buffer;
}
EXPORT_SYMBOL_GPL(apple_sio_desc0_alloc);

void apple_sio_desc0_free(struct apple_sio *sio, u32 first_slot,
			  unsigned int slots)
{
	unsigned long flags;

	if (!slots || first_slot >= APPLE_SIO_DESC0_COUNT ||
	    slots > APPLE_SIO_DESC0_COUNT - first_slot)
		return;

	spin_lock_irqsave(&sio->desc0_lock, flags);
	bitmap_clear(sio->desc0_bitmap, first_slot, slots);
	spin_unlock_irqrestore(&sio->desc0_lock, flags);
}
EXPORT_SYMBOL_GPL(apple_sio_desc0_free);

static bool apple_sio_can_read(u32 status)
{
	return (status & APPLE_SIO_MBOX_ENABLE) &&
	       !(status & (APPLE_SIO_MBOX_ERROR | APPLE_SIO_MBOX_EMPTY));
}

static bool apple_sio_can_write(u32 status)
{
	return (status & APPLE_SIO_MBOX_ENABLE) &&
	       !(status & APPLE_SIO_MBOX_ERROR) &&
	       (status & APPLE_SIO_MBOX_EMPTY);
}

static bool apple_sio_is_ready(u64 message)
{
	return FIELD_GET(APPLE_SIO_MSG_ENDPOINT, message) == 0 &&
	       FIELD_GET(APPLE_SIO_MSG_TAG, message) == 0 &&
	       FIELD_GET(APPLE_SIO_MSG_OPCODE, message) == APPLE_SIO_OP_READY &&
	       FIELD_GET(APPLE_SIO_MSG_PARAMETER, message) ==
			APPLE_SIO_READY_PARAMETER;
}

static void apple_sio_dispatch(struct apple_sio *sio, u64 message)
{
	struct apple_sio_async *request;
	unsigned long flags;
	bool matched = false;
	u8 endpoint = FIELD_GET(APPLE_SIO_MSG_ENDPOINT, message);
	u8 tag = FIELD_GET(APPLE_SIO_MSG_TAG, message);
	u8 opcode = FIELD_GET(APPLE_SIO_MSG_OPCODE, message);
	unsigned long key = apple_sio_request_key(endpoint, tag);

	if (apple_sio_is_ready(message)) {
		if (!READ_ONCE(sio->running) && !READ_ONCE(sio->stopping)) {
			WRITE_ONCE(sio->running, true);
			apple_sio_submit_setup(sio);
		}
		complete_all(&sio->ready);
		return;
	}

	xa_lock_irqsave(&sio->pending, flags);
	request = xa_load(&sio->pending, key);
	if (request && (opcode == request->expected_opcode ||
			opcode == APPLE_SIO_OP_ERROR)) {
		__xa_erase(&sio->pending, key);
		request->active = false;
		request->response = message;
		request->status = opcode == APPLE_SIO_OP_ERROR ? -EIO : 0;
		matched = true;
	}
	xa_unlock_irqrestore(&sio->pending, flags);

	if (!matched) {
		sio->unmatched_count++;
		return;
	}

	ida_free(&sio->tags, request->tag);
	if (request->callback)
		request->callback(request->callback_data, request->status,
				  request->response);
	complete_all(&request->done);
	apple_sio_async_put(request);
}

static irqreturn_t apple_sio_irq(int irq, void *data)
{
	struct apple_sio *sio = data;
	unsigned int count = 0;
	u32 status;

	sio->irq_count++;
	sio->last_irq_legacy =
		readl_relaxed(sio->base + APPLE_SIO_IRQ_STATUS_LEGACY);
	sio->last_irq = readl_relaxed(sio->base + APPLE_SIO_IRQ_STATUS);
	while (count < APPLE_SIO_RX_DRAIN_LIMIT) {
		status = readl_relaxed(sio->base + APPLE_SIO_RX_STATUS);
		sio->last_rx_status = status;
		if (!apple_sio_can_read(status))
			break;

		sio->last_rx_message =
			readq_relaxed(sio->base + APPLE_SIO_RX_MESSAGE);
		apple_sio_dispatch(sio, sio->last_rx_message);
		count++;
	}

	if (count == APPLE_SIO_RX_DRAIN_LIMIT)
		dev_warn_ratelimited(sio->dev, "RX queue drain limit reached\n");

	return IRQ_HANDLED;
}

static int apple_sio_send(struct apple_sio *sio, u64 message)
{
	unsigned long flags;
	u32 status;
	int ret;

	mutex_lock(&sio->tx_lock);
	if (READ_ONCE(sio->stopping)) {
		ret = -ESHUTDOWN;
		goto out_unlock;
	}

	ret = readl_poll_timeout(sio->base + APPLE_SIO_TX_STATUS, status,
				 apple_sio_can_write(status), 10,
				 APPLE_SIO_TX_TIMEOUT_US);
	if (ret)
		goto out_unlock;

	spin_lock_irqsave(&sio->tx_hw_lock, flags);
	if (sio->stopping) {
		ret = -ESHUTDOWN;
		goto out_hw_unlock;
	}

	status = readl_relaxed(sio->base + APPLE_SIO_TX_STATUS);
	if (!apple_sio_can_write(status)) {
		ret = -EBUSY;
		goto out_hw_unlock;
	}

	dma_wmb();
	sio->last_tx_message = message;
	writeq_relaxed(message, sio->base + APPLE_SIO_TX_MESSAGE);

out_hw_unlock:
	spin_unlock_irqrestore(&sio->tx_hw_lock, flags);
out_unlock:
	mutex_unlock(&sio->tx_lock);
	return ret;
}

static int apple_sio_send_atomic(struct apple_sio *sio, u64 message)
{
	unsigned long flags;
	u32 status;
	int ret = 0;

	spin_lock_irqsave(&sio->tx_hw_lock, flags);
	if (sio->stopping) {
		ret = -ESHUTDOWN;
		goto out_unlock;
	}

	status = readl_relaxed(sio->base + APPLE_SIO_TX_STATUS);
	sio->last_tx_status = status;
	if (!apple_sio_can_write(status)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	dma_wmb();
	sio->last_tx_message = message;
	writeq_relaxed(message, sio->base + APPLE_SIO_TX_MESSAGE);

out_unlock:
	spin_unlock_irqrestore(&sio->tx_hw_lock, flags);
	return ret;
}

void apple_sio_async_put(struct apple_sio_async *request)
{
	if (refcount_dec_and_test(&request->refs))
		kfree(request);
}
EXPORT_SYMBOL_GPL(apple_sio_async_put);

static void apple_sio_async_finish(struct apple_sio_async *request,
				   int status, u64 response, bool notify)
{
	struct apple_sio *sio = request->sio;

	ida_free(&sio->tags, request->tag);
	request->status = status;
	request->response = response;
	if (notify && request->callback)
		request->callback(request->callback_data, status, response);
	complete_all(&request->done);
	apple_sio_async_put(request);
}

static struct apple_sio_async *
apple_sio_request_async_common(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data, bool atomic, int fixed_tag)
{
	struct apple_sio_async *request;
	gfp_t gfp = atomic ? GFP_ATOMIC : GFP_KERNEL;
	unsigned long flags;
	u64 message;
	bool finish = false;
	int tag;
	int ret;

	if (!READ_ONCE(sio->running) || READ_ONCE(sio->stopping))
		return ERR_PTR(-ESHUTDOWN);

	request = kzalloc_obj(*request, gfp);
	if (!request)
		return ERR_PTR(-ENOMEM);

	if (fixed_tag >= 0)
		tag = ida_alloc_range(&sio->tags, fixed_tag, fixed_tag, gfp);
	else
		tag = ida_alloc_range(&sio->tags, 1, APPLE_SIO_MAX_TAG, gfp);
	if (tag < 0) {
		kfree(request);
		return ERR_PTR(tag);
	}

	request->sio = sio;
	INIT_LIST_HEAD(&request->shutdown_node);
	init_completion(&request->done);
	refcount_set(&request->refs, 2);
	request->callback = callback;
	request->callback_data = callback_data;
	request->key = apple_sio_request_key(endpoint, tag);
	request->tag = tag;
	request->expected_opcode = expected_opcode;
	request->status = -EINPROGRESS;
	request->active = true;

	ret = xa_insert(&sio->pending, request->key, request, gfp);
	if (ret) {
		ida_free(&sio->tags, tag);
		apple_sio_async_put(request);
		apple_sio_async_put(request);
		return ERR_PTR(ret);
	}

	xa_lock_irqsave(&sio->pending, flags);
	if (sio->stopping &&
	    xa_load(&sio->pending, request->key) == request) {
		__xa_erase(&sio->pending, request->key);
		request->active = false;
		finish = true;
	}
	xa_unlock_irqrestore(&sio->pending, flags);
	if (finish) {
		apple_sio_async_finish(request, -ESHUTDOWN, 0, true);
		return request;
	}

	message = FIELD_PREP(APPLE_SIO_MSG_ENDPOINT, endpoint) |
		  FIELD_PREP(APPLE_SIO_MSG_TAG, tag) |
		  FIELD_PREP(APPLE_SIO_MSG_OPCODE, opcode) |
		  FIELD_PREP(APPLE_SIO_MSG_PARAMETER, parameter) |
		  FIELD_PREP(APPLE_SIO_MSG_DATA, data);

	ret = atomic ? apple_sio_send_atomic(sio, message) :
		       apple_sio_send(sio, message);
	if (ret) {
		xa_lock_irqsave(&sio->pending, flags);
		if (xa_load(&sio->pending, request->key) == request) {
			__xa_erase(&sio->pending, request->key);
			request->active = false;
			finish = true;
		}
		xa_unlock_irqrestore(&sio->pending, flags);
		if (finish)
			apple_sio_async_finish(request, ret, 0, true);
	}

	return request;
}

struct apple_sio_async *
apple_sio_request_async(struct apple_sio *sio, u8 endpoint, u8 opcode,
			u8 parameter, u32 data, u8 expected_opcode,
			apple_sio_async_callback_t callback,
			void *callback_data)
{
	return apple_sio_request_async_common(sio, endpoint, opcode, parameter,
					      data, expected_opcode, callback,
					      callback_data, false, -1);
}
EXPORT_SYMBOL_GPL(apple_sio_request_async);

struct apple_sio_async *
apple_sio_request_async_atomic(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data)
{
	return apple_sio_request_async_common(sio, endpoint, opcode, parameter,
					      data, expected_opcode, callback,
					      callback_data, true, -1);
}
EXPORT_SYMBOL_GPL(apple_sio_request_async_atomic);

static void apple_sio_setup_done(void *data, int status, u64 response)
{
	struct apple_sio *sio = data;

	(void)response;
	if (status) {
		sio->setup_status = status;
		complete_all(&sio->setup_done);
		return;
	}

	sio->setup_index++;
	if (sio->setup_index == APPLE_SIO_SETUP_COMMANDS) {
		sio->setup_status = 0;
		complete_all(&sio->setup_done);
		return;
	}

	apple_sio_submit_setup(sio);
}

static void apple_sio_submit_setup(struct apple_sio *sio)
{
	static const u8 endpoints[APPLE_SIO_SETUP_COMMANDS] = {
		0, 0, 0, 0, 3, 3, 0, 0, 0, 0,
	};
	static const u8 tags[APPLE_SIO_SETUP_COMMANDS] = {
		1, 2, 3, 4, 1, 2, 5, 6, 7, 8,
	};
	static const u8 parameters[APPLE_SIO_SETUP_COMMANDS] = {
		0x0f, 0x10, 0x1a, 0x1b, 0x0d,
		0x0e, 0x01, 0x02, 0x0b, 0x0c,
	};
	struct apple_sio_async *request;
	u32 index = sio->setup_index;
	u32 value;

	switch (index) {
	case 0:
		value = lower_32_bits(sio->ep0_general.dma >> 12);
		break;
	case 1:
		value = SZ_16K;
		break;
	case 2:
		value = lower_32_bits(sio->ep0_spi.dma >> 12);
		break;
	case 3:
		value = SZ_8K;
		break;
	case 4:
		value = lower_32_bits(sio->ep3_aux.dma >> 12);
		break;
	case 5:
		value = SZ_16K;
		break;
	case 6:
		value = lower_32_bits(sio->desc0.dma >> 12);
		break;
	case 7:
		value = APPLE_SIO_DESC0_COUNT;
		break;
	case 8:
		value = lower_32_bits(sio->desc1.dma >> 12);
		break;
	case 9:
		value = APPLE_SIO_DESC1_COUNT;
		break;
	default:
		sio->setup_status = -EINVAL;
		complete_all(&sio->setup_done);
		return;
	}

	request = apple_sio_request_async_common(sio, endpoints[index], 0x03,
						 parameters[index], value, 0x65,
						 apple_sio_setup_done, sio, true,
						 tags[index]);
	if (IS_ERR(request)) {
		sio->setup_status = PTR_ERR(request);
		complete_all(&sio->setup_done);
		return;
	}

	apple_sio_async_put(request);
}

int apple_sio_async_cancel(struct apple_sio_async *request)
{
	struct apple_sio *sio = request->sio;
	unsigned long flags;
	bool cancelled = false;

	xa_lock_irqsave(&sio->pending, flags);
	if (request->active &&
	    xa_load(&sio->pending, request->key) == request) {
		__xa_erase(&sio->pending, request->key);
		request->active = false;
		cancelled = true;
	}
	xa_unlock_irqrestore(&sio->pending, flags);

	if (cancelled) {
		apple_sio_async_finish(request, -ECANCELED, 0, false);
		return 0;
	}

	wait_for_completion(&request->done);
	return -EALREADY;
}
EXPORT_SYMBOL_GPL(apple_sio_async_cancel);

int apple_sio_request(struct apple_sio *sio, u8 endpoint, u8 opcode,
		      u8 parameter, u32 data, u8 expected_opcode,
		      unsigned long timeout, u64 *response)
{
	struct apple_sio_async *request;
	bool timed_out = false;
	int ret;

	request = apple_sio_request_async(sio, endpoint, opcode, parameter,
					  data, expected_opcode, NULL, NULL);
	if (IS_ERR(request))
		return PTR_ERR(request);

	if (!wait_for_completion_timeout(&request->done, timeout)) {
		if (!apple_sio_async_cancel(request))
			timed_out = true;
	}

	ret = timed_out ? -ETIMEDOUT : request->status;
	if (!ret && response)
		*response = request->response;

	apple_sio_async_put(request);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_sio_request);

static void apple_sio_fail_pending(struct apple_sio *sio)
{
	struct apple_sio_async *request;
	LIST_HEAD(shutdown);
	unsigned long index;
	unsigned long flags;

	xa_lock_irqsave(&sio->pending, flags);
	sio->stopping = true;
	xa_for_each(&sio->pending, index, request) {
		__xa_erase(&sio->pending, index);
		request->active = false;
		list_add_tail(&request->shutdown_node, &shutdown);
	}
	xa_unlock_irqrestore(&sio->pending, flags);

	while (!list_empty(&shutdown)) {
		request = list_first_entry(&shutdown, struct apple_sio_async,
					   shutdown_node);
		list_del_init(&request->shutdown_node);
		apple_sio_async_finish(request, -ESHUTDOWN, 0, true);
	}
}

static void apple_sio_stop(struct apple_sio *sio)
{
	u32 control;

	WRITE_ONCE(sio->running, false);
	WRITE_ONCE(sio->stopping, true);

	mutex_lock(&sio->tx_lock);
	spin_lock_irq(&sio->tx_hw_lock);
	control = readl(sio->base + APPLE_SIO_CPU_CONTROL);
	writel(control & ~APPLE_SIO_CPU_RUN,
	       sio->base + APPLE_SIO_CPU_CONTROL);
	readl(sio->base + APPLE_SIO_CPU_CONTROL);
	writel(0, sio->base + APPLE_SIO_REMAP_ENABLE);
	spin_unlock_irq(&sio->tx_hw_lock);
	mutex_unlock(&sio->tx_lock);

	apple_sio_fail_pending(sio);
}

static int apple_sio_start(struct apple_sio *sio, const char *firmware_name)
{
	const struct firmware *firmware;
	dma_addr_t dma;
	void *buffer;
	size_t size;
	u64 counter;
	unsigned long ready_timeout;
	u32 control;
	u32 status;
	int ret;

	control = readl(sio->base + APPLE_SIO_CPU_CONTROL);
	if (control & APPLE_SIO_CPU_RUN)
		return dev_err_probe(sio->dev, -EBUSY,
				     "SmartIO CPU is already running\n");

	ret = request_firmware(&firmware, firmware_name, sio->dev);
	if (ret)
		return dev_err_probe(sio->dev, ret,
				     "failed to load firmware %s\n",
				     firmware_name);

	size = ALIGN(firmware->size, SZ_4K);
	buffer = dma_alloc_coherent(sio->dev, size, &dma, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto out_release_firmware;
	}

	memset(buffer, 0, size);
	memcpy(buffer, firmware->data, firmware->size);
	dma_wmb();

	sio->firmware_buffer = buffer;
	sio->firmware_dma = dma;
	sio->firmware_size = size;

	writel(0, sio->base + APPLE_SIO_REMAP_IOP_LO);
	writel(0, sio->base + APPLE_SIO_REMAP_IOP_HI);
	writel(lower_32_bits(size), sio->base + APPLE_SIO_REMAP_SIZE_LO);
	writel(upper_32_bits(size), sio->base + APPLE_SIO_REMAP_SIZE_HI);
	writel(lower_32_bits(dma), sio->base + APPLE_SIO_REMAP_AP_LO);
	writel(upper_32_bits(dma), sio->base + APPLE_SIO_REMAP_AP_HI);
	writel(1, sio->base + APPLE_SIO_REMAP_ENABLE);

	writel(0x1111, sio->base + APPLE_SIO_QUEUE_INIT_A);
	writel(0x1111, sio->base + APPLE_SIO_QUEUE_INIT_B);
	writel(1, sio->base + APPLE_SIO_QUEUE_START_A);
	writel(1, sio->base + APPLE_SIO_QUEUE_START_B);
	writel(0x1000, sio->base + APPLE_SIO_QUEUE_START_A);
	writel(0x1000, sio->base + APPLE_SIO_QUEUE_START_B);
	status = readl(sio->base + APPLE_SIO_RX_STATUS);
	writel(status | APPLE_SIO_MBOX_ENABLE,
	       sio->base + APPLE_SIO_RX_STATUS);

	control = readl(sio->base + APPLE_SIO_TIME_CONTROL);
	writel(control & ~BIT(1), sio->base + APPLE_SIO_TIME_CONTROL);
	counter = arch_timer_read_counter();
	writel(lower_32_bits(counter), sio->base + APPLE_SIO_TIME_LO);
	writel(upper_32_bits(counter), sio->base + APPLE_SIO_TIME_HI);
	writel(control | BIT(1), sio->base + APPLE_SIO_TIME_CONTROL);

	control = readl(sio->base + APPLE_SIO_CPU_CONTROL);
	writel(control | APPLE_SIO_CPU_RUN,
	       sio->base + APPLE_SIO_CPU_CONTROL);

	ready_timeout = msecs_to_jiffies(APPLE_SIO_READY_TIMEOUT_MS);
	if (!wait_for_completion_timeout(&sio->ready, ready_timeout)) {
		ret = -ETIMEDOUT;
		goto out_stop;
	}

	WRITE_ONCE(sio->running, true);
	release_firmware(firmware);
	dev_dbg(sio->dev, "SmartIO firmware ready\n");
	return 0;

out_stop:
	apple_sio_stop(sio);
	dma_free_coherent(sio->dev, size, buffer, dma);
	sio->firmware_buffer = NULL;
	dev_err(sio->dev, "SmartIO firmware did not become ready\n");
out_release_firmware:
	release_firmware(firmware);
	return ret;
}

static int apple_sio_alloc_region(struct apple_sio *sio,
				  struct apple_sio_region *region, size_t size)
{
	region->buffer = dmam_alloc_coherent(sio->dev, size, &region->dma,
					     GFP_KERNEL);
	if (!region->buffer)
		return -ENOMEM;

	region->size = size;
	memset(region->buffer, 0, size);
	return 0;
}

static int apple_sio_alloc_shared_regions(struct apple_sio *sio)
{
	int ret;

	ret = apple_sio_alloc_region(sio, &sio->ep0_general, SZ_16K);
	if (ret)
		return ret;

	ret = apple_sio_alloc_region(sio, &sio->ep0_spi, SZ_8K);
	if (ret)
		return ret;

	ret = apple_sio_alloc_region(sio, &sio->ep3_aux, SZ_16K);
	if (ret)
		return ret;

	ret = apple_sio_alloc_region(sio, &sio->desc0,
				     APPLE_SIO_DESC0_SIZE);
	if (ret)
		return ret;

	return apple_sio_alloc_region(sio, &sio->desc1,
				      APPLE_SIO_DESC1_SIZE);
}

static int apple_sio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *firmware_name;
	unsigned long setup_timeout;
	struct apple_sio *sio;
	int irq;
	int ret;

	sio = devm_kzalloc(dev, sizeof(*sio), GFP_KERNEL);
	if (!sio)
		return -ENOMEM;

	sio->dev = dev;
	init_completion(&sio->ready);
	init_completion(&sio->setup_done);
	sio->setup_status = -EINPROGRESS;
	mutex_init(&sio->tx_lock);
	spin_lock_init(&sio->tx_hw_lock);
	ida_init(&sio->tags);
	xa_init(&sio->pending);
	spin_lock_init(&sio->desc0_lock);
	platform_set_drvdata(pdev, sio);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto out_destroy_allocators;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to activate power domain\n");
		goto out_destroy_allocators;
	}

	sio->base = devm_platform_ioremap_resource_byname(pdev, "a7iop");
	if (IS_ERR(sio->base)) {
		ret = PTR_ERR(sio->base);
		goto out_pm_put;
	}

	irq = platform_get_irq(pdev, APPLE_SIO_RX_IRQ_INDEX);
	if (irq < 0) {
		ret = irq;
		goto out_pm_put;
	}

	ret = devm_request_irq(dev, irq, apple_sio_irq, 0,
			       dev_name(dev), sio);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to request RX interrupt\n");
		goto out_pm_put;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to set DMA mask\n");
		goto out_pm_put;
	}

	sio->desc0_bitmap = devm_bitmap_zalloc(dev, APPLE_SIO_DESC0_COUNT,
					       GFP_KERNEL);
	if (!sio->desc0_bitmap) {
		ret = -ENOMEM;
		goto out_pm_put;
	}

	ret = apple_sio_alloc_shared_regions(sio);
	if (ret)
		goto out_pm_put;

	ret = of_property_read_string(dev->of_node, "firmware-name",
				      &firmware_name);
	if (ret) {
		ret = dev_err_probe(dev, ret, "missing firmware-name\n");
		goto out_pm_put;
	}

	ret = apple_sio_start(sio, firmware_name);
	if (ret)
		goto out_pm_put;

	setup_timeout = msecs_to_jiffies(APPLE_SIO_REQUEST_TIMEOUT_MS);
	if (!wait_for_completion_timeout(&sio->setup_done, setup_timeout)) {
		ret = -ETIMEDOUT;
		dev_err(dev,
			"SmartIO setup timed out: index=%u irqs=%u unmatched=%u irq=%#x/%#x rx_status=%#x tx_status=%#x rx=%#llx tx=%#llx\n",
			sio->setup_index, sio->irq_count, sio->unmatched_count,
			sio->last_irq_legacy, sio->last_irq,
			sio->last_rx_status, sio->last_tx_status,
			sio->last_rx_message, sio->last_tx_message);
		goto out_stop;
	}
	ret = sio->setup_status;
	if (ret) {
		dev_err_probe(dev, ret, "SmartIO shared-region setup failed\n");
		goto out_stop;
	}

	sio->dma_pdev = platform_device_register_data(dev, "apple-sio-dma",
						      PLATFORM_DEVID_AUTO,
						      &sio, sizeof(sio));
	if (IS_ERR(sio->dma_pdev)) {
		ret = dev_err_probe(dev, PTR_ERR(sio->dma_pdev),
				    "failed to register DMA child\n");
		sio->dma_pdev = NULL;
		goto out_stop;
	}

	return 0;

out_stop:
	apple_sio_stop(sio);
	dma_free_coherent(dev, sio->firmware_size, sio->firmware_buffer,
			  sio->firmware_dma);
	sio->firmware_buffer = NULL;

out_pm_put:
	pm_runtime_put_sync(dev);
out_destroy_allocators:
	xa_destroy(&sio->pending);
	ida_destroy(&sio->tags);
	return ret;
}

static void apple_sio_remove(struct platform_device *pdev)
{
	struct apple_sio *sio = platform_get_drvdata(pdev);

	platform_device_unregister(sio->dma_pdev);
	apple_sio_stop(sio);
	dma_free_coherent(sio->dev, sio->firmware_size,
			  sio->firmware_buffer, sio->firmware_dma);
	pm_runtime_put_sync(sio->dev);
	xa_destroy(&sio->pending);
	ida_destroy(&sio->tags);
}

static const struct of_device_id apple_sio_of_match[] = {
	{ .compatible = "apple,t8010-sio" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_sio_of_match);

static struct platform_driver apple_sio_driver = {
	.driver = {
		.name = "apple-sio",
		.of_match_table = apple_sio_of_match,
	},
	.probe = apple_sio_probe,
	.remove = apple_sio_remove,
};
module_platform_driver(apple_sio_driver);

MODULE_FIRMWARE("apple/t8010-smartio.bin");
MODULE_DESCRIPTION("Apple T8010 SmartIO coprocessor driver");
MODULE_LICENSE("Dual MIT/GPL");

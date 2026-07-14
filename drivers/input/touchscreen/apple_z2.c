// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Z2 touchscreen driver
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bitrev.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define APPLE_Z2_NUM_FINGERS_OFFSET      16
#define APPLE_Z2_FINGERS_OFFSET          24
#define APPLE_Z2_TOUCH_STARTED           3
#define APPLE_Z2_TOUCH_MOVED             4
#define APPLE_Z2_CMD_READ_INTERRUPT_DATA 0xEB
#define APPLE_Z2_REPLY_INTERRUPT_DATA    0xE1
#define APPLE_Z2_FRAME_INPUT_REPORT      0x90
#define APPLE_Z2_INTERFACE_GRAPE         BIT(1)
#define APPLE_Z2_HID_REPORT_ID           0x76
#define APPLE_Z2_J172_HEADER_SIZE         32
#define APPLE_Z2_J172_CONTACT_SIZE        48
#define APPLE_Z2_J172_MAX_CONTACTS        32
#define APPLE_Z2_J172_FLOAT_FORMAT        BIT(1)
#define APPLE_Z2_REPORT_SURFACE           0xd9
#define APPLE_Z2_SURFACE_DESCRIPTOR_SIZE  16
#define APPLE_Z2_CMD_DEVICE_INFO         0xE2
#define APPLE_Z2_CMD_REPORT_INFO         0xE3
#define APPLE_Z2_CMD_CTRL_WRITE_SHORT    0xE4
#define APPLE_Z2_CMD_CTRL_READ_SHORT     0xE6
#define APPLE_Z2_CMD_CTRL_READ_LONG      0xE7
#define APPLE_Z2_CMD_LAST                0xE1
#define APPLE_Z2_CMD_WAKE                0xEE
#define APPLE_Z2_CMD_SIZE                16
#define APPLE_Z2_RAW_XFER_MAX_SIZE       64
#define APPLE_Z2_HBPP_CMD_BLOB           0x3001
#define APPLE_Z2_FW_MAGIC                0x5746325A
#define APPLE_Z2_RX_BUF_SIZE             4000
#define LOAD_COMMAND_INIT_PAYLOAD        0
#define LOAD_COMMAND_SEND_BLOB           1
#define LOAD_COMMAND_SEND_CALIBRATION    2
#define LOAD_COMMAND_WAIT_IRQ            3
#define LOAD_COMMAND_SET_CONFIG          4
#define LOAD_COMMAND_RAW_XFER            5
#define APPLE_Z2_RAW_XFER_RX_BIT_REVERSE BIT(0)
#define CAL_PROP_NAME                    "apple,z2-cal-blob"

#define APPLE_Z2_FW_CONFIG_WORDS         11
#define APPLE_Z2_FW_CONFIG_MIN_DMA       BIT(0)
#define APPLE_Z2_FW_CONFIG_Z2_DELAY      BIT(1)
#define APPLE_Z2_FW_CONFIG_CS_DELAY      BIT(2)
#define APPLE_Z2_FW_CONFIG_CPHA          BIT(3)
#define APPLE_Z2_FW_CONFIG_CPOL          BIT(4)

enum apple_z2_protocol_state {
	APPLE_Z2_STATE_OFF,
	APPLE_Z2_STATE_POWERED,
	APPLE_Z2_STATE_SPI_CONFIGURED,
	APPLE_Z2_STATE_BOOT_IRQ,
	APPLE_Z2_STATE_HBPP_READY,
	APPLE_Z2_STATE_FIRMWARE_READY,
	APPLE_Z2_STATE_DEVICE_INFO,
	APPLE_Z2_STATE_RUNTIME,
	APPLE_Z2_STATE_REPORTS_READY,
};

struct apple_z2 {
	struct spi_device *spidev;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_ana_gpio;
	struct gpio_desc *power_ldo_gpio;
	struct gpio_desc *display_sync_gpio;
	struct gpio_desc *display_sync1_gpio;
	struct pinctrl *j172_sync_pinctrl;
	struct pinctrl_state *j172_sync_active_state;
	struct clk *clk;
	struct input_dev *input_dev;
	struct completion boot_irq;
	struct mutex io_lock; /* Serializes command and IRQ transfers. */
	bool j172;
	bool no_init_ack;
	bool booted;
	bool clk_enabled;
	bool platform_powered;
	bool upload_irq_masked;
	bool runtime_frame_valid;
	bool surface_descriptor_valid;
	bool j172_sync_pinctrl_active;
	enum apple_z2_protocol_state protocol_state;
	unsigned int bpw16_min_len;
	unsigned int z2_inter_packet_delay_us;
	unsigned int z2_cs_delay_us;
	s16 sensor_max_x;
	s16 sensor_min_x;
	s16 sensor_max_y;
	s16 sensor_min_y;
	int index_parity;
	struct touchscreen_properties props;
	const char *fw_name;
	u8 *tx_buf;
	u8 *rx_buf;
};

struct apple_z2_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_z2_hbpp_blob_hdr {
	__le16 cmd;
	__le16 len;
	__le32 addr;
	__le16 checksum;
};

struct apple_z2_fw_hdr {
	__le32 magic;
	__le32 version;
};

static void apple_z2_apply_z2_delays(struct apple_z2 *z2);
static void apple_z2_post_z2_xfer_delay(struct apple_z2 *z2);
static void apple_z2_set_gpio(struct gpio_desc *gpio, int value);
static void apple_z2_z2_checksum(u8 *cmd, unsigned int len);

static bool apple_z2_float_to_integer(const u8 *raw, s64 *value)
{
	u32 bits = get_unaligned_le32(raw);
	u32 exponent = (bits >> 23) & 0xff;
	u64 significand;
	u64 magnitude;
	int shift;

	if (exponent == 0xff)
		return false;
	if (!exponent) {
		*value = 0;
		return true;
	}

	significand = BIT_ULL(23) | (bits & GENMASK(22, 0));
	shift = exponent - 127 - 23;
	if (shift >= 0) {
		if (shift > 7)
			return false;
		magnitude = significand << shift;
	} else if (-shift >= 64) {
		magnitude = 0;
	} else {
		magnitude = DIV_ROUND_CLOSEST_ULL(significand,
						  BIT_ULL(-shift));
	}

	if (magnitude > S32_MAX)
		return false;
	*value = bits & BIT(31) ? -(s64)magnitude : magnitude;
	return true;
}

static bool apple_z2_scale_j172_coord(const u8 *raw, s16 maximum,
				      s16 minimum, unsigned int pixels,
				      unsigned int *position)
{
	s64 coordinate;
	s64 numerator;
	s64 denominator;

	if (maximum <= minimum || !apple_z2_float_to_integer(raw, &coordinate))
		return false;

	/* The report uses micrometres; the descriptor uses 0.01 mm. */
	numerator = coordinate - 10LL * minimum;
	denominator = 10LL * (maximum - minimum);
	if (numerator <= 0) {
		*position = 0;
	} else if (numerator >= denominator) {
		*position = pixels;
	} else {
		*position = DIV_ROUND_CLOSEST_ULL((u64)numerator * pixels,
						  denominator);
	}

	return true;
}

static void apple_z2_parse_j172_touches(struct apple_z2 *z2,
					const u8 *msg, size_t msg_len)
{
	unsigned int contact_offset;
	unsigned int contact_bytes;
	unsigned int nfingers;
	unsigned int x;
	unsigned int y;
	unsigned int i;
	const u8 *contact;
	int slot;
	bool active;
	bool coords_valid;

	if (msg_len < APPLE_Z2_J172_HEADER_SIZE)
		return;

	contact_offset = msg[2] + msg[14];
	contact_bytes = get_unaligned_le16(msg + 16);
	nfingers = msg[22];
	if (msg[2] < APPLE_Z2_J172_HEADER_SIZE ||
	    !(get_unaligned_le16(msg + 12) & APPLE_Z2_J172_FLOAT_FORMAT) ||
	    nfingers > APPLE_Z2_J172_MAX_CONTACTS ||
	    contact_bytes != nfingers * APPLE_Z2_J172_CONTACT_SIZE ||
	    contact_offset > msg_len ||
	    contact_bytes > msg_len - contact_offset) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid J172 touch packet layout len=%zu header=%u contacts=%u bytes=%u\n",
				     msg_len, msg[2], nfingers, contact_bytes);
		return;
	}

	if (nfingers && !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "J172 touch packet without surface descriptor\n");
		return;
	}

	contact = msg + contact_offset;
	for (i = 0; i < nfingers; i++, contact += APPLE_Z2_J172_CONTACT_SIZE) {
		slot = input_mt_get_slot_by_key(z2->input_dev, contact[0]);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev,
				 "unable to get slot for J172 finger\n");
			continue;
		}

		active = contact[1] == APPLE_Z2_TOUCH_STARTED ||
			 contact[1] == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, active))
			continue;

		coords_valid = apple_z2_scale_j172_coord(contact + 4,
							 z2->sensor_max_x,
							 z2->sensor_min_x,
							 z2->props.max_x, &x);
		if (coords_valid)
			coords_valid = apple_z2_scale_j172_coord(contact + 8,
								 z2->sensor_max_y,
								 z2->sensor_min_y,
								 z2->props.max_y, &y);
		if (!coords_valid) {
			dev_warn_ratelimited(&z2->spidev->dev,
					     "invalid J172 touch coordinates\n");
			continue;
		}

		touchscreen_report_pos(z2->input_dev, &z2->props, x,
				       z2->props.max_y - y, true);
	}

	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

static void apple_z2_parse_touches(struct apple_z2 *z2,
				   const u8 *msg, size_t msg_len)
{
	int i;
	int nfingers;
	int slot;
	int slot_valid;
	struct apple_z2_finger *fingers;

	if (msg_len >= APPLE_Z2_J172_HEADER_SIZE &&
	    msg[0] == APPLE_Z2_HID_REPORT_ID && msg[3] == 4) {
		apple_z2_parse_j172_touches(z2, msg, msg_len);
		return;
	}

	if (msg_len <= APPLE_Z2_NUM_FINGERS_OFFSET)
		return;
	nfingers = msg[APPLE_Z2_NUM_FINGERS_OFFSET];
	if (msg_len < APPLE_Z2_FINGERS_OFFSET ||
	    nfingers > (msg_len - APPLE_Z2_FINGERS_OFFSET) /
		       sizeof(*fingers)) {
		dev_warn(&z2->spidev->dev,
			 "invalid touch packet length %zu for %d fingers\n",
			 msg_len, nfingers);
		return;
	}
	fingers = (struct apple_z2_finger *)(msg + APPLE_Z2_FINGERS_OFFSET);
	for (i = 0; i < nfingers; i++) {
		slot = input_mt_get_slot_by_key(z2->input_dev, fingers[i].finger);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev, "unable to get slot for finger\n");
			continue;
		}
		slot_valid = fingers[i].state == APPLE_Z2_TOUCH_STARTED ||
			     fingers[i].state == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, slot_valid))
			continue;
		touchscreen_report_pos(z2->input_dev, &z2->props,
				       le16_to_cpu(fingers[i].abs_x),
				       le16_to_cpu(fingers[i].abs_y),
				       true);
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MAJOR,
				 le16_to_cpu(fingers[i].tool_major));
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MINOR,
				 le16_to_cpu(fingers[i].tool_minor));
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION,
				 le16_to_cpu(fingers[i].orientation));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 le16_to_cpu(fingers[i].touch_major));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 le16_to_cpu(fingers[i].touch_minor));
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

static void apple_z2_dispatch_frame(struct apple_z2 *z2, const u8 *payload,
				    size_t payload_len)
{
	const u8 *report;
	u16 report_len;

	if (payload_len < 4 || payload[0] != APPLE_Z2_FRAME_INPUT_REPORT)
		return;

	report_len = get_unaligned_le16(payload + 2);
	if (report_len != payload_len - 4) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid input report length %u for payload %zu\n",
				     report_len, payload_len);
		return;
	}

	if (!(payload[1] & APPLE_Z2_INTERFACE_GRAPE) || !report_len)
		return;

	report = payload + 4;
	if (report[0] != APPLE_Z2_HID_REPORT_ID)
		return;

	apple_z2_parse_touches(z2, report, report_len);
}

static bool apple_z2_gen2_packet_valid(const u8 *buf, size_t buf_len,
				       u8 counter)
{
	u16 payload_len;
	u16 checksum;
	int i;

	if (buf_len < 7)
		return false;

	payload_len = get_unaligned_le16(buf + 2);
	if (((buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) & 0xff) ||
	    (buf[0] & 0xfe) != 0xea || buf[1] != counter ||
	    payload_len < 2 || payload_len + 5 > buf_len)
		return false;

	checksum = get_unaligned_le16(buf + 5 + payload_len - 2);
	for (i = 0; i < payload_len - 2; i++)
		checksum -= buf[5 + i];

	return !checksum;
}

static int apple_z2_read_packet(struct apple_z2 *z2)
{
	struct spi_transfer xfer = { };
	int error;
	size_t pkt_len;
	u16 payload_len;
	u8 counter;

	z2->runtime_frame_valid = false;
	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
	counter = z2->index_parity + 1;
	z2->tx_buf[1] = counter;
	apple_z2_z2_checksum(z2->tx_buf, APPLE_Z2_CMD_SIZE);
	xfer.tx_buf = z2->tx_buf;
	xfer.rx_buf = z2->rx_buf;
	xfer.len = APPLE_Z2_CMD_SIZE;

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);

	if (z2->rx_buf[0] != APPLE_Z2_REPLY_INTERRUPT_DATA)
		return 0;

	pkt_len = (get_unaligned_le16(z2->rx_buf + 1) + 8) & 0xfffffffc;
	if (pkt_len > APPLE_Z2_RX_BUF_SIZE) {
		dev_warn(&z2->spidev->dev, "packet too large: %zu\n", pkt_len);
		return -EMSGSIZE;
	}

	if (z2->j172) {
		memset(z2->rx_buf, 0xa5, pkt_len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = pkt_len;
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
	} else {
		error = spi_read(z2->spidev, z2->rx_buf, pkt_len);
	}
	if (error)
		return error;

	z2->runtime_frame_valid =
		apple_z2_gen2_packet_valid(z2->rx_buf, pkt_len, counter);
	if (z2->runtime_frame_valid) {
		payload_len = get_unaligned_le16(z2->rx_buf + 2);
		apple_z2_dispatch_frame(z2, z2->rx_buf + 5,
					payload_len - 2);
		z2->index_parity = !z2->index_parity;
	}

	return 0;
}

static void apple_z2_reset_protocol(struct apple_z2 *z2)
{
	z2->protocol_state = APPLE_Z2_STATE_OFF;
	z2->index_parity = 0;
	z2->runtime_frame_valid = false;
	z2->surface_descriptor_valid = false;
}

static int apple_z2_advance_protocol(struct apple_z2 *z2,
				     enum apple_z2_protocol_state expected,
				     enum apple_z2_protocol_state next)
{
	if (z2->protocol_state != expected)
		return -EPROTO;

	z2->protocol_state = next;
	return 0;
}

static void apple_z2_z2_checksum(u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];
	put_unaligned_le16(checksum, cmd + len - 2);
}

static bool apple_z2_z2_checksum_valid(const u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	if (len < 2)
		return false;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];

	return get_unaligned_le16(cmd + len - 2) == checksum;
}

static void apple_z2_set_spi_delay(struct spi_delay *delay,
				   unsigned int usecs)
{
	if (usecs > U16_MAX)
		usecs = U16_MAX;

	delay->value = usecs;
	delay->unit = SPI_DELAY_UNIT_USECS;
}

static unsigned int apple_z2_effective_cs_delay_us(struct apple_z2 *z2)
{
	/* J172 firmware expresses this field in ns; Linux SPI uses us here. */
	return z2->j172 ? 10 : z2->z2_cs_delay_us;
}

static void apple_z2_apply_z2_delays(struct apple_z2 *z2)
{
	unsigned int cs_delay_us = apple_z2_effective_cs_delay_us(z2);
	unsigned int inactive_us = z2->j172 ? 10 :
				  z2->z2_inter_packet_delay_us;

	if (cs_delay_us) {
		apple_z2_set_spi_delay(&z2->spidev->cs_setup, cs_delay_us);
		apple_z2_set_spi_delay(&z2->spidev->cs_hold, cs_delay_us);
	}
	if (inactive_us)
		apple_z2_set_spi_delay(&z2->spidev->cs_inactive, inactive_us);
}

static void apple_z2_post_z2_xfer_delay(struct apple_z2 *z2)
{
	unsigned int delay_us = z2->z2_inter_packet_delay_us;

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static void apple_z2_post_ack_delay(struct apple_z2 *z2)
{
	unsigned int delay_us = apple_z2_effective_cs_delay_us(z2);

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static int apple_z2_z2_xfer_locked_delay(struct apple_z2 *z2,
					 const char *tag, bool post_delay)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = APPLE_Z2_CMD_SIZE,
	};
	int error;

	apple_z2_z2_checksum(z2->tx_buf, APPLE_Z2_CMD_SIZE);
	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error) {
		dev_err(&z2->spidev->dev, "%s transfer failed: %d\n",
			tag, error);
		return error;
	}
	if (post_delay)
		apple_z2_post_z2_xfer_delay(z2);

	dev_dbg(&z2->spidev->dev, "%s tx=%*ph rx=%*ph\n", tag,
		APPLE_Z2_CMD_SIZE, z2->tx_buf, APPLE_Z2_CMD_SIZE,
		z2->rx_buf);
	return 0;
}

static int apple_z2_z2_xfer_locked(struct apple_z2 *z2, const char *tag)
{
	return apple_z2_z2_xfer_locked_delay(z2, tag, true);
}

static int apple_z2_z2_cmd_locked_delay(struct apple_z2 *z2, u8 cmd,
					u8 report, const char *tag,
					bool post_delay)
{
	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = cmd;
	z2->tx_buf[1] = report;

	return apple_z2_z2_xfer_locked_delay(z2, tag, post_delay);
}

static int apple_z2_z2_cmd_locked(struct apple_z2 *z2, u8 cmd, u8 report,
				  const char *tag)
{
	return apple_z2_z2_cmd_locked_delay(z2, cmd, report, tag, true);
}

static int apple_z2_read_device_info_locked(struct apple_z2 *z2)
{
	u16 max_packet_size;
	u8 interface_version;
	u8 platform_id;
	int error;

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_WAKE, 0,
					     "device-info-wake", false);
	if (error)
		return error;
	usleep_range(2000, 2100);

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_DEVICE_INFO, 0,
					     "device-info", false);
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_LAST, 0,
					     "device-info-last", false);
	if (error)
		return error;

	if (z2->rx_buf[0] != APPLE_Z2_CMD_DEVICE_INFO ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
		return -EPROTO;

	platform_id = z2->rx_buf[1];
	interface_version = z2->rx_buf[2];
	max_packet_size = get_unaligned_le16(z2->rx_buf + 3);

	dev_dbg(&z2->spidev->dev,
		"device info: platform=%u interface=%u max-packet=%u\n",
		 platform_id, interface_version, max_packet_size);
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_FIRMWARE_READY,
					 APPLE_Z2_STATE_DEVICE_INFO);
}

static int apple_z2_write_report_locked(struct apple_z2 *z2, u8 report,
					const u8 *data, u8 len,
					const char *tag)
{
	char last_tag[32];
	int error;

	if (len > 11)
		return -EINVAL;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_CTRL_WRITE_SHORT;
	z2->tx_buf[1] = report;
	z2->tx_buf[2] = len;
	memcpy(z2->tx_buf + 3, data, len);
	error = apple_z2_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_LAST;
	snprintf(last_tag, sizeof(last_tag), "%s-last", tag);

	return apple_z2_z2_xfer_locked(z2, last_tag);
}

static int apple_z2_read_report_info_locked(struct apple_z2 *z2, u8 report,
					    u8 *status, u16 *len,
					    const char *tag)
{
	char xfer_tag[40];
	unsigned int i;
	int error;

	for (i = 0; i < 2; i++) {
		snprintf(xfer_tag, sizeof(xfer_tag), "%s-info-%u", tag, i + 1);
		error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_REPORT_INFO,
					       report, xfer_tag);
		if (error)
			return error;
	}

	if (z2->rx_buf[0] != APPLE_Z2_CMD_REPORT_INFO ||
	    z2->rx_buf[1] != report ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
		return -EPROTO;

	*status = z2->rx_buf[2];
	*len = get_unaligned_le16(z2->rx_buf + 3);
	dev_dbg(&z2->spidev->dev,
		"J172 %s report=%02x status=%u len=%u\n", tag, report,
		*status, *len);

	return 0;
}

static int apple_z2_read_report_locked(struct apple_z2 *z2, u8 report,
				       u16 len, const char *tag)
{
	struct spi_transfer xfer = { };
	char last_tag[32];
	unsigned int data_len;
	int error;

	if (!len || len > APPLE_Z2_RX_BUF_SIZE - 5)
		return -EMSGSIZE;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = len <= 11 ? APPLE_Z2_CMD_CTRL_READ_SHORT :
					 APPLE_Z2_CMD_CTRL_READ_LONG;
	z2->tx_buf[1] = report;
	put_unaligned_le16(len, z2->tx_buf + 3);
	error = apple_z2_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	if (len <= 11) {
		snprintf(last_tag, sizeof(last_tag), "%s-last", tag);
		error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_LAST, 0,
					       last_tag);
		if (!error)
			dev_dbg(&z2->spidev->dev, "J172 %s short read done\n",
				tag);
		return error;
	}

	data_len = len + 5;
	memset(z2->rx_buf, 0xa5, data_len);
	xfer.tx_buf = z2->rx_buf;
	xfer.rx_buf = z2->rx_buf;
	xfer.len = data_len;
	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);
	dev_dbg(&z2->spidev->dev, "J172 %s long read len=%u\n", tag,
		data_len);

	return 0;
}

static int apple_z2_store_surface_descriptor(struct apple_z2 *z2, u16 len)
{
	const u8 *descriptor = z2->rx_buf + 3;
	unsigned int x_pixels;
	unsigned int y_pixels;
	unsigned int x_res;
	unsigned int y_res;
	s16 max_x;
	s16 min_x;
	s16 max_y;
	s16 min_y;
	u32 width;
	u32 height;

	if (len != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE ||
	    z2->rx_buf[0] != APPLE_Z2_CMD_CTRL_READ_LONG ||
	    z2->rx_buf[1] != APPLE_Z2_REPORT_SURFACE || z2->rx_buf[2] ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, len + 5))
		return -EPROTO;

	width = get_unaligned_le32(descriptor);
	height = get_unaligned_le32(descriptor + 4);
	min_x = (s16)get_unaligned_le16(descriptor + 8);
	min_y = (s16)get_unaligned_le16(descriptor + 10);
	max_x = (s16)get_unaligned_le16(descriptor + 12);
	max_y = (s16)get_unaligned_le16(descriptor + 14);
	if (!width || !height || max_x <= min_x || max_y <= min_y)
		return -EPROTO;

	z2->sensor_max_x = max_x;
	z2->sensor_min_x = min_x;
	z2->sensor_max_y = max_y;
	z2->sensor_min_y = min_y;
	z2->surface_descriptor_valid = true;

	x_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_X) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_X) + 1;
	y_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_Y) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_Y) + 1;
	x_res = max(DIV_ROUND_CLOSEST(x_pixels * 100, width), 1U);
	y_res = max(DIV_ROUND_CLOSEST(y_pixels * 100, height), 1U);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_Y, y_res);
	if (test_bit(ABS_X, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_X, x_res);
	if (test_bit(ABS_Y, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_Y, y_res);

	dev_dbg(&z2->spidev->dev,
		"surface %ux%u, bounds x=%d..%d y=%d..%d, resolution %ux%u units/mm\n",
		 width, height, min_x, max_x, min_y, max_y, x_res, y_res);

	return 0;
}

static int apple_z2_j172_init_reads_locked(struct apple_z2 *z2)
{
	u8 status;
	u16 db_len;
	u16 prop_len;
	u16 surface_len;
	unsigned int i;
	int error;

	error = apple_z2_read_report_info_locked(z2, APPLE_Z2_REPORT_SURFACE,
						 &status, &surface_len, "d9");
	if (error)
		return error;
	if (status || surface_len != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, APPLE_Z2_REPORT_SURFACE,
					    surface_len, "d9-read");
	if (error)
		return error;
	error = apple_z2_store_surface_descriptor(z2, surface_len);
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xdb, &status, &db_len,
						 "db");
	if (error)
		return error;
	if (status || db_len != 0x7d)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, 0xdb, db_len, "db-read-1");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0x73, &status, &prop_len,
						 "73");
	if (error)
		return error;
	if (status || prop_len != 8)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, 0x73, prop_len, "73-read-1");
	if (error)
		return error;

	for (i = 2; i <= 3; i++) {
		char db_tag[16];
		char prop_tag[16];

		snprintf(db_tag, sizeof(db_tag), "db-read-%u", i);
		error = apple_z2_read_report_locked(z2, 0xdb, db_len, db_tag);
		if (error)
			return error;

		snprintf(prop_tag, sizeof(prop_tag), "73-read-%u", i);
		error = apple_z2_read_report_locked(z2, 0x73, prop_len,
						    prop_tag);
		if (error)
			return error;
	}

	return 0;
}

static int apple_z2_j172_init_writes_locked(struct apple_z2 *z2)
{
	static const u8 rep9d[] = { 0x01, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00 };
	static const u8 repbf[] = { 0x9d, 0x81, 0x09, 0x00 };
	u8 status;
	u16 len;
	int error;

	error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_WAKE, 0, "j172-wake");
	if (error)
		return error;
	usleep_range(2200, 2300);

	error = apple_z2_read_report_info_locked(z2, 0x9d, &status, &len,
						 "9d");
	if (error)
		return error;
	if (status || len != sizeof(rep9d))
		return -EPROTO;

	error = apple_z2_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-1");
	if (error)
		return error;
	usleep_range(50500, 50700);
	error = apple_z2_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-2");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xbf, &status, &len,
						 "bf");
	if (error)
		return error;
	if (status || len != sizeof(repbf))
		return -EPROTO;
	error = apple_z2_write_report_locked(z2, 0xbf, repbf,
					     sizeof(repbf), "j172-bf");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xaf, &status, &len,
						 "af");
	if (error)
		return error;
	if (status != 2 || len)
		return -EPROTO;

	dev_dbg(&z2->spidev->dev, "J172 report initialization complete\n");
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_RUNTIME,
					 APPLE_Z2_STATE_REPORTS_READY);
}

static irqreturn_t apple_z2_irq(int irq, void *data)
{
	struct apple_z2 *z2 = data;

	if (unlikely(!z2->booted)) {
		complete(&z2->boot_irq);
	} else {
		mutex_lock(&z2->io_lock);
		apple_z2_read_packet(z2);
		mutex_unlock(&z2->io_lock);
	}

	return IRQ_HANDLED;
}

static void apple_z2_set_gpio(struct gpio_desc *gpio, int value)
{
	if (gpio)
		gpiod_set_value_cansleep(gpio, value);
}

static void apple_z2_pulse_gpio_pair(struct gpio_desc *gpio0,
				     struct gpio_desc *gpio1)
{
	apple_z2_set_gpio(gpio0, 1);
	apple_z2_set_gpio(gpio1, 1);
	usleep_range(1000, 2000);
	apple_z2_set_gpio(gpio0, 0);
	apple_z2_set_gpio(gpio1, 0);
	usleep_range(1000, 2000);
}

static int apple_z2_j172_sync_claim_gpios(struct apple_z2 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	if (!z2->display_sync1_gpio) {
		z2->display_sync1_gpio =
			devm_gpiod_get_optional(dev, "display-sync1",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync1_gpio)) {
			error = PTR_ERR(z2->display_sync1_gpio);
			z2->display_sync1_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio) {
		z2->display_sync_gpio =
			devm_gpiod_get_optional(dev, "display-sync",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync_gpio)) {
			error = PTR_ERR(z2->display_sync_gpio);
			z2->display_sync_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio || !z2->display_sync1_gpio)
		return -ENODEV;

	return 0;
}

static int apple_z2_j172_sync_set_inactive(struct apple_z2 *z2)
{
	int error;

	if (z2->j172_sync_pinctrl_active) {
		devm_pinctrl_put(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		z2->j172_sync_active_state = NULL;
		z2->j172_sync_pinctrl_active = false;
	}

	error = apple_z2_j172_sync_claim_gpios(z2);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync1_gpio);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync_gpio);
	if (error)
		return error;

	dev_dbg(&z2->spidev->dev, "display-sync pins inactive\n");
	return 0;
}

static int apple_z2_j172_sync_set_active(struct apple_z2 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	error = apple_z2_j172_sync_set_inactive(z2);
	if (error)
		return error;

	devm_gpiod_put(dev, z2->display_sync1_gpio);
	z2->display_sync1_gpio = NULL;
	devm_gpiod_put(dev, z2->display_sync_gpio);
	z2->display_sync_gpio = NULL;

	z2->j172_sync_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(z2->j172_sync_pinctrl)) {
		error = PTR_ERR(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		goto reclaim_gpios;
	}

	z2->j172_sync_active_state =
		pinctrl_lookup_state(z2->j172_sync_pinctrl,
				     "display-sync-active");
	if (IS_ERR(z2->j172_sync_active_state)) {
		error = PTR_ERR(z2->j172_sync_active_state);
		z2->j172_sync_active_state = NULL;
		goto put_pinctrl;
	}

	error = pinctrl_select_state(z2->j172_sync_pinctrl,
				     z2->j172_sync_active_state);
	if (error)
		goto put_pinctrl;

	z2->j172_sync_pinctrl_active = true;
	dev_dbg(dev, "display-sync pins active\n");
	return 0;

put_pinctrl:
	devm_pinctrl_put(z2->j172_sync_pinctrl);
	z2->j172_sync_pinctrl = NULL;
	z2->j172_sync_active_state = NULL;
reclaim_gpios:
	if (apple_z2_j172_sync_claim_gpios(z2))
		dev_warn(dev, "failed to reclaim display-sync GPIOs\n");
	return error;
}

static void apple_z2_platform_power_off(struct apple_z2 *z2)
{
	int error;

	if (z2->clk_enabled) {
		clk_disable_unprepare(z2->clk);
		z2->clk_enabled = false;
	}
	if (z2->j172)
		usleep_range(1000, 2000);

	if (z2->j172) {
		error = apple_z2_j172_sync_set_inactive(z2);
		if (error)
			dev_warn(&z2->spidev->dev,
				 "failed to set display-sync inactive: %d\n",
				 error);
	} else {
		apple_z2_set_gpio(z2->display_sync1_gpio, 0);
		apple_z2_set_gpio(z2->display_sync_gpio, 0);
	}
	apple_z2_set_gpio(z2->power_ldo_gpio, 0);
	apple_z2_set_gpio(z2->power_ana_gpio, 0);
	z2->platform_powered = false;
}

static int apple_z2_enable_clock(struct apple_z2 *z2)
{
	int error;

	if (!z2->clk || z2->clk_enabled)
		return 0;

	error = clk_prepare_enable(z2->clk);
	if (error)
		return error;

	z2->clk_enabled = true;
	return 0;
}

static int apple_z2_j172_power_on(struct apple_z2 *z2)
{
	int error;

	apple_z2_platform_power_off(z2);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	msleep(50);

	apple_z2_set_gpio(z2->power_ana_gpio, 1);
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	error = apple_z2_j172_sync_set_active(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	error = apple_z2_enable_clock(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	usleep_range(15000, 16000);
	z2->platform_powered = true;
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_OFF,
					 APPLE_Z2_STATE_POWERED);
}

static int apple_z2_platform_power_on(struct apple_z2 *z2)
{
	int error;

	if (z2->platform_powered)
		return 0;

	if (z2->j172)
		return apple_z2_j172_power_on(z2);

	apple_z2_set_gpio(z2->power_ana_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_pulse_gpio_pair(z2->display_sync_gpio,
				 z2->display_sync1_gpio);

	error = apple_z2_enable_clock(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	usleep_range(1000, 2000);
	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_STATE_POWERED;
	return 0;
}

static int apple_z2_j172_reset(struct apple_z2 *z2)
{
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	usleep_range(1000, 2000);
	return 0;
}

/* Build calibration blob, caller is responsible for freeing the blob data. */
static const u8 *apple_z2_build_cal_blob(struct apple_z2 *z2,
					 u32 address, size_t *size)
{
	u8 *cal_data;
	int cal_size;
	size_t blob_size;
	u32 checksum;
	u16 checksum_hdr;
	int i;
	struct apple_z2_hbpp_blob_hdr *hdr;
	int error;

	if (!device_property_present(&z2->spidev->dev, CAL_PROP_NAME))
		return NULL;

	cal_size = device_property_count_u8(&z2->spidev->dev, CAL_PROP_NAME);
	if (cal_size < 0)
		return ERR_PTR(cal_size);

	blob_size = sizeof(struct apple_z2_hbpp_blob_hdr) + cal_size + sizeof(__le32);
	u8 *blob_data __free(kfree) = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data)
		return ERR_PTR(-ENOMEM);

	hdr = (struct apple_z2_hbpp_blob_hdr *)blob_data;
	hdr->cmd = cpu_to_le16(APPLE_Z2_HBPP_CMD_BLOB);
	hdr->len = cpu_to_le16(round_up(cal_size, 4) / 4);
	hdr->addr = cpu_to_le32(address);

	checksum_hdr = 0;
	for (i = 2; i < 8; i++)
		checksum_hdr += blob_data[i];
	hdr->checksum = cpu_to_le16(checksum_hdr);

	cal_data = blob_data + sizeof(struct apple_z2_hbpp_blob_hdr);
	error = device_property_read_u8_array(&z2->spidev->dev, CAL_PROP_NAME,
					      cal_data, cal_size);
	if (error)
		return ERR_PTR(error);

	checksum = 0;
	for (i = 0; i < cal_size; i++)
		checksum += cal_data[i];
	put_unaligned_le32(checksum, cal_data + cal_size);

	*size = blob_size;
	return no_free_ptr(blob_data);
}

static int apple_z2_send_firmware_blob(struct apple_z2 *z2, const u8 *data,
				       u32 size, bool init, bool defer_irq_wait)
{
	struct spi_transfer blob_xfer = {
		.tx_buf = data,
		.len = size,
		.bits_per_word = 8,
	};
	struct spi_transfer ack_xfer = {
		.tx_buf = z2->tx_buf,
		.len = 2,
		.bits_per_word = 8,
	};
	bool ready;
	int error;

	if (!init && size >= z2->bpw16_min_len)
		blob_xfer.bits_per_word = 16;

	if (z2->j172 && init) {
		if (size != 4 || data[0] != 0x1a || data[1] != 0xa1 ||
		    data[2] != 0x18 || data[3] != 0xe1)
			return dev_err_probe(&z2->spidev->dev, -EINVAL,
					     "invalid HBPP14 readiness command\n");
		memset(z2->rx_buf, 0, size);
		blob_xfer.rx_buf = z2->rx_buf;
	}

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	reinit_completion(&z2->boot_irq);
	error = spi_sync_transfer(z2->spidev, &blob_xfer, 1);
	if (error)
		return error;

	if (z2->j172 && init) {
		ready = z2->rx_buf[0] == 0x1f && z2->rx_buf[1] == 0x01;
		if (!ready)
			return dev_err_probe(&z2->spidev->dev, -EPROTO,
					     "touch controller not ready\n");
		dev_dbg(&z2->spidev->dev, "HBPP14 ready\n");
	}

	if (!init || !z2->no_init_ack) {
		error = spi_sync_transfer(z2->spidev, &ack_xfer, 1);
		if (error)
			return error;
		apple_z2_post_ack_delay(z2);
	}
	if (z2->j172 && init) {
		error = apple_z2_advance_protocol(z2, APPLE_Z2_STATE_BOOT_IRQ,
						  APPLE_Z2_STATE_HBPP_READY);
		if (error)
			return error;
	}

	if (defer_irq_wait && z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		enable_irq(z2->spidev->irq);
	}

	if (!defer_irq_wait)
		wait_for_completion_timeout(&z2->boot_irq,
					    msecs_to_jiffies(20));

	return 0;
}

static int apple_z2_send_firmware_raw_xfer(struct apple_z2 *z2,
					   const u8 *data, u32 size)
{
	struct spi_transfer xfer = { };
	const u8 *logical_tx;
	u32 tx_len, rx_len, flags, xfer_len;
	unsigned int i;
	int error;

	if (size < 3 * sizeof(__le32))
		return -EINVAL;

	tx_len = get_unaligned_le32(data);
	rx_len = get_unaligned_le32(data + sizeof(__le32));
	flags = get_unaligned_le32(data + 2 * sizeof(__le32));
	if (tx_len > APPLE_Z2_RAW_XFER_MAX_SIZE ||
	    rx_len > APPLE_Z2_RAW_XFER_MAX_SIZE ||
	    flags & ~APPLE_Z2_RAW_XFER_RX_BIT_REVERSE ||
	    size != 3 * sizeof(__le32) + tx_len ||
	    (!tx_len && !rx_len))
		return -EINVAL;

	xfer_len = max(tx_len, rx_len);
	if ((tx_len && tx_len != xfer_len) ||
	    (rx_len && rx_len != xfer_len))
		return -EINVAL;

	logical_tx = data + 3 * sizeof(__le32);
	if (tx_len)
		xfer.tx_buf = logical_tx;
	if (rx_len) {
		memset(z2->rx_buf, 0, rx_len);
		xfer.rx_buf = z2->rx_buf;
	}
	xfer.len = xfer_len;
	xfer.bits_per_word = 8;

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	if (flags & APPLE_Z2_RAW_XFER_RX_BIT_REVERSE) {
		for (i = 0; i < rx_len; i++)
			z2->rx_buf[i] = bitrev8(z2->rx_buf[i]);
	}

	dev_dbg(&z2->spidev->dev,
		"firmware raw xfer tx_len=%u rx_len=%u flags=%#x tx=%*ph rx=%*ph\n",
		 tx_len, rx_len, flags, min(tx_len, 16U), logical_tx,
		 min(rx_len, 16U), z2->rx_buf);

	return 0;
}

static bool apple_z2_wait_firmware_irq(struct apple_z2 *z2, u32 timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);

	if (!timeout)
		timeout = msecs_to_jiffies(1);

	if (!wait_for_completion_timeout(&z2->boot_irq, timeout)) {
		dev_warn(&z2->spidev->dev,
			 "firmware IRQ wait timed out after %u ms\n", timeout_ms);
		return false;
	}

	return true;
}

static int apple_z2_wait_ready_irq(struct apple_z2 *z2, u32 timeout_ms)
{
	if (!apple_z2_wait_firmware_irq(z2, timeout_ms))
		return -ETIMEDOUT;
	if (!z2->j172)
		return 0;

	if (!apple_z2_wait_firmware_irq(z2, timeout_ms))
		return -ETIMEDOUT;

	dev_dbg(&z2->spidev->dev, "firmware ready IRQs received\n");
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_HBPP_READY,
					 APPLE_Z2_STATE_FIRMWARE_READY);
}

static u32 apple_z2_fw_config_word(const u8 *data, unsigned int word)
{
	return get_unaligned_le32(data + word * sizeof(__le32));
}

static int apple_z2_apply_fw_config(struct apple_z2 *z2, const u8 *data,
				    size_t size)
{
	struct spi_device *spi = z2->spidev;
	u32 valid, min_dma, z2_delay, cs_delay, cpha, cpol;
	u32 mode = spi->mode;
	int error;

	if (size < APPLE_Z2_FW_CONFIG_WORDS * sizeof(__le32))
		return -EINVAL;

	valid = apple_z2_fw_config_word(data, 0);
	min_dma = apple_z2_fw_config_word(data, 1);
	z2_delay = apple_z2_fw_config_word(data, 2);
	cs_delay = apple_z2_fw_config_word(data, 3);
	cpha = apple_z2_fw_config_word(data, 4);
	cpol = apple_z2_fw_config_word(data, 5);
	if ((valid & APPLE_Z2_FW_CONFIG_MIN_DMA) && min_dma)
		z2->bpw16_min_len = min_dma;
	if (valid & APPLE_Z2_FW_CONFIG_Z2_DELAY)
		z2->z2_inter_packet_delay_us = z2_delay;
	if (valid & APPLE_Z2_FW_CONFIG_CS_DELAY)
		z2->z2_cs_delay_us = cs_delay;

	if (valid & APPLE_Z2_FW_CONFIG_CPHA) {
		if (cpha)
			mode |= SPI_CPHA;
		else
			mode &= ~SPI_CPHA;
	}
	if (valid & APPLE_Z2_FW_CONFIG_CPOL) {
		if (cpol)
			mode |= SPI_CPOL;
		else
			mode &= ~SPI_CPOL;
	}

	if (mode != spi->mode) {
		u32 old_mode = spi->mode;

		spi->mode = mode;
		error = spi_setup(spi);
		if (error) {
			spi->mode = old_mode;
			return error;
		}
	}

	apple_z2_apply_z2_delays(z2);
	dev_dbg(&spi->dev,
		"firmware SPI config: valid=%#x min-dma=%u z2-delay=%u cs-delay=%u mode=%#x\n",
		 valid, z2->bpw16_min_len, z2->z2_inter_packet_delay_us,
		 z2->z2_cs_delay_us, spi->mode);

	return 0;
}

static int apple_z2_upload_firmware(struct apple_z2 *z2)
{
	const struct apple_z2_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	size_t next_idx;
	size_t size;
	u32 load_cmd;
	u32 address;
	u32 timeout_ms;
	bool defer_irq_wait;
	bool init;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return dev_err_probe(&z2->spidev->dev, error,
				     "unable to load firmware\n");
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return dev_err_probe(&z2->spidev->dev, -EINVAL,
				     "invalid firmware header\n");

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		switch (load_cmd) {
		case LOAD_COMMAND_INIT_PAYLOAD:
		case LOAD_COMMAND_SEND_BLOB: {
			const u8 *blob = fw->data + fw_idx;

			if (size > fw->size - fw_idx)
				return -EINVAL;
			init = load_cmd == LOAD_COMMAND_INIT_PAYLOAD;
			next_idx = round_up(fw_idx + size, 4);
			defer_irq_wait = next_idx <= fw->size &&
				fw->size - next_idx >= 2 * sizeof(__le32) &&
				get_unaligned_le32(fw->data + next_idx) ==
					LOAD_COMMAND_WAIT_IRQ;
			error = apple_z2_send_firmware_blob(z2, blob, size, init,
							    defer_irq_wait);
			if (error)
				return error;
			fw_idx += size;
			break;
		}
		case LOAD_COMMAND_RAW_XFER:
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_send_firmware_raw_xfer(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case LOAD_COMMAND_SET_CONFIG:
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_apply_fw_config(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case LOAD_COMMAND_SEND_CALIBRATION: {
			const u8 *data __free(kfree) = NULL;

			address = size;
			data = apple_z2_build_cal_blob(z2, address, &size);
			if (IS_ERR(data))
				return PTR_ERR(data);
			if (data) {
				error = apple_z2_send_firmware_blob(z2, data, size, false, false);
				if (error)
					return error;
			}
			break;
		}
		case LOAD_COMMAND_WAIT_IRQ:
			if (size != sizeof(__le32) || size > fw->size - fw_idx)
				return -EINVAL;
			timeout_ms = get_unaligned_le32(fw->data + fw_idx);
			fw_idx += size;
			error = apple_z2_wait_ready_irq(z2, timeout_ms);
			if (error)
				return error;
			if (z2->j172) {
				disable_irq(z2->spidev->irq);
				error = apple_z2_read_device_info_locked(z2);
				enable_irq(z2->spidev->irq);
				if (error)
					return error;
			}
			break;
		default:
			return -EINVAL;
		}

		fw_idx = round_up(fw_idx, 4);
	}

	z2->booted = true;
	mutex_lock(&z2->io_lock);
	error = apple_z2_read_packet(z2);
	if (!error && z2->j172 && !z2->runtime_frame_valid)
		error = -EPROTO;
	if (!error && z2->j172)
		error = apple_z2_advance_protocol(z2, APPLE_Z2_STATE_DEVICE_INFO,
						  APPLE_Z2_STATE_RUNTIME);
	if (!error && z2->j172)
		error = apple_z2_j172_init_reads_locked(z2);
	mutex_unlock(&z2->io_lock);
	if (error)
		return error;

	if (!z2->j172)
		return 0;

	msleep(2260);
	mutex_lock(&z2->io_lock);
	error = apple_z2_j172_init_writes_locked(z2);
	mutex_unlock(&z2->io_lock);
	return error;
}

static int apple_z2_apply_initial_config(struct apple_z2 *z2)
{
	const struct apple_z2_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	u32 load_cmd;
	size_t size;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return error;
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return -EINVAL;

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		if (load_cmd == LOAD_COMMAND_SEND_CALIBRATION)
			continue;
		if (size > fw->size - fw_idx)
			return -EINVAL;
		if (load_cmd == LOAD_COMMAND_SET_CONFIG)
			return apple_z2_apply_fw_config(z2,
						fw->data + fw_idx, size);
		if (load_cmd != LOAD_COMMAND_INIT_PAYLOAD &&
		    load_cmd != LOAD_COMMAND_SEND_BLOB &&
		    load_cmd != LOAD_COMMAND_RAW_XFER &&
		    load_cmd != LOAD_COMMAND_WAIT_IRQ)
			return -EINVAL;

		fw_idx = round_up(fw_idx + size, 4);
	}

	return -EINVAL;
}

static int apple_z2_configure_spi(struct apple_z2 *z2)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.len = 4,
		.bits_per_word = 8,
	};

	memset(z2->tx_buf, 0, xfer.len);
	int error;

	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (!error)
		error = apple_z2_advance_protocol(z2, APPLE_Z2_STATE_POWERED,
						  APPLE_Z2_STATE_SPI_CONFIGURED);

	return error;
}

static int apple_z2_boot(struct apple_z2 *z2)
{
	bool irq_enabled = false;
	int error;

	z2->booted = false;
	apple_z2_reset_protocol(z2);
	error = apple_z2_platform_power_on(z2);
	if (error)
		return error;

	if (z2->j172) {
		error = apple_z2_apply_initial_config(z2);
		if (error)
			goto err_stop;
		error = apple_z2_configure_spi(z2);
		if (error)
			goto err_stop;
	}

	reinit_completion(&z2->boot_irq);
	enable_irq(z2->spidev->irq);
	irq_enabled = true;
	if (z2->j172) {
		error = apple_z2_j172_reset(z2);
	} else {
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		error = 0;
	}
	if (error)
		goto err_stop;

	if (!wait_for_completion_timeout(&z2->boot_irq,
					 msecs_to_jiffies(20))) {
		error = -ETIMEDOUT;
		goto err_stop;
	}
	if (z2->j172) {
		error = apple_z2_advance_protocol(z2,
						  APPLE_Z2_STATE_SPI_CONFIGURED,
						  APPLE_Z2_STATE_BOOT_IRQ);
		if (error)
			goto err_stop;
	}

	if (z2->j172) {
		disable_irq(z2->spidev->irq);
		z2->upload_irq_masked = true;
	}

	error = apple_z2_upload_firmware(z2);
	if (error)
		goto err_stop;

	return 0;

err_stop:
	z2->booted = false;
	if (z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		enable_irq(z2->spidev->irq);
	}
	if (irq_enabled)
		disable_irq(z2->spidev->irq);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	apple_z2_platform_power_off(z2);
	apple_z2_reset_protocol(z2);
	return error;
}

static int apple_z2_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_z2 *z2;
	unsigned int slots;
	int error;

	z2 = devm_kzalloc(dev, sizeof(*z2), GFP_KERNEL);
	if (!z2)
		return -ENOMEM;

	z2->tx_buf = devm_kzalloc(dev, APPLE_Z2_CMD_SIZE, GFP_KERNEL);
	if (!z2->tx_buf)
		return -ENOMEM;
	z2->rx_buf = devm_kzalloc(dev, APPLE_Z2_RX_BUF_SIZE, GFP_KERNEL);
	if (!z2->rx_buf)
		return -ENOMEM;

	z2->spidev = spi;
	z2->j172 = of_device_is_compatible(dev->of_node, "apple,j172-touchscreen");
	z2->bpw16_min_len = 1;
	device_property_read_u32(dev, "apple,z2-bpw16-min-len",
				 &z2->bpw16_min_len);
	if (!z2->bpw16_min_len)
		z2->bpw16_min_len = 1;
	z2->no_init_ack = device_property_read_bool(dev,
						    "apple,z2-no-init-ack");
	init_completion(&z2->boot_irq);
	mutex_init(&z2->io_lock);
	spi_set_drvdata(spi, z2);

	z2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(z2->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->reset_gpio),
				     "unable to get reset GPIO\n");

	z2->power_ana_gpio =
		devm_gpiod_get_optional(dev, "power-ana", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ana_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ana_gpio),
				     "unable to get analog power GPIO\n");
	z2->power_ldo_gpio =
		devm_gpiod_get_optional(dev, "power-ldo", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ldo_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ldo_gpio),
				     "unable to get LDO power GPIO\n");
	z2->display_sync_gpio =
		devm_gpiod_get_optional(dev, "display-sync", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync_gpio),
				     "unable to get display-sync GPIO\n");
	z2->display_sync1_gpio =
		devm_gpiod_get_optional(dev, "display-sync1", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync1_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync1_gpio),
				     "unable to get display-sync1 GPIO\n");

	z2->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(z2->clk))
		return dev_err_probe(dev, PTR_ERR(z2->clk),
				     "unable to get clock\n");

	error = devm_request_threaded_irq(dev, spi->irq, NULL, apple_z2_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  "apple-z2-irq", z2);
	if (error)
		return dev_err_probe(dev, error, "unable to request IRQ\n");

	error = device_property_read_string(dev, "firmware-name", &z2->fw_name);
	if (error)
		return dev_err_probe(dev, error, "unable to get firmware name\n");

	z2->input_dev = devm_input_allocate_device(dev);
	if (!z2->input_dev)
		return -ENOMEM;
	z2->input_dev->name = (char *)spi_get_device_id(spi)->driver_data;
	z2->input_dev->phys = "apple_z2";
	z2->input_dev->id.bustype = BUS_SPI;

	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	touchscreen_parse_properties(z2->input_dev, true, &z2->props);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_ORIENTATION, -32768, 32767,
			     0, 0);

	slots = z2->j172 ? APPLE_Z2_J172_MAX_CONTACTS : 256;
	error = input_mt_init_slots(z2->input_dev, slots, INPUT_MT_DIRECT);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to initialize multitouch slots\n");
	error = input_register_device(z2->input_dev);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to register input device\n");

	usleep_range(5000, 10000);
	return apple_z2_boot(z2);
}

static void apple_z2_stop(struct apple_z2 *z2)
{
	disable_irq(z2->spidev->irq);
	gpiod_direction_output(z2->reset_gpio, 1);
	z2->booted = false;
	apple_z2_platform_power_off(z2);
	apple_z2_reset_protocol(z2);
}

static void apple_z2_remove(struct spi_device *spi)
{
	apple_z2_stop(spi_get_drvdata(spi));
}

static int apple_z2_suspend(struct device *dev)
{
	apple_z2_stop(spi_get_drvdata(to_spi_device(dev)));

	return 0;
}

static int apple_z2_resume(struct device *dev)
{
	struct apple_z2 *z2 = spi_get_drvdata(to_spi_device(dev));

	return apple_z2_boot(z2);
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_z2_pm, apple_z2_suspend, apple_z2_resume);

static const struct of_device_id apple_z2_of_match[] = {
	{ .compatible = "apple,j293-touchbar" },
	{ .compatible = "apple,j493-touchbar" },
	{ .compatible = "apple,j172-touchscreen" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_z2_of_match);

static struct spi_device_id apple_z2_of_id[] = {
	{ .name = "j293-touchbar", .driver_data = (kernel_ulong_t)"MacBookPro17,1 Touch Bar" },
	{ .name = "j493-touchbar", .driver_data = (kernel_ulong_t)"Mac14,7 Touch Bar" },
	{ .name = "j172-touchscreen", .driver_data = (kernel_ulong_t)"iPad7,12 Touchscreen" },
	{}
};
MODULE_DEVICE_TABLE(spi, apple_z2_of_id);

static struct spi_driver apple_z2_driver = {
	.driver = {
		.name	= "apple-z2",
		.pm	= pm_sleep_ptr(&apple_z2_pm),
		.of_match_table = apple_z2_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = apple_z2_of_id,
	.probe    = apple_z2_probe,
	.remove   = apple_z2_remove,
};

module_spi_driver(apple_z2_driver);

MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/dfrmtfw-*.bin");
MODULE_DESCRIPTION("Apple Z2 touchscreens driver");

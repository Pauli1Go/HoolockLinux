// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SN2400 USB charger driver
 *
 * Copyright (c) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SN2400_REG_STATUS		0x07
#define SN2400_STATUS_VBUS		BIT(5)
#define SN2400_STATUS_CHARGING		BIT(7)

#define SN2400_REG_INPUT_LIMIT		0x09
#define SN2400_REG_CHARGE_LIMIT		0x0d
#define SN2400_REG_CONTROL		0x10
#define SN2400_CONTROL_SPREAD_SPECTRUM	GENMASK(1, 0)
#define SN2400_CONTROL_RESERVED		BIT(2)
#define SN2400_CONTROL_OFF		BIT(3)

#define SN2400_REG_ADC_CONTROL		0x11
#define SN2400_ADC_CHANNEL_MASK		GENMASK(3, 1)
#define SN2400_ADC_CHANNEL_VBUS		0
#define SN2400_ADC_CHANNEL_IBUS		BIT(1)
#define SN2400_ADC_START			BIT(4)
#define SN2400_REG_ADC_DATA		0x12

#define SN2400_INPUT_MIN_UA		90000
#define SN2400_INPUT_START_UA		300000
#define SN2400_INPUT_STEP_UA		25000
#define SN2400_INPUT_HW_MAX_UA		2000000
#define SN2400_INPUT_REG_STEP_UA	10000

#define SN2400_MAX_VBUS_DROOP_UV	400000
#define SN2400_MIN_VBUS_UV		4000000
#define SN2400_FAST_POLL_MS		300
#define SN2400_STABLE_POLL_MS		2000
#define SN2400_VBUS_DETACH_MS		5000
#define SN2400_ADC_POLL_US		5000
#define SN2400_ADC_TIMEOUT_US		100000

enum sn2400_foldback_state {
	SN2400_FOLDBACK_IDLE,
	SN2400_FOLDBACK_NOLOAD,
	SN2400_FOLDBACK_PROBE,
	SN2400_FOLDBACK_DEBOUNCE,
	SN2400_FOLDBACK_SETTLE,
};

struct sn2400_charger {
	struct device *dev;
	struct regmap *regmap;
	struct mux_state *hdq_mux;
	struct power_supply *psy;
	struct delayed_work work;
	/* Protects cached power-supply data and the stopped flag. */
	struct mutex lock;
	enum sn2400_foldback_state state;
	unsigned int max_input_ua;
	unsigned int session_max_input_ua;
	unsigned int max_charge_ua;
	unsigned int target_limit_ua;
	unsigned int input_limit_ua;
	unsigned int vbus_noload_uv;
	unsigned int status;
	int status_error;
	int vbus_uv;
	int ibus_ua;
	unsigned long vbus_absent_since;
	bool vbus_absent;
	bool stopped;
};

static int sn2400_read_adc(struct sn2400_charger *charger,
			   unsigned int channel, int *result)
{
	unsigned int value;
	int ret;

	ret = regmap_write(charger->regmap, SN2400_REG_ADC_CONTROL,
			   (channel & SN2400_ADC_CHANNEL_MASK) |
			   SN2400_ADC_START);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(charger->regmap,
				       SN2400_REG_ADC_CONTROL, value,
				       !(value & SN2400_ADC_START),
				       SN2400_ADC_POLL_US,
				       SN2400_ADC_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read(charger->regmap, SN2400_REG_ADC_DATA, &value);
	if (ret)
		return ret;

	switch (channel) {
	case SN2400_ADC_CHANNEL_VBUS:
		*result = value * 25000;
		break;
	case SN2400_ADC_CHANNEL_IBUS:
		*result = value * 10000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sn2400_read_vbus(struct sn2400_charger *charger, int *vbus_uv)
{
	int ret;

	ret = sn2400_read_adc(charger, SN2400_ADC_CHANNEL_VBUS, vbus_uv);
	if (ret)
		return ret;

	return *vbus_uv >= SN2400_MIN_VBUS_UV ? 0 : -ERANGE;
}

static unsigned int sn2400_encode_input_limit(unsigned int input_ua)
{
	return (input_ua - SN2400_INPUT_MIN_UA) /
		SN2400_INPUT_REG_STEP_UA;
}

static unsigned int sn2400_decode_input_limit(unsigned int value)
{
	return SN2400_INPUT_MIN_UA + value * SN2400_INPUT_REG_STEP_UA;
}

static int sn2400_set_input_limit(struct sn2400_charger *charger,
				  unsigned int input_ua,
				  unsigned int *applied_ua)
{
	unsigned int input_reg = 0;
	unsigned int control;
	int deselect_ret;
	int ret;

	if (input_ua) {
		input_ua = clamp(input_ua, SN2400_INPUT_MIN_UA,
				 SN2400_INPUT_HW_MAX_UA);
		input_reg = sn2400_encode_input_limit(input_ua);
		input_ua = sn2400_decode_input_limit(input_reg);
	}

	if (!input_ua) {
		control = SN2400_CONTROL_RESERVED | SN2400_CONTROL_OFF;
		ret = regmap_write(charger->regmap, SN2400_REG_CONTROL,
				   control);
		if (!ret)
			ret = regmap_write(charger->regmap,
					   SN2400_REG_INPUT_LIMIT, 0);
		if (!ret)
			*applied_ua = 0;

		return ret;
	}

	/*
	 * Give the charger the HDQ bus before enabling its input path. The
	 * control-before-limit order is required by the D111 firmware sequence.
	 */
	ret = mux_state_select(charger->hdq_mux);
	if (ret)
		return ret;

	control = SN2400_CONTROL_SPREAD_SPECTRUM |
		  SN2400_CONTROL_RESERVED;
	ret = regmap_write(charger->regmap, SN2400_REG_CONTROL, control);
	if (ret)
		goto disable;

	ret = regmap_write(charger->regmap, SN2400_REG_INPUT_LIMIT, input_reg);
	if (ret)
		goto disable;

	goto out_deselect;

disable:
	regmap_write(charger->regmap, SN2400_REG_CONTROL,
		     SN2400_CONTROL_RESERVED | SN2400_CONTROL_OFF);
	input_ua = 0;

out_deselect:
	deselect_ret = mux_state_deselect(charger->hdq_mux);
	if (!ret && deselect_ret) {
		regmap_write(charger->regmap, SN2400_REG_CONTROL,
			     SN2400_CONTROL_RESERVED | SN2400_CONTROL_OFF);
		input_ua = 0;
		ret = deselect_ret;
	}

	if (!ret)
		*applied_ua = input_ua;

	return ret;
}

static int sn2400_disable_input(struct sn2400_charger *charger)
{
	unsigned int applied_ua;

	return sn2400_set_input_limit(charger, 0, &applied_ua);
}

static void sn2400_cache(struct sn2400_charger *charger, int error,
			 unsigned int status, int vbus_uv, int ibus_ua,
			 unsigned int input_limit_ua)
{
	bool changed;

	mutex_lock(&charger->lock);
	changed = charger->status_error != error ||
		  (!error && (charger->status != status ||
			      charger->vbus_uv != vbus_uv ||
			      charger->ibus_ua != ibus_ua ||
			      charger->input_limit_ua != input_limit_ua));
	charger->status_error = error;
	if (!error) {
		charger->status = status;
		charger->vbus_uv = vbus_uv;
		charger->ibus_ua = ibus_ua;
		charger->input_limit_ua = input_limit_ua;
	}
	mutex_unlock(&charger->lock);

	if (changed && charger->psy)
		power_supply_changed(charger->psy);
}

static void sn2400_schedule(struct sn2400_charger *charger,
			    unsigned int delay_ms)
{
	mutex_lock(&charger->lock);
	if (!charger->stopped)
		schedule_delayed_work(&charger->work,
				      msecs_to_jiffies(delay_ms));
	mutex_unlock(&charger->lock);
}

static bool sn2400_vbus_drooped(struct sn2400_charger *charger, int vbus_uv)
{
	return charger->vbus_noload_uv > vbus_uv &&
		charger->vbus_noload_uv - vbus_uv > SN2400_MAX_VBUS_DROOP_UV;
}

static int sn2400_raise_input_limit(struct sn2400_charger *charger,
				    unsigned int *input_limit_ua)
{
	unsigned int requested;
	int ret;

	requested = min(charger->target_limit_ua + SN2400_INPUT_STEP_UA,
			charger->session_max_input_ua);
	ret = sn2400_set_input_limit(charger, requested, input_limit_ua);
	if (!ret)
		charger->target_limit_ua = requested;

	return ret;
}

static int sn2400_lower_input_limit(struct sn2400_charger *charger,
				    unsigned int *input_limit_ua)
{
	unsigned int requested;
	int ret;

	if (charger->target_limit_ua <= SN2400_INPUT_MIN_UA)
		requested = 0;
	else
		requested = max(charger->target_limit_ua - SN2400_INPUT_STEP_UA,
				SN2400_INPUT_MIN_UA);

	ret = sn2400_set_input_limit(charger, requested, input_limit_ua);
	if (!ret)
		charger->target_limit_ua = requested;

	return ret;
}

static void sn2400_charger_work(struct work_struct *work)
{
	struct sn2400_charger *charger =
		container_of(to_delayed_work(work), struct sn2400_charger, work);
	unsigned int input_limit_ua = charger->input_limit_ua;
	unsigned int next_poll_ms = SN2400_STABLE_POLL_MS;
	unsigned int status;
	int ibus_ua = 0;
	int vbus_uv = 0;
	int ret;

	ret = regmap_read(charger->regmap, SN2400_REG_STATUS, &status);
	if (ret)
		goto fail_safe;

	if (!(status & SN2400_STATUS_VBUS)) {
		if (input_limit_ua) {
			ret = sn2400_disable_input(charger);
			if (ret)
				goto fail_safe;
			input_limit_ua = 0;
		}
		charger->state = SN2400_FOLDBACK_IDLE;
		charger->target_limit_ua = 0;
		if (!charger->vbus_absent) {
			charger->vbus_absent = true;
			charger->vbus_absent_since = jiffies;
		} else if (time_is_before_jiffies(charger->vbus_absent_since +
				msecs_to_jiffies(SN2400_VBUS_DETACH_MS))) {
			/* A sustained detach starts a new input-current session. */
			charger->session_max_input_ua = charger->max_input_ua;
		}
		sn2400_cache(charger, 0, status, 0, 0, 0);
		goto out_schedule;
	}
	if (charger->vbus_absent &&
	    time_is_before_jiffies(charger->vbus_absent_since +
				   msecs_to_jiffies(SN2400_VBUS_DETACH_MS)))
		charger->session_max_input_ua = charger->max_input_ua;
	charger->vbus_absent = false;

	switch (charger->state) {
	case SN2400_FOLDBACK_IDLE:
		ret = sn2400_disable_input(charger);
		if (ret)
			goto fail_safe;
		input_limit_ua = 0;
		charger->state = SN2400_FOLDBACK_NOLOAD;
		next_poll_ms = SN2400_FAST_POLL_MS;
		break;
	case SN2400_FOLDBACK_NOLOAD:
		ret = sn2400_read_vbus(charger, &vbus_uv);
		if (ret)
			goto fail_safe;
		charger->vbus_noload_uv = vbus_uv;
		charger->target_limit_ua = min(charger->session_max_input_ua,
					       SN2400_INPUT_START_UA);
		ret = sn2400_set_input_limit(charger, charger->target_limit_ua,
					     &input_limit_ua);
		if (ret)
			goto fail_safe;
		charger->state = SN2400_FOLDBACK_PROBE;
		next_poll_ms = SN2400_FAST_POLL_MS;
		break;
	case SN2400_FOLDBACK_PROBE:
		ret = sn2400_read_vbus(charger, &vbus_uv);
		if (ret)
			goto fail_safe;
		ret = sn2400_read_adc(charger, SN2400_ADC_CHANNEL_IBUS,
				      &ibus_ua);
		if (ret)
			goto fail_safe;
		if (sn2400_vbus_drooped(charger, vbus_uv)) {
			charger->state = SN2400_FOLDBACK_DEBOUNCE;
		} else if (input_limit_ua >= charger->session_max_input_ua) {
			charger->state = SN2400_FOLDBACK_SETTLE;
		} else {
			ret = sn2400_raise_input_limit(charger, &input_limit_ua);
			if (ret)
				goto fail_safe;
		}
		next_poll_ms = SN2400_FAST_POLL_MS;
		break;
	case SN2400_FOLDBACK_DEBOUNCE:
		ret = sn2400_read_vbus(charger, &vbus_uv);
		if (ret)
			goto fail_safe;
		ret = sn2400_read_adc(charger, SN2400_ADC_CHANNEL_IBUS,
				      &ibus_ua);
		if (ret)
			goto fail_safe;
		if (sn2400_vbus_drooped(charger, vbus_uv)) {
			ret = sn2400_lower_input_limit(charger, &input_limit_ua);
			if (ret)
				goto fail_safe;
			charger->state = input_limit_ua ?
				SN2400_FOLDBACK_SETTLE : SN2400_FOLDBACK_NOLOAD;
		} else {
			charger->state = SN2400_FOLDBACK_PROBE;
		}
		next_poll_ms = SN2400_FAST_POLL_MS;
		break;
	case SN2400_FOLDBACK_SETTLE:
		ret = sn2400_read_vbus(charger, &vbus_uv);
		if (ret)
			goto fail_safe;
		ret = sn2400_read_adc(charger, SN2400_ADC_CHANNEL_IBUS,
				      &ibus_ua);
		if (ret)
			goto fail_safe;
		if (sn2400_vbus_drooped(charger, vbus_uv)) {
			ret = sn2400_lower_input_limit(charger, &input_limit_ua);
			if (ret)
				goto fail_safe;
			if (!input_limit_ua)
				charger->state = SN2400_FOLDBACK_NOLOAD;
		} else if (charger->target_limit_ua) {
			charger->session_max_input_ua =
				min(charger->session_max_input_ua,
				    charger->target_limit_ua);
		}
		break;
	}

	sn2400_cache(charger, 0, status, vbus_uv, ibus_ua,
		     input_limit_ua);
	goto out_schedule;

fail_safe:
	if (sn2400_disable_input(charger))
		dev_err_ratelimited(charger->dev,
				    "failed to disable USB input after error\n");
	charger->state = SN2400_FOLDBACK_IDLE;
	charger->target_limit_ua = 0;
	sn2400_cache(charger, ret, 0, 0, 0, 0);

out_schedule:
	sn2400_schedule(charger, next_poll_ms);
}

static int sn2400_charger_get_property(struct power_supply *psy,
				       enum power_supply_property property,
				       union power_supply_propval *val)
{
	struct sn2400_charger *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&charger->lock);
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = charger->status_error;
		if (ret)
			break;
		if (!(charger->status & SN2400_STATUS_VBUS))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (charger->status & SN2400_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = charger->status_error;
		if (ret)
			break;
		val->intval = !!(charger->status & SN2400_STATUS_VBUS);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = charger->status_error;
		if (ret)
			break;
		val->intval = charger->vbus_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = charger->status_error;
		if (ret)
			break;
		val->intval = charger->ibus_ua;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = charger->status_error;
		if (ret)
			break;
		val->intval = charger->input_limit_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = charger->max_charge_ua;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&charger->lock);
	return ret;
}

static const enum power_supply_property sn2400_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc sn2400_charger_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sn2400_charger_properties,
	.num_properties = ARRAY_SIZE(sn2400_charger_properties),
	.get_property = sn2400_charger_get_property,
};

static int sn2400_charger_stop(struct sn2400_charger *charger)
{
	bool changed;
	int ret;

	mutex_lock(&charger->lock);
	charger->stopped = true;
	mutex_unlock(&charger->lock);
	cancel_delayed_work_sync(&charger->work);

	ret = sn2400_disable_input(charger);
	mutex_lock(&charger->lock);
	changed = charger->status_error != ret ||
		  (!ret && charger->input_limit_ua);
	charger->status_error = ret;
	if (!ret)
		charger->input_limit_ua = 0;
	mutex_unlock(&charger->lock);

	if (changed && charger->psy)
		power_supply_changed(charger->psy);

	return ret;
}

static void sn2400_charger_start(struct sn2400_charger *charger,
				 int status_error)
{
	mutex_lock(&charger->lock);
	charger->state = SN2400_FOLDBACK_IDLE;
	charger->target_limit_ua = 0;
	charger->status_error = status_error;
	charger->stopped = false;
	mutex_unlock(&charger->lock);

	if (charger->psy)
		power_supply_changed(charger->psy);
	schedule_delayed_work(&charger->work,
			      msecs_to_jiffies(SN2400_FAST_POLL_MS));
}

static void sn2400_charger_cache_suspend(struct sn2400_charger *charger,
					 unsigned int status,
					 unsigned int input_limit_ua)
{
	bool changed;

	mutex_lock(&charger->lock);
	changed = charger->status_error || charger->status != status ||
		  charger->input_limit_ua != input_limit_ua ||
		  (!(status & SN2400_STATUS_VBUS) &&
		   (charger->vbus_uv || charger->ibus_ua));
	charger->status_error = 0;
	charger->status = status;
	charger->input_limit_ua = input_limit_ua;
	charger->state = SN2400_FOLDBACK_IDLE;
	charger->target_limit_ua = input_limit_ua;
	if (!(status & SN2400_STATUS_VBUS)) {
		charger->vbus_uv = 0;
		charger->ibus_ua = 0;
	}
	mutex_unlock(&charger->lock);

	if (changed && charger->psy)
		power_supply_changed(charger->psy);
}

static int sn2400_charger_probe(struct platform_device *pdev)
{
	struct power_supply_battery_info *battery_info;
	struct power_supply_config psy_config = {};
	struct sn2400_charger *charger;
	u32 input_limit_ua;
	unsigned int status;
	int ret;

	charger = devm_kzalloc(&pdev->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->dev = &pdev->dev;
	charger->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!charger->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "failed to get parent regmap\n");

	charger->hdq_mux = devm_mux_state_get(&pdev->dev, "hdq");
	if (IS_ERR(charger->hdq_mux))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->hdq_mux),
				     "failed to get charger HDQ mux state\n");

	ret = device_property_read_u32(&pdev->dev,
				       "input-current-limit-microamp",
				       &input_limit_ua);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing input current limit\n");
	if (input_limit_ua < SN2400_INPUT_START_UA ||
	    input_limit_ua > SN2400_INPUT_HW_MAX_UA)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid input current limit %u uA\n",
				     input_limit_ua);
	input_limit_ua =
		sn2400_decode_input_limit(sn2400_encode_input_limit(input_limit_ua));
	charger->max_input_ua = input_limit_ua;
	charger->session_max_input_ua = input_limit_ua;

	ret = devm_mutex_init(&pdev->dev, &charger->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&charger->work, sn2400_charger_work);
	charger->state = SN2400_FOLDBACK_IDLE;
	charger->status_error = -EAGAIN;
	charger->stopped = true;
	platform_set_drvdata(pdev, charger);

	ret = regmap_read(charger->regmap, SN2400_REG_STATUS, &status);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to read initial charger status\n");
	charger->status = status;
	charger->status_error = 0;

	psy_config.drv_data = charger;
	psy_config.fwnode = dev_fwnode(&pdev->dev);
	psy_config.no_wakeup_source = true;
	charger->psy = devm_power_supply_register(&pdev->dev,
						  &sn2400_charger_desc,
						 &psy_config);
	if (IS_ERR(charger->psy))
		return PTR_ERR(charger->psy);

	ret = power_supply_get_battery_info(charger->psy, &battery_info);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to get monitored battery limits\n");
	charger->max_charge_ua = battery_info->constant_charge_current_max_ua;
	power_supply_put_battery_info(charger->psy, battery_info);
	if (charger->max_charge_ua < SN2400_INPUT_REG_STEP_UA ||
	    charger->max_charge_ua > U8_MAX * SN2400_INPUT_REG_STEP_UA)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid battery charge current limit %u uA\n",
				     charger->max_charge_ua);
	charger->max_charge_ua -=
		charger->max_charge_ua % SN2400_INPUT_REG_STEP_UA;

	ret = regmap_write(charger->regmap, SN2400_REG_CHARGE_LIMIT,
			   charger->max_charge_ua /
			    SN2400_INPUT_REG_STEP_UA);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to program battery charge current limit\n");
	power_supply_changed(charger->psy);

	mutex_lock(&charger->lock);
	charger->stopped = false;
	mutex_unlock(&charger->lock);
	schedule_delayed_work(&charger->work,
			      msecs_to_jiffies(SN2400_FAST_POLL_MS));

	return 0;
}

static void sn2400_charger_remove(struct platform_device *pdev)
{
	struct sn2400_charger *charger = platform_get_drvdata(pdev);

	if (sn2400_charger_stop(charger))
		dev_warn(&pdev->dev, "failed to disable USB input on remove\n");
}

static void sn2400_charger_shutdown(struct platform_device *pdev)
{
	struct sn2400_charger *charger = platform_get_drvdata(pdev);

	if (sn2400_charger_stop(charger))
		dev_warn(&pdev->dev, "failed to disable USB input on shutdown\n");
}

static int sn2400_charger_suspend(struct device *dev)
{
	struct sn2400_charger *charger = dev_get_drvdata(dev);
	unsigned int input_limit_ua;
	unsigned int status;
	int ret;

	mutex_lock(&charger->lock);
	charger->stopped = true;
	mutex_unlock(&charger->lock);
	cancel_delayed_work_sync(&charger->work);

	ret = regmap_read(charger->regmap, SN2400_REG_STATUS, &status);
	if (ret)
		goto restart;

	if (status & SN2400_STATUS_VBUS) {
		input_limit_ua = min(charger->session_max_input_ua,
				     SN2400_INPUT_START_UA);
		ret = sn2400_set_input_limit(charger, input_limit_ua,
					     &input_limit_ua);
	} else {
		ret = sn2400_set_input_limit(charger, 0, &input_limit_ua);
	}
	if (ret)
		goto restart;

	sn2400_charger_cache_suspend(charger, status, input_limit_ua);
	return 0;

restart:
	sn2400_charger_start(charger, ret);

	return ret;
}

static int sn2400_charger_resume(struct device *dev)
{
	struct sn2400_charger *charger = dev_get_drvdata(dev);

	sn2400_charger_start(charger, -EAGAIN);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sn2400_charger_pm_ops,
				 sn2400_charger_suspend,
				 sn2400_charger_resume);

static const struct of_device_id sn2400_charger_of_match[] = {
	{ .compatible = "apple,sn2400-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, sn2400_charger_of_match);

static struct platform_driver sn2400_charger_driver = {
	.probe = sn2400_charger_probe,
	.remove = sn2400_charger_remove,
	.shutdown = sn2400_charger_shutdown,
	.driver = {
		.name = "apple-sn2400-charger",
		.of_match_table = sn2400_charger_of_match,
		.pm = pm_sleep_ptr(&sn2400_charger_pm_ops),
	},
};
module_platform_driver(sn2400_charger_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple SN2400 USB charger driver");
MODULE_LICENSE("GPL");

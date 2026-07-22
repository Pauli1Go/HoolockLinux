// SPDX-License-Identifier: GPL-2.0-only
/*
 * Charging status and guarded input-current policy for the Apple D2365 PMIC
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define D2365_STATUS_CHARGE	0x0a
#define D2365_STATUS_CHARGE_STATE	GENMASK(1, 0)
#define D2365_STATUS_CHARGE_END	BIT(2)
#define D2365_STATUS_CHARGE_TIMEOUT	BIT(4)
#define D2365_CONTROL_INPUT	0x00
#define D2365_CONTROL_INPUT_LIMIT	0x03
#define D2365_CONTROL_MAX_CHARGE	0x0c
#define D2365_CONTROL_MAX_CHARGE_MASK	GENMASK(5, 0)
#define D2365_POLICY_CONTROL	0x00
#define D2365_POLICY_MIN_TEMP	100
#define D2365_POLICY_MAX_TEMP	400
#define D2365_POLICY_MIN_VOLTAGE_UV	3000000
#define D2365_POLICY_MAX_VOLTAGE_UV	4350000
#define D2365_POLICY_INTERVAL_MS	5000
#define D2365_RESTORE_RETRY_MS	100
#define D2365_STOP_RESTORE_ATTEMPTS	3

struct apple_d2365_config {
	u8 handoff_input_limit;
	u8 charging_input_limit;
	u8 max_charge;
};

static const struct apple_d2365_config apple_d2365_j172_config = {
	.handoff_input_limit = 0x02,
	.charging_input_limit = 0x6c,
	.max_charge = 0x34,
};

struct apple_d2365_battery_snapshot {
	int present;
	int temperature;
	int voltage_uv;
};

struct apple_d2365_charger {
	/* Protects cached properties while probe completes. */
	struct mutex lock;
	/* Serializes policy validation, writes and restore. */
	struct mutex policy_lock;
	struct delayed_work charge_policy_work;
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	const struct apple_d2365_config *config;
	u32 status_base;
	u32 control_base;
	unsigned int raw_status;
	int status_error;
	unsigned int raw_input_limit;
	int input_limit_error;
	u8 policy_old_input_limit;
	bool policy_old_input_valid;
	bool policy_enabled;
	bool restore_pending;
	bool shutting_down;
};

static int apple_d2365_charger_online(struct power_supply *psy,
				      union power_supply_propval *val);

static int apple_d2365_get_bank(struct device *dev, const char *name,
				u32 min_size, u32 *base)
{
	u64 address;
	u64 size;
	int index;
	int ret;

	index = of_property_match_string(dev->of_node, "reg-names", name);
	if (index < 0)
		return dev_err_probe(dev, index,
				     "missing %s register range\n", name);

	ret = of_property_read_reg(dev->of_node, index, &address, &size);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read %s register range\n", name);

	if (size < min_size || address + min_size - 1 > U32_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "invalid %s register range\n", name);

	*base = address;
	return 0;
}

static int apple_d2365_battery_status(union power_supply_propval *val)
{
	struct power_supply *battery;
	int ret;

	battery = power_supply_get_by_name("battery");
	if (!battery)
		return -ENODEV;

	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_STATUS, val);
	power_supply_put(battery);

	return ret;
}

static void apple_d2365_cache_status(struct apple_d2365_charger *charger,
				     int ret, unsigned int status)
{
	bool changed;

	mutex_lock(&charger->lock);
	changed = charger->status_error != ret ||
		  (!ret && charger->raw_status != status);
	charger->status_error = ret;
	if (!ret)
		charger->raw_status = status;
	mutex_unlock(&charger->lock);

	if (changed && charger->psy)
		power_supply_changed(charger->psy);
}

static void
apple_d2365_cache_input_limit(struct apple_d2365_charger *charger, int ret,
			      unsigned int input_limit)
{
	bool changed;

	mutex_lock(&charger->lock);
	changed = charger->input_limit_error != ret ||
		  (!ret && charger->raw_input_limit != input_limit);
	charger->input_limit_error = ret;
	if (!ret)
		charger->raw_input_limit = input_limit;
	mutex_unlock(&charger->lock);

	if (changed && charger->psy)
		power_supply_changed(charger->psy);
}

static int
apple_d2365_battery_snapshot(struct apple_d2365_battery_snapshot *snapshot)
{
	union power_supply_propval val;
	struct power_supply *battery;
	int ret;

	battery = power_supply_get_by_name("battery");
	if (!battery)
		return -ENODEV;

	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret)
		goto out_put;
	snapshot->present = val.intval;

	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		goto out_put;
	snapshot->temperature = val.intval;

	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&val);
	if (ret)
		goto out_put;
	snapshot->voltage_uv = val.intval;

out_put:
	power_supply_put(battery);
	return ret;
}

static int
apple_d2365_restore_input_limit(struct apple_d2365_charger *charger,
				u8 old_input_limit)
{
	unsigned int readback;
	int ret = -EIO;
	int attempt;

	for (attempt = 1; attempt <= 3; attempt++) {
		ret = regmap_write(charger->regmap,
				   charger->control_base + D2365_CONTROL_INPUT_LIMIT,
				   old_input_limit);
		if (!ret) {
			ret = regmap_read(charger->regmap,
					  charger->control_base +
					  D2365_CONTROL_INPUT_LIMIT, &readback);
			if (!ret && readback == old_input_limit) {
				apple_d2365_cache_input_limit(charger, 0,
							      old_input_limit);
				return 0;
			}
			if (!ret)
				ret = -EIO;
		}

		if (attempt < 3)
			msleep(20);
	}
	apple_d2365_cache_input_limit(charger, ret, 0);

	return ret;
}

static bool
apple_d2365_battery_is_safe(const struct apple_d2365_battery_snapshot *snapshot)
{
	return snapshot->present == 1 &&
		snapshot->temperature >= D2365_POLICY_MIN_TEMP &&
		snapshot->temperature <= D2365_POLICY_MAX_TEMP &&
		snapshot->voltage_uv >= D2365_POLICY_MIN_VOLTAGE_UV &&
		snapshot->voltage_uv <= D2365_POLICY_MAX_VOLTAGE_UV;
}

static int
apple_d2365_policy_restore_locked(struct apple_d2365_charger *charger,
				  const char *reason)
{
	unsigned int control;
	int ret;

	if (!charger->restore_pending)
		return 0;
	if (!charger->policy_old_input_valid)
		return -EINVAL;

	charger->policy_enabled = false;

	ret = apple_d2365_restore_input_limit(charger,
					      charger->policy_old_input_limit);
	if (ret) {
		dev_err(charger->dev,
			"failed to restore input limit (%s): %d\n", reason, ret);
		return ret;
	}
	charger->restore_pending = false;
	charger->policy_old_input_valid = false;
	power_supply_changed(charger->psy);

	ret = regmap_read(charger->regmap,
			  charger->control_base + D2365_CONTROL_INPUT, &control);
	if (ret || control != D2365_POLICY_CONTROL) {
		dev_err(charger->dev,
			"D2365 charging policy control integrity failed (%s): ret=%d control=0x%02x\n",
			reason, ret, ret ? 0 : control);
		return ret ?: -EIO;
	}

	return 0;
}

static int apple_d2365_policy_read_controls(struct apple_d2365_charger *charger,
					    unsigned int *control,
					    unsigned int *input_limit,
					    unsigned int *max_charge,
					    unsigned int *status)
{
	int ret;

	ret = regmap_read(charger->regmap,
			  charger->status_base + D2365_STATUS_CHARGE, status);
	apple_d2365_cache_status(charger, ret, ret ? 0 : *status);
	if (ret)
		return ret;

	ret = regmap_read(charger->regmap,
			  charger->control_base + D2365_CONTROL_INPUT, control);
	if (ret)
		return ret;

	ret = regmap_read(charger->regmap,
			  charger->control_base + D2365_CONTROL_INPUT_LIMIT,
			  input_limit);
	apple_d2365_cache_input_limit(charger, ret, ret ? 0 : *input_limit);
	if (ret)
		return ret;

	return regmap_read(charger->regmap,
			   charger->control_base + D2365_CONTROL_MAX_CHARGE,
			   max_charge);
}

static void apple_d2365_charge_policy_work(struct work_struct *work)
{
	struct apple_d2365_charger *charger;
	struct apple_d2365_battery_snapshot battery;
	union power_supply_propval online;
	unsigned int control;
	unsigned int input_limit;
	unsigned int max_charge;
	unsigned int readback;
	unsigned int status;
	const char *reject_reason = NULL;
	int ret;

	charger = container_of(to_delayed_work(work),
			       struct apple_d2365_charger, charge_policy_work);

	mutex_lock(&charger->policy_lock);
	if (charger->shutting_down)
		goto out_unlock;
	if (charger->restore_pending && !charger->policy_enabled) {
		apple_d2365_policy_restore_locked(charger, "retry");
		goto out_reschedule;
	}

	ret = apple_d2365_charger_online(charger->psy, &online);
	if (ret || online.intval != 1) {
		reject_reason = "supplier offline";
		goto out_stop;
	}

	ret = apple_d2365_battery_snapshot(&battery);
	if (ret || !apple_d2365_battery_is_safe(&battery)) {
		reject_reason = "battery guard";
		goto out_stop;
	}

	ret = apple_d2365_policy_read_controls(charger, &control, &input_limit,
					       &max_charge, &status);
	if (ret) {
		reject_reason = "control read";
		goto out_stop;
	}

	/*
	 * Keep the input budget in place after charge termination so the system
	 * can remain externally powered while the PMIC controls battery current.
	 */
	if (status & D2365_STATUS_CHARGE_TIMEOUT) {
		reject_reason = "charge timeout";
		goto out_stop;
	}

	if (control != D2365_POLICY_CONTROL ||
	    (max_charge & D2365_CONTROL_MAX_CHARGE_MASK) !=
			    charger->config->max_charge) {
		reject_reason = "control integrity";
		goto out_stop;
	}

	if (charger->policy_enabled) {
		if (input_limit != charger->config->charging_input_limit) {
			reject_reason = "input-limit integrity";
			goto out_stop;
		}
		goto out_reschedule;
	}

	if (input_limit != charger->config->handoff_input_limit) {
		reject_reason = "unexpected handoff input-limit";
		goto out_stop;
	}

	charger->policy_old_input_limit = input_limit;
	charger->policy_old_input_valid = true;
	charger->restore_pending = true;
	ret = regmap_write(charger->regmap,
			   charger->control_base + D2365_CONTROL_INPUT_LIMIT,
			   charger->config->charging_input_limit);
	if (ret)
		goto out_write_error;

	ret = regmap_read(charger->regmap,
			  charger->control_base + D2365_CONTROL_INPUT_LIMIT,
			  &readback);
	apple_d2365_cache_input_limit(charger, ret, ret ? 0 : readback);
	if (ret || readback != charger->config->charging_input_limit) {
		if (!ret)
			ret = -EIO;
		goto out_write_error;
	}

	charger->policy_enabled = true;
	power_supply_changed(charger->psy);

out_reschedule:
	if (!charger->shutting_down)
		mod_delayed_work(system_power_efficient_wq,
				 &charger->charge_policy_work,
				 msecs_to_jiffies(D2365_POLICY_INTERVAL_MS));
	goto out_unlock;

out_write_error:
	dev_err(charger->dev, "D2365 charging policy enable failed: %d\n", ret);
	apple_d2365_policy_restore_locked(charger, "enable error");
	goto out_reschedule;

out_stop:
	if (charger->policy_enabled || charger->restore_pending)
		apple_d2365_policy_restore_locked(charger, reject_reason);
	goto out_reschedule;

out_unlock:
	mutex_unlock(&charger->policy_lock);
}

static int apple_d2365_charger_online(struct power_supply *psy,
				      union power_supply_propval *val)
{
	return power_supply_get_property_from_supplier(psy,
						       POWER_SUPPLY_PROP_ONLINE,
						       val);
}

static int apple_d2365_charger_status(struct power_supply *psy,
				      union power_supply_propval *val)
{
	struct apple_d2365_charger *charger = power_supply_get_drvdata(psy);
	union power_supply_propval battery_status;
	union power_supply_propval online;
	unsigned int charge_state;
	unsigned int raw_status;
	int battery_ret;
	int ret;

	mutex_lock(&charger->lock);
	ret = charger->status_error;
	raw_status = charger->raw_status;
	mutex_unlock(&charger->lock);

	if (ret)
		return ret;

	charge_state = raw_status & D2365_STATUS_CHARGE_STATE;

	ret = apple_d2365_charger_online(psy, &online);
	if (!ret && !online.intval) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	if (raw_status & D2365_STATUS_CHARGE_TIMEOUT) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	battery_ret = apple_d2365_battery_status(&battery_status);
	if (!ret && online.intval == 1 && !battery_ret &&
	    battery_status.intval == POWER_SUPPLY_STATUS_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (raw_status & D2365_STATUS_CHARGE_END) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	/*
	 * Apple uses the non-zero D2365 field as IsCharging, but live data shows
	 * it can remain set while Bellatrix reports USB offline. Require both
	 * the PMIC state and positive external-power evidence before exporting
	 * Charging. A missing supplier remains conservative.
	 */
	if (!ret)
		val->intval = charge_state ? POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		val->intval = charge_state ? POWER_SUPPLY_STATUS_UNKNOWN :
			POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int
apple_d2365_charger_input_limit(struct power_supply *psy,
				union power_supply_propval *val)
{
	struct apple_d2365_charger *charger = power_supply_get_drvdata(psy);
	unsigned int input_limit;
	int ret;

	mutex_lock(&charger->lock);
	ret = charger->input_limit_error;
	input_limit = charger->raw_input_limit;
	mutex_unlock(&charger->lock);
	if (ret)
		return ret;

	/* D2365 code 0 is 75 mA; each following step adds 12.5 mA. */
	val->intval = 75000 + DIV_ROUND_CLOSEST(input_limit * 25000, 2);
	return 0;
}

static int apple_d2365_charger_charge_type(struct power_supply *psy,
					   union power_supply_propval *val)
{
	union power_supply_propval status;
	int ret;

	ret = apple_d2365_charger_status(psy, &status);
	if (ret)
		return ret;

	val->intval = status.intval == POWER_SUPPLY_STATUS_CHARGING ||
		status.intval == POWER_SUPPLY_STATUS_UNKNOWN ?
		POWER_SUPPLY_CHARGE_TYPE_UNKNOWN : POWER_SUPPLY_CHARGE_TYPE_NONE;

	return 0;
}

static int apple_d2365_charger_get_property(struct power_supply *psy,
					    enum power_supply_property property,
					    union power_supply_propval *val)
{
	int ret;

	switch (property) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = apple_d2365_charger_online(psy, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = apple_d2365_charger_status(psy, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = apple_d2365_charger_charge_type(psy, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = apple_d2365_charger_input_limit(psy, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void
apple_d2365_charger_external_power_changed(struct power_supply *psy)
{
	struct apple_d2365_charger *charger = power_supply_get_drvdata(psy);

	if (charger->config && !READ_ONCE(charger->shutting_down))
		mod_delayed_work(system_power_efficient_wq,
				 &charger->charge_policy_work, 0);

	power_supply_changed(psy);
}

static const enum power_supply_property apple_d2365_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static const struct power_supply_desc apple_d2365_charger_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = apple_d2365_charger_properties,
	.num_properties = ARRAY_SIZE(apple_d2365_charger_properties),
	.get_property = apple_d2365_charger_get_property,
	.external_power_changed = apple_d2365_charger_external_power_changed,
};

static int apple_d2365_charger_probe(struct platform_device *pdev)
{
	struct power_supply_config config = {};
	struct device *dev = &pdev->dev;
	struct apple_d2365_charger *charger;
	struct power_supply *psy;
	struct regmap *regmap;
	unsigned int status;
	u32 control_base;
	u32 status_base;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	ret = apple_d2365_get_bank(dev, "status",
				   D2365_STATUS_CHARGE + 1, &status_base);
	if (ret)
		return ret;

	ret = apple_d2365_get_bank(dev, "control",
				   D2365_CONTROL_MAX_CHARGE + 1,
				   &control_base);
	if (ret)
		return ret;

	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &charger->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &charger->policy_lock);
	if (ret)
		return ret;

	charger->dev = dev;
	charger->regmap = regmap;
	charger->config = device_get_match_data(dev);
	charger->status_base = status_base;
	charger->control_base = control_base;
	charger->status_error = -ENODATA;
	charger->input_limit_error = -ENODATA;
	ret = devm_delayed_work_autocancel(dev, &charger->charge_policy_work,
					   apple_d2365_charge_policy_work);
	if (ret)
		return ret;

	config.drv_data = charger;
	config.fwnode = dev_fwnode(dev);
	config.no_wakeup_source = true;

	psy = devm_power_supply_register(dev, &apple_d2365_charger_desc,
					 &config);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "failed to register power supply\n");
	charger->psy = psy;
	platform_set_drvdata(pdev, charger);

	/*
	 * Do not access the PMIC before all supplier probe deferrals are done.
	 */
	ret = regmap_read(regmap, status_base + D2365_STATUS_CHARGE, &status);
	apple_d2365_cache_status(charger, ret, ret ? 0 : status);

	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read charging status\n");

	ret = regmap_read(regmap, control_base + D2365_CONTROL_INPUT_LIMIT,
			  &status);
	apple_d2365_cache_input_limit(charger, ret, ret ? 0 : status);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read input-current limit\n");

	if (charger->config) {
		mod_delayed_work(system_power_efficient_wq,
				 &charger->charge_policy_work, 0);
	}

	power_supply_changed(psy);

	return 0;
}

static void apple_d2365_charger_stop(struct apple_d2365_charger *charger,
				     const char *reason)
{
	bool restore_pending = false;
	int attempt;

	mutex_lock(&charger->policy_lock);
	charger->shutting_down = true;
	mutex_unlock(&charger->policy_lock);

	cancel_delayed_work_sync(&charger->charge_policy_work);

	for (attempt = 1; attempt <= D2365_STOP_RESTORE_ATTEMPTS; attempt++) {
		mutex_lock(&charger->policy_lock);
		apple_d2365_policy_restore_locked(charger, reason);
		restore_pending = charger->restore_pending;
		mutex_unlock(&charger->policy_lock);

		if (!restore_pending)
			break;
		if (attempt < D2365_STOP_RESTORE_ATTEMPTS)
			msleep(D2365_RESTORE_RETRY_MS);
	}

	if (restore_pending)
		dev_crit(charger->dev,
			 "input-limit restore remains pending after %s\n", reason);
}

static void apple_d2365_charger_remove(struct platform_device *pdev)
{
	struct apple_d2365_charger *charger = platform_get_drvdata(pdev);

	apple_d2365_charger_stop(charger, "remove");
}

static void apple_d2365_charger_shutdown(struct platform_device *pdev)
{
	struct apple_d2365_charger *charger = platform_get_drvdata(pdev);

	apple_d2365_charger_stop(charger, "shutdown");
}

static const struct of_device_id apple_d2365_charger_of_match[] = {
	{
		.compatible = "apple,j172-d2365-charger",
		.data = &apple_d2365_j172_config,
	},
	{ .compatible = "apple,d2365-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_d2365_charger_of_match);

static struct platform_driver apple_d2365_charger_driver = {
	.probe = apple_d2365_charger_probe,
	.remove = apple_d2365_charger_remove,
	.shutdown = apple_d2365_charger_shutdown,
	.driver = {
		.name = "apple-d2365-charger",
		.of_match_table = apple_d2365_charger_of_match,
	},
};
module_platform_driver(apple_d2365_charger_driver);

MODULE_AUTHOR("Paul Praschl");
MODULE_DESCRIPTION("Apple D2365 charger status and guarded charging policy");
MODULE_LICENSE("GPL");

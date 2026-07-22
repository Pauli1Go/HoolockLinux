// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only USB power status for the Apple Bellatrix controller
 */

#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>

#define BELLATRIX_VBUS_STATUS		0x16
#define BELLATRIX_VBUS_EXT_VBUS	BIT(1)
#define BELLATRIX_STATUS2		0x17
#define BELLATRIX_STATUS2_PROVIDER	BIT(0)
#define BELLATRIX_STATUS2_CONSUMER	BIT(1)
#define BELLATRIX_CHIP_ID		0x73
#define BELLATRIX_CHIP_ID_VALUE		0x74

#define BELLATRIX_INIT_DELAY_MS		10
#define BELLATRIX_INIT_RETRY_DELAY_MS	100
#define BELLATRIX_INIT_ATTEMPTS		10
#define BELLATRIX_POLL_INTERVAL_MS	1000

struct apple_bellatrix {
	struct i2c_client *client;
	struct power_supply *psy;
	struct delayed_work init_work;
	/* Protects cached properties. */
	struct mutex lock;
	unsigned int init_attempts;
	bool initialized;
	int online;
	int online_error;
};

static int apple_bellatrix_read(struct i2c_client *client, u8 reg)
{
	u8 value;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_STOP,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &value,
		},
	};
	int ret;

	/*
	 * Bellatrix register reads require a STOP after the subaddress. Keep the
	 * selector write and data read in one locked adapter segment so no other
	 * transfer can run between them.
	 */
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);

	if (ret != ARRAY_SIZE(msgs)) {
		if (ret >= 0)
			ret = -EIO;
		return ret;
	}

	ret = value;

	return ret;
}

static int apple_bellatrix_update(struct apple_bellatrix *bellatrix,
				  bool *changed)
{
	int old_error;
	int old_online;
	int chip_id;
	int status2;
	int vbus;
	int online = 0;
	int ret = 0;

	*changed = false;

	chip_id = apple_bellatrix_read(bellatrix->client, BELLATRIX_CHIP_ID);
	if (chip_id < 0) {
		ret = chip_id;
		goto out_store;
	}

	if (chip_id != BELLATRIX_CHIP_ID_VALUE) {
		dev_err(&bellatrix->client->dev,
			"unexpected chip id 0x%02x\n", chip_id);
		ret = -ENODEV;
		goto out_store;
	}

	vbus = apple_bellatrix_read(bellatrix->client, BELLATRIX_VBUS_STATUS);
	if (vbus < 0) {
		ret = vbus;
		goto out_store;
	}

	if (!(vbus & BELLATRIX_VBUS_EXT_VBUS)) {
		online = 0;
		goto out_store;
	}

	status2 = apple_bellatrix_read(bellatrix->client, BELLATRIX_STATUS2);
	if (status2 < 0) {
		ret = status2;
		goto out_store;
	}

	if (status2 & BELLATRIX_STATUS2_PROVIDER)
		online = 0;
	else
		online = !!(status2 & BELLATRIX_STATUS2_CONSUMER);

out_store:
	mutex_lock(&bellatrix->lock);
	old_error = bellatrix->online_error;
	old_online = bellatrix->online;
	bellatrix->online_error = ret;
	if (!ret)
		bellatrix->online = online;
	*changed = old_error != ret || (!ret && old_online != online);
	mutex_unlock(&bellatrix->lock);

	return ret;
}

static int apple_bellatrix_get_property(struct power_supply *psy,
					enum power_supply_property property,
					union power_supply_propval *val)
{
	struct apple_bellatrix *bellatrix = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&bellatrix->lock);
	switch (property) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bellatrix->online_error;
		if (!ret)
			val->intval = bellatrix->online;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&bellatrix->lock);

	return ret;
}

static const enum power_supply_property apple_bellatrix_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc apple_bellatrix_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = apple_bellatrix_properties,
	.num_properties = ARRAY_SIZE(apple_bellatrix_properties),
	.get_property = apple_bellatrix_get_property,
};

static void apple_bellatrix_init_work(struct work_struct *work)
{
	struct apple_bellatrix *bellatrix =
		container_of(to_delayed_work(work), struct apple_bellatrix,
			     init_work);
	bool changed;
	int ret;

	ret = apple_bellatrix_update(bellatrix, &changed);
	if (changed)
		power_supply_changed(bellatrix->psy);

	if (!ret) {
		bellatrix->initialized = true;
		schedule_delayed_work(&bellatrix->init_work,
				      msecs_to_jiffies(BELLATRIX_POLL_INTERVAL_MS));
		return;
	}

	if (bellatrix->initialized) {
		schedule_delayed_work(&bellatrix->init_work,
				      msecs_to_jiffies(BELLATRIX_POLL_INTERVAL_MS));
		return;
	}

	bellatrix->init_attempts++;

	if ((ret == -ENXIO || ret == -EIO) &&
	    bellatrix->init_attempts < BELLATRIX_INIT_ATTEMPTS) {
		schedule_delayed_work(&bellatrix->init_work,
				      msecs_to_jiffies(BELLATRIX_INIT_RETRY_DELAY_MS));
		return;
	}

	dev_err(&bellatrix->client->dev,
		"failed to read USB power status after %u attempt(s): %pe\n",
		bellatrix->init_attempts, ERR_PTR(ret));
}

static int apple_bellatrix_probe(struct i2c_client *client)
{
	struct power_supply_config config = {};
	struct apple_bellatrix *bellatrix;
	int ret;

	bellatrix = devm_kzalloc(&client->dev, sizeof(*bellatrix), GFP_KERNEL);
	if (!bellatrix)
		return -ENOMEM;

	ret = devm_mutex_init(&client->dev, &bellatrix->lock);
	if (ret)
		return ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(&client->dev, -EOPNOTSUPP,
				     "plain I2C transfers unsupported\n");

	bellatrix->client = client;
	bellatrix->online_error = -ENODATA;

	config.drv_data = bellatrix;
	config.fwnode = dev_fwnode(&client->dev);
	config.no_wakeup_source = true;

	bellatrix->psy = devm_power_supply_register(&client->dev,
						    &apple_bellatrix_desc,
						    &config);
	if (IS_ERR(bellatrix->psy))
		return PTR_ERR(bellatrix->psy);

	ret = devm_delayed_work_autocancel(&client->dev,
					   &bellatrix->init_work,
					   apple_bellatrix_init_work);
	if (ret)
		return ret;

	schedule_delayed_work(&bellatrix->init_work,
			      msecs_to_jiffies(BELLATRIX_INIT_DELAY_MS));

	return 0;
}

static const struct of_device_id apple_bellatrix_of_match[] = {
	{ .compatible = "apple,bellatrix" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_bellatrix_of_match);

static struct i2c_driver apple_bellatrix_driver = {
	.probe = apple_bellatrix_probe,
	.driver = {
		.name = "apple-bellatrix-power",
		.of_match_table = apple_bellatrix_of_match,
	},
};
module_i2c_driver(apple_bellatrix_driver);

MODULE_AUTHOR("Paul Praschl");
MODULE_DESCRIPTION("Read-only Apple Bellatrix USB power status");
MODULE_LICENSE("GPL");

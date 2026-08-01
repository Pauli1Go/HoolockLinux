// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple PMIC touchscreen supply regulators
 *
 * Copyright (C) 2026 Paul Praschl
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

static const struct regulator_ops apple_pmic_touch_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc apple_adelyn_touch_core = {
	.name = "adelyn-touch-core",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x31f,
	.enable_mask = 0x01,
	.enable_val = 0x01,
	.disable_val = 0,
	.enable_time = 1000,
	.off_on_delay = 1000,
};

static const struct regulator_desc apple_chestnut_touch_hv = {
	.name = "chestnut-touch-hv",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x05,
	.enable_mask = 0x10,
	.enable_val = 0x10,
	.disable_val = 0,
	.enable_time = 1000,
	.off_on_delay = 1000,
};

static int apple_pmic_touch_regulator_probe(struct platform_device *pdev)
{
	const struct regulator_desc *desc;
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct regmap *regmap;

	desc = device_get_match_data(dev);
	if (!desc)
		return dev_err_probe(dev, -EINVAL, "missing regulator match data\n");

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV,
				     "parent PMIC has no regmap\n");

	config.dev = dev;
	config.of_node = dev->of_node;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node, desc);
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, desc, &config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id apple_pmic_touch_regulator_of_match[] = {
	{
		.compatible = "apple,adelyn-touch-core-regulator",
		.data = &apple_adelyn_touch_core,
	},
	{
		.compatible = "apple,chestnut-touch-hv-regulator",
		.data = &apple_chestnut_touch_hv,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pmic_touch_regulator_of_match);

static struct platform_driver apple_pmic_touch_regulator_driver = {
	.probe = apple_pmic_touch_regulator_probe,
	.driver = {
		.name = "apple-pmic-touch-regulator",
		.of_match_table = apple_pmic_touch_regulator_of_match,
	},
};
module_platform_driver(apple_pmic_touch_regulator_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple PMIC touchscreen supply regulators");
MODULE_LICENSE("GPL");

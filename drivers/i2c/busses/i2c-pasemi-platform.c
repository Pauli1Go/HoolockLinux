// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * PA Semi PWRficient SMBus host driver for Apple SoCs
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/types.h>

#include "i2c-pasemi-core.h"

#define APPLE_I2C_REG_RESET		0x10
#define APPLE_I2C_RESET			BIT(31)
#define APPLE_I2C_REG_FILTER		0x38
#define APPLE_I2C_FILTER_DEFAULT	BIT(9)

struct pasemi_platform_i2c_match_data {
	void (*hw_init)(struct pasemi_smbus *smbus);
	u32 ctl_flags;
};

struct pasemi_platform_i2c_data {
	struct pasemi_smbus smbus;
	struct clk *clk_ref;
	struct i2c_bus_recovery_info recovery;
};

static void pasemi_platform_i2c_t8010_init(struct pasemi_smbus *smbus)
{
	u32 filter;

	iowrite32(APPLE_I2C_RESET, smbus->ioaddr + APPLE_I2C_REG_RESET);

	filter = ioread32(smbus->ioaddr + APPLE_I2C_REG_FILTER);
	iowrite32(filter | APPLE_I2C_FILTER_DEFAULT,
		  smbus->ioaddr + APPLE_I2C_REG_FILTER);
}

static const struct pasemi_platform_i2c_match_data t8010_i2c_data = {
	.hw_init = pasemi_platform_i2c_t8010_init,
	.ctl_flags = 0,
};

static int
pasemi_platform_i2c_calc_clk_div(struct pasemi_platform_i2c_data *data,
				 u32 frequency)
{
	unsigned long clk_rate = clk_get_rate(data->clk_ref);

	if (!clk_rate)
		return -EINVAL;

	data->smbus.clk_div = DIV_ROUND_UP(clk_rate, 16 * frequency);
	if (data->smbus.clk_div < 4)
		return dev_err_probe(data->smbus.dev, -EINVAL,
				     "Bus frequency %d is too fast.\n",
				     frequency);
	if (data->smbus.clk_div > 0xff)
		return dev_err_probe(data->smbus.dev, -EINVAL,
				     "Bus frequency %d is too slow.\n",
				     frequency);

	return 0;
}

static int pasemi_platform_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct pasemi_platform_i2c_match_data *match_data;
	struct pasemi_platform_i2c_data *data;
	struct reset_control *reset;
	struct pasemi_smbus *smbus;
	u32 frequency;
	int error;
	int irq_num;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	smbus = &data->smbus;
	smbus->dev = dev;
	smbus->ctl_flags = PASEMI_CTL_LEGACY_FLAGS;
	match_data = device_get_match_data(dev);
	if (match_data) {
		smbus->hw_init = match_data->hw_init;
		smbus->ctl_flags = match_data->ctl_flags;
	}

	smbus->ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(smbus->ioaddr))
		return PTR_ERR(smbus->ioaddr);

	if (device_property_read_u32(dev, "clock-frequency", &frequency))
		frequency = I2C_MAX_STANDARD_MODE_FREQ;

	data->clk_ref = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(data->clk_ref))
		return PTR_ERR(data->clk_ref);

	error = pasemi_platform_i2c_calc_clk_div(data, frequency);
	if (error)
		return error;

	reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get reset\n");

	if (reset) {
		error = reset_control_reset(reset);
		if (error)
			return dev_err_probe(dev, error,
					     "failed to reset controller\n");
	}

	smbus->adapter.dev.of_node = pdev->dev.of_node;
	if (device_property_present(dev, "scl-gpios")) {
		data->recovery.recover_bus = i2c_generic_scl_recovery;
		smbus->adapter.bus_recovery_info = &data->recovery;
	}

	error = pasemi_i2c_common_probe(smbus);
	if (error)
		return error;

	irq_num = platform_get_irq(pdev, 0);
	error = devm_request_irq(smbus->dev, irq_num, pasemi_irq_handler, 0,
				 "pasemi_apple_i2c", smbus);

	if (!error)
		smbus->use_irq = 1;
	platform_set_drvdata(pdev, data);

	return 0;
}

static void pasemi_platform_i2c_remove(struct platform_device *pdev) { }

static const struct of_device_id pasemi_platform_i2c_of_match[] = {
	{ .compatible = "apple,t8010-i2c", .data = &t8010_i2c_data },
	{ .compatible = "apple,t8103-i2c" },
	{ .compatible = "apple,i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, pasemi_platform_i2c_of_match);

static struct platform_driver pasemi_platform_i2c_driver = {
	.driver	= {
		.name			= "i2c-apple",
		.of_match_table		= pasemi_platform_i2c_of_match,
	},
	.probe	= pasemi_platform_i2c_probe,
	.remove = pasemi_platform_i2c_remove,
};
module_platform_driver(pasemi_platform_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple/PASemi SMBus platform driver");

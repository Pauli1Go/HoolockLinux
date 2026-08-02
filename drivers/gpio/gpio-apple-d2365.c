// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for Apple/Dialog PMICs used with the T8010 SoC.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define D2365_GPIO_SPLIT		18
#define D2365_GPIO_CFG_0_17	0x900
#define D2365_GPIO_CFG_18_23	0x8bd

#define D2365_GPIO_VALUE		BIT(0)
#define D2365_GPIO_INPUT		BIT(3)
#define D2365_GPIO_FUNCTION	BIT(4)

#define D2333_GPIO_VALUE		BIT(0)
#define D2333_GPIO_DIRECTION_MASK	GENMASK(7, 6)
#define D2333_GPIO_INPUT		0
#define D2333_GPIO_OUTPUT	BIT(7)

struct apple_pmic_gpio_info {
	const char *label;
	unsigned int ngpio;
	unsigned int input_base;
	unsigned int value_mask;
	unsigned int direction_mask;
	unsigned int input_mode;
	unsigned int output_mode;
	unsigned int function_mask;
	unsigned int (*config_reg)(unsigned int offset);
};

struct apple_pmic_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
	const struct apple_pmic_gpio_info *info;
};

static unsigned int d2365_gpio_config_reg(unsigned int offset)
{
	if (offset < D2365_GPIO_SPLIT)
		return D2365_GPIO_CFG_0_17 + 2 * offset;

	return D2365_GPIO_CFG_18_23 + 6 * offset;
}

static const unsigned int d2333_gpio_config_regs[] = {
	0x900, 0x902, 0x904, 0x906, 0x908, 0x90a, 0x90c,
	0x90e, 0x910, 0x912, 0x914, 0x916, 0x918, 0x91a,
	0x91c, 0x91e, 0x920, 0x927, 0x92d, 0x933, 0x939,
};

static unsigned int d2333_gpio_config_reg(unsigned int offset)
{
	return d2333_gpio_config_regs[offset];
}

static int apple_pmic_gpio_get_direction(struct gpio_chip *chip,
					 unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, info->config_reg(offset), &value);
	if (ret)
		return ret;
	if (value & info->function_mask)
		return -EINVAL;

	if ((value & info->direction_mask) == info->input_mode)
		return GPIO_LINE_DIRECTION_IN;
	if ((value & info->direction_mask) == info->output_mode)
		return GPIO_LINE_DIRECTION_OUT;

	return -EINVAL;
}

static int apple_pmic_gpio_direction_input(struct gpio_chip *chip,
					   unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->direction_mask | info->value_mask,
				  info->input_mode);
}

static int apple_pmic_gpio_direction_output(struct gpio_chip *chip,
					    unsigned int offset, int value)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->direction_mask | info->value_mask,
				  info->output_mode |
				  (value ? info->value_mask : 0));
}

static int apple_pmic_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, info->config_reg(offset), &value);
	if (ret)
		return ret;
	if ((value & info->direction_mask) == info->output_mode)
		return !!(value & info->value_mask);
	if ((value & info->direction_mask) != info->input_mode)
		return -EINVAL;

	ret = regmap_read(gpio->regmap, info->input_base + offset / 8,
			  &value);
	if (ret)
		return ret;

	return !!(value & BIT(offset % 8));
}

static int apple_pmic_gpio_set(struct gpio_chip *chip, unsigned int offset,
			       int value)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->value_mask,
				  value ? info->value_mask : 0);
}

static int apple_pmic_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmic_gpio *gpio;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->info = device_get_match_data(dev);
	if (!gpio->info)
		return -EINVAL;

	gpio->regmap = dev_get_regmap(dev->parent, NULL);
	if (!gpio->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	gpio->chip.label = gpio->info->label;
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = gpio->info->ngpio;
	gpio->chip.can_sleep = true;
	gpio->chip.get_direction = apple_pmic_gpio_get_direction;
	gpio->chip.direction_input = apple_pmic_gpio_direction_input;
	gpio->chip.direction_output = apple_pmic_gpio_direction_output;
	gpio->chip.get = apple_pmic_gpio_get;
	gpio->chip.set = apple_pmic_gpio_set;

	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static const struct apple_pmic_gpio_info d2333_gpio_info = {
	.label = "d2333-pmic-gpio",
	.ngpio = ARRAY_SIZE(d2333_gpio_config_regs),
	.input_base = 0x186,
	.value_mask = D2333_GPIO_VALUE,
	.direction_mask = D2333_GPIO_DIRECTION_MASK,
	.input_mode = D2333_GPIO_INPUT,
	.output_mode = D2333_GPIO_OUTPUT,
	.config_reg = d2333_gpio_config_reg,
};

static const struct apple_pmic_gpio_info d2365_gpio_info = {
	.label = "d2365-pmic-gpio",
	.ngpio = 24,
	.input_base = 0x186,
	.value_mask = D2365_GPIO_VALUE,
	.direction_mask = D2365_GPIO_INPUT | D2365_GPIO_FUNCTION,
	.input_mode = D2365_GPIO_INPUT,
	.output_mode = 0,
	.function_mask = D2365_GPIO_FUNCTION,
	.config_reg = d2365_gpio_config_reg,
};

static const struct of_device_id d2365_gpio_of_match[] = {
	{
		.compatible = "apple,d2333-pmic-gpio",
		.data = &d2333_gpio_info,
	},
	{
		.compatible = "apple,d2365-pmic-gpio",
		.data = &d2365_gpio_info,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_gpio_of_match);

static struct platform_driver d2365_gpio_driver = {
	.probe = apple_pmic_gpio_probe,
	.driver = {
		.name = "apple-d2365-pmic-gpio",
		.of_match_table = d2365_gpio_of_match,
	},
};
module_platform_driver(d2365_gpio_driver);

MODULE_DESCRIPTION("Apple T8010 PMIC GPIO driver");
MODULE_LICENSE("GPL");

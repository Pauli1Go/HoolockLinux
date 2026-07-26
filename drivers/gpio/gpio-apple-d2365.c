// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for the Apple/Dialog D2365 PMIC.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define D2365_GPIO_COUNT		24
#define D2365_GPIO_SPLIT		18
#define D2365_GPIO_CFG_0_17	0x900
#define D2365_GPIO_CFG_18_23	0x8bd
#define D2365_GPIO_INPUT0	0x186

#define D2365_GPIO_VALUE		BIT(0)
#define D2365_GPIO_INPUT		BIT(3)
#define D2365_GPIO_FUNCTION	BIT(4)
#define D2365_GPIO_MODE_MASK	(D2365_GPIO_VALUE | D2365_GPIO_INPUT | \
				 D2365_GPIO_FUNCTION)

struct d2365_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
};

static unsigned int d2365_gpio_config_reg(unsigned int offset)
{
	if (offset < D2365_GPIO_SPLIT)
		return D2365_GPIO_CFG_0_17 + 2 * offset;

	return D2365_GPIO_CFG_18_23 + 6 * offset;
}

static int d2365_gpio_get_direction(struct gpio_chip *chip,
				    unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, d2365_gpio_config_reg(offset), &value);
	if (ret)
		return ret;
	if (value & D2365_GPIO_FUNCTION)
		return -EINVAL;

	return value & D2365_GPIO_INPUT ? GPIO_LINE_DIRECTION_IN :
					  GPIO_LINE_DIRECTION_OUT;
}

static int d2365_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);

	return regmap_update_bits(gpio->regmap,
				  d2365_gpio_config_reg(offset),
				  D2365_GPIO_MODE_MASK, D2365_GPIO_INPUT);
}

static int d2365_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);

	return regmap_update_bits(gpio->regmap,
				  d2365_gpio_config_reg(offset),
				  D2365_GPIO_MODE_MASK,
				  value ? D2365_GPIO_VALUE : 0);
}

static int d2365_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, d2365_gpio_config_reg(offset), &value);
	if (ret)
		return ret;
	if (!(value & D2365_GPIO_INPUT))
		return !!(value & D2365_GPIO_VALUE);

	ret = regmap_read(gpio->regmap, D2365_GPIO_INPUT0 + offset / 8,
			  &value);
	if (ret)
		return ret;

	return !!(value & BIT(offset % 8));
}

static int d2365_gpio_set(struct gpio_chip *chip, unsigned int offset,
			  int value)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);

	return regmap_update_bits(gpio->regmap,
				  d2365_gpio_config_reg(offset),
				  D2365_GPIO_VALUE,
				  value ? D2365_GPIO_VALUE : 0);
}

static int d2365_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct d2365_gpio *gpio;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->regmap = dev_get_regmap(dev->parent, NULL);
	if (!gpio->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	gpio->chip.label = "d2365-pmic-gpio";
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = D2365_GPIO_COUNT;
	gpio->chip.can_sleep = true;
	gpio->chip.get_direction = d2365_gpio_get_direction;
	gpio->chip.direction_input = d2365_gpio_direction_input;
	gpio->chip.direction_output = d2365_gpio_direction_output;
	gpio->chip.get = d2365_gpio_get;
	gpio->chip.set = d2365_gpio_set;

	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static const struct of_device_id d2365_gpio_of_match[] = {
	{ .compatible = "apple,d2365-pmic-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_gpio_of_match);

static struct platform_driver d2365_gpio_driver = {
	.probe = d2365_gpio_probe,
	.driver = {
		.name = "apple-d2365-pmic-gpio",
		.of_match_table = d2365_gpio_of_match,
	},
};
module_platform_driver(d2365_gpio_driver);

MODULE_DESCRIPTION("Apple D2365 PMIC GPIO driver");
MODULE_LICENSE("GPL");

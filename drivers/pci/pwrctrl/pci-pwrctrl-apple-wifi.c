// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple PCIe Wi-Fi power control
 *
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci-pwrctrl.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct apple_wifi_pwrctrl {
	struct pci_pwrctrl pwrctrl;
	struct gpio_desc *device_wake;
	struct regulator *vdd;
};

static int apple_wifi_power_on(struct pci_pwrctrl *pwrctrl)
{
	struct apple_wifi_pwrctrl *wifi =
		container_of(pwrctrl, struct apple_wifi_pwrctrl, pwrctrl);
	int ret;

	ret = regulator_enable(wifi->vdd);
	if (ret)
		return ret;

	msleep(100);

	return 0;
}

static int apple_wifi_power_off(struct pci_pwrctrl *pwrctrl)
{
	struct apple_wifi_pwrctrl *wifi =
		container_of(pwrctrl, struct apple_wifi_pwrctrl, pwrctrl);
	int ret;

	gpiod_set_value_cansleep(wifi->device_wake, 0);

	ret = regulator_disable(wifi->vdd);

	return ret;
}

static int apple_wifi_link_uart_provider(struct device *dev)
{
	struct platform_device *uart_pdev;
	struct device_node *uart_np;
	struct device_link *link;
	int ret;

	uart_np = of_parse_phandle(dev->of_node, "apple,wlan-uart", 0);
	if (!uart_np)
		return 0;

	uart_pdev = of_find_device_by_node(uart_np);
	of_node_put(uart_np);
	if (!uart_pdev)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for WLAN UART provider\n");

	if (!device_is_bound(&uart_pdev->dev)) {
		ret = dev_err_probe(dev, -EPROBE_DEFER,
				    "waiting for bound WLAN UART provider\n");
		goto out_put_device;
	}

	link = device_link_add(dev, &uart_pdev->dev,
			       DL_FLAG_AUTOREMOVE_CONSUMER |
			       DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	ret = link ? 0 : -EINVAL;

out_put_device:
	platform_device_put(uart_pdev);
	return ret;
}

static int apple_wifi_pwrctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_wifi_pwrctrl *wifi;
	int ret;

	wifi = devm_kzalloc(dev, sizeof(*wifi), GFP_KERNEL);
	if (!wifi)
		return -ENOMEM;

	wifi->device_wake = devm_gpiod_get(dev, "device-wake", GPIOD_OUT_LOW);
	if (IS_ERR(wifi->device_wake))
		return dev_err_probe(dev, PTR_ERR(wifi->device_wake),
				     "failed to get device-wake GPIO\n");

	wifi->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(wifi->vdd))
		return dev_err_probe(dev, PTR_ERR(wifi->vdd),
				     "failed to get VDD supply\n");

	ret = apple_wifi_link_uart_provider(dev);
	if (ret)
		return ret;

	wifi->pwrctrl.power_on = apple_wifi_power_on;
	wifi->pwrctrl.power_off = apple_wifi_power_off;
	pci_pwrctrl_init(&wifi->pwrctrl, dev);

	ret = devm_pci_pwrctrl_device_set_ready(dev, &wifi->pwrctrl);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register power control\n");

	return 0;
}

static const struct of_device_id apple_wifi_pwrctrl_of_match[] = {
	{ .compatible = "pci14e4,43dc" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_wifi_pwrctrl_of_match);

static struct platform_driver apple_wifi_pwrctrl_driver = {
	.driver = {
		.name = "pci-pwrctrl-apple-wifi",
		.of_match_table = apple_wifi_pwrctrl_of_match,
	},
	.probe = apple_wifi_pwrctrl_probe,
};
module_platform_driver(apple_wifi_pwrctrl_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple PCIe Wi-Fi power control driver");
MODULE_LICENSE("GPL");

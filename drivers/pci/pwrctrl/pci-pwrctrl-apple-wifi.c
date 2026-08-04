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

struct apple_wifi_pwrctrl_data {
	bool pulse_device_wake;
	unsigned int step_delay_min_us;
	unsigned int step_delay_max_us;
	unsigned int stabilization_delay_ms;
};

struct apple_wifi_pwrctrl {
	struct pci_pwrctrl pwrctrl;
	struct gpio_desc *device_wake;
	struct regulator *vdd;
	const struct apple_wifi_pwrctrl_data *data;
};

static const struct apple_wifi_pwrctrl_data apple_iphone7_plus_wifi_data = {
	.pulse_device_wake = true,
	.step_delay_min_us = 2500,
	.step_delay_max_us = 5000,
};

static const struct apple_wifi_pwrctrl_data apple_j172_wifi_data = {
	.stabilization_delay_ms = 100,
};

static void apple_wifi_step_delay(const struct apple_wifi_pwrctrl *wifi)
{
	if (wifi->data->step_delay_min_us)
		usleep_range(wifi->data->step_delay_min_us,
			     wifi->data->step_delay_max_us);
}

static int apple_wifi_set_device_wake(struct apple_wifi_pwrctrl *wifi,
				      int value)
{
	return gpiod_set_value_cansleep(wifi->device_wake, value);
}

static int apple_wifi_set_vdd(struct apple_wifi_pwrctrl *wifi, bool enable)
{
	return enable ? regulator_enable(wifi->vdd) :
			regulator_disable(wifi->vdd);
}

static int apple_wifi_power_on(struct pci_pwrctrl *pwrctrl)
{
	struct apple_wifi_pwrctrl *wifi =
		container_of(pwrctrl, struct apple_wifi_pwrctrl, pwrctrl);
	int rollback;
	int ret;

	if (wifi->data->pulse_device_wake) {
		apple_wifi_step_delay(wifi);
		ret = apple_wifi_set_device_wake(wifi, 1);
		if (ret)
			return ret;
		apple_wifi_step_delay(wifi);
		ret = apple_wifi_set_device_wake(wifi, 0);
		if (ret)
			return ret;

		ret = apple_wifi_set_vdd(wifi, true);
		if (ret)
			return ret;

		apple_wifi_step_delay(wifi);
		ret = apple_wifi_set_device_wake(wifi, 1);
		if (ret) {
			rollback = apple_wifi_set_vdd(wifi, false);
			if (rollback)
				dev_err(pwrctrl->dev,
					"failed to roll back VDD: %d\n", rollback);
			apple_wifi_step_delay(wifi);
			return ret;
		}
		apple_wifi_step_delay(wifi);
	} else {
		ret = apple_wifi_set_vdd(wifi, true);
		if (ret)
			return ret;
	}

	if (wifi->data->stabilization_delay_ms)
		msleep(wifi->data->stabilization_delay_ms);

	return 0;
}

static int apple_wifi_power_off(struct pci_pwrctrl *pwrctrl)
{
	struct apple_wifi_pwrctrl *wifi =
		container_of(pwrctrl, struct apple_wifi_pwrctrl, pwrctrl);
	int wake_ret;
	int ret;

	wake_ret = apple_wifi_set_device_wake(wifi, 0);
	apple_wifi_step_delay(wifi);
	ret = apple_wifi_set_vdd(wifi, false);
	apple_wifi_step_delay(wifi);

	return ret ?: wake_ret;
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

	wifi->data = device_get_match_data(dev);
	if (!wifi->data)
		return dev_err_probe(dev, -EINVAL, "missing match data\n");

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
	{
		.compatible = "apple,d11-bcm4355-wifi",
		.data = &apple_iphone7_plus_wifi_data,
	},
	{
		.compatible = "apple,d111-bcm4355-wifi",
		.data = &apple_iphone7_plus_wifi_data,
	},
	{
		.compatible = "pci14e4,43dc",
		.data = &apple_j172_wifi_data,
	},
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

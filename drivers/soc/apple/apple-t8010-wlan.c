// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple WLAN platform-control provider
 *
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/serial_s3c.h>

struct apple_wlan_provider {
	void __iomem *regs;
};

static void apple_wlan_provider_prepare_uart(struct device *dev)
{
	struct apple_wlan_provider *provider = dev_get_drvdata(dev);
	u32 ucon;

	ucon = readl_relaxed(provider->regs + S3C2410_UCON);
	ucon &= ~APPLE_S5L_UCON_MASK;
	writel_relaxed(ucon, provider->regs + S3C2410_UCON);
	writel_relaxed(APPLE_S5L_UTRSTAT_ALL_FLAGS,
		       provider->regs + S3C2410_UTRSTAT);

	ucon = readl_relaxed(provider->regs + S3C2410_UCON);
	ucon &= APPLE_S5L_UCON_MASK;
	writel_relaxed(ucon | APPLE_S5L_UCON_DEFAULT,
		       provider->regs + S3C2410_UCON);
	writel_relaxed(S3C2410_UFCON_DEFAULT | S3C2410_UFCON_RESETBOTH,
		       provider->regs + S3C2410_UFCON);
	writel_relaxed(S3C2410_UFCON_DEFAULT, provider->regs + S3C2410_UFCON);

	udelay(1);
}

static int apple_wlan_provider_runtime_resume(struct device *dev)
{
	apple_wlan_provider_prepare_uart(dev);
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(apple_wlan_provider_pm_ops,
				 NULL,
				 apple_wlan_provider_runtime_resume,
				 NULL);

static int apple_wlan_provider_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_wlan_provider *provider;
	struct pinctrl *pinctrl;
	int ret;

	provider = devm_kzalloc(dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	provider->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(provider->regs))
		return PTR_ERR(provider->regs);

	platform_set_drvdata(pdev, provider);

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		return dev_err_probe(dev, PTR_ERR(pinctrl),
				     "failed to select WLAN UART2 pins\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable runtime PM\n");

	return 0;
}

static const struct of_device_id apple_wlan_provider_of_match[] = {
	{ .compatible = "apple,t8010-wlan-uart-provider" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_wlan_provider_of_match);

static struct platform_driver apple_wlan_provider_driver = {
	.driver = {
		.name = "apple-wlan-provider",
		.of_match_table = apple_wlan_provider_of_match,
		.pm = pm_ptr(&apple_wlan_provider_pm_ops),
	},
	.probe = apple_wlan_provider_probe,
};
module_platform_driver(apple_wlan_provider_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple WLAN platform-control provider");
MODULE_LICENSE("GPL");

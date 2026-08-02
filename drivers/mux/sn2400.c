// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SN2400 HDQ multiplexer driver
 *
 * Copyright (c) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define SN2400_REG_STATUS		0x07
#define SN2400_STATUS_CHARGING		BIT(7)

#define SN2400_REG_HDQ			0x1d
#define SN2400_HDQ_REQUEST_AP		BIT(2)
#define SN2400_HDQ_ACK_AP		BIT(5)
#define SN2400_HDQ_FORCE_CHARGER		BIT(1)

#define SN2400_HDQ_ROUTE_CHARGER		0
#define SN2400_HDQ_ROUTE_AP		1

struct sn2400_hdq {
	struct regmap *regmap;
};

static int sn2400_hdq_request_ap(struct regmap *regmap)
{
	unsigned int value;
	int ret;

	ret = regmap_write(regmap, SN2400_REG_HDQ, SN2400_HDQ_REQUEST_AP);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(regmap, SN2400_REG_HDQ, value,
					(value & SN2400_HDQ_ACK_AP),
					10000, 1000000);
}

static int sn2400_hdq_set(struct mux_control *mux, int state)
{
	struct sn2400_hdq *hdq = mux_chip_priv(mux->chip);
	struct regmap *regmap = hdq->regmap;
	unsigned int status;
	int ret;

	if (state == SN2400_HDQ_ROUTE_AP)
		return sn2400_hdq_request_ap(regmap);

	if (state != SN2400_HDQ_ROUTE_CHARGER)
		return -EINVAL;

	ret = regmap_read(regmap, SN2400_REG_STATUS, &status);
	if (ret)
		return ret;

	if (status & SN2400_STATUS_CHARGING)
		return regmap_write(regmap, SN2400_REG_HDQ, 0);

	ret = sn2400_hdq_request_ap(regmap);
	if (ret)
		return ret;

	return regmap_write(regmap, SN2400_REG_HDQ,
			    SN2400_HDQ_REQUEST_AP | SN2400_HDQ_FORCE_CHARGER);
}

static const struct mux_control_ops sn2400_hdq_ops = {
	.set = sn2400_hdq_set,
};

static int sn2400_hdq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sn2400_hdq *hdq;
	struct mux_chip *mux_chip;
	struct regmap *regmap;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get parent regmap\n");

	mux_chip = devm_mux_chip_alloc(dev, 1, sizeof(*hdq));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	hdq = mux_chip_priv(mux_chip);
	hdq->regmap = regmap;
	mux_chip->ops = &sn2400_hdq_ops;
	mux_chip->mux[0].states = 2;
	mux_chip->mux[0].idle_state = SN2400_HDQ_ROUTE_CHARGER;

	return devm_mux_chip_register(dev, mux_chip);
}

static const struct of_device_id sn2400_hdq_of_match[] = {
	{ .compatible = "apple,sn2400-hdq-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, sn2400_hdq_of_match);

static struct platform_driver sn2400_hdq_driver = {
	.probe = sn2400_hdq_probe,
	.driver = {
		.name = "sn2400-hdq-mux",
		.of_match_table = sn2400_hdq_of_match,
	},
};
module_platform_driver(sn2400_hdq_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple SN2400 HDQ multiplexer driver");
MODULE_LICENSE("GPL");

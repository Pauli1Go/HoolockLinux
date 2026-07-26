// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset-controller.h>
#include <linux/module.h>

#define APPLE_PMGR_RESET        BIT(31)
#define APPLE_PMGR_AUTO_ENABLE  BIT(28)
#define APPLE_PMGR_PS_AUTO      GENMASK(27, 24)
#define APPLE_PMGR_PS_MIN       GENMASK(19, 16)
#define APPLE_PMGR_PARENT_OFF   BIT(11)
#define APPLE_PMGR_DEV_DISABLE  BIT(10)
#define APPLE_PMGR_WAS_CLKGATED BIT(9)
#define APPLE_PMGR_WAS_PWRGATED BIT(8)
#define APPLE_PMGR_PS_ACTUAL    GENMASK(7, 4)
#define APPLE_PMGR_PS_TARGET    GENMASK(3, 0)

#define APPLE_PMGR_FLAGS        (APPLE_PMGR_WAS_CLKGATED | APPLE_PMGR_WAS_PWRGATED)

#define APPLE_PMGR_PS_ACTIVE    0xf
#define APPLE_PMGR_PS_CLKGATE   0x4
#define APPLE_PMGR_PS_PWRGATE   0x0

#define APPLE_PMGR_PS_SET_TIMEOUT 100
#define APPLE_PMGR_RESET_TIME 1
struct apple_pmgr_ps;

struct apple_pmgr_gate {
	struct clk_hw hw;
	struct apple_pmgr_ps *ps;
	u32 off_state;
};

struct apple_pmgr_ps {
	struct device *dev;
	struct generic_pm_domain genpd;
	struct reset_controller_dev rcdev;
	struct apple_pmgr_gate gate;
	struct regmap *regmap;
	/* Protects shared PMGR state register accesses. */
	spinlock_t lock;
	const char *name;
	u32 offset;
	u32 min_state;
	bool has_genpd;
	bool manual_pm;
};

#define genpd_to_apple_pmgr_ps(_genpd) container_of(_genpd, struct apple_pmgr_ps, genpd)
#define rcdev_to_apple_pmgr_ps(_rcdev) container_of(_rcdev, struct apple_pmgr_ps, rcdev)
#define clk_hw_to_apple_pmgr_gate(_hw) container_of(_hw, struct apple_pmgr_gate, hw)

static int apple_pmgr_ps_set(struct apple_pmgr_ps *ps, u32 pstate, bool auto_enable)
{
	int ret;
	u32 reg;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret < 0)
		return ret;

	/* Resets are synchronous, and only work if the device is powered and clocked. */
	if (reg & APPLE_PMGR_RESET && pstate != APPLE_PMGR_PS_ACTIVE)
		dev_err(ps->dev, "PS %s: powering off with RESET active\n",
			ps->name);

	reg &= ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_PS_TARGET);
	reg |= FIELD_PREP(APPLE_PMGR_PS_TARGET, pstate);

	dev_dbg(ps->dev, "PS %s: pwrstate = 0x%x: 0x%x\n", ps->name, pstate, reg);

	regmap_write(ps->regmap, ps->offset, reg);

	ret = regmap_read_poll_timeout_atomic(
		ps->regmap, ps->offset, reg,
		(FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == pstate), 1,
		APPLE_PMGR_PS_SET_TIMEOUT);
	if (ret < 0)
		dev_err(ps->dev, "PS %s: Failed to reach power state 0x%x (now: 0x%x)\n",
			ps->name, pstate, reg);

	if (auto_enable) {
		/* Not all devices implement this; this is a no-op where not implemented. */
		reg &= ~APPLE_PMGR_FLAGS;
		reg |= APPLE_PMGR_AUTO_ENABLE;
		regmap_write(ps->regmap, ps->offset, reg);
	}

	return ret;
}

static bool apple_pmgr_ps_is_active(struct apple_pmgr_ps *ps)
{
	u32 reg = 0;

	regmap_read(ps->regmap, ps->offset, &reg);
	/*
	 * We consider domains as active if they are actually on, or if they have auto-PM
	 * enabled and the intended target is on.
	 */
	return (FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE ||
		(FIELD_GET(APPLE_PMGR_PS_TARGET, reg) == APPLE_PMGR_PS_ACTIVE &&
		 reg & APPLE_PMGR_AUTO_ENABLE));
}

static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ps->lock, flags);
	ret = apple_pmgr_ps_set(ps, APPLE_PMGR_PS_ACTIVE, !ps->manual_pm);
	spin_unlock_irqrestore(&ps->lock, flags);

	return ret;
}

static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ps->lock, flags);
	ret = apple_pmgr_ps_set(ps, APPLE_PMGR_PS_PWRGATE, false);
	spin_unlock_irqrestore(&ps->lock, flags);

	return ret;
}

static int apple_pmgr_clk_enable(struct clk_hw *hw)
{
	struct apple_pmgr_gate *gate = clk_hw_to_apple_pmgr_gate(hw);
	struct apple_pmgr_ps *ps = gate->ps;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ps->lock, flags);
	/* Preserve firmware state when the provider is already acquired. */
	if (apple_pmgr_ps_is_active(ps))
		ret = 0;
	else
		ret = apple_pmgr_ps_set(ps, APPLE_PMGR_PS_ACTIVE, false);
	spin_unlock_irqrestore(&ps->lock, flags);

	return ret;
}

static void apple_pmgr_clk_disable(struct clk_hw *hw)
{
	struct apple_pmgr_gate *gate = clk_hw_to_apple_pmgr_gate(hw);
	struct apple_pmgr_ps *ps = gate->ps;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ps->lock, flags);
	ret = apple_pmgr_ps_set(ps, gate->off_state, false);
	spin_unlock_irqrestore(&ps->lock, flags);

	if (ret)
		dev_err(ps->dev, "failed to clock-gate %s: %d\n",
			ps->name, ret);
}

static int apple_pmgr_clk_is_enabled(struct clk_hw *hw)
{
	struct apple_pmgr_gate *gate = clk_hw_to_apple_pmgr_gate(hw);

	return apple_pmgr_ps_is_active(gate->ps);
}

static const struct clk_ops apple_pmgr_clk_ops = {
	.enable = apple_pmgr_clk_enable,
	.disable = apple_pmgr_clk_disable,
	.is_enabled = apple_pmgr_clk_is_enabled,
};

static int apple_pmgr_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;

	spin_lock_irqsave(&ps->lock, flags);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: asserting RESET while powered down\n", ps->offset);

	dev_dbg(ps->dev, "PS 0x%x: assert reset\n", ps->offset);
	/* Quiesce device before asserting reset */
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE,
			   APPLE_PMGR_DEV_DISABLE);
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET,
			   APPLE_PMGR_RESET);

	spin_unlock_irqrestore(&ps->lock, flags);

	return 0;
}

static int apple_pmgr_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;

	spin_lock_irqsave(&ps->lock, flags);

	dev_dbg(ps->dev, "PS 0x%x: deassert reset\n", ps->offset);
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET, 0);
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE, 0);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: RESET was deasserted while powered down\n", ps->offset);

	spin_unlock_irqrestore(&ps->lock, flags);

	return 0;
}

static int apple_pmgr_reset_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = apple_pmgr_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(APPLE_PMGR_RESET_TIME, 2 * APPLE_PMGR_RESET_TIME);

	return apple_pmgr_reset_deassert(rcdev, id);
}

static int apple_pmgr_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	u32 reg = 0;

	regmap_read(ps->regmap, ps->offset, &reg);

	return !!(reg & APPLE_PMGR_RESET);
}

static const struct reset_control_ops apple_pmgr_reset_ops = {
	.assert		= apple_pmgr_reset_assert,
	.deassert	= apple_pmgr_reset_deassert,
	.reset		= apple_pmgr_reset_reset,
	.status		= apple_pmgr_reset_status,
};

static int apple_pmgr_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int apple_pmgr_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmgr_ps *ps;
	struct clk_init_data clk_init = {};
	struct regmap *regmap;
	struct of_phandle_iterator it;
	u32 clock_cells;
	int ret;
	const char *name;
	bool active;
	bool clock_only;

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->dev = dev;
	ps->regmap = regmap;
	spin_lock_init(&ps->lock);

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0) {
		dev_err(dev, "missing label property\n");
		return ret;
	}
	ps->name = name;

	ret = of_property_read_u32(node, "reg", &ps->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	clock_only = of_property_read_bool(node, "apple,clock-gate-only");
	if (!clock_only) {
		ps->genpd.flags |= GENPD_FLAG_IRQ_SAFE;
		ps->genpd.name = name;
		ps->genpd.power_on = apple_pmgr_ps_power_on;
		ps->genpd.power_off = apple_pmgr_ps_power_off;
	}

	ret = of_property_read_u32(node, "apple,min-state", &ps->min_state);
	if (ret == 0 && ps->min_state <= APPLE_PMGR_PS_ACTIVE)
		regmap_update_bits(regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_PS_MIN,
				   FIELD_PREP(APPLE_PMGR_PS_MIN, ps->min_state));

	active = apple_pmgr_ps_is_active(ps);
	ps->manual_pm = of_property_read_bool(node, "apple,manual-pm");
	if (!clock_only && of_property_read_bool(node, "apple,always-on")) {
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		if (!active) {
			dev_warn(dev, "always-on domain %s is not on at boot\n", name);
			/* Turn it on so pm_genpd_init does not fail */
			active = apple_pmgr_ps_power_on(&ps->genpd) == 0;
		}
	}

	/* Turn on auto-PM if the domain is already on */
	if (active)
		regmap_update_bits(regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_AUTO_ENABLE,
				   ps->manual_pm ? 0 : APPLE_PMGR_AUTO_ENABLE);

	if (!clock_only) {
		ret = pm_genpd_init(&ps->genpd, NULL, !active);
		if (ret < 0) {
			dev_err(dev, "pm_genpd_init failed\n");
			return ret;
		}

		ret = of_genpd_add_provider_simple(node, &ps->genpd);
		if (ret < 0) {
			dev_err(dev, "of_genpd_add_provider_simple failed\n");
			pm_genpd_remove(&ps->genpd);
			return ret;
		}
		ps->has_genpd = true;

		of_for_each_phandle(&it, ret, node, "power-domains",
				    "#power-domain-cells", -1) {
			struct of_phandle_args parent, child;

			parent.np = it.node;
			parent.args_count =
				of_phandle_iterator_args(&it, parent.args,
							 MAX_PHANDLE_ARGS);
			child.np = node;
			child.args_count = 0;
			ret = of_genpd_add_subdomain(&parent, &child);

			if (ret == -EPROBE_DEFER) {
				of_node_put(parent.np);
				goto err_remove;
			} else if (ret < 0) {
				dev_err(dev,
					"failed to add to parent domain: %d (%s -> %s)\n",
					ret, it.node->name, node->name);
				of_node_put(parent.np);
				goto err_remove;
			}
		}

		/*
		 * Do not participate in regular PM; parent power domains are
		 * handled via the genpd hierarchy.
		 */
		pm_genpd_remove_device(dev);

		ps->rcdev.owner = THIS_MODULE;
		ps->rcdev.nr_resets = 1;
		ps->rcdev.ops = &apple_pmgr_reset_ops;
		ps->rcdev.of_node = dev->of_node;
		ps->rcdev.of_reset_n_cells = 0;
		ps->rcdev.of_xlate = apple_pmgr_reset_xlate;

		ret = devm_reset_controller_register(dev, &ps->rcdev);
		if (ret < 0)
			goto err_remove;
	}

	if (of_property_present(node, "#clock-cells")) {
		ret = of_property_read_u32(node, "#clock-cells", &clock_cells);
		if (ret)
			goto err_remove;
		if (clock_cells) {
			ret = -EINVAL;
			goto err_remove;
		}

		ps->gate.ps = ps;
		ps->gate.off_state = APPLE_PMGR_PS_CLKGATE;
		clk_init.name = name;
		clk_init.ops = &apple_pmgr_clk_ops;
		ps->gate.hw.init = &clk_init;

		ret = devm_clk_hw_register(dev, &ps->gate.hw);
		if (ret < 0)
			goto err_remove;

		ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
						  &ps->gate.hw);
		if (ret < 0)
			goto err_remove;
	}

	return 0;
err_remove:
	if (ps->has_genpd) {
		of_genpd_del_provider(node);
		pm_genpd_remove(&ps->genpd);
	}
	return ret;
}

static const struct of_device_id apple_pmgr_ps_of_match[] = {
	{ .compatible = "apple,t8103-pmgr-pwrstate" },
	{ .compatible = "apple,pmgr-pwrstate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_ps_of_match);

static struct platform_driver apple_pmgr_ps_driver = {
	.probe = apple_pmgr_ps_probe,
	.driver = {
		.name = "apple-pmgr-pwrstate",
		.of_match_table = apple_pmgr_ps_of_match,
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR power state driver for Apple SoCs");

module_platform_driver(apple_pmgr_ps_driver);

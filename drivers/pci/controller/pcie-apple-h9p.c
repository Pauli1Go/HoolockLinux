// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple H9P/T8010 SoCs.
 *
 * The controller exposes an ECAM-compatible root complex after the SoC-specific
 * power, clock and PHY sequence has brought a port out of reset. The hardware
 * differs enough from the Apple Silicon PCIe controller to keep the early H9P
 * bring-up sequence separate, while still using the generic PCI host bridge
 * and MSI subsystems.
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sizes.h>

#include <linux/apple-ans.h>

#include "pci-host-common.h"

#define H9P_NUM_PORTS			4
#define H9P_NUM_MSI			32
#define H9P_MSI_PER_PORT		(H9P_NUM_MSI / H9P_NUM_PORTS)

#define H9P_CFG_PORT_STRIDE		0x8000
#define H9P_CFG_PORT_MISC		0x08e0

#define H9P_RC_COMMON_CTL0		0x0004
#define H9P_RC_COMMON_CTL1		0x0014
#define H9P_RC_COMMON_CTL2		0x0024
#define H9P_RC_COMMON_CTL3		0x0034
#define H9P_RC_COMMON_CTL_ENABLE	BIT(0)
#define H9P_RC_COMMON_CTL_INIT	BIT(4)
#define H9P_RC_PORT_STRIDE		0x0080
#define H9P_RC_PORTSTAT(port)		(0x0100 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL0(port)	(0x0100 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL1(port)	(0x0124 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_PORT_CTL2(port)	(0x0134 + (port) * H9P_RC_PORT_STRIDE)
#define H9P_RC_COMMON_STAT		0x0028
#define H9P_RC_COMMON_STAT_INIT_DONE	BIT(4)
#define H9P_RC_COMMON_STAT_READY	BIT(0)
#define H9P_RC_PORT_LINK_RATE(port)	(0x4020 + (port) * 0x0040)
#define H9P_PHY_PORTMASK		0x000c

#define H9P_PHY_IP_EQ_COMMON0		0x0180
#define H9P_PHY_IP_EQ_COMMON1		0x0184
#define H9P_PHY_IP_EQ_TIME0		0x0090
#define H9P_PHY_IP_EQ_TIME1		0x0098
#define H9P_PHY_IP_PORT_STRIDE		0x0800
#define H9P_PHY_IP_PORT(port, reg)	((reg) + (port) * H9P_PHY_IP_PORT_STRIDE)
#define H9P_PHY_IP_PORT_EQ_CTL		0x10088
#define H9P_PHY_IP_PORT_IDLE		0x10784
#define H9P_PHY_IP_PORT_EQ_PRESET	0x10004
#define H9P_PHY_IP_PORT_RX_CTL0		0x20788
#define H9P_PHY_IP_PORT_RX_CTL1		0x207a0
#define H9P_PHY_IP_PORT_RX_CTL2		0x207a8
#define H9P_PHY_IP_PORT_RX_CTL3		0x20400
#define H9P_PHY_IP_PORT_TIMER0		0x2009c
#define H9P_PHY_IP_PORT_TIMER1		0x200dc
#define H9P_PHY_IP_PORT_TIMER2		0x200a0
#define H9P_PHY_IP_PORT_TIMER3		0x200e0
#define H9P_PHY_IP_PORT_TIMER4		0x200a4
#define H9P_PHY_IP_PORT_TIMER5		0x200e4
#define H9P_PHY_IP_PORT_CLEAR0		0x20330
#define H9P_PHY_IP_PORT_CLEAR1		0x20340
#define H9P_PHY_IP_PORT_CLEAR2		0x20350

#define H9P_PORT_LTSSMCTL		0x0080
#define H9P_PORT_LTSSM_ENABLE		BIT(0)
#define H9P_PORT_IRQSTAT		0x0100
#define H9P_PORT_IRQMASK		0x0104
#define H9P_PORT_IRQMASK_PRE_LINK	0xff002fff
#define H9P_PORT_IRQSTAT_PRE_LINK	0x00ffd000
#define H9P_PORT_IRQMASK_LINK_UP	0xff002f0f
#define H9P_PORT_PWRCTL		0x0124
#define H9P_PORT_PWRCTL_INIT		0x31
#define H9P_PORT_MSIVECBASE		0x0128
#define H9P_PORT_ENABLE		0x0140
#define H9P_PORT_ENABLE_APPLE		BIT(31)
#define H9P_PORT_LINKSTS		0x0208
#define H9P_PORT_LINKSTS_LTSSM		GENMASK(13, 8)
#define H9P_PORT_LTSSM_DETECT		0x11
#define H9P_PORT_LTSSM_L0		0x14

#define H9P_LINK_SPEED_2_5GT		1
#define H9P_LINK_SPEED_8GT		3

#define H9P_NVMMU_TCB_CTRL		0x0004
#define H9P_NVMMU_TCB_BASE_LO		0x0008
#define H9P_NVMMU_TCB_BASE_HI		0x000c
#define H9P_NVMMU_TCB_TABLE_LO		0x0010
#define H9P_NVMMU_TCB_TABLE_HI		0x0014
#define H9P_NVMMU_SART_CTRL		0x0020
#define H9P_NVMMU_SART_VA_BASE		0x0024
#define H9P_NVMMU_SART_VA_END		0x0028
#define H9P_NVMMU_SART_PA_BASE		0x002c

#define H9P_NVMMU_TCB_BYTES		0x80
#define H9P_NVMMU_TCB_DWORDS		(H9P_NVMMU_TCB_BYTES / sizeof(u32))
#define H9P_NVMMU_SGL_WORDS		APPLE_ANS_NVMMU_MAX_PAGES
#define H9P_NVMMU_FLATDMA_BASE		0x40000000ULL
#define H9P_NVMMU_FLATDMA_STRIDE	SZ_8M
#define H9P_NVMMU_SART_ALIGNMENT	SZ_1M
#define H9P_NVMMU_TCB_READ		0x100
#define H9P_NVMMU_TCB_WRITE		0x200

#define H9P_DEFAULT_MSI_DOORBELL	0xbffff000ULL

struct apple_h9p_pcie;

struct apple_h9p_nvmmu {
	struct apple_h9p_pcie *pcie;
	void __iomem *base;
	u64 pa_base;
	u32 va_base;
	u32 size;
	void *tcb;
	void *tcb_table;
	void *tcb_sgl;
	size_t tcb_size;
	size_t tcb_table_size;
	size_t tcb_sgl_size;
	dma_addr_t tcb_dma;
	dma_addr_t tcb_table_dma;
	dma_addr_t tcb_sgl_dma;
};

struct apple_h9p_pcie {
	struct device *dev;
	struct platform_device *pdev;
	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;

	void __iomem *base_config;
	void __iomem *base_rc;
	void __iomem *base_phy;
	void __iomem *base_phy_ip;
	void __iomem *base_port[H9P_NUM_PORTS];

	struct clk_bulk_data clks[3];
	struct device_node *port_nodes[H9P_NUM_PORTS];
	struct gpio_desc *perst[H9P_NUM_PORTS];
	struct gpio_desc *clkreq[H9P_NUM_PORTS];
	struct gpio_descs *devpwr;
	struct pinctrl *pinctrl;
	u32 enabled_ports;

	struct apple_h9p_nvmmu nvmmu;

	struct device **pd_dev;
	struct device_link **pd_link;
	int pd_count;

	DECLARE_BITMAP(used_msi[H9P_NUM_PORTS], H9P_MSI_PER_PORT);
	u64 msi_doorbell;
	/* Protects the per-port MSI allocation bitmaps. */
	spinlock_t used_msi_lock;
	struct irq_domain *irq_dom;
	struct irq_fwspec msi_fwspec;
	u32 nvecs;
};

static inline void h9p_rmw(void __iomem *addr, u32 clear, u32 set)
{
	writel((readl(addr) & ~clear) | set, addr);
}

static inline void h9p_rmww(void __iomem *addr, u16 clear, u16 set)
{
	writew((readw(addr) & ~clear) | set, addr);
}

static inline void h9p_writel_flush(u32 value, void __iomem *addr)
{
	writel(value, addr);
	readl(addr);
}

static void apple_h9p_pcie_detach_genpd(struct apple_h9p_pcie *pcie)
{
	int i;

	for (i = pcie->pd_count - 1; i >= 0; i--) {
		if (pcie->pd_link[i])
			device_link_del(pcie->pd_link[i]);
		if (!IS_ERR_OR_NULL(pcie->pd_dev[i]))
			dev_pm_domain_detach(pcie->pd_dev[i], true);
	}
}

static int apple_h9p_pcie_attach_genpd(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int i;

	pcie->pd_count = of_count_phandle_with_args(dev->of_node,
						    "power-domains",
						    "#power-domain-cells");
	if (pcie->pd_count <= 1)
		return 0;

	pcie->pd_dev = devm_kcalloc(dev, pcie->pd_count,
				    sizeof(*pcie->pd_dev), GFP_KERNEL);
	if (!pcie->pd_dev)
		return -ENOMEM;

	pcie->pd_link = devm_kcalloc(dev, pcie->pd_count,
				     sizeof(*pcie->pd_link), GFP_KERNEL);
	if (!pcie->pd_link)
		return -ENOMEM;

	for (i = 0; i < pcie->pd_count; i++) {
		pcie->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(pcie->pd_dev[i])) {
			apple_h9p_pcie_detach_genpd(pcie);
			return PTR_ERR(pcie->pd_dev[i]);
		}

		pcie->pd_link[i] = device_link_add(dev, pcie->pd_dev[i],
						   DL_FLAG_STATELESS |
						   DL_FLAG_PM_RUNTIME |
						   DL_FLAG_RPM_ACTIVE);
		if (!pcie->pd_link[i]) {
			apple_h9p_pcie_detach_genpd(pcie);
			return -EINVAL;
		}
	}

	return 0;
}

static void apple_h9p_pcie_genpd_cleanup(void *data)
{
	apple_h9p_pcie_detach_genpd(data);
}

static void apple_h9p_pcie_clk_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	clk_bulk_disable_unprepare(ARRAY_SIZE(pcie->clks), pcie->clks);
}

static struct apple_h9p_pcie *apple_h9p_pcie_lookup(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	return bridge ? pci_host_bridge_priv(bridge) : NULL;
}

static int apple_h9p_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_h9p_pcie *pcie;

	if (bus->number == cfg->busr.start) {
		pcie = apple_h9p_pcie_lookup(cfg->parent);
		if (!pcie || PCI_SLOT(devfn) >= H9P_NUM_PORTS ||
		    !(pcie->enabled_ports & BIT(PCI_SLOT(devfn))))
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	return pci_generic_config_read(bus, devfn, where, size, val);
}

static int apple_h9p_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_h9p_pcie *pcie;

	if (bus->number == cfg->busr.start) {
		pcie = apple_h9p_pcie_lookup(cfg->parent);
		if (!pcie || PCI_SLOT(devfn) >= H9P_NUM_PORTS ||
		    !(pcie->enabled_ports & BIT(PCI_SLOT(devfn))))
			return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (where <= PCI_INTERRUPT_LINE && where + size > PCI_INTERRUPT_LINE)
		val |= 0xffu << ((PCI_INTERRUPT_LINE - where) << 3);

	return pci_generic_config_write(bus, devfn, where, size, val);
}

static unsigned int apple_h9p_pcie_bus_to_port(struct apple_h9p_pcie *pcie,
					       unsigned int bus)
{
	unsigned int port;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		u32 cfg, sec, sub;

		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		cfg = readl(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
			    PCI_PRIMARY_BUS);
		sec = (cfg >> 8) & 0xff;
		sub = (cfg >> 16) & 0xff;

		if (!sec || !sub || sec == 0xff || sub == 0xff)
			continue;
		if (bus >= sec && bus <= sub)
			return port;
	}

	return H9P_NUM_PORTS;
}

static int apple_h9p_pcie_device_port(struct apple_h9p_pcie *pcie,
				      struct device *dev)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(dev))
		return -ENODEV;

	pdev = to_pci_dev(dev);
	if (!pdev->bus)
		return -ENODEV;

	return apple_h9p_pcie_bus_to_port(pcie, pdev->bus->number);
}

static void apple_h9p_msi_compose_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct apple_h9p_pcie *pcie = irq_data_get_irq_chip_data(d);

	msg->address_lo = lower_32_bits(pcie->msi_doorbell);
	msg->address_hi = upper_32_bits(pcie->msi_doorbell);
	msg->data = d->hwirq;
}

static struct irq_chip apple_h9p_msi_bottom_chip = {
	.name = "Apple H9P PCIe MSI",
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_eoi = irq_chip_eoi_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_compose_msi_msg = apple_h9p_msi_compose_msg,
};

static int apple_h9p_msi_alloc(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs, void *args)
{
	struct apple_h9p_pcie *pcie = domain->host_data;
	struct irq_fwspec fwspec = pcie->msi_fwspec;
	msi_alloc_info_t *info = args;
	struct msi_desc *desc = info ? info->desc : NULL;
	struct device *msi_dev = desc ? msi_desc_to_dev(desc) : NULL;
	struct pci_dev *pdev;
	unsigned long flags;
	unsigned int port;
	int ret, slot;

	if (nr_irqs != 1 || !msi_dev || !dev_is_pci(msi_dev))
		return -ENOSPC;

	pdev = to_pci_dev(msi_dev);
	if (!pdev->bus)
		return -ENOSPC;

	port = apple_h9p_pcie_bus_to_port(pcie, pdev->bus->number);
	if (port >= H9P_NUM_PORTS)
		return -ENOSPC;
	if (!(pcie->enabled_ports & BIT(port)))
		return -ENOSPC;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	slot = find_first_zero_bit(pcie->used_msi[port], H9P_MSI_PER_PORT);
	if (slot >= H9P_MSI_PER_PORT) {
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return -ENOSPC;
	}
	__set_bit(slot, pcie->used_msi[port]);
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);

	fwspec.param[fwspec.param_count - 2] +=
		port * H9P_MSI_PER_PORT + slot;
	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (ret) {
		spin_lock_irqsave(&pcie->used_msi_lock, flags);
		__clear_bit(slot, pcie->used_msi[port]);
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return ret;
	}

	irq_domain_set_hwirq_and_chip(domain, virq,
				      port * H9P_MSI_PER_PORT + slot,
				      &apple_h9p_msi_bottom_chip, pcie);
	return 0;
}

static void apple_h9p_msi_free(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_h9p_pcie *pcie = d ? irq_data_get_irq_chip_data(d) : NULL;
	irq_hw_number_t hwirq;
	unsigned long flags;
	unsigned int i;

	if (!pcie || !d)
		return;

	hwirq = d->hwirq;
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	for (i = 0; i < nr_irqs; i++) {
		irq_hw_number_t irq = hwirq + i;
		unsigned int port = irq / H9P_MSI_PER_PORT;
		unsigned int slot = irq % H9P_MSI_PER_PORT;

		if (port < H9P_NUM_PORTS)
			__clear_bit(slot, pcie->used_msi[port]);
	}
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops apple_h9p_msi_domain_ops = {
	.alloc = apple_h9p_msi_alloc,
	.free = apple_h9p_msi_free,
};

static const struct msi_parent_ops apple_h9p_msi_parent_ops = {
	.supported_flags = MSI_GENERIC_FLAGS_MASK | MSI_FLAG_PCI_MSIX |
			   MSI_FLAG_MULTI_PCI_MSI,
	.required_flags = MSI_FLAG_USE_DEF_DOM_OPS |
			  MSI_FLAG_USE_DEF_CHIP_OPS |
			  MSI_FLAG_PCI_MSI_MASK_PARENT,
	.chip_flags = MSI_CHIP_FLAG_SET_EOI,
	.bus_select_token = DOMAIN_BUS_PCI_MSI,
	.init_dev_msi_info = msi_lib_init_dev_msi_info,
};

static int apple_h9p_pcie_setup_msi(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct irq_domain_info info = {
		.fwnode = fwnode,
		.ops = &apple_h9p_msi_domain_ops,
		.size = H9P_NUM_MSI,
		.host_data = pcie,
	};
	struct of_phandle_args args = {};
	int ret;

	ret = of_parse_phandle_with_args(to_of_node(fwnode), "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(to_of_node(fwnode), "msi-ranges",
					 args.args_count + 1, &pcie->nvecs);
	if (ret)
		goto out_put_node;
	if (pcie->nvecs != H9P_NUM_MSI) {
		ret = -EINVAL;
		goto out_put_node;
	}

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &pcie->msi_fwspec);
	info.parent = irq_find_matching_fwspec(&pcie->msi_fwspec,
					       DOMAIN_BUS_WIRED);
	if (!info.parent) {
		ret = -ENXIO;
		goto out_put_node;
	}

	pcie->irq_dom = msi_create_parent_irq_domain(&info,
						     &apple_h9p_msi_parent_ops);
	if (!pcie->irq_dom)
		ret = -ENOMEM;

out_put_node:
	of_node_put(args.np);
	return ret;
}

static void apple_h9p_pcie_msi_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	irq_domain_remove(pcie->irq_dom);
}

static u64 apple_h9p_read_pci_cap(struct apple_h9p_pcie *pcie,
				  unsigned int busdevfn, u32 type)
{
	void __iomem *cfg = pcie->base_config + (busdevfn << 12);
	u32 ptr = readl(cfg + PCI_CAPABILITY_LIST) & 0xff;
	unsigned int ttl = 48;

	while (ptr && ttl--) {
		u32 next = readl(cfg + ptr);

		if (ptr < PCI_STD_HEADER_SIZEOF || ptr > 0xfc || (ptr & 3))
			break;
		if ((next & 0xff) == type)
			return ptr;
		ptr = (next >> 8) & 0xff;
	}

	return 0;
}

static int apple_h9p_wait(void __iomem *addr, u32 mask, u32 min, u32 max,
			  unsigned long timeout_us)
{
	u32 val;

	return readl_poll_timeout(addr, val, (val & mask) >= min &&
				  (val & mask) <= max, 1000, timeout_us);
}

static int apple_h9p_wait_gpio(struct gpio_desc *desc, int value,
			       unsigned long timeout_us)
{
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us);
	int ret;

	do {
		ret = gpiod_get_value_cansleep(desc);
		if (ret < 0)
			return ret;
		if (ret == value)
			return 0;
		usleep_range(1000, 2000);
	} while (ktime_before(ktime_get(), timeout));

	return -ETIMEDOUT;
}

static irqreturn_t apple_h9p_nvmmu_irq(int irq, void *data)
{
	struct apple_h9p_nvmmu *nvmmu = data;
	struct apple_h9p_pcie *pcie = nvmmu->pcie;

	dev_err_ratelimited(pcie->dev, "storage-port NVMMU fault interrupt\n");
	return IRQ_HANDLED;
}

static int apple_h9p_setup_nvmmu(struct apple_h9p_pcie *pcie)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu;
	struct device *dev = pcie->dev;
	struct device_node *mem_np;
	struct resource res;
	resource_size_t size;
	u32 iova;
	int irq;
	int ret;

	if (!(pcie->enabled_ports & BIT(0)) || !nvmmu->base)
		return 0;

	mem_np = of_parse_phandle(dev->of_node, "apple,nvme-hmb", 0);
	if (!mem_np)
		return dev_err_probe(dev, -EINVAL,
				     "NVMMU is missing apple,nvme-hmb\n");

	ret = of_address_to_resource(mem_np, 0, &res);
	if (ret)
		goto out_put_node;

	ret = of_property_read_u32(dev->of_node, "apple,nvmmu-iova", &iova);
	if (ret)
		goto out_put_node;

	size = resource_size(&res);
	if (size < H9P_NVMMU_SART_ALIGNMENT || size > U32_MAX ||
	    !IS_ALIGNED(res.start, H9P_NVMMU_SART_ALIGNMENT) ||
	    !IS_ALIGNED(iova, H9P_NVMMU_SART_ALIGNMENT) ||
	    iova < 0x80000000U ||
	    iova > U32_MAX - (H9P_NVMMU_SART_ALIGNMENT - 1) ||
	    size > U32_MAX - iova - (H9P_NVMMU_SART_ALIGNMENT - 1)) {
		ret = -EINVAL;
		goto out_put_node;
	}

	nvmmu->pcie = pcie;
	nvmmu->pa_base = res.start;
	nvmmu->va_base = iova;
	nvmmu->size = size;
	nvmmu->tcb_size = round_up(APPLE_ANS_NVMMU_MAX_REQS *
				   H9P_NVMMU_TCB_BYTES, PAGE_SIZE);
	nvmmu->tcb_table_size = PAGE_SIZE * 16;
	nvmmu->tcb_sgl_size = round_up(APPLE_ANS_NVMMU_MAX_REQS *
				       H9P_NVMMU_SGL_WORDS * sizeof(u32),
				       PAGE_SIZE);

	nvmmu->tcb = dmam_alloc_attrs(dev, nvmmu->tcb_size, &nvmmu->tcb_dma,
				      GFP_KERNEL | __GFP_ZERO,
				      DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	nvmmu->tcb_table = dmam_alloc_attrs(dev, nvmmu->tcb_table_size,
					    &nvmmu->tcb_table_dma,
					    GFP_KERNEL | __GFP_ZERO,
					    DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_table) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	nvmmu->tcb_sgl = dmam_alloc_attrs(dev, nvmmu->tcb_sgl_size,
					  &nvmmu->tcb_sgl_dma,
					  GFP_KERNEL | __GFP_ZERO,
					  DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_sgl) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	h9p_writel_flush(lower_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_HI);
	h9p_writel_flush(lower_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_HI);
	h9p_writel_flush(0x10000, nvmmu->base + H9P_NVMMU_TCB_CTRL);

	ret = apple_h9p_wait(nvmmu->base + H9P_NVMMU_TCB_CTRL, 0x10, 0, 0,
			     250000);
	if (ret)
		goto out_put_node;

	h9p_writel_flush(nvmmu->va_base - 0x80000000U,
			 nvmmu->base + H9P_NVMMU_SART_VA_BASE);
	h9p_writel_flush(round_up(nvmmu->va_base + nvmmu->size,
				  H9P_NVMMU_SART_ALIGNMENT) - 0x80100000U,
			 nvmmu->base + H9P_NVMMU_SART_VA_END);
	h9p_writel_flush(nvmmu->pa_base >> 20,
			 nvmmu->base + H9P_NVMMU_SART_PA_BASE);
	h9p_writel_flush(1, nvmmu->base + H9P_NVMMU_SART_CTRL);

	irq = platform_get_irq_byname_optional(pcie->pdev, "nvmmu");
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, apple_h9p_nvmmu_irq, 0,
				       dev_name(dev), nvmmu);
		if (ret)
			goto out_put_node;
	} else if (irq != -ENXIO) {
		ret = irq;
		goto out_put_node;
	}

	dev_dbg(dev, "storage-port NVMMU window %#x@%pa size %#x\n",
		nvmmu->va_base, &res.start, nvmmu->size);

out_put_node:
	of_node_put(mem_np);
	return ret;
}

int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
			const u64 *pages, unsigned int npages,
			dma_addr_t *iova)
{
	struct apple_h9p_nvmmu *nvmmu;
	struct apple_h9p_pcie *pcie;
	struct device *host_dev = dev;
	unsigned int port;
	unsigned int i;
	u64 sgl_dma;
	u32 *tcb;
	u32 *sgl;
	int ret;

	if (tag >= APPLE_ANS_NVMMU_MAX_REQS ||
	    npages > APPLE_ANS_NVMMU_MAX_PAGES)
		return -EINVAL;
	if (npages && !pages)
		return -EINVAL;

	while (host_dev && host_dev->bus == dev->bus)
		host_dev = host_dev->parent;
	if (!host_dev || !host_dev->parent)
		return -ENODEV;

	pcie = apple_h9p_pcie_lookup(host_dev->parent);
	if (!pcie)
		return -ENODEV;

	ret = apple_h9p_pcie_device_port(pcie, dev);
	if (ret < 0)
		return ret;
	port = ret;
	if (port != 0 || !(pcie->enabled_ports & BIT(0)))
		return -ENODEV;

	nvmmu = &pcie->nvmmu;
	if (!nvmmu->base || !nvmmu->tcb || !nvmmu->tcb_sgl)
		return -EOPNOTSUPP;

	tcb = (u32 *)nvmmu->tcb + tag * H9P_NVMMU_TCB_DWORDS;
	sgl = (u32 *)nvmmu->tcb_sgl + tag * H9P_NVMMU_SGL_WORDS;
	memset(tcb, 0, H9P_NVMMU_TCB_BYTES);
	memset(sgl, 0, H9P_NVMMU_SGL_WORDS * sizeof(*sgl));

	if (npages) {
		tcb[0] = H9P_NVMMU_TCB_READ | H9P_NVMMU_TCB_WRITE;
		tcb[1] = npages;
		tcb[2] = pages[0] >> ilog2(APPLE_ANS_NVMMU_PAGE_SIZE);
		for (i = 0; i < npages; i++)
			sgl[i] = pages[i] >> ilog2(APPLE_ANS_NVMMU_PAGE_SIZE);

		sgl_dma = nvmmu->tcb_sgl_dma +
			  tag * H9P_NVMMU_SGL_WORDS * sizeof(*sgl);
		memcpy(&tcb[4], &sgl_dma, sizeof(sgl_dma));
		if (iova)
			*iova = H9P_NVMMU_FLATDMA_BASE +
				tag * H9P_NVMMU_FLATDMA_STRIDE;
	} else {
		dma_wmb();
		h9p_writel_flush(tag, nvmmu->base + H9P_NVMMU_TCB_CTRL);
		if (iova)
			*iova = 0;
		return 0;
	}

	dma_wmb();
	return 0;
}
EXPORT_SYMBOL_GPL(apple_ans_nvmmu_map);

static bool apple_h9p_link_up(struct apple_h9p_pcie *pcie, unsigned int port)
{
	u32 linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);

	linksts = FIELD_GET(H9P_PORT_LINKSTS_LTSSM, linksts);
	return linksts >= H9P_PORT_LTSSM_DETECT && linksts <= H9P_PORT_LTSSM_L0;
}

static int apple_h9p_setup_port(struct apple_h9p_pcie *pcie, unsigned int port)
{
	struct device *dev = pcie->dev;
	u64 cap;
	int ret;

	if (apple_h9p_link_up(pcie, port))
		return 0;

	ret = gpiod_direction_output(pcie->perst[port], 1);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assert port %u PERST#\n", port);

	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 1, 0);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0, 1);

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u init timeout\n", port);

	usleep_range(250, 1000);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0, 1);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0x100, 0);
	usleep_range(500, 1000);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 0, 1);

	writel(port ? 0 : H9P_LINK_SPEED_8GT,
	       pcie->base_rc + H9P_RC_PORT_LINK_RATE(port));
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0x100, 0);

	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (cap)
		h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
			 cap + PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS,
			 port ? H9P_LINK_SPEED_2_5GT : H9P_LINK_SPEED_8GT);

	h9p_rmw(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
		H9P_CFG_PORT_MISC, 0, 1);

	writel(H9P_PORT_IRQMASK_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	writel(H9P_PORT_IRQSTAT_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQSTAT);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_ENABLE, 0,
		H9P_PORT_ENABLE_APPLE);
	writel(H9P_PORT_PWRCTL_INIT, pcie->base_port[port] + H9P_PORT_PWRCTL);
	writel(port * 0x10001 * H9P_MSI_PER_PORT,
	       pcie->base_port[port] + H9P_PORT_MSIVECBASE);

	usleep_range(250, 1000);
	ret = apple_h9p_wait_gpio(pcie->clkreq[port], 1, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u CLKREQ# timeout\n",
				     port);

	ret = gpiod_direction_output(pcie->perst[port], 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert port %u PERST#\n", port);
	usleep_range(250, 1000);

	ret = apple_h9p_wait(pcie->base_phy + H9P_PHY_PORTMASK,
			     BIT(port), BIT(port), BIT(port), 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u PHY up timeout\n",
				     port);

	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON0, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON1, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME0, 0xfff, 100);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME1, 0xfff, 25);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_EQ_CTL),
		0, 0x4000);
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_IDLE));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_EQ_PRESET), 0xfff, 0x600);
	writel(0x3105, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL0));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL1), 0xff, 0x9f);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL2), 0xff, 0x01);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_RX_CTL3), 0x1f, 0x0a);
	writel(175, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER0));
	writel(175, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER1));
	writel(333, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER2));
	writel(333, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER3));
	writel(530, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER4));
	writel(530, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_TIMER5));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR0));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR1));
	writel(0, pcie->base_phy_ip + H9P_PHY_IP_PORT(port, H9P_PHY_IP_PORT_CLEAR2));

	writel(H9P_PORT_IRQMASK_LINK_UP,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	usleep_range(5000, 10000);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_LTSSMCTL, 0,
		H9P_PORT_LTSSM_ENABLE);
	ret = apple_h9p_wait(pcie->base_port[port] + H9P_PORT_LINKSTS,
			     H9P_PORT_LINKSTS_LTSSM,
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_DETECT),
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_L0),
			     500000);
	if (ret)
		dev_warn(dev, "port %u link did not reach L0\n", port);

	return 0;
}

static int apple_h9p_setup_ports(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	writel(H9P_RC_COMMON_CTL_INIT,
	       pcie->base_rc + H9P_RC_COMMON_CTL0);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(0), 0,
		H9P_RC_COMMON_CTL_ENABLE);

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY init timeout\n");

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_READY,
			     H9P_RC_COMMON_STAT_READY,
			     H9P_RC_COMMON_STAT_READY, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY ready timeout\n");

	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL3);
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL1);
	usleep_range(5000, 10000);
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL2);
	usleep_range(500, 1000);

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		ret = apple_h9p_setup_port(pcie, port);
		if (ret)
			return ret;
	}

	return 0;
}

static int apple_h9p_pcie_init(struct pci_config_window *cfg)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(cfg->parent);
	int ret;

	if (!pcie)
		return -ENODEV;

	pcie->cfgwin = cfg;
	pcie->base_config = cfg->win;

	ret = apple_h9p_setup_ports(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_setup_nvmmu(pcie);
	return ret;
}

static const struct pci_ecam_ops apple_h9p_pcie_ecam_ops = {
	.bus_shift = 20,
	.init = apple_h9p_pcie_init,
	.pci_ops = {
		.map_bus = pci_ecam_map_bus,
		.read = apple_h9p_pcie_config_read,
		.write = apple_h9p_pcie_config_write,
	},
};

static int apple_h9p_pcie_map_resources(struct platform_device *pdev,
					struct apple_h9p_pcie *pcie)
{
	unsigned int i;

	pcie->base_rc = devm_platform_ioremap_resource_byname(pdev, "rc");
	if (IS_ERR(pcie->base_rc))
		return PTR_ERR(pcie->base_rc);

	pcie->base_phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(pcie->base_phy))
		return PTR_ERR(pcie->base_phy);

	pcie->base_phy_ip =
		devm_platform_ioremap_resource_byname(pdev, "phy-ip");
	if (IS_ERR(pcie->base_phy_ip))
		return PTR_ERR(pcie->base_phy_ip);

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		char name[8];

		if (!(pcie->enabled_ports & BIT(i)))
			continue;

		snprintf(name, sizeof(name), "port%u", i);
		pcie->base_port[i] = devm_platform_ioremap_resource_byname(pdev,
									   name);
		if (IS_ERR(pcie->base_port[i]))
			return PTR_ERR(pcie->base_port[i]);
	}

	pcie->nvmmu.base =
		devm_platform_ioremap_resource_byname(pdev, "nvmmu");
	if (IS_ERR(pcie->nvmmu.base))
		return PTR_ERR(pcie->nvmmu.base);

	return 0;
}

static void apple_h9p_pcie_put_port_nodes(void *data)
{
	struct apple_h9p_pcie *pcie = data;
	unsigned int i;

	for (i = 0; i < H9P_NUM_PORTS; i++)
		of_node_put(pcie->port_nodes[i]);
}

static int apple_h9p_pcie_parse_ports(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *np;
	u32 reg, port;
	int ret;

	for_each_available_child_of_node(dev->of_node, np) {
		if (!of_node_name_eq(np, "pci"))
			continue;

		ret = of_property_read_u32_index(np, "reg", 0, &reg);
		if (ret)
			goto out_put_child;

		port = reg >> 11;
		if (port >= H9P_NUM_PORTS || pcie->port_nodes[port]) {
			ret = -EINVAL;
			goto out_put_child;
		}

		pcie->port_nodes[port] = of_node_get(np);
		pcie->enabled_ports |= BIT(port);
	}

	if (!pcie->enabled_ports)
		return -ENODEV;

	return devm_add_action_or_reset(dev, apple_h9p_pcie_put_port_nodes,
					pcie);

out_put_child:
	of_node_put(np);
	apple_h9p_pcie_put_port_nodes(pcie);
	return ret;
}

static int apple_h9p_pcie_get_gpios(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned int i;

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		struct fwnode_handle *fwnode;
		struct gpio_desc *gpio;

		if (!(pcie->enabled_ports & BIT(i)))
			continue;

		fwnode = of_fwnode_handle(pcie->port_nodes[i]);
		gpio = devm_fwnode_gpiod_get(dev, fwnode, "reset",
					     GPIOD_OUT_HIGH, "PERST#");
		pcie->perst[i] = gpio;
		if (IS_ERR(pcie->perst[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->perst[i]),
					     "failed to get PERST#%u\n", i);

		gpio = devm_fwnode_gpiod_get(dev, fwnode, "clkreq", GPIOD_IN, "CLKREQ#");
		pcie->clkreq[i] = gpio;
		if (IS_ERR(pcie->clkreq[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->clkreq[i]),
					     "failed to get CLKREQ#%u\n", i);
	}

	pcie->devpwr = devm_gpiod_get_array_optional(dev, "devpwr", GPIOD_ASIS);
	if (IS_ERR(pcie->devpwr))
		return dev_err_probe(dev, PTR_ERR(pcie->devpwr),
				     "failed to get device power GPIOs\n");

	return 0;
}

static int apple_h9p_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct apple_h9p_pcie *pcie;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;
	pcie->pdev = pdev;
	pcie->bridge = bridge;
	spin_lock_init(&pcie->used_msi_lock);

	ret = apple_h9p_pcie_parse_ports(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse root ports\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	ret = apple_h9p_pcie_attach_genpd(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach power domains\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_genpd_cleanup, pcie);
	if (ret)
		return ret;

	pcie->clks[0].id = "core";
	pcie->clks[1].id = "aux";
	pcie->clks[2].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_clk_cleanup, pcie);
	if (ret)
		return ret;

	pcie->pinctrl = devm_pinctrl_get_select_default(dev);
	if (PTR_ERR(pcie->pinctrl) == -ENODEV)
		pcie->pinctrl = NULL;
	else if (IS_ERR(pcie->pinctrl))
		return dev_err_probe(dev, PTR_ERR(pcie->pinctrl),
				     "failed to select pinctrl state\n");

	ret = apple_h9p_pcie_map_resources(pdev, pcie);
	if (ret)
		return ret;

	ret = apple_h9p_pcie_get_gpios(pcie);
	if (ret)
		return ret;

	ret = of_property_read_u64(dev->of_node, "apple,msi-doorbell",
				   &pcie->msi_doorbell);
	if (ret)
		pcie->msi_doorbell = H9P_DEFAULT_MSI_DOORBELL;

	ret = apple_h9p_pcie_setup_msi(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up MSI\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_msi_cleanup, pcie);
	if (ret)
		return ret;

	return pci_host_common_init(pdev, bridge, &apple_h9p_pcie_ecam_ops);
}

static const struct of_device_id apple_h9p_pcie_of_match[] = {
	{ .compatible = "apple,t8010-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_h9p_pcie_of_match);

static struct platform_driver apple_h9p_pcie_driver = {
	.probe = apple_h9p_pcie_probe,
	.driver = {
		.name = "pcie-apple-h9p",
		.of_match_table = apple_h9p_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(apple_h9p_pcie_driver);

MODULE_DESCRIPTION("Apple H9P/T8010 PCIe host bridge driver");
MODULE_LICENSE("GPL");

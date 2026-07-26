// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple H9P/T8010 SoCs.
 *
 * The controller exposes an ECAM-compatible root complex after the SoC-specific
 * power, clock and PHY sequence has brought a port out of reset. The hardware
 * differs enough from the Apple Silicon PCIe controller to keep the H9P
 * initialization sequence separate, while still using the generic PCI bridge
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
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pci-pwrctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <linux/apple-dart.h>
#include <linux/apple-ans.h>

#include "pci-host-common.h"

#define H9P_NUM_PORTS			4
#define H9P_NUM_MSI			32
#define H9P_MSI_PER_PORT		(H9P_NUM_MSI / H9P_NUM_PORTS)
#define H9P_NUM_CLOCKS			2

#define H9P_CFG_PORT_STRIDE		0x8000
#define H9P_CFG_PORT_MISC		0x08e0
#define H9P_CFG_AER_CAP			0x0100
#define H9P_BRIDGE_HEADER_FIRST		PCI_REVISION_ID
#define H9P_BRIDGE_HEADER_LAST		PCI_INTERRUPT_LINE
#define H9P_BRIDGE_HEADER_DWORDS		\
	((H9P_BRIDGE_HEADER_LAST - H9P_BRIDGE_HEADER_FIRST) / sizeof(u32) + 1)
#define H9P_BRIDGE_WINDOW_DISABLED	0x0000fff0
#define H9P_PORT_LINK_EVENT		BIT(12)
#define H9P_J172_QUIRK_CFG_FIRST		PCI_BASE_ADDRESS_0
#define H9P_J172_QUIRK_CFG_LAST		PCI_ROM_ADDRESS
#define H9P_J172_QUIRK_CFG_DWORDS	\
	((H9P_J172_QUIRK_CFG_LAST - H9P_J172_QUIRK_CFG_FIRST) / \
	 sizeof(u32) + 1)
#define H9P_ROOT_PORT_DEVCTL		(PCI_EXP_DEVCTL_CERE | \
					 PCI_EXP_DEVCTL_NFERE | \
					 PCI_EXP_DEVCTL_FERE | \
					 PCI_EXP_DEVCTL_URRE | \
					 PCI_EXP_DEVCTL_RELAX_EN | \
					 PCI_EXP_DEVCTL_PAYLOAD_256B | \
					 PCI_EXP_DEVCTL_READRQ_512B)

#define H9P_RC_COMMON_CTL0		0x0004
#define H9P_RC_COMMON_CTL1		0x0014
#define H9P_RC_COMMON_CTL2		0x0024
#define H9P_RC_COMMON_CTL3		0x0034
#define H9P_RC_COMMON_CTL_ENABLE	BIT(0)
#define H9P_RC_COMMON_CTL_INIT	BIT(4)
#define H9P_RC_PORT_STRIDE		0x0080
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
#define H9P_PORT_IRQMASK_ALL		GENMASK(31, 0)
#define H9P_PORT_IRQMASK_PRE_LINK	0xff002fff
#define H9P_PORT_IRQSTAT_PRE_LINK	0x00ffd000
#define H9P_PORT_IRQMASK_LINK_UP	0xff002f0f
#define H9P_PORT_PWRCTL		0x0124
#define H9P_PORT_PWRCTL_INIT		0x31
#define H9P_PORT_MSIVECBASE		0x0128
#define H9P_PORT_ENABLE		0x0140
#define H9P_PORT_ENABLE_APPLE		BIT(31)
#define H9P_PORT_LINKSTS		0x0208
#define H9P_PORT_LINKSTS_HW_STATE	GENMASK(31, 24)
#define H9P_PORT_LINKSTS_HW_READY	0xab
#define H9P_PORT_LINKSTS_LTSSM		GENMASK(13, 8)
#define H9P_PORT_LINKSTS_LINK_PRESENT	BIT(3)
#define H9P_PORT_LTSSM_L0		0x11
#define H9P_PORT_LTSSM_L1_IDLE		0x14
#define H9P_J172_QUIRK_FIRST_SETTLE_MIN_US	52000
#define H9P_J172_QUIRK_FIRST_SETTLE_MAX_US	55000
#define H9P_J172_QUIRK_ACK_SETTLE_MIN_US		550
#define H9P_J172_QUIRK_ACK_SETTLE_MAX_US		650
#define H9P_J172_QUIRK_PREFIX_SETTLE_MIN_US	6400
#define H9P_J172_QUIRK_PREFIX_SETTLE_MAX_US	7000

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

struct apple_h9p_tunable {
	u32 offset;
	u32 mask;
	u32 value;
};

/* Serialized by T8010 boot firmware as apcie-common-tunables. */
static const struct apple_h9p_tunable h9p_rc_common_tunables[] = {
	{ 0x0008, 0x7f7f7f7f, 0x00000000 },
	{ 0x000c, 0x00073f3f, 0x00043f00 },
	{ 0x0010, 0x00000700, 0x00000000 },
	{ 0x0018, 0x00ffffff, 0x000c0960 },
	{ 0x001c, 0x00001fff, 0x0000092c },
	{ 0x002c, 0x000000ff, 0x00000009 },
	{ 0x003c, 0x80000000, 0x00000000 },
	{ 0x0100, 0x31100010, 0x01000000 },
	{ 0x0108, 0x00000707, 0x00000000 },
	{ 0x010c, 0x00073f3f, 0x00043f00 },
	{ 0x0110, 0x00000011, 0x00000001 },
	{ 0x0114, 0x00000007, 0x00000000 },
	{ 0x0118, 0x00073f3f, 0x00043f00 },
	{ 0x0120, 0x0333003f, 0x0111000f },
	{ 0x0130, 0x000000ff, 0x0000000f },
	{ 0x0138, 0x0000007f, 0x0000003e },
	{ 0x0180, 0x31100010, 0x01000000 },
	{ 0x0188, 0x00000707, 0x00000000 },
	{ 0x018c, 0x00073f3f, 0x00043f00 },
	{ 0x01a0, 0x0333003f, 0x0111000f },
	{ 0x01b0, 0x000000ff, 0x0000000f },
	{ 0x01b8, 0x0000007f, 0x0000003e },
	{ 0x0200, 0x31100010, 0x01000000 },
	{ 0x0208, 0x00000707, 0x00000000 },
	{ 0x020c, 0x00073f3f, 0x00043f00 },
	{ 0x0220, 0x0333003f, 0x0111000f },
	{ 0x0230, 0x000000ff, 0x0000000f },
	{ 0x0238, 0x0000007f, 0x0000003e },
	{ 0x0280, 0x31100010, 0x01000000 },
	{ 0x0288, 0x00000707, 0x00000000 },
	{ 0x028c, 0x00073f3f, 0x00043f00 },
	{ 0x02a0, 0x0333003f, 0x0111000f },
	{ 0x02b0, 0x000000ff, 0x0000000f },
	{ 0x02b8, 0x0000007f, 0x0000003e },
	{ 0x0100, 0x00000010, 0x00000010 },
	{ 0x0180, 0x00000010, 0x00000000 },
	{ 0x0200, 0x00000010, 0x00000000 },
	{ 0x0280, 0x00000010, 0x00000000 },
};

/* Serialized per root port as pcie-rc-tunables. */
static const struct apple_h9p_tunable h9p_config_tunables[] = {
	{ 0x0098, 0x0000000f, 0x00000000 },
	{ 0x0164, 0x00f8ff00, 0x00000000 },
	{ 0x08e0, 0x00000005, 0x00000005 },
};

/* Serialized per root port as apcie-config-tunables. */
static const struct apple_h9p_tunable h9p_port_tunables[] = {
	{ 0x0090, 0x000000ff, 0x00000028 },
	{ 0x0130, 0x0000000d, 0x00000005 },
	{ 0x0134, 0x00000001, 0x00000001 },
	{ 0x0138, 0x00007f7f, 0x00000000 },
	{ 0x013c, 0x00000002, 0x00000002 },
	{ 0x0140, 0x0073ffff, 0x00704c4b },
};

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

struct apple_h9p_port_irq {
	struct apple_h9p_pcie *pcie;
	unsigned int index;
	int irq;
	bool enabled;
};

struct apple_h9p_j172_bridge_quirk_state;

struct apple_h9p_pcie {
	struct device *dev;
	struct platform_device *pdev;
	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;
	struct apple_h9p_j172_bridge_quirk_state *j172_quirk_state;

	void __iomem *base_config;
	void __iomem *base_rc;
	void __iomem *base_phy;
	void __iomem *base_phy_ip;
	void __iomem *base_port[H9P_NUM_PORTS];

	struct clk_bulk_data clks[H9P_NUM_CLOCKS];
	unsigned int clks_enabled;
	struct device_node *port_nodes[H9P_NUM_PORTS];
	struct device_node *manual_dart[H9P_NUM_PORTS];
	u32 manual_dart_sid[H9P_NUM_PORTS];
	struct gpio_desc *perst[H9P_NUM_PORTS];
	struct gpio_desc *clkreq[H9P_NUM_PORTS];
	struct apple_h9p_port_irq port_irq[H9P_NUM_PORTS];
	u8 phy_lane[H9P_NUM_PORTS];
	u8 max_link_speed[H9P_NUM_PORTS];
	u16 pcie_cap[H9P_NUM_PORTS];
	struct pinctrl *pinctrl;
	u32 enabled_ports;
	u32 preinit_phy_ports;
	u32 link_retry_ports;
	u32 hardware_trained_phy_ports;
	u32 deferred_power_ports;
	u32 inherited_perst_ports;
	u32 inherited_perst_pending;
	bool pwrctrl_powered;
	bool j172_bridge_quirk;

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

struct apple_h9p_j172_bridge_quirk_state {
	u32 root[3][H9P_J172_QUIRK_CFG_DWORDS];
	u32 root_aer_uncor_mask[3];
	u32 root_aer_cor_mask[3];
	u32 root_devctlsts[3];
	u32 nvme[H9P_J172_QUIRK_CFG_DWORDS];
	u32 rc_port_ctl1;
	bool nvme_valid;
};

enum apple_h9p_port_result {
	APPLE_H9P_PORT_READY,
	APPLE_H9P_PORT_LINK_FAILED,
	APPLE_H9P_PORT_DISABLED,
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

static void apple_h9p_apply_tunables(void __iomem *base,
				     const struct apple_h9p_tunable *tunables,
				     unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		void __iomem *addr = base + tunables[i].offset;
		u32 value = readl(addr);

		if ((value & tunables[i].mask) == tunables[i].value)
			continue;

		writel((value & ~tunables[i].mask) | tunables[i].value, addr);
	}
}

static void apple_h9p_save_bridge_header(void __iomem *config, u32 *header)
{
	unsigned int i;

	for (i = 0; i < H9P_BRIDGE_HEADER_DWORDS; i++)
		header[i] = readl(config + H9P_BRIDGE_HEADER_FIRST + i * sizeof(u32));
}

static void apple_h9p_restore_bridge_header(void __iomem *config,
					    const u32 *header)
{
	unsigned int i;

	for (i = 0; i < H9P_BRIDGE_HEADER_DWORDS; i++)
		writel(header[i], config + H9P_BRIDGE_HEADER_FIRST +
			       i * sizeof(u32));
}

static u32 apple_h9p_bridge_bus_numbers(unsigned int bus)
{
	return bus << 8 | bus << 16;
}

/*
 * Temporary J172-only compatibility quirk. This is an observed firmware
 * transaction sequence, not an understood H9P controller state, and must not
 * be included in an upstream series until the hardware operation is known.
 */
static const unsigned int h9p_j172_quirk_ports[] = { 0, 2, 3 };

static void apple_h9p_j172_quirk_read_capabilities(void __iomem *config)
{
	static const u16 legacy_caps[] = { 0x40, 0x50, 0x70 };
	unsigned int i;

	readl(config + PCI_COMMAND);
	readl(config + PCI_CAPABILITY_LIST);
	for (i = 0; i < ARRAY_SIZE(legacy_caps); i++)
		readl(config + legacy_caps[i]);
}

static void
apple_h9p_j172_quirk_postwrite_readback(struct apple_h9p_pcie *pcie, u64 cap)
{
	static const u16 extended_caps[] = {
		H9P_CFG_AER_CAP, 0x148, 0x160,
	};
	void __iomem *config;
	unsigned int i;
	unsigned int j;

	/*
	 * These reads reproduce the ordering barriers in the firmware bridge
	 * walk. They are MMIO transactions with hardware side effects, not
	 * values consumed by software, so they must not be folded together.
	 */
	config = pcie->base_config + 3 * H9P_CFG_PORT_STRIDE;
	apple_h9p_j172_quirk_read_capabilities(config);
	readl(config + PCI_COMMAND);
	apple_h9p_j172_quirk_read_capabilities(config);
	for (i = 0; i < ARRAY_SIZE(extended_caps); i++)
		readl(config + extended_caps[i]);
	readl(config + PCI_COMMAND);
	apple_h9p_j172_quirk_read_capabilities(config);
	for (i = 0; i < ARRAY_SIZE(extended_caps); i++)
		readl(config + extended_caps[i]);

	for (i = 0; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		for (j = PCI_VENDOR_ID; j <= PCI_INTERRUPT_LINE;
		     j += sizeof(u32))
			readl(config + j);
		readl(config + cap + PCI_EXP_LNKCAP);
	}
}

static void
apple_h9p_j172_quirk_save_state(struct apple_h9p_pcie *pcie, u64 cap,
				struct apple_h9p_j172_bridge_quirk_state *state)
{
	void __iomem *config;
	unsigned int i;
	unsigned int j;

	state->rc_port_ctl1 = readl(pcie->base_rc + H9P_RC_PORT_CTL1(0));
	for (i = 0; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		for (j = 0; j < H9P_J172_QUIRK_CFG_DWORDS; j++)
			state->root[i][j] =
				readl(config + H9P_J172_QUIRK_CFG_FIRST +
				      j * sizeof(u32));
		state->root_aer_uncor_mask[i] =
			readl(config + H9P_CFG_AER_CAP + PCI_ERR_UNCOR_MASK);
		state->root_aer_cor_mask[i] =
			readl(config + H9P_CFG_AER_CAP + PCI_ERR_COR_MASK);
		state->root_devctlsts[i] =
			readl(config + cap + PCI_EXP_DEVCTL);
	}
}

static void
apple_h9p_j172_quirk_replay(struct apple_h9p_pcie *pcie, u64 cap,
			    struct apple_h9p_j172_bridge_quirk_state *state)
{
	void __iomem *nvme = pcie->base_config + (3 << 20);
	void __iomem *config;
	u32 devctlsts = 0;
	unsigned int i;

	/*
	 * Replay the bounded firmware pre-enumeration transaction sequence.
	 * The caller saved every BAR/window dword, the AER masks, Device
	 * Control and the RC port control register that this sequence changes;
	 * apple_h9p_j172_quirk_restore_state() restores them before Linux
	 * enumerates the bus. W1C AER status writes intentionally acknowledge
	 * pending hardware state and therefore are not restorable.
	 */
	for (i = 0; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		writel(~0, config + PCI_BASE_ADDRESS_0);
		readl(config + PCI_BASE_ADDRESS_0);
		writel(0, config + PCI_BASE_ADDRESS_0);
		writel(~0, config + PCI_BASE_ADDRESS_1);
		readl(config + PCI_BASE_ADDRESS_1);
		writel(0, config + PCI_BASE_ADDRESS_1);
	}

	writel(apple_h9p_bridge_bus_numbers(3),
	       pcie->base_config + PCI_PRIMARY_BUS);
	writel(apple_h9p_bridge_bus_numbers(2),
	       pcie->base_config + 2 * H9P_CFG_PORT_STRIDE + PCI_PRIMARY_BUS);
	writel(apple_h9p_bridge_bus_numbers(1),
	       pcie->base_config + 3 * H9P_CFG_PORT_STRIDE + PCI_PRIMARY_BUS);
	readl(pcie->base_config + PCI_PRIMARY_BUS);

	writel(BIT(20), pcie->base_config + H9P_CFG_AER_CAP +
	       PCI_ERR_UNCOR_STATUS);

	state->nvme_valid = readl(nvme + PCI_VENDOR_ID) != ~0U;
	if (state->nvme_valid) {
		for (i = 0; i < H9P_J172_QUIRK_CFG_DWORDS; i++)
			state->nvme[i] =
				readl(nvme + H9P_J172_QUIRK_CFG_FIRST +
				      i * sizeof(u32));
	}

	writel(PCI_ROM_ADDRESS_MASK, nvme + PCI_ROM_ADDRESS);
	readl(nvme + PCI_ROM_ADDRESS);
	writel(0, nvme + PCI_ROM_ADDRESS);
	writel(~0, nvme + PCI_BASE_ADDRESS_0);
	readl(nvme + PCI_BASE_ADDRESS_0);
	writel(PCI_BASE_ADDRESS_MEM_TYPE_64, nvme + PCI_BASE_ADDRESS_0);
	writel(0, nvme + PCI_BASE_ADDRESS_1);
	for (i = PCI_BASE_ADDRESS_2; i <= PCI_BASE_ADDRESS_5;
	     i += sizeof(u32)) {
		writel(~0, nvme + i);
		readl(nvme + i);
		writel(0, nvme + i);
	}

	config = pcie->base_config;
	writel(0, config + PCI_IO_BASE_UPPER16);
	writel(H9P_BRIDGE_WINDOW_DISABLED, config + PCI_MEMORY_BASE);
	writel(H9P_BRIDGE_WINDOW_DISABLED, config + PCI_PREF_MEMORY_BASE);
	writel(0, config + PCI_PREF_LIMIT_UPPER32);
	writel(~0, config + PCI_PREF_BASE_UPPER32);

	for (i = 1; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		writel(0, config + PCI_IO_BASE_UPPER16);
		writel(H9P_BRIDGE_WINDOW_DISABLED,
		       config + PCI_MEMORY_BASE);
		writel(~0, config + PCI_PREF_BASE_UPPER32);
		writel(0, config + PCI_PREF_LIMIT_UPPER32);
		writel(H9P_BRIDGE_WINDOW_DISABLED,
		       config + PCI_PREF_MEMORY_BASE);
		/*
		 * Firmware repeats the prefetch-window disable after updating
		 * both upper dwords. Preserve the second transaction: omitting
		 * it does not reproduce the natural port-3 lifecycle.
		 */
		writel(H9P_BRIDGE_WINDOW_DISABLED,
		       config + PCI_PREF_MEMORY_BASE);
		writel(0, config + PCI_PREF_LIMIT_UPPER32);
		writel(~0, config + PCI_PREF_BASE_UPPER32);
	}

	config = pcie->base_config;
	writel(0xc000c000, config + PCI_MEMORY_BASE);
	writel(H9P_BRIDGE_WINDOW_DISABLED, config + PCI_PREF_MEMORY_BASE);
	writel(0, config + PCI_PREF_LIMIT_UPPER32);
	writel(~0, config + PCI_PREF_BASE_UPPER32);
	writel(0xc0000000, nvme + PCI_BASE_ADDRESS_0);

	for (i = 0; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		writel(1, config + H9P_CFG_AER_CAP + PCI_ERR_UNCOR_STATUS);
		writel(1, config + H9P_CFG_AER_CAP + PCI_ERR_UNCOR_MASK);
		writel(0, config + H9P_CFG_AER_CAP + PCI_ERR_COR_STATUS);
		writel(0, config + H9P_CFG_AER_CAP + PCI_ERR_COR_MASK);

		devctlsts = readl(config + cap + PCI_EXP_DEVCTL);
		devctlsts = (devctlsts & GENMASK(31, 16)) |
			    H9P_ROOT_PORT_DEVCTL;
		writel(devctlsts, config + cap + PCI_EXP_DEVCTL);
	}
	apple_h9p_j172_quirk_postwrite_readback(pcie, cap);
}

static void
apple_h9p_j172_quirk_restore_state(struct apple_h9p_pcie *pcie, u64 cap,
				   const struct apple_h9p_j172_bridge_quirk_state *state)
{
	void __iomem *nvme = pcie->base_config + (3 << 20);
	void __iomem *config;
	unsigned int i;
	unsigned int j;

	if (state->nvme_valid) {
		for (i = 0; i < H9P_J172_QUIRK_CFG_DWORDS; i++)
			writel(state->nvme[i],
			       nvme + H9P_J172_QUIRK_CFG_FIRST +
			       i * sizeof(u32));
	}

	for (i = 0; i < ARRAY_SIZE(h9p_j172_quirk_ports); i++) {
		config = pcie->base_config +
			 h9p_j172_quirk_ports[i] * H9P_CFG_PORT_STRIDE;
		for (j = 0; j < H9P_J172_QUIRK_CFG_DWORDS; j++)
			writel(state->root[i][j],
			       config + H9P_J172_QUIRK_CFG_FIRST +
			       j * sizeof(u32));
		writel(state->root_aer_uncor_mask[i],
		       config + H9P_CFG_AER_CAP + PCI_ERR_UNCOR_MASK);
		writel(state->root_aer_cor_mask[i],
		       config + H9P_CFG_AER_CAP + PCI_ERR_COR_MASK);
		writel(state->root_devctlsts[i],
		       config + cap + PCI_EXP_DEVCTL);
	}

	writel(state->rc_port_ctl1,
	       pcie->base_rc + H9P_RC_PORT_CTL1(0));
	readl(pcie->base_rc + H9P_RC_PORT_CTL1(0));
}

static void apple_h9p_run_j172_bridge_quirk(struct apple_h9p_pcie *pcie,
					    u64 cap)
{
	struct apple_h9p_j172_bridge_quirk_state *state =
		pcie->j172_quirk_state;

	usleep_range(H9P_J172_QUIRK_FIRST_SETTLE_MIN_US,
		     H9P_J172_QUIRK_FIRST_SETTLE_MAX_US);
	apple_h9p_j172_quirk_save_state(pcie, cap, state);
	usleep_range(H9P_J172_QUIRK_ACK_SETTLE_MIN_US,
		     H9P_J172_QUIRK_ACK_SETTLE_MAX_US);
	apple_h9p_j172_quirk_replay(pcie, cap, state);
	usleep_range(H9P_J172_QUIRK_PREFIX_SETTLE_MIN_US,
		     H9P_J172_QUIRK_PREFIX_SETTLE_MAX_US);
	apple_h9p_j172_quirk_restore_state(pcie, cap, state);
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

	while (pcie->clks_enabled) {
		unsigned int index = --pcie->clks_enabled;

		clk_disable_unprepare(pcie->clks[index].clk);
	}
}

static irqreturn_t apple_h9p_port_irq(int irq, void *data)
{
	struct apple_h9p_port_irq *port_irq = data;
	struct apple_h9p_pcie *pcie = port_irq->pcie;
	void __iomem *base = pcie->base_port[port_irq->index];
	u32 mask, status;

	mask = readl(base + H9P_PORT_IRQMASK);
	status = readl(base + H9P_PORT_IRQSTAT) & ~mask;
	if (!status)
		return IRQ_NONE;

	if (port_irq->index == 3 && (status & H9P_PORT_LINK_EVENT))
		writel(1, pcie->base_rc + H9P_RC_PORT_CTL1(0));

	writel(status, base + H9P_PORT_IRQSTAT);
	readl(base + H9P_PORT_IRQSTAT);

	return IRQ_HANDLED;
}

static void apple_h9p_arm_port_irq(struct apple_h9p_pcie *pcie,
				   unsigned int port)
{
	struct apple_h9p_port_irq *port_irq = &pcie->port_irq[port];
	void __iomem *base = pcie->base_port[port];

	if (port_irq->enabled) {
		disable_irq(port_irq->irq);
		port_irq->enabled = false;
	}

	writel(H9P_PORT_IRQMASK_ALL, base + H9P_PORT_IRQMASK);
	writel(H9P_PORT_IRQSTAT_PRE_LINK, base + H9P_PORT_IRQSTAT);
	readl(base + H9P_PORT_IRQSTAT);
	writel(H9P_PORT_IRQMASK_PRE_LINK, base + H9P_PORT_IRQMASK);
	readl(base + H9P_PORT_IRQMASK);
	enable_irq(port_irq->irq);
	port_irq->enabled = true;
}

static void apple_h9p_disable_port_irq(struct apple_h9p_pcie *pcie,
				       unsigned int port)
{
	struct apple_h9p_port_irq *port_irq = &pcie->port_irq[port];

	if (!port_irq->enabled)
		return;

	disable_irq(port_irq->irq);
	port_irq->enabled = false;
}

static int apple_h9p_pcie_request_port_irqs(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned int port;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		struct apple_h9p_port_irq *port_irq = &pcie->port_irq[port];
		char name[8];
		int irq, ret;

		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		snprintf(name, sizeof(name), "port%u", port);
		irq = platform_get_irq_byname(pcie->pdev, name);
		if (irq < 0)
			return irq;

		port_irq->pcie = pcie;
		port_irq->index = port;
		port_irq->irq = irq;
		ret = devm_request_irq(dev, irq, apple_h9p_port_irq,
				       port == 3 ? IRQF_NO_AUTOEN : 0,
				       dev_name(dev), port_irq);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request port %u IRQ\n",
					     port);
		port_irq->enabled = port != 3;
	}

	return 0;
}

static struct apple_h9p_pcie *apple_h9p_pcie_lookup(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	return bridge ? pci_host_bridge_priv(bridge) : NULL;
}

static void apple_h9p_limit_config_field(u32 *val, int where, int size,
					 int reg, u32 mask, u32 value)
{
	u32 access_mask;
	unsigned int shift;

	if (where < reg || where + size > reg + sizeof(u32))
		return;

	shift = (where - reg) * 8;
	access_mask = GENMASK(size * 8 - 1, 0);
	mask = (mask >> shift) & access_mask;
	value = (value >> shift) & access_mask;
	*val = (*val & ~mask) | (value & mask);
}

static int apple_h9p_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_h9p_pcie *pcie;
	unsigned int port;
	u32 max_speed;
	int ret;

	if (bus->number == cfg->busr.start) {
		pcie = apple_h9p_pcie_lookup(cfg->parent);
		port = PCI_SLOT(devfn);
		if (!pcie || port >= H9P_NUM_PORTS ||
		    !(pcie->enabled_ports & BIT(port)))
			return PCIBIOS_DEVICE_NOT_FOUND;
	} else {
		pcie = NULL;
	}

	ret = pci_generic_config_read(bus, devfn, where, size, val);
	if (ret != PCIBIOS_SUCCESSFUL || !pcie)
		return ret;

	max_speed = pcie->max_link_speed[port];
	if (!max_speed || !pcie->pcie_cap[port])
		return ret;

	/*
	 * The H9P capability dwords are hardwired. Expose the operational
	 * max-link-speed limit through config space for ports whose runtime
	 * speed-change sequence is not implemented.
	 */
	apple_h9p_limit_config_field(val, where, size,
				     pcie->pcie_cap[port] + PCI_EXP_LNKCAP,
				     PCI_EXP_LNKCAP_SLS, max_speed);
	apple_h9p_limit_config_field(val, where, size,
				     pcie->pcie_cap[port] + PCI_EXP_LNKCAP2,
				     PCI_EXP_LNKCAP2_SLS,
				     GENMASK(max_speed, 1));

	return ret;
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

static bool apple_h9p_ltssm_link_up(u32 ltssm)
{
	return ltssm >= H9P_PORT_LTSSM_L0 && ltssm <= H9P_PORT_LTSSM_L1_IDLE;
}

static bool apple_h9p_link_up(struct apple_h9p_pcie *pcie, unsigned int port)
{
	u32 linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);

	linksts = FIELD_GET(H9P_PORT_LINKSTS_LTSSM, linksts);
	return apple_h9p_ltssm_link_up(linksts);
}

static int apple_h9p_wait_link(struct apple_h9p_pcie *pcie, unsigned int port,
			       u32 *last_linksts)
{
	ktime_t timeout = ktime_add_us(ktime_get(), 500000);

	do {
		u32 linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);
		u32 ltssm = FIELD_GET(H9P_PORT_LINKSTS_LTSSM, linksts);

		*last_linksts = linksts;

		if (apple_h9p_ltssm_link_up(ltssm))
			return 0;

		if (!ktime_before(ktime_get(), timeout))
			break;
		usleep_range(1000, 2000);
	} while (true);

	return -ETIMEDOUT;
}

static int apple_h9p_enable_rc_port(struct apple_h9p_pcie *pcie,
				    unsigned int port)
{
	u32 linksts;
	int ret;

	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 1, 0);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0, 1);

	ret = apple_h9p_wait(pcie->base_rc + H9P_RC_COMMON_STAT,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE,
			     H9P_RC_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return ret;

	usleep_range(100, 200);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0, 1);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL0(port), 0x100, 0);
	usleep_range(100, 200);
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL2(port), 0, 1);
	if (port == 3) {
		ret = readl_poll_timeout_atomic(pcie->base_port[port] +
						 H9P_PORT_LINKSTS,
						 linksts,
						 FIELD_GET(H9P_PORT_LINKSTS_HW_STATE,
							   linksts) ==
						 H9P_PORT_LINKSTS_HW_READY,
						 1, 100);
		if (ret)
			return ret;
	}
	writel(H9P_LINK_SPEED_8GT,
	       pcie->base_rc + H9P_RC_PORT_LINK_RATE(port));
	h9p_rmw(pcie->base_rc + H9P_RC_PORT_CTL1(port), 0x100, 0);

	return 0;
}

static int apple_h9p_preinit_phy_port(struct apple_h9p_pcie *pcie,
				      unsigned int port)
{
	int ret;

	ret = apple_h9p_enable_rc_port(pcie, port);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "port %u preinit timeout\n", port);

	ret = apple_h9p_wait(pcie->base_phy + H9P_PHY_PORTMASK,
			     BIT(port), BIT(port), BIT(port), 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "port %u preinit PHY timeout\n", port);

	return 0;
}

static void apple_h9p_program_phy_port(struct apple_h9p_pcie *pcie,
				       unsigned int port)
{
	unsigned int phy_lane = pcie->phy_lane[port];

	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON0, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_COMMON1, 0, 0x4000);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME0, 0xfff, 100);
	h9p_rmw(pcie->base_phy_ip + H9P_PHY_IP_EQ_TIME1, 0xfff, 25);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_EQ_CTL),
		0, 0x4000);
	writel(0, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_IDLE));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_EQ_PRESET),
		0xfff, 0x600);
	writel(0x3105, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_RX_CTL0));
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_RX_CTL1),
		0xff, 0x9f);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_RX_CTL2),
		0xff, 0x01);
	h9p_rmw(pcie->base_phy_ip +
		H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_RX_CTL3),
		0x1f, 0x0a);
	writel(175, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER0));
	writel(175, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER1));
	writel(333, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER2));
	writel(333, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER3));
	writel(530, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER4));
	writel(530, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_TIMER5));
	writel(0, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_CLEAR0));
	writel(0, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_CLEAR1));
	writel(0, pcie->base_phy_ip +
	       H9P_PHY_IP_PORT(phy_lane, H9P_PHY_IP_PORT_CLEAR2));
}

static int apple_h9p_setup_port(struct apple_h9p_pcie *pcie, unsigned int port,
				enum apple_h9p_port_result *result)
{
	struct device *dev = pcie->dev;
	void __iomem *config = pcie->base_config + port * H9P_CFG_PORT_STRIDE;
	unsigned int phy_lane = pcie->phy_lane[port];
	u32 bridge_header[H9P_BRIDGE_HEADER_DWORDS];
	u64 cap;
	u32 linksts;
	int ret;

	*result = APPLE_H9P_PORT_READY;
	if (apple_h9p_link_up(pcie, port))
		return 0;

	if (pcie->inherited_perst_pending & BIT(port)) {
		pcie->inherited_perst_pending &= ~BIT(port);
	} else {
		ret = gpiod_direction_output(pcie->perst[port], 1);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to assert port %u PERST#\n",
					     port);
		usleep_range(250, 1000);
	}

	ret = apple_h9p_enable_rc_port(pcie, port);
	if (ret)
		return dev_err_probe(dev, ret, "port %u init timeout\n", port);

	if (pcie->manual_dart[port]) {
		ret = apple_dart_reload_configuration(pcie->manual_dart[port],
						      pcie->manual_dart_sid[port]);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to reload port %u DART\n", port);
	}

	ret = apple_h9p_wait(pcie->base_phy + H9P_PHY_PORTMASK,
			     BIT(phy_lane), BIT(phy_lane), BIT(phy_lane),
			     250000);
	if (ret)
		return dev_err_probe(dev, ret,
				     "port %u PHY lane %u up timeout\n",
				     port, phy_lane);

	if (!(pcie->hardware_trained_phy_ports & BIT(port)))
		apple_h9p_program_phy_port(pcie, port);
	if (pcie->clkreq[port]) {
		ret = apple_h9p_wait_gpio(pcie->clkreq[port], 1, 250000);
		if (ret)
			return dev_err_probe(dev, ret,
					     "port %u CLKREQ# timeout\n", port);
	}

	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (cap) {
		pcie->pcie_cap[port] = cap;
		h9p_rmww(config + cap + PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS,
			 port ? H9P_LINK_SPEED_2_5GT : H9P_LINK_SPEED_8GT);
	}

	apple_h9p_apply_tunables(config, h9p_config_tunables,
				 ARRAY_SIZE(h9p_config_tunables));
	if (port != 3)
		apple_h9p_save_bridge_header(config, bridge_header);
	apple_h9p_apply_tunables(pcie->base_port[port], h9p_port_tunables,
				 ARRAY_SIZE(h9p_port_tunables));

	h9p_rmw(config + H9P_CFG_PORT_MISC, 0, 1);

	if (port == 3) {
		if (!cap) {
			linksts = readl(pcie->base_port[port] +
					H9P_PORT_LINKSTS);
			ret = -ENODEV;
			goto disable_port;
		}
	}

	if (port == 3) {
		apple_h9p_arm_port_irq(pcie, port);
	} else {
		writel(H9P_PORT_IRQMASK_PRE_LINK,
		       pcie->base_port[port] + H9P_PORT_IRQMASK);
		writel(H9P_PORT_IRQSTAT_PRE_LINK,
		       pcie->base_port[port] + H9P_PORT_IRQSTAT);
	}

	h9p_rmw(pcie->base_port[port] + H9P_PORT_ENABLE, 0,
		H9P_PORT_ENABLE_APPLE);
	writel(H9P_PORT_PWRCTL_INIT, pcie->base_port[port] + H9P_PORT_PWRCTL);
	writel(port * 0x10001 * H9P_MSI_PER_PORT,
	       pcie->base_port[port] + H9P_PORT_MSIVECBASE);

	if (port == 3) {
		if (pcie->j172_bridge_quirk)
			apple_h9p_run_j172_bridge_quirk(pcie, cap);
		h9p_rmww(config + cap + PCI_EXP_LNKCTL, 0,
			 PCI_EXP_LNKCTL_LBMIE | PCI_EXP_LNKCTL_LABIE);
	}

	writel(H9P_PORT_IRQMASK_LINK_UP,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	if ((pcie->deferred_power_ports & BIT(port)) &&
	    !pcie->pwrctrl_powered) {
		ret = pci_pwrctrl_power_on_devices(pcie->dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to power port %u endpoint\n",
					     port);
		pcie->pwrctrl_powered = true;
	}
	if (port == 3) {
		linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);
		if (linksts & H9P_PORT_LINKSTS_LINK_PRESENT) {
			ret = -ETIMEDOUT;
			goto disable_port;
		}
	}
	if (port != 3)
		apple_h9p_restore_bridge_header(config, bridge_header);

	ret = gpiod_direction_output(pcie->perst[port], 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert port %u PERST#\n", port);

	if (pcie->hardware_trained_phy_ports & BIT(port)) {
		h9p_rmww(config + cap + PCI_EXP_LNKCTL2,
			 PCI_EXP_LNKCTL2_TLS, H9P_LINK_SPEED_2_5GT);
		h9p_rmw(pcie->base_phy + H9P_PHY_PORTMASK, 0, BIT(phy_lane));
		apple_h9p_program_phy_port(pcie, port);
	}

	h9p_rmw(pcie->base_port[port] + H9P_PORT_LTSSMCTL, 0,
		H9P_PORT_LTSSM_ENABLE);
	ret = apple_h9p_wait_link(pcie, port, &linksts);
	if (ret) {
		dev_warn(dev,
			 "port %u link did not reach L0 (linksts=%#x)\n",
			 port, linksts);
		*result = APPLE_H9P_PORT_LINK_FAILED;
	} else {
		writel(H9P_PORT_IRQMASK_LINK_UP,
		       pcie->base_port[port] + H9P_PORT_IRQMASK);
	}

	return 0;

disable_port:
	if (port == 3)
		apple_h9p_disable_port_irq(pcie, port);
	dev_warn(dev,
		 "disabling port %u after setup failed: %d (linksts=%#x)\n",
		 port, ret, linksts);
	*result = APPLE_H9P_PORT_DISABLED;
	return 0;
}

static int apple_h9p_setup_common(struct apple_h9p_pcie *pcie)
{
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
	apple_h9p_apply_tunables(pcie->base_rc, h9p_rc_common_tunables,
				 ARRAY_SIZE(h9p_rc_common_tunables));
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL1);
	usleep_range(5000, 10000);
	writel(H9P_RC_COMMON_CTL_ENABLE,
	       pcie->base_rc + H9P_RC_COMMON_CTL2);
	usleep_range(500, 1000);

	return 0;
}

static int apple_h9p_prepare_phy_ports(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!(pcie->preinit_phy_ports & BIT(port)))
			continue;

		ret = apple_h9p_preinit_phy_port(pcie, port);
		if (ret)
			return ret;
	}

	return 0;
}

static int apple_h9p_setup_ports(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		enum apple_h9p_port_result result;

		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		ret = apple_h9p_setup_port(pcie, port, &result);
		if (ret)
			return ret;

		if (result == APPLE_H9P_PORT_READY)
			continue;
		if (!(pcie->link_retry_ports & BIT(port))) {
			if (result == APPLE_H9P_PORT_DISABLED)
				pcie->enabled_ports &= ~BIT(port);
			continue;
		}

		h9p_rmw(pcie->base_port[port] + H9P_PORT_LTSSMCTL,
			H9P_PORT_LTSSM_ENABLE, 0);
		ret = gpiod_direction_output(pcie->perst[port], 1);
		if (ret)
			return dev_err_probe(pcie->dev, ret,
					     "failed to assert port %u PERST# for retry\n",
					     port);

		if (pcie->pwrctrl_powered) {
			pci_pwrctrl_power_off_devices(pcie->dev);
			pcie->pwrctrl_powered = false;
			msleep(100);
		}

		ret = pci_pwrctrl_power_on_devices(pcie->dev);
		if (ret)
			return dev_err_probe(pcie->dev, ret,
					     "failed to repower port %u endpoint\n",
					     port);
		pcie->pwrctrl_powered = true;

		ret = apple_h9p_setup_port(pcie, port, &result);
		if (ret)
			return ret;
		if (result == APPLE_H9P_PORT_DISABLED)
			pcie->enabled_ports &= ~BIT(port);
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
	for (i = 0; i < H9P_NUM_PORTS; i++)
		of_node_put(pcie->manual_dart[i]);
}

static int apple_h9p_pcie_parse_ports(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *np;
	u32 reg, port, phy_lane;
	int max_link_speed, ret;

	ret = of_property_read_u32(dev->of_node,
				   "apple,preinit-phy-port-mask",
				   &pcie->preinit_phy_ports);
	if (ret && ret != -EINVAL)
		return ret;
	if (pcie->preinit_phy_ports & ~GENMASK(H9P_NUM_PORTS - 1, 0))
		return -EINVAL;

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
		if (of_property_read_bool(np,
					  "apple,link-retry-after-power-cycle"))
			pcie->link_retry_ports |= BIT(port);
		if (of_property_read_bool(np,
					  "apple,hardware-trained-phy-lane"))
			pcie->hardware_trained_phy_ports |= BIT(port);
		if (of_property_read_bool(np,
					  "apple,endpoint-power-after-link-setup"))
			pcie->deferred_power_ports |= BIT(port);
		if (of_property_read_bool(np,
					  "apple,inherited-perst-before-link-setup"))
			pcie->inherited_perst_ports |= BIT(port);

		phy_lane = port;
		ret = of_property_read_u32(np, "apple,phy-lane", &phy_lane);
		if (ret && ret != -EINVAL)
			goto out_put_child;
		if (phy_lane >= H9P_NUM_PORTS) {
			ret = -EINVAL;
			goto out_put_child;
		}
		pcie->phy_lane[port] = phy_lane;

		max_link_speed = of_pci_get_max_link_speed(np);
		if (max_link_speed > H9P_LINK_SPEED_8GT) {
			ret = -EINVAL;
			goto out_put_child;
		}
		if (max_link_speed > 0)
			pcie->max_link_speed[port] = max_link_speed;

		{
			struct of_phandle_args dart;

			ret = of_parse_phandle_with_args(np,
							 "apple,manual-availability-iommu",
							 "#iommu-cells", 0, &dart);
			if (!ret) {
				if (dart.args_count != 1) {
					of_node_put(dart.np);
					ret = -EINVAL;
					goto out_put_child;
				}
				pcie->manual_dart[port] = dart.np;
				pcie->manual_dart_sid[port] = dart.args[0];
			} else if (ret != -ENOENT) {
				goto out_put_child;
			}
		}
	}

	if (!pcie->enabled_ports)
		return -ENODEV;
	if (pcie->preinit_phy_ports & pcie->enabled_ports)
		return -EINVAL;
	if (hweight32(pcie->deferred_power_ports) > 1)
		return -EINVAL;
	pcie->inherited_perst_pending = pcie->inherited_perst_ports;

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
		enum gpiod_flags flags;

		if (!(pcie->enabled_ports & BIT(i)))
			continue;

		fwnode = of_fwnode_handle(pcie->port_nodes[i]);
		flags = pcie->inherited_perst_ports & BIT(i) ?
			GPIOD_IN : GPIOD_OUT_HIGH;
		gpio = devm_fwnode_gpiod_get(dev, fwnode, "reset",
					     flags, "PERST#");
		pcie->perst[i] = gpio;
		if (IS_ERR(pcie->perst[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->perst[i]),
					     "failed to get PERST#%u\n", i);

		gpio = devm_fwnode_gpiod_get(dev, fwnode, "clkreq", GPIOD_IN, "CLKREQ#");
		if (IS_ERR(gpio) && PTR_ERR(gpio) == -ENOENT) {
			pcie->clkreq[i] = NULL;
			continue;
		}
		if (IS_ERR(gpio))
			return dev_err_probe(dev, PTR_ERR(gpio),
					     "failed to get CLKREQ#%u\n", i);
		pcie->clkreq[i] = gpio;
	}

	return 0;
}

static void apple_h9p_pcie_pwrctrl_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	if (pcie->pwrctrl_powered)
		pci_pwrctrl_power_off_devices(pcie->dev);
	pci_pwrctrl_destroy_devices(pcie->dev);
}

static int apple_h9p_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const bool *j172_bridge_quirk;
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
	j172_bridge_quirk = of_device_get_match_data(dev);
	pcie->j172_bridge_quirk =
		j172_bridge_quirk && *j172_bridge_quirk;
	if (pcie->j172_bridge_quirk) {
		pcie->j172_quirk_state =
			devm_kzalloc(dev, sizeof(*pcie->j172_quirk_state),
				     GFP_KERNEL);
		if (!pcie->j172_quirk_state)
			return -ENOMEM;
	}

	ret = apple_h9p_pcie_parse_ports(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse root ports\n");

	ret = apple_h9p_pcie_map_resources(pdev, pcie);
	if (ret)
		return ret;

	ret = pci_pwrctrl_create_devices(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to create PCI power controls\n");

	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_pwrctrl_cleanup,
				       pcie);
	if (ret)
		return ret;

	ret = pci_pwrctrl_devices_ready(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "waiting for PCI power controls\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	pcie->clks[0].id = "aux";
	pcie->clks[1].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = apple_h9p_pcie_attach_genpd(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach power domains\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_genpd_cleanup, pcie);
	if (ret)
		return ret;

	while (pcie->clks_enabled < ARRAY_SIZE(pcie->clks)) {
		struct clk *clock =
			pcie->clks[pcie->clks_enabled].clk;
		const char *clock_name =
			pcie->clks[pcie->clks_enabled].id;

		ret = clk_prepare_enable(clock);
		if (ret) {
			apple_h9p_pcie_clk_cleanup(pcie);
			return dev_err_probe(dev, ret,
					     "failed to enable %s clock\n",
					     clock_name);
		}
		pcie->clks_enabled++;
	}

	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_clk_cleanup, pcie);
	if (ret)
		return ret;

	pcie->pinctrl = devm_pinctrl_get_select_default(dev);
	if (PTR_ERR(pcie->pinctrl) == -ENODEV)
		pcie->pinctrl = NULL;
	else if (IS_ERR(pcie->pinctrl))
		return dev_err_probe(dev, PTR_ERR(pcie->pinctrl),
				     "failed to select pinctrl state\n");

	ret = apple_h9p_pcie_get_gpios(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_pcie_request_port_irqs(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_setup_common(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_prepare_phy_ports(pcie);
	if (ret)
		return ret;

	if (!pcie->deferred_power_ports) {
		ret = pci_pwrctrl_power_on_devices(dev);
		if (ret) {
			return dev_err_probe(dev, ret,
					     "failed to power on PCI devices\n");
		}
		pcie->pwrctrl_powered = true;
	}

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

static const bool apple_h9p_j172_bridge_quirk = true;

static const struct of_device_id apple_h9p_pcie_of_match[] = {
	{
		.compatible = "apple,j172-pcie",
		.data = &apple_h9p_j172_bridge_quirk,
	},
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

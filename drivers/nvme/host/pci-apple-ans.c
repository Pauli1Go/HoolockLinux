// SPDX-License-Identifier: GPL-2.0
/*
 * Apple ANS PCI NVMe glue for H8 and H9 SoCs.
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2026 Paul Praschl <praschlpaul@g-p.at>
 */

#include <linux/apple-ans.h>
#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/iopoll.h>
#include <linux/mempool.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>

#include "pci-internal.h"

#define NVME_CMD_APPLE_ANS_FLATDMA	BIT(5)

#define APPLE_ANS_REG_INIT		0x1800
#define APPLE_ANS_REG_INIT_REGULAR	0
#define APPLE_ANS_REG_HMB_SIZE_REQ	0x1808
#define APPLE_ANS_REG_HMB_ALIGN_REQ	0x180c
#define APPLE_ANS_REG_HMB_BASE_LO	0x1810
#define APPLE_ANS_REG_HMB_BASE_HI	0x1814
#define APPLE_ANS_REG_HMB_SIZE		0x1818
#define APPLE_ANS_REG_CORE_MASK		0x1824
#define APPLE_ANS_REG_LOG_SIZE		0x1828
#define APPLE_ANS_PCI_CFG_17C		0x17c
#define APPLE_ANS_PCI_CFG_17C_VALUE	0x10081008
#define APPLE_ANS_PCI_CFG_188		0x188
#define APPLE_ANS_PCI_CFG_188_VALUE	0x40550000
#define APPLE_ANS_PCI_CFG_18C		0x18c
#define APPLE_ANS_NVME_MAX_SECTORS \
	(APPLE_ANS_NVMMU_MAX_PAGES * APPLE_ANS_NVMMU_PAGE_SIZE >> SECTOR_SHIFT)

struct apple_ans_nvme_req {
	u64 pages[APPLE_ANS_NVMMU_MAX_PAGES];
};

struct apple_ans_nvme {
	dma_addr_t hmb_dma;
	u32 hmb_size;
	struct apple_ans_nvme_req req[APPLE_ANS_NVMMU_MAX_REQS];
	DECLARE_BITMAP(used_req, APPLE_ANS_NVMMU_MAX_REQS);
	unsigned int last_req;
	/* Protects the FlatDMA request-slot bitmap. */
	spinlock_t req_lock;
};

static struct apple_ans_nvme *nvme_pci_apple_ans(struct nvme_dev *dev)
{
	return dev->dma_data;
}

static int nvme_pci_apple_ans_init(struct nvme_dev *dev, int node)
{
	struct apple_ans_nvme *ans;

	ans = kzalloc_node(sizeof(*ans), GFP_KERNEL, node);
	if (!ans)
		return -ENOMEM;

	spin_lock_init(&ans->req_lock);
	dev->dma_data = ans;
	return 0;
}

static void nvme_pci_apple_ans_exit(struct nvme_dev *dev)
{
	kfree(dev->dma_data);
	dev->dma_data = NULL;
}

static int nvme_pci_apple_ans_find_hmb(struct nvme_dev *dev, u32 size_req,
				       u32 align_req)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	struct pci_host_bridge *bridge;
	struct device_node *pcie_np;
	struct device_node *mem_np;
	struct resource res;
	resource_size_t size;
	u32 iova;
	int ret;

	bridge = pci_find_host_bridge(pdev->bus);
	if (!bridge)
		return -ENODEV;

	pcie_np = bridge->dev.parent ? bridge->dev.parent->of_node : NULL;
	if (!pcie_np)
		pcie_np = bridge->dev.of_node;
	if (!pcie_np)
		return -ENODEV;

	mem_np = of_parse_phandle(pcie_np, "apple,nvme-hmb", 0);
	if (!mem_np)
		return -ENODEV;

	ret = of_address_to_resource(mem_np, 0, &res);
	if (ret)
		goto out_put_mem;

	ret = of_property_read_u32(pcie_np, "apple,nvmmu-iova", &iova);
	if (ret)
		goto out_put_mem;

	if (!size_req || size_req == U32_MAX) {
		ret = -EINVAL;
		goto out_put_mem;
	}
	if (!align_req || align_req == U32_MAX)
		align_req = 1;

	size = resource_size(&res);
	if (size < size_req || size > U32_MAX) {
		ret = -ENOSPC;
		goto out_put_mem;
	}
	if (!IS_ALIGNED(res.start, align_req) || !IS_ALIGNED(iova, align_req)) {
		ret = -EINVAL;
		goto out_put_mem;
	}

	ans->hmb_dma = iova;
	ans->hmb_size = size;
	dev_dbg(dev->dev, "Apple ANS HMB %#x@%pa as DMA %#llx\n",
		ans->hmb_size, &res.start, (u64)ans->hmb_dma);

out_put_mem:
	of_node_put(mem_np);
	return ret;
}

static int nvme_pci_apple_ans_reset_dma(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int tag;
	int ret;

	for (tag = 0; tag < APPLE_ANS_NVMMU_MAX_REQS; tag++) {
		ret = apple_ans_nvmmu_map(dev->dev, tag, NULL, 0, NULL);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&ans->req_lock, flags);
	bitmap_zero(ans->used_req, APPLE_ANS_NVMMU_MAX_REQS);
	ans->last_req = 0;
	spin_unlock_irqrestore(&ans->req_lock, flags);
	return 0;
}

static int nvme_pci_apple_ans_preinit(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	u32 csts, core_mask, log_size;
	u32 hmb_size, hmb_align;
	int ret;

	if (!ans)
		return 0;

	if (pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0x40) !=
	    PCIBIOS_SUCCESSFUL ||
	    pci_write_config_dword(pdev, APPLE_ANS_PCI_CFG_17C,
				   APPLE_ANS_PCI_CFG_17C_VALUE) !=
	    PCIBIOS_SUCCESSFUL ||
	    pci_write_config_dword(pdev, APPLE_ANS_PCI_CFG_18C, 0) !=
	    PCIBIOS_SUCCESSFUL ||
	    pci_write_config_dword(pdev, APPLE_ANS_PCI_CFG_188,
				   APPLE_ANS_PCI_CFG_188_VALUE) !=
	    PCIBIOS_SUCCESSFUL)
		return -EIO;

	core_mask = readl(dev->bar + APPLE_ANS_REG_CORE_MASK);
	log_size = readl(dev->bar + APPLE_ANS_REG_LOG_SIZE);
	hmb_size = readl(dev->bar + APPLE_ANS_REG_HMB_SIZE_REQ);
	hmb_align = readl(dev->bar + APPLE_ANS_REG_HMB_ALIGN_REQ);

	writel(0, dev->bar + NVME_REG_CC);
	ret = readl_poll_timeout(dev->bar + NVME_REG_CSTS, csts,
				 !(csts & (NVME_CSTS_RDY | NVME_CSTS_CFS)),
				 1000, 2000000);
	if (ret)
		return ret;

	if (!ans->hmb_size) {
		ret = nvme_pci_apple_ans_find_hmb(dev, hmb_size, hmb_align);
		if (ret)
			return ret;
	}

	ret = nvme_pci_apple_ans_reset_dma(dev);
	if (ret)
		return dev_err_probe(dev->dev, ret,
				     "failed to reset ANS DMA slots\n");

	dev_dbg(dev->dev,
		"Apple ANS core_mask=%#x log_size=%#x hmb_req=%#x align=%#x\n",
		core_mask, log_size, hmb_size, hmb_align);
	return 0;
}

static int nvme_pci_apple_ans_prepare_enable(struct nvme_dev *dev)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);

	if (!ans)
		return 0;
	if (!ans->hmb_size)
		return -EINVAL;

	writel(APPLE_ANS_REG_INIT_REGULAR, dev->bar + APPLE_ANS_REG_INIT);
	writel(lower_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_LO);
	writel(upper_32_bits(ans->hmb_dma),
	       dev->bar + APPLE_ANS_REG_HMB_BASE_HI);
	writel(ans->hmb_size, dev->bar + APPLE_ANS_REG_HMB_SIZE);
	readl(dev->bar + APPLE_ANS_REG_HMB_SIZE);

	return 0;
}

static blk_status_t nvme_pci_apple_ans_alloc_req(struct nvme_dev *dev,
						 struct apple_ans_nvme_req **req,
						 unsigned int *tag)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int idx;

	spin_lock_irqsave(&ans->req_lock, flags);
	idx = find_next_zero_bit(ans->used_req, APPLE_ANS_NVMMU_MAX_REQS,
				 (ans->last_req + 1) %
				 APPLE_ANS_NVMMU_MAX_REQS);
	if (idx >= APPLE_ANS_NVMMU_MAX_REQS)
		idx = find_first_zero_bit(ans->used_req,
					  APPLE_ANS_NVMMU_MAX_REQS);
	if (idx >= APPLE_ANS_NVMMU_MAX_REQS) {
		spin_unlock_irqrestore(&ans->req_lock, flags);
		dev_dbg_ratelimited(dev->dev,
				    "Apple ANS NVMe FlatDMA slots exhausted\n");
		return BLK_STS_RESOURCE;
	}

	ans->last_req = idx;
	__set_bit(idx, ans->used_req);
	spin_unlock_irqrestore(&ans->req_lock, flags);

	*req = &ans->req[idx];
	*tag = idx;
	memset((*req)->pages, 0, sizeof((*req)->pages));
	return BLK_STS_OK;
}

static void nvme_pci_apple_ans_free_req(struct nvme_dev *dev,
					struct apple_ans_nvme_req *req)
{
	struct apple_ans_nvme *ans = nvme_pci_apple_ans(dev);
	unsigned long flags;
	unsigned int tag;
	int ret;

	if (!ans || !req)
		return;

	tag = req - ans->req;
	if (tag >= APPLE_ANS_NVMMU_MAX_REQS)
		return;

	ret = apple_ans_nvmmu_map(dev->dev, tag, NULL, 0, NULL);
	if (ret)
		dev_warn_ratelimited(dev->dev,
				     "failed to release ANS DMA slot %u: %d\n",
				     tag, ret);

	spin_lock_irqsave(&ans->req_lock, flags);
	__clear_bit(tag, ans->used_req);
	spin_unlock_irqrestore(&ans->req_lock, flags);
}

static blk_status_t nvme_pci_apple_ans_map_bvec(struct nvme_dev *dev,
						struct request *req,
						struct nvme_iod *iod,
						struct apple_ans_nvme_req *areq,
						const struct bio_vec *bv,
						unsigned int total,
						unsigned int *npages,
						unsigned int *consumed,
						u64 *offs)
{
	dma_addr_t dma_addr;
	u64 phys;
	unsigned int len = bv->bv_len;

	if (!len)
		return BLK_STS_OK;

	if (WARN_ON_ONCE(iod->nr_dma_vecs >= dev->ctrl.max_segments))
		return BLK_STS_IOERR;

	dma_addr = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, dma_addr))
		return BLK_STS_RESOURCE;

	iod->dma_vecs[iod->nr_dma_vecs].addr = dma_addr;
	iod->dma_vecs[iod->nr_dma_vecs].len = len;
	iod->nr_dma_vecs++;

	phys = page_to_phys(bv->bv_page) + bv->bv_offset;
	if (!*consumed) {
		*offs = phys & (APPLE_ANS_NVMMU_PAGE_SIZE - 1);
		phys -= *offs;
		len += *offs;
	} else if (phys & (APPLE_ANS_NVMMU_PAGE_SIZE - 1)) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA segment is not page-aligned: phys=%#llx\n",
				    phys);
		return BLK_STS_IOERR;
	}

	if (*consumed + bv->bv_len != total &&
	    (len & (APPLE_ANS_NVMMU_PAGE_SIZE - 1))) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA segment length is not page-aligned: len=%#x\n",
				    len);
		return BLK_STS_IOERR;
	}

	while (len) {
		if (*npages >= APPLE_ANS_NVMMU_MAX_PAGES)
			return BLK_STS_IOERR;

		areq->pages[(*npages)++] = phys;
		phys += APPLE_ANS_NVMMU_PAGE_SIZE;
		len = len > APPLE_ANS_NVMMU_PAGE_SIZE ?
			len - APPLE_ANS_NVMMU_PAGE_SIZE : 0;
	}

	*consumed += bv->bv_len;
	return BLK_STS_OK;
}

static bool nvme_pci_apple_ans_unmap_data(struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	unsigned int i;

	if (!iod->dma_private)
		return false;

	for (i = 0; i < iod->nr_dma_vecs; i++)
		dma_unmap_page(dev->dev, iod->dma_vecs[i].addr,
			       iod->dma_vecs[i].len, rq_dma_dir(req));
	if (iod->dma_vecs) {
		mempool_free(iod->dma_vecs, dev->dmavec_mempool);
		iod->dma_vecs = NULL;
	}
	iod->nr_dma_vecs = 0;

	nvme_pci_apple_ans_free_req(dev, iod->dma_private);
	iod->dma_private = NULL;
	iod->cmd.common.flags &= ~NVME_CMD_APPLE_ANS_FLATDMA;
	return true;
}

static blk_status_t nvme_pci_apple_ans_map_data(struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	struct apple_ans_nvme_req *areq;
	struct req_iterator iter;
	struct bio_vec bv;
	dma_addr_t flatdma;
	u64 offs = 0;
	unsigned int tag, npages = 0, consumed = 0;
	unsigned int total = blk_rq_payload_bytes(req);
	blk_status_t status;
	int ret;

	if (!nvme_pci_apple_ans(dev))
		return BLK_STS_NOTSUPP;
	if (total > APPLE_ANS_NVMMU_MAX_PAGES * APPLE_ANS_NVMMU_PAGE_SIZE)
		return BLK_STS_IOERR;

	status = nvme_pci_apple_ans_alloc_req(dev, &areq, &tag);
	if (status)
		return status;

	iod->dma_private = areq;
	iod->dma_vecs = mempool_alloc(dev->dmavec_mempool, GFP_ATOMIC);
	if (!iod->dma_vecs) {
		status = BLK_STS_RESOURCE;
		goto out_unmap;
	}

	if (req->rq_flags & RQF_SPECIAL_PAYLOAD) {
		bv = req_bvec(req);
		status = nvme_pci_apple_ans_map_bvec(dev, req, iod, areq, &bv,
						     total, &npages, &consumed,
						     &offs);
		if (status)
			goto out_unmap;
	} else {
		rq_for_each_bvec(bv, req, iter) {
			status = nvme_pci_apple_ans_map_bvec(dev, req, iod,
							     areq, &bv, total,
							     &npages, &consumed,
							     &offs);
			if (status)
				goto out_unmap;
		}
	}

	if (consumed != total) {
		dev_err_ratelimited(dev->dev,
				    "Apple ANS FlatDMA payload length mismatch: consumed=%#x total=%#x\n",
				    consumed, total);
		status = BLK_STS_IOERR;
		goto out_unmap;
	}

	ret = apple_ans_nvmmu_map(dev->dev, tag, areq->pages, npages, &flatdma);
	if (ret) {
		status = errno_to_blk_status(ret);
		if (status == BLK_STS_NOTSUPP)
			status = BLK_STS_IOERR;
		goto out_unmap;
	}

	iod->total_len = total;
	iod->cmd.common.flags |= NVME_CMD_APPLE_ANS_FLATDMA;
	iod->cmd.common.dptr.prp1 = cpu_to_le64(flatdma + offs);
	iod->cmd.common.dptr.prp2 = 0;
	return BLK_STS_OK;

out_unmap:
	nvme_pci_apple_ans_unmap_data(req);
	return status;
}

static u32 nvme_pci_apple_ans_queue_depth(struct nvme_dev *dev, u32 depth)
{
	return min_t(u32, depth, APPLE_ANS_NVMMU_MAX_REQS);
}

static u32 nvme_pci_apple_ans_max_hw_sectors(struct nvme_dev *dev,
					     u32 max_hw_sectors)
{
	return min_t(u32, max_hw_sectors, APPLE_ANS_NVME_MAX_SECTORS);
}

const struct nvme_pci_dma_ops nvme_pci_apple_ans_ops = {
	.init		= nvme_pci_apple_ans_init,
	.exit		= nvme_pci_apple_ans_exit,
	.preinit	= nvme_pci_apple_ans_preinit,
	.prepare_enable = nvme_pci_apple_ans_prepare_enable,
	.map_data	= nvme_pci_apple_ans_map_data,
	.unmap_data	= nvme_pci_apple_ans_unmap_data,
	.queue_depth	= nvme_pci_apple_ans_queue_depth,
	.max_hw_sectors = nvme_pci_apple_ans_max_hw_sectors,
};

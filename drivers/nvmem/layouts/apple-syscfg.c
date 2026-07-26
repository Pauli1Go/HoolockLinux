// SPDX-License-Identifier: GPL-2.0-only
/*
 * NVMEM layout driver for Apple SysCfg data.
 */

#include <linux/device.h>
#include <linux/limits.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/slab.h>

#define APPLE_SYSCFG_MAGIC		"gfCS"
#define APPLE_SYSCFG_BTNC		"BTNC"
#define APPLE_SYSCFG_KEY_LEN		4
#define APPLE_SYSCFG_INLINE_SIZE	16

struct apple_syscfg_hdr {
	u8 magic[4];
	__le32 unk0;
	__le32 size;
	__le32 version;
	__le32 unk1;
	__le32 nkeys;
} __packed;

struct apple_syscfg_key {
	u8 name[APPLE_SYSCFG_KEY_LEN];
	union {
		u8 value[APPLE_SYSCFG_INLINE_SIZE];
		struct {
			u8 name[APPLE_SYSCFG_KEY_LEN];
			__le32 size;
			__le32 offset;
			__le32 rsvd;
		} jumbo;
	};
} __packed;

static void apple_syscfg_key_name(char *out, const u8 *in)
{
	int i;

	for (i = 0; i < APPLE_SYSCFG_KEY_LEN; i++)
		out[i] = in[APPLE_SYSCFG_KEY_LEN - 1 - i];
	out[APPLE_SYSCFG_KEY_LEN] = '\0';
}

static int apple_syscfg_find_key(struct device *dev, const u8 *data,
				 size_t data_size, const char *wanted,
				 unsigned int *offset, unsigned int *bytes)
{
	const struct apple_syscfg_hdr *hdr = (const void *)data;
	const struct apple_syscfg_key *keys;
	u32 declared_size;
	u32 nkeys;
	u32 i;

	declared_size = le32_to_cpu(hdr->size);
	nkeys = le32_to_cpu(hdr->nkeys);

	if (declared_size < sizeof(*hdr) || declared_size > data_size ||
	    nkeys > (declared_size - sizeof(*hdr)) / sizeof(*keys)) {
		dev_err(dev, "SysCfg key table outside declared size\n");
		return -EINVAL;
	}

	keys = (const void *)(data + sizeof(*hdr));

	for (i = 0; i < nkeys; i++) {
		char name[APPLE_SYSCFG_KEY_LEN + 1];

		if (!memcmp(keys[i].name, APPLE_SYSCFG_BTNC,
			    APPLE_SYSCFG_KEY_LEN)) {
			u32 jumbo_offset = le32_to_cpu(keys[i].jumbo.offset);
			u32 jumbo_size = le32_to_cpu(keys[i].jumbo.size);

			apple_syscfg_key_name(name, keys[i].jumbo.name);
			if (strcmp(name, wanted))
				continue;

			if (jumbo_offset > declared_size ||
			    jumbo_size > declared_size - jumbo_offset) {
				dev_err(dev, "SysCfg jumbo key %s outside declared size\n",
					name);
				return -EINVAL;
			}

			*offset = jumbo_offset;
			*bytes = jumbo_size;
			return 0;
		}

		apple_syscfg_key_name(name, keys[i].name);
		if (strcmp(name, wanted))
			continue;

		*offset = sizeof(*hdr) + i * sizeof(*keys) +
			  offsetof(struct apple_syscfg_key, value);
		*bytes = APPLE_SYSCFG_INLINE_SIZE;
		return 0;
	}

	return -ENOENT;
}

static int apple_syscfg_add_cells(struct nvmem_layout *layout)
{
	struct nvmem_device *nvmem = layout->nvmem;
	struct device *dev = &layout->dev;
	struct device_node *layout_np;
	struct apple_syscfg_hdr hdr;
	size_t data_size;
	u8 *data;
	int ret;

	ret = nvmem_device_read(nvmem, 0, sizeof(hdr), &hdr);
	if (ret < 0)
		return ret;
	if (ret != sizeof(hdr))
		return -EIO;

	if (memcmp(hdr.magic, APPLE_SYSCFG_MAGIC, sizeof(hdr.magic))) {
		dev_err(dev, "invalid SysCfg magic\n");
		return -EINVAL;
	}

	data_size = le32_to_cpu(hdr.size);
	if (data_size < sizeof(hdr) || data_size > nvmem_dev_size(nvmem)) {
		dev_err(dev, "invalid SysCfg size %zu\n", data_size);
		return -EINVAL;
	}

	data = kzalloc(data_size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, 0, data_size, data);
	if (ret < 0)
		goto out_free;
	if (ret != data_size) {
		ret = -EIO;
		goto out_free;
	}

	layout_np = of_nvmem_layout_get_container(nvmem);
	if (!layout_np) {
		ret = -ENOENT;
		goto out_free;
	}

	for_each_child_of_node_scoped(layout_np, child) {
		struct nvmem_cell_info info = {};
		const char *key_name;
		u32 cell_size;
		unsigned int offset;
		unsigned int bytes;

		ret = of_property_read_string(child, "apple,syscfg-key",
					      &key_name);
		if (ret) {
			ret = 0;
			continue;
		}

		if (strlen(key_name) != APPLE_SYSCFG_KEY_LEN) {
			dev_err(dev, "invalid SysCfg key name %s on %pOF\n",
				key_name, child);
			ret = -EINVAL;
			break;
		}

		ret = apple_syscfg_find_key(dev, data, data_size, key_name,
					    &offset, &bytes);
		if (ret) {
			if (ret == -ENOENT)
				dev_err(dev, "SysCfg key %s not found\n",
					key_name);
			break;
		}

		if (!of_property_read_u32(child, "apple,syscfg-size",
					  &cell_size)) {
			if (!cell_size || cell_size > bytes) {
				dev_err(dev, "invalid SysCfg cell size on %pOF\n",
					child);
				ret = -EINVAL;
				break;
			}
			bytes = cell_size;
		}

		info.name = kasprintf(GFP_KERNEL, "%pOFn", child);
		if (!info.name) {
			ret = -ENOMEM;
			break;
		}

		info.offset = offset;
		info.bytes = bytes;
		info.np = of_node_get(child);

		ret = nvmem_add_one_cell(nvmem, &info);
		kfree(info.name);
		if (ret) {
			of_node_put(info.np);
			break;
		}
	}

	of_node_put(layout_np);

out_free:
	kfree(data);

	return ret;
}

static int apple_syscfg_probe(struct nvmem_layout *layout)
{
	layout->add_cells = apple_syscfg_add_cells;

	return nvmem_layout_register(layout);
}

static void apple_syscfg_remove(struct nvmem_layout *layout)
{
	nvmem_layout_unregister(layout);
}

static const struct of_device_id apple_syscfg_of_match_table[] = {
	{ .compatible = "apple,syscfg" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_syscfg_of_match_table);

static struct nvmem_layout_driver apple_syscfg_layout = {
	.driver = {
		.name = "apple-syscfg-layout",
		.of_match_table = apple_syscfg_of_match_table,
	},
	.probe = apple_syscfg_probe,
	.remove = apple_syscfg_remove,
};
module_nvmem_layout_driver(apple_syscfg_layout);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVMEM layout driver for Apple SysCfg data");

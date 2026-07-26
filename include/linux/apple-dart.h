/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_APPLE_DART_H_
#define _LINUX_APPLE_DART_H_

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device_node;

#if IS_REACHABLE(CONFIG_APPLE_DART)
int apple_dart_reload_configuration(struct device_node *np, u32 sid);
#else
static inline int apple_dart_reload_configuration(struct device_node *np,
						  u32 sid)
{
	return -ENODEV;
}
#endif

#endif

/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _TSL2583_H
#define _TSL2583_H

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>

#define CT821_CAL_FACTOR_SCALE 255

static inline int ct821_calculate_gain_factor(u32 previous, u16 ratio, u32 *next)
{
	u64 value;

	if (!previous || !ratio)
		return -EINVAL;

	value = DIV_ROUND_CLOSEST_ULL((u64)previous * ratio,
				      CT821_CAL_FACTOR_SCALE);
	if (!value || value > U32_MAX)
		return -ERANGE;

	*next = value;

	return 0;
}

#endif /* _TSL2583_H */

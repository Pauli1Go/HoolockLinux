/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _TSL2583_H
#define _TSL2583_H

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>

/*
 * Fixed-point scale of the per-gain-step ratios in an Apple LSCI ambient-light
 * calibration record. The record stores the sensitivity of one gain step
 * relative to the previous one, so the factors have to be folded together.
 */
#define CT821_CAL_RATIO_SCALE 255

static inline int apple_als_calculate_gain_factor(u32 previous, u16 ratio,
						  u32 scale, u32 *next)
{
	u64 value;

	if (!previous || !ratio || !scale)
		return -EINVAL;

	value = DIV_ROUND_CLOSEST_ULL((u64)previous * ratio, scale);
	if (!value || value > U32_MAX)
		return -ERANGE;

	*next = value;

	return 0;
}

#endif /* _TSL2583_H */

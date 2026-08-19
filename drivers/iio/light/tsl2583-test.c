// SPDX-License-Identifier: GPL-2.0-or-later

#include <kunit/test.h>
#include <linux/module.h>

#include "tsl2583.h"

static void ct821_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 510,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 131072U, factor);
}

static void ct821_gain_factor_zero_test(struct kunit *test)
{
	u32 factor = 1;

	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(65536, 0,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(0, 255,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1U, factor);
}

static void ct821_gain_factor_overflow_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_ASSERT_EQ(test, 1U << 24, factor);
	KUNIT_EXPECT_EQ(test, -ERANGE,
			apple_als_calculate_gain_factor(factor, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
}

static struct kunit_case ct821_gain_factor_test_cases[] = {
	KUNIT_CASE(ct821_gain_factor_valid_test),
	KUNIT_CASE(ct821_gain_factor_zero_test),
	KUNIT_CASE(ct821_gain_factor_overflow_test),
	{}
};

static struct kunit_suite ct821_gain_factor_test_suite = {
	.name = "ct821-gain-factor",
	.test_cases = ct821_gain_factor_test_cases,
};

kunit_test_suite(ct821_gain_factor_test_suite);

MODULE_DESCRIPTION("KUnit tests for the CT821 calibration gain factors");
MODULE_LICENSE("GPL");

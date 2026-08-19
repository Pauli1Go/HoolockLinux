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

static void ct819_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	/*
	 * The gain ratios of the factory record of an iPad 7 fold into the
	 * nominal 8x, 16x and 128x gain steps of the part.
	 */
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 2010,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 514560U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 521,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1047210U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 1862,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 7616816U, factor);
}

static struct kunit_case ct819_gain_factor_test_cases[] = {
	KUNIT_CASE(ct819_gain_factor_valid_test),
	{}
};

static struct kunit_suite ct819_gain_factor_test_suite = {
	.name = "ct819-gain-factor",
	.test_cases = ct819_gain_factor_test_cases,
};

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

kunit_test_suites(&ct819_gain_factor_test_suite,
		  &ct821_gain_factor_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple ALS calibration gain factors");
MODULE_LICENSE("GPL");

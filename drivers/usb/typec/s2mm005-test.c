// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include "s2mm005.h"

static void s2mm005_roles_test(struct kunit *test)
{
	struct s2mm005_state s;

	/* A powered dock must not turn on boost merely because data is host. */
	s = s2mm005_decode(21 | S2MM005_ATTACHED | S2MM005_DFP, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_HOST);
	KUNIT_EXPECT_FALSE(test, s.source);
	/* An ordinary OTG adapter needs power before ATTACH_DONE. */
	s = s2mm005_decode(3 | S2MM005_DFP | S2MM005_SOURCE, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_HOST);
	KUNIT_EXPECT_TRUE(test, s.source);
	/* Preserve the PC gadget connection. */
	s = s2mm005_decode(17 | S2MM005_ATTACHED, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_DEVICE);
	KUNIT_EXPECT_FALSE(test, s.source);
	/* Samsung handles firmware state 29 as a sink attachment. */
	s = s2mm005_decode(29 | S2MM005_ATTACHED, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_DEVICE);
	KUNIT_EXPECT_FALSE(test, s.source);
	/* Data device and power source is also a valid PD combination. */
	s = s2mm005_decode(6 | S2MM005_SOURCE, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_DEVICE);
	KUNIT_EXPECT_TRUE(test, s.source);
}

static void s2mm005_power_swap_test(struct kunit *test)
{
	struct s2mm005_state s;

	/* Firmware asks for power off before updating IS_SOURCE. */
	s = s2mm005_decode(52 | S2MM005_DFP | S2MM005_SOURCE, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_HOST);
	KUNIT_EXPECT_FALSE(test, s.source);
	/* Keep power off throughout the source-to-sink transition. */
	s = s2mm005_decode(53 | S2MM005_DFP | S2MM005_SOURCE,
			   S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_HOST);
	KUNIT_EXPECT_FALSE(test, s.source);
	/* Firmware asks for power on before updating IS_SOURCE. */
	s = s2mm005_decode(64 | S2MM005_DFP, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_HOST);
	KUNIT_EXPECT_TRUE(test, s.source);
}

static void s2mm005_fault_test(struct kunit *test)
{
	const u32 source = 6 | S2MM005_ATTACHED | S2MM005_SOURCE | S2MM005_DFP;
	struct s2mm005_state s;

	s = s2mm005_decode(source, S2MM005_DRY | S2MM005_WATER);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_NONE);
	KUNIT_EXPECT_FALSE(test, s.source);
	s = s2mm005_decode(source | S2MM005_SHORT, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_NONE);
	KUNIT_EXPECT_FALSE(test, s.source);
	s = s2mm005_decode(source, 0);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_NONE);
	s = s2mm005_decode(S2MM005_SOURCE | S2MM005_DFP, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_NONE);
	KUNIT_EXPECT_FALSE(test, s.source);
	s = s2mm005_decode(29 | S2MM005_SOURCE | S2MM005_DFP, S2MM005_DRY);
	KUNIT_EXPECT_EQ(test, s.role, USB_ROLE_NONE);
}

static void s2mm005_pd_contract_test(struct kunit *test)
{
	u8 caps[32] = {}, request[8] = {};

	put_unaligned_le16(2 << 12, caps);
	put_unaligned_le32(PDO_FIXED(5000, 1500, 0), caps + 4);
	put_unaligned_le32(PDO_FIXED(9000, 2000, 0), caps + 8);
	put_unaligned_le32(RDO_FIXED(1, 1000, 1000, 0), request + 4);
	KUNIT_EXPECT_EQ(test, s2mm005_pd_current(caps, request), 1000000);
	put_unaligned_le32(RDO_FIXED(2, 1000, 1000, 0), request + 4);
	KUNIT_EXPECT_EQ(test, s2mm005_pd_current(caps, request), -ERANGE);
	put_unaligned_le32(RDO_FIXED(3, 1000, 1000, 0), request + 4);
	KUNIT_EXPECT_EQ(test, s2mm005_pd_current(caps, request), -EPROTO);
	put_unaligned_le32(RDO_FIXED(1, 2000, 2000, 0), request + 4);
	KUNIT_EXPECT_EQ(test, s2mm005_pd_current(caps, request), -EPROTO);
	put_unaligned_le32(0, request + 4);
	KUNIT_EXPECT_EQ(test, s2mm005_pd_current(caps, request), -EPROTO);
}

static struct kunit_case s2mm005_cases[] = {
	KUNIT_CASE(s2mm005_roles_test),
	KUNIT_CASE(s2mm005_power_swap_test),
	KUNIT_CASE(s2mm005_fault_test),
	KUNIT_CASE(s2mm005_pd_contract_test),
	{}
};

static struct kunit_suite s2mm005_suite = {
	.name = "s2mm005-state",
	.test_cases = s2mm005_cases,
};
kunit_test_suite(s2mm005_suite);

MODULE_LICENSE("GPL");

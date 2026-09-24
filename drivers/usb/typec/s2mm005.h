/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __S2MM005_STATE_H
#define __S2MM005_STATE_H

#include <linux/bitfield.h>
#include <linux/usb/role.h>
#include <linux/usb/pd.h>
#include <linux/unaligned.h>

#define S2MM005_STATE		GENMASK(7, 0)
#define S2MM005_ATTACHED		BIT(24)
#define S2MM005_SOURCE		BIT(25)
#define S2MM005_DFP		BIT(26)
#define S2MM005_RP		GENMASK(28, 27)
#define S2MM005_SHORT		GENMASK(30, 29)
#define S2MM005_RESET		BIT(31)
#define S2MM005_WATER		BIT(3)
#define S2MM005_DRY		BIT(5)
#define S2MM005_SLEEP_CABLE_DETECT	BIT(8)
#define S2MM005_SRC_WAIT_NEW_CAPABILITIES	14

struct s2mm005_state {
	enum usb_role role;
	bool source;
};

static inline struct s2mm005_state s2mm005_decode(u32 func, u32 lp)
{
	struct s2mm005_state state = {};
	u8 pd = FIELD_GET(S2MM005_STATE, func);
	bool attached;

	if (!pd || (func & S2MM005_SHORT) ||
	    (lp & S2MM005_WATER) || !(lp & S2MM005_DRY))
		return state;
	/* Samsung's driver handles firmware state 29 as a sink attachment. */
	if (pd == 29) {
		if (!(func & S2MM005_DFP))
			state.role = USB_ROLE_DEVICE;
		return state;
	}
	/* Firmware requests VBUS before ATTACH_DONE on a source connection. */
	attached = (func & S2MM005_ATTACHED) || (pd >= 3 && pd <= 8) ||
		   (pd >= 17 && pd <= 21) || (pd >= 52 && pd <= 64);
	if (!attached)
		return state;
	state.role = func & S2MM005_DFP ? USB_ROLE_HOST : USB_ROLE_DEVICE;
	state.source = func & S2MM005_SOURCE;
	/* PR swap sequencing precedes the final IS_SOURCE update. */
	if ((pd >= 52 && pd < 64) || (pd >= 9 && pd <= 11))
		state.source = false;
	else if (pd == 64)
		state.source = true;
	return state;
}
/* Firmware message buffers have a four-byte header followed by PDOs. */
static inline int s2mm005_pd_current(const u8 caps[32], const u8 request[8])
{
	u32 rdo = get_unaligned_le32(request + 4);
	unsigned int index = rdo_index(rdo);
	unsigned int count = pd_header_cnt(get_unaligned_le16(caps));
	u32 pdo;

	if (!count || !index || index > count)
		return -EPROTO;
	pdo = get_unaligned_le32(caps + index * 4);
	if (pdo_type(pdo) != PDO_TYPE_FIXED || pdo_fixed_voltage(pdo) != 5000)
		return -ERANGE;
	if (!rdo_op_current(rdo) || rdo_op_current(rdo) > pdo_max_current(pdo))
		return -EPROTO;
	return rdo_op_current(rdo) * 1000;
}
#endif

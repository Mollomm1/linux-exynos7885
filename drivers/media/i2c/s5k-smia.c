// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K4HA and S5K5E9 raw Bayer camera sensors.
 *
 * Both sensors use SMIA/CCS style registers. Register tables and power
 * sequences come from the Samsung Galaxy Tab A 10.1 (SM-T510) kernel.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define S5K_REG_CHIP_ID			CCI_REG16(0x0000)
#define S5K_REG_AGAIN_MIN		CCI_REG16(0x0084)
#define S5K_REG_AGAIN_MAX		CCI_REG16(0x0086)
#define S5K_REG_MODE_SELECT		CCI_REG8(0x0100)
#define  S5K_MODE_STREAMING		0x01
#define S5K_REG_ORIENTATION		CCI_REG8(0x0101)
#define  S5K_HFLIP			BIT(0)
#define  S5K_VFLIP			BIT(1)
#define S5K_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K_REG_AGAIN			CCI_REG16(0x0204)
#define S5K_REG_FLL			CCI_REG16(0x0340)
#define S5K_REG_TEST_PATTERN		CCI_REG16(0x0600)
#define S5K_REG_FW_PAGE			CCI_REG16(0xfcfc)
#define S5K_REG_FW_START		CCI_REG16(0x6010)

#define S5K_MCLK_RATE			26000000
#define S5K_AGAIN_ONE			0x20
#define S5K_FLL_MAX			0xffff

static const struct cci_reg_sequence s5k4ha_global[] = {
	{ CCI_REG8(0x0100), 0x2 },
	{ CCI_REG8(0x0a02), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3b45), 0x2 },
	{ CCI_REG8(0x3264), 0x2 },
	{ CCI_REG8(0x3290), 0x0000000000000002 },
	{ CCI_REG8(0x0b05), 0x2 },
	{ CCI_REG8(0x3069), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3074), 0x000002 },
	{ CCI_REG8(0x3075), 0x00000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3068), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30c6), 0x2 },
	{ CCI_REG8(0x301f), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x306b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3091), 0x0000000000000000000000000000002 },
	{ CCI_REG8(0x306e), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x306f), 0x0000000000000000000000000000000000000002 },
	{ CCI_REG8(0x306d), 0x00000002 },
	{ CCI_REG8(0x3084), 0x0000000000000000000002 },
	{ CCI_REG8(0x3070), 0x000000000000002 },
	{ CCI_REG8(0x306a), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30b0), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30c2), 0x00002 },
	{ CCI_REG8(0x30c4), 0x000002 },
	{ CCI_REG8(0x3081), 0x0000002 },
	{ CCI_REG8(0x307b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x307a), 0x0000000002 },
	{ CCI_REG8(0x3079), 0x0000000002 },
	{ CCI_REG8(0x308a), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x308b), 0x00000002 },
	{ CCI_REG8(0x308c), 0x00000000002 },
	{ CCI_REG8(0x392f), 0x2 },
	{ CCI_REG8(0x3930), 0x2 },
	{ CCI_REG8(0x3924), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3925), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c08), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c09), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c31), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c32), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
};

static const struct cci_reg_sequence s5k4ha_3264x2448[] = {
	{ CCI_REG8(0x0136), 0x00000000000000000000000002 },
	{ CCI_REG8(0x0137), 0x2 },
	{ CCI_REG8(0x0301), 0x0002 },
	{ CCI_REG8(0x0305), 0x00002 },
	{ CCI_REG8(0x0306), 0x2 },
	{ CCI_REG8(0x0307), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c1f), 0x2 },
	{ CCI_REG8(0x030d), 0x00002 },
	{ CCI_REG8(0x030e), 0x2 },
	{ CCI_REG8(0x030f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c17), 0x2 },
	{ CCI_REG8(0x0340), 0x000000002 },
	{ CCI_REG8(0x0341), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0342), 0x00000000000002 },
	{ CCI_REG8(0x0343), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0344), 0x2 },
	{ CCI_REG8(0x0345), 0x00000002 },
	{ CCI_REG8(0x0346), 0x2 },
	{ CCI_REG8(0x0347), 0x00000002 },
	{ CCI_REG8(0x0348), 0x000000000002 },
	{ CCI_REG8(0x0349), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034a), 0x000000002 },
	{ CCI_REG8(0x034b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034c), 0x000000000002 },
	{ CCI_REG8(0x034d), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034e), 0x000000002 },
	{ CCI_REG8(0x034f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0381), 0x2 },
	{ CCI_REG8(0x0383), 0x2 },
	{ CCI_REG8(0x0385), 0x2 },
	{ CCI_REG8(0x0387), 0x2 },
	{ CCI_REG8(0x0900), 0x2 },
	{ CCI_REG8(0x0901), 0x2 },
	{ CCI_REG8(0x0204), 0x2 },
	{ CCI_REG8(0x0205), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x0200), 0x0000000000002 },
	{ CCI_REG8(0x0201), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0202), 0x2 },
	{ CCI_REG8(0x0203), 0x02 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0822), 0x2 },
	{ CCI_REG8(0x0823), 0x2 },
	{ CCI_REG8(0x0112), 0x0000000002 },
	{ CCI_REG8(0x0113), 0x0000000002 },
	{ CCI_REG8(0x0114), 0x002 },
	{ CCI_REG8(0x3929), 0x000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0101), 0x2 },
	{ CCI_REG8(0x3941), 0x000000000000002 },
	{ CCI_REG8(0x3942), 0x0000000000000000000000000000000000000000000000000000000000002 },
};

static const struct cci_reg_sequence s5k4ha_1632x1224[] = {
	{ CCI_REG8(0x0136), 0x00000000000000000000000002 },
	{ CCI_REG8(0x0137), 0x2 },
	{ CCI_REG8(0x0301), 0x0002 },
	{ CCI_REG8(0x0305), 0x00002 },
	{ CCI_REG8(0x0306), 0x2 },
	{ CCI_REG8(0x0307), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c1f), 0x2 },
	{ CCI_REG8(0x030d), 0x00002 },
	{ CCI_REG8(0x030e), 0x2 },
	{ CCI_REG8(0x030f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c17), 0x2 },
	{ CCI_REG8(0x0340), 0x000000002 },
	{ CCI_REG8(0x0341), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0342), 0x00000000000002 },
	{ CCI_REG8(0x0343), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0344), 0x2 },
	{ CCI_REG8(0x0345), 0x00000002 },
	{ CCI_REG8(0x0346), 0x2 },
	{ CCI_REG8(0x0347), 0x00000002 },
	{ CCI_REG8(0x0348), 0x000000000002 },
	{ CCI_REG8(0x0349), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034a), 0x000000002 },
	{ CCI_REG8(0x034b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034c), 0x000002 },
	{ CCI_REG8(0x034d), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034e), 0x0002 },
	{ CCI_REG8(0x034f), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0381), 0x2 },
	{ CCI_REG8(0x0383), 0x2 },
	{ CCI_REG8(0x0385), 0x2 },
	{ CCI_REG8(0x0387), 0x002 },
	{ CCI_REG8(0x0900), 0x2 },
	{ CCI_REG8(0x0901), 0x0000000000000000000000000000000002 },
	{ CCI_REG8(0x0204), 0x2 },
	{ CCI_REG8(0x0205), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x0200), 0x0000000000002 },
	{ CCI_REG8(0x0201), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0202), 0x2 },
	{ CCI_REG8(0x0203), 0x02 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0822), 0x2 },
	{ CCI_REG8(0x0823), 0x2 },
	{ CCI_REG8(0x0112), 0x0000000002 },
	{ CCI_REG8(0x0113), 0x0000000002 },
	{ CCI_REG8(0x0114), 0x002 },
	{ CCI_REG8(0x3929), 0x000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0101), 0x2 },
	{ CCI_REG8(0x3941), 0x000000000000002 },
	{ CCI_REG8(0x3942), 0x0000000000000000000000000000000000000000000000000000000000002 },
};

static const struct cci_reg_sequence s5k5e9_global[] = {
	{ CCI_REG8(0x0100), 0x2 },
	{ CCI_REG8(0x0a02), 0x000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3b45), 0x2 },
	{ CCI_REG8(0x3290), 0x0000000000000002 },
	{ CCI_REG8(0x0b05), 0x2 },
	{ CCI_REG8(0x3069), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3074), 0x000002 },
	{ CCI_REG8(0x3075), 0x00000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x301f), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x306b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3091), 0x0000000000000000000002 },
	{ CCI_REG8(0x306e), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x306f), 0x0000000000000000000000000000000000000002 },
	{ CCI_REG8(0x306d), 0x00000002 },
	{ CCI_REG8(0x3084), 0x0000000000000000000002 },
	{ CCI_REG8(0x3070), 0x000000000000002 },
	{ CCI_REG8(0x306a), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30b0), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30c2), 0x00002 },
	{ CCI_REG8(0x30c4), 0x000002 },
	{ CCI_REG8(0x3012), 0x000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3080), 0x00000002 },
	{ CCI_REG8(0x3083), 0x00000000000000000002 },
	{ CCI_REG8(0x3200), 0x2 },
	{ CCI_REG8(0x3081), 0x0000002 },
	{ CCI_REG8(0x307b), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x307a), 0x0000000002 },
	{ CCI_REG8(0x3079), 0x0000000002 },
	{ CCI_REG8(0x308a), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x308b), 0x00000002 },
	{ CCI_REG8(0x308c), 0x00000000002 },
	{ CCI_REG8(0x392f), 0x2 },
	{ CCI_REG8(0x3930), 0x2 },
	{ CCI_REG8(0x3924), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3925), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c08), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c09), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c31), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c32), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
};

static const struct cci_reg_sequence s5k5e9_2592x1944[] = {
	{ CCI_REG8(0x0136), 0x00000000000000000000000002 },
	{ CCI_REG8(0x0137), 0x2 },
	{ CCI_REG8(0x0301), 0x000002 },
	{ CCI_REG8(0x0305), 0x00002 },
	{ CCI_REG8(0x0306), 0x2 },
	{ CCI_REG8(0x0307), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c1f), 0x2 },
	{ CCI_REG8(0x030d), 0x00002 },
	{ CCI_REG8(0x030e), 0x2 },
	{ CCI_REG8(0x030f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c17), 0x2 },
	{ CCI_REG8(0x0340), 0x0000002 },
	{ CCI_REG8(0x0341), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0342), 0x000000000002 },
	{ CCI_REG8(0x0343), 0x0000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0344), 0x2 },
	{ CCI_REG8(0x0345), 0x00000002 },
	{ CCI_REG8(0x0346), 0x2 },
	{ CCI_REG8(0x0347), 0x00000002 },
	{ CCI_REG8(0x0348), 0x0000000002 },
	{ CCI_REG8(0x0349), 0x000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034a), 0x0000002 },
	{ CCI_REG8(0x034b), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034c), 0x0000000002 },
	{ CCI_REG8(0x034d), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x034e), 0x0000002 },
	{ CCI_REG8(0x034f), 0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0381), 0x2 },
	{ CCI_REG8(0x0383), 0x2 },
	{ CCI_REG8(0x0385), 0x2 },
	{ CCI_REG8(0x0387), 0x2 },
	{ CCI_REG8(0x0900), 0x2 },
	{ CCI_REG8(0x0901), 0x2 },
	{ CCI_REG8(0x0204), 0x2 },
	{ CCI_REG8(0x0205), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x0200), 0x00000000002 },
	{ CCI_REG8(0x0201), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0202), 0x2 },
	{ CCI_REG8(0x0203), 0x02 },
	{ CCI_REG8(0x0820), 0x002 },
	{ CCI_REG8(0x0821), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0822), 0x2 },
	{ CCI_REG8(0x0823), 0x2 },
	{ CCI_REG8(0x0112), 0x0000000002 },
	{ CCI_REG8(0x0113), 0x0000000002 },
	{ CCI_REG8(0x0114), 0x2 },
	{ CCI_REG8(0x3929), 0x000000000000002 },
	{ CCI_REG8(0x0101), 0x2 },
	{ CCI_REG8(0x30b8), 0x000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30ba), 0x0000000000000000000000000000000000000000000002 },
};

static const struct cci_reg_sequence s5k5e9_1296x972[] = {
	{ CCI_REG8(0x0136), 0x00000000000000000000000002 },
	{ CCI_REG8(0x0137), 0x2 },
	{ CCI_REG8(0x0301), 0x000002 },
	{ CCI_REG8(0x0305), 0x00002 },
	{ CCI_REG8(0x0306), 0x2 },
	{ CCI_REG8(0x0307), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c1f), 0x2 },
	{ CCI_REG8(0x030d), 0x00002 },
	{ CCI_REG8(0x030e), 0x2 },
	{ CCI_REG8(0x030f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x3c17), 0x2 },
	{ CCI_REG8(0x0340), 0x002 },
	{ CCI_REG8(0x0341), 0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0342), 0x000000000002 },
	{ CCI_REG8(0x0343), 0x0000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0344), 0x2 },
	{ CCI_REG8(0x0345), 0x00000002 },
	{ CCI_REG8(0x0346), 0x2 },
	{ CCI_REG8(0x0347), 0x00000002 },
	{ CCI_REG8(0x0348), 0x0000000002 },
	{ CCI_REG8(0x0349), 0x000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034a), 0x0000002 },
	{ CCI_REG8(0x034b), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x034c), 0x00002 },
	{ CCI_REG8(0x034d), 0x0000000000000002 },
	{ CCI_REG8(0x034e), 0x002 },
	{ CCI_REG8(0x034f), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0381), 0x2 },
	{ CCI_REG8(0x0383), 0x2 },
	{ CCI_REG8(0x0385), 0x2 },
	{ CCI_REG8(0x0387), 0x002 },
	{ CCI_REG8(0x0900), 0x2 },
	{ CCI_REG8(0x0901), 0x0000000000000000000000000000000002 },
	{ CCI_REG8(0x0204), 0x2 },
	{ CCI_REG8(0x0205), 0x00000000000000000000000000000002 },
	{ CCI_REG8(0x0200), 0x00000000002 },
	{ CCI_REG8(0x0201), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0202), 0x2 },
	{ CCI_REG8(0x0203), 0x02 },
	{ CCI_REG8(0x0820), 0x002 },
	{ CCI_REG8(0x0821), 0x000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x0822), 0x2 },
	{ CCI_REG8(0x0823), 0x2 },
	{ CCI_REG8(0x0112), 0x0000000002 },
	{ CCI_REG8(0x0113), 0x0000000002 },
	{ CCI_REG8(0x0114), 0x2 },
	{ CCI_REG8(0x3929), 0x000000000000002 },
	{ CCI_REG8(0x0101), 0x2 },
	{ CCI_REG8(0x30b8), 0x000000000000000000000000000000000000000002 },
	{ CCI_REG8(0x30ba), 0x0000000000000000000000000000000000000000000002 },
};

struct s5k_mode {
	u32 width;
	u32 height;
	u32 fll;	/* frame length lines at the mode's highest rate */
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k_chip {
	const char *name;
	u16 id;
	u64 pixel_rate;
	s64 link_freq;
	unsigned int lanes;
	u32 llp;		/* line length in pixel clocks */
	struct v4l2_rect active;	/* read-out area of the full mode */
	u32 native_width;
	u32 native_height;
	u32 exposure_min;
	u32 exposure_margin;
	bool fw_start;
	const struct cci_reg_sequence *global;
	unsigned int num_global;
	const struct s5k_mode *modes;
	unsigned int num_modes;
	const char * const *supplies;
	const unsigned int *supply_delay_us;
	unsigned int num_supplies;
};

struct s5k {
	struct device *dev;
	struct regmap *regmap;
	const struct s5k_chip *chip;
	struct clk *clk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[3];
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	u64 again_min;
	u64 again_max;
};

static const char * const s5k4ha_supplies[] = { "vana", "vdig", "vio" };
static const unsigned int s5k4ha_supply_delay_us[] = { 0, 0, 2000 };
static const char * const s5k5e9_supplies[] = { "vdig", "vana", "vio" };
static const unsigned int s5k5e9_supply_delay_us[] = { 500, 500, 500 };

static const struct s5k_mode s5k4ha_modes[] = {
	{ 3264, 2448, 2538, s5k4ha_3264x2448, ARRAY_SIZE(s5k4ha_3264x2448) },
	{ 1632, 1224, 2538, s5k4ha_1632x1224, ARRAY_SIZE(s5k4ha_1632x1224) },
};

static const struct s5k_mode s5k5e9_modes[] = {
	{ 2592, 1944, 2030, s5k5e9_2592x1944, ARRAY_SIZE(s5k5e9_2592x1944) },
	{ 1296, 972, 1018, s5k5e9_1296x972, ARRAY_SIZE(s5k5e9_1296x972) },
};

static const struct s5k_chip s5k4ha_chip = {
	.name = "s5k4ha",
	.id = 0x48ab,
	/* 26 MHz / 5 * 108 / 4, two pixels per clock */
	.pixel_rate = 280800000,
	/* 671 Mbps per lane */
	.link_freq = 335400000,
	.lanes = 4,
	.llp = 3688,
	.active = { 8, 8, 3264, 2448 },
	.native_width = 3280,
	.native_height = 2464,
	.exposure_min = 1,
	.exposure_margin = 4,
	.fw_start = true,
	.global = s5k4ha_global,
	.num_global = ARRAY_SIZE(s5k4ha_global),
	.modes = s5k4ha_modes,
	.num_modes = ARRAY_SIZE(s5k4ha_modes),
	.supplies = s5k4ha_supplies,
	.supply_delay_us = s5k4ha_supply_delay_us,
	.num_supplies = ARRAY_SIZE(s5k4ha_supplies),
};

static const struct s5k_chip s5k5e9_chip = {
	.name = "s5k5e9",
	.id = 0x559b,
	/* 26 MHz / 5 * 109 / 6, two pixels per clock */
	.pixel_rate = 188933332,
	/* 936 Mbps per lane */
	.link_freq = 468000000,
	.lanes = 2,
	.llp = 3112,
	.active = { 8, 8, 2592, 1944 },
	.native_width = 2608,
	.native_height = 1960,
	.exposure_min = 2,
	.exposure_margin = 5,
	.global = s5k5e9_global,
	.num_global = ARRAY_SIZE(s5k5e9_global),
	.modes = s5k5e9_modes,
	.num_modes = ARRAY_SIZE(s5k5e9_modes),
	.supplies = s5k5e9_supplies,
	.supply_delay_us = s5k5e9_supply_delay_us,
	.num_supplies = ARRAY_SIZE(s5k5e9_supplies),
};

static const char * const s5k_test_patterns[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

/* Without flips both sensors start their read-out on a green-red row. */
static const u32 s5k_codes[2][2] = {
	/* [vflip][hflip] */
	{ MEDIA_BUS_FMT_SGRBG10_1X10, MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, MEDIA_BUS_FMT_SGBRG10_1X10 },
};

static inline struct s5k *to_s5k(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k, sd);
}

static u32 s5k_code(struct s5k *s5k)
{
	return s5k_codes[s5k->vflip->val][s5k->hflip->val];
}

static const struct s5k_mode *s5k_find_mode(struct s5k *s5k, u32 w, u32 h)
{
	return v4l2_find_nearest_size(s5k->chip->modes, s5k->chip->num_modes,
				      width, height, w, h);
}

static void s5k_update_limits(struct s5k *s5k, const struct s5k_mode *mode)
{
	const struct s5k_chip *chip = s5k->chip;
	u32 vblank_min = mode->fll - mode->height;
	u32 hblank = chip->llp - mode->width;

	__v4l2_ctrl_modify_range(s5k->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(s5k->vblank, vblank_min,
				 S5K_FLL_MAX - mode->height, 1, vblank_min);
	__v4l2_ctrl_s_ctrl(s5k->vblank, vblank_min);
}

static int s5k_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k *s5k = container_of(ctrl->handler, struct s5k, ctrls);
	const struct v4l2_mbus_framefmt *fmt;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&s5k->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 max = fmt->height + ctrl->val - s5k->chip->exposure_margin;

		__v4l2_ctrl_modify_range(s5k->exposure, s5k->exposure->minimum,
					 max, 1, min(s5k->exposure->default_value,
						     (s64)max));
	}

	if (!pm_runtime_get_if_in_use(s5k->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(s5k->regmap, S5K_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(s5k->regmap, S5K_REG_AGAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(s5k->regmap, S5K_REG_FLL, fmt->height + ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(s5k->regmap, S5K_REG_TEST_PATTERN, ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(s5k->regmap, S5K_REG_ORIENTATION,
			  (s5k->hflip->val ? S5K_HFLIP : 0) |
			  (s5k->vflip->val ? S5K_VFLIP : 0), &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k_ctrl_ops = {
	.s_ctrl = s5k_set_ctrl,
};

static int s5k_init_controls(struct s5k *s5k)
{
	const struct s5k_chip *chip = s5k->chip;
	const struct s5k_mode *mode = &chip->modes[0];
	struct v4l2_ctrl_handler *hdl = &s5k->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u32 vblank = mode->fll - mode->height;
	int ret;

	ret = v4l2_fwnode_device_parse(s5k->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 12);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &s5k_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, &chip->link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  chip->pixel_rate, chip->pixel_rate, 1,
			  chip->pixel_rate);
	s5k->hblank = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_HBLANK,
					chip->llp - mode->width,
					chip->llp - mode->width, 1,
					chip->llp - mode->width);
	if (s5k->hblank)
		s5k->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s5k->vblank = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_VBLANK,
					vblank, S5K_FLL_MAX - mode->height, 1,
					vblank);
	s5k->exposure = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops,
					  V4L2_CID_EXPOSURE,
					  chip->exposure_min,
					  mode->fll - chip->exposure_margin, 1,
					  (mode->fll - chip->exposure_margin) / 2);
	v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  s5k->again_min, s5k->again_max, 1, s5k->again_min);
	v4l2_ctrl_new_std_menu_items(hdl, &s5k_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k_test_patterns) - 1, 0, 0,
				     s5k_test_patterns);
	s5k->hflip = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_HFLIP,
				       0, 1, 1, 0);
	if (s5k->hflip)
		s5k->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	s5k->vflip = v4l2_ctrl_new_std(hdl, &s5k_ctrl_ops, V4L2_CID_VFLIP,
				       0, 1, 1, 0);
	if (s5k->vflip)
		s5k->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	v4l2_ctrl_new_fwnode_properties(hdl, &s5k_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	s5k->sd.ctrl_handler = hdl;
	return 0;
}

static int s5k_start_streaming(struct s5k *s5k,
			       struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	const struct s5k_mode *mode = s5k_find_mode(s5k, fmt->width,
						    fmt->height);
	int ret;

	ret = pm_runtime_resume_and_get(s5k->dev);
	if (ret)
		return ret;

	if (s5k->chip->fw_start) {
		/* Start the sensor's own firmware before the global table. */
		cci_write(s5k->regmap, S5K_REG_FW_PAGE, 0x4000, &ret);
		cci_write(s5k->regmap, S5K_REG_FW_START, 0x0001, &ret);
		if (ret)
			goto err_put;
		usleep_range(3000, 3500);
	}
	ret = cci_multi_reg_write(s5k->regmap, s5k->chip->global,
				  s5k->chip->num_global, NULL);
	if (ret)
		goto err_put;
	ret = cci_multi_reg_write(s5k->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_put;
	ret = __v4l2_ctrl_handler_setup(&s5k->ctrls);
	if (ret)
		goto err_put;
	ret = cci_write(s5k->regmap, S5K_REG_MODE_SELECT, S5K_MODE_STREAMING,
			NULL);
	if (ret)
		goto err_put;

	__v4l2_ctrl_grab(s5k->hflip, true);
	__v4l2_ctrl_grab(s5k->vflip, true);
	return 0;

err_put:
	pm_runtime_put(s5k->dev);
	return ret;
}

static void s5k_stop_streaming(struct s5k *s5k)
{
	int ret;

	ret = cci_write(s5k->regmap, S5K_REG_MODE_SELECT, 0, NULL);
	if (ret)
		dev_err(s5k->dev, "failed to stop streaming: %d\n", ret);
	__v4l2_ctrl_grab(s5k->hflip, false);
	__v4l2_ctrl_grab(s5k->vflip, false);
	pm_runtime_put(s5k->dev);
}

static int s5k_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k *s5k = to_s5k(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (enable)
		ret = s5k_start_streaming(s5k, state);
	else
		s5k_stop_streaming(s5k);
	v4l2_subdev_unlock_state(state);
	return ret;
}

static void s5k_fill_format(struct s5k *s5k, struct v4l2_mbus_framefmt *fmt,
			    const struct s5k_mode *mode)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = s5k_code(s5k);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = s5k_code(to_s5k(sd));
	return 0;
}

static int s5k_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k *s5k = to_s5k(sd);
	const struct s5k_mode *mode;

	if (fse->index >= s5k->chip->num_modes ||
	    fse->code != s5k_code(s5k))
		return -EINVAL;
	mode = &s5k->chip->modes[fse->index];
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;
	return 0;
}

static int s5k_set_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k *s5k = to_s5k(sd);
	const struct s5k_mode *mode;

	mode = s5k_find_mode(s5k, fmt->format.width, fmt->format.height);
	s5k_fill_format(s5k, &fmt->format, mode);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		s5k_update_limits(s5k, mode);
	return 0;
}

static int s5k_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_selection *sel)
{
	const struct s5k_chip *chip = to_s5k(sd)->chip;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		/* Every mode reads out the full active area, binned or not. */
		sel->r = chip->active;
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = chip->native_width;
		sel->r.height = chip->native_height;
		return 0;
	}
	return -EINVAL;
}

static int s5k_init_state(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct s5k *s5k = to_s5k(sd);

	s5k_fill_format(s5k, v4l2_subdev_state_get_format(state, 0),
			&s5k->chip->modes[0]);
	return 0;
}

static const struct v4l2_subdev_video_ops s5k_video_ops = {
	.s_stream = s5k_set_stream,
};

static const struct v4l2_subdev_pad_ops s5k_pad_ops = {
	.enum_mbus_code = s5k_enum_mbus_code,
	.enum_frame_size = s5k_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k_set_format,
	.get_selection = s5k_get_selection,
};

static const struct v4l2_subdev_ops s5k_subdev_ops = {
	.video = &s5k_video_ops,
	.pad = &s5k_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k_internal_ops = {
	.init_state = s5k_init_state,
};

static int s5k_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k *s5k = to_s5k(sd);
	const struct s5k_chip *chip = s5k->chip;
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(s5k->reset, 1);
	for (i = 0; i < chip->num_supplies; i++) {
		ret = regulator_enable(s5k->supplies[i].consumer);
		if (ret)
			goto err_supplies;
		fsleep(chip->supply_delay_us[i]);
	}
	ret = clk_prepare_enable(s5k->clk);
	if (ret)
		goto err_supplies;
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(s5k->reset, 0);
	usleep_range(3000, 3500);
	return 0;

err_supplies:
	while (i--)
		regulator_disable(s5k->supplies[i].consumer);
	return ret;
}

static int s5k_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k *s5k = to_s5k(sd);
	unsigned int i;

	gpiod_set_value_cansleep(s5k->reset, 1);
	usleep_range(1500, 2000);
	clk_disable_unprepare(s5k->clk);
	for (i = s5k->chip->num_supplies; i--;)
		regulator_disable(s5k->supplies[i].consumer);
	return 0;
}

static int s5k_identify(struct s5k *s5k)
{
	u64 id, val;
	int ret;

	ret = cci_read(s5k->regmap, S5K_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(s5k->dev, ret, "failed to read chip id\n");
	if (id != s5k->chip->id)
		return dev_err_probe(s5k->dev, -ENODEV,
				     "chip id %#llx, expected %#x\n", id,
				     s5k->chip->id);

	s5k->again_min = S5K_AGAIN_ONE;
	s5k->again_max = 16 * S5K_AGAIN_ONE;
	if (!cci_read(s5k->regmap, S5K_REG_AGAIN_MIN, &val, NULL) && val)
		s5k->again_min = val;
	if (!cci_read(s5k->regmap, S5K_REG_AGAIN_MAX, &val, NULL) &&
	    val > s5k->again_min)
		s5k->again_max = val;
	dev_info(s5k->dev, "%s, analogue gain %#llx-%#llx\n", s5k->chip->name,
		 s5k->again_min, s5k->again_max);
	return 0;
}

static int s5k_check_endpoint(struct s5k *s5k)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fw;
	unsigned long bitmap;
	int ret;

	fw = fwnode_graph_get_next_endpoint(dev_fwnode(s5k->dev), NULL);
	if (!fw)
		return dev_err_probe(s5k->dev, -EINVAL, "no endpoint\n");
	ret = v4l2_fwnode_endpoint_alloc_parse(fw, &ep);
	fwnode_handle_put(fw);
	if (ret)
		return dev_err_probe(s5k->dev, ret, "invalid endpoint\n");

	if (ep.bus.mipi_csi2.num_data_lanes != s5k->chip->lanes) {
		ret = dev_err_probe(s5k->dev, -EINVAL,
				    "%u data lanes are required\n",
				    s5k->chip->lanes);
		goto out;
	}
	ret = v4l2_link_freq_to_bitmap(s5k->dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       &s5k->chip->link_freq, 1, &bitmap);
out:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int s5k_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k *s5k;
	unsigned int i;
	int ret;

	s5k = devm_kzalloc(dev, sizeof(*s5k), GFP_KERNEL);
	if (!s5k)
		return -ENOMEM;
	s5k->dev = dev;
	s5k->chip = device_get_match_data(dev);

	ret = s5k_check_endpoint(s5k);
	if (ret)
		return ret;

	s5k->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k->regmap),
				     "failed to init CCI\n");

	s5k->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s5k->clk))
		return dev_err_probe(dev, PTR_ERR(s5k->clk), "no clock\n");
	if (clk_get_rate(s5k->clk) != S5K_MCLK_RATE) {
		ret = clk_set_rate(s5k->clk, S5K_MCLK_RATE);
		if (ret)
			return dev_err_probe(dev, ret, "cannot set 26 MHz\n");
	}

	s5k->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k->reset))
		return dev_err_probe(dev, PTR_ERR(s5k->reset), "no reset\n");

	for (i = 0; i < s5k->chip->num_supplies; i++)
		s5k->supplies[i].supply = s5k->chip->supplies[i];
	ret = devm_regulator_bulk_get(dev, s5k->chip->num_supplies,
				      s5k->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	v4l2_i2c_subdev_init(&s5k->sd, client, &s5k_subdev_ops);
	/* Userspace identifies the sensor model by the entity name. */
	v4l2_i2c_subdev_set_name(&s5k->sd, client, s5k->chip->name, NULL);

	ret = s5k_power_on(dev);
	if (ret)
		return ret;
	ret = s5k_identify(s5k);
	if (ret)
		goto err_power;

	ret = s5k_init_controls(s5k);
	if (ret)
		goto err_power;

	s5k->sd.internal_ops = &s5k_internal_ops;
	s5k->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s5k->sd.entity, 1, &s5k->pad);
	if (ret)
		goto err_ctrls;

	s5k->sd.state_lock = s5k->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&s5k->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(dev);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&s5k->sd);
err_entity:
	media_entity_cleanup(&s5k->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s5k->ctrls);
err_power:
	s5k_power_off(dev);
	return ret;
}

static void s5k_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k *s5k = to_s5k(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k_pm_ops, s5k_power_off, s5k_power_on,
				 NULL);

static const struct of_device_id s5k_of_match[] = {
	{ .compatible = "samsung,s5k4ha", .data = &s5k4ha_chip },
	{ .compatible = "samsung,s5k5e9", .data = &s5k5e9_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k_of_match);

static struct i2c_driver s5k_driver = {
	.probe = s5k_probe,
	.remove = s5k_remove,
	.driver = {
		.name = "s5k-smia",
		.of_match_table = s5k_of_match,
		.pm = pm_ptr(&s5k_pm_ops),
	},
};
module_i2c_driver(s5k_driver);

MODULE_DESCRIPTION("Samsung S5K4HA and S5K5E9 camera sensors");
MODULE_LICENSE("GPL");

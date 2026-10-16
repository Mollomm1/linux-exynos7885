// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Dávid Virág <virag.david003@gmail.com>
 * Author: Dávid Virág <virag.david003@gmail.com>
 *
 * Common Clock Framework support for Exynos7885 SoC.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include <dt-bindings/clock/exynos7885.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/* NOTE: Must be equal to the last clock ID increased by one */
#define CLKS_NR_TOP			(CLK_DOUT_CIS_CLK1 + 1)
#define CLKS_NR_CORE			(CLK_GOUT_ADC_S1 + 1)
#define CLKS_NR_PERI			(CLK_GOUT_TMU_PCLK + 1)
#define CLKS_NR_FSYS			(CLK_FSYS_USB30DRD_REF_CLK + 1)
#define CLKS_NR_DISPAUD			(CLK_GOUT_SMMU_DPU_CLK + 1)
#define CLKS_NR_G3D			(CLK_DOUT_G3D_BUSP + 1)
#define CLKS_NR_CPUCL			(CLK_DOUT_CPUCL_CPU + 1)

static bool exynos7885_enable_dispaud;

/* ---- CMU_TOP ------------------------------------------------------------- */

/* Register Offset definitions for CMU_TOP (0x12060000) */
#define PLL_LOCKTIME_PLL_SHARED0		0x0000
#define PLL_LOCKTIME_PLL_SHARED1		0x0004
#define PLL_CON0_PLL_SHARED0			0x0100
#define PLL_CON0_PLL_SHARED1			0x0120
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0		0x1008
#define CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1		0x100c
#define CLK_CON_MUX_MUX_CLKCMU_CORE_BUS		0x1014
#define CLK_CON_MUX_MUX_CLKCMU_CORE_CCI		0x1018
#define CLK_CON_MUX_MUX_CLKCMU_CORE_G3D		0x101c
#define CLK_CON_MUX_MUX_CLKCMU_DISPAUD_CPU	0x1024
#define CLK_CON_MUX_MUX_CLKCMU_FSYS_BUS		0x1028
#define CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_CARD	0x102c
#define CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_EMBD	0x1030
#define CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_SDIO	0x1034
#define CLK_CON_MUX_MUX_CLKCMU_FSYS_USB30DRD	0x1038
#define CLK_CON_MUX_MUX_CLKCMU_PERI_BUS		0x1058
#define CLK_CON_MUX_MUX_CLKCMU_PERI_SPI0	0x105c
#define CLK_CON_MUX_MUX_CLKCMU_PERI_SPI1	0x1060
#define CLK_CON_MUX_MUX_CLKCMU_PERI_UART0	0x1064
#define CLK_CON_MUX_MUX_CLKCMU_PERI_UART1	0x1068
#define CLK_CON_MUX_MUX_CLKCMU_PERI_UART2	0x106c
#define CLK_CON_MUX_MUX_CLKCMU_PERI_USI0	0x1070
#define CLK_CON_MUX_MUX_CLKCMU_PERI_USI1	0x1074
#define CLK_CON_MUX_MUX_CLKCMU_PERI_USI2	0x1078
#define CLK_CON_DIV_CLKCMU_CIS_CLK0		0x1810
#define CLK_CON_DIV_CLKCMU_CIS_CLK1		0x1814
#define CLK_CON_DIV_CLKCMU_CORE_BUS		0x181c
#define CLK_CON_DIV_CLKCMU_CORE_CCI		0x1820
#define CLK_CON_DIV_CLKCMU_CORE_G3D		0x1824
#define CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH	0x182c
#define CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH	0x1830
#define CLK_CON_DIV_CLKCMU_CPUCL2_SWITCH	0x1834
#define CLK_CON_DIV_CLKCMU_DISPAUD_CPU		0x1840
#define CLK_CON_DIV_CLKCMU_FSYS_BUS		0x1844
#define CLK_CON_DIV_CLKCMU_FSYS_MMC_CARD	0x1848
#define CLK_CON_DIV_CLKCMU_FSYS_MMC_EMBD	0x184c
#define CLK_CON_DIV_CLKCMU_FSYS_MMC_SDIO	0x1850
#define CLK_CON_DIV_CLKCMU_FSYS_USB30DRD	0x1854
#define CLK_CON_DIV_CLKCMU_G3D_SWITCH		0x1858
#define CLK_CON_DIV_CLKCMU_PERI_BUS		0x1874
#define CLK_CON_DIV_CLKCMU_PERI_SPI0		0x1878
#define CLK_CON_DIV_CLKCMU_PERI_SPI1		0x187c
#define CLK_CON_DIV_CLKCMU_PERI_UART0		0x1880
#define CLK_CON_DIV_CLKCMU_PERI_UART1		0x1884
#define CLK_CON_DIV_CLKCMU_PERI_UART2		0x1888
#define CLK_CON_DIV_CLKCMU_PERI_USI0		0x188c
#define CLK_CON_DIV_CLKCMU_PERI_USI1		0x1890
#define CLK_CON_DIV_CLKCMU_PERI_USI2		0x1894
#define CLK_CON_DIV_PLL_SHARED0_DIV2		0x189c
#define CLK_CON_DIV_PLL_SHARED0_DIV3		0x18a0
#define CLK_CON_DIV_PLL_SHARED0_DIV4		0x18a4
#define CLK_CON_DIV_PLL_SHARED0_DIV5		0x18a8
#define CLK_CON_DIV_PLL_SHARED1_DIV2		0x18ac
#define CLK_CON_DIV_PLL_SHARED1_DIV3		0x18b0
#define CLK_CON_DIV_PLL_SHARED1_DIV4		0x18b4
#define CLK_CON_GAT_GATE_CLKCMUC_PERI_UART1	0x2004
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK0	0x2010
#define CLK_CON_GAT_GATE_CLKCMU_CIS_CLK1	0x2014
#define CLK_CON_GAT_GATE_CLKCMU_CORE_BUS	0x201c
#define CLK_CON_GAT_GATE_CLKCMU_CORE_CCI	0x2020
#define CLK_CON_GAT_GATE_CLKCMU_CORE_G3D	0x2024
#define CLK_CON_GAT_GATE_CLKCMU_CPUCL0_SWITCH	0x202c
#define CLK_CON_GAT_GATE_CLKCMU_CPUCL1_SWITCH	0x2030
#define CLK_CON_GAT_GATE_CLKCMU_CPUCL2_SWITCH	0x2034
#define CLK_CON_GAT_GATE_CLKCMU_DISPAUD_VCLK	0x2040
#define CLK_CON_GAT_GATE_CLKCMU_FSYS_BUS	0x2044
#define CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_CARD	0x2048
#define CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_EMBD	0x204c
#define CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_SDIO	0x2050
#define CLK_CON_GAT_GATE_CLKCMU_FSYS_USB30DRD	0x2054
#define CLK_CON_GAT_GATE_CLKCMU_G3D_SWITCH	0x2058
#define CLK_CON_GAT_GATE_CLKCMU_PERI_BUS	0x207c
#define CLK_CON_GAT_GATE_CLKCMU_PERI_SPI0	0x2080
#define CLK_CON_GAT_GATE_CLKCMU_PERI_SPI1	0x2084
#define CLK_CON_GAT_GATE_CLKCMU_PERI_UART0	0x2088
#define CLK_CON_GAT_GATE_CLKCMU_PERI_UART2	0x208c
#define CLK_CON_GAT_GATE_CLKCMU_PERI_USI0	0x2090
#define CLK_CON_GAT_GATE_CLKCMU_PERI_USI1	0x2094
#define CLK_CON_GAT_GATE_CLKCMU_PERI_USI2	0x2098

static const unsigned long top_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_SHARED0,
	PLL_LOCKTIME_PLL_SHARED1,
	PLL_CON0_PLL_SHARED0,
	PLL_CON0_PLL_SHARED1,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0,
	CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1,
	CLK_CON_MUX_MUX_CLKCMU_CORE_BUS,
	CLK_CON_MUX_MUX_CLKCMU_CORE_CCI,
	CLK_CON_MUX_MUX_CLKCMU_CORE_G3D,
	CLK_CON_MUX_MUX_CLKCMU_DISPAUD_CPU,
	CLK_CON_MUX_MUX_CLKCMU_FSYS_BUS,
	CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_CARD,
	CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_EMBD,
	CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_SDIO,
	CLK_CON_MUX_MUX_CLKCMU_FSYS_USB30DRD,
	CLK_CON_MUX_MUX_CLKCMU_PERI_BUS,
	CLK_CON_MUX_MUX_CLKCMU_PERI_SPI0,
	CLK_CON_MUX_MUX_CLKCMU_PERI_SPI1,
	CLK_CON_MUX_MUX_CLKCMU_PERI_UART0,
	CLK_CON_MUX_MUX_CLKCMU_PERI_UART1,
	CLK_CON_MUX_MUX_CLKCMU_PERI_UART2,
	CLK_CON_MUX_MUX_CLKCMU_PERI_USI0,
	CLK_CON_MUX_MUX_CLKCMU_PERI_USI1,
	CLK_CON_MUX_MUX_CLKCMU_PERI_USI2,
	CLK_CON_DIV_CLKCMU_CIS_CLK0,
	CLK_CON_DIV_CLKCMU_CIS_CLK1,
	CLK_CON_DIV_CLKCMU_CORE_BUS,
	CLK_CON_DIV_CLKCMU_CORE_CCI,
	CLK_CON_DIV_CLKCMU_CORE_G3D,
	CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH,
	CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH,
	CLK_CON_DIV_CLKCMU_CPUCL2_SWITCH,
	CLK_CON_DIV_CLKCMU_DISPAUD_CPU,
	CLK_CON_DIV_CLKCMU_FSYS_BUS,
	CLK_CON_DIV_CLKCMU_FSYS_MMC_CARD,
	CLK_CON_DIV_CLKCMU_FSYS_MMC_EMBD,
	CLK_CON_DIV_CLKCMU_FSYS_MMC_SDIO,
	CLK_CON_DIV_CLKCMU_FSYS_USB30DRD,
	CLK_CON_DIV_CLKCMU_G3D_SWITCH,
	CLK_CON_DIV_CLKCMU_PERI_BUS,
	CLK_CON_DIV_CLKCMU_PERI_SPI0,
	CLK_CON_DIV_CLKCMU_PERI_SPI1,
	CLK_CON_DIV_CLKCMU_PERI_UART0,
	CLK_CON_DIV_CLKCMU_PERI_UART1,
	CLK_CON_DIV_CLKCMU_PERI_UART2,
	CLK_CON_DIV_CLKCMU_PERI_USI0,
	CLK_CON_DIV_CLKCMU_PERI_USI1,
	CLK_CON_DIV_CLKCMU_PERI_USI2,
	CLK_CON_DIV_PLL_SHARED0_DIV2,
	CLK_CON_DIV_PLL_SHARED0_DIV3,
	CLK_CON_DIV_PLL_SHARED0_DIV4,
	CLK_CON_DIV_PLL_SHARED0_DIV5,
	CLK_CON_DIV_PLL_SHARED1_DIV2,
	CLK_CON_DIV_PLL_SHARED1_DIV3,
	CLK_CON_DIV_PLL_SHARED1_DIV4,
	CLK_CON_GAT_GATE_CLKCMUC_PERI_UART1,
	CLK_CON_GAT_GATE_CLKCMU_CIS_CLK0,
	CLK_CON_GAT_GATE_CLKCMU_CIS_CLK1,
	CLK_CON_GAT_GATE_CLKCMU_CORE_BUS,
	CLK_CON_GAT_GATE_CLKCMU_CORE_CCI,
	CLK_CON_GAT_GATE_CLKCMU_CORE_G3D,
	CLK_CON_GAT_GATE_CLKCMU_FSYS_BUS,
	CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_CARD,
	CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_EMBD,
	CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_SDIO,
	CLK_CON_GAT_GATE_CLKCMU_FSYS_USB30DRD,
	CLK_CON_GAT_GATE_CLKCMU_PERI_BUS,
	CLK_CON_GAT_GATE_CLKCMU_PERI_SPI0,
	CLK_CON_GAT_GATE_CLKCMU_PERI_SPI1,
	CLK_CON_GAT_GATE_CLKCMU_PERI_UART0,
	CLK_CON_GAT_GATE_CLKCMU_PERI_UART2,
	CLK_CON_GAT_GATE_CLKCMU_PERI_USI0,
	CLK_CON_GAT_GATE_CLKCMU_PERI_USI1,
	CLK_CON_GAT_GATE_CLKCMU_PERI_USI2,
};

static const struct samsung_pll_clock top_pll_clks[] __initconst = {
	PLL(pll_1417x, CLK_FOUT_SHARED0_PLL, "fout_shared0_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED0, PLL_CON0_PLL_SHARED0,
	    NULL),
	PLL(pll_1417x, CLK_FOUT_SHARED1_PLL, "fout_shared1_pll", "oscclk",
	    PLL_LOCKTIME_PLL_SHARED1, PLL_CON0_PLL_SHARED1,
	    NULL),
};

/* List of parent clocks for Muxes in CMU_TOP */
PNAME(mout_shared0_pll_p)	= { "oscclk", "fout_shared0_pll" };
PNAME(mout_shared1_pll_p)	= { "oscclk", "fout_shared1_pll" };

/* List of parent clocks for Muxes in CMU_TOP: for CMU_CORE */
PNAME(mout_core_bus_p)		= { "dout_shared0_div2", "dout_shared1_div2",
				    "dout_shared0_div3", "dout_shared0_div3" };
PNAME(mout_core_cci_p)		= { "dout_shared0_div2", "dout_shared1_div2",
				    "dout_shared0_div3", "dout_shared0_div3" };
PNAME(mout_core_g3d_p)		= { "dout_shared0_div2", "dout_shared1_div2",
				    "dout_shared0_div3", "dout_shared0_div3" };

/* List of parent clocks for Muxes in CMU_TOP: for CMU_PERI */
PNAME(mout_peri_bus_p)		= { "dout_shared0_div4", "dout_shared1_div4" };
PNAME(mout_peri_spi0_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_spi1_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_uart0_p)	= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_uart1_p)	= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_uart2_p)	= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_usi0_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_usi1_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_peri_usi2_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_cis_clk0_p)		= { "oscclk", "dout_shared0_div4" };
PNAME(mout_cis_clk1_p)		= { "oscclk", "dout_shared0_div4" };

/* List of parent clocks for Muxes in CMU_TOP: for CMU_FSYS */
PNAME(mout_fsys_bus_p)		= { "dout_shared0_div2", "dout_shared1_div2" };
PNAME(mout_fsys_mmc_card_p)	= { "dout_shared0_div2", "dout_shared1_div2" };
PNAME(mout_fsys_mmc_embd_p)	= { "dout_shared0_div2", "dout_shared1_div2" };
PNAME(mout_fsys_mmc_sdio_p)	= { "dout_shared0_div2", "dout_shared1_div2" };
PNAME(mout_fsys_usb30drd_p)	= { "dout_shared0_div4", "dout_shared1_div4" };
PNAME(mout_dispaud_cpu_p)	= { "dout_shared0_div2", "dout_shared1_div2" };

static const struct samsung_mux_clock top_mux_clks[] __initconst = {
	/* TOP */
	MUX(CLK_MOUT_SHARED0_PLL, "mout_shared0_pll", mout_shared0_pll_p,
	    PLL_CON0_PLL_SHARED0, 4, 1),
	MUX(CLK_MOUT_SHARED1_PLL, "mout_shared1_pll", mout_shared1_pll_p,
	    PLL_CON0_PLL_SHARED1, 4, 1),
	MUX(CLK_MOUT_DISPAUD_CPU, "mout_dispaud_cpu", mout_dispaud_cpu_p,
	    CLK_CON_MUX_MUX_CLKCMU_DISPAUD_CPU, 0, 1),

	/* CORE */
	MUX(CLK_MOUT_CORE_BUS, "mout_core_bus", mout_core_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_CORE_BUS, 0, 2),
	MUX(CLK_MOUT_CORE_CCI, "mout_core_cci", mout_core_cci_p,
	    CLK_CON_MUX_MUX_CLKCMU_CORE_CCI, 0, 2),
	MUX(CLK_MOUT_CORE_G3D, "mout_core_g3d", mout_core_g3d_p,
	    CLK_CON_MUX_MUX_CLKCMU_CORE_G3D, 0, 2),

	/* PERI */
	MUX(CLK_MOUT_PERI_BUS, "mout_peri_bus", mout_peri_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_BUS, 0, 1),
	MUX(CLK_MOUT_PERI_SPI0, "mout_peri_spi0", mout_peri_spi0_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_SPI0, 0, 1),
	MUX(CLK_MOUT_PERI_SPI1, "mout_peri_spi1", mout_peri_spi1_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_SPI1, 0, 1),
	MUX(CLK_MOUT_PERI_UART0, "mout_peri_uart0", mout_peri_uart0_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_UART0, 0, 1),
	MUX(CLK_MOUT_PERI_UART1, "mout_peri_uart1", mout_peri_uart1_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_UART1, 0, 1),
	MUX(CLK_MOUT_PERI_UART2, "mout_peri_uart2", mout_peri_uart2_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_UART2, 0, 1),
	MUX(CLK_MOUT_PERI_USI0, "mout_peri_usi0", mout_peri_usi0_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_USI0, 0, 1),
	MUX(CLK_MOUT_PERI_USI1, "mout_peri_usi1", mout_peri_usi1_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_USI1, 0, 1),
	/* Camera sensor master clocks, 26 MHz from OSCCLK in Samsung's setup */
	MUX(CLK_MOUT_CIS_CLK0, "mout_cis_clk0", mout_cis_clk0_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK0, 0, 1),
	MUX(CLK_MOUT_CIS_CLK1, "mout_cis_clk1", mout_cis_clk1_p,
	    CLK_CON_MUX_MUX_CLKCMU_CIS_CLK1, 0, 1),
	MUX(CLK_MOUT_PERI_USI2, "mout_peri_usi2", mout_peri_usi2_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERI_USI2, 0, 1),

	/* FSYS */
	MUX(CLK_MOUT_FSYS_BUS, "mout_fsys_bus", mout_fsys_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS_BUS, 0, 1),
	MUX(CLK_MOUT_FSYS_MMC_CARD, "mout_fsys_mmc_card", mout_fsys_mmc_card_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_CARD, 0, 1),
	MUX(CLK_MOUT_FSYS_MMC_EMBD, "mout_fsys_mmc_embd", mout_fsys_mmc_embd_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_EMBD, 0, 1),
	MUX(CLK_MOUT_FSYS_MMC_SDIO, "mout_fsys_mmc_sdio", mout_fsys_mmc_sdio_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS_MMC_SDIO, 0, 1),
	MUX(CLK_MOUT_FSYS_USB30DRD, "mout_fsys_usb30drd", mout_fsys_usb30drd_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS_USB30DRD, 0, 1),
};

static const struct samsung_div_clock top_div_clks[] __initconst = {
	/* TOP */
	DIV(CLK_DOUT_SHARED0_DIV2, "dout_shared0_div2", "mout_shared0_pll",
	    CLK_CON_DIV_PLL_SHARED0_DIV2, 0, 1),
	DIV(CLK_DOUT_SHARED0_DIV3, "dout_shared0_div3", "mout_shared0_pll",
	    CLK_CON_DIV_PLL_SHARED0_DIV3, 0, 2),
	DIV(CLK_DOUT_SHARED0_DIV4, "dout_shared0_div4", "dout_shared0_div2",
	    CLK_CON_DIV_PLL_SHARED0_DIV4, 0, 1),
	DIV(CLK_DOUT_SHARED0_DIV5, "dout_shared0_div5", "mout_shared0_pll",
	    CLK_CON_DIV_PLL_SHARED0_DIV5, 0, 3),
	DIV(CLK_DOUT_SHARED1_DIV2, "dout_shared1_div2", "mout_shared1_pll",
	    CLK_CON_DIV_PLL_SHARED1_DIV2, 0, 1),
	DIV(CLK_DOUT_SHARED1_DIV3, "dout_shared1_div3", "mout_shared1_pll",
	    CLK_CON_DIV_PLL_SHARED1_DIV3, 0, 2),
	DIV(CLK_DOUT_SHARED1_DIV4, "dout_shared1_div4", "dout_shared1_div2",
	    CLK_CON_DIV_PLL_SHARED1_DIV4, 0, 1),
	DIV(CLK_DOUT_DISPAUD_CPU, "dout_dispaud_cpu", "gout_dispaud_cpu",
	    CLK_CON_DIV_CLKCMU_DISPAUD_CPU, 0, 3),
	DIV(CLK_DOUT_G3D_SWITCH, "dout_g3d_switch", "gout_g3d_switch",
	    CLK_CON_DIV_CLKCMU_G3D_SWITCH, 0, 3),
	DIV(CLK_DOUT_CPUCL0_SWITCH, "dout_cpucl0_switch", "gout_cpucl0_switch",
	    CLK_CON_DIV_CLKCMU_CPUCL0_SWITCH, 0, 4),
	DIV(CLK_DOUT_CPUCL1_SWITCH, "dout_cpucl1_switch", "gout_cpucl1_switch",
	    CLK_CON_DIV_CLKCMU_CPUCL1_SWITCH, 0, 3),
	DIV(CLK_DOUT_CPUCL2_SWITCH, "dout_cpucl2_switch", "gout_cpucl2_switch",
	    CLK_CON_DIV_CLKCMU_CPUCL2_SWITCH, 0, 3),

	/* CORE */
	DIV(CLK_DOUT_CORE_BUS, "dout_core_bus", "gout_core_bus",
	    CLK_CON_DIV_CLKCMU_CORE_BUS, 0, 3),
	DIV(CLK_DOUT_CORE_CCI, "dout_core_cci", "gout_core_cci",
	    CLK_CON_DIV_CLKCMU_CORE_CCI, 0, 3),
	DIV(CLK_DOUT_CORE_G3D, "dout_core_g3d", "gout_core_g3d",
	    CLK_CON_DIV_CLKCMU_CORE_G3D, 0, 3),

	/* PERI */
	DIV(CLK_DOUT_PERI_BUS, "dout_peri_bus", "gout_peri_bus",
	    CLK_CON_DIV_CLKCMU_PERI_BUS, 0, 4),
	DIV(CLK_DOUT_PERI_SPI0, "dout_peri_spi0", "gout_peri_spi0",
	    CLK_CON_DIV_CLKCMU_PERI_SPI0, 0, 6),
	DIV(CLK_DOUT_PERI_SPI1, "dout_peri_spi1", "gout_peri_spi1",
	    CLK_CON_DIV_CLKCMU_PERI_SPI1, 0, 6),
	DIV(CLK_DOUT_PERI_UART0, "dout_peri_uart0", "gout_peri_uart0",
	    CLK_CON_DIV_CLKCMU_PERI_UART0, 0, 4),
	DIV(CLK_DOUT_PERI_UART1, "dout_peri_uart1", "gout_peri_uart1",
	    CLK_CON_DIV_CLKCMU_PERI_UART1, 0, 4),
	DIV(CLK_DOUT_PERI_UART2, "dout_peri_uart2", "gout_peri_uart2",
	    CLK_CON_DIV_CLKCMU_PERI_UART2, 0, 4),
	DIV(CLK_DOUT_PERI_USI0, "dout_peri_usi0", "gout_peri_usi0",
	    CLK_CON_DIV_CLKCMU_PERI_USI0, 0, 4),
	DIV(CLK_DOUT_PERI_USI1, "dout_peri_usi1", "gout_peri_usi1",
	    CLK_CON_DIV_CLKCMU_PERI_USI1, 0, 4),
	DIV(CLK_DOUT_CIS_CLK0, "dout_cis_clk0", "gout_cis_clk0",
	    CLK_CON_DIV_CLKCMU_CIS_CLK0, 0, 5),
	DIV(CLK_DOUT_CIS_CLK1, "dout_cis_clk1", "gout_cis_clk1",
	    CLK_CON_DIV_CLKCMU_CIS_CLK1, 0, 5),
	DIV(CLK_DOUT_PERI_USI2, "dout_peri_usi2", "gout_peri_usi2",
	    CLK_CON_DIV_CLKCMU_PERI_USI2, 0, 4),

	/* FSYS */
	DIV(CLK_DOUT_FSYS_BUS, "dout_fsys_bus", "gout_fsys_bus",
	    CLK_CON_DIV_CLKCMU_FSYS_BUS, 0, 4),
	DIV(CLK_DOUT_FSYS_MMC_CARD, "dout_fsys_mmc_card", "gout_fsys_mmc_card",
	    CLK_CON_DIV_CLKCMU_FSYS_MMC_CARD, 0, 9),
	DIV(CLK_DOUT_FSYS_MMC_EMBD, "dout_fsys_mmc_embd", "gout_fsys_mmc_embd",
	    CLK_CON_DIV_CLKCMU_FSYS_MMC_EMBD, 0, 9),
	DIV(CLK_DOUT_FSYS_MMC_SDIO, "dout_fsys_mmc_sdio", "gout_fsys_mmc_sdio",
	    CLK_CON_DIV_CLKCMU_FSYS_MMC_SDIO, 0, 9),
	DIV(CLK_DOUT_FSYS_USB30DRD, "dout_fsys_usb30drd", "gout_fsys_usb30drd",
	    CLK_CON_DIV_CLKCMU_FSYS_USB30DRD, 0, 4),
};

static const struct samsung_gate_clock top_gate_clks[] __initconst = {
	/* The audio branch is untouched until exynos7885.dispaud=1 is used. */
	GATE(CLK_GOUT_DISPAUD_CPU, "gout_dispaud_cpu", "mout_dispaud_cpu",
	     CLK_CON_GAT_GATE_CLKCMU_DISPAUD_VCLK, 21, CLK_IGNORE_UNUSED, 0),
	/*
	 * Left out of top_clk_regs, so it keeps the hardware auto-gating mode
	 * the bootloader set. It feeds CMU_G3D, which keeps it enabled.
	 */
	GATE(CLK_GOUT_G3D_SWITCH, "gout_g3d_switch", "dout_shared0_div2",
	     CLK_CON_GAT_GATE_CLKCMU_G3D_SWITCH, 21, 0, 0),
	/*
	 * CPU cluster switching clocks, likewise left in hardware mode. A
	 * cluster runs from one while its PLL relocks; its CMU keeps it
	 * enabled.
	 */
	GATE(CLK_GOUT_CPUCL0_SWITCH, "gout_cpucl0_switch", "mout_shared0_pll",
	     CLK_CON_GAT_GATE_CLKCMU_CPUCL0_SWITCH, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL1_SWITCH, "gout_cpucl1_switch", "dout_shared0_div2",
	     CLK_CON_GAT_GATE_CLKCMU_CPUCL1_SWITCH, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CPUCL2_SWITCH, "gout_cpucl2_switch", "dout_shared0_div2",
	     CLK_CON_GAT_GATE_CLKCMU_CPUCL2_SWITCH, 21, CLK_IGNORE_UNUSED, 0),
	/* CORE */
	GATE(CLK_GOUT_CORE_BUS, "gout_core_bus", "mout_core_bus",
	     CLK_CON_GAT_GATE_CLKCMU_CORE_BUS, 21, 0, 0),
	GATE(CLK_GOUT_CORE_CCI, "gout_core_cci", "mout_core_cci",
	     CLK_CON_GAT_GATE_CLKCMU_CORE_CCI, 21, 0, 0),
	GATE(CLK_GOUT_CORE_G3D, "gout_core_g3d", "mout_core_g3d",
	     CLK_CON_GAT_GATE_CLKCMU_CORE_G3D, 21, 0, 0),

	/* PERI */
	GATE(CLK_GOUT_PERI_BUS, "gout_peri_bus", "mout_peri_bus",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_BUS, 21, 0, 0),
	GATE(CLK_GOUT_PERI_SPI0, "gout_peri_spi0", "mout_peri_spi0",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_SPI0, 21, 0, 0),
	GATE(CLK_GOUT_PERI_SPI1, "gout_peri_spi1", "mout_peri_spi1",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_SPI1, 21, 0, 0),
	GATE(CLK_GOUT_PERI_UART0, "gout_peri_uart0", "mout_peri_uart0",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_UART0, 21, 0, 0),
	GATE(CLK_GOUT_PERI_UART1, "gout_peri_uart1", "mout_peri_uart1",
	     CLK_CON_GAT_GATE_CLKCMUC_PERI_UART1, 21, 0, 0),
	GATE(CLK_GOUT_PERI_UART2, "gout_peri_uart2", "mout_peri_uart2",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_UART2, 21, 0, 0),
	GATE(CLK_GOUT_PERI_USI0, "gout_peri_usi0", "mout_peri_usi0",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_USI0, 21, 0, 0),
	GATE(CLK_GOUT_PERI_USI1, "gout_peri_usi1", "mout_peri_usi1",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_USI1, 21, 0, 0),
	GATE(CLK_GOUT_CIS_CLK0, "gout_cis_clk0", "mout_cis_clk0",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK0, 21, 0, 0),
	GATE(CLK_GOUT_CIS_CLK1, "gout_cis_clk1", "mout_cis_clk1",
	     CLK_CON_GAT_GATE_CLKCMU_CIS_CLK1, 21, 0, 0),
	GATE(CLK_GOUT_PERI_USI2, "gout_peri_usi2", "mout_peri_usi2",
	     CLK_CON_GAT_GATE_CLKCMU_PERI_USI2, 21, 0, 0),

	/* FSYS */
	GATE(CLK_GOUT_FSYS_BUS, "gout_fsys_bus", "mout_fsys_bus",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS_BUS, 21, 0, 0),
	GATE(CLK_GOUT_FSYS_MMC_CARD, "gout_fsys_mmc_card", "mout_fsys_mmc_card",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_CARD, 21, 0, 0),
	GATE(CLK_GOUT_FSYS_MMC_EMBD, "gout_fsys_mmc_embd", "mout_fsys_mmc_embd",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_EMBD, 21, 0, 0),
	GATE(CLK_GOUT_FSYS_MMC_SDIO, "gout_fsys_mmc_sdio", "mout_fsys_mmc_sdio",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS_MMC_SDIO, 21, 0, 0),
	GATE(CLK_GOUT_FSYS_USB30DRD, "gout_fsys_usb30drd", "mout_fsys_usb30drd",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS_USB30DRD, 21, 0, 0),
};

static const struct samsung_cmu_info top_cmu_info __initconst = {
	.pll_clks		= top_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(top_pll_clks),
	.mux_clks		= top_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(top_mux_clks),
	.div_clks		= top_div_clks,
	.nr_div_clks		= ARRAY_SIZE(top_div_clks),
	.gate_clks		= top_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(top_gate_clks),
	.nr_clk_ids		= CLKS_NR_TOP,
	.clk_regs		= top_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(top_clk_regs),
};

static void __init exynos7885_cmu_top_init(struct device_node *np)
{
	struct samsung_cmu_info cmu = top_cmu_info;
	unsigned long regs[ARRAY_SIZE(top_clk_regs) + 1];

	if (exynos7885_enable_dispaud) {
		memcpy(regs, top_clk_regs, sizeof(top_clk_regs));
		regs[ARRAY_SIZE(top_clk_regs)] =
			CLK_CON_GAT_GATE_CLKCMU_DISPAUD_VCLK;
		cmu.clk_regs = regs;
		cmu.nr_clk_regs = ARRAY_SIZE(regs);
	}

	exynos_arm64_register_cmu(NULL, np, &cmu);
}

/* Register CMU_TOP early, as it's a dependency for other early domains */
CLK_OF_DECLARE(exynos7885_cmu_top, "samsung,exynos7885-cmu-top",
	       exynos7885_cmu_top_init);

/* ---- CMU_PERI ------------------------------------------------------------ */

/* Register Offset definitions for CMU_PERI (0x10010000) */
#define PLL_CON0_MUX_CLKCMU_PERI_BUS_USER	0x0100
#define PLL_CON0_MUX_CLKCMU_PERI_SPI0_USER	0x0120
#define PLL_CON0_MUX_CLKCMU_PERI_SPI1_USER	0x0140
#define PLL_CON0_MUX_CLKCMU_PERI_UART0_USER	0x0160
#define PLL_CON0_MUX_CLKCMU_PERI_UART1_USER	0x0180
#define PLL_CON0_MUX_CLKCMU_PERI_UART2_USER	0x01a0
#define PLL_CON0_MUX_CLKCMU_PERI_USI0_USER	0x01c0
#define PLL_CON0_MUX_CLKCMU_PERI_USI1_USER	0x01e0
#define PLL_CON0_MUX_CLKCMU_PERI_USI2_USER	0x0200
#define CLK_CON_GAT_GOUT_PERI_TMU_PCLK		0x2010
#define CLK_CON_GAT_GOUT_PERI_GPIO_TOP_PCLK	0x2024
#define CLK_CON_GAT_GOUT_PERI_HSI2C_0_PCLK	0x2028
#define CLK_CON_GAT_GOUT_PERI_HSI2C_1_PCLK	0x202c
#define CLK_CON_GAT_GOUT_PERI_HSI2C_2_PCLK	0x2030
#define CLK_CON_GAT_GOUT_PERI_HSI2C_3_PCLK	0x2034
#define CLK_CON_GAT_GOUT_PERI_I2C_0_PCLK	0x2038
#define CLK_CON_GAT_GOUT_PERI_I2C_1_PCLK	0x203c
#define CLK_CON_GAT_GOUT_PERI_I2C_2_PCLK	0x2040
#define CLK_CON_GAT_GOUT_PERI_I2C_3_PCLK	0x2044
#define CLK_CON_GAT_GOUT_PERI_I2C_4_PCLK	0x2048
#define CLK_CON_GAT_GOUT_PERI_I2C_5_PCLK	0x204c
#define CLK_CON_GAT_GOUT_PERI_I2C_6_PCLK	0x2050
#define CLK_CON_GAT_GOUT_PERI_I2C_7_PCLK	0x2054
#define CLK_CON_GAT_GOUT_PERI_PWM_MOTOR_PCLK	0x2058
#define CLK_CON_GAT_GOUT_PERI_SPI_0_PCLK	0x205c
#define CLK_CON_GAT_GOUT_PERI_SPI_0_EXT_CLK	0x2060
#define CLK_CON_GAT_GOUT_PERI_SPI_1_PCLK	0x2064
#define CLK_CON_GAT_GOUT_PERI_SPI_1_EXT_CLK	0x2068
#define CLK_CON_GAT_GOUT_PERI_UART_0_EXT_UCLK	0x206c
#define CLK_CON_GAT_GOUT_PERI_UART_0_PCLK	0x2070
#define CLK_CON_GAT_GOUT_PERI_UART_1_EXT_UCLK	0x2074
#define CLK_CON_GAT_GOUT_PERI_UART_1_PCLK	0x2078
#define CLK_CON_GAT_GOUT_PERI_UART_2_EXT_UCLK	0x207c
#define CLK_CON_GAT_GOUT_PERI_UART_2_PCLK	0x2080
#define CLK_CON_GAT_GOUT_PERI_USI0_PCLK		0x2084
#define CLK_CON_GAT_GOUT_PERI_USI0_SCLK		0x2088
#define CLK_CON_GAT_GOUT_PERI_USI1_PCLK		0x208c
#define CLK_CON_GAT_GOUT_PERI_USI1_SCLK		0x2090
#define CLK_CON_GAT_GOUT_PERI_USI2_PCLK		0x2094
#define CLK_CON_GAT_GOUT_PERI_USI2_SCLK		0x2098
#define CLK_CON_GAT_GOUT_PERI_MCT_PCLK		0x20a0
#define CLK_CON_GAT_GOUT_PERI_SYSREG_PERI_PCLK	0x20b0
#define CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER0_PCLK	0x20b4
#define CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER1_PCLK	0x20b8

static const unsigned long peri_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_PERI_BUS_USER,
	PLL_CON0_MUX_CLKCMU_PERI_SPI0_USER,
	PLL_CON0_MUX_CLKCMU_PERI_SPI1_USER,
	PLL_CON0_MUX_CLKCMU_PERI_UART0_USER,
	PLL_CON0_MUX_CLKCMU_PERI_UART1_USER,
	PLL_CON0_MUX_CLKCMU_PERI_UART2_USER,
	PLL_CON0_MUX_CLKCMU_PERI_USI0_USER,
	PLL_CON0_MUX_CLKCMU_PERI_USI1_USER,
	PLL_CON0_MUX_CLKCMU_PERI_USI2_USER,
	CLK_CON_GAT_GOUT_PERI_TMU_PCLK,
	CLK_CON_GAT_GOUT_PERI_GPIO_TOP_PCLK,
	CLK_CON_GAT_GOUT_PERI_HSI2C_0_PCLK,
	CLK_CON_GAT_GOUT_PERI_HSI2C_1_PCLK,
	CLK_CON_GAT_GOUT_PERI_HSI2C_2_PCLK,
	CLK_CON_GAT_GOUT_PERI_HSI2C_3_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_0_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_1_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_2_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_3_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_4_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_5_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_6_PCLK,
	CLK_CON_GAT_GOUT_PERI_I2C_7_PCLK,
	CLK_CON_GAT_GOUT_PERI_PWM_MOTOR_PCLK,
	CLK_CON_GAT_GOUT_PERI_SPI_0_PCLK,
	CLK_CON_GAT_GOUT_PERI_SPI_0_EXT_CLK,
	CLK_CON_GAT_GOUT_PERI_SPI_1_PCLK,
	CLK_CON_GAT_GOUT_PERI_SPI_1_EXT_CLK,
	CLK_CON_GAT_GOUT_PERI_UART_0_EXT_UCLK,
	CLK_CON_GAT_GOUT_PERI_UART_0_PCLK,
	CLK_CON_GAT_GOUT_PERI_UART_1_EXT_UCLK,
	CLK_CON_GAT_GOUT_PERI_UART_1_PCLK,
	CLK_CON_GAT_GOUT_PERI_UART_2_EXT_UCLK,
	CLK_CON_GAT_GOUT_PERI_UART_2_PCLK,
	CLK_CON_GAT_GOUT_PERI_USI0_PCLK,
	CLK_CON_GAT_GOUT_PERI_USI0_SCLK,
	CLK_CON_GAT_GOUT_PERI_USI1_PCLK,
	CLK_CON_GAT_GOUT_PERI_USI1_SCLK,
	CLK_CON_GAT_GOUT_PERI_USI2_PCLK,
	CLK_CON_GAT_GOUT_PERI_USI2_SCLK,
	CLK_CON_GAT_GOUT_PERI_MCT_PCLK,
	CLK_CON_GAT_GOUT_PERI_SYSREG_PERI_PCLK,
	CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER0_PCLK,
	CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER1_PCLK,
};

/* List of parent clocks for Muxes in CMU_PERI */
PNAME(mout_peri_bus_user_p)	= { "oscclk", "dout_peri_bus" };
PNAME(mout_peri_spi0_user_p)	= { "oscclk", "dout_peri_spi0" };
PNAME(mout_peri_spi1_user_p)	= { "oscclk", "dout_peri_spi1" };
PNAME(mout_peri_uart0_user_p)	= { "oscclk", "dout_peri_uart0" };
PNAME(mout_peri_uart1_user_p)	= { "oscclk", "dout_peri_uart1" };
PNAME(mout_peri_uart2_user_p)	= { "oscclk", "dout_peri_uart2" };
PNAME(mout_peri_usi0_user_p)	= { "oscclk", "dout_peri_usi0" };
PNAME(mout_peri_usi1_user_p)	= { "oscclk", "dout_peri_usi1" };
PNAME(mout_peri_usi2_user_p)	= { "oscclk", "dout_peri_usi2" };

static const struct samsung_mux_clock peri_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERI_BUS_USER, "mout_peri_bus_user", mout_peri_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_PERI_BUS_USER, 4, 1),
	MUX(CLK_MOUT_PERI_SPI0_USER, "mout_peri_spi0_user", mout_peri_spi0_user_p,
	    PLL_CON0_MUX_CLKCMU_PERI_SPI0_USER, 4, 1),
	MUX(CLK_MOUT_PERI_SPI1_USER, "mout_peri_spi1_user", mout_peri_spi1_user_p,
	    PLL_CON0_MUX_CLKCMU_PERI_SPI1_USER, 4, 1),
	MUX(CLK_MOUT_PERI_UART0_USER, "mout_peri_uart0_user",
	    mout_peri_uart0_user_p, PLL_CON0_MUX_CLKCMU_PERI_UART0_USER, 4, 1),
	MUX(CLK_MOUT_PERI_UART1_USER, "mout_peri_uart1_user",
	    mout_peri_uart1_user_p, PLL_CON0_MUX_CLKCMU_PERI_UART1_USER, 4, 1),
	MUX(CLK_MOUT_PERI_UART2_USER, "mout_peri_uart2_user",
	    mout_peri_uart2_user_p, PLL_CON0_MUX_CLKCMU_PERI_UART2_USER, 4, 1),
	MUX(CLK_MOUT_PERI_USI0_USER, "mout_peri_usi0_user",
	    mout_peri_usi0_user_p, PLL_CON0_MUX_CLKCMU_PERI_USI0_USER, 4, 1),
	MUX(CLK_MOUT_PERI_USI1_USER, "mout_peri_usi1_user",
	    mout_peri_usi1_user_p, PLL_CON0_MUX_CLKCMU_PERI_USI1_USER, 4, 1),
	MUX(CLK_MOUT_PERI_USI2_USER, "mout_peri_usi2_user",
	    mout_peri_usi2_user_p, PLL_CON0_MUX_CLKCMU_PERI_USI2_USER, 4, 1),
};

static const struct samsung_gate_clock peri_gate_clks[] __initconst = {
	/* TODO: Should be enabled in GPIO driver (or made CLK_IS_CRITICAL) */
	GATE(CLK_GOUT_GPIO_TOP_PCLK, "gout_gpio_top_pclk",
	     "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_GPIO_TOP_PCLK, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI2C0_PCLK, "gout_hsi2c0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_HSI2C_0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_HSI2C1_PCLK, "gout_hsi2c1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_HSI2C_1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_HSI2C2_PCLK, "gout_hsi2c2_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_HSI2C_2_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_HSI2C3_PCLK, "gout_hsi2c3_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_HSI2C_3_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C0_PCLK, "gout_i2c0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C1_PCLK, "gout_i2c1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C2_PCLK, "gout_i2c2_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_2_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C3_PCLK, "gout_i2c3_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_3_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C4_PCLK, "gout_i2c4_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_4_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C5_PCLK, "gout_i2c5_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_5_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C6_PCLK, "gout_i2c6_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_6_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_I2C7_PCLK, "gout_i2c7_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_I2C_7_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_PWM_MOTOR_PCLK, "gout_pwm_motor_pclk",
	     "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_PWM_MOTOR_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_SPI0_PCLK, "gout_spi0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_SPI_0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_SPI0_EXT_CLK, "gout_spi0_ipclk", "mout_peri_spi0_user",
	     CLK_CON_GAT_GOUT_PERI_SPI_0_EXT_CLK, 21, 0, 0),
	GATE(CLK_GOUT_SPI1_PCLK, "gout_spi1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_SPI_1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_SPI1_EXT_CLK, "gout_spi1_ipclk", "mout_peri_spi1_user",
	     CLK_CON_GAT_GOUT_PERI_SPI_1_EXT_CLK, 21, 0, 0),
	GATE(CLK_GOUT_UART0_EXT_UCLK, "gout_uart0_ext_uclk", "mout_peri_uart0_user",
	     CLK_CON_GAT_GOUT_PERI_UART_0_EXT_UCLK, 21, 0, 0),
	GATE(CLK_GOUT_UART0_PCLK, "gout_uart0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_UART_0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_UART1_EXT_UCLK, "gout_uart1_ext_uclk", "mout_peri_uart1_user",
	     CLK_CON_GAT_GOUT_PERI_UART_1_EXT_UCLK, 21, 0, 0),
	GATE(CLK_GOUT_UART1_PCLK, "gout_uart1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_UART_1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_UART2_EXT_UCLK, "gout_uart2_ext_uclk", "mout_peri_uart2_user",
	     CLK_CON_GAT_GOUT_PERI_UART_2_EXT_UCLK, 21, 0, 0),
	GATE(CLK_GOUT_UART2_PCLK, "gout_uart2_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_UART_2_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI0_PCLK, "gout_usi0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_USI0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI0_SCLK, "gout_usi0_sclk", "mout_peri_usi0_user",
	     CLK_CON_GAT_GOUT_PERI_USI0_SCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI1_PCLK, "gout_usi1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_USI1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI1_SCLK, "gout_usi1_sclk", "mout_peri_usi1_user",
	     CLK_CON_GAT_GOUT_PERI_USI1_SCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI2_PCLK, "gout_usi2_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_USI2_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_USI2_SCLK, "gout_usi2_sclk", "mout_peri_usi2_user",
	     CLK_CON_GAT_GOUT_PERI_USI2_SCLK, 21, 0, 0),
	GATE(CLK_GOUT_MCT_PCLK, "gout_mct_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_MCT_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_SYSREG_PERI_PCLK, "gout_sysreg_peri_pclk",
	     "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_SYSREG_PERI_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_WDT0_PCLK, "gout_wdt0_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER0_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_WDT1_PCLK, "gout_wdt1_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_WDT_CLUSTER1_PCLK, 21, 0, 0),
	GATE(CLK_GOUT_TMU_PCLK, "gout_tmu_pclk", "mout_peri_bus_user",
	     CLK_CON_GAT_GOUT_PERI_TMU_PCLK, 21, 0, 0),
};

static const struct samsung_cmu_info peri_cmu_info __initconst = {
	.mux_clks		= peri_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(peri_mux_clks),
	.gate_clks		= peri_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(peri_gate_clks),
	.nr_clk_ids		= CLKS_NR_PERI,
	.clk_regs		= peri_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(peri_clk_regs),
	.clk_name		= "dout_peri_bus",
};

static void __init exynos7885_cmu_peri_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &peri_cmu_info);
}

/* Register CMU_PERI early, as it's needed for MCT timer */
CLK_OF_DECLARE(exynos7885_cmu_peri, "samsung,exynos7885-cmu-peri",
	       exynos7885_cmu_peri_init);

/* ---- CMU_CORE ------------------------------------------------------------ */

/* Register Offset definitions for CMU_CORE (0x12000000) */
#define PLL_CON0_MUX_CLKCMU_CORE_BUS_USER		0x0100
#define PLL_CON0_MUX_CLKCMU_CORE_CCI_USER		0x0120
#define PLL_CON0_MUX_CLKCMU_CORE_G3D_USER		0x0140
#define CLK_CON_MUX_MUX_CLK_CORE_GIC			0x1000
#define CLK_CON_DIV_DIV_CLK_CORE_BUSP			0x1800
#define CLK_CON_GAT_GOUT_CORE_CCI_550_ACLK		0x2054
#define CLK_CON_GAT_GOUT_CORE_GIC400_CLK		0x2058
#define CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_ACLK		0x215c
#define CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_GCLK		0x2160
#define CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_PCLK		0x2164
#define CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_ACLK_P_CORE	0x2168
#define CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_CCLK_P_CORE	0x216c
#define CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK		0x2170
#define CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK_P_CORE	0x2174

#define CLK_CON_GAT_GOUT_CORE_ADC_S0			0x2178
#define CLK_CON_GAT_GOUT_CORE_ADC_S1			0x217c

static const unsigned long core_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLKCMU_CORE_BUS_USER,
	PLL_CON0_MUX_CLKCMU_CORE_CCI_USER,
	PLL_CON0_MUX_CLKCMU_CORE_G3D_USER,
	CLK_CON_MUX_MUX_CLK_CORE_GIC,
	CLK_CON_DIV_DIV_CLK_CORE_BUSP,
	CLK_CON_GAT_GOUT_CORE_CCI_550_ACLK,
	CLK_CON_GAT_GOUT_CORE_GIC400_CLK,
	CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_ACLK,
	CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_GCLK,
	CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_PCLK,
	CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_ACLK_P_CORE,
	CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_CCLK_P_CORE,
	CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK,
	CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK_P_CORE,
	CLK_CON_GAT_GOUT_CORE_ADC_S0,
	CLK_CON_GAT_GOUT_CORE_ADC_S1,
};

/* List of parent clocks for Muxes in CMU_CORE */
PNAME(mout_core_bus_user_p)		= { "oscclk", "dout_core_bus" };
PNAME(mout_core_cci_user_p)		= { "oscclk", "dout_core_cci" };
PNAME(mout_core_g3d_user_p)		= { "oscclk", "dout_core_g3d" };
PNAME(mout_core_gic_p)			= { "dout_core_busp", "oscclk" };

static const struct samsung_mux_clock core_mux_clks[] __initconst = {
	MUX(CLK_MOUT_CORE_BUS_USER, "mout_core_bus_user", mout_core_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_CORE_BUS_USER, 4, 1),
	MUX(CLK_MOUT_CORE_CCI_USER, "mout_core_cci_user", mout_core_cci_user_p,
	    PLL_CON0_MUX_CLKCMU_CORE_CCI_USER, 4, 1),
	MUX(CLK_MOUT_CORE_G3D_USER, "mout_core_g3d_user", mout_core_g3d_user_p,
	    PLL_CON0_MUX_CLKCMU_CORE_G3D_USER, 4, 1),
	MUX(CLK_MOUT_CORE_GIC, "mout_core_gic", mout_core_gic_p,
	    CLK_CON_MUX_MUX_CLK_CORE_GIC, 0, 1),
};

static const struct samsung_div_clock core_div_clks[] __initconst = {
	DIV(CLK_DOUT_CORE_BUSP, "dout_core_busp", "mout_core_bus_user",
	    CLK_CON_DIV_DIV_CLK_CORE_BUSP, 0, 2),
};

static const struct samsung_gate_clock core_gate_clks[] __initconst = {
	GATE(CLK_GOUT_ADC_S0, "gout_adc_s0", "dout_core_busp",
	     CLK_CON_GAT_GOUT_CORE_ADC_S0, 21, 0, 0),
	GATE(CLK_GOUT_ADC_S1, "gout_adc_s1", "dout_core_busp",
	     CLK_CON_GAT_GOUT_CORE_ADC_S1, 21, 0, 0),
	/* CCI (interconnect) clock must be always running */
	GATE(CLK_GOUT_CCI_ACLK, "gout_cci_aclk", "mout_core_cci_user",
	     CLK_CON_GAT_GOUT_CORE_CCI_550_ACLK, 21, CLK_IS_CRITICAL, 0),
	/* GIC (interrupt controller) clock must be always running */
	GATE(CLK_GOUT_GIC400_CLK, "gout_gic400_clk", "mout_core_gic",
	     CLK_CON_GAT_GOUT_CORE_GIC400_CLK, 21, CLK_IS_CRITICAL, 0),
	/*
	 * TREX D and P Core (seems to be related to "bus traffic shaper")
	 * clocks must always be running
	 */
	GATE(CLK_GOUT_TREX_D_CORE_ACLK, "gout_trex_d_core_aclk", "mout_core_bus_user",
	     CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_ACLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_D_CORE_GCLK, "gout_trex_d_core_gclk", "mout_core_g3d_user",
	     CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_GCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_D_CORE_PCLK, "gout_trex_d_core_pclk", "dout_core_busp",
	     CLK_CON_GAT_GOUT_CORE_TREX_D_CORE_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_P_CORE_ACLK_P_CORE, "gout_trex_p_core_aclk_p_core",
	     "mout_core_bus_user", CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_ACLK_P_CORE, 21,
	     CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_P_CORE_CCLK_P_CORE, "gout_trex_p_core_cclk_p_core",
	     "mout_core_cci_user", CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_CCLK_P_CORE, 21,
	     CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_P_CORE_PCLK, "gout_trex_p_core_pclk", "dout_core_busp",
	     CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_TREX_P_CORE_PCLK_P_CORE, "gout_trex_p_core_pclk_p_core",
	     "dout_core_busp", CLK_CON_GAT_GOUT_CORE_TREX_P_CORE_PCLK_P_CORE, 21,
	     CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info core_cmu_info __initconst = {
	.mux_clks		= core_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(core_mux_clks),
	.div_clks		= core_div_clks,
	.nr_div_clks		= ARRAY_SIZE(core_div_clks),
	.gate_clks		= core_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(core_gate_clks),
	.nr_clk_ids		= CLKS_NR_CORE,
	.clk_regs		= core_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(core_clk_regs),
	.clk_name		= "dout_core_bus",
};

/* ---- CMU_FSYS ------------------------------------------------------------ */

/* Register Offset definitions for CMU_FSYS (0x13400000) */
#define PLL_LOCKTIME_PLL_USB				0x0000
#define PLL_CON0_MUX_CLKCMU_FSYS_BUS_USER		0x0100
#define PLL_CON0_MUX_CLKCMU_FSYS_MMC_CARD_USER		0x0120
#define PLL_CON0_MUX_CLKCMU_FSYS_MMC_EMBD_USER		0x0140
#define PLL_CON0_MUX_CLKCMU_FSYS_MMC_SDIO_USER		0x0160
#define PLL_CON0_MUX_CLKCMU_FSYS_USB30DRD_USER		0x0180
#define PLL_CON0_PLL_USB				0x01a0
#define CLK_CON_GAT_CLK_FSYS_USB20PHY_CLKCORE		0x200c
#define CLK_CON_GAT_GOUT_FSYS_MMC_CARD_I_ACLK		0x2030
#define CLK_CON_GAT_GOUT_FSYS_MMC_CARD_SDCLKIN		0x2034
#define CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_I_ACLK		0x2038
#define CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_SDCLKIN		0x203c
#define CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_I_ACLK		0x2040
#define CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_SDCLKIN		0x2044
#define CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_20PHYCTRL	0x2068
#define CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_0	0x206c
#define CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_1	0x2070
#define CLK_CON_GAT_GOUT_FSYS_USB30DRD_BUS_CLK_EARLY	0x2074
#define CLK_CON_GAT_GOUT_FSYS_USB30DRD_REF_CLK		0x2078

static const unsigned long fsys_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_USB,
	PLL_CON0_MUX_CLKCMU_FSYS_BUS_USER,
	PLL_CON0_MUX_CLKCMU_FSYS_MMC_CARD_USER,
	PLL_CON0_MUX_CLKCMU_FSYS_MMC_EMBD_USER,
	PLL_CON0_MUX_CLKCMU_FSYS_MMC_SDIO_USER,
	PLL_CON0_MUX_CLKCMU_FSYS_USB30DRD_USER,
	PLL_CON0_PLL_USB,
	CLK_CON_GAT_CLK_FSYS_USB20PHY_CLKCORE,
	CLK_CON_GAT_GOUT_FSYS_MMC_CARD_I_ACLK,
	CLK_CON_GAT_GOUT_FSYS_MMC_CARD_SDCLKIN,
	CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_I_ACLK,
	CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_SDCLKIN,
	CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_I_ACLK,
	CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_SDCLKIN,
	CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_20PHYCTRL,
	CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_0,
	CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_1,
	CLK_CON_GAT_GOUT_FSYS_USB30DRD_BUS_CLK_EARLY,
	CLK_CON_GAT_GOUT_FSYS_USB30DRD_REF_CLK,
};

static const struct samsung_pll_rate_table pll_usb_rate_table[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 50000000U, 400, 13, 4),
};

static const struct samsung_pll_clock fsys_pll_clks[] __initconst = {
	PLL(pll_1418x, CLK_FOUT_USB_PLL, "fout_usb_pll", "oscclk",
	    PLL_LOCKTIME_PLL_USB, PLL_CON0_PLL_USB,
	    pll_usb_rate_table),
};

/* List of parent clocks for Muxes in CMU_FSYS */
PNAME(mout_fsys_bus_user_p)		= { "oscclk", "dout_fsys_bus" };
PNAME(mout_fsys_mmc_card_user_p)	= { "oscclk", "dout_fsys_mmc_card" };
PNAME(mout_fsys_mmc_embd_user_p)	= { "oscclk", "dout_fsys_mmc_embd" };
PNAME(mout_fsys_mmc_sdio_user_p)	= { "oscclk", "dout_fsys_mmc_sdio" };
PNAME(mout_fsys_usb30drd_user_p)	= { "oscclk", "dout_fsys_usb30drd" };
PNAME(mout_usb_pll_p)			= { "oscclk", "fout_usb_pll" };

static const struct samsung_mux_clock fsys_mux_clks[] __initconst = {
	MUX(CLK_MOUT_FSYS_BUS_USER, "mout_fsys_bus_user", mout_fsys_bus_user_p,
	    PLL_CON0_MUX_CLKCMU_FSYS_BUS_USER, 4, 1),
	MUX_F(CLK_MOUT_FSYS_MMC_CARD_USER, "mout_fsys_mmc_card_user",
	      mout_fsys_mmc_card_user_p, PLL_CON0_MUX_CLKCMU_FSYS_MMC_CARD_USER,
	      4, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(CLK_MOUT_FSYS_MMC_EMBD_USER, "mout_fsys_mmc_embd_user",
	      mout_fsys_mmc_embd_user_p, PLL_CON0_MUX_CLKCMU_FSYS_MMC_EMBD_USER,
	      4, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(CLK_MOUT_FSYS_MMC_SDIO_USER, "mout_fsys_mmc_sdio_user",
	      mout_fsys_mmc_sdio_user_p, PLL_CON0_MUX_CLKCMU_FSYS_MMC_SDIO_USER,
	      4, 1, CLK_SET_RATE_PARENT, 0),
	MUX(CLK_MOUT_FSYS_USB30DRD_USER, "mout_fsys_usb30drd_user",
	      mout_fsys_usb30drd_user_p, PLL_CON0_MUX_CLKCMU_FSYS_USB30DRD_USER,
	      4, 1),
	nMUX_F(CLK_MOUT_USB_PLL, "mout_usb_pll", mout_usb_pll_p,
	    PLL_CON0_PLL_USB, 4, 1, CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_gate_clock fsys_gate_clks[] __initconst = {
	GATE(CLK_FSYS_USB20PHY_CLKCORE, "clk_fsys_usb20phy_clkcore", "mout_usb_pll",
	     CLK_CON_GAT_CLK_FSYS_USB20PHY_CLKCORE, 21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MMC_CARD_ACLK, "gout_mmc_card_aclk", "mout_fsys_bus_user",
	     CLK_CON_GAT_GOUT_FSYS_MMC_CARD_I_ACLK, 21, 0, 0),
	GATE(CLK_GOUT_MMC_CARD_SDCLKIN, "gout_mmc_card_sdclkin",
	     "mout_fsys_mmc_card_user", CLK_CON_GAT_GOUT_FSYS_MMC_CARD_SDCLKIN,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MMC_EMBD_ACLK, "gout_mmc_embd_aclk", "mout_fsys_bus_user",
	     CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_I_ACLK, 21, 0, 0),
	GATE(CLK_GOUT_MMC_EMBD_SDCLKIN, "gout_mmc_embd_sdclkin",
	     "mout_fsys_mmc_embd_user", CLK_CON_GAT_GOUT_FSYS_MMC_EMBD_SDCLKIN,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_MMC_SDIO_ACLK, "gout_mmc_sdio_aclk", "mout_fsys_bus_user",
	     CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_I_ACLK, 21, 0, 0),
	GATE(CLK_GOUT_MMC_SDIO_SDCLKIN, "gout_mmc_sdio_sdclkin",
	     "mout_fsys_mmc_sdio_user", CLK_CON_GAT_GOUT_FSYS_MMC_SDIO_SDCLKIN,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_FSYS_USB30DRD_ACLK_20PHYCTRL, "clk_fsys_usb30drd_aclk_20phyctrl",
	     "mout_fsys_bus_user", CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_20PHYCTRL, 21, 0, 0),
	GATE(CLK_FSYS_USB30DRD_ACLK_30PHYCTRL_0, "clk_fsys_usb30drd_aclk_30phyctrl_0",
	     "mout_fsys_bus_user", CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_0, 21, 0, 0),
	GATE(CLK_FSYS_USB30DRD_ACLK_30PHYCTRL_1, "clk_fsys_usb30drd_aclk_30phyctrl_1",
	     "mout_fsys_bus_user", CLK_CON_GAT_GOUT_FSYS_USB30DRD_ACLK_30PHYCTRL_1, 21, 0, 0),
	GATE(CLK_FSYS_USB30DRD_BUS_CLK_EARLY, "clk_fsys_usb30drd_bus_clk_early",
	     "mout_fsys_bus_user", CLK_CON_GAT_GOUT_FSYS_USB30DRD_BUS_CLK_EARLY, 21, 0, 0),
	GATE(CLK_FSYS_USB30DRD_REF_CLK, "clk_fsys_usb30drd_ref_clk", "mout_fsys_usb30drd_user",
	     CLK_CON_GAT_GOUT_FSYS_USB30DRD_REF_CLK, 21, 0, 0),
};

static const struct samsung_cmu_info fsys_cmu_info __initconst = {
	.pll_clks		= fsys_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(fsys_pll_clks),
	.mux_clks		= fsys_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(fsys_mux_clks),
	.gate_clks		= fsys_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(fsys_gate_clks),
	.nr_clk_ids		= CLKS_NR_FSYS,
	.clk_regs		= fsys_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(fsys_clk_regs),
	.clk_name		= "dout_fsys_bus",
};

/* ---- CMU_DISPAUD -------------------------------------------------------- */

#define PLL_CON0_PLL_AUD		0x0160
#define PLL_CON3_PLL_AUD		0x016c
#define CLK_CON_MUX_AUD_CPU		0x1000
#define CLK_CON_MUX_AUD_CPU_HCH		0x1004
#define CLK_CON_MUX_AUD_UAIF0		0x100c
#define CLK_CON_MUX_AUD_UAIF3		0x1014
#define PLL_CON0_MUX_AUD_CPU_USER	0x0100
#define CLK_CON_DIV_AUDIF		0x1800
#define CLK_CON_DIV_AUD_BUS		0x1804
#define CLK_CON_DIV_AUD_CPU_ACLK	0x1808
#define CLK_CON_DIV_AUD_PLL		0x1818
#define CLK_CON_DIV_UAIF0		0x181c
#define CLK_CON_DIV_UAIF3		0x1824
#define CLK_CON_GAT_ABOX_ACLK		0x200c
#define CLK_CON_GAT_UAIF0_BCLK		0x2014
#define CLK_CON_GAT_UAIF3_BCLK		0x201c
#define CLK_CON_GAT_ABOX_CA7		0x2024
#define CLK_CON_GAT_UAIF0_SYNC		0x20bc
#define CLK_CON_GAT_UAIF3_SYNC		0x20c4
#define CLK_CON_GAT_SMMU_ABOX_CLK	0x20d0
#define CLK_CON_GAT_SMMU_DPU_CLK	0x20d4

static const unsigned long dispaud_clk_regs[] __initconst = {
	PLL_CON0_PLL_AUD,
	PLL_CON3_PLL_AUD,
	CLK_CON_MUX_AUD_CPU,
	CLK_CON_MUX_AUD_CPU_HCH,
	CLK_CON_MUX_AUD_UAIF0,
	CLK_CON_MUX_AUD_UAIF3,
	PLL_CON0_MUX_AUD_CPU_USER,
	CLK_CON_DIV_AUDIF,
	CLK_CON_DIV_AUD_BUS,
	CLK_CON_DIV_AUD_CPU_ACLK,
	CLK_CON_DIV_AUD_PLL,
	CLK_CON_DIV_UAIF0,
	CLK_CON_DIV_UAIF3,
	CLK_CON_GAT_ABOX_ACLK,
	CLK_CON_GAT_UAIF0_BCLK,
	CLK_CON_GAT_UAIF3_BCLK,
	CLK_CON_GAT_ABOX_CA7,
	CLK_CON_GAT_UAIF0_SYNC,
	CLK_CON_GAT_UAIF3_SYNC,
	CLK_CON_GAT_SMMU_ABOX_CLK,
};

static const struct samsung_pll_rate_table dispaud_pll_rates[] __initconst = {
	PLL_36XX_RATE(26 * MHZ, 1179648040U, 45, 1, 0, 24319),
	PLL_36XX_RATE(26 * MHZ, 1083801605U, 42, 1, 0, -20665),
};

static const struct samsung_pll_clock dispaud_pll_clks[] __initconst = {
	PLL(pll_1431x, CLK_FOUT_AUD_PLL, "fout_aud_pll", "oscclk",
	    0, PLL_CON0_PLL_AUD, dispaud_pll_rates),
};

static const struct samsung_fixed_rate_clock dispaud_fixed_clks[] __initconst = {
	/* External AUDCDCLK3 input rate from the downstream clock model. */
	FRATE(IOCLK_AUDIOCDCLK0, "ioclk_audiocdclk0", NULL, 0, 100000000),
	FRATE(IOCLK_AUDIOCDCLK3, "ioclk_audiocdclk3", NULL, 0, 100000000),
};

PNAME(mout_aud_cpu_user_p) = { "oscclk", "dout_dispaud_cpu" };
PNAME(mout_aud_cpu_p) = { "dout_aud_pll", "mout_aud_cpu_user" };
PNAME(mout_aud_cpu_hch_p) = { "mout_aud_cpu", "oscclk" };
PNAME(mout_aud_uaif0_p) = { "dout_uaif0", "ioclk_audiocdclk0" };
PNAME(mout_aud_uaif3_p) = { "dout_uaif3", "ioclk_audiocdclk3" };

static const struct samsung_mux_clock dispaud_mux_clks[] __initconst = {
	MUX(CLK_MOUT_AUD_CPU_USER, "mout_aud_cpu_user",
	    mout_aud_cpu_user_p, PLL_CON0_MUX_AUD_CPU_USER, 4, 1),
	MUX(CLK_MOUT_AUD_CPU, "mout_aud_cpu", mout_aud_cpu_p,
	    CLK_CON_MUX_AUD_CPU, 0, 1),
	MUX(CLK_MOUT_AUD_CPU_HCH, "mout_aud_cpu_hch",
	    mout_aud_cpu_hch_p, CLK_CON_MUX_AUD_CPU_HCH, 0, 1),
	MUX(CLK_MOUT_AUD_UAIF0, "mout_aud_uaif0", mout_aud_uaif0_p,
	    CLK_CON_MUX_AUD_UAIF0, 0, 1),
	MUX(CLK_MOUT_AUD_UAIF3, "mout_aud_uaif3", mout_aud_uaif3_p,
	    CLK_CON_MUX_AUD_UAIF3, 0, 1),
};

static const struct samsung_div_clock dispaud_div_clks[] __initconst = {
	DIV(CLK_DOUT_AUDIF, "dout_audif", "fout_aud_pll",
	    CLK_CON_DIV_AUDIF, 0, 9),
	DIV(CLK_DOUT_AUD_BUS, "dout_aud_bus", "fout_aud_pll",
	    CLK_CON_DIV_AUD_BUS, 0, 3),
	DIV(CLK_DOUT_UAIF0, "dout_uaif0", "dout_audif",
	    CLK_CON_DIV_UAIF0, 0, 5),
	DIV(CLK_DOUT_UAIF3, "dout_uaif3", "dout_audif",
	    CLK_CON_DIV_UAIF3, 0, 5),
	DIV(CLK_DOUT_AUD_PLL, "dout_aud_pll", "fout_aud_pll",
	    CLK_CON_DIV_AUD_PLL, 0, 4),
	DIV(CLK_DOUT_AUD_CPU_ACLK, "dout_aud_cpu_aclk",
	    "mout_aud_cpu_hch", CLK_CON_DIV_AUD_CPU_ACLK, 0, 3),
};

static const struct samsung_gate_clock dispaud_gate_clks[] __initconst = {
	GATE(CLK_GOUT_ABOX_ACLK, "gout_abox_aclk", "dout_aud_bus",
	     CLK_CON_GAT_ABOX_ACLK, 21, 0, 0),
	GATE(CLK_GOUT_SMMU_ABOX_CLK, "gout_smmu_abox_clk",
	     "dout_aud_bus", CLK_CON_GAT_SMMU_ABOX_CLK, 21, 0, 0),
	GATE(CLK_GOUT_UAIF0_BCLK, "gout_uaif0_bclk", "mout_aud_uaif0",
	     CLK_CON_GAT_UAIF0_BCLK, 21, 0, 0),
	GATE(CLK_GOUT_UAIF0_SYNC, "gout_uaif0_sync", "mout_aud_uaif0",
	     CLK_CON_GAT_UAIF0_SYNC, 21, 0, 0),
	GATE(CLK_GOUT_UAIF3_BCLK, "gout_uaif3_bclk", "mout_aud_uaif3",
	     CLK_CON_GAT_UAIF3_BCLK, 21, 0, 0),
	GATE(CLK_GOUT_UAIF3_SYNC, "gout_uaif3_sync", "mout_aud_uaif3",
	     CLK_CON_GAT_UAIF3_SYNC, 21, 0, 0),
	GATE(CLK_GOUT_ABOX_CA7, "gout_abox_ca7", "mout_aud_cpu_hch",
	     CLK_CON_GAT_ABOX_CA7, 21, 0, 0),
	/*
	 * The DPU System MMU clock comes from the DISPAUD bus clock, which is
	 * not modelled. The display runs from the bootloader on that bus, so
	 * this gate is left out of dispaud_clk_regs: it stays in the hardware
	 * mode the bootloader set (manual bit clear). The System MMU driver
	 * requires a clock; it only turns this one off while the display is
	 * stopped for system sleep.
	 */
	GATE(CLK_GOUT_SMMU_DPU_CLK, "gout_smmu_dpu_clk", "oscclk",
	     CLK_CON_GAT_SMMU_DPU_CLK, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info dispaud_cmu_info __initconst = {
	.pll_clks		= dispaud_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(dispaud_pll_clks),
	.fixed_clks		= dispaud_fixed_clks,
	.nr_fixed_clks		= ARRAY_SIZE(dispaud_fixed_clks),
	.mux_clks		= dispaud_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(dispaud_mux_clks),
	.div_clks		= dispaud_div_clks,
	.nr_div_clks		= ARRAY_SIZE(dispaud_div_clks),
	.gate_clks		= dispaud_gate_clks,
	.nr_gate_clks		= ARRAY_SIZE(dispaud_gate_clks),
	.nr_clk_ids		= CLKS_NR_DISPAUD,
	.clk_regs		= dispaud_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(dispaud_clk_regs),
	.clk_name		= "oscclk",
};

/* ---- CMU_G3D ------------------------------------------------------------- */

/* Register Offset definitions for CMU_G3D (0x11400000) */
#define PLL_LOCKTIME_PLL_G3D			0x0000
#define PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER	0x0100
#define PLL_CON0_PLL_G3D			0x0120
#define CLK_CON_MUX_MUX_CLK_G3D_BUSD		0x1004
#define CLK_CON_DIV_DIV_CLK_G3D_BUSP		0x1800

/*
 * Only PLL, mux and divider registers are listed: CMU_G3D gates keep the
 * hardware auto-gating mode the bootloader set, so probing writes nothing.
 */
static const unsigned long g3d_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_G3D,
	PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER,
	PLL_CON0_PLL_G3D,
	CLK_CON_MUX_MUX_CLK_G3D_BUSD,
	CLK_CON_DIV_DIV_CLK_G3D_BUSP,
};

/* GPU DVFS levels, with the P/M/S settings from the bootloader's ECT. */
static const struct samsung_pll_rate_table g3d_pll_rates[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 845000000U, 130, 4, 0),
	PLL_35XX_RATE(26 * MHZ, 676000000U, 104, 4, 0),
	PLL_35XX_RATE(26 * MHZ, 545000000U, 545, 13, 1),
	PLL_35XX_RATE(26 * MHZ, 450000000U, 450, 13, 1),
	PLL_35XX_RATE(26 * MHZ, 343000000U, 343, 13, 1),
};

static const struct samsung_pll_clock g3d_pll_clks[] __initconst = {
	PLL(pll_1417x, CLK_FOUT_G3D_PLL, "fout_g3d_pll", "oscclk",
	    PLL_LOCKTIME_PLL_G3D, PLL_CON0_PLL_G3D, g3d_pll_rates),
};

PNAME(mout_g3d_pll_p)		= { "oscclk", "fout_g3d_pll" };
PNAME(mout_g3d_switch_user_p)	= { "oscclk", "dout_g3d_switch" };
PNAME(mout_g3d_busd_p)		= { "mout_g3d_pll", "mout_g3d_switch_user" };

static const struct samsung_mux_clock g3d_mux_clks[] __initconst = {
	MUX_F(CLK_MOUT_G3D_PLL, "mout_g3d_pll", mout_g3d_pll_p,
	      PLL_CON0_PLL_G3D, 4, 1,
	      CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT, 0),
	MUX(CLK_MOUT_G3D_SWITCH_USER, "mout_g3d_switch_user",
	    mout_g3d_switch_user_p, PLL_CON0_MUX_CLKCMU_G3D_SWITCH_USER, 4, 1),
	/*
	 * Clocks both the Mali core and the G3D data bus. GPU frequency
	 * requests go to PLL_G3D; reparenting is left to the notifier below.
	 */
	MUX_F(CLK_MOUT_G3D_BUSD, "mout_g3d_busd", mout_g3d_busd_p,
	      CLK_CON_MUX_MUX_CLK_G3D_BUSD, 0, 1,
	      CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT, 0),
};

static const struct samsung_div_clock g3d_div_clks[] __initconst = {
	DIV(CLK_DOUT_G3D_BUSP, "dout_g3d_busp", "mout_g3d_busd",
	    CLK_CON_DIV_DIV_CLK_G3D_BUSP, 0, 3),
};

static const struct samsung_cmu_info g3d_cmu_info __initconst = {
	.pll_clks		= g3d_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(g3d_pll_clks),
	.mux_clks		= g3d_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(g3d_mux_clks),
	.div_clks		= g3d_div_clks,
	.nr_div_clks		= ARRAY_SIZE(g3d_div_clks),
	.nr_clk_ids		= CLKS_NR_G3D,
	.clk_regs		= g3d_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(g3d_clk_regs),
	/* Keep the switching clock running while CMU_G3D is in use. */
	.clk_name		= "dout_g3d_switch",
};

#define G3D_BUSD_SEL_SWITCH	BIT(0)
#define G3D_MUX_BUSY		BIT(16)

static void __iomem *g3d_busd_reg;
static bool g3d_parked;

static void exynos7885_g3d_busd_select(bool use_switch)
{
	u32 val = readl(g3d_busd_reg);

	if (use_switch)
		val |= G3D_BUSD_SEL_SWITCH;
	else
		val &= ~G3D_BUSD_SEL_SWITCH;
	writel(val, g3d_busd_reg);

	if (readl_poll_timeout_atomic(g3d_busd_reg, val,
				      !(val & G3D_MUX_BUSY), 1, 100))
		pr_warn("exynos7885: G3D BUSD mux stays busy\n");
}

/*
 * Retuning PLL_G3D leaves its output unstable until it locks again, so run
 * the GPU from the CMU_TOP switching clock (about 400 MHz) meanwhile, as the
 * vendor DVFS does. The supply voltage is at least the lower of the two
 * levels' voltages, which covers 400 MHz.
 */
static int exynos7885_g3d_pll_notifier(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	switch (event) {
	case PRE_RATE_CHANGE:
		g3d_parked = !(readl(g3d_busd_reg) & G3D_BUSD_SEL_SWITCH);
		if (g3d_parked)
			exynos7885_g3d_busd_select(true);
		break;
	case POST_RATE_CHANGE:
	case ABORT_RATE_CHANGE:
		if (g3d_parked)
			exynos7885_g3d_busd_select(false);
		g3d_parked = false;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos7885_g3d_pll_nb = {
	.notifier_call = exynos7885_g3d_pll_notifier,
};

static int exynos7885_g3d_register_notifier(struct device *dev)
{
	struct of_phandle_args args = {
		.np = dev->of_node,
		.args_count = 1,
		.args = { CLK_FOUT_G3D_PLL },
	};
	struct clk *pll;

	/* Mapped like the CMU registers themselves, for the driver's lifetime. */
	g3d_busd_reg = of_iomap(dev->of_node, 0);
	if (!g3d_busd_reg)
		return -ENOMEM;
	g3d_busd_reg += CLK_CON_MUX_MUX_CLK_G3D_BUSD;

	pll = of_clk_get_from_provider(&args);
	if (IS_ERR(pll))
		return PTR_ERR(pll);

	return clk_notifier_register(pll, &exynos7885_g3d_pll_nb);
}

/* ---- CMU_CPUCL0 (A73), CMU_CPUCL1, CMU_CPUCL2 (A53) ------------------------ */

#define PLL_LOCKTIME_PLL_CPUCL			0x0000
#define PLL_CON0_PLL_CPUCL0			0x0140
#define PLL_CON0_MUX_CLKCMU_CPUCL0_SWITCH_USER	0x0120
#define PLL_CON0_PLL_CPUCL12			0x0120
#define PLL_CON0_MUX_CLKCMU_CPUCL12_SWITCH_USER	0x0100
#define CLK_CON_MUX_MUX_CLK_CPUCL_PLL		0x1000
#define CLK_CON_DIV_DIV_CLK_CPUCL_CPU		0x1810

/*
 * Only PLL, mux and divider registers are listed, so probing writes
 * nothing. The CPU divider is read-only and the ACLK/ATCLK/PCLK dividers
 * keep the bootloader's ratios: the vendor DVFS tables change only the PLL
 * between levels.
 */
static const unsigned long cpucl0_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_CPUCL,
	PLL_CON0_PLL_CPUCL0,
	PLL_CON0_MUX_CLKCMU_CPUCL0_SWITCH_USER,
	CLK_CON_MUX_MUX_CLK_CPUCL_PLL,
	CLK_CON_DIV_DIV_CLK_CPUCL_CPU,
};

static const unsigned long cpucl12_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_CPUCL,
	PLL_CON0_PLL_CPUCL12,
	PLL_CON0_MUX_CLKCMU_CPUCL12_SWITCH_USER,
	CLK_CON_MUX_MUX_CLK_CPUCL_PLL,
	CLK_CON_DIV_DIV_CLK_CPUCL_CPU,
};

/* Levels from the bootloader's ECT PLL tables. */
static const struct samsung_pll_rate_table cpucl0_pll_rates[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 1768000000U, 204, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1664000000U, 192, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1560000000U, 180, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1352000000U, 156, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1144000000U, 132, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 936000000U, 216, 3, 1),
};

static const struct samsung_pll_rate_table cpucl12_pll_rates[] __initconst = {
	PLL_35XX_RATE(26 * MHZ, 1482000000U, 171, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1352000000U, 156, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1248000000U, 144, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1144000000U, 132, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 1014000000U, 117, 3, 0),
	PLL_35XX_RATE(26 * MHZ, 676000000U, 156, 3, 1),
	PLL_35XX_RATE(26 * MHZ, 546000000U, 126, 3, 1),
};

/*
 * While its PLL relocks, a cluster runs from its switch-user clock. The
 * bootloader leaves that mux on the 26 MHz oscillator; the board DT can
 * select the CMU_TOP switching clock instead, at a rate no higher than the
 * cluster's lowest OPP, so the cluster keeps running at a useful speed.
 */
#define CPUCL_CLKS(n, pll_con0, switch_user, rates)			\
static const struct samsung_pll_clock cpucl##n##_pll_clks[] __initconst = { \
	PLL_PARK(pll_1417x, CLK_FOUT_CPUCL_PLL, "fout_cpucl" #n "_pll",	\
		 "oscclk", PLL_LOCKTIME_PLL_CPUCL, pll_con0, rates,	\
		 CLK_CON_MUX_MUX_CLK_CPUCL_PLL),			\
};									\
PNAME(mout_cpucl##n##_pll_p) = { "oscclk", "fout_cpucl" #n "_pll" };	\
PNAME(mout_cpucl##n##_switch_user_p) = { "oscclk",			\
					  "dout_cpucl" #n "_switch" };	\
PNAME(mout_cpucl##n##_cpu_p) = { "mout_cpucl" #n "_pll",		\
				  "mout_cpucl" #n "_switch_user" };	\
static const struct samsung_mux_clock cpucl##n##_mux_clks[] __initconst = { \
	MUX_F(CLK_MOUT_CPUCL_PLL, "mout_cpucl" #n "_pll",		\
	      mout_cpucl##n##_pll_p, pll_con0, 4, 1,			\
	      CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT, 0),	\
	MUX(CLK_MOUT_CPUCL_SWITCH_USER, "mout_cpucl" #n "_switch_user",	\
	    mout_cpucl##n##_switch_user_p, switch_user, 4, 1),		\
	MUX_F(CLK_MOUT_CPUCL_CPU, "mout_cpucl" #n "_cpu",		\
	      mout_cpucl##n##_cpu_p, CLK_CON_MUX_MUX_CLK_CPUCL_PLL, 0, 1, \
	      CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT, 0),	\
};									\
static const struct samsung_div_clock cpucl##n##_div_clks[] __initconst = { \
	DIV_F(CLK_DOUT_CPUCL_CPU, "dout_cpucl" #n "_cpu",		\
	      "mout_cpucl" #n "_cpu", CLK_CON_DIV_DIV_CLK_CPUCL_CPU, 0, 3, \
	      CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,			\
	      CLK_DIVIDER_READ_ONLY),					\
}

CPUCL_CLKS(0, PLL_CON0_PLL_CPUCL0, PLL_CON0_MUX_CLKCMU_CPUCL0_SWITCH_USER,
	   cpucl0_pll_rates);
CPUCL_CLKS(1, PLL_CON0_PLL_CPUCL12, PLL_CON0_MUX_CLKCMU_CPUCL12_SWITCH_USER,
	   cpucl12_pll_rates);
CPUCL_CLKS(2, PLL_CON0_PLL_CPUCL12, PLL_CON0_MUX_CLKCMU_CPUCL12_SWITCH_USER,
	   cpucl12_pll_rates);

#define CPUCL_CMU_INFO(n, regs)						\
static const struct samsung_cmu_info cpucl##n##_cmu_info __initconst = { \
	.pll_clks		= cpucl##n##_pll_clks,			\
	.nr_pll_clks		= ARRAY_SIZE(cpucl##n##_pll_clks),	\
	.mux_clks		= cpucl##n##_mux_clks,			\
	.nr_mux_clks		= ARRAY_SIZE(cpucl##n##_mux_clks),	\
	.div_clks		= cpucl##n##_div_clks,			\
	.nr_div_clks		= ARRAY_SIZE(cpucl##n##_div_clks),	\
	.nr_clk_ids		= CLKS_NR_CPUCL,			\
	.clk_regs		= regs,					\
	.nr_clk_regs		= ARRAY_SIZE(regs),			\
	/* Keep the switching clock running for the PLL park. */	\
	.clk_name		= "dout_cpucl" #n "_switch",		\
}

CPUCL_CMU_INFO(0, cpucl0_clk_regs);
CPUCL_CMU_INFO(1, cpucl12_clk_regs);
CPUCL_CMU_INFO(2, cpucl12_clk_regs);

static int __init exynos7885_dispaud_setup(char *str)
{
	if (!str || strcmp(str, "1"))
		return -EINVAL;

	exynos7885_enable_dispaud = true;
	return 0;
}
early_param("exynos7885.dispaud", exynos7885_dispaud_setup);

/* ---- platform_driver ----------------------------------------------------- */

static int __init exynos7885_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *info;
	struct device *dev = &pdev->dev;

	info = of_device_get_match_data(dev);
	/* Registering this CMU writes gate manual/automatic bits at probe. */
	if (info == &dispaud_cmu_info && !exynos7885_enable_dispaud)
		return -ENODEV;

	exynos_arm64_register_cmu(dev, dev->of_node, info);

	if (info == &g3d_cmu_info)
		return exynos7885_g3d_register_notifier(dev);

	return 0;
}

static const struct of_device_id exynos7885_cmu_of_match[] = {
	{
		.compatible = "samsung,exynos7885-cmu-core",
		.data = &core_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-fsys",
		.data = &fsys_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-dispaud",
		.data = &dispaud_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-g3d",
		.data = &g3d_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-cpucl0",
		.data = &cpucl0_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-cpucl1",
		.data = &cpucl1_cmu_info,
	}, {
		.compatible = "samsung,exynos7885-cmu-cpucl2",
		.data = &cpucl2_cmu_info,
	}, {
	},
};

static struct platform_driver exynos7885_cmu_driver __refdata = {
	.driver	= {
		.name = "exynos7885-cmu",
		.of_match_table = exynos7885_cmu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = exynos7885_cmu_probe,
};

static int __init exynos7885_cmu_init(void)
{
	return platform_driver_register(&exynos7885_cmu_driver);
}
core_initcall(exynos7885_cmu_init);

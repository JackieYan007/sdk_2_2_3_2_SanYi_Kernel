/*
* clock driver for BST A1000
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/


#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/module.h>
#include "clk-bst.h"
#include <dt-bindings/clock/bst-a1000.h>



/* bst function declaration */
static int bst_pll_enable(struct clk_hw *hw);
static void bst_pll_disable(struct clk_hw *hw);
static int bst_pll_is_enabled(struct clk_hw *hw);
static long bst_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate);
static int bst_pll_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate);
static unsigned long bst_pll_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate);
static int bst_sec_pll_enable(struct clk_hw *hw);
static void bst_sec_pll_disable(struct clk_hw *hw);
static int bst_sec_pll_is_enabled(struct clk_hw *hw);
static long bst_sec_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate);
static int bst_sec_pll_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate);
static unsigned long bst_sec_pll_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate);
static int bst_pll_fpd_enable(struct clk_hw *hw);
static void bst_pll_fpd_disable(struct clk_hw *hw);
static int bst_pll_fpd_is_enabled(struct clk_hw *hw);
static long bst_pll_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate);
static int bst_pll_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate);
static int bst_pll_only_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
       unsigned long parent_rate);
static long bst_pll_only_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
       unsigned long *prate);
static unsigned long bst_pll_fpd_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate);
static int bst_pll_gmac_fpd_enable(struct clk_hw *hw);
static int bst_sec_pll_fpd_enable(struct clk_hw *hw);
static void bst_sec_pll_fpd_disable(struct clk_hw *hw);
static int bst_sec_pll_fpd_is_enabled(struct clk_hw *hw);
static long bst_sec_pll_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate);
static int bst_sec_pll_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate);
static unsigned long bst_sec_pll_fpd_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate);
/* bst function declaration end*/

/* bst global variable define */
static DEFINE_SPINLOCK(clk_spinlock); //for normal clock unit, such as gate/mux/divider...
static DEFINE_SPINLOCK(pll_cpu_spinlock);
static DEFINE_SPINLOCK(pll_dsu_spinlock);
static DEFINE_SPINLOCK(pll_sysbus_spinlock);
static DEFINE_SPINLOCK(pll_disp_spinlock);
static DEFINE_SPINLOCK(pll_coreip_spinlock);
static DEFINE_SPINLOCK(pll_hsplsp_spinlock);
static DEFINE_SPINLOCK(pll_gmac_spinlock);

static DEFINE_SPINLOCK(pll_main_spinlock);
static DEFINE_SPINLOCK(pll_lsp_spinlock);

void __iomem *a1000_clk_base_addr[4];
static struct clk *clks[CLK_MAX];
static struct clk_onecell_data bst_clk_data;

/*
 * When changing pll settings should first switch mux to osc output
 * And switch to pll output when pll lock stable
 */
static bst_pll_mux pll_cpu_mux[] ={
    // CLK_CPU_1400_SEL
    {
        .mux_oft = 27,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_dsu_mux[] = {
    // CLK_DSU_1300_SEL
    {
        .mux_oft = 28,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_sysbus_mux[] = {
    // CLK_2400_SYSBUS_SEL
    {
        .mux_oft = 25,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_sysbus_ptd_mux[] = {
    // CLK_800_SYSBUS_SEL
    {
        .mux_oft = 26,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_disp_ptd_mux[] = {
    // CLK_DISPLAY_1650_SEL
    {
        .mux_oft = 24,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_coreip_ptd_mux[] = {
    // CLK_800_COREIP_SEL
    {
        .mux_oft = 23,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_hsplsp_mux[] = {
    // CLK_2000_SEL
    {
        .mux_oft = 22,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_main_ptd_mux[] = {
    // SEC_SAFE_CLK_SEL0
    {
        .mux_oft = 0,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    },
    // SEC_SAFE_CLK_SEL1
    {
        .mux_oft = 1,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    },
    // SEC_SAFE_CLK_SEL2
    {
        .mux_oft = 2,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    },
    // SEC_SAFE_CLK_SEL3
    {
        .mux_oft = 3,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

static bst_pll_mux pll_lsp_ptd_mux[] = {
    // SEC_SAFE_CLK_SEL4
    {
        .mux_oft = 4,
        .mux_width = 1,
        .mux_osc_val = 0,
        .mux_pll_val = 1,
    }
};

/* top crm muxs */
static const char *const cpu_1400_sel_parents[] = { MAIN_CLK, PLL_CPU };
static const char *const dsu_1300_sel_parents[] = { MAIN_CLK, PLL_DSU };
static const char *const clk_1400_cpucore_sel_parents[] = { MAIN_CLK, CLK_CPU_1400_SEL, 
    CLK_2000_SEL_FIX_FACTOR_2_1, CLK_CPU_1400_SEL_FIX_FACTOR_2_1 };
static const char *const clk_dsu_sel_parents[] = { CLK_1400_CPUCORE_SEL,  CLK_DSU_1300_SEL };
static const char *const clk_2000_sel_parents[] = { MAIN_CLK, PLL_HSP_LSP };
static const char *const clk_800_sysbus_sel_parents[] = { MAIN_CLK, PLL_SYSBUS_PTD };
static const char *const clk_400_sysnoc_work_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1 };
static const char *const clk_2400_sysbus_sel_parents[] = { MAIN_CLK, PLL_SYSBUS };
static const char *const clk_800_corenoc_work_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL,
    CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1 };
static const char *const clk_400_sysbus_axi_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1,
    CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1 };
static const char *const clk_800_cpunoc_work_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_800_gdma_axi_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_100_sysbus_apb_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1, MAIN_CLK };
static const char *const clk_200_sysbus_ahb_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1 };
static const char *const clk_800_sysbus_axi_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_125_ptp_sel_parents[] = { PLL_GMAC_PTD_FIX_FACTOR_4_1, CLK_2000_SEL_FIX_FACTOR_16_1,
    CLK_2000_SEL_FIX_FACTOR_8_1, MAIN_CLK };
static const char *const pll_gmac_ref_25m_125m_sel_parents[] = { MAIN_CLK, CLK_2000_SEL_FIX_FACTOR_16_1 };
static const char *const pll_gmac_ref_clk_sel_parents[] = { PLL_GMAC_REF_25M_125M_SEL, RGMII0_RXCLK,
    RGMII1_RXCLK, PTP_CLK };
static const char *const clk_200_sdemmc0_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_200_sdemmc1_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_400_gdma_core_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1 };
static const char *const clk_800_coreip_sel_parents[] = { MAIN_CLK, PLL_COREIP_PTD_FIX_FACTOR };
static const char *const clk_800_cv_core_sel_parents[] = { MAIN_CLK, CLK_800_COREIP_SEL,
    CLK_800_COREIP_SEL_FIX_FACTOR_2_1, CLK_800_COREIP_SEL_FIX_FACTOR_4_1 };
static const char *const clk_800_net_sel_parents[] = { MAIN_CLK, CLK_800_COREIP_SEL,
    CLK_800_COREIP_SEL_FIX_FACTOR_2_1, CLK_800_COREIP_SEL_FIX_FACTOR_4_1 };
static const char *const clk_800_dspcore_sel_parents[] = { MAIN_CLK, CLK_800_COREIP_SEL,
    CLK_800_COREIP_SEL_FIX_FACTOR_2_1, CLK_800_COREIP_SEL_FIX_FACTOR_4_1 };
static const char *const clk_display_1650_sel_parents[] = { MAIN_CLK, PLL_DISP_PTD };
static const char *const clk_800_gpu_sel_parents[] = { CLK_800_SYSBUS_SEL,  CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1 };
static const char *const clk_200_sysbus_apb_sel_parents[] = { MAIN_CLK, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1,
    CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1, CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1 };


/* sec safe crm muxs */
static const char *const sec_safe_clk_sel0_parents[] = { MAIN_CLK, PLL_MAIN_PTD };
static const char *const sec_safe_clk_sel1_parents[] = { MAIN_CLK, PLL_MAIN_PTD_FIX_FACTOR_2_1 };
static const char *const sec_safe_clk_sel2_parents[] = { MAIN_CLK, PLL_MAIN_PTD_FIX_FACTOR_4_1 };
static const char *const sec_safe_clk_sel3_parents[] = { MAIN_CLK, PLL_MAIN_PTD_FIX_FACTOR_8_1 };
static const char *const sec_safe_clk_sel9_8_parents[] = { MAIN_CLK, CLK_QSPI_REF_DIV,
    SEC_SAFE_CLK_SEL0, SEC_SAFE_CLK_SEL2 };
static const char *const sec_safe_clk_sel7_parents[] = { MAIN_CLK, LB_SEC_SAFE_CANFD_200M_CLK_GATE_EN };
static const char *const sec_safe_clk_sel4_parents[] = { MAIN_CLK, PLL_LSP_PTD_FIX_FACTOR };
static const char *const lsp0_i2c_mux_parents[] = { LSP0_I2C_WCLK_FIX_FACTOR_2_1, LSP0_WCLK_EN };
static const char *const lsp1_i2c_mux_parents[] = { LSP1_I2C_WCLK_FIX_FACTOR_2_1, LSP1_WCLK_EN };


/* PLL FOUT operations 
 * FOUT mode PLL can ajust PLL_FBDIV/FRAC to set rate
 */
static const struct clk_ops bst_pll_fo_ops = {
	.enable         = bst_pll_enable,
	.disable        = bst_pll_disable,
	.is_enabled     = bst_pll_is_enabled,
	.round_rate     = bst_pll_round_rate,
	.set_rate       = bst_pll_set_rate,
	.recalc_rate    = bst_pll_recalc_rate,
};

/* PLL FOUTPOSTDIV operations 
 * POSTDIV mode PLL only allow ajust postdiv to set rate
 */
static const struct clk_ops bst_pll_fpd_ops = {
	.enable         = bst_pll_fpd_enable,
	.disable        = bst_pll_fpd_disable,
	.is_enabled     = bst_pll_fpd_is_enabled,
	.round_rate     = bst_pll_fpd_round_rate,
	.set_rate       = bst_pll_fpd_set_rate,
	.recalc_rate    = bst_pll_fpd_recalc_rate,
};

/* ONLY PLL FOUTPOSTDIV operations */
static const struct clk_ops bst_pll_only_fpd_ops = {
	.enable         = bst_pll_fpd_enable,
	.disable        = bst_pll_fpd_disable,
	.is_enabled     = bst_pll_fpd_is_enabled,
	.round_rate     = bst_pll_only_fpd_round_rate,
	.set_rate       = bst_pll_only_fpd_set_rate,
	.recalc_rate    = bst_pll_fpd_recalc_rate,
};

/* PLL FOUTPOSTDIV operations 
 * POSTDIV mode PLL only allow ajust postdiv to set rate
 */
static const struct clk_ops bst_pll_gmac_fpd_ops = {
	.enable         = bst_pll_gmac_fpd_enable,
	.disable        = bst_pll_fpd_disable,
	.is_enabled     = bst_pll_fpd_is_enabled,
	.round_rate     = bst_pll_fpd_round_rate,
	.set_rate       = bst_pll_fpd_set_rate,
	.recalc_rate    = bst_pll_fpd_recalc_rate,
};

static const struct clk_ops bst_sec_pll_ops = {
	.enable         = bst_sec_pll_enable,
	.disable        = bst_sec_pll_disable,
	.is_enabled     = bst_sec_pll_is_enabled,
	.round_rate     = bst_sec_pll_round_rate,
	.set_rate       = bst_sec_pll_set_rate,
	.recalc_rate    = bst_sec_pll_recalc_rate,
};

/* PLL FOUTPOSTDIV operations 
 * POSTDIV mode PLL only allow ajust postdiv to set rate
 */
static const struct clk_ops bst_sec_pll_fpd_ops = {
	.enable         = bst_sec_pll_fpd_enable,
	.disable        = bst_sec_pll_fpd_disable,
	.is_enabled     = bst_sec_pll_fpd_is_enabled,
	.round_rate     = bst_sec_pll_fpd_round_rate,
	.set_rate       = bst_sec_pll_fpd_set_rate,
	.recalc_rate    = bst_sec_pll_fpd_recalc_rate,
};

/* bst clock global variable define end*/

/**
 * bst_cal_pdiv1_pdiv2 - find pdiv1*pdiv2 = pdiv
 */
static void bst_cal_pdiv1_pdiv2(u32 pdiv, u32 *pdiv1, u32 *pdiv2)
{
    u32 i, j;

    *pdiv1 = 1;
    *pdiv2 = 1;
    for(i=1; i <= PLL_PSTDIV_MASK; i++) {
        for(j=1; j <= PLL_PSTDIV_MASK; j++) {
            if(pdiv == i*j) {
                *pdiv1 = i;
                *pdiv2 = j;
                return;
            }
        }
    }
}

/**
 * bst_cal_pll_val - calculate FOUT*(2^24)/FREF
 * @fout: FOUT, will raise to u64 type
 * @fref: FREF, will raise to u64 type
 * return the u64 type result
 */
static inline u64 bst_cal_pll_val(u64 fout, u64 fref)
{
    return (fout<<PLL_FRAC_SHIFT)/fref;
}

/**
 * bst_cal_pll_val - calculate FOUT*(2^24)/FREF
 * @fbdiv: integer divider
 * @fref: FREF, will raise to u64 type
 * return the u64 type result
 */
static inline u64 integer_mode_fout_rate(u64 fbdiv, u64 fref)
{
    return fbdiv*fref;
}

/**
 * bst_cal_pll_val - calculate FOUT*(2^24)/FREF
 * @fbdiv: integer divider
 * @fbdiv: fraction divider
 * @fref: FREF, will raise to u64 type
 * return the u64 type result
 */
static inline u64 fraction_mode_fout_rate(u64 fbdiv, u64 frac, u64 fref)
{
    return DIV_ROUND_CLOSEST( ((fbdiv<<PLL_FRAC_SHIFT)+frac)*fref, (1<<PLL_FRAC_SHIFT));
}

/**
 * bst_pll_enable - Enable clock
 * @hw:		Handle between common and hardware-specific interfaces
 * Returns 0 on success
 */
static int bst_pll_enable(struct clk_hw *hw)
{
	struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
	u32 reg;

	if (bst_pll_is_enabled(hw))
		return 0;

	/* Power up PLL and wait for lock */
	spin_lock_irqsave(clk->pll_spinlock, flags);

	reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg |= (1 << PLL_PLLEN);
	writel(reg, clk->base_addr + PLL_REG_CONFIG0);
	while (!(readl(clk->base_addr + PLL_REG_CONFIG2) & (1 << PLL_LOCK))) {
    }

	spin_unlock_irqrestore(clk->pll_spinlock, flags);

	return 0;
}

/**
 * bst_pll_disable - Disable clock
 * @hw:		Handle between common and hardware-specific interfaces
 */
static void bst_pll_disable(struct clk_hw *hw)
{
	struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
	u32 reg;

	if (! bst_pll_is_enabled(hw))
		return;

	/* shut down PLL */
	spin_lock_irqsave(clk->pll_spinlock, flags);

	reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= (~(1 << PLL_PLLEN));
	writel(reg, clk->base_addr + PLL_REG_CONFIG0);

	spin_unlock_irqrestore(clk->pll_spinlock, flags);
}

/**
 * bst_pll_is_enabled - Check if a clock is enabled
 * @hw:		Handle between common and hardware-specific interfaces
 * Returns 1 if the clock is enabled, 0 otherwise.
 *
 */
static int bst_pll_is_enabled(struct clk_hw *hw)
{
	struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
	u32 reg;

    spin_lock_irqsave(clk->pll_spinlock, flags);

	reg = readl(clk->base_addr + PLL_REG_CONFIG0);

	spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return (reg & (1<<PLL_PLLEN)) ? 1 : 0;
}

/**
 * bst_pll_round_rate() - Round a clock frequency
 * @hw:		Handle between common and hardware-specific interfaces
 * @rate:	Desired clock frequency
 * @prate:	Clock frequency of parent clock
 * Returns frequency closest to @rate the hardware can generate.
 */
static long bst_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
    u64 pll_val;

    /* 
     * 5M <= FREF <= 1200M, FREF is from oscillator
     * 10M <= FOUT <= 1200M
     */
    if(rate > PLL_FOUT_MAX_RATE) {
        rate = PLL_FOUT_MAX_RATE;
    } else if(rate < PLL_FOUT_MIN_RATE) {
        rate = PLL_FOUT_MIN_RATE;
    }

    /*
     * FBDIV minimal value is 1(also treat 0 as 1)
     */
    if(rate < *prate) {
        return *prate;
    }
    
    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     */
    pll_val = bst_cal_pll_val(rate, *prate);

    /*
     * =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     * =>FOUT = pll_val*FREF/(2^PLL_FRAC_SHIFT)
     */
    return DIV_ROUND_CLOSEST( ((*prate)*pll_val), (1<<PLL_FRAC_SHIFT));
}

/**
 * bst_pll_only_fpd_round_rate() - Round a clock frequency
 * @hw:		Handle between common and hardware-specific interfaces
 * @rate:	Desired clock frequency
 * @prate:	Clock frequency of parent clock
 * Returns frequency closest to @rate the hardware can generate.
 */
static long bst_pll_only_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *prate)
{
    u64 pll_val;
    u32 ptd_fix_factor= 1, val;
    struct bst_pll *clk = to_bst_pll(hw);

    val = readl(clk->base_addr+PLL_REG_CONFIG0);
    ptd_fix_factor = (val>>PLL_PSTDIV1)&PLL_PSTDIV_MASK; //postdiv1 can be 0!
    ptd_fix_factor = (ptd_fix_factor==0) ? 1: ptd_fix_factor;

    val = (val>>PLL_PSTDIV2)&PLL_PSTDIV_MASK; //postdiv2 can be 0!
    val = (val==0) ? 1: val;
    ptd_fix_factor = val*ptd_fix_factor;

    rate = rate*ptd_fix_factor;

    /*
     * 5M <= FREF <= 1200M, FREF is from oscillator
     * 10M <= FOUT <= 1200M
     */
    if(rate > PLL_FOUT_MAX_RATE) {
        rate = PLL_FOUT_MAX_RATE;
    } else if(rate < PLL_FOUT_MIN_RATE) {
        rate = PLL_FOUT_MIN_RATE;
    }

    /*
     * FBDIV minimal value is 1(also treat 0 as 1)
     */
    if(rate < *prate) {
        return *prate;
    }

    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     */
    pll_val = bst_cal_pll_val(rate, *prate);

    /*
     * =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     * =>FOUT = pll_val*FREF/(2^PLL_FRAC_SHIFT)
     */
    return (DIV_ROUND_CLOSEST( ((*prate)*pll_val), (1<<PLL_FRAC_SHIFT)))/ptd_fix_factor;
}

static int bst_pll_only_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
    u64 pll_val;
	struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
	u32 reg, fbdiv, frac, mask, i, val, ptd_fix_factor= 1;

    val = readl(clk->base_addr+PLL_REG_CONFIG0);
    ptd_fix_factor = (val>>PLL_PSTDIV1)&PLL_PSTDIV_MASK; //postdiv1 can be 0!
    ptd_fix_factor = (ptd_fix_factor==0) ? 1: ptd_fix_factor;

    val = (val>>PLL_PSTDIV2)&PLL_PSTDIV_MASK; //postdiv2 can be 0!
    val = (val==0) ? 1: val;
    ptd_fix_factor = val*ptd_fix_factor;

    rate = rate*ptd_fix_factor;

    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     * fbdiv = pll_val / (2^PLL_FRAC_SHIFT)
     * frac = pll_val % (2^PLL_FRAC_SHIFT)
     */
    pll_val = bst_cal_pll_val(rate, parent_rate);
    fbdiv = (u32) (pll_val >> PLL_FRAC_SHIFT);
    frac = (u32) (pll_val & ((1<<PLL_FRAC_SHIFT) - 1));

    spin_lock_irqsave(clk->pll_spinlock, flags);

    /* switch all muxs to osc */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        (clk->pll_mux)[i].mux_pll_val = (reg>>((clk->pll_mux)[i].mux_oft)) & mask;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_osc_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= ~(1<<PLL_PLLEN); //PLLEN = disable
    reg &= ~(PLL_FBDIV_MASK<<PLL_FBDIV); //clear fbdiv
    reg |= (fbdiv & PLL_FBDIV_MASK)<<PLL_FBDIV; //re-write fbdiv
    if(frac) {
        reg |= (1<<PLL_DSMEN); //enable DSMEN
    } else {
        reg &= ~(1<<PLL_DSMEN); //disable DSMEN
    }
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    if(frac) {
        reg = readl(clk->base_addr + PLL_REG_CONFIG1);
        reg &= ~(PLL_FRAC_MASK<<PLL_FRAC); //clear frac
        reg |= (frac & PLL_FRAC_MASK)<<PLL_FRAC; //re-write frac
        writel(reg, clk->base_addr + PLL_REG_CONFIG1);
    }

/* need memory barrier insert here?? */

    writel(readl(clk->base_addr + PLL_REG_CONFIG0)|(1<<PLL_PLLEN), clk->base_addr + PLL_REG_CONFIG0); //PLLEN = enable

    /* wait pll lock */
    while (!(readl(clk->base_addr + PLL_REG_CONFIG2) & (1 << PLL_LOCK))) {
    }

    /* switch from osc to pll output */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_pll_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}

/**
 * bst_pll_set_rate() - Change the rate of this clock. The requested rate is specified
 * by the second argument, which should typically be the return of .round_rate call.
 * The third argument gives the parent rate which is likely helpful for most .set_rate implementation.
 * @hw:             Handle between common and hardware-specific interfaces
 * @rate:	        Desired clock frequency
 * @parent_rate:    Clock frequency of parent clock
 * Returns 0 on success, -EERROR otherwise.
 */
static int bst_pll_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
    u64 pll_val;
	struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
	u32 reg, fbdiv, frac, mask, i;

    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     * fbdiv = pll_val / (2^PLL_FRAC_SHIFT)
     * frac = pll_val % (2^PLL_FRAC_SHIFT)
     */
    pll_val = bst_cal_pll_val(rate, parent_rate);
    fbdiv = (u32) (pll_val >> PLL_FRAC_SHIFT);
    frac = (u32) (pll_val & ((1<<PLL_FRAC_SHIFT) - 1));
    
    spin_lock_irqsave(clk->pll_spinlock, flags);

    /* switch all muxs to osc */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        (clk->pll_mux)[i].mux_pll_val = (reg>>((clk->pll_mux)[i].mux_oft)) & mask;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_osc_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= ~(1<<PLL_PLLEN); //PLLEN = disable
    reg &= ~(PLL_FBDIV_MASK<<PLL_FBDIV); //clear fbdiv
    reg |= (fbdiv & PLL_FBDIV_MASK)<<PLL_FBDIV; //re-write fbdiv
    if(frac) {
        reg |= (1<<PLL_DSMEN); //enable DSMEN
    } else {
        reg &= ~(1<<PLL_DSMEN); //disable DSMEN
    }
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    if(frac) {
        reg = readl(clk->base_addr + PLL_REG_CONFIG1);
        reg &= ~(PLL_FRAC_MASK<<PLL_FRAC); //clear frac
        reg |= (frac & PLL_FRAC_MASK)<<PLL_FRAC; //re-write frac
        writel(reg, clk->base_addr + PLL_REG_CONFIG1);
    }

/* need memory barrier insert here?? */
    
    writel(readl(clk->base_addr + PLL_REG_CONFIG0)|(1<<PLL_PLLEN), clk->base_addr + PLL_REG_CONFIG0); //PLLEN = enable
    
    /* wait pll lock */
    while (!(readl(clk->base_addr + PLL_REG_CONFIG2) & (1 << PLL_LOCK))) {
    }

    /* switch from osc to pll output */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_pll_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);
    
    return 0;
}      


/**
 * bst_pll_recalc_rate() - Recalculate clock frequency
 * @hw:         Handle between common and hardware-specific interfaces
 * @parent_rate:    Clock frequency of parent clock
 * Returns current clock frequency.
 */
static unsigned long bst_pll_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg_config0, reg_config1, fbdiv;

    spin_lock_irqsave(clk->pll_spinlock, flags);

	reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    fbdiv = (reg_config0>>PLL_FBDIV) & PLL_FBDIV_MASK;
    fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
    if(reg_config0 & (1<<PLL_DSMEN)) {
        return fraction_mode_fout_rate(fbdiv, (reg_config1>>PLL_FRAC) & PLL_FRAC_MASK, parent_rate);
    } else {
        return integer_mode_fout_rate(fbdiv, parent_rate);
    }
}

/**
 * bst_sec_pll_enable - Enable clock
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 0 on success
 */
static int bst_sec_pll_enable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (bst_sec_pll_is_enabled(hw))
        return 0;

    /* Power up PLL and wait for lock */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg |= (1 << PLL_SEC_PLLEN);
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);
    while (!(readl(clk->base_addr + PLL_REG_STATUS) & (1 << PLL_SEC_LOCK))) {
    }

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}

/**
 * bst_sec_pll_disable - Disable clock
 * @hw:     Handle between common and hardware-specific interfaces
 */
static void bst_sec_pll_disable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (! bst_sec_pll_is_enabled(hw))
        return;

    /* shut down PLL */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= (~(1 << PLL_SEC_PLLEN));
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);
}

/**
 * bst_sec_pll_is_enabled - Check if a clock is enabled
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 1 if the clock is enabled, 0 otherwise.
 *
 */
static int bst_sec_pll_is_enabled(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return (reg & (1<<PLL_SEC_PLLEN)) ? 1 : 0;
}

/**
 * bst_sec_pll_round_rate() - Round a clock frequency
 * @hw:     Handle between common and hardware-specific interfaces
 * @rate:   Desired clock frequency
 * @prate:  Clock frequency of parent clock
 * Returns frequency closest to @rate the hardware can generate.
 */
static long bst_sec_pll_round_rate(struct clk_hw *hw, unsigned long rate,
        unsigned long *prate)
{
    u64 pll_val;

    /* 
     * 5M <= FREF <= 1200M, FREF is from oscillator
     * 10M <= FOUT <= 1200M
     */
    if(rate > PLL_FOUT_MAX_RATE) {
        rate = PLL_FOUT_MAX_RATE;
    } else if(rate < PLL_FOUT_MIN_RATE) {
        rate = PLL_FOUT_MIN_RATE;
    }

    /*
     * FBDIV minimal value is 1(also treat 0 as 1)
     */
    if(rate < *prate) {
        return *prate;
    }
    
    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     */
    pll_val = bst_cal_pll_val(rate, *prate);

    /*
     * =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     * =>FOUT = pll_val*FREF/(2^PLL_FRAC_SHIFT)
     */
    return DIV_ROUND_CLOSEST( ((*prate)*pll_val), (1<<PLL_FRAC_SHIFT));
}

/**
 * bst_sec_pll_set_rate() - Change the rate of this clock. The requested rate is specified
 * by the second argument, which should typically be the return of .round_rate call.
 * The third argument gives the parent rate which is likely helpful for most .set_rate implementation.
 * @hw:             Handle between common and hardware-specific interfaces
 * @rate:           Desired clock frequency
 * @parent_rate:    Clock frequency of parent clock
 * Returns 0 on success, -EERROR otherwise.
 */
static int bst_sec_pll_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
    u64 pll_val;
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg, fbdiv, frac, mask, i;

    /*
     * FOUT = (FREF/REFDIV)*(FBDIV + FRAC/(2^PLL_FRAC_SHIFT))
     * FOUT: here is rate
     * FREF: here is *prate
     * REFDIV: designer suggest fix to 1
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC = FOUT*(2^PLL_FRAC_SHIFT)/FREF
     *  =>FBDIV*(2^PLL_FRAC_SHIFT) + FRAC: pll_val
     * fbdiv = pll_val / (2^PLL_FRAC_SHIFT)
     * frac = pll_val % (2^PLL_FRAC_SHIFT)
     */
    pll_val = bst_cal_pll_val(rate, parent_rate);
    fbdiv = (u32) (pll_val >> PLL_FRAC_SHIFT);
    frac = (u32) (pll_val & ((1<<PLL_FRAC_SHIFT) - 1));
    
    spin_lock_irqsave(clk->pll_spinlock, flags);

    /* switch all muxs to osc */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        (clk->pll_mux)[i].mux_pll_val = (reg>>((clk->pll_mux)[i].mux_oft)) & mask;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_osc_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= ~(1<<PLL_SEC_PLLEN); //PLLEN = disable
    reg &= ~(PLL_SEC_FBDIV_MASK<<PLL_SEC_FBDIV); //clear fbdiv
    reg |= (fbdiv & PLL_SEC_FBDIV_MASK)<<PLL_SEC_FBDIV; //re-write fbdiv
    if(frac) {
        reg |= (1<<PLL_SEC_DSMEN); //enable DSMEN
    } else {
        reg &= ~(1<<PLL_SEC_DSMEN); //disable DSMEN
    }
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    if(frac) {
        reg = readl(clk->base_addr + PLL_REG_CONFIG1);
        reg &= ~(PLL_SEC_FRAC_MASK<<PLL_FRAC); //clear frac
        reg |= (frac & PLL_SEC_FRAC_MASK)<<PLL_FRAC; //re-write frac
        writel(reg, clk->base_addr + PLL_REG_CONFIG1);
    }

/* need memory barrier insert here?? */
    
    writel(readl(clk->base_addr + PLL_REG_CONFIG0)|(1<<PLL_SEC_PLLEN), clk->base_addr + PLL_REG_CONFIG0); //PLLEN = enable
    
    /* wait pll lock */
    while (!(readl(clk->base_addr + PLL_REG_STATUS) & (1 << PLL_SEC_LOCK))) {
    }

    /* switch from osc to pll output */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_pll_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);
    
    return 0;
}      


/**
 * bst_sec_pll_recalc_rate() - Recalculate clock frequency
 * @hw:         Handle between common and hardware-specific interfaces
 * @parent_rate:    Clock frequency of parent clock
 * Returns current clock frequency.
 */
static unsigned long bst_sec_pll_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg_config0, reg_config1, fbdiv;

    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    fbdiv = (reg_config0>>PLL_SEC_FBDIV) & PLL_SEC_FBDIV_MASK;
    fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
    if(reg_config0 & (1<<PLL_SEC_DSMEN)) {
        return fraction_mode_fout_rate(fbdiv, (reg_config1>>PLL_SEC_FRAC) & PLL_SEC_FRAC_MASK, parent_rate);
    } else {
        return integer_mode_fout_rate(fbdiv, parent_rate);
    }
}


/**
 * bst_pll_gmac_fpd_enable - Enable clock
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 0 on success
 */
static int bst_pll_gmac_fpd_enable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (bst_pll_is_enabled(hw))
        return 0;

    /* Power up PLL and wait for lock */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg |= ((1 << PLL_PLLEN) | (1 << PLL_FOUT4PHASEEN));
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);
    while (!(readl(clk->base_addr + PLL_REG_CONFIG2) & (1 << PLL_GMAC_LOCK))) {
    }

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}

/**
 * bst_pll_enable - Enable clock
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 0 on success
 */
static int bst_pll_fpd_enable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (bst_pll_is_enabled(hw))
        return 0;

    /* Power up PLL and wait for lock */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg |= ((1 << PLL_PLLEN) | (1 << PLL_FOUT4PHASEEN));
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);
    while (!(readl(clk->base_addr + PLL_REG_CONFIG2) & (1 << PLL_LOCK))) {
    }

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}

/**
 * bst_pll_disable - Disable clock
 * @hw:     Handle between common and hardware-specific interfaces
 */
static void bst_pll_fpd_disable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (! bst_pll_is_enabled(hw))
        return;

    /* shut down PLL */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= (~(1 << PLL_PLLEN)); //only disable PLLEN is ok
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);
}

/**
 * bst_pll_is_enabled - Check if a clock is enabled
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 1 if the clock is enabled, 0 otherwise.
 *
 */
static int bst_pll_fpd_is_enabled(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return (reg & (1<<PLL_PLLEN)) && (reg & (1<<PLL_FOUT4PHASEEN));
}

/**
 * bst_pll_fpd_round_rate() - please refer bst_pll_round_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static long bst_pll_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
        unsigned long *prate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, postdiv, frac, fout;
    u32 reg_config0, reg_config1;
    unsigned long flags = 0;

    spin_lock_irqsave(clk->pll_spinlock, flags);

	reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    if(reg_config0 & (1<<PLL_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_FRAC)&PLL_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, *(prate));
        postdiv = fout/rate;
        
        if(postdiv < PLL_PSTDIV1_MIN*PLL_PSTDIV2_MIN) {
            return fout;
        } else if(postdiv > PLL_PSTDIV1_MAX*PLL_PSTDIV2_MAX) {
            return fout / PLL_PSTDIV1_MAX / PLL_PSTDIV2_MAX;
        } else {
            return fout/postdiv;
        }
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        postdiv = *prate * (fbdiv);
        postdiv = postdiv/rate; //POSTDIV1*POSTDIV2

        if(postdiv < PLL_PSTDIV1_MIN*PLL_PSTDIV2_MIN) {
            return *prate * (fbdiv);
        } else if(postdiv > PLL_PSTDIV1_MAX*PLL_PSTDIV2_MAX) {
            return *prate *(fbdiv) / PLL_PSTDIV1_MAX / PLL_PSTDIV2_MAX;
        } else {
            return *prate *(fbdiv) / postdiv;
        }
    }
}

/**
 * bst_pll_fdp_set_rate() - please refer bst_pll_set_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static int bst_pll_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, postdiv, frac, fout;
    u32 reg_config0, reg_config1, reg;
    u32 pdiv1, pdiv2, mask, i;
    unsigned long flags = 0;
    
    spin_lock_irqsave(clk->pll_spinlock, flags);

    /* switch all muxs to osc */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        (clk->pll_mux)[i].mux_pll_val = (reg>>((clk->pll_mux)[i].mux_oft)) & mask;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_osc_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);
    
    if(reg_config0 & (1<<PLL_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_FRAC)&PLL_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, parent_rate);
        postdiv = fout/rate;
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        fout = integer_mode_fout_rate(fbdiv, parent_rate);
        postdiv = fout/rate;
    }
    
    postdiv = (postdiv==0) ? 1 : postdiv;
    postdiv = postdiv>PLL_PSTDIV1_MAX*PLL_PSTDIV2_MAX ? PLL_PSTDIV1_MAX*PLL_PSTDIV2_MAX : postdiv;
    bst_cal_pdiv1_pdiv2(postdiv, &pdiv1, &pdiv2);

    reg_config0 &= ~((PLL_PSTDIV_MASK<<PLL_PSTDIV1) | (PLL_PSTDIV_MASK<<PLL_PSTDIV2)); //clear pdiv1 and pdiv2
    reg_config0 |= ((pdiv1<<PLL_PSTDIV1) | (pdiv2<<PLL_PSTDIV2)); //re-write pdiv1 and pdiv2
    writel(reg_config0, clk->base_addr + PLL_REG_CONFIG0);

    /* switch from osc to pll output */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_pll_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}      


/**
 * bst_pll_fpd_recalc_rate() - please refer bst_pll_recalc_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static unsigned long bst_pll_fpd_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, frac, fout;
    u32 reg_config0, reg_config1;
    u32 pdiv1, pdiv2;
    unsigned long flags = 0;
    
    spin_lock_irqsave(clk->pll_spinlock, flags);
    
    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    pdiv1 = (reg_config0>>PLL_PSTDIV1)&PLL_PSTDIV_MASK;
    pdiv2 = (reg_config0>>PLL_PSTDIV2)&PLL_PSTDIV_MASK;
    pdiv1 = pdiv1 ? pdiv1 : 1; //treat 0 as 1
    pdiv2 = pdiv2 ? pdiv2 : 1; //treat 0 as 1
    if(reg_config0 & (1<<PLL_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_FRAC)&PLL_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, parent_rate);
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_FBDIV)&PLL_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        fout = integer_mode_fout_rate(fbdiv, parent_rate);
    }

    return fout/(pdiv1*pdiv2);
}

/**
 * bst_pll_enable - Enable clock
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 0 on success
 */
static int bst_sec_pll_fpd_enable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (bst_sec_pll_fpd_is_enabled(hw))
        return 0;

    /* Power up PLL and wait for lock */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg |= (1 << PLL_SEC_PLLEN);
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    reg = readl(clk->base_addr + PLL_REG_CONFIG1);
    reg |= (1 << PLL_SEC_FOUT4PHASEEN);
    writel(reg, clk->base_addr + PLL_REG_CONFIG1);
    while (!(readl(clk->base_addr + PLL_REG_STATUS) & (1 << PLL_SEC_LOCK))) {
    }

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}

/**
 * bst_pll_disable - Disable clock
 * @hw:     Handle between common and hardware-specific interfaces
 */
static void bst_sec_pll_fpd_disable(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg;

    if (! bst_sec_pll_fpd_is_enabled(hw))
        return;

    /* shut down PLL */
    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg &= (~(1 << PLL_SEC_PLLEN)); //only disable PLLEN is ok
    writel(reg, clk->base_addr + PLL_REG_CONFIG0);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);
}

/**
 * bst_pll_is_enabled - Check if a clock is enabled
 * @hw:     Handle between common and hardware-specific interfaces
 * Returns 1 if the clock is enabled, 0 otherwise.
 *
 */
static int bst_sec_pll_fpd_is_enabled(struct clk_hw *hw)
{
    struct bst_pll *clk = to_bst_pll(hw);
    unsigned long flags = 0;
    u32 reg0, reg1;

    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg1 = readl(clk->base_addr + PLL_REG_CONFIG1);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return (reg0 & (1<<PLL_SEC_PLLEN)) && (reg1 & (1<<PLL_SEC_FOUT4PHASEEN));
}

/**
 * bst_pll_fpd_round_rate() - please refer bst_pll_round_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static long bst_sec_pll_fpd_round_rate(struct clk_hw *hw, unsigned long rate,
        unsigned long *prate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, postdiv, frac, fout;
    u32 reg_config0, reg_config1;
    unsigned long flags = 0;

    spin_lock_irqsave(clk->pll_spinlock, flags);

    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);

    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    if(reg_config0 & (1<<PLL_SEC_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_SEC_FRAC)&PLL_SEC_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, *(prate));
        postdiv = fout/rate;
        
        if(postdiv < PLL_SEC_PSTDIV1_MIN*PLL_SEC_PSTDIV2_MIN) {
            return fout;
        } else if(postdiv > PLL_SEC_PSTDIV1_MAX*PLL_SEC_PSTDIV2_MAX) {
            return fout / PLL_SEC_PSTDIV1_MAX / PLL_SEC_PSTDIV2_MAX;
        } else {
            return fout/postdiv;
        }
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        postdiv = *prate * (fbdiv);
        postdiv = postdiv/rate; //POSTDIV1*POSTDIV2

        if(postdiv < PLL_SEC_PSTDIV1_MIN*PLL_SEC_PSTDIV2_MIN) {
            return *prate *(fbdiv);
        } else if(postdiv > PLL_SEC_PSTDIV1_MAX*PLL_SEC_PSTDIV2_MAX) {
            return *prate *(fbdiv) / PLL_SEC_PSTDIV1_MAX / PLL_SEC_PSTDIV2_MAX;
        } else {
            return *prate *(fbdiv) / postdiv;
        }
    }
}

/**
 * bst_pll_fdp_set_rate() - please refer bst_pll_set_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static int bst_sec_pll_fpd_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, postdiv, frac, fout;
    u32 reg_config0, reg_config1, reg;
    u32 pdiv1, pdiv2, mask, i;
    unsigned long flags = 0;
    
    spin_lock_irqsave(clk->pll_spinlock, flags);

    /* switch all muxs to osc */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        (clk->pll_mux)[i].mux_pll_val = (reg>>((clk->pll_mux)[i].mux_oft)) & mask;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_osc_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);
    
    if(reg_config0 & (1<<PLL_SEC_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_SEC_FRAC)&PLL_SEC_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, parent_rate);
        postdiv = fout/rate;
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        fout = integer_mode_fout_rate(fbdiv, parent_rate);
        postdiv = fout/rate;
    }
    
    postdiv = (postdiv==0) ? 1 : postdiv;
    postdiv = postdiv>PLL_SEC_PSTDIV1_MAX*PLL_SEC_PSTDIV2_MAX ? PLL_SEC_PSTDIV1_MAX*PLL_SEC_PSTDIV2_MAX : postdiv;
    bst_cal_pdiv1_pdiv2(postdiv, &pdiv1, &pdiv2);

    reg_config1 &= ~((PLL_SEC_PSTDIV_MASK<<PLL_SEC_PSTDIV1) | (PLL_SEC_PSTDIV_MASK<<PLL_SEC_PSTDIV2)); //clear pdiv1 and pdiv2
    reg_config1 |= ((pdiv1<<PLL_SEC_PSTDIV1) | (pdiv2<<PLL_SEC_PSTDIV2)); //re-write pdiv1 and pdiv2
    writel(reg_config1, clk->base_addr + PLL_REG_CONFIG1);

    /* switch from osc to pll output */
    for(i = 0; i < clk->pll_mux_cnt; i++) {
        reg = readl((clk->pll_mux)[i].mux_addr);
        mask = BIT((clk->pll_mux)[i].mux_width) - 1;
        reg &= ~(mask<<((clk->pll_mux)[i].mux_oft));
        reg |= ((clk->pll_mux)[i].mux_pll_val)<<((clk->pll_mux)[i].mux_oft);
        writel(reg, (clk->pll_mux)[i].mux_addr);
    }
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    return 0;
}      


/**
 * bst_pll_fpd_recalc_rate() - please refer bst_pll_recalc_rate()
 * fdp = fout/postdiv1/postdiv2
 * fdp only ajust postdiv1 or postdiv2 to set rate
 */
static unsigned long bst_sec_pll_fpd_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
    struct bst_pll *clk = to_bst_pll(hw);
    u64 fbdiv, frac, fout;
    u32 reg_config0, reg_config1;
    u32 pdiv1, pdiv2;
    unsigned long flags = 0;
    
    spin_lock_irqsave(clk->pll_spinlock, flags);
    
    reg_config0 = readl(clk->base_addr + PLL_REG_CONFIG0);
    reg_config1 = readl(clk->base_addr + PLL_REG_CONFIG1);
    
    spin_unlock_irqrestore(clk->pll_spinlock, flags);

    pdiv1 = (reg_config1>>PLL_SEC_PSTDIV1)&PLL_SEC_PSTDIV_MASK;
    pdiv2 = (reg_config1>>PLL_SEC_PSTDIV2)&PLL_SEC_PSTDIV_MASK;
    pdiv1 = pdiv1 ? pdiv1 : 1; //treat 0 as 1
    pdiv2 = pdiv2 ? pdiv2 : 1; //treat 0 as 1
    if(reg_config0 & (1<<PLL_SEC_DSMEN)) {
        /*
         * FOUTPOSTDIV = FREF*(FBDIV+FRAC/(2^24))/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        frac = (reg_config1>>PLL_SEC_FRAC)&PLL_SEC_FRAC_MASK; //FRAC
        fout = fraction_mode_fout_rate(fbdiv, frac, parent_rate);
    } else {
        /*
         * FOUTPOSTDIV = FREF*FBDIV/(REFDIV*POSTDIV1*POSTDIV2), where REFDIV = 1
         */
        fbdiv = (reg_config0>>PLL_SEC_FBDIV)&PLL_SEC_FBDIV_MASK; //FBDIV
        fbdiv = fbdiv ? fbdiv : 1; //treat 0 as 1
        fout = integer_mode_fout_rate(fbdiv, parent_rate);
    }

    return fout/(pdiv1*pdiv2);
}

/**
 * clk_register_bst_pll_fo() - Register PLL with the clock framework
 * @name	PLL name
 * @parent	Parent clock name
 * @base_addr	pll base register address
 * @pll_spinlock	Register lock
 * @pll_mux     muxs directly connect to pll
 * @pll_mux_cnt     total mux count in pll_mux
 * Returns handle to the registered clock.
 */
static struct clk *clk_register_bst_pll(const char *name, const char *parent,
		void __iomem *base_addr, spinlock_t *spinlock,
		bst_pll_mux *pll_mux, u32 pll_mux_cnt, u32 is_fdp)
{
    struct clk *clk;
    struct bst_pll *pll;
	const char *parent_arr[1] = {parent};
	struct clk_init_data initd = {
		.name = name,
		.parent_names = parent_arr,
		.ops = &bst_pll_fo_ops,
		.num_parents = 1,
		.flags = CLK_IGNORE_UNUSED
	};

    if(is_fdp == 1){
        initd.ops = &bst_pll_fpd_ops;
    }
    else if(is_fdp == 2){
        initd.ops = &bst_pll_only_fpd_ops;
    }


	pll = kmalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	/* Populate the struct */
	pll->hw.init = &initd;
	pll->base_addr = base_addr;
	pll->pll_spinlock = spinlock;
    pll->pll_mux = pll_mux;
    pll->pll_mux_cnt = pll_mux_cnt;

	clk = clk_register(NULL, &pll->hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(pll);
        return 0;
    }

	return clk;
}

/**
 * clk_register_bst_pll_gmac() - Register PLL with the clock framework, gmac register offset is special
 * @name    PLL name
 * @parent  Parent clock name
 * @base_addr   pll base register address
 * @pll_spinlock    Register lock
 * @pll_mux     muxs directly connect to pll
 * @pll_mux_cnt     total mux count in pll_mux
 * Returns handle to the registered clock.
 */
static struct clk *clk_register_bst_pll_gmac(const char *name, const char *parent,
        void __iomem *base_addr, spinlock_t *spinlock,
        bst_pll_mux *pll_mux, u32 pll_mux_cnt, u32 is_fdp)
{
    struct clk *clk;
    struct bst_pll *pll;
    const char *parent_arr[1] = {parent};
    struct clk_init_data initd = {
        .name = name,
        .parent_names = parent_arr,
        .ops = &bst_pll_gmac_fpd_ops,
        .num_parents = 1,
        .flags = 0
    };

    if(! is_fdp) {
        pr_err("%s,%d: only support ptd pll.\n", __func__, __LINE__);
        return 0;
    }
    
    pll = kmalloc(sizeof(*pll), GFP_KERNEL);
    if (!pll)
        return ERR_PTR(-ENOMEM);

    /* Populate the struct */
    pll->hw.init = &initd;
    pll->base_addr = base_addr;
    pll->pll_spinlock = spinlock;
    pll->pll_mux = pll_mux;
    pll->pll_mux_cnt = pll_mux_cnt;

    clk = clk_register(NULL, &pll->hw);
    if (WARN_ON(IS_ERR(clk))) {
        kfree(pll);
        return 0;
    }

    return clk;
}

/**
 * clk_register_bst_sec_pll() - Register PLL with the clock framework
 * @name    PLL name
 * @parent  Parent clock name
 * @base_addr   pll base register address
 * @pll_spinlock    Register lock
 * @pll_mux     muxs directly connect to pll
 * @pll_mux_cnt     total mux count in pll_mux
 * Returns handle to the registered clock.
 */
static struct clk *clk_register_bst_sec_pll(const char *name, const char *parent,
        void __iomem *base_addr, spinlock_t *spinlock,
        bst_pll_mux *pll_mux, u32 pll_mux_cnt, u32 is_fdp)
{
    struct clk *clk;
    struct bst_pll *pll;
    const char *parent_arr[1] = {parent};
    struct clk_init_data initd = {
        .name = name,
        .parent_names = parent_arr,
        .ops = &bst_sec_pll_ops,
        .num_parents = 1,
        .flags = 0
    };

    if(is_fdp) {
        initd.ops = &bst_sec_pll_fpd_ops;
    }
    
    pll = kmalloc(sizeof(*pll), GFP_KERNEL);
    if (!pll)
        return ERR_PTR(-ENOMEM);

    /* Populate the struct */
    pll->hw.init = &initd;
    pll->base_addr = base_addr;
    pll->pll_spinlock = spinlock;
    pll->pll_mux = pll_mux;
    pll->pll_mux_cnt = pll_mux_cnt;

    clk = clk_register(NULL, &pll->hw);
    if (WARN_ON(IS_ERR(clk))) {
        kfree(pll);
        return 0;
    }

    return clk;
}

/**
 * pll_mux_addr_init() - assign the pll mux address value
 */
static void __init pll_mux_addr_init(void)
{
    pll_cpu_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_dsu_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_sysbus_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_sysbus_ptd_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_disp_ptd_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_coreip_ptd_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_hsplsp_mux[0].mux_addr = TOP_CRM_REG_R_PLL_CLKMUX_SEL;
    pll_main_ptd_mux[0].mux_addr = SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL;
    pll_main_ptd_mux[1].mux_addr = SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL;
    pll_main_ptd_mux[2].mux_addr = SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL;
    pll_main_ptd_mux[3].mux_addr = SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL;
    pll_lsp_ptd_mux[0].mux_addr = SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL;
}

/**
 * osc_clk_register() - Register osc clock node
 * @np    node pointer
 */
static void __init main_clk_register(struct device_node *np)
{
    int ret;
    u32 main_clk;
    
    ret = of_property_read_u32(np, "osc-clk-frequency", &main_clk);
    if(ret) {
        pr_err("%s,%d: can't read osc clock value(Err:%d).\n", __func__, __LINE__, ret);
        return;
    }

    clks[MAIN_OSC_CLK] = clk_register_fixed_rate(NULL, MAIN_CLK, NULL, 0, main_clk);
}

/**
 * misc_fix_rate_clocks_register() - Register module used fix rate clocks, such as gmac/pcie external clocks
 * @np    node pointer
 */
static void __init misc_fix_rate_clocks_register(struct device_node *np)
{
    int ret;
    u32 clk_rate;

    ret = of_property_read_u32(np, "rgmii0-rxclk-frequency", &clk_rate);
    if(ret) {
        pr_err("%s,%d: can't read osc clock value(Err:%d).\n", __func__, __LINE__, ret);
        return;
    }
    clk_register_fixed_rate(NULL, RGMII0_RXCLK, NULL, 0, clk_rate);

    ret = of_property_read_u32(np, "rgmii1-rxclk-frequency", &clk_rate);
    if(ret) {
        pr_err("%s,%d: can't read osc clock value(Err:%d).\n", __func__, __LINE__, ret);
        return;
    }
    clk_register_fixed_rate(NULL, RGMII1_RXCLK, NULL, 0, clk_rate);

    ret = of_property_read_u32(np, "ptp_clk-frequency", &clk_rate);
    if(ret) {
        pr_err("%s,%d: can't read osc clock value(Err:%d).\n", __func__, __LINE__, ret);
        return;
    }
    clk_register_fixed_rate(NULL, PTP_CLK, NULL, 0, clk_rate);
}

/**
 * pll_cpu_clocks_register()
 * @np    node pointer
 */
static void __init pll_cpu_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_CPU, MAIN_CLK, PLL_CPU_BASE, &pll_cpu_spinlock, 
        (bst_pll_mux *)pll_cpu_mux, ARRAY_SIZE(pll_cpu_mux), 0);
    
    clk_register_mux(NULL, CLK_CPU_1400_SEL, cpu_1400_sel_parents, ARRAY_SIZE(cpu_1400_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 27, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_CPU_1400_SEL_FIX_FACTOR_2_1, CLK_CPU_1400_SEL, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_mux(NULL, CLK_1400_CPUCORE_SEL, clk_1400_cpucore_sel_parents, ARRAY_SIZE(clk_1400_cpucore_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 26, 2, 0, &clk_spinlock);
    clks[LB_CPU_CLK] = clk_register_gate(NULL, LB_CPU_CLK_GATE_EN, CLK_1400_CPUCORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 29, 0, &clk_spinlock);
}

/**
 * pll_dsu_clocks_register()
 * @np    node pointer
 */
static void __init pll_dsu_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_DSU, MAIN_CLK, PLL_DSU_BASE, &pll_dsu_spinlock, 
        (bst_pll_mux *)pll_dsu_mux, ARRAY_SIZE(pll_dsu_mux), 0);
    clk_register_mux(NULL, CLK_DSU_1300_SEL, dsu_1300_sel_parents, ARRAY_SIZE(dsu_1300_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 28, 1, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_DSU_SEL, clk_dsu_sel_parents, ARRAY_SIZE(clk_dsu_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 28, 1, 0, &clk_spinlock);
    clks[LB_CPU_DSU_CLK] = clk_register_gate(NULL, LB_CPU_DSU_CLK_GATE_EN, CLK_DSU_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 30, 0, &clk_spinlock);
}

/**
 * pll_hsp_lsp_clocks_register()
 * @np    node pointer
 */
static void __init pll_hsp_lsp_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_HSP_LSP, MAIN_CLK, PLL_HSP_LSP_BASE, &pll_hsplsp_spinlock, 
        (bst_pll_mux *)pll_hsplsp_mux, ARRAY_SIZE(pll_hsplsp_mux), 0);
    clk_register_mux(NULL, CLK_2000_SEL, clk_2000_sel_parents, ARRAY_SIZE(clk_2000_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 22, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_2000_SEL_FIX_FACTOR_2_1, CLK_2000_SEL, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_fixed_factor(NULL, CLK_2000_SEL_FIX_FACTOR_8_1, CLK_2000_SEL, CLK_SET_RATE_PARENT, 1, 8);
    clk_register_fixed_factor(NULL, CLK_2000_SEL_FIX_FACTOR_16_1, CLK_2000_SEL, CLK_SET_RATE_PARENT, 1, 16);
    clk_register_fixed_factor(NULL, CLK_2000_SEL_FIX_FACTOR_10_1, CLK_2000_SEL, CLK_SET_RATE_PARENT, 1, 10);
    clk_register_fixed_factor(NULL, CLK_2000_SEL_FIX_FACTOR_10_1_4_1, CLK_2000_SEL_FIX_FACTOR_10_1, CLK_SET_RATE_PARENT, 1, 4);

    /* can-fd wclk */
    clk_register_gate(NULL, LB_SEC_SAFE_CANFD_200M_CLK_GATE_EN, CLK_2000_SEL_FIX_FACTOR_10_1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 27, 0, &clk_spinlock);
}

/**
 * pll_hsp_lsp_ptd_clocks_register()
 * @np    node pointer
 */
static void __init pll_hsp_lsp_ptd_clocks_register(struct device_node *np)
{
    clks[LB_PCIE_REF_ALT_CLK_P] = clk_register_bst_pll(PLL_HSP_LSP_PTD, MAIN_CLK, PLL_HSP_LSP_BASE, &pll_hsplsp_spinlock, 
        (bst_pll_mux *)NULL, 0, 1);
}

/**
 * pll_sysbus_clocks_register()
 * @np    node pointer
 */
static void __init pll_sysbus_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_SYSBUS, MAIN_CLK, PLL_SYSBUS_BASE, &pll_sysbus_spinlock, 
        (bst_pll_mux *)pll_sysbus_mux, ARRAY_SIZE(pll_sysbus_mux), 0);
    
    clk_register_mux(NULL, CLK_2400_SYSBUS_SEL, clk_2400_sysbus_sel_parents, ARRAY_SIZE(clk_2400_sysbus_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 25, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_2400_SYSBUS_SEL, CLK_SET_RATE_PARENT, 1, 4);
}

/**
 * pll_sysbus_ptd_clocks_register()
 * @np    node pointer
 */
static void __init pll_sysbus_ptd_clocks_register(struct device_node *np)
{
    /* mini system */
    clk_register_bst_pll(PLL_SYSBUS_PTD, MAIN_CLK, PLL_SYSBUS_BASE, &pll_sysbus_spinlock, 
        (bst_pll_mux *)pll_sysbus_ptd_mux, ARRAY_SIZE(pll_sysbus_ptd_mux), 1);
    
    clk_register_mux(NULL, CLK_800_SYSBUS_SEL, clk_800_sysbus_sel_parents, ARRAY_SIZE(clk_800_sysbus_sel_parents),
		CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 26, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1, CLK_800_SYSBUS_SEL, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_fixed_factor(NULL, CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1, CLK_800_SYSBUS_SEL, CLK_SET_RATE_PARENT, 1, 4);
    clk_register_fixed_factor(NULL, CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1, CLK_800_SYSBUS_SEL, CLK_SET_RATE_PARENT, 1, 8);
    clk_register_fixed_factor(NULL, CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1, CLK_800_SYSBUS_SEL, CLK_SET_RATE_PARENT, 1, 16);
    
    clks[CLK_800_CORENOC_WORK] = clk_register_mux(NULL, CLK_800_CORENOC_WORK_SEL, clk_800_corenoc_work_sel_parents, 
        ARRAY_SIZE(clk_800_corenoc_work_sel_parents), CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 24, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_800_SYSBUS_AXI_SEL, clk_800_sysbus_axi_sel_parents, ARRAY_SIZE(clk_800_sysbus_axi_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 14, 2, 0, &clk_spinlock);
    clks[CLK_800_CPUNOC_WORK] = clk_register_mux(NULL, CLK_800_CPUNOC_WORK_SEL, clk_800_cpunoc_work_sel_parents, 
        ARRAY_SIZE(clk_800_cpunoc_work_sel_parents), CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 22, 2, 0, &clk_spinlock);
    clks[CLK_400_SYSNOC_WORK] = clk_register_mux(NULL, CLK_400_SYSNOC_WORK_SEL, clk_400_sysnoc_work_sel_parents, 
        ARRAY_SIZE(clk_400_sysnoc_work_sel_parents), CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 24, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_800_GDMA_AXI_SEL, clk_800_gdma_axi_sel_parents, ARRAY_SIZE(clk_800_gdma_axi_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 29, 2, 0, &clk_spinlock);
    clks[LB_MATRIX_SYSNOC_APB_S_PCLK] = clk_register_mux(NULL, CLK_100_SYSBUS_APB_SEL, clk_100_sysbus_apb_sel_parents, 
        ARRAY_SIZE(clk_100_sysbus_apb_sel_parents), CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 26, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_200_SYSBUS_AHB_SEL, clk_200_sysbus_ahb_sel_parents, ARRAY_SIZE(clk_200_sysbus_ahb_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 10, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_200_SYSBUS_APB_SEL, clk_200_sysbus_apb_sel_parents, ARRAY_SIZE(clk_200_sysbus_apb_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 8, 2, 0, &clk_spinlock);
    
    clks[LB_LPDDR0_S0_ACLK] = clk_register_gate(NULL, LB_LPDDR0_S0_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 28, 0, &clk_spinlock);
    clks[LB_LPDDR0_S1_ACLK] = clk_register_gate(NULL, LB_LPDDR0_S1_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 27, 0, &clk_spinlock);
    clks[LB_LPDDR0_S2_ACLK] = clk_register_gate(NULL, LB_LPDDR0_S2_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 26, 0, &clk_spinlock);
    clks[LB_LPDDR0_S3_ACLK] = clk_register_gate(NULL, LB_LPDDR0_S3_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
		TOP_CRM_REG_R_CLKGATE_EN0, 25, 0, &clk_spinlock);

    clks[LB_LPDDR1_S0_ACLK] = clk_register_gate(NULL, LB_LPDDR1_S0_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 24, 0, &clk_spinlock);
    clks[LB_LPDDR1_S1_ACLK] = clk_register_gate(NULL, LB_LPDDR1_S1_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 23, 0, &clk_spinlock);
    clks[LB_LPDDR1_S2_ACLK] = clk_register_gate(NULL, LB_LPDDR1_S2_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 22, 0, &clk_spinlock);
    clks[LB_LPDDR1_S3_ACLK] = clk_register_gate(NULL, LB_LPDDR1_S3_ACLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 21, 0, &clk_spinlock);

    clks[LB_LPDDR0_PCLK] = clk_register_gate(NULL, LB_LPDDR0_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 20, 0, &clk_spinlock);
    clks[LB_LPDDR1_PCLK] = clk_register_gate(NULL, LB_LPDDR1_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 19, 0, &clk_spinlock);

    /* PCIE */
    clk_register_mux(NULL, CLK_400_SYSBUS_AXI_SEL, clk_400_sysbus_axi_sel_parents, ARRAY_SIZE(clk_400_sysbus_axi_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 12, 2, 0, &clk_spinlock);
    clks[LB_PCIE_AXI_ACLK] = clk_register_gate(NULL, LB_PCIE_AXI_ACLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 6, 0, &clk_spinlock);
    clks[LB_PCIE_AUX_CLK] = clk_register_gate(NULL, LB_PCIE_AUX_CLK_GATE_EN, MAIN_CLK, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 5, 0, &clk_spinlock);
    clks[LB_PCIE_APB_PCLK] = clk_register_gate(NULL, LB_PCIE_APB_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 4, 0, &clk_spinlock);

    /* USB20 */
    clks[LB_USB2_AHB_HCLK] = clk_register_gate(NULL, LB_USB2_AHB_HCLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 8, 0, &clk_spinlock);
    clks[LB_USB2_REF_ALT_CLK] = clk_register_gate(NULL, LB_USB2_REF_ALT_CLK_GATE_EN, MAIN_CLK, CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 28, 0, &clk_spinlock);
    clks[LB_USB2_APB_PCLK] = clk_register_gate(NULL, LB_USB2_APB_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 7, 0, &clk_spinlock);

    /* USB30 */
    clks[LB_USB3_SUSPEND_CLK] = clks[MAIN_OSC_CLK];
    clks[LB_USB3_REF_ALT_CLK] = clk_register_gate(NULL, LB_USB3_REF_ALT_CLK_GATE_EN, MAIN_CLK, CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 29, 0, &clk_spinlock);
    clks[LB_USB3_AXI_ACLK] = clk_register_gate(NULL, LB_USB3_AXI_ACLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 10, 0, &clk_spinlock);
    clks[LB_USB3_APB_PCLK] = clk_register_gate(NULL, LB_USB3_APB_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 9, 0, &clk_spinlock);

    /* SDEMMC */
    clk_register_mux(NULL, CLK_200_SDEMMC0_SEL, clk_200_sdemmc0_sel_parents, ARRAY_SIZE(clk_200_sdemmc0_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 14, 1, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_200_SDEMMC1_SEL, clk_200_sdemmc1_sel_parents, ARRAY_SIZE(clk_200_sdemmc1_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 15, 1, 0, &clk_spinlock);
    clks[LB_SDEMMC0_W_BCLK] = clk_register_gate(NULL, LB_SDEMMC0_W_BCLK_GATE_EN, CLK_200_SDEMMC0_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 3, 0, &clk_spinlock);
    clks[LB_SDEMMC0_HCLK] = clk_register_gate(NULL, LB_SDEMMC0_HCLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 2, 0, &clk_spinlock);
    clks[LB_SDEMMC1_W_BCLK] = clk_register_gate(NULL, LB_SDEMMC1_W_BCLK_GATE_EN, CLK_200_SDEMMC1_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 1, 0, &clk_spinlock);
    clks[LB_SDEMMC1_HCLK] = clk_register_gate(NULL, LB_SDEMMC1_HCLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 0, 0, &clk_spinlock);

    /* GDMA */    
    clk_register_mux(NULL, CLK_400_GDMA_CORE_SEL, clk_400_gdma_core_sel_parents, ARRAY_SIZE(clk_400_gdma_core_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 16, 2, 0, &clk_spinlock);
    clks[LB_GDMA_CORE_CLK] = clk_register_gate(NULL, LB_GDMA_CORE_CLK_GATE_EN, CLK_400_GDMA_CORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 3, 0, &clk_spinlock);
    clks[LB_GDMA_AXI_ACLK] = clk_register_gate(NULL, LB_GDMA_AXI_ACLK_GATE_EN, CLK_800_GDMA_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 11, 0, &clk_spinlock);
    clks[LB_GDMA_AHB_CLK] = clk_register_gate(NULL, LB_GDMA_AHB_CLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 10, 0, &clk_spinlock);

    /* ISP */
    clks[LB_ISP_SCLK] = clk_register_gate(NULL, LB_ISP_SCLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 18, 0, &clk_spinlock);
    clks[LB_ISP_AHB_HCLK] = clk_register_gate(NULL, LB_ISP_AHB_HCLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 17, 0, &clk_spinlock);

    /* MIPI */
    clks[LB_MIPI0_PHY_CFG_CLK] = clks[MAIN_OSC_CLK];
    clks[LB_MIPI1_PHY_CFG_CLK] = clks[MAIN_OSC_CLK];
    clks[LB_MIPI2_PHY_CFG_CLK] = clks[MAIN_OSC_CLK];
    clks[LB_MIPI0_APB_CFG_CLK] = clk_register_gate(NULL, LB_MIPI0_APB_CFG_CLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 24, 0, &clk_spinlock);
    clks[LB_MIPI1_APB_CFG_CLK] = clk_register_gate(NULL, LB_MIPI1_APB_CFG_CLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 23, 0, &clk_spinlock);
    clks[LB_MIPI2_APB_CFG_CLK] = clk_register_gate(NULL, LB_MIPI2_APB_CFG_CLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 22, 0, &clk_spinlock);

    /* GPU */
    clk_register_mux(NULL, CLK_800_GPU_SEL, clk_800_gpu_sel_parents, ARRAY_SIZE(clk_800_gpu_sel_parents), 
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 18, 2, 0, &clk_spinlock);
    clks[LB_GPU_AXI_ACLK] = clk_register_gate(NULL, LB_GPU_AXI_ACLK_RSTSYNC_I_GATE_EN, CLK_800_GPU_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 12, 0, &clk_spinlock);
    clks[LB_GPU_APB_S_PCLK] = clk_register_gate(NULL, LB_GPU_APB_S_PCLK_EN, CLK_200_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 4, 0, &clk_spinlock);

    /* IPC */
    clks[IPC_APB_S_PCLK] = clk_register_gate(NULL, LB_MATRIX_IPC_APB_S_PCLK_GATE_EN, CLK_200_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 7, 0, &clk_spinlock);

    /* TEMP SENSOR */
    clks[TEMPSENSOR_1M25_CLK] = clk_register_gate(NULL, LB_MATRIX_TEMPSENSOR_1M25_CLK_GATE_EN, MAIN_CLK_FIX_FACTOR_20_1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 9, 0, &clk_spinlock);
    clks[TEMP_25M_CLK] = clk_register_gate(NULL, LB_TEMP_25M_CLK_GATE_EN, MAIN_CLK, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 6, 0, &clk_spinlock);

}

/**
 * pll_main_ptd_clocks_register()
 * @np    node pointer
 */
static void __init pll_main_ptd_clocks_register(struct device_node *np)
{
    clk_register_bst_sec_pll(PLL_MAIN_PTD, MAIN_CLK, PLL_SAFE_MAIN, &pll_main_spinlock, 
        (bst_pll_mux *)pll_main_ptd_mux, ARRAY_SIZE(pll_main_ptd_mux), 1);

    clk_register_fixed_factor(NULL, PLL_MAIN_PTD_FIX_FACTOR_2_1, PLL_MAIN_PTD, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_fixed_factor(NULL, PLL_MAIN_PTD_FIX_FACTOR_4_1, PLL_MAIN_PTD, CLK_SET_RATE_PARENT, 1, 4);
    clk_register_fixed_factor(NULL, PLL_MAIN_PTD_FIX_FACTOR_8_1, PLL_MAIN_PTD, CLK_SET_RATE_PARENT, 1, 8);
    clk_register_mux(NULL, SEC_SAFE_CLK_SEL0, sec_safe_clk_sel0_parents, ARRAY_SIZE(sec_safe_clk_sel0_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 0, 1, 0, &clk_spinlock);
    clk_register_mux(NULL, SEC_SAFE_CLK_SEL1, sec_safe_clk_sel1_parents, ARRAY_SIZE(sec_safe_clk_sel1_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 1, 1, 0, &clk_spinlock);
    clks[SAFE_NOC_ACLK] = clk_register_mux(NULL, SEC_SAFE_CLK_SEL2, sec_safe_clk_sel2_parents, ARRAY_SIZE(sec_safe_clk_sel2_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 2, 1, 0, &clk_spinlock);
    clks[SAFE_NOC_HCLK] = clk_register_mux(NULL, SEC_SAFE_CLK_SEL3, sec_safe_clk_sel3_parents, ARRAY_SIZE(sec_safe_clk_sel3_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 3, 1, 0, &clk_spinlock);
    clks[SAFE_NOC_PCLK] = clk_register_fixed_factor(NULL, SEC_SAFE_CLK_SEL3_FIX_FACTOR_2_1, SEC_SAFE_CLK_SEL3, CLK_SET_RATE_PARENT, 1, 2);
    clks[GIC_WCLK] = clks[SAFE_NOC_ACLK];

    // double check the divider value flags
    clk_register_divider(NULL, CLK_QSPI_REF_DIV, SEC_SAFE_CLK_SEL0, 0/*CLK_SET_RATE_PARENT*/, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 12, 4,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clk_register_mux(NULL, SEC_SAFE_CLK_SEL_9_8, sec_safe_clk_sel9_8_parents, ARRAY_SIZE(sec_safe_clk_sel9_8_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 8, 2, 0, &clk_spinlock);

    clks[QSPI0_HCLK] = clk_register_gate(NULL, QSPI0_HCLK_EN, SEC_SAFE_CLK_SEL3, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 9, 0, &clk_spinlock);
    clks[QSPI0_WCLK] = clk_register_gate(NULL, QSPI0_WCLK_EN, SEC_SAFE_CLK_SEL_9_8, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 10, 0, &clk_spinlock);
    clks[QSPI1_HCLK] = clk_register_gate(NULL, QSPI1_HCLK_EN, SEC_SAFE_CLK_SEL3, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 11, 0, &clk_spinlock);
    clks[QSPI1_WCLK] = clk_register_gate(NULL, QSPI1_WCLK_EN, SEC_SAFE_CLK_SEL_9_8, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 12, 0, &clk_spinlock);

    clk_register_gate(NULL, GIC_WCLK_EN, SEC_SAFE_CLK_SEL2, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 2, 0, &clk_spinlock);
    
    clks[LSP0_PCLK] = clk_register_gate(NULL, LSP0_PCLK_EN, SEC_SAFE_CLK_SEL3, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 13, 0, &clk_spinlock);
    clks[LSP0_WCLK] = clk_register_gate(NULL, LSP0_WCLK_EN, SEC_SAFE_CLK_SEL2, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 14, 0, &clk_spinlock);
    clks[UART_WCLK] = clk_register_gate(NULL, UART_WCLK_EN, MAIN_CLK, CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 20, 0, &clk_spinlock);
    clks[LSP1_PCLK] = clk_register_gate(NULL, LSP1_PCLK_EN, SEC_SAFE_CLK_SEL3, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 16, 0, &clk_spinlock);
    clks[LSP1_WCLK] = clk_register_gate(NULL, LSP1_WCLK_EN, SEC_SAFE_CLK_SEL2, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 17, 0, &clk_spinlock);

    /* can-fd */
    clks[CAN_FD0_PCLK] = clk_register_gate(NULL, CAN_FD0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 12, 0, &clk_spinlock);
    clks[CAN_FD1_PCLK] = clk_register_gate(NULL, CAN_FD1_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 8, 0, &clk_spinlock);
    clk_register_mux(NULL, SEC_SAFE_CLK_SEL_7, sec_safe_clk_sel7_parents, ARRAY_SIZE(sec_safe_clk_sel7_parents),
		CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 7, 1, 0, &clk_spinlock);
    clk_register_gate(NULL, CAN_FD0_WCLK_EN, SEC_SAFE_CLK_SEL_7, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 21, 0, &clk_spinlock);
    clk_register_gate(NULL, CAN_FD1_WCLK_EN, SEC_SAFE_CLK_SEL_7, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 22, 0, &clk_spinlock);
    clks[CAN_FD0_WCLK] = clk_register_gate(NULL, CAN_FD0_WCLK_GATE_EN, CAN_FD0_WCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 12, 0, &clk_spinlock);
    clks[CAN_FD1_WCLK] = clk_register_gate(NULL, CAN_FD1_WCLK_GATE_EN, CAN_FD1_WCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 8, 0, &clk_spinlock);

    /* uart */
    clk_register_divider(NULL, LSP0_UART0_DIVIDER, UART_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG, 7, 6,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO | CLK_DIVIDER_READ_ONLY, &clk_spinlock);
    clks[LSP0_UART0_WCLK] = clk_register_gate(NULL, LSP0_UART0_WCLK_GATE_EN, LSP0_UART0_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
            LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 10, 0, &clk_spinlock);
    clks[LSP0_UART0_PCLK] = clk_register_gate(NULL, LSP0_UART0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
            LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 10, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP0_UART1_DIVIDER, UART_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG, 0, 6,
        CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO | CLK_DIVIDER_READ_ONLY, &clk_spinlock);
    clks[LSP0_UART1_WCLK] = clk_register_gate(NULL, LSP0_UART1_WCLK_GATE_EN, LSP0_UART1_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 31, 0, &clk_spinlock);
    clks[LSP0_UART1_PCLK] = clk_register_gate(NULL, LSP0_UART1_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 31, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP1_UART0_DIVIDER, UART_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL1_REG, 18, 6,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO | CLK_DIVIDER_READ_ONLY, &clk_spinlock);
    clks[LSP1_UART0_WCLK] = clk_register_gate(NULL, LSP1_UART0_WCLK_GATE_EN, LSP1_UART0_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 27, 0, &clk_spinlock);
    clks[LSP1_UART0_PCLK] = clk_register_gate(NULL, LSP1_UART0_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 27, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP1_UART1_DIVIDER, UART_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL_REG, 0, 6,
        CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO | CLK_DIVIDER_READ_ONLY, &clk_spinlock);
    clks[LSP1_UART1_WCLK] = clk_register_gate(NULL, LSP1_UART1_WCLK_GATE_EN, LSP1_UART1_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 7, 0, &clk_spinlock);
    clks[LSP1_UART1_PCLK] = clk_register_gate(NULL, LSP1_UART1_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 7, 0, &clk_spinlock);

    /* I2C */
    clk_register_fixed_factor(NULL, LSP0_I2C_WCLK_FIX_FACTOR_2_1, LSP0_WCLK_EN, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_mux(NULL, LSP0_I2C_MUX, lsp0_i2c_mux_parents, ARRAY_SIZE(lsp0_i2c_mux_parents),
        CLK_SET_RATE_PARENT, LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 14, 1, 0, &clk_spinlock);
    clks[LSP0_I2C0_WCLK] = clk_register_gate(NULL, LSP0_I2C0_WCLK_GATE_EN, LSP0_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 0, 0, &clk_spinlock);
    clks[LSP0_I2C0_PCLK] = clk_register_gate(NULL, LSP0_I2C0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 0, 0, &clk_spinlock);
    clks[LSP0_I2C1_WCLK] = clk_register_gate(NULL, LSP0_I2C1_WCLK_GATE_EN, LSP0_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 1, 0, &clk_spinlock);
    clks[LSP0_I2C1_PCLK] = clk_register_gate(NULL, LSP0_I2C1_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 1, 0, &clk_spinlock);
    clks[LSP0_I2C2_WCLK] = clk_register_gate(NULL, LSP0_I2C2_WCLK_GATE_EN, LSP0_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 2, 0, &clk_spinlock);
    clks[LSP0_I2C2_PCLK] = clk_register_gate(NULL, LSP0_I2C2_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 2, 0, &clk_spinlock);

    clk_register_fixed_factor(NULL, LSP1_I2C_WCLK_FIX_FACTOR_2_1, LSP1_WCLK_EN, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_mux(NULL, LSP1_I2C_MUX, lsp1_i2c_mux_parents, ARRAY_SIZE(lsp1_i2c_mux_parents),
        CLK_SET_RATE_PARENT, LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 22, 1, 0, &clk_spinlock);
    clks[LSP1_I2C3_WCLK] = clk_register_gate(NULL, LSP1_I2C3_WCLK_GATE_EN, LSP1_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 0, 0, &clk_spinlock);
    clks[LSP1_I2C3_PCLK] = clk_register_gate(NULL, LSP1_I2C3_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 0, 0, &clk_spinlock);
    clks[LSP1_I2C4_WCLK] = clk_register_gate(NULL, LSP1_I2C4_WCLK_GATE_EN, LSP1_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 1, 0, &clk_spinlock);
    clks[LSP1_I2C4_PCLK] = clk_register_gate(NULL, LSP1_I2C4_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 1, 0, &clk_spinlock);
    clks[LSP1_I2C5_WCLK] = clk_register_gate(NULL, LSP1_I2C5_WCLK_GATE_EN, LSP1_I2C_MUX, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 26, 0, &clk_spinlock);
    clks[LSP1_I2C5_PCLK] = clk_register_gate(NULL, LSP1_I2C5_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 26, 0, &clk_spinlock);

    /* TIMER */
    clk_register_divider(NULL, LSP0_TIMER_DIVIDER, LSP0_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL_REG, 0, 13,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clks[LSP0_TIMER0_WCLK] = clk_register_gate(NULL, LSP0_TIMER0_WCLK_GATE_EN, LSP0_TIMER_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 3, 0, &clk_spinlock);
    clks[LSP0_TIMER0_PCLK] = clk_register_gate(NULL, LSP0_TIMER0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 5, 0, &clk_spinlock);
    clks[LSP0_TIMER1_WCLK] = clk_register_gate(NULL, LSP0_TIMER1_WCLK_GATE_EN, LSP0_TIMER_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 4, 0, &clk_spinlock);
    clks[LSP0_TIMER1_PCLK] = clk_register_gate(NULL, LSP0_TIMER1_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 5, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP1_TIMER_DIVIDER, LSP1_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL_REG, 7, 13,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clks[LSP1_TIMER2_WCLK] = clk_register_gate(NULL, LSP1_TIMER2_WCLK_GATE_EN, LSP1_TIMER_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 2, 0, &clk_spinlock);
    clks[LSP1_TIMER2_PCLK] = clk_register_gate(NULL, LSP1_TIMER2_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 4, 0, &clk_spinlock);
    clks[LSP1_TIMER3_WCLK] = clk_register_gate(NULL, LSP1_TIMER3_WCLK_GATE_EN, LSP1_TIMER_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 3, 0, &clk_spinlock);
    clks[LSP1_TIMER3_PCLK] = clk_register_gate(NULL, LSP1_TIMER3_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 4, 0, &clk_spinlock);

    /* SPI */
    clk_register_divider(NULL, LSP0_SPI0_DIVIDER, LSP0_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL_REG, 14, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock); //sync should special handle, not support modify div value now!!
    clks[LSP0_SPI0_WCLK] = clk_register_gate(NULL, LSP0_SPI0_WCLK_GATE_EN, LSP0_SPI0_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 11, 0, &clk_spinlock);
    clks[LSP0_SPI0_PCLK] = clk_register_gate(NULL, LSP0_SPI0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 11, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP1_SPI1_DIVIDER, LSP1_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL_REG, 21, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock); //sync should special handle, not support modify div value now!!
    clks[LSP1_SPI1_WCLK] = clk_register_gate(NULL, LSP1_SPI1_WCLK_GATE_EN, LSP1_SPI1_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 9, 0, &clk_spinlock);
    clks[LSP1_SPI1_PCLK] = clk_register_gate(NULL, LSP0_SPI0_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 9, 0, &clk_spinlock);

    /* WDT */
    clk_register_divider(NULL, LSP0_WDT0_DIVIDER, LSP0_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG, 14, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clk_register_divider(NULL, LSP0_WDT1_DIVIDER, LSP0_WCLK_EN, 0, LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG, 23, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clks[LSP0_WDT0_WCLK] = clk_register_gate(NULL, LSP0_WDT0_WCLK_GATE_EN, LSP0_WDT0_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 8, 0, &clk_spinlock);
    clks[LSP0_WDT0_PCLK] = clk_register_gate(NULL, LSP0_WDT0_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 8, 0, &clk_spinlock);
    clks[LSP0_WDT1_WCLK] = clk_register_gate(NULL, LSP0_WDT1_WCLK_GATE_EN, LSP0_WDT1_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 9, 0, &clk_spinlock);
    clks[LSP0_WDT1_PCLK] = clk_register_gate(NULL, LSP0_WDT1_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 9, 0, &clk_spinlock);

    clk_register_divider(NULL, LSP1_WDT2_DIVIDER, LSP1_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL1_REG, 0, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clk_register_divider(NULL, LSP1_WDT3_DIVIDER, LSP1_WCLK_EN, 0, LB_LSP1_TOP_R_LSP1_DIV_CTRL1_REG, 9, 8,
		CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_spinlock);
    clks[LSP1_WDT2_WCLK] = clk_register_gate(NULL, LSP1_WDT2_WCLK_GATE_EN, LSP1_WDT2_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 5, 0, &clk_spinlock);
    clks[LSP1_WDT2_PCLK] = clk_register_gate(NULL, LSP1_WDT2_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 5, 0, &clk_spinlock);
    clks[LSP1_WDT3_WCLK] = clk_register_gate(NULL, LSP1_WDT3_WCLK_GATE_EN, LSP1_WDT3_DIVIDER, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 6, 0, &clk_spinlock);
    clks[LSP1_WDT3_PCLK] = clk_register_gate(NULL, LSP1_WDT3_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 6, 0, &clk_spinlock);

    clks[LSP0_I2SM_PCLK] = clk_register_gate(NULL, LSP0_I2SM_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 6, 0, &clk_spinlock);

    clks[LSP1_I2SS_SCK] = clk_register_gate(NULL, LSP1_I2SS_SCK_GATE_EN, NULL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 10, 0, &clk_spinlock);
    clks[LSP1_I2SS_PCLK] = clk_register_gate(NULL, LSP1_I2SS_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 10, 0, &clk_spinlock);

    /* GPIO */
    clks[LSP0_GPIO_PCLK] = clk_register_gate(NULL, LSP0_GPIO_PCLK_GATE_EN, LSP0_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG, 7, 0, &clk_spinlock);
    clks[LSP1_GPIO_PCLK] = clk_register_gate(NULL, LSP1_GPIO_PCLK_GATE_EN, LSP1_PCLK_EN, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG, 11, 0, &clk_spinlock);
}

/**
 * pll_lsp_ptd_clocks_register()
 * @np    node pointer
 */
static void __init  pll_lsp_ptd_clocks_register(struct device_node *np)
{
    clk_register_bst_sec_pll(PLL_LSP_PTD, MAIN_CLK, PLL_SAFE_LSP, &pll_lsp_spinlock, 
        (bst_pll_mux *)pll_lsp_ptd_mux, ARRAY_SIZE(pll_lsp_ptd_mux), 1);

    clk_register_mux(NULL, SEC_SAFE_CLK_SEL4, sec_safe_clk_sel4_parents, ARRAY_SIZE(sec_safe_clk_sel4_parents),
        CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 4, 1, 0, &clk_spinlock);
    clks[LSP0_I2SM_CLK] = clk_register_gate(NULL, LSP0_I2SM_CLK_EN, SEC_SAFE_CLK_SEL4, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 15, 0, &clk_spinlock);
}

/**
 * pll_lsp_clocks_register()
 * @np    node pointer
 */
static void __init pll_lsp_clocks_register(struct device_node *np)
{
    u32 ptd_fix_factor= 1, val;
    
    clk_register_bst_sec_pll(PLL_LSP, MAIN_CLK, PLL_SAFE_LSP, &pll_lsp_spinlock, 
        (bst_pll_mux *)pll_lsp_ptd_mux, ARRAY_SIZE(pll_lsp_ptd_mux), 0); //just use ptd mux

    val = readl(PLL_SAFE_LSP+PLL_REG_CONFIG1);
    ptd_fix_factor = (val>>PLL_SEC_PSTDIV1)&PLL_SEC_PSTDIV_MASK; //postdiv1 can be 0!
    ptd_fix_factor = (ptd_fix_factor==0) ? 1: ptd_fix_factor;
    
    val = (val>>PLL_SEC_PSTDIV2)&PLL_SEC_PSTDIV_MASK; //postdiv2 can be 0!
    val = (val==0) ? 1: val;
    ptd_fix_factor = val*ptd_fix_factor;

    clk_register_fixed_factor(NULL, PLL_LSP_PTD_FIX_FACTOR, PLL_LSP, CLK_SET_RATE_PARENT, 1, ptd_fix_factor);
    
    clk_register_mux(NULL, SEC_SAFE_CLK_SEL4, sec_safe_clk_sel4_parents, ARRAY_SIZE(sec_safe_clk_sel4_parents),
        CLK_SET_RATE_PARENT, SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL, 4, 1, 0, &clk_spinlock);
    clks[LSP0_I2SM_CLK] = clk_register_gate(NULL, LSP0_I2SM_CLK_EN, SEC_SAFE_CLK_SEL4, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN, 15, 0, &clk_spinlock);
}

/**
 * pll_gmac_ptd_clocks_register()
 * @np    node pointer
 */
static void __init pll_gmac_ptd_clocks_register(struct device_node *np)
{
    clk_register_bst_pll_gmac(PLL_GMAC_PTD, PLL_GMAC_REF_CLK_SEL, PLL_GMAC_BASE, &pll_gmac_spinlock, 
        (bst_pll_mux *)NULL, 0, 1);

    clk_register_fixed_factor(NULL, PLL_GMAC_PTD_FIX_FACTOR_4_1, PLL_GMAC_PTD, CLK_SET_RATE_PARENT, 1, 4);
    clk_register_mux(NULL, PLL_GMAC_REF_25M_125M_SEL, pll_gmac_ref_25m_125m_sel_parents, ARRAY_SIZE(pll_gmac_ref_25m_125m_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 21, 1, 0, &clk_spinlock);
    clk_register_mux(NULL, PLL_GMAC_REF_CLK_SEL, pll_gmac_ref_clk_sel_parents, ARRAY_SIZE(pll_gmac_ref_clk_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 19, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_125_PTP_SEL, clk_125_ptp_sel_parents, ARRAY_SIZE(clk_125_ptp_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL0, 20, 2, 0, &clk_spinlock);
    
    clks[LB_GMAC0_WCLK] = clk_register_gate(NULL, LB_GMAC0_WCLK_GATE_EN, CLK_2000_SEL_FIX_FACTOR_16_1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 18, 0, &clk_spinlock);
    clks[LB_GMAC1_WLCK] = clk_register_gate(NULL, LB_GMAC1_WCLK_GATE_EN, CLK_2000_SEL_FIX_FACTOR_16_1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 17, 0, &clk_spinlock);
    clks[LB_GMAC0_AXIM_ACLK] = clk_register_gate(NULL, LB_GMAC0_AXIM_ACLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 16, 0, &clk_spinlock);
    clks[LB_GMAC1_AXIM_ACLK] = clk_register_gate(NULL, LB_GMAC1_AXIM_ACLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 15, 0, &clk_spinlock);
    
    clks[LB_GMAC0_APB_S_PCLK] = clk_register_gate(NULL, LB_GMAC0_APB_S_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 14, 0, &clk_spinlock);
    clks[LB_GMAC1_APB_S_PCLK] = clk_register_gate(NULL, LB_GMAC1_APB_S_PCLK_GATE_EN, CLK_100_SYSBUS_APB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 13, 0, &clk_spinlock);
    clks[LB_GMAC0_PTP_CLK] = clk_register_gate(NULL, LB_GMAC0_PTP_CLK_GATE_EN, CLK_125_PTP_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 12, 0, &clk_spinlock);
    clks[LB_GMAC1_PTP_CLK] = clk_register_gate(NULL, LB_GMAC1_PTP_CLK_GATE_EN, CLK_125_PTP_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN0, 11, 0, &clk_spinlock);
}

/**
 * pll_coreip_ptd_clocks_register()
 * @np    node pointer
 */
static void __init  pll_coreip_ptd_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_COREIP_PTD, MAIN_CLK, PLL_COREIP_BASE, &pll_coreip_spinlock, 
        (bst_pll_mux *)pll_coreip_ptd_mux, ARRAY_SIZE(pll_coreip_ptd_mux), 1);

    clk_register_mux(NULL, CLK_800_COREIP_SEL, clk_800_coreip_sel_parents, ARRAY_SIZE(clk_800_coreip_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 23, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_800_COREIP_SEL_FIX_FACTOR_2_1, PLL_COREIP_PTD, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_fixed_factor(NULL, CLK_800_COREIP_SEL_FIX_FACTOR_4_1, PLL_COREIP_PTD, CLK_SET_RATE_PARENT, 1, 4);
    clk_register_fixed_factor(NULL, MAIN_CLK_FIX_FACTOR_20_1, MAIN_CLK, CLK_SET_RATE_PARENT, 1, 20);

    /* CV */
    clk_register_mux(NULL, CLK_800_CV_CORE_SEL, clk_800_cv_core_sel_parents, ARRAY_SIZE(clk_800_cv_core_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 16, 2, 0, &clk_spinlock);
    clks[LB_CV_CORE_CLK] = clk_register_gate(NULL, LB_CV_CORE_CLK_GATE_EN, CLK_800_CV_CORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 16, 0, &clk_spinlock);
    clks[LB_CV_AXIM0_CLK] = clk_register_gate(NULL, LB_CV_AXIM0_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 15, 0, &clk_spinlock);
    clks[LB_CV_AXIM1_CLK] = clk_register_gate(NULL, LB_CV_AXIM1_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 14, 0, &clk_spinlock);
    clks[LB_CV_AXIS_CLK] = clk_register_gate(NULL, LB_CV_AXIS_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 13, 0, &clk_spinlock);

    /* NET */
    clk_register_mux(NULL, CLK_800_NET_SEL, clk_800_net_sel_parents, ARRAY_SIZE(clk_800_net_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 18, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_800_DSPCORE_SEL, clk_800_dspcore_sel_parents, ARRAY_SIZE(clk_800_dspcore_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 20, 2, 0, &clk_spinlock);
    clks[LB_NET_CLK] = clk_register_gate(NULL, LB_NET_CLK_I_GATE_EN, CLK_800_NET_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 25, 0, &clk_spinlock);
    clks[LB_NET_DSPCORE_CLK] = clk_register_gate(NULL, LB_NET_DSPCORE_CLK_I_GATE_EN, CLK_800_DSPCORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 26, 0, &clk_spinlock);
}

/**
 * pll_coreip_clocks_register()
 * @np    node pointer
 */
static void __init pll_coreip_clocks_register(struct device_node *np)
{
    u32 ptd_fix_factor= 1, val;
    
    clk_register_bst_pll(PLL_COREIP, MAIN_CLK, PLL_COREIP_BASE, &pll_coreip_spinlock, 
        (bst_pll_mux *)pll_coreip_ptd_mux, ARRAY_SIZE(pll_coreip_ptd_mux), 0);

    val = readl(PLL_COREIP_BASE+PLL_REG_CONFIG0);
    ptd_fix_factor = (val>>PLL_PSTDIV1)&PLL_PSTDIV_MASK; //postdiv1 can be 0!
    ptd_fix_factor = (ptd_fix_factor==0) ? 1: ptd_fix_factor;

    val = (val>>PLL_PSTDIV2)&PLL_PSTDIV_MASK; //postdiv2 can be 0!
    val = (val==0) ? 1: val;
    ptd_fix_factor = val*ptd_fix_factor;
    
    clk_register_fixed_factor(NULL, PLL_COREIP_PTD_FIX_FACTOR, PLL_COREIP, CLK_SET_RATE_PARENT, 1, ptd_fix_factor);
    clk_register_mux(NULL, CLK_800_COREIP_SEL, clk_800_coreip_sel_parents, ARRAY_SIZE(clk_800_coreip_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 23, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_800_COREIP_SEL_FIX_FACTOR_2_1, PLL_COREIP_PTD_FIX_FACTOR, CLK_SET_RATE_PARENT, 1, 2);
    clk_register_fixed_factor(NULL, CLK_800_COREIP_SEL_FIX_FACTOR_4_1, PLL_COREIP_PTD_FIX_FACTOR, CLK_SET_RATE_PARENT, 1, 4);
    clk_register_fixed_factor(NULL, MAIN_CLK_FIX_FACTOR_20_1, MAIN_CLK, CLK_SET_RATE_PARENT, 1, 20);

    /* CV */
    clk_register_mux(NULL, CLK_800_CV_CORE_SEL, clk_800_cv_core_sel_parents, ARRAY_SIZE(clk_800_cv_core_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 16, 2, 0, &clk_spinlock);
    clks[LB_CV_CORE_CLK] = clk_register_gate(NULL, LB_CV_CORE_CLK_GATE_EN, CLK_800_CV_CORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 16, 0, &clk_spinlock);
    clks[LB_CV_AXIM0_CLK] = clk_register_gate(NULL, LB_CV_AXIM0_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 15, 0, &clk_spinlock);
    clks[LB_CV_AXIM1_CLK] = clk_register_gate(NULL, LB_CV_AXIM1_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 14, 0, &clk_spinlock);
    clks[LB_CV_AXIS_CLK] = clk_register_gate(NULL, LB_CV_AXIS_CLK_GATE_EN, CLK_800_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 13, 0, &clk_spinlock);

    /* NET */
    clk_register_mux(NULL, CLK_800_NET_SEL, clk_800_net_sel_parents, ARRAY_SIZE(clk_800_net_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 18, 2, 0, &clk_spinlock);
    clk_register_mux(NULL, CLK_800_DSPCORE_SEL, clk_800_dspcore_sel_parents, ARRAY_SIZE(clk_800_dspcore_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_CLKMUX_SEL1, 20, 2, 0, &clk_spinlock);
    clks[LB_NET_CLK] = clk_register_gate(NULL, LB_NET_CLK_I_GATE_EN, CLK_800_NET_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 25, 0, &clk_spinlock);
    clks[LB_NET_DSPCORE_CLK] = clk_register_gate(NULL, LB_NET_DSPCORE_CLK_I_GATE_EN, CLK_800_DSPCORE_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 26, 0, &clk_spinlock);
}


/**
 * pll_disp_ptd_clocks_register()
 * @np    node pointer
 */
static void __init pll_disp_ptd_clocks_register(struct device_node *np)
{
    clk_register_bst_pll(PLL_DISP_PTD, MAIN_CLK, PLL_DISP_BASE, &pll_disp_spinlock, 
        (bst_pll_mux *)pll_disp_ptd_mux, ARRAY_SIZE(pll_disp_ptd_mux), 2);
    
    clk_register_mux(NULL, CLK_DISPLAY_1650_SEL, clk_display_1650_sel_parents, ARRAY_SIZE(clk_display_1650_sel_parents),
        CLK_SET_RATE_PARENT, TOP_CRM_REG_R_PLL_CLKMUX_SEL, 24, 1, 0, &clk_spinlock);
    clk_register_fixed_factor(NULL, CLK_DISPLAY_1650_SEL_FIX_FACTOR_10_1, CLK_DISPLAY_1650_SEL, CLK_SET_RATE_PARENT, 1, 10);

    /* VSP */
    clks[LB_VSP_DISP_CLK] = clk_register_gate(NULL, LB_VSP_DISP_CLK_GATE_EN, CLK_DISPLAY_1650_SEL_FIX_FACTOR_10_1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 21, 0, &clk_spinlock);
    clks[LB_VSP_CLK] = clk_register_gate(NULL, LB_VSP_CLK_GATE_EN, CLK_400_SYSBUS_AXI_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 20, 0, &clk_spinlock);
    clks[LB_VSP_AHB_CLK] = clk_register_gate(NULL, LB_VSP_AHB_HCLK_GATE_EN, CLK_200_SYSBUS_AHB_SEL, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED,
        TOP_CRM_REG_R_CLKGATE_EN1, 19, 0, &clk_spinlock);
}

static void __init bst_clk_setup(struct device_node *np)
{
    struct resource res;
    u32 i;

    /* resource init */
    for(i = RES_IDX_TOP_CRM; i <= RES_IDX_LB_LSP1_TOP; i++) {
        memset(&res, 0, sizeof(res));
        if (of_address_to_resource(np, i, &res)) {
            pr_err("Couldn't address to resource for reserved memory.\n");
            return;
        }

        a1000_clk_base_addr[i] = ioremap(res.start, resource_size(&res));
        if (! a1000_clk_base_addr[i]) {
            pr_err("%s,%d: ioremap fail.\n", __func__, __LINE__);
        }
    }

    pll_mux_addr_init();

    main_clk_register(np);
    misc_fix_rate_clocks_register(np);
    pll_cpu_clocks_register(np);
    pll_dsu_clocks_register(np);
    pll_hsp_lsp_clocks_register(np);
    pll_hsp_lsp_ptd_clocks_register(np);
    pll_sysbus_clocks_register(np);
    pll_sysbus_ptd_clocks_register(np);
    pll_gmac_ptd_clocks_register(np);
    pll_disp_ptd_clocks_register(np);
    pll_coreip_clocks_register(np);

    pll_main_ptd_clocks_register(np);
    pll_lsp_clocks_register(np);

    bst_clk_data.clks = clks;
	bst_clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(np, of_clk_src_onecell_get, &bst_clk_data);

}

int  clk_pr_debug_test(void)
{
	pr_debug("/****** clk: this is pr_debug in module clk. ******/\n");
	return 0;
}
EXPORT_SYMBOL(clk_pr_debug_test);

CLK_OF_DECLARE(bst_clkc, "bst,a1000-clkc", bst_clk_setup);

MODULE_DESCRIPTION("BST A1000 clock driver");
MODULE_AUTHOR("Kun Niu <kun.niu@bst.ai>");
MODULE_LICENSE("GPL v2");

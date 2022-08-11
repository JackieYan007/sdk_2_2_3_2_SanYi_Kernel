// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 *
 * Copyright (C) 2018 Synaptics Incorporated
 *
 * Author: Jisheng Zhang <jszhang@kernel.org>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/ktime.h>

#include "../sdhci-pltfm.h"
#include "bst-sdhci.h"

struct dwcmshc_priv {
	struct clk	*bus_clk;
};
#if 1//add by wangfei
#define REG_SDMMC_SOFTRST_SEL        0x33002004
#define REG_SD_EMMC_SEL       0x33000064  //bit0 for sd0/emmc0  bit1 for sd1/emmc1  (0:sd  1:emmc)
#define BST_SDMMC_VER_ID        0x3138302A
#define SDHCI_VENDOR_PTR_R	0xE8
#define SYS_CTRL_SDEMMC_DIV_CTRL     0x3300003C
#define SYS_CTRL_SDEMMC_CTRL_EN_CLR        0x33000068

/* Synopsys vendor specific registers */
#define reg_offset_addr_vendor		(sdhci_readw(host, SDHCI_VENDOR_PTR_R))
#define SDHC_MHSC_VER_ID_R				(reg_offset_addr_vendor)
#define SDHC_MHSC_VER_TPYE_R				(reg_offset_addr_vendor+0X4)
#define SDHC_MHSC_CTRL_R					(reg_offset_addr_vendor+0X8)
#define SDHC_MBIU_CTRL_R					(reg_offset_addr_vendor+0X10)
#define SDHC_EMMC_CTRL_R					(reg_offset_addr_vendor+0X2C)
#define SDHC_BOOT_CTRL_R					(reg_offset_addr_vendor+0X2E)
#define SDHC_GP_IN_R						(reg_offset_addr_vendor+0X30)
#define SDHC_GP_OUT_R						(reg_offset_addr_vendor+0X34)
#define SDHC_AT_CTRL_R					(reg_offset_addr_vendor+0X40)
#define SDHC_AT_STAT_R					(reg_offset_addr_vendor+0X44)

#define SDHC_SW_TUNE_EN		0x00000010

/* MMCM DRP */
#define SDHC_MMCM_DIV_REG	0x1020
#define DIV_REG_100_MHZ		0x1145
#define DIV_REG_200_MHZ		0x1083
#define SDHC_MMCM_CLKFBOUT	0x1024
#define CLKFBOUT_100_MHZ	0x0000
#define CLKFBOUT_200_MHZ	0x0080
#define SDHC_CCLK_MMCM_RST	0x00000001
#define DRIVER_NAME "sdhci_bst"

#define SDHCI_DUMP_BST(f, x...) \
	pr_err("%s: " DRIVER_NAME ": " f, mmc_hostname(host->mmc), ## x)

static void sdhci_bst_print_vendor(struct sdhci_host *host)
{
	SDHCI_DUMP_BST("============ SDHCI VENDOR REGISTER DUMP ===========\n");

	SDHCI_DUMP_BST("VER_ID:  0x%08x | VER_TPYE:  0x%08x\n",
		   sdhci_readl(host, SDHC_MHSC_VER_ID_R),
		   sdhci_readl(host, SDHC_MHSC_VER_TPYE_R));
	SDHCI_DUMP_BST("MHSC_CTRL:  0x%08x |MBIU_CTRL:  0x%08x\n",
		   sdhci_readw(host, SDHC_MHSC_CTRL_R),
		   sdhci_readw(host, SDHC_MBIU_CTRL_R));
	SDHCI_DUMP_BST("EMMC_CTRL:  0x%08x | BOOT_CTRL: 0x%08x\n",
		   sdhci_readl(host, SDHC_EMMC_CTRL_R),
		   sdhci_readw(host, SDHC_BOOT_CTRL_R));
	SDHCI_DUMP_BST("GP_IN:   0x%08x | GP_OUT: 0x%08x\n",
		   sdhci_readl(host, SDHC_GP_IN_R),
		   sdhci_readb(host, SDHC_GP_OUT_R));
	SDHCI_DUMP_BST("AT_CTRL:     0x%08x | AT_STAT:  0x%08x\n",
		   sdhci_readb(host, SDHC_AT_CTRL_R),
		   sdhci_readb(host, SDHC_AT_STAT_R));

}
EXPORT_SYMBOL_GPL(sdhci_bst_print_vendor);

static u32 mali_read_phys_bst(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}


static void mali_write_phys_bst(u32 phys_addr, u32 value)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}
}
static unsigned int bst_get_max_clock(struct sdhci_host *host)
{
	return host->mmc->f_max;
}

static unsigned int bst_get_min_clock(struct sdhci_host *host)
{
	return host->mmc->f_min;
}

void sdhci_enable_bst_clk(struct sdhci_host *host, u16 clk)
{
    #define default_max_freq 200000ul
    u32 div;

    if (clk == 0)
    {
	//SDHCI_DUMP_BST("bad clk :%d\n", clk);
	//return;
	div = clk;
    } else if(clk>default_max_freq)
    {
	div = clk/1000;
        div = default_max_freq/div;
    } else if(clk<1500)
    {
        div = clk*2;
	//div=clk;
    } else
    {
	div = default_max_freq*100;
	div = div/clk;
        div /= 100;
    }

    clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk &= ~SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
    if (div != 0)
    {
        mali_write_phys_bst(SYS_CTRL_SDEMMC_DIV_CTRL, ((div&0x3ff)<<10)|(div&0x3ff));
        mali_write_phys_bst(SYS_CTRL_SDEMMC_CTRL_EN_CLR, mali_read_phys_bst(SYS_CTRL_SDEMMC_CTRL_EN_CLR)|BIT(8)|BIT(4));
    }
    clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
    clk |= SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

}
static u16 bst_sdhci_calc_clk(struct sdhci_host *host, unsigned int clock,
		   unsigned int *actual_clock)
{
	int div = 0; /* Initialized for compiler warning */
	int real_div = div, clk_mul = 1;
	u16 clk = 0;
	bool switch_base_clk = false;

	if (host->version >= SDHCI_SPEC_300) {
		if (host->clk_mul) {
			for (div = 1; div <= 1024; div++) {
				if ((host->max_clk * host->clk_mul / div)
					<= clock)
					break;
			}
			if ((host->max_clk * host->clk_mul / div) <= clock) {
				/*
				 * Set Programmable Clock Mode in the Clock
				 * Control register.
				 */

				clk = SDHCI_PROG_CLOCK_MODE;
				real_div = div;
				clk_mul = host->clk_mul;
				div--;
			} else {
				/*
				 * Divisor can be too small to reach clock
				 * speed requirement. Then use the base clock.
				 */
				switch_base_clk = true;
			}
		}

		if (!host->clk_mul || switch_base_clk) {
			/* Version 3.00 divisors must be a multiple of 2. */
			if (host->max_clk <= clock)
				div = 1;
			else {
				for (div = 2; div < SDHCI_MAX_DIV_SPEC_300;
				     div += 2) {
					if ((host->max_clk / div) <= clock)
						break;
				}
			}
			real_div = div;
			div >>= 1;
			if ((host->quirks2 & SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN)
				&& !div && host->max_clk <= 25000000)
				div = 1;
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((host->max_clk / div) <= clock)
				break;
		}
		real_div = div;

	}

clock_set:
	if (real_div)
		*actual_clock = (host->max_clk * clk_mul) / real_div;
    clk = real_div;
	return clk;
}

void sdhci_set_bst_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;

	host->mmc->actual_clock = 0;

	if (clock == 0)
		return;
	clk = bst_sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	sdhci_enable_bst_clk(host, clk);
}

static void sdhci_bst_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	sdhci_reset(host, mask);
}
static void sdhci_bst_timeout(struct sdhci_host *host, struct mmc_command *cmd)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
    sdhci_writeb(host, 0xE, SDHCI_TIMEOUT_CONTROL);
}

static void sdhci_bst_set_power(struct sdhci_host *host, unsigned char mode,
		     unsigned short vdd)
{
	if (!IS_ERR(host->mmc->supply.vmmc)) {
		struct mmc_host *mmc = host->mmc;

		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, vdd);
	}
	sdhci_set_power(host, mode, vdd);
    sdhci_writeb(host, 0xF, SDHCI_POWER_CONTROL);
}

static u32 dwcmshc_readl_fixup(struct sdhci_host *host,
				     int spec_reg, u32 value)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *dwcmshc = sdhci_pltfm_priv(pltfm_host);
	u32 ret;

	/*
	 * The bit of ADMA flag in eSDHC is not compatible with standard
	 * SDHC register, so set fake flag SDHCI_CAN_DO_ADMA2 when ADMA is
	 * supported by eSDHC.
	 * And for many FSL eSDHC controller, the reset value of field
	 * SDHCI_CAN_DO_ADMA1 is 1, but some of them can't support ADMA,
	 * only these vendor version is greater than 2.2/0x12 support ADMA.
	 */
	if ((spec_reg == SDHCI_CAPABILITIES) && (value & SDHCI_CAN_DO_ADMA1)) {
		{
			ret = value | SDHCI_CAN_DO_ADMA2;
			return ret;
		}
	}
	/*
	 * The DAT[3:0] line signal levels and the CMD line signal level are
	 * not compatible with standard SDHC register. The line signal levels
	 * DAT[7:0] are at bits 31:24 and the command line signal level is at
	 * bit 23. All other bits are the same as in the standard SDHC
	 * register.
	 */
	if (spec_reg == SDHCI_PRESENT_STATE) {
		ret = value & 0x000fffff;
		ret |= (value >> 4) & SDHCI_DATA_LVL_MASK;
		ret |= (value << 1) & SDHCI_CMD_LVL;
		return ret;
	}

	/*
	 * DTS properties of mmc host are used to enable each speed mode
	 * according to soc and board capability. So clean up
	 * SDR50/SDR104/DDR50 support bits here.
	 */
	if (spec_reg == SDHCI_CAPABILITIES_1) {
		ret = value & ~(SDHCI_SUPPORT_SDR104 | SDHCI_SUPPORT_DDR50);
		return ret;
	}

	ret = value;
	return ret;
}

static u16 dwcmshc_readw_fixup(struct sdhci_host *host,
				     int spec_reg, u32 value)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *dwcmshc = sdhci_pltfm_priv(pltfm_host);
	u16 ret;
	int shift = (spec_reg & 0x2) * 8;

	if (spec_reg == SDHCI_HOST_VERSION)
		ret = value & 0xffff;
	else
		ret = (value >> shift) & 0xffff;
	return ret;
}

static u8 dwcmshc_readb_fixup(struct sdhci_host *host,
				     int spec_reg, u32 value)
{
	u8 ret;
	u8 dma_bits;
	int shift = (spec_reg & 0x3) * 8;

	ret = (value >> shift) & 0xff;

	/*
	 * "DMA select" locates at offset 0x28 in SD specification, but on
	 * P5020 or P3041, it locates at 0x29.
	 */
	if (spec_reg == SDHCI_HOST_CONTROL) {
		/* DMA select is 22,23 bits in Protocol Control Register */
		dma_bits = (value >> 5) & SDHCI_CTRL_DMA_MASK;
		/* fixup the result */
		ret &= ~SDHCI_CTRL_DMA_MASK;
		ret |= dma_bits;
	}
	return ret;
}

/**
 * dwcmshc_write*_fixup - Fixup the SD spec register value so that it could be
 *			written into eSDHC register.
 *
 * @host: pointer to sdhci_host
 * @spec_reg: SD spec register address
 * @value: 8/16/32bit SD spec register value that would be written
 * @old_value: 32bit eSDHC register value on spec_reg address
 *
 * In SD spec, there are 8/16/32/64 bits registers, while all of eSDHC
 * registers are 32 bits. There are differences in register size, register
 * address, register function, bit position and function between eSDHC spec
 * and SD spec.
 *
 * Return a fixed up register value
 */
static u32 dwcmshc_writel_fixup(struct sdhci_host *host,
				     int spec_reg, u32 value, u32 old_value)
{
	u32 ret;

	/*
	 * Enabling IRQSTATEN[BGESEN] is just to set IRQSTAT[BGE]
	 * when SYSCTL[RSTD] is set for some special operations.
	 * No any impact on other operation.
	 */
	if (spec_reg == SDHCI_INT_ENABLE)
		ret = value | SDHCI_INT_BLK_GAP;
	else
		ret = value;

	return ret;
}

static u32 dwcmshc_writew_fixup(struct sdhci_host *host,
				     int spec_reg, u16 value, u32 old_value)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int shift = (spec_reg & 0x2) * 8;
	u32 ret;

	switch (spec_reg) {
	case SDHCI_TRANSFER_MODE:
		/*
		 * Postpone this write, we must do it together with a
		 * command write that is down below. Return old value.
		 */
		pltfm_host->xfer_mode_shadow = value;
		return old_value;
	case SDHCI_COMMAND:
		ret = (value << 16) | pltfm_host->xfer_mode_shadow;
		return ret;
	}

	ret = old_value & (~(0xffff << shift));
	ret |= (value << shift);

	if (spec_reg == SDHCI_BLOCK_SIZE) {
		/*
		 * Two last DMA bits are reserved, and first one is used for
		 * non-standard blksz of 4096 bytes that we don't support
		 * yet. So clear the DMA boundary bits.
		 */
		ret &= (~SDHCI_MAKE_BLKSZ(0x7, 0));
	}
	return ret;
}

static u32 dwcmshc_writeb_fixup(struct sdhci_host *host,
				     int spec_reg, u8 value, u32 old_value)
{
	u32 ret;
	u32 dma_bits;
	u8 tmp;
	int shift = (spec_reg & 0x3) * 8;

	/*
	 * eSDHC doesn't have a standard power control register, so we do
	 * nothing here to avoid incorrect operation.
	 */
	if (spec_reg == SDHCI_POWER_CONTROL)
		return old_value;
	/*
	 * "DMA select" location is offset 0x28 in SD specification, but on
	 * P5020 or P3041, it's located at 0x29.
	 */
	if (spec_reg == SDHCI_HOST_CONTROL) {
		/*
		 * If host control register is not standard, exit
		 * this function
		 */
		if (host->quirks2 & SDHCI_QUIRK2_BROKEN_HOST_CONTROL)
			return old_value;

		/* DMA select is 22,23 bits in Protocol Control Register */
		dma_bits = (value & SDHCI_CTRL_DMA_MASK) << 5;
		ret = (old_value & (~(SDHCI_CTRL_DMA_MASK << 5))) | dma_bits;
		tmp = (value & (~SDHCI_CTRL_DMA_MASK)) |
		      (old_value & SDHCI_CTRL_DMA_MASK);
		ret = (ret & (~0xff)) | tmp;

		return ret;
	}

	ret = (old_value & (~(0xff << shift))) | (value << shift);
	return ret;
}

static u32 dwcmshc_bst_readl(struct sdhci_host *host, int reg)
{
	u32 ret;
	u32 value;

	value = readl(host->ioaddr + reg);
	ret = dwcmshc_readl_fixup(host, reg, value);

	return ret;
}


static u16 dwcmshc_bst_readw(struct sdhci_host *host, int reg)
{
	u16 ret;
	u32 value;
	int base = reg & ~0x3;

	value = readl(host->ioaddr + base);
	ret = dwcmshc_readw_fixup(host, reg, value);
	return ret;
}


static u8 dwcmshc_bst_readb(struct sdhci_host *host, int reg)
{
	u8 ret;
	u32 value;
	int base = reg & ~0x3;

	value = readl(host->ioaddr + base);
	ret = dwcmshc_readb_fixup(host, reg, value);
	return ret;
}


static void dwcmshc_bst_writel(struct sdhci_host *host, u32 val, int reg)
{
	u32 value;

	value = dwcmshc_writel_fixup(host, reg, val, 0);
	writel(value, host->ioaddr + reg);
}


static void dwcmshc_bst_writew(struct sdhci_host *host, u16 val, int reg)
{
	int base = reg & ~0x3;
	u32 value;
	u32 ret;

	value = readw(host->ioaddr + base);
	ret = dwcmshc_writew_fixup(host, reg, val, value);
	if (reg != SDHCI_TRANSFER_MODE)
		writew(ret, host->ioaddr + base);
}



static void dwcmshc_bst_writeb(struct sdhci_host *host, u8 val, int reg)
{
	int base = reg & ~0x3;
	u32 value;
	u32 ret;

	value = readw(host->ioaddr + base);
	ret = dwcmshc_writeb_fixup(host, reg, val, value);
	writew(ret, host->ioaddr + base);
}



static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock		= sdhci_set_bst_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
	.get_min_clock		= bst_get_min_clock,
	.get_max_clock		= bst_get_max_clock,
	.reset			= sdhci_bst_reset,
	.set_power = sdhci_bst_set_power,
    .set_timeout = sdhci_bst_timeout,
};
#endif
static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_DELAY_AFTER_POWER|SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
    .quirks2 = SDHCI_QUIRK2_BROKEN_DDR50,
};

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;
    char c;

    dev_err(&pdev->dev, "dwcmshc_probe\n");

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
    {
		clk_prepare_enable(priv->bus_clk);
    }
	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	sdhci_get_of_property(pdev);
    mali_write_phys_bst(0x3300005C, 0xc8c8c8c8);//timer 默认 25m
    mali_write_phys_bst(0x33000060, 0x90);//  tx 反相
    //mali_write_phys_bst(0x3300003C,0xFA3E8);//  200m 分频系数改为200k
    //mali_write_phys_bst(0x33000068,0xFF0);//  分频系数锁存使能
    mali_write_phys_bst(0x330010A0, mali_read_phys_bst(0x330010A0)|0x06060606);//  io strength    
    mali_write_phys_bst(0x330010A4, mali_read_phys_bst(0x330010A4)|0x06060606);//  io strength    
    mali_write_phys_bst(0x330010A8, mali_read_phys_bst(0x330010A8)|0x06060606);//  io strength    
    mali_write_phys_bst(0x330010AC, mali_read_phys_bst(0x330010AC)|0x06060606);//  io strength    
    if (sdhci_readl(host, SDHC_MHSC_VER_ID_R) != BST_SDMMC_VER_ID)
    {
	dev_err(&pdev->dev, "dwcmshc_probe wrong ver id\n");
	goto err_clk;
    }

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "snps,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.of_match_table = sdhci_dwcmshc_dt_ids,
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Synopsys DWC MSHC");
MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_LICENSE("GPL v2");

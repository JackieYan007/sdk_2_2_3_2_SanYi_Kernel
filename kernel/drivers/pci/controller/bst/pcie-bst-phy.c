// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare PCIe PHY driver
 *
 * Copyright (C) 2022 Black Sesame Tec Co., Ltd.
 *		https://www.bst.ai
 *
 * Author: GaoQingpeng <qingpeng.gao@bst.ai>
 */

#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_gpio.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>

#include "../../pci.h"
#include "pcie-bst.h"
#include "pcie-bst-phy.h"

static u32 pcie_phy_init_state;
static u32 __iomem *phy_base;

static inline void pcie_phy_write(struct pcie_phy *phy, u32 offset, u32 val)
{
	dw_pcie_write(phy->phy_base + offset, 4, val);
}

static u32 pcie_phy_read(struct pcie_phy *phy, u32 offset)
{
	u32 reg_val;

	dw_pcie_read((phy->phy_base + offset), 4, &reg_val);
	return reg_val;
}

/* return link status */
int pcie_is_link(struct dw_pcie *pci)
{
	u32 temp = 0;
	struct pcie_phy *phy = pci->phy;

	/* Get link status form LTSSM bit[5:0] */
	if (pci->ctrl_id == 0) {
		temp = pcie_phy_read(phy, 0xd4) & 0x3f;
		//printk(KERN_ERR "RC:LTSSM 0x%x\n",temp);
	} else if (pci->ctrl_id == 1) {
		temp = pcie_phy_read(phy, 0xec) & 0x3f;
		//printk(KERN_ERR "EP:LTSSM 0x%x\n",temp);
	} else {
		pr_err("status: ctrl_id error\n");
		return 0;
	}
	phy->ltssm = temp;

	/* >= state L0 */
	return (temp >= 0x11) ? 1 : 0;
}

static void pcie_sw_reset(void)
{
	void __iomem *top_crm;
	u32 reg_value;

	top_crm = ioremap(0x33002000, 0x1000);
	if (top_crm == NULL) {
		pr_info("ioremap fail 0x33002000");
		return;
	}

#ifdef CONFIG_ARCH_BSTA1000A
	reg_value = readl(top_crm + 0xe8);
	reg_value &= (~(0x1 << 8));
	writel(reg_value, top_crm + 0xe8);
	mdelay(10);
	reg_value |= (0x1 << 8);
	writel(reg_value, top_crm + 0xe8);
#endif

#ifdef CONFIG_ARCH_BSTA1000B
	reg_value = readl(top_crm + 0x180);
	reg_value &= (~(0x1 << 10));
	writel(reg_value, top_crm + 0x180);
	mdelay(10);
	reg_value |= (0x1 << 10);
	writel(reg_value, top_crm + 0x180);
#endif

	iounmap(top_crm);
}

void pcie_ltssm_enable(struct dw_pcie *pci)
{
	u32 reg_val;
	struct pcie_phy *phy = pci->phy;

	/* 1=enable bit0:dmc(x4c) bit1:epc(x2c) */
	if (pci->ctrl_id == 0) {
		reg_val = pcie_phy_read(phy, PCIE_LTSSM_EN);
		reg_val |= 0x1;
		pcie_phy_write(phy, PCIE_LTSSM_EN, reg_val);
		pr_info("pcie ctrl 0 ltssm enable\n");
	} else if (pci->ctrl_id == 1) {
		reg_val = pcie_phy_read(phy, PCIE_LTSSM_EN);
		reg_val |= 0x2;
		pcie_phy_write(phy, PCIE_LTSSM_EN, reg_val);
		pr_info("pcie ctrl 1 ltssm enable\n");
	}
}

void pcie_ltssm_disable(struct dw_pcie *pci)
{
	u32 reg_val;
	struct pcie_phy *phy = pci->phy;

	/* 0=disable bit0:dmc(x4c) bit1:epc(x2c) */
	if (pci->ctrl_id == 0) {
		reg_val = pcie_phy_read(phy, PCIE_LTSSM_EN);
		reg_val &= ~0x1;
		pcie_phy_write(phy, PCIE_LTSSM_EN, reg_val);
		pr_info("pcie ctrl 0 ltssm disable\n");
	} else if (pci->ctrl_id == 1) {
		reg_val = pcie_phy_read(phy, PCIE_LTSSM_EN);
		reg_val &= ~0x2;
		pcie_phy_write(phy, PCIE_LTSSM_EN, reg_val);
		pr_info("pcie ctrl 1 ltssm disable\n");
	}
}

static int pcie_phy_is_inited(void)
{
	return pcie_phy_init_state;
}

static void a1000_pcie_phyinit(struct pcie_phy *phy)
{
	u32 reg_val;

	/* PCIe sw-reset */
	pcie_sw_reset();
	pr_info("Release PCIe Soft-reset from top_crm");

	/* Reset PCIe PHY and Controller(0 and 1) */
	pcie_phy_write(phy, PCIE_LOCAL_RST, 0x00);
	pr_info("Reset PCIe PHY and Controller(0 and 1)");

	/* Configure PCIe-PHY ref clk from board or inner */
	if (phy->inner_clk) {
		pcie_phy_write(phy, PCIE_CLK_CTRL, 0x240);//bit[6,7]
		pr_info("Configure PCIe-PHY ref clk from inner clk");
	} else {
		pcie_phy_write(phy, PCIE_CLK_CTRL, 0xc0);//bit[6,7]
		pr_info("Configure PCIe-PHY ref clk from PAD on board clk_gen");
	}
	/*
	 * Configure DMx4 controler As RC or EP mode
	 * bit1: 1=RC,0=EP bit0: 1=2x2ports,0=1x4ports
	 */
	reg_val = phy->dmc_mode | phy->dmc_lane;
	pcie_phy_write(phy, PCIE_MODE, reg_val);
	pr_info("Configure PCIe mode:%#x", reg_val);

#if (defined CONFIG_ARCH_BSTA1000B)
	if (bst_get_soc_board_type() == BSTA1000B_BOARD_EVB) {
		reg_val = pcie_phy_read(phy, PCIE_LANEX_LINK_NUM);
		reg_val = reg_val | 0x1111;
		pcie_phy_write(phy, PCIE_LANEX_LINK_NUM, reg_val);

		reg_val = pcie_phy_read(phy, PCIE_PHY_SRC_SEL);
		reg_val = reg_val | 0x55;
		pcie_phy_write(phy, PCIE_PHY_SRC_SEL, reg_val);

		pcie_phy_write(phy, PCIE_LANE_FLIP_EN, 0xc);
	}

#elif (defined CONFIG_ARCH_BSTA1000A)
	if (bst_get_soc_board_type() == BSTA1000_BOARD_FADB) {
		reg_val = pcie_phy_read(phy, PCIE_LANEX_LINK_NUM);
		reg_val = reg_val | 0x1100;
		pcie_phy_write(phy, PCIE_LANEX_LINK_NUM, reg_val); //lane2和3

		reg_val = pcie_phy_read(phy, PCIE_PHY_SRC_SEL);
		reg_val = reg_val | 0x50;
		pcie_phy_write(phy, PCIE_PHY_SRC_SEL, reg_val);

		pcie_phy_write(phy, PCIE_LANE_FLIP_EN, 0xc);
	}
#endif

	/* PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	pcie_phy_write(phy, PCIE_PHY_SRAM_CTRL, 0);//PHY Sram enable
	pr_info("Configure PCIe PHY Sram enable");

	/* PCIe-Controller APP-Hold-Phy */
	pcie_phy_write(phy, PCIE_PHY_CTRL, 0x06);
	pr_info("Configure PCIe-Controller APP-Hold-Phy");

	/* Release PCIe PHY reset */
	pcie_phy_write(phy, PCIE_LOCAL_RST, 0x4);

	/* waiting PHY SRAM intitial Done */
	while (pcie_phy_read(phy, PCIE_PHY_SRAM_INIT_STATUS) != 0x3)
		udelay(100);

	/* PCIe PHY SRAM intitial Done Finish */
	pcie_phy_write(phy, PCIE_PHY_SRAM_CTRL, 0x3);
	pr_info("PCIe PHY SRAM intitial Done Finish");

	/* Release PCIe Controller reset */
	reg_val = 0x4 | (phy->pcie_ctl0 << 0) | (phy->pcie_ctl1 << 1);
	pcie_phy_write(phy, PCIE_LOCAL_RST, reg_val);
	pr_info("Release PCIe Controller reset:%#x\n", reg_val);

	/* Configure PCIe-Controller APP-Hold-Phy */
	pcie_phy_write(phy, PCIE_PHY_CTRL, 0x00);
	pr_info("Configure PCIe-Controller APP-Hold-Phy");

	/* Disable LTSSM */
	pcie_phy_write(phy, PCIE_LTSSM_EN, 0x0);
}

/* mask high speed error */
static void mask_high_speed_error(void)
{
	u32 reg_value;
	void __iomem *sysctrl_va;

	sysctrl_va = ioremap(0x33000000, 0x3000);
	if (sysctrl_va == NULL) {
		pr_info("ioremap fail 0x33000000");
		return;
	}

	reg_value = readl(sysctrl_va + 0x124);
	reg_value &= (~(1 << 4));
	writel(reg_value, sysctrl_va + 0x124);
	reg_value = readl(sysctrl_va + 0x128);
	reg_value &= (~(1 << 4));
	writel(reg_value, sysctrl_va + 0x128);

	iounmap(sysctrl_va);
}

int bst_pcie_phyinit(struct dw_pcie *pci)
{
	int ret;
	int ssd_en_gpio;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct device_node *phy_np = NULL;
	struct pcie_phy *phy;
	struct clk *ref_clk, *aux_clk, *axi_clk, *apb_clk;

	pr_info("pcie-phy init...\n");
	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	pci->phy = phy;

	phy_np = of_parse_phandle(np, "pcie-phy", 0);
	if (!phy_np) {
		pr_err("dts pcie-phy undefined\n");
		return -ENODEV;
	}

	if (!phy_base) {
		phy_base = of_iomap(phy_np, 0);
		if (!phy_base)
			return -ENOMEM;
	}
	phy->phy_base = phy_base;
	pr_info("pcie-phy: phy:%#lx, phy_base:%#lx\n", (uintptr_t) phy, (uintptr_t) phy_base);

	ssd_en_gpio = of_get_named_gpio(np, "ssd-en-gpio", 0);
	if (ssd_en_gpio >= 0) {
		//devm_gpio_request(dev, ssd_en_gpio, dev_name(dev));
		gpio_direction_output(ssd_en_gpio, 1);
		pr_info("ssd-en-gpio enable\n");
	}

	/* bst pcie-phy init only do once */
	if (pcie_phy_is_inited()) {
		pr_info("pcie phy already init\n");
		return 0;
	}
	mask_high_speed_error();

	ret = of_property_read_u32(phy_np, "dmc-mode", &phy->dmc_mode);
	if (ret) {
		phy->dmc_mode = PCIE_DWC_RC_MODE;
		pr_info("dmc-mode undefined, use default:%#x\n", phy->dmc_mode);
	}
	ret = of_property_read_u32(phy_np, "dmc-lane", &phy->dmc_lane);
	if (ret) {
		phy->dmc_lane = PCIE_DWC_LINEX4;
		pr_info("dmc-lane undefined, use default:%#x\n", phy->dmc_lane);
	}

	ret = of_property_read_u32(phy_np, "pcie-ctl0", &phy->pcie_ctl0);
	if (ret) {
		phy->pcie_ctl0 = 1;
		pr_info("pcie-ctl0 undefined, use default:%#x\n", phy->pcie_ctl0);
	}
	ret = of_property_read_u32(phy_np, "pcie-ctl1", &phy->pcie_ctl1);
	if (ret) {
		phy->pcie_ctl1 = 1;
		pr_info("pcie-ctl1 undefined, use default:%#x\n", phy->pcie_ctl1);
	}
	pr_info("pcie phy mode:%#x,lane:%#x,ctl0:%d,ctl1:%d\n",
		phy->dmc_mode, phy->dmc_lane, phy->pcie_ctl0, phy->pcie_ctl1);
	phy->inner_clk = of_property_read_bool(phy_np, "inner-clk");
	if (phy->inner_clk) {
		/* inner clock setup */
		ref_clk = of_clk_get_by_name(phy_np, "ref_clk");
		if (IS_ERR(ref_clk)) {
			dev_warn(dev, "Cannot get ref_clk\n");
			ref_clk = NULL;
		} else {
			clk_prepare_enable(ref_clk);
		}
		aux_clk = of_clk_get_by_name(phy_np, "aux_clk");
		if (IS_ERR(aux_clk)) {
			dev_warn(dev, "Cannot get ref_clk\n");
			aux_clk = NULL;
		} else {
			clk_prepare_enable(aux_clk);
		}
		axi_clk = of_clk_get_by_name(phy_np, "axi_clk");
		if (IS_ERR(axi_clk)) {
			dev_warn(dev, "Cannot get axi_clk\n");
			axi_clk = NULL;
		} else {
			clk_prepare_enable(axi_clk);
		}
		apb_clk = of_clk_get_by_name(phy_np, "apb_clk");
		if (IS_ERR(apb_clk)) {
			dev_warn(dev, "Cannot get apb_clk\n");
			apb_clk = NULL;
		} else {
			clk_prepare_enable(apb_clk);
		}
	}

	a1000_pcie_phyinit(phy);
	pcie_phy_init_state = 1;
	pr_info("pcie-phy init complete\n");

	return 0;
}

void pcie_reset_ctrl(struct dw_pcie *pci)
{
	u32 val;
	struct pcie_phy *phy = pci->phy;
	int hc_id = pci->ctrl_id;

	val = pcie_phy_read(phy, PCIE_LOCAL_RST); //reset pcie controller
	val &= ~(1 << hc_id);
	pcie_phy_write(phy, PCIE_LOCAL_RST, val);

	mdelay(10);

	val |= (1 << hc_id);
	pcie_phy_write(phy, PCIE_LOCAL_RST, val); //release pcie controller
}
EXPORT_SYMBOL(pcie_reset_ctrl);

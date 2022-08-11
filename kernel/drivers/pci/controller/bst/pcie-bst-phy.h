// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare PCIe PHY driver
 *
 * Copyright (C) 2022 Black Sesame Tec Co., Ltd.
 *		https://www.bst.ai
 *
 * Author: GaoQingpeng <qingpeng.gao@bst.ai>
 */

#ifndef _PCIE_BST_PHY_H
#define _PCIE_BST_PHY_H

#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>

#define BSTPCIE_NEW_MSIBUF		1

#define     PCIE_LTSSM_EN             0x00
#define     PCIE_PHY_CTRL             0x04
#define     PCIE_MODE                 0x08
#define     PCIE_CLK_CTRL             0x0c
#define     PCIE_PHY_LPBK             0x10
#define     PCIE_LANE_FLIP_EN         0x14
#define     PCIE_DIAG_CTRL            0x18
#define     PCIE_DEBUG_PIN_OUT        0x1c
#define     PCIE_PHY_SRAM_CTRL        0x20
#define     PCIE_PHY_SRAM_INIT_STATUS 0x24
#define     PCIE_PHY_CRI_TYPE         0x28
#define     PCIE_PHY_CRI_SEL          0x2c
#define     PCIE_PHY_CRI_ADDR         0x30
#define     PCIE_PHY_CRT_WDATA        0x34
#define     PCIE_PHY_CRI_RDATA        0x38

#define     PCIE_LOCAL_RST			0x11c
#define     PCIE_PHY_SRC_SEL		0x13c
#define     PCIE_LANEX_LINK_NUM		0x140

/* PHY-REG PCIE_MODE CFG*/
#define     PCIE_DWC_EP_MODE   0x0
#define     PCIE_DWC_RC_MODE   0x2
#define     PCIE_DWC_LINEX2    0x1
#define     PCIE_DWC_LINEX4    0x0

struct pcie_phy {
	void  __iomem  *phy_base;
	u32     dmc_mode;
	u32     dmc_lane;
	u32     pcie_ctl0;
	u32     pcie_ctl1;
	u32     ltssm;
	bool	inner_clk;
};

int pcie_is_link(struct dw_pcie *pci);
void pcie_ltssm_enable(struct dw_pcie *pci);
void pcie_ltssm_disable(struct dw_pcie *pci);
int bst_pcie_phyinit(struct dw_pcie *pci);
void pcie_reset_ctrl(struct dw_pcie *pci);

#endif //_PCIE_BST_PHY_H

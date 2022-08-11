// SPDX-License-Identifier: GPL-2.0

/*
 * VOUT DRM device definitions for BST
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "bst_vout_dev.h"

struct pll_display_data {
	u32 vout_freq; // KHz
	u32 fbdiv;
	u32 postdiv1;
	u32 postdiv2;
	u32 refdiv;
	u32 frac; // reserve, unused now
};

// vout_freq = 25MHz * fbdiv / (postdiv1 * postdiv2) / refdiv / 10
struct pll_display_data s_pll_array[] = {
	{ 165000, 0x42, 0x1, 0x1, 0x1, 0x0 }, // HDMI 1080P
	{ 82500, 0x42, 0x2, 0x1, 0x1, 0x0 }, // HDMI 720P
	{ 148500, 0x129, 0x1, 0x1, 0x5, 0x0 }, // RGB 1080P
	{ 74250, 0x129, 0x2, 0x1, 0x5, 0x0 }, // RGB 720P
};

void vout_reg_write(void *reg, uint32_t mask, uint32_t val)
{
	uint32_t status;

	status = readl_relaxed(reg);
	status = (status & mask | val);
	writel_relaxed(status, reg);
}

void vout_pclk_enable(int val)
{
	void *reg0;

	reg0 = ioremap(PLL_DISPLAY_REG0, 0x4);
	vout_reg_write(reg0, PLLEN_MASK, (val << PLLEN_SHIFT));
	iounmap(reg0);
}

int vout_pclk_init(struct vop_device *pvop)
{
	void *reg0;
	void *reg1;
	int array_size;
	int i;
	u32 postdiv;
	u32 refdiv;

	array_size = sizeof(s_pll_array) / sizeof(struct pll_display_data);

	reg0 = ioremap(PLL_DISPLAY_REG0, 0x4);
	reg1 = ioremap(PLL_DISPLAY_REG1, 0x4);

	// disable pll before config
	vout_reg_write(reg0, PLLEN_MASK, 0x00);
	for (i = 0; i < array_size; i++) {
		if (pvop->pclk_freq == s_pll_array[i].vout_freq)
			break;
	}

	if (i == array_size) {
		pr_err("ERROR: no matched vout frequency found\n");
		return -1;
	}

	vout_reg_write(reg0, FBDIV_MASK, (s_pll_array[i].fbdiv << FBDIV_SHIFT));
	postdiv = (s_pll_array[i].postdiv1 << POSTDIV1_SHIFT) |
		  (s_pll_array[i].postdiv2 << POSTDIV2_SHIFT);
	vout_reg_write(reg0, POSTDIV_MASK, postdiv);
	vout_reg_write(reg1, REFDIV_MASK,
		       (s_pll_array[i].refdiv << REFDIV_SHIFT));

	iounmap(reg0);
	iounmap(reg1);

	return 0;
}

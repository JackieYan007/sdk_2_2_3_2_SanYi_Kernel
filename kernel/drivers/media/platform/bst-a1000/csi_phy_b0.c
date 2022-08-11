// SPDX-License-Identifier: GPL-2.0

/*
 * CSI PHY driver for BST A1000B0
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>

#define Test0Clk	(1 << 1)
#define Test0Clr	(1 << 0)
#define Test1En		(1 << 16)
#define Test1DataOut(x) ((x & 0xff) << 8)
#define Test1DataIn(x)	(x & 0xff)

#define BIT_SET(x)  (x)
#define BIT_CLR(x)  (~x)
#define BIT_CLR_ALL 0

struct dphy_regs {
	// uint32_t *base;  //+0
	uint32_t *dphy_n_lanes; //+4
	uint32_t *csi2_resetz; //+8
	uint32_t *dphy_shutdownz; //+0x40
	uint32_t *dphy_rstz; //+0x44
	uint32_t *dphy_stop_state; //+0x4c
	uint32_t *dphy_test_ctrl0; //+0x50
	uint32_t *dphy_test_ctrl1; //+0x54
	uint32_t *mipi_test;
	uint32_t *mipi_mode;
};

static void dphy_reset(struct dphy_regs *pregs)
{
	*pregs->dphy_test_ctrl0 = BIT_SET(Test0Clr);
	msleep(100);
	*pregs->dphy_test_ctrl0 = BIT_CLR_ALL;
	msleep(100);
}

static void dphy_write4BitMSB(struct dphy_regs *pregs, unsigned char valMsb)
{
	// a. Ensure that Test0Clk and Test1En is set to low
	(*pregs->dphy_test_ctrl0) = BIT_CLR_ALL;
	(*pregs->dphy_test_ctrl1) = BIT_CLR_ALL;

	// b. Set Test1En to high
	(*pregs->dphy_test_ctrl1) |= Test1En;

	// c. Set Test0Clk to high
	(*pregs->dphy_test_ctrl0) |= Test0Clk;

	// d. Place 0x00 in testdin
	(*pregs->dphy_test_ctrl1) |= Test1DataIn(0);

	// e. Set Test0Clk to low (with the falling edge on Test0Clk, the
	// testdin signal content is latched internally)
	(*pregs->dphy_test_ctrl0) &= ~Test0Clk;

	// f. Set Test1En to low
	(*pregs->dphy_test_ctrl1) &= ~Test1En;

	// g. Place the 8-bit word corresponding to the testcode MSBs in testdin
	(*pregs->dphy_test_ctrl1) |= Test1DataIn(valMsb);

	// h. Set Test0Clk to high
	(*pregs->dphy_test_ctrl0) |= Test0Clk;
}

static void dphy_write8BitLSB(struct dphy_regs *pregs, unsigned char valLsb)
{
	// a. Set testclk to low
	(*pregs->dphy_test_ctrl0) &= ~Test0Clk;

	// b. Set testen to high
	(*pregs->dphy_test_ctrl1) |= Test1En;

	// c. Set testclk to high
	(*pregs->dphy_test_ctrl0) |= Test0Clk;

	// d. Place the 8-bit word test data in testdin
	// tmp = (*pregs->dphy_test_ctrl1);
	// tmp &= 0xffffff00;
	// tmp |=  Test1DataIn(valLsb);
	//(*pregs->dphy_test_ctrl1) =  tmp;
	(*pregs->dphy_test_ctrl1) = Test1En | (valLsb & 0xff);

	// e. Set testclk to low (with the falling edge on testclk, the testdin
	// signal content is latched internally)
	(*pregs->dphy_test_ctrl0) &= ~Test0Clk;

	// f. Set testen to low
	(*pregs->dphy_test_ctrl1) &= ~Test1En;
}

static void dphy_write8BitData(struct dphy_regs *pregs, unsigned char data)
{
	// a. Place the 8-bit word corresponding to the page offset in testdin
	//(*pregs->dphy_test_ctrl1) |=  Test1DataIn(data);
	(*pregs->dphy_test_ctrl1) = Test1DataIn(data);

	// b. Set testclk to high (test data is programmed internally)
	(*pregs->dphy_test_ctrl0) |= Test0Clk;

	(*pregs->dphy_test_ctrl0) &= ~Test0Clk;
}

static void dphy_writeReg(struct dphy_regs *pregs, unsigned short addr,
			  unsigned char val)
{
	dphy_write4BitMSB(pregs, (addr & 0xf00) >> 8);
	dphy_write8BitLSB(pregs, addr & 0xff);
	dphy_write8BitData(pregs, val);
}

static unsigned char dphy_readReg(struct dphy_regs *pregs, unsigned short addr)
{
	dphy_write4BitMSB(pregs, (addr & 0xf00) >> 8);
	dphy_write8BitLSB(pregs, addr & 0xff);

	// pr_info("\nDPHY_TEST_CTRL1:0x%x",REG32(DPHY_TEST_CTRL1));
	return (((*pregs->dphy_test_ctrl1) & 0xff00) >> 8);
}

static void dphyTst_setCfg_lanes(struct dphy_regs *pregs, int csi_speed,
				 int lane_num)
{
	unsigned char val;

	dphy_reset(pregs);

	(*pregs->dphy_shutdownz) = 0;
	pr_info("\nDPHY_SHUTDOWNZ(40) = %d", (*pregs->dphy_shutdownz));
	if (csi_speed <= 1500) {
		dphy_writeReg(pregs, 0x1, 0x20);
		if (csi_speed == 400)
			dphy_writeReg(pregs, 0x2, 0x05);
		else if (csi_speed == 800)
			dphy_writeReg(pregs, 0x2, 0x09);
		else if (csi_speed == 1200)
			dphy_writeReg(pregs, 0x2, 0x0b);
		dphy_writeReg(pregs, 0xe5, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x10);

		dphy_writeReg(pregs, 0x1ac, 0x4b);

		dphy_writeReg(pregs, 0xe2, 0xcc);
		dphy_writeReg(pregs, 0xe3, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x11);

		dphy_writeReg(pregs, 0x60a, 0xcc);
		dphy_writeReg(pregs, 0x60b, 0x1);
		dphy_writeReg(pregs, 0x60c, 0x1);

		dphy_writeReg(pregs, 0x80a, 0xcc);
		dphy_writeReg(pregs, 0x80b, 0x1);
		dphy_writeReg(pregs, 0x80c, 0x1);

		dphy_writeReg(pregs, 0x8, 0x38);

		dphy_writeReg(pregs, 0x307, 0x80);

		dphy_writeReg(pregs, 0x308, 0x10);
	} else if (csi_speed == 1600) {
		dphy_writeReg(pregs, 0x1, 0x20);
		dphy_writeReg(pregs, 0x2, 0x0d);
		dphy_writeReg(pregs, 0xe5, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x10);
		dphy_writeReg(pregs, 0x1ac, 0x4b);
		dphy_writeReg(pregs, 0xe2, 0x27);
		dphy_writeReg(pregs, 0xe3, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x1);
		dphy_writeReg(pregs, 0x307, 0x80);
		dphy_writeReg(pregs, 0x308, 0x10);
	} else if (csi_speed == 2500) {
		dphy_writeReg(pregs, 0xe5, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x10);
		dphy_writeReg(pregs, 0x1ac, 0x4b);
		dphy_writeReg(pregs, 0xe2, 0x27);
		dphy_writeReg(pregs, 0xe3, 0x1);
		dphy_writeReg(pregs, 0xe4, 0x1);
		dphy_writeReg(pregs, 0x307, 0x80);
		dphy_writeReg(pregs, 0x308, 0x10);
	}
	val = dphy_readReg(pregs, 0Xe5);
	pr_info("\nreg e5 value is 0x%x", val);
	val = dphy_readReg(pregs, 0X1ac);
	pr_info("\nreg 1ac value is 0x%x", val);
	val = dphy_readReg(pregs, 0Xe4);
	pr_info("nreg e4 value is 0x%x", val);
	val = dphy_readReg(pregs, 8);
	pr_info("\nreg 8 value is 0x%x", val);

	/*this register config MIPI lane number,O,1,2,3*/
	(*pregs->dphy_n_lanes) = (lane_num - 1);
	pr_info("\nDPHY_N_LANES(4) = %d(ENABLE RX)", (*pregs->dphy_n_lanes));

	(*pregs->mipi_test) |= 0x003c0000;
	pr_info("\nforce rxmode = 0x%x", (*pregs->mipi_test));
}

static void dphyTst_release(struct dphy_regs *pregs)
{
	uint8_t timeout;
	(*pregs->dphy_shutdownz) = 1;
	pr_err("\nDPHY_SHUTDOWNZ(40) = %d", (*pregs->dphy_shutdownz));

	(*pregs->dphy_rstz) = 1;
	pr_err("\nDPHY_RSTZ(44) = %d", (*pregs->dphy_rstz));

	for (timeout = 0; timeout < 20; timeout++) {
		if ((dphy_readReg(pregs, 0x0c9) & 0x10) != 0x10)
			usleep_range(1000, 2000);
		else
			break;
	}
	if (timeout == 20)
		pr_err("%s timeout", __func__);
	// while((dphy_readReg(index,0x0c9) & 0x10) != 0x10);
	//(*pregs->mipi_test) &= 0xfffffffe;
	pr_err("\ndphy0 enable done.");
}

static void dphyTst_release_2lane(struct dphy_regs *pregs)
{
	uint8_t timeout;
	(*pregs->dphy_shutdownz) = 1;
	pr_err("\nDPHY_SHUTDOWNZ(40) = %d", (*pregs->dphy_shutdownz));

	(*pregs->dphy_rstz) = 1;
	pr_err("\nDPHY_RSTZ(44) = %d", (*pregs->dphy_rstz));

	for (timeout = 0; timeout < 20; timeout++) {
		if (((*pregs->dphy_stop_state) & 0x010003) != 0x010003)
			usleep_range(1000, 2000);
		else
			break;
	}
	if (timeout == 20)
		pr_err("dphyTst_release timeout");
	pr_err("\nmipi3 dphy enable done.");

	(*pregs->csi2_resetz) = 1;
}

static void dphyTst_release_1_4lane(struct dphy_regs *pregs, uint8_t index)
{
	/*MIPI 0 1 used*/
	//(*pregs->MIPI_TEST) |= 0x8;
	// pr_info("\nDPHY_1_SHUTDOWNZ = %x",(*pregs->MIPI_TEST));
	/*MIPI2 used*/
	uint8_t timeout;
	(*pregs->mipi_test) |= 0x4;
	(*pregs->mipi_test) |= 0x8;

	pr_info("\nDPHY_1_RSTZ = %x", (*pregs->mipi_test));

	for (timeout = 0; timeout < 20; timeout++) {
		// if ((dphy_readReg(pregs, 0x0c9) & 0x10) != 0x10)
		if (((*pregs->dphy_stop_state) & 0x01000f) != 0x01000f)
			usleep_range(1000, 2000);
		else
			break;
	}
	if (timeout == 20)
		pr_info("%s timeout", __func__);
	// wait dphy enter stopstate
	// while(((*pregs->DPHY_STOPSTATE)& 0x1000f) != 0x1000f );
	pr_info("\ndphy0 and dphy1 enter stopstate.");

	// release force_rxmode
	(*pregs->mipi_test) &= 0xffc3ffff;
	pr_info("\nrelease force rxmode = 0x%x", (*pregs->mipi_test));

	// release csi2_rstz
	(*pregs->csi2_resetz) = 1;
}

void dphy_config(int index, int speed, int lane_num)
{
	const uint32_t MIPI_CSI_PHY_BASE[] = { 0x30f00000, 0x31000000,
					       0x31100000, 0x31101000 };
	const uint32_t MIPI_TEST_PHY_BASE[] = { 0X30f00800, 0X31000800,
						0X31100800, 0X31100800 };
	struct dphy_regs dphy_regs;

	pr_info("Enter %s\n", __func__);
	// dphy_regs.base            = ioremap(MIPI_CSI_PHY_BASE[index]  +
	// 0x00,4);
	dphy_regs.dphy_n_lanes = ioremap(MIPI_CSI_PHY_BASE[index] + 0x04, 4);
	dphy_regs.csi2_resetz = ioremap(MIPI_CSI_PHY_BASE[index] + 0x08, 4);
	dphy_regs.dphy_shutdownz = ioremap(MIPI_CSI_PHY_BASE[index] + 0x40, 4);
	dphy_regs.dphy_rstz = ioremap(MIPI_CSI_PHY_BASE[index] + 0x44, 4);
	dphy_regs.dphy_stop_state = ioremap(MIPI_CSI_PHY_BASE[index] + 0x4c, 4);
	dphy_regs.dphy_test_ctrl0 = ioremap(MIPI_CSI_PHY_BASE[index] + 0x50, 4);
	dphy_regs.dphy_test_ctrl1 = ioremap(MIPI_CSI_PHY_BASE[index] + 0x54, 4);
	dphy_regs.mipi_test = ioremap(MIPI_TEST_PHY_BASE[index] + 0x00, 4);
	dphy_regs.mipi_mode = ioremap(MIPI_TEST_PHY_BASE[index] + 0x3c, 4);

	if (!dphy_regs.dphy_n_lanes) {
		pr_err("dphy_n_lanes ioremap error!\n");
		return;
	}
	if (!dphy_regs.csi2_resetz) {
		pr_err("csi2_resetz ioremap error!\n");
		return;
	}
	if (!dphy_regs.dphy_shutdownz) {
		pr_err("dphy_shutdownz ioremap error!\n");
		return;
	}
	if (!dphy_regs.dphy_rstz) {
		pr_err("dphy_rstz ioremap error!\n");
		return;
	}
	if (!dphy_regs.dphy_test_ctrl0) {
		pr_err("dphy_test_ctrl0 ioremap error!\n");
		return;
	}
	if (!dphy_regs.dphy_test_ctrl1) {
		pr_err("dphy_test_ctrl1 ioremap error!\n");
		return;
	}
	if (!dphy_regs.mipi_test) {
		pr_err("mipi_test ioremap error!\n");
		return;
	}

	if (index == 2 || index == 3)
		*dphy_regs.mipi_mode = 1;

	if (index == 3) {
		pr_err("dphyTst_setCfg_lanes\n");
		dphyTst_setCfg_lanes(&dphy_regs, speed, lane_num);
		pr_err("dphyTst_release\n");
		dphyTst_release_2lane(&dphy_regs);
		pr_err("dphyTst_release_2lane\n finish");

	} else {
		pr_err("dphyTst_setCfg_lanes\n");
		dphyTst_setCfg_lanes(&dphy_regs, speed, lane_num);
		pr_err("dphyTst_release\n");
		dphyTst_release(&dphy_regs);
		pr_err("dphyTst_release_1_4lane\n");
		dphyTst_release_1_4lane(&dphy_regs, index);
		pr_err("dphyTst_release_1_4lane finish\n");
	}

	// iounmap(dphy_regs.base);
	iounmap(dphy_regs.dphy_n_lanes);
	iounmap(dphy_regs.csi2_resetz);
	iounmap(dphy_regs.dphy_shutdownz);
	iounmap(dphy_regs.dphy_rstz);
	iounmap(dphy_regs.dphy_stop_state);
	iounmap(dphy_regs.dphy_test_ctrl0);
	iounmap(dphy_regs.dphy_test_ctrl1);
	iounmap(dphy_regs.mipi_test);
	iounmap(dphy_regs.mipi_mode);
}

// SPDX-License-Identifier: GPL-2.0

/*
 * CSI DPHY driver for BST
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
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

#include "cam_entity.h"
#include "csi_dphy.h"
#include "isp_core.h"

uint32_t MIPI_CSI_BASE_PHY[3] = { 0x30f00000, 0x31000000, 0x31100000 };
uint32_t MIPI_TEST_PHY[3];
uint32_t *MIPI_CSI_BASE[3];
uint32_t *MIPI_TEST[3];
uint32_t *DPHY_N_LANES[3];
uint32_t *CSI2_RESETZ[3];
uint32_t *DPHY_SHUTDOWNZ[3];
uint32_t *DPHY_RSTZ[3];
uint32_t *DPHY_TEST_CTRL0[3];
uint32_t *DPHY_TEST_CTRL1[3];
uint32_t *DPHY_STOPSTATE[3];
uint32_t *MIPI2_TIME_MONITOR;

void dphy_reset(uint8_t index)
{
	REG32(DPHY_TEST_CTRL0[index]) = BIT_SET(Test0Clr);
	udelay(100);
	REG32(DPHY_TEST_CTRL0[index]) = BIT_CLR_ALL;
	udelay(100);
}

void dphy_write4BitMSB(uint8_t index, unsigned char valMsb)
{
	// a. Ensure that Test0Clk and Test1En is set to low
	REG32(DPHY_TEST_CTRL0[index]) = BIT_CLR_ALL;
	REG32(DPHY_TEST_CTRL1[index]) = BIT_CLR_ALL;

	// b. Set Test1En to high
	REG32(DPHY_TEST_CTRL1[index]) |= Test1En;

	// c. Set Test0Clk to high
	REG32(DPHY_TEST_CTRL0[index]) |= Test0Clk;

	// d. Place 0x00 in testdin
	REG32(DPHY_TEST_CTRL1[index]) |= Test1DataIn(0);

	// e. Set Test0Clk to low (with the falling edge on Test0Clk, the
	// testdin signal content is latched internally)
	REG32(DPHY_TEST_CTRL0[index]) &= ~Test0Clk;

	// f. Set Test1En to low
	REG32(DPHY_TEST_CTRL1[index]) &= ~Test1En;

	// g. Place the 8-bit word corresponding to the testcode MSBs in testdin
	REG32(DPHY_TEST_CTRL1[index]) |= Test1DataIn(valMsb);

	// h. Set Test0Clk to high
	REG32(DPHY_TEST_CTRL0[index]) |= Test0Clk;
}

void dphy_write8BitLSB(uint8_t index, unsigned char valLsb)
{
	// a. Set testclk to low
	REG32(DPHY_TEST_CTRL0[index]) &= ~Test0Clk;

	// b. Set testen to high
	REG32(DPHY_TEST_CTRL1[index]) |= Test1En;

	// c. Set testclk to high
	REG32(DPHY_TEST_CTRL0[index]) |= Test0Clk;

	// d. Place the 8-bit word test data in testdin
	// tmp = REG32(DPHY_TEST_CTRL1);
	// tmp &= 0xffffff00;
	// tmp |=  Test1DataIn(valLsb);
	// REG32(DPHY_TEST_CTRL1) =  tmp;
	REG32(DPHY_TEST_CTRL1[index]) = Test1En | (valLsb & 0xff);

	// e. Set testclk to low (with the falling edge on testclk, the testdin
	// signal content is latched internally)
	REG32(DPHY_TEST_CTRL0[index]) &= ~Test0Clk;

	// f. Set testen to low
	REG32(DPHY_TEST_CTRL1[index]) &= ~Test1En;
}

void dphy_write8BitData(uint8_t index, unsigned char data)
{
	// a. Place the 8-bit word corresponding to the page offset in testdin
	// REG32(DPHY_TEST_CTRL1) |=  Test1DataIn(data);
	REG32(DPHY_TEST_CTRL1[index]) = Test1DataIn(data);

	// b. Set testclk to high (test data is programmed internally)
	REG32(DPHY_TEST_CTRL0[index]) |= Test0Clk;

	REG32(DPHY_TEST_CTRL0[index]) &= ~Test0Clk;
}

void dphy_writeReg(uint8_t index, unsigned short addr, unsigned char val)
{
	dphy_write4BitMSB(index, (addr & 0xf00) >> 8);
	dphy_write8BitLSB(index, addr & 0xff);
	dphy_write8BitData(index, val);
}

unsigned char dphy_readReg(uint8_t index, unsigned short addr)
{
	dphy_write4BitMSB(index, (addr & 0xf00) >> 8);
	dphy_write8BitLSB(index, addr & 0xff);

	// pr_info("DPHY_TEST_CTRL1:0x%x\n",REG32(DPHY_TEST_CTRL1));
	return ((REG32(DPHY_TEST_CTRL1[index]) & 0xff00) >> 8);
}

void dphyTst_setCfg_lanes(int csi_speed, uint8_t index, int lane_num)
{
	unsigned char val;

	dphy_reset(index);

	REG32(DPHY_SHUTDOWNZ[index]) = 0;
	pr_info("DPHY_SHUTDOWNZ(40) = %d\n", REG32(DPHY_SHUTDOWNZ[index]));
	if (csi_speed <= 1500) {
		dphy_writeReg(index, 0x1, 0x20);
		if (csi_speed == 400)
			dphy_writeReg(index, 0x2, 0x05);
		else if (csi_speed == 800)
			dphy_writeReg(index, 0x2, 0x09);
		else if (csi_speed == 1200)
			dphy_writeReg(index, 0x2, 0x0b);
		dphy_writeReg(index, 0xe5, 0x1);
		dphy_writeReg(index, 0xe4, 0x10);

		dphy_writeReg(index, 0x1ac, 0x4b);

		dphy_writeReg(index, 0xe2, 0xcc);
		dphy_writeReg(index, 0xe3, 0x1);
		dphy_writeReg(index, 0xe4, 0x11);

		dphy_writeReg(index, 0x60a, 0xcc);
		dphy_writeReg(index, 0x60b, 0x1);
		dphy_writeReg(index, 0x60c, 0x1);

		dphy_writeReg(index, 0x80a, 0xcc);
		dphy_writeReg(index, 0x80b, 0x1);
		dphy_writeReg(index, 0x80c, 0x1);

		dphy_writeReg(index, 0x8, 0x38);

		dphy_writeReg(index, 0x307, 0x80);

		dphy_writeReg(index, 0x308, 0x10);
	} else if (csi_speed == 1600) {
		dphy_writeReg(index, 0x1, 0x20);
		dphy_writeReg(index, 0x2, 0x0d);
		dphy_writeReg(index, 0xe5, 0x1);
		dphy_writeReg(index, 0xe4, 0x10);
		dphy_writeReg(index, 0x1ac, 0x4b);
		dphy_writeReg(index, 0xe2, 0x27);
		dphy_writeReg(index, 0xe3, 0x1);
		dphy_writeReg(index, 0xe4, 0x1);
		dphy_writeReg(index, 0x307, 0x80);
		dphy_writeReg(index, 0x308, 0x10);
	}
	val = dphy_readReg(index, 0Xe5);
	val = dphy_readReg(index, 0X1ac);
	val = dphy_readReg(index, 0Xe4);
	val = dphy_readReg(index, 8);

	/*this register config MIPI lane number,O,1,2,3*/
	REG32(DPHY_N_LANES[index]) = (lane_num - 1);
	pr_info("DPHY_N_LANES(4) = %d(ENABLE RX)\n",
		REG32(DPHY_N_LANES[index]));

	REG32(MIPI_TEST[index]) |= 0x003c0000;
	pr_info("force rxmode = 0x%x\n", REG32(MIPI_TEST[index]));
}

void dphyTst_release(struct bst_csi_device *csi_dev, uint8_t index)
{
	REG32(DPHY_SHUTDOWNZ[index]) = 1;
	pr_info("DPHY_SHUTDOWNZ(40) = %d\n", REG32(DPHY_SHUTDOWNZ[index]));

	REG32(DPHY_RSTZ[index]) = 1;
	pr_info("DPHY_RSTZ(44) = %d\n", REG32(DPHY_RSTZ[index]));

	if (of_device_is_compatible(csi_dev->dev->of_node, "bst,a1000_csi2")) {
		// dev_info(csi_dev->dev, "Wait dphy0 enable\n");
		// wait dphy0 enable
		REG32(MIPI_TEST[index]) |= 0x1;
		REG32(MIPI_TEST[index]) &= 0xfffffffe;
	}
}

void dphyTst_release_1_4lane(struct bst_csi_device *csi_dev, uint8_t index)
{
	/*MIPI 0 1 used*/
	// REG32(MIPI_TEST) |= 0x8;
	// pr_info("\nDPHY_1_SHUTDOWNZ = %x",REG32(MIPI_TEST));
	/*MIPI2 used*/
	uint8_t timeout;

	if (of_device_is_compatible(csi_dev->dev->of_node, "bst,a1000_csi2")) {
		dev_info(csi_dev->dev, "%s\n", __func__);
		if (index == 2)
			REG32(MIPI2_TIME_MONITOR) |= 0x4;
		else
			REG32(MIPI_TEST[index]) |= 0x8;

		REG32(MIPI_TEST[index]) |= 0x4;
		pr_info("DPHY_1_RSTZ = %x\n", REG32(MIPI_TEST[index]));
	}

	if (of_device_is_compatible(csi_dev->dev->of_node, "bst,a1000b_csi2")) {
		dev_info(csi_dev->dev, "%s\n", __func__);
		REG32(MIPI_TEST[index]) |= 0x4;
		REG32(MIPI_TEST[index]) |= 0x8;
		pr_info("DPHY_1_RSTZ = %x\n", REG32(MIPI_TEST[index]));

		// wait dphy enter stopstate
		for (timeout = 0; timeout < 50; timeout++) {
			if ((REG32(DPHY_STOPSTATE[index]) & 0x1000f) !=
			    0x1000f) {
				usleep_range(1000, 2000);
			} else {
				break;
			}
		}

		if (timeout == 20)
			pr_err("%s timeout\n", __func__);
	}
	// pr_info("dphy0 and dphy1 enter stopstate.\n");

	// release force_rxmode
	REG32(MIPI_TEST[index]) &= 0xffc3ffff;
	pr_info("release force rxmode = 0x%x\n", REG32(MIPI_TEST[index]));

	// release csi2_rstz
	REG32(CSI2_RESETZ[index]) = 1;
}

int csi_dphy_config_lanes(struct bst_csi_device *csi_dev, int speed, int index,
			  int lane_num)
{
	if (of_device_is_compatible(csi_dev->dev->of_node, "bst,a1000_csi2")) {
		MIPI_TEST_PHY[0] = 0x33000048;
		MIPI_TEST_PHY[1] = 0x3300004c;
		MIPI_TEST_PHY[2] = 0x33000050;
	} else {
		MIPI_TEST_PHY[0] = 0x30f00800;
		MIPI_TEST_PHY[1] = 0x31000800;
		MIPI_TEST_PHY[2] = 0x31000800;
	}

	MIPI_CSI_BASE[index] = (uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index], 4);
	if (MIPI_CSI_BASE[index] == NULL) {
		pr_err("IOREMAP MIPI_CSI_BASE_PHY%x FAILED\n",
		       MIPI_CSI_BASE_PHY[index]);
	}
	MIPI_TEST[index] = (uint32_t *)ioremap(MIPI_TEST_PHY[index], 4);
	if (MIPI_TEST[index] == NULL) {
		pr_err("IOREMAP MIPI_TEST_PHY%x FAILED\n",
		       MIPI_TEST_PHY[index]);
	}

	DPHY_N_LANES[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x4, 4);

	CSI2_RESETZ[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x8, 4);

	DPHY_SHUTDOWNZ[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x40, 4);

	DPHY_RSTZ[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x44, 4);

	DPHY_TEST_CTRL0[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x50, 4);

	DPHY_TEST_CTRL1[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x54, 4);

	DPHY_STOPSTATE[index] =
		(uint32_t *)ioremap(MIPI_CSI_BASE_PHY[index] + 0x4c, 4);

	if (of_device_is_compatible(csi_dev->dev->of_node, "bst,a1000_csi2")) {
		if (index == 2) {
			MIPI2_TIME_MONITOR =
				(uint32_t *)ioremap(MIPI2_TIME_MONITOR_PHY, 4);
			if (MIPI2_TIME_MONITOR == NULL) {
				pr_err("IOREMAP DPHY_STOPSTATE_PHY%x FAILED\n",
				       MIPI2_TIME_MONITOR_PHY);
			}
		}
	}

	dphyTst_setCfg_lanes(speed, index, lane_num);
	dphyTst_release(csi_dev, index);
	dphyTst_release_1_4lane(csi_dev, index);

	return 0;
}

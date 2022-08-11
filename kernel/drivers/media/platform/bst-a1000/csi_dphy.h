/* SPDX-License-Identifier: GPL-2.0 */

/*
 * CSI DPHY definitions for BST
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

#ifndef __BST_CSI_DPHY_H__
#define __BST_CSI_DPHY_H__

#include "csi.h"

#define MIPI2_TIME_MONITOR_PHY 0x33000078

#define Test0Clk	(1 << 1)
#define Test0Clr	(1 << 0)
#define Test1En		(1 << 16)
#define Test1DataOut(x) ((x & 0xff) << 8)
#define Test1DataIn(x)	(x & 0xff)

#define BIT_SET(x)  (x)
#define BIT_CLR(x)  (~x)
#define BIT_CLR_ALL 0

#define REG32(x) (*x)

int csi_dphy_config_lanes(struct bst_csi_device *csi_dev, int csi_speed,
			  int index, int lane_num);
void dphy_config(int index, int speed, int lane_num);

#endif // __BST_CSI_DPHY_H__

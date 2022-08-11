/* SPDX-License-Identifier: GPL-2.0 */

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

#ifndef __BST_DRM_DEV_H__
#define __BST_DRM_DEV_H__

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include "bst_drm_drv.h"

//#define CTRL_REG3 0x33002048
#define PLL_DISPLAY_REG0 0x33002048
#define PLL_DISPLAY_REG1 0x3300204c
#define PLL_DISPLAY_REG2 0x33002050
#define PLL_DISPLAY_REG3 0x33002054

// FBDIV, bit27 ~ 16
#define FBDIV_MASK     0xf000ffff
#define FBDIV_SHIFT    16
// DIV1, bit11 ~ 9, DIV2, bit8 ~ 6
// DIV, bit11 ~ 6
#define POSTDIV_MASK   0xfffff03f
#define POSTDIV_SHIFT  6
// DIV1, bit11 ~ 9,
#define POSTDIV1_MASK  0xfffff1ff
#define POSTDIV1_SHIFT 9
// DIV2, bit8 ~ 6
#define POSTDIV2_MASK  0xfffffe3f
#define POSTDIV2_SHIFT 6
// PLLEN, bit5
#define PLLEN_MASK     0xffffffdf
#define PLLEN_SHIFT    5
// REFDIV, bit29 ~ 24
#define REFDIV_MASK    0xa0ffffff
#define REFDIV_SHIFT   24

int vout_pclk_init(struct vop_device *pvop);
void vout_pclk_enable(int val);

#endif

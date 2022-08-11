/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VOUT timing parser declarations for BST
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

#ifndef __BST_VOUT_TIMING_PARSE_H__
#define __BST_VOUT_TIMING_PARSE_H__

#include "bst_drm_drv.h"
#include <video/display_timing.h>

int bst_get_videomode_from_edid(struct vop_device *pvop, int hsize, int vsize,
				int fresh);
void bst_get_videomode_from_timing(struct display_timing *timing,
				   struct vop_device *pvop);
void bst_get_output_format(struct vop_device *pvop, const char *format);
int bst_vout_display_parse_dt(struct vop_device *pvop);

#endif

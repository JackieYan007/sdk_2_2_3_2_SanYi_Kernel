// SPDX-License-Identifier: GPL-2.0

/*
 * VOUT timing parser for BST
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

#include <drm/drm_edid.h>

#include "bst_drm_drv.h"

#include <linux/coreip/bst_vsp_msg.h>
#include <video/of_display_timing.h>

void bst_get_videomode_from_timing(struct display_timing *timing,
				   struct vop_device *pvop)
{
	pvop->display_param->pixelclk = timing->pixelclock.typ;
	pvop->display_param->hactive = timing->hactive.typ;
	pvop->display_param->vactive = timing->vactive.typ;

	pvop->display_param->hfront_porch = timing->hfront_porch.typ;
	pvop->display_param->hback_porch = timing->hback_porch.typ;
	pvop->display_param->hsync_len = timing->hsync_len.typ;

	pvop->display_param->vfront_porch = timing->vfront_porch.typ;
	pvop->display_param->vback_porch = timing->vback_porch.typ;
	pvop->display_param->vsync_len = timing->vsync_len.typ;

	if (timing->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		pvop->display_param->hsync_active = 1;
	else
		pvop->display_param->hsync_active = 0;

	if (timing->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		pvop->display_param->vsync_active = 1;
	else
		pvop->display_param->vsync_active = 0;
}

int bst_get_videomode_from_edid(struct vop_device *pvop, int hsize, int vsize,
				int fresh)
{
	struct drm_display_mode *mode = NULL;

	mode = drm_mode_find_dmt(pvop->drm, hsize, vsize, fresh, false);
	if (mode == NULL) {
		dev_err(pvop->dev, "invalid  %s DT property\n",
			"output-hsize or output-vsize");
		return -ENODEV;
	}

	pvop->display_param->pixelclk = mode->clock * 1000;
	pvop->display_param->hactive = mode->hdisplay;
	pvop->display_param->vactive = mode->vdisplay;

	pvop->display_param->hfront_porch = mode->hsync_start - mode->hdisplay;
	pvop->display_param->hback_porch = mode->htotal - mode->hsync_end;
	pvop->display_param->hsync_len = mode->hsync_end - mode->hsync_start;

	pvop->display_param->vfront_porch = mode->vsync_start - mode->vdisplay;
	pvop->display_param->vback_porch = mode->vtotal - mode->vsync_end;
	pvop->display_param->vsync_len = mode->vsync_end - mode->vsync_start;

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		pvop->display_param->hsync_active = 1;
	else
		pvop->display_param->hsync_active = 0;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		pvop->display_param->vsync_active = 1;
	else
		pvop->display_param->vsync_active = 0;

	return 0;
}

void bst_get_output_format(struct vop_device *pvop, const char *format)
{
	if (strcmp(format, "HDMI_YUV444") == 0)
		pvop->display_param->output_mode = 0x10;
	else if (strcmp(format, "HDMI_RGB") == 0)
		pvop->display_param->output_mode = 0x11;
	else if (strcmp(format, "RGB888") == 0)
		pvop->display_param->output_mode = 0x11;
	else if (strcmp(format, "D601_8") == 0)
		pvop->display_param->output_mode = 7;
	else {
		dev_err(pvop->dev,
			"invalid %s DT property, Use RGB888 to default output_format\n",
			"output-format");
		pvop->display_param->output_mode = 0x11;
	}
}

int bst_vout_display_parse_dt(struct vop_device *pvop)
{
	int ret;
	struct device_node *np = pvop->dev->of_node;
	struct display_timing timing;
	const char *format;
	int fresh;
	int hsize, vsize;

	ret = of_property_read_string(np, "output-format", &format);
	if (ret < 0) {
		dev_err(pvop->dev, "%pOF: invalid or missing %s DT property\n",
			np, "output-format");
		return -ENODEV;
	}

	if (strncmp(format, "HDMI", 4) == 0) {
		pvop->hdmi_flags = 1;
		ret = of_property_read_u32(np, "output-hsize", &hsize);
		if (ret < 0) {
			dev_err(pvop->dev,
				"%pOF: invalid or missing %s DT property\n", np,
				"output-hsize");
			return -ENODEV;
		}
		pvop->hsize = hsize;

		ret = of_property_read_u32(np, "output-vsize", &vsize);
		if (ret < 0) {
			dev_err(pvop->dev,
				"%pOF: invalid or missing %s DT property\n", np,
				"output-vsize");
			return -ENODEV;
		}
		pvop->vsize = vsize;

		ret = of_property_read_u32(np, "output-fresh", &fresh);
		if (ret < 0) {
			dev_err(pvop->dev,
				"%pOF: invalid or missing %s DT property\n", np,
				"output-fresh");
			return -ENODEV;
		}
		pvop->fresh = fresh;
		pvop->fresh_old = fresh;

		bst_get_videomode_from_edid(pvop, hsize, vsize, fresh);

	} else {
		// pvop->display_param->output_mode = 0x11;
		ret = of_get_display_timing(np, "panel-timing", &timing);
		if (ret < 0) {
			dev_err(pvop->dev,
				"%pOF: problems parsing panel-timing (%d)\n",
				np, ret);
			return ret;
		}

		videomode_from_timing(&timing, &pvop->video_mode);

		bst_get_videomode_from_timing(&timing, pvop);
	}

	// get output_format
	bst_get_output_format(pvop, format);

	// fix osd to ARGB8888 format for weston
	pvop->display_param->osd_format = OSD_32BIT_8888_ARGB;

	return 0;
}

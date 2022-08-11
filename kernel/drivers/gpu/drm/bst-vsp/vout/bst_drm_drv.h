/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VOUT DRM driver definitions for BST
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

#ifndef __BST_DRM_DRV_H__
#define __BST_DRM_DRV_H__

#include <drm/drm.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_vblank.h>
#include <linux/clk.h>
#include <linux/coreip/bst_vsp_msg.h>
#include <linux/hrtimer.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define XRES_MIN 32
#define YRES_MIN 32

#define XRES_DEF 1920
#define YRES_DEF 1080

#define XRES_MAX 8192
#define YRES_MAX 8192

#define VOUT_SIZE_1080P 1080
#define VOUT_SIZE_720P	720

#define NV12_SIZE_1080P (1920 * 1080 * 3 / 2)
#define Y_SIZE_1080P	(1920 * 1080)

#define NV12_SIZE_720P (1280 * 720 * 3 / 2)
#define Y_SIZE_720P    (1280 * 720)

/*vsp device info*/
#define DRIVER_NAME  "bst-vsp"
#define DRIVER_DESC  "BST SOC DRM"
#define DRIVER_DATE  "20200416"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

static const u32 primary_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_XRGB8888,
};

static const u32 overlay_formats[] = {
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_ARGB8888,
};

struct vsp_output {
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct hrtimer vblank_hrtimer;
	ktime_t period_ns;
	struct drm_pending_vblank_event *event;
};

struct vout_receiver_ops {
	int (*reset)(void);
};

struct vop_device {
	struct drm_device *drm;
	struct vsp_output output;
	struct device *dev;
	struct vout_receiver_ops *ops;
	dma_addr_t primary_default_addr;
	dma_addr_t primary_paddr;
	dma_addr_t osd_paddr;
	u8 vout_dev_init;
	int enable_primary;

	struct clk *clock;
	dma_addr_t display_param_phyaddr;
	struct VoutCfg_t *display_param;
	int hsize;
	int vsize;
	int fresh;
	int fresh_old;
	int hdmi_flags;
	struct videomode video_mode; // for lvds
	int primary_flags;
	int osd_flags;
	int weston_flags;
};

#define drm_crtc_to_vsp_output(target) \
	container_of(target, struct vsp_output, crtc)

#define drm_device_to_vop_device(target) \
	container_of(target, struct vop_device, drm)

extern struct completion vout_completion;

/* CRTC */
int vsp_output_init(struct vop_device *vspdev);

bool vsp_get_vblank_timestamp(struct drm_device *dev, unsigned int pipe,
			      int *max_error, ktime_t *vblank_time,
			      bool in_vblank_irq);

int of_drm_dev_parse(struct platform_device *pdev, struct vop_device *pvop);

int vsp_debug_sysfs_init(struct vop_device *pvop);

int vout_register_receiver_ops(struct vout_receiver_ops *recv_ops);

int vout_receiver_reset(void);

#endif /* __BST_DRM_DRV_H__ */

// SPDX-License-Identifier: GPL-2.0

/*
 * VOUT DRM operations for BST
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

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <linux/completion.h>

#include "bst_drm_drv.h"
#include "bst_vout_dev.h"
#include "bst_vout_msg.h"
#include "bst_vout_timing_parse.h"

#include "../bst_ipc_drv.h"

DECLARE_COMPLETION(vout_completion); // sync fw

static enum hrtimer_restart vsp_vblank_simulate(struct hrtimer *timer)
{
	struct vsp_output *output =
		container_of(timer, struct vsp_output, vblank_hrtimer);
	struct drm_crtc *crtc = &output->crtc;
	int ret_overrun;
	bool ret;

	ret = drm_crtc_handle_vblank(crtc);
	if (!ret)
		DRM_ERROR("vsp failure on handling vblank");

	ret_overrun =
		hrtimer_forward_now(&output->vblank_hrtimer, output->period_ns);

	return HRTIMER_RESTART;
}

static int vsp_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	unsigned int pipe = drm_crtc_index(crtc);
	struct drm_vblank_crtc *vblank = &dev->vblank[pipe];
	struct vsp_output *out = drm_crtc_to_vsp_output(crtc);

	drm_calc_timestamping_constants(crtc, &crtc->mode);

	hrtimer_init(&out->vblank_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	out->vblank_hrtimer.function = &vsp_vblank_simulate;
	out->period_ns = ktime_set(0, vblank->framedur_ns);
	hrtimer_start(&out->vblank_hrtimer, out->period_ns, HRTIMER_MODE_REL);

	return 0;
}

static void vsp_disable_vblank(struct drm_crtc *crtc)
{
	struct vsp_output *out = drm_crtc_to_vsp_output(crtc);

	hrtimer_cancel(&out->vblank_hrtimer);
}

bool vsp_get_vblank_timestamp(struct drm_device *dev, unsigned int pipe,
			      int *max_error, ktime_t *vblank_time,
			      bool in_vblank_irq)
{
	struct vop_device *vspdev = dev->dev_private;
	struct vsp_output *output = &vspdev->output;

	*vblank_time = output->vblank_hrtimer.node.expires;

	if (!in_vblank_irq)
		*vblank_time -= output->period_ns;

	return true;
}

static const struct drm_crtc_funcs vsp_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = vsp_enable_vblank,
	.disable_vblank = vsp_disable_vblank,
};

static void vsp_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_state)
{
	struct vop_device *pvop;
	unsigned long clk;

	pvop = crtc->dev->dev_private;

	clk = clk_round_rate(pvop->clock, pvop->display_param->pixelclk);

	pr_info("VSP VOUT clk_round_rate is %ld\n", clk);
	clk_set_rate(pvop->clock, pvop->display_param->pixelclk);

	drm_crtc_vblank_on(crtc);
}

static void vsp_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_state)
{
	struct vop_device *pvop;

	drm_crtc_vblank_off(crtc);
	pvop = crtc->dev->dev_private;
	clk_disable_unprepare(pvop->clock);
}

static void vsp_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_crtc_state)
{
	unsigned long flags;

	if (crtc->state->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);

		if (drm_crtc_vblank_get(crtc) != 0)
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);

		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

		crtc->state->event = NULL;
	}
}

static const struct drm_crtc_helper_funcs vsp_crtc_helper_funcs = {
	.atomic_flush = vsp_crtc_atomic_flush,
	.atomic_enable = vsp_crtc_atomic_enable,
	.atomic_disable = vsp_crtc_atomic_disable,
};

int vsp_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
		  struct drm_plane *primary, struct drm_plane *overlay)
{
	int ret;

	ret = drm_crtc_init_with_planes(dev, crtc, primary, NULL,
					&vsp_crtc_funcs, NULL);
	if (ret) {
		DRM_ERROR("Failed to init CRTC\n");
		return ret;
	}

	drm_crtc_helper_add(crtc, &vsp_crtc_helper_funcs);

	return ret;
}

static const struct drm_plane_funcs vsp_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static void vsp_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = plane->state->fb;
	struct drm_gem_object *obj = fb->obj[0];
	struct drm_gem_cma_object *cma_obj;
	struct vop_device *pvop;
	static int combine_flag;

	pvop = obj->dev->dev_private;
	cma_obj = to_drm_gem_cma_obj(obj);

	if (pvop->primary_flags && (!pvop->osd_flags)) { // only primary
		pvop->primary_paddr = cma_obj->paddr;
		vsp_cmd_primary_flip(pvop);
	} else if (pvop->primary_flags && pvop->osd_flags) { // primary and osd
		if (plane->type) {
			combine_flag++;
			pvop->primary_paddr = cma_obj->paddr;
		} else {
			combine_flag++;
			pvop->osd_paddr = cma_obj->paddr;
		}
		if (combine_flag == 2) {
			vsp_cmd_primary_flip(pvop);
			combine_flag = 0;
		}
	} else if (!pvop->primary_flags && pvop->osd_flags) { // only osd for
							      // weston
		pvop->primary_paddr = pvop->primary_default_addr;
		pvop->osd_paddr = cma_obj->paddr;
		vsp_cmd_primary_flip(pvop);
	} else {
	}
}

static int vsp_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct drm_framebuffer *fb;
	struct drm_gem_object *obj;
	struct drm_gem_cma_object *cma_obj;
	struct vop_device *pvop;
	static int weston_flag;
	u32 src_x, src_y, src_w, src_h;

	DRM_DEBUG_DRIVER("\n");

	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;
	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;

	if (src_w != state->crtc_w || src_h != state->crtc_h) {
		DRM_ERROR("Scaling is not supported");
		return -EINVAL;
	}

	fb = plane->state->fb;
	if (!fb)
		return 0;

	obj = fb->obj[0];
	pvop = obj->dev->dev_private;
	if (!pvop)
		return 0;

	if (pvop->vout_dev_init > 1)
		return 0;

	cma_obj = to_drm_gem_cma_obj(obj);
	if (!cma_obj)
		return 0;

	if (!plane->type) { // osd
		pvop->osd_flags = 1;
		pvop->osd_paddr = cma_obj->paddr;
		pvop->primary_paddr = pvop->primary_default_addr;
	} else { // primary
		pvop->primary_flags = 1;
		if (fb->format->format != DRM_FORMAT_NV12) {
			weston_flag++;
			if (weston_flag == 2) {
				pvop->weston_flags = 1;
				pvop->primary_flags = 0;
				pvop->osd_flags = 1;
				weston_flag = 0;
				pvop->osd_paddr = cma_obj->paddr;
				pvop->primary_paddr =
					pvop->primary_default_addr;

			} else {
				pvop->primary_paddr = cma_obj->paddr;
			}
		} else {
			pvop->primary_paddr = cma_obj->paddr;
		}
	}

	if (pvop->vout_dev_init == 0) {
		vsp_cmd_primary_close();
		if (pvop->hdmi_flags) {
			if ((pvop->hsize != pvop->display_param->hactive) ||
			    (pvop->vsize != pvop->display_param->vactive) ||
			    (pvop->fresh != pvop->fresh_old)) {
				if (bst_get_videomode_from_edid(
					    pvop, pvop->hsize, pvop->vsize,
					    pvop->fresh))
					pr_info("set vout resolution failed, input hsize, vsize or fresh is invalid\n");
				else {
					pvop->fresh_old = pvop->fresh;
					// unsigned long clk =
					// clk_round_rate(pvop->clock,
					// pvop->display_param->pixelclk);
					clk_set_rate(
						pvop->clock,
						pvop->display_param->pixelclk);
				}
			}
		}
		vsp_cmd_display_timing_set(pvop->display_param_phyaddr);
		clk_prepare_enable(pvop->clock);
		// vsp_cmd_overlay_ctrl(1, pvop->display_param->hactive,
		// pvop->display_param->vactive); //enable osd
		// vout_pclk_enable(1); //enable pclk
		// vsp_cmd_overlay_region(pvop);
	} else if (pvop->vout_dev_init == 1) {
		if (pvop->osd_flags)
			vsp_cmd_overlay_ctrl(
				1, pvop->display_param->hactive,
				pvop->display_param->vactive); // enable osd
		else
			vsp_cmd_overlay_ctrl(
				0, pvop->display_param->hactive,
				pvop->display_param->vactive); // disable osd

		vsp_cmd_sync_mode_set(1);
		vsp_cmd_primary_flip(pvop);
		pr_info("send ipc to fw, primary_paddr = 0x%llx, osd_paddr = 0x%llx\n",
			pvop->primary_paddr, pvop->osd_paddr);
		// vsp_cmd_primary_start(pvop);

		wait_for_completion_timeout(&vout_completion,
					    msecs_to_jiffies(50));

		vout_receiver_reset();
	}
	pvop->vout_dev_init++;

	return 0;
}

static void vsp_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_plane_state *oldstate)
{
	struct vop_device *pvop;

	pvop = plane->dev->dev_private;

	if (plane->type) {
		if (pvop->weston_flags) {
			pvop->weston_flags = 0;
			pvop->osd_flags = 0;
		} else {
			pvop->primary_flags = 0;
		}
	} else
		pvop->osd_flags = 0;

	if (pvop->vout_dev_init > 1) {
		vsp_cmd_primary_close();
		// pvop->vout_dev_init = 0;
		// pvop->primary_flags = 0;
		// pvop->osd_flags = 0;
		// vout_pclk_enable(0); //disable pclk
	}
	if ((pvop->primary_flags == 0) && (pvop->osd_flags == 0))
		pvop->vout_dev_init = 0;

	DRM_DEBUG_DRIVER("CRTC:%d plane:%d\n", oldstate->crtc->base.id,
			 plane->base.id);
}

static const struct drm_plane_helper_funcs vsp_plane_helper_funcs = {
	.atomic_check = vsp_plane_atomic_check,
	.atomic_update = vsp_plane_atomic_update,
	.atomic_disable = vsp_plane_atomic_disable,
};

struct drm_plane *vsp_plane_create(struct vop_device *vspdev,
				   enum drm_plane_type type)
{
	struct drm_device *dev = vspdev->drm;
	struct drm_plane *plane;
	const u32 *formats;
	int ret, nformats;

	plane = kzalloc(sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	if (type == DRM_PLANE_TYPE_PRIMARY) {
		formats = primary_formats;
		nformats = ARRAY_SIZE(primary_formats);
	} else {
		formats = overlay_formats;
		nformats = ARRAY_SIZE(overlay_formats);
	}

	ret = drm_universal_plane_init(dev, plane, 1, &vsp_plane_funcs, formats,
				       nformats, NULL, type, NULL);
	if (ret) {
		kfree(plane);
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(plane, &vsp_plane_helper_funcs);
	DRM_INFO("plane:%d created\n", plane->base.id);

	return plane;
}

static void vsp_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs vsp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = vsp_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_encoder_funcs vsp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int vsp_conn_get_modes(struct drm_connector *connector)
{
	int count;
	struct vop_device *pvop;

	pvop = connector->dev->dev_private;

	count = drm_add_modes_noedid(connector, XRES_MAX, YRES_MAX);
	// drm_set_preferred_mode(connector, XRES_DEF, YRES_DEF);

	if (pvop->hdmi_flags)
		drm_set_preferred_mode(connector, XRES_DEF, YRES_DEF);
	else {
		// add display mode for lvds
		struct drm_display_mode *mode;

		mode = drm_mode_create(connector->dev);
		if (!mode)
			return 0;
		drm_display_mode_from_videomode(&pvop->video_mode, mode);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static const struct drm_connector_helper_funcs vsp_conn_helper_funcs = {
	.get_modes = vsp_conn_get_modes,
};

int vsp_output_init(struct vop_device *vspdev)
{
	struct vsp_output *output = &vspdev->output;
	struct drm_device *dev = vspdev->drm;
	struct drm_connector *connector = &output->connector;
	struct drm_encoder *encoder = &output->encoder;
	struct drm_crtc *crtc = &output->crtc;
	struct drm_plane *primary, *overlay = NULL;
	int ret;

	primary = vsp_plane_create(vspdev, DRM_PLANE_TYPE_PRIMARY);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	overlay = vsp_plane_create(vspdev, DRM_PLANE_TYPE_OVERLAY);
	if (IS_ERR(overlay))
		return PTR_ERR(overlay);

	ret = vsp_crtc_init(dev, crtc, primary, overlay);
	if (ret)
		goto err_crtc;

	ret = drm_connector_init(dev, connector, &vsp_connector_funcs,
				 DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret) {
		DRM_ERROR("Failed to init connector\n");
		goto err_connector;
	}

	drm_connector_helper_add(connector, &vsp_conn_helper_funcs);

	ret = drm_connector_register(connector);
	if (ret) {
		DRM_ERROR("Failed to register connector\n");
		goto err_connector_register;
	}

	ret = drm_encoder_init(dev, encoder, &vsp_encoder_funcs,
			       DRM_MODE_ENCODER_VIRTUAL, NULL);
	if (ret) {
		DRM_ERROR("Failed to init encoder\n");
		goto err_encoder;
	}
	encoder->possible_crtcs = 1;

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("Failed to attach connector to encoder\n");
		goto err_attach;
	}

	drm_mode_config_reset(dev);

	return 0;

err_attach:
	drm_encoder_cleanup(encoder);

err_encoder:
	drm_connector_unregister(connector);

err_connector_register:
	drm_connector_cleanup(connector);

err_connector:
	drm_crtc_cleanup(crtc);

err_crtc:
	drm_plane_cleanup(primary);
	drm_plane_cleanup(overlay);
	return ret;
}

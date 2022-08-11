// SPDX-License-Identifier: GPL-2.0

/*
 * VOUT DRM driver for BST
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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <linux/coreip/bst_vsp_msg.h>

#include "bst_drm_drv.h"
#include "bst_vout_dev.h"
#include "bst_vout_msg.h"
#include "bst_vout_timing_parse.h"

#include "../bst_ipc_drv.h"

static int drm_drv_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = vsp_fw_boot_done();
	if (ret)
		DRM_ERROR("VSP loading fw failed!\n");

	ret = drm_open(inode, filp);
	if (ret)
		DRM_ERROR("DRM open failed\n");

	return ret;
}

static const struct file_operations vsp_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_drv_open,
	.mmap = drm_gem_cma_mmap,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = no_llseek,
	.release = drm_release,
};

static struct drm_driver vsp_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_GEM,
	//.release = vsp_release,
	.fops = &vsp_driver_fops,

	.dumb_create = drm_gem_cma_dumb_create,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	// .get_vblank_timestamp = vsp_get_vblank_timestamp,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
};

static const struct drm_mode_config_funcs vsp_mode_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void bst_vsp_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	/* Apply the atomic update. */
	drm_atomic_helper_commit_modeset_disables(dev, old_state);
	drm_atomic_helper_commit_planes(dev, old_state, 0);
	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_fake_vblank(old_state);
	drm_atomic_helper_commit_hw_done(old_state);
	// wait FW done
	wait_for_completion_timeout(&vout_completion, msecs_to_jiffies(50));
	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static const struct drm_mode_config_helper_funcs bst_vsp_mode_config_helper = {
	.atomic_commit_tail = bst_vsp_atomic_commit_tail,
};

static int vsp_modeset_init(struct vop_device *vspdev)
{
	struct drm_device *dev = vspdev->drm;

	drmm_mode_config_init(dev);
	dev->mode_config.funcs = &vsp_mode_funcs;
	dev->mode_config.min_width = XRES_MIN;
	dev->mode_config.min_height = YRES_MIN;
	dev->mode_config.max_width = XRES_MAX;
	dev->mode_config.max_height = YRES_MAX;
	dev->mode_config.helper_private = &bst_vsp_mode_config_helper;

	return vsp_output_init(vspdev);
}

static struct vout_receiver_ops *s_receiver_ops;

int vout_register_receiver_ops(struct vout_receiver_ops *recv_ops)
{
	if (s_receiver_ops == NULL)
		s_receiver_ops = recv_ops;
	else
		pr_err("!!!!! duplicate to call %s\n", __func__);

	return 0;
}

int vout_receiver_reset(void)
{
	if (s_receiver_ops && s_receiver_ops->reset)
		s_receiver_ops->reset();

	return 0;
}

static int bst_drm_platform_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct drm_device *ddev;
	struct vop_device *pvop;

	pvop = devm_kzalloc(&pdev->dev, sizeof(struct vop_device), GFP_KERNEL);
	if (!pvop) {
		DRM_ERROR("devm kzalloc failed!\n");
		return -ENOMEM;
	}

	ddev = drm_dev_alloc(&vsp_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	pvop->dev = &pdev->dev;
	pvop->drm = ddev;
	ddev->dev_private = pvop;
	platform_set_drvdata(pdev, pvop);

	pvop->clock = devm_clk_get(pvop->dev, "vout_display_clk");
	if (IS_ERR(pvop->clock)) {
		dev_err(pvop->dev, "no clock for bsta1000 vout\n");
		return PTR_ERR(pvop->clock);
	}

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		DRM_ERROR("Couldn't claim our memory region\n");

	pvop->display_param =
		dma_alloc_coherent(pvop->dev, sizeof(struct VoutCfg_t),
				   &pvop->display_param_phyaddr, GFP_KERNEL);
	if (pvop->display_param == NULL) {
		dev_err(pvop->dev, "alloc memory for display_param failed\n");
		return -ENOMEM;
	}

	ret = bst_vout_display_parse_dt(pvop);
	if (ret < 0)
		return ret;

	pvop->vout_dev_init = 0;
	pvop->primary_default_addr = 0;

	pvop->drm->irq_enabled = true;

	ret = vsp_modeset_init(pvop);
	if (ret)
		goto out_fini;
	ret = drm_vblank_init(pvop->drm, 1);
	if (ret) {
		DRM_ERROR("Failed to vblank\n");
		goto out_fini;
	}

	ret = drm_dev_register(pvop->drm, 0);
	if (ret)
		goto out_fini;

	vsp_core_callback_register(VSP_CORE_VOUT, vout_ipc_msg_recv);

	/*create sysfs interface*/
	ret = vsp_debug_sysfs_init(pvop);
	pvop->enable_primary = 1;
	pr_info("%s exit!\n", __func__);

	return 0;

out_fini:
	// drm_dev_fini(&pvop->drm);
	kfree(pvop);
	return ret;
}

static int bst_drm_platform_remove(struct platform_device *pdev)
{
	struct vop_device *pvop = platform_get_drvdata(pdev);

	DRM_DEBUG("%s\n", __func__);

	if (!pvop)
		DRM_INFO("pvop is NULL.\n");

	drm_dev_unregister(pvop->drm);
	drm_dev_put(pvop->drm);

	kfree(pvop);

	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{ .compatible = "bst,bst-vsp" },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct platform_driver bst_drm_platform_driver = {
	.probe = bst_drm_platform_probe,
	.remove = bst_drm_platform_remove,
	.driver = {
		.name = "vsp",
		.of_match_table = drv_dt_ids,
	},
};

static int __init bst_vop_init(void)
{
	return platform_driver_register(&bst_drm_platform_driver);
}

late_initcall_sync(bst_vop_init);
MODULE_AUTHOR("jim.zheng@bst.ai");
MODULE_LICENSE("GPL v2");

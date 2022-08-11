// SPDX-License-Identifier: GPL-2.0

/*
 * VSP IPC driver for BST
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

#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "bst_ipc_drv.h"

#include <linux/coreip/bst_vsp_msg.h>
#include <linux/ipc_interface.h>

int32_t session_id = -1;
struct vsp_device *pvsp;

void vsp_core_callback_register(enum vsp_core_id cid, callback func)
{
	pvsp->msg_handle[cid] = func;
}

#define A1000B_VSP_ENABLE_REG 0x33002180
#define A1000B_VSP_CLK_ENABLE 0x53090000
#define A1000B_VSP_PLL_SELECT 0x33002158

static void vout_reg_write(void *reg, uint32_t mask, uint32_t val)
{
	uint32_t status;

	status = readl_relaxed(reg);
	status = ((status & mask) | val);
	writel_relaxed(status, reg);
}

static int a1000b_vout_init(void)
{
	void *vsp_enable_reg = NULL;
	void *vsp_clk_reg = NULL;
	void *vsp_pll_select = NULL;
	uint32_t status;

	vsp_enable_reg = ioremap(A1000B_VSP_ENABLE_REG, 0x4);
	if (vsp_enable_reg == NULL) {
		pr_err("ioremap(A1000B_VSP_ENABLE_REG failed\n");
		return 0;
	}

	vsp_clk_reg = ioremap(A1000B_VSP_CLK_ENABLE, 0x4);
	if (vsp_clk_reg == NULL) {
		pr_err("ioremap(A1000B_VSP_CLK_ENABLE failed\n");
		return 0;
	}

	vsp_pll_select = ioremap(A1000B_VSP_PLL_SELECT, 0x4);
	if (vsp_pll_select == NULL) {
		pr_err("ioremap(A1000B_VSP_PLL_SELECT failed\n");
		return 0;
	}

	vout_reg_write(vsp_enable_reg, 0xffffffff, (0x1 << 11));
	vout_reg_write(vsp_clk_reg, 0xffffffff, 0xffffffff);
	// vout_reg_write(vsp_pll_select, 0xfffeffff, 0x0);

	status = readl_relaxed(vsp_enable_reg);
	status = readl_relaxed(vsp_clk_reg);
	// status = readl_relaxed(vsp_pll_select);

	return 0;
}

static int bst_vsp_ipc_probe(struct platform_device *pdev)
{
	int ret = 0;
	int32_t session_id;
	struct device *dev = &pdev->dev;
	struct task_struct *recv_thread;

	pvsp = devm_kzalloc(&pdev->dev, sizeof(struct vsp_device), GFP_KERNEL);
	if (!pvsp)
		return -ENOMEM;

	pvsp->dev = &pdev->dev;
	platform_set_drvdata(pdev, pvsp);

	mutex_init(&pvsp->fw_boot_lock);

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		pr_err("Couldn't claim our memory region\n");

	a1000b_vout_init();

	ret = of_device_parse(pdev, pvsp);
	if (ret && ret != -ENODEV)
		pr_err("parse vsp fw failed!\n");

	session_id = ipc_init(IPC_CORE_VSP, IPC_CORE_ARM1, &pdev->dev);
	if (session_id < 0) {
		pr_err("ipc_init(IPC_CORE_VSP) failed, ret = %d", session_id);
		return -1;
	}
	pvsp->session_id = session_id;

	ret = vsp_fw_mem_init(pvsp);
	if (ret < 0)
		pr_err("Failed to init vsp fw parttion.");

	ret = vsp_dev_init(pvsp);
	if (ret < 0)
		pr_err("Failed to init vsp register.");

	pvsp->fw_boot_done = 0;

	// recv thread
	recv_thread = kthread_run(vsp_recv_msg_thread, pdev, "recv_thread");
	if (IS_ERR(recv_thread)) {
		pr_err("Failed to create msg recv thread");
		return PTR_ERR(recv_thread);
	}

	return 0;
}

static int bst_vsp_ipc_remove(struct platform_device *pdev)
{
	struct vsp_device *pvsp_dev = platform_get_drvdata(pdev);

	mutex_destroy(&pvsp_dev->fw_boot_lock);
	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{ .compatible = "bst,bst-vsp-ipc" },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct platform_driver bst_vsp_ipc_driver = {
	.probe = bst_vsp_ipc_probe,
	.remove = bst_vsp_ipc_remove,
	.driver = {
		.name = "vsp-ipc",
		.of_match_table = drv_dt_ids,
	},
};

static int __init bst_vsp_ipc_init(void)
{
	return platform_driver_register(&bst_vsp_ipc_driver);
}

late_initcall(bst_vsp_ipc_init);
MODULE_AUTHOR("jim.zheng@bst.ai");
MODULE_LICENSE("GPL v2");

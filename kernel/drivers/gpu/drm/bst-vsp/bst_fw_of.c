// SPDX-License-Identifier: GPL-2.0

/*
 * VSP device tree parser for BST
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/coreip/proto_api_common.h>

#include "bst_ipc_drv.h"

int of_device_parse(struct platform_device *pdev, struct vsp_device *pvsp)
{
	int ret;
	struct resource *fw;
	struct resource *vsp_fw_fbuf_base;
	struct resource *regfw, *regctl;
	// struct resource *regsta;
	int total_fw_size = 0;
	phys_addr_t base_phy_addr;

	fw = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pvsp->firmware_c0.phy_addr = fw->start;
	pvsp->firmware_c0.size = fw->end - fw->start + 1; // end is 0xf00fffff
	total_fw_size += pvsp->firmware_c0.size;
	base_phy_addr = pvsp->firmware_c0.phy_addr;
	// dev_info(&pdev->dev,
	//	 "firmware 0, phy_addr = 0x%lx, end = 0x%lx, size = 0x%lx\n",
	//	 fw->start, fw->end, pvsp->firmware_c0.size);

	fw = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pvsp->firmware_c1.phy_addr = fw->start;
	pvsp->firmware_c1.size = fw->end - fw->start + 1;
	total_fw_size += pvsp->firmware_c1.size;
	// dev_info(&pdev->dev,
	//	 "firmware 1, phy_addr = 0x%lx, end = 0x%lx, size = 0x%lx\n",
	//	 fw->start, fw->end, pvsp->firmware_c1.size);

	fw = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	pvsp->firmware_c2.phy_addr = fw->start;
	pvsp->firmware_c2.size = fw->end - fw->start + 1;
	total_fw_size += pvsp->firmware_c2.size;
	// dev_info(&pdev->dev,
	//	 "firmware 2, phy_addr = 0x%lx, end = 0x%lx, size = 0x%lx\n",
	//	 fw->start, fw->end, pvsp->firmware_c2.size);

	regctl = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	regfw = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	// regsta = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	vsp_fw_fbuf_base = platform_get_resource(pdev, IORESOURCE_MEM, 6);
	pvsp->fw_fbuf_phy_addr = vsp_fw_fbuf_base->start;
	pvsp->fw_fbuf_size =
		vsp_fw_fbuf_base->end - vsp_fw_fbuf_base->start + 1;
	// dev_info(&pdev->dev,
	//	 "vsp fw fbuf, phy_addr = 0x%lx, end = 0x%lx, size = 0x%lx\n",
	//	 vsp_fw_fbuf_base->start, vsp_fw_fbuf_base->end,
	//	 pvsp->fw_fbuf_size);

	pvsp->intp_phy_addr = base_phy_addr + total_fw_size;
	if (pvsp->intp_phy_addr & SONE_BUF_ADDR_ALIGN_MASK) {
		// dev_err(&pdev->dev,
		//	"base_phy_addr = 0x%lx, total_fw_size = 0x%lx, should be
		// 1MB align\n", base_phy_addr, total_fw_size);
		return -1;
	}

	pvsp->vreg.regctl = devm_ioremap_resource(&pdev->dev, regctl);
	pvsp->vreg.regfw = devm_ioremap_resource(&pdev->dev, regfw);
	//	pvsp->vreg.regsta = devm_ioremap_resource(&pdev->dev, regsta);

	if (IS_ERR(pvsp->vreg.regctl) | IS_ERR(pvsp->vreg.regfw)) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "Failed to remap vsp mem: %d\n", ret);
		return ret;
	}

	return 0;
}

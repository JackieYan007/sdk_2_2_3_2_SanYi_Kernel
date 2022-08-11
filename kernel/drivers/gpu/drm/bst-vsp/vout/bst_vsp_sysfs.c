// SPDX-License-Identifier: GPL-2.0

/*
 * VSP sysfs entry for BST
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

#include "bst_drm_drv.h"

#include "../bst_ipc_drv.h"

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	struct vop_device *pvop;
	int enable;

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));
	pvop->enable_primary = enable;

	pr_debug("enable_primary = %d\n", pvop->enable_primary);

	if (pvop->enable_primary == 0) {
		// primary will always need to flip
		// in this mode, flip the default buffer
		if (pvop->primary_default_addr == 0) {
			uint8_t *vaddr;
			int nv12_size;
			int y_size;

			nv12_size = pvop->display_param->hactive *
				    pvop->display_param->vactive * 3 / 2;
			y_size = pvop->display_param->hactive *
				 pvop->display_param->vactive;

			vaddr = dma_alloc_coherent(
				pvop->dev, nv12_size,
				&(pvop->primary_default_addr), GFP_KERNEL);
			// set uv part to 128, to make nv12 layer black
			memset(vaddr + y_size, 128, (nv12_size - y_size));
			// pr_debug("dma_alloc_coherent nv12_size = %d, y_size =
			// %d, default addr = 0x%08x\n",
			//      nv12_size, y_size, pvop->primary_default_addr);
		}
		pvop->primary_paddr = pvop->primary_default_addr;
	}

	return len;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	ssize_t ret = 0;
	struct vop_device *pvop;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));
	sprintf(buf, "VSP nv12 palne status: %d\n", pvop->enable_primary);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(enable, 0644, enable_show, enable_store);

static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "VSP version = 0x%x.%x.%x\n", VERSION, LEVEL, SUBLEVEL);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(version, 0444, version_show, NULL);

static ssize_t hsize_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	struct vop_device *pvop;
	int hsize;

	if (kstrtoint(buf, 10, &hsize))
		return -EINVAL;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));
	pvop->hsize = hsize;

	return len;
}

static ssize_t hsize_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	ssize_t ret = 0;
	struct vop_device *pvop;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));

	sprintf(buf, "VSP vout hsize: %d\n", pvop->hsize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(hsize, 0644, hsize_show, hsize_store);

static ssize_t vsize_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	struct vop_device *pvop;
	int vsize;

	if (kstrtoint(buf, 10, &vsize))
		return -EINVAL;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));
	pvop->vsize = vsize;

	return len;
}

static ssize_t vsize_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	ssize_t ret = 0;
	struct vop_device *pvop;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));

	sprintf(buf, "VSP vout vsize: %d\n", pvop->vsize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vsize, 0644, vsize_show, vsize_store);

static ssize_t fresh_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	struct vop_device *pvop;
	int fresh;

	if (kstrtoint(buf, 10, &fresh))
		return -EINVAL;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));
	pvop->fresh = fresh;

	return len;
}

static ssize_t fresh_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	ssize_t ret = 0;
	struct vop_device *pvop;

	pvop = platform_get_drvdata(
		container_of(dev, struct platform_device, dev));

	sprintf(buf, "VSP vout fresh: %d\n", pvop->fresh);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(fresh, 0644, fresh_show, fresh_store);

int vsp_debug_sysfs_init(struct vop_device *pvop)
{
	int ret = 0;
	struct device *dev = pvop->dev;

	ret = device_create_file(dev, &dev_attr_enable);
	ret = device_create_file(dev, &dev_attr_version);
	ret = device_create_file(dev, &dev_attr_hsize);
	ret = device_create_file(dev, &dev_attr_vsize);
	ret = device_create_file(dev, &dev_attr_fresh);

	return ret;
}

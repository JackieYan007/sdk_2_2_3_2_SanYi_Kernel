// SPDX-License-Identifier: GPL-2.0

/*
 * CSI device driver for BST
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "csi.h"
#include "csi_dphy.h"

static int csi_channel_get_port_info(struct bst_csi_device *pcsi_dev,
				     struct device_node *node)
{
	struct device_node *port = NULL;
	struct bst_csi_channel *channel;
	int i;

	for (i = 0; i < MAX_VC_PER_CSI; i++) {
		channel = &pcsi_dev->csi_vc[i];
		channel->csi_dev_id = pcsi_dev->csi_id;
		channel->index = i;
		channel->csi_chn_id = ((pcsi_dev->csi_id << 2) | i);
		channel->sn_in_all_csi =
			((pcsi_dev->csi_id * MAX_VC_PER_CSI) + i);
		channel->csi = pcsi_dev;

		port = of_graph_get_port_by_id(node, i);
		if (port != NULL) {
			pr_info("mipi chn %d connected\n", i);
		} else {
			pr_info("mipi chn %d not connected\n", i);
			continue;
		}
	}

	return 0;
}

static int csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int csi_device_init(struct bst_csi_device *csi_dev)
{
	if (of_device_is_compatible(csi_dev->dev->of_node,
				    "bst,a1000b-csi2-2x2"))
		dphy_config(csi_dev->csi_id, csi_dev->lane_speed,
			    csi_dev->num_lanes);
	else
		csi_dphy_config_lanes(csi_dev, csi_dev->lane_speed,
				      csi_dev->csi_id, csi_dev->num_lanes);

	return 0;
}

static int csi_s_power(struct v4l2_subdev *sd, int enable)
{
	struct bst_csi_device *pcsi_dev;
	struct deser_hub_dev *pdeser_dev;

	pcsi_dev = container_of(sd, struct bst_csi_device, subdev);
	if (pcsi_dev == NULL)
		return -1;

	pdeser_dev = pcsi_dev->deser;
	if (pdeser_dev == NULL)
		return -1;

	if (enable)
		csi_device_init(pcsi_dev);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int csi_get_format(struct v4l2_subdev *subdev,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int csi_set_format(struct v4l2_subdev *subdev,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
};

static struct v4l2_subdev_pad_ops csi_pad_ops = {
	.get_fmt = csi_get_format,
	.set_fmt = csi_set_format,
};

static struct v4l2_subdev_core_ops csi_core_ops = {
	.s_power = csi_s_power,
};

static struct v4l2_subdev_ops csi_sub_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int csi_notify_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	struct deser_hub_dev *pdeser_dev;
	struct bst_csi_device *pcsi_dev;

	pdeser_dev = container_of(sd, struct deser_hub_dev, subdev);
	pcsi_dev = container_of(asd, struct bst_csi_device, async_dev);
	pcsi_dev->deser = pdeser_dev;
	pdeser_dev->csi_dev = pcsi_dev;
	pdeser_dev->sd_state = BST_SUBDEV_STATE_BOUND;

	// pr_info("csi dev name = %s, deser dev name = %s\n",
	//	pcsi_dev->subdev.name, pdeser_dev->subdev.name);

	// return media_create_pad_link(
	//	&sd->entity, 0, &csi_channel->subdev.entity, 0,
	//	MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);

	// :: todo ::

	return 0;
}

// :: todo ::
// update link
static void csi_notify_unbind(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations csi_async_ops = {
	.bound = csi_notify_bound,
	.unbind = csi_notify_unbind,
};

int update_camera_status_in_csi(struct bst_csi_device *pcsi_dev)
{
	struct deser_hub_dev *pdeser_dev;
	struct deser_channel *pdeser_chn;
	int i;

	if (pcsi_dev == NULL)
		return -1;

	pdeser_dev = pcsi_dev->deser;
	if (pdeser_dev == NULL)
		return -1;

	// pr_info("update_camera_status_in_csi pdeser_dev->num_cameras = %d\n"
	//	, pdeser_dev->num_cameras);
	for (i = 0; i < pdeser_dev->num_cameras; i++) {
		pdeser_chn = &(pdeser_dev->chn[i]);
		if (pdeser_chn == NULL)
			continue;

		if (pdeser_chn->camera_bound) {
			// pr_info("=== update_camera_status_in_csi id = %d, i =
			// %d camera_bound\n",	pcsi_dev->csi_id, i);
			pcsi_dev->csi_vc[i].cam_dev = pdeser_chn->cam_dev;
			pcsi_dev->csi_vc[i].connected =
				pdeser_chn->cam_dev->power_on;

			// return media_create_pad_link(
			//	&sd->entity, 0, &pcsi_dev->subdev.entity, 0,
			//	MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
		}
	}

	return 0;
}

static int init_csi_channel_one(struct bst_csi_channel *pcsi_channel, int index)
{
	// struct v4l2_subdev *sd;
	int ret;

	pcsi_channel->pads[CSI_CHANNEL_SINK_PAD].flags = MEDIA_PAD_FL_SINK;
	pcsi_channel->pads[CSI_CHANNEL_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	// atomic_set(&pcam->is_streaming, 0);

	pcsi_channel->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	pcsi_channel->entity.ops = &csi_media_ops;

	ret = media_entity_pads_init(&pcsi_channel->entity, 2,
				     pcsi_channel->pads);
	if (ret < 0)
		return ret;

	return 0;
}

static int init_csi_channel_devs(struct bst_csi_device *pcsi_dev)
{
	int i;
	struct bst_csi_channel *pcsi_channel;

	for (i = 0; i < pcsi_dev->num_vc; i++) {
		pcsi_channel = &pcsi_dev->csi_vc[i];
		pcsi_channel->csi = pcsi_dev;
		init_csi_channel_one(pcsi_channel, i);
	}

	return 0;
}

static int bst_csi_parse_dt(struct bst_csi_device *pcsi_dev)
{
	struct device_node *remote_ep;
	struct v4l2_fwnode_endpoint v4l2_ep;
	int ret;
	int id;
	int num_channels = 0;
	int lane_speed;
	struct device_node *node = pcsi_dev->dev->of_node;
	struct device_node *link_dt = NULL;
	// struct device_node *channel_dt = NULL;
	struct v4l2_subdev *sd;

	if (!node)
		return -EINVAL;

	ret = of_property_read_u32(node, "id", &id);
	if (ret) {
		pr_err("mipi find id error\n");
		return -2;
	}
	pcsi_dev->csi_id = id;
	ret = of_property_read_u32(node, "lane-speed", &lane_speed);
	if (ret) {
		pr_err("mipi find id error\n");
		return -2;
	}
	pcsi_dev->lane_speed = lane_speed;

	memset(&v4l2_ep, 0, sizeof(v4l2_ep));
	/*parse this port to v4l2_fwnode_endpoint */
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &v4l2_ep);
	if (ret) {
		dev_err(pcsi_dev->dev,
			"mipi v4l2_fwnode_endpoint_parse error\n");
		return -4;
	}
	pcsi_dev->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
	dev_dbg(pcsi_dev->dev, "num_lanes = %d\n", pcsi_dev->num_lanes);

	link_dt = of_get_child_by_name(node, "csi-link");
	if (link_dt == NULL) {
		dev_err(pcsi_dev->dev, "get csi-link error\n");
		return -5;
	}
	remote_ep = of_graph_get_remote_node(link_dt, 0, 0);
	// remote_ep = of_graph_get_remote_parent(link_dt, 0, 0);
	if (remote_ep == NULL) {
		pr_err("can not find remote ep\n");
		return -1;
	}
	dev_dbg(pcsi_dev->dev, "remote_ep name = %s, fullname = %s\n",
		remote_ep->name, remote_ep->full_name);

	pcsi_dev->csi_fwnode = of_fwnode_handle(node);
	pcsi_dev->remote_fwnode = of_fwnode_handle(remote_ep);
	dev_dbg(pcsi_dev->dev,
		"node = %s, csi_fwnode = 0x%p, remote = %s, fwnode = 0x%p\n",
		node->full_name, pcsi_dev->csi_fwnode, remote_ep->full_name,
		pcsi_dev->remote_fwnode);

	sd = &pcsi_dev->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &csi_sub_ops);
	sd->dev = pcsi_dev->dev;
	v4l2_set_subdevdata(sd, pcsi_dev);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &csi_media_ops;
	sd->fwnode = pcsi_dev->csi_fwnode;
	// pr_info("===== sd->fwnode = 0x%x\n", sd->fwnode);
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(pcsi_dev->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		pr_err("failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	pcsi_dev->async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
	pcsi_dev->async_dev.match.fwnode = pcsi_dev->remote_fwnode;
	v4l2_async_notifier_init(&pcsi_dev->notifier);
	ret = v4l2_async_notifier_add_subdev(&pcsi_dev->notifier,
					     &pcsi_dev->async_dev);
	if (ret < 0) {
		dev_err(pcsi_dev->dev,
			"Failed to do v4l2_async_notifier_add_subdev\n");
	}

	pcsi_dev->notifier.ops = &csi_async_ops;
	ret = v4l2_async_subdev_notifier_register(&pcsi_dev->subdev,
						  &(pcsi_dev->notifier));
	if (ret < 0) {
		dev_err(pcsi_dev->dev,
			"v4l2_async_subdev_notifier_register register failed\n");
	}

	num_channels = csi_channel_get_port_info(pcsi_dev, node);
	pcsi_dev->num_vc = num_channels;

	return 0;
}

#define A1000B_ALL_ENABLE_REG 0x33002180

static void enable_all(void)
{
	uint32_t status;
	void *all_enable_reg = NULL;

	all_enable_reg = ioremap(A1000B_ALL_ENABLE_REG, 0x4);
	if (all_enable_reg == NULL) {
		pr_err("ioremap(A1000B_VSP_ENABLE_REG failed\n");
		return;
	}

	status = readl_relaxed(all_enable_reg);
	if (status != 0x1ffff) {
		status = 0xffffffff;
		writel_relaxed(status, all_enable_reg);
	}

	status = readl_relaxed(all_enable_reg);
}

static int a1000_csi_probe(struct platform_device *pdev)
{
	struct bst_csi_device *pcsi_dev;
	struct device *dev = &pdev->dev;

	struct reset_control *rst_contrl = NULL;

	pcsi_dev = devm_kzalloc(dev, sizeof(struct bst_csi_device), GFP_KERNEL);
	if (!pcsi_dev)
		return -ENOMEM;

	dev_info(dev, "%s\n", __func__);
	if (of_device_is_compatible(dev->of_node, "bst,a1000b_csi2") ||
	    of_device_is_compatible(dev->of_node, "bst,a1000b-csi2-2x2")) {
		dev_info(dev, "%s\n", __func__);
		enable_all();
	}

	pcsi_dev->pdev = pdev;
	pcsi_dev->dev = dev;
	mutex_init(&pcsi_dev->mutex);
	rst_contrl =
		devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	reset_control_deassert(rst_contrl);
	bst_csi_parse_dt(pcsi_dev);
	init_csi_channel_devs(pcsi_dev);

	return 0;
}

static int a1000_csi_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device_id a1000_csi_id_table[] = {
	{ .name = "a1000_csi2", .driver_data = 0 },
	{},
};
MODULE_DEVICE_TABLE(platform, a1000_isp_id_table);

static const struct of_device_id a1000_csi_of_table[] = {
	{ .compatible = "bst,a1000_csi2" },
	{ .compatible = "bst,a1000b_csi2" },
	{ .compatible = "bst,a1000b-csi2-2x2" },
	{},
};
MODULE_DEVICE_TABLE(of, a1000_csi_of_table);

static struct platform_driver a1000_csi_driver = {
	.probe  = a1000_csi_probe,
	.remove = a1000_csi_remove,
	.id_table = a1000_csi_id_table,
	.driver = {
		.name = "a1000-csi2",
		.of_match_table = a1000_csi_of_table,
	},
};

module_platform_driver(a1000_csi_driver);

MODULE_AUTHOR("BST Corporation");
MODULE_DESCRIPTION("BST A1000 CSI2.0 driver");
MODULE_LICENSE("GPL");

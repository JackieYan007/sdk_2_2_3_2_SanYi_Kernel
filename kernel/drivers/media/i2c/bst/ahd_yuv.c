// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ahd camera driver for BST Cameras Driver
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/media-device.h>

#include "n4.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,ahd-isp"

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *cam = container_of(sd, struct camera_dev, subdev);

	if (!enable)
		return 0;

	cam->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_core_ops camera_core_ops = {
	.s_power = camera_s_power,
};

static struct v4l2_subdev_ops camera_ops = {
	.core = &camera_core_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations camera_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int camera_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *cam;
	struct device *dev = &client->dev;
	int ret;

	cam = devm_kzalloc(&client->dev, sizeof(struct camera_dev), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	if (client->adapter == NULL) {
		dev_err(dev, "client->adapter NULL\n");
		return -ENXIO;
	}

	cam->i2c_client = client;
	cam->dev = dev;
	cam->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(cam, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(cam, &camera_ops
		, &camera_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int camera_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id camera_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id camera_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, camera_of_match);
MODULE_DEVICE_TABLE(i2c, camera_id);

static struct i2c_driver camera_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(camera_of_match),
	},
	.probe		= camera_probe,
	.remove		= camera_remove,
	.id_table	= camera_id,
};

module_i2c_driver(camera_driver);
MODULE_DESCRIPTION("Driver for ahd yuv camera");
MODULE_LICENSE("GPL v2");

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ar0231 yuv camera for BST Cameras Driver
 *
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

#include "ti_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,ar0231-isp"

static int camera_g_register(struct camera_dev *camera, uint16_t regaddr, uint16_t *regval)
{
	struct i2c_adapter *adap = camera->i2c_client->adapter;

	if (strncmp(camera->deser_parent->ctl_level, "fad-lis", 7) == 0)
		return -EPERM;

	return bst_i2c_read_word_data_word_reg(adap, camera->sensor_alias_id, regaddr, regval);
}

static int camera_s_register(struct camera_dev *camera, uint16_t regaddr, uint16_t regval)
{
	struct i2c_adapter *adap = camera->i2c_client->adapter;

	if (strncmp(camera->deser_parent->ctl_level, "fad-lis", 7) == 0)
		return -EPERM;

	return bst_i2c_write_word_data_word_reg(adap, camera->sensor_alias_id, regaddr, regval);
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *camera_ar0231;
	int ret = 0;

	if (!enable)
		return 0;

	camera_ar0231 = container_of(sd, struct camera_dev, subdev);
	if (camera_ar0231->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(camera_ar0231);
	if (!ret)
		return -EINVAL;

	camera_ar0231->power_on = true;

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

static const struct media_entity_operations ar0231_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ar0231_isp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *camera_ar0231;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	camera_ar0231 = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!camera_ar0231)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	camera_ar0231->i2c_client = client;
	camera_ar0231->dev = dev;
	bus_index = client->adapter->nr;

	dev_dbg(dev, "chip address is %x,addr is %x\r\n",
		camera_ar0231->i2c_client->addr,
		camera_ar0231->i2c_client->adapter->nr);

	camera_ar0231->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	camera_ar0231->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	camera_ar0231->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;
	camera_ar0231->ops.g_register = camera_g_register;
	camera_ar0231->ops.s_register = camera_s_register;

	ret = parse_camera_endpoint(camera_ar0231, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(camera_ar0231, &camera_ops
			, &ar0231_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ar0231_isp_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ar0231_isp_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ar0231_isp_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ar0231_isp_of_match);
MODULE_DEVICE_TABLE(i2c, ar0231_isp_id);

static struct i2c_driver ar0231_isp_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ar0231_isp_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ar0231_isp_probe,
	.remove		= ar0231_isp_remove,
	.id_table	= ar0231_isp_id,
};

module_i2c_driver(ar0231_isp_driver);

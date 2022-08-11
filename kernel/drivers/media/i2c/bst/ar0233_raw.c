// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ar0233 raw camera for BST Cameras Driver
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
#include "ar0233_config.h"

#define MODULE_NAME "bst,ar0233"

static uint8_t template_ser[6][2] = {
	{ 0x02, 0x73 }, { 0x03, 0x10 }, { 0x06, 0x41 },
	{ 0x07, 0x28 }, { 0x0e, 0x10 }, { 0x0d, 0x10 },
};

static int ar0233_start(struct camera_dev *cam_dev)
{
	int ret = bst_i2c_write_word_data_word_reg(
			cam_dev->i2c_client->adapter,
			cam_dev->sensor_alias_id,
			0x0000,
			0x00);
	if (ret) {
		dev_err(cam_dev->dev, "ar0233 sensor 0x0000, 0x00 write failed");
		return -1;
	}

	return 0;
}

static int ar0233_power_on(struct camera_dev *cam_dev)
{
	int i, sensor_cfg_size;
	u16 reg_value;
	int ret;
	struct i2c_adapter *adap;

	if (strncmp(cam_dev->deser_parent->ctl_level, "fad-lis", 7) == 0)
		return 0;

	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	sensor_cfg_size = sizeof(sensor_base_settings) /
			  sizeof(struct ar0233_sensor_base_cfg);

	for (i = 0; i < 6; i++) {
		ret = bst_i2c_write_byte_data_byte_reg(
			    adap,
			    cam_dev->ser_alias_id,
			    template_ser[i][0],
			    template_ser[i][1]);
		if (ret < 0) {
			dev_err(cam_dev->dev, "ar0233 serializer write error: %x ---> %x",
			       template_ser[i][0], template_ser[i][1]);
			return ret;
		}
	}

	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_word_data_word_reg(
			    adap, cam_dev->sensor_alias_id,
			    sensor_base_settings[i].reg,
			    sensor_base_settings[i].value);
		if (ret) {
			dev_err(cam_dev->dev, "ar0233 sensor %x,%x write failed",
			       sensor_base_settings[i].reg,
			       sensor_base_settings[i].value);
			continue;
		}

		ret = bst_i2c_read_word_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			sensor_base_settings[i].reg,
			&reg_value);
		if (ret) {
			dev_err(cam_dev->dev, "ar0233 sensor %x,%x read failed",
			       sensor_base_settings[i].reg,
			       sensor_base_settings[i].value);
			continue;
		}

	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *sensor_ar0233;
	int ret = 0;

	if (!enable)
		return 0;

	sensor_ar0233 = container_of(sd, struct camera_dev, subdev);
	if (sensor_ar0233->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(sensor_ar0233);
	if (!ret)
		return -EINVAL;

	ret = ar0233_power_on(sensor_ar0233);
	if (ret)
		return ret;

	sensor_ar0233->power_on = true;

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

static const struct media_entity_operations ar0233_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ar0233_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *sensor_ar0233;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	sensor_ar0233 = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!sensor_ar0233)
		return -ENOMEM;

	if (client->adapter == NULL) {
		dev_err(dev, "client->adapter == NULL, error\n");
		return -1;
	}

	sensor_ar0233->i2c_client = client;
	sensor_ar0233->dev = dev;
	bus_index = client->adapter->nr;

	ret = parse_camera_endpoint(sensor_ar0233, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(sensor_ar0233, &camera_ops
				, &ar0233_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	sensor_ar0233->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	sensor_ar0233->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_WORD_REG_BYTE_DATA;
	sensor_ar0233->isp_data.sensorType = BST_SENSOR_TYPE_AR0233_RAW;
	sensor_ar0233->isp_data.rawinfo.dvpDummyLines = 1;

	return 0;
}

static int ar0233_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ar0233_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ar0233_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ar0233_of_match);
MODULE_DEVICE_TABLE(i2c, ar0233_id);

static struct i2c_driver ar0233_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ar0233_of_match),
	},
	.probe		= ar0233_probe,
	.remove		= ar0233_remove,
	.id_table	= ar0233_id,
};

module_i2c_driver(ar0233_driver);

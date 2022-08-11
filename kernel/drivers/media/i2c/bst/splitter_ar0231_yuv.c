// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * splitter ar0231 yuv Camera for BST Camera Driver
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

#define MODULE_NAME "bst,ar0231-splitter"

static uint8_t template_ser[1][2] = {
	{ 0x02, 0x53 },
};

static int splitter_ar0231_ser_cfg(struct camera_dev *cam_dev)
{
	int i;
	int ret;
	struct i2c_adapter *adap;

	if (strncmp(cam_dev->deser_parent->ctl_level, "fad-lis", 7) == 0)
		return 0;

	if (cam_dev->i2c_client == NULL)
		return -EINVAL;


	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	pr_info("%s : ser iic address is %x", __func__, cam_dev->ser_alias_id);
	for (i = 0; i < 1; i++) {
		ret = bst_i2c_write_byte_data_byte_reg(
				    adap,
					cam_dev->ser_alias_id,
				    template_ser[i][0],
				    template_ser[i][1]);
		if (ret < 0) {
			pr_info(" splitter_ar0231_ser write error: %x ---> %x",
			       template_ser[i][0], template_ser[i][1]);
			return -1;
		}
		usleep_range(1000, 2000);
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *ar0231_splitter;
	int ret = 0;

	if (!enable)
		return 0;

	ar0231_splitter = container_of(sd, struct camera_dev, subdev);
	if (ar0231_splitter->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(ar0231_splitter);
	if (!ret)
		return -EINVAL;


	if (!is_slave_soc_model(ar0231_splitter))
		ret = splitter_ar0231_ser_cfg(ar0231_splitter);
		if (ret)
			return ret;

	ar0231_splitter->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int camera_get_format(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int camera_set_format(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */

static struct v4l2_subdev_pad_ops camera_pad_ops = {
	.get_fmt = camera_get_format,
	.set_fmt = camera_set_format,
};

static struct v4l2_subdev_core_ops camera_core_ops = {
	.s_power = camera_s_power,
};

static struct v4l2_subdev_ops camera_ops = {
	.core = &camera_core_ops,
	.pad = &camera_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations ar0231_splitter_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ar0231_isp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *ar0231_splitter;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	ar0231_splitter = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!ar0231_splitter)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	ar0231_splitter->i2c_client = client;
	ar0231_splitter->dev = dev;
	bus_index = client->adapter->nr;

	//cam_dev->parent_iic_address =
	//	cam_dev->i2c_client->adapter->nr * IIC_OFFSET +
	//	IIC_BASE;

	ar0231_splitter->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	ar0231_splitter->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA;
	ar0231_splitter->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(ar0231_splitter, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ar0231_splitter, &camera_ops
				, &ar0231_splitter_media_ops);
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

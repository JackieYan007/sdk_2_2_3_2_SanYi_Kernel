// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ox8b yuv camera for BST Camera Driver
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

#define MAX9295_ID_REG		0x0d
#define MAX9295_ID		0x91
#define MAX9295_I2CADDR		0x40

#define MODULE_NAME "bst,ox08b-isp"
/*max9296<------>max9295*/
unsigned short max9295_regs02[][2] = {
	{0x2be, 0x10},
	{0x57, 0x12},
	{0x5b, 0x11},
	{0x318, 0x5e},
};

static int ox08b_ser_cfg(struct camera_dev *ox08b_isp)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (ox08b_isp == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}

	adap = ox08b_isp->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	if (ox08b_isp->deser_parent->type == DESER_TYPE_MAX9296) {
		pr_info("MAX9296 %s : ser iic address is %x", __func__,
				ox08b_isp->ser_alias_id);
		for (i = 0;
		     i < ARRAY_SIZE(max9295_regs02);
		     i++) {
			retry_times = 10;
			ret = -1;
			while (retry_times > 0 && ret) {
				ret = bst_i2c_write_byte_data_word_reg(
					adap, ox08b_isp->ser_alias_id,
					max9295_regs02[i][0],
					max9295_regs02[i][1]);
				if (ret) {
					pr_info(": write_max9295_reg failed!\n");
					retry_times--;
					usleep_range(2000, 2500);
					continue;
				}
				pr_info("write max9295 reg:%#x, val:%#x",
				       max9295_regs02[i][0],
				       max9295_regs02[i][1]);
			}
		}
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox08b_isp =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;


	ret = ox08b_ser_cfg(ox08b_isp);
	if (ret)
		return ret;

	ox08b_isp->power_on = true;

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

static const struct media_entity_operations ox08b_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox08b_isp_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct camera_dev *ox08b_isp;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	ox08b_isp = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				 GFP_KERNEL);
	if (!ox08b_isp)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	ox08b_isp->i2c_client = client;
	ox08b_isp->dev = dev;
	bus_index = client->adapter->nr;

	//cam_dev->parent_iic_address =
	//	cam_dev->i2c_client->adapter->nr * IIC_OFFSET +
	//	IIC_BASE;

	ox08b_isp->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	ox08b_isp->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA;
	ox08b_isp->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(ox08b_isp, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	bst_i2c_write_byte_data_word_reg(client->adapter, MAX9295_I2CADDR,
					 0x0000,
					 (ox08b_isp->ser_alias_id << 1));

	ret = init_camera_dev(ox08b_isp, &camera_ops, &ox08b_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ox08b_isp_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ox08b_isp_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox08b_isp_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ox08b_isp_of_match);
MODULE_DEVICE_TABLE(i2c, ox08b_isp_id);

static struct i2c_driver ox08b_isp_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox08b_isp_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox08b_isp_probe,
	.remove		= ox08b_isp_remove,
	.id_table	= ox08b_isp_id,
};

module_i2c_driver(ox08b_isp_driver);

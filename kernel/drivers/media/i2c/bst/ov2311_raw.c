// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM96712 Deserializer for BST Deserializer Driver
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

#define MODULE_NAME "bst,ov2311"

struct ov2311_sensor_base_cfg {
	uint16_t reg;
	uint8_t value;
};

static uint8_t template_ser[8][2] = {
	{ 0x02, 0x53 }, { 0x03, 0x4b }, { 0x05, 0x03 }, { 0x32, 0x09 },
	{ 0x06, 0x23 }, { 0x07, 0xfa }, { 0x0e, 0x69 }, { 0x0d, 0x06 },
};

static struct ov2311_sensor_base_cfg sensor_base_settings[] = {
	//@@ 1600x1300 30fps MIPI
	{ 0x0103, 0x01 }, { 0x0300, 0x01 }, { 0x0302, 0x32 }, { 0x0303, 0x00 },
	{ 0x0304, 0x03 }, { 0x0305, 0x02 }, { 0x0306, 0x01 }, { 0x030d, 0x5a },
	{ 0x030e, 0x04 }, { 0x030f, 0x05 }, { 0x3001, 0x02 }, { 0x3004, 0x00 },
	{ 0x3005, 0x00 }, { 0x3006, 0x00 }, { 0x3011, 0x0d }, { 0x3014, 0x04 },
	{ 0x301c, 0xf0 }, { 0x3020, 0x00 }, { 0x302c, 0x00 }, { 0x302d, 0x12 },
	{ 0x302e, 0x4c }, { 0x302f, 0x8c }, { 0x3030, 0x10 }, { 0x303f, 0x03 },
	{ 0x3103, 0x00 }, { 0x3106, 0x08 }, { 0x31ff, 0x01 }, { 0x3501, 0x00 },
	{ 0x3502, 0xdf }, { 0x3506, 0x00 }, { 0x3507, 0x00 }, { 0x3620, 0x67 },
	{ 0x3633, 0x78 }, { 0x3662, 0x65 }, { 0x3666, 0x00 }, { 0x3670, 0x68 },
	{ 0x3674, 0x10 }, { 0x3675, 0x00 }, { 0x3680, 0x84 }, { 0x36a2, 0x04 },
	{ 0x36a3, 0x80 }, { 0x36b0, 0x00 }, { 0x3700, 0x35 }, { 0x3704, 0x59 },
	{ 0x3712, 0x00 }, { 0x3713, 0x02 }, { 0x379b, 0x01 }, { 0x379c, 0x10 },
	{ 0x3800, 0x00 }, { 0x3801, 0x00 }, { 0x3802, 0x00 }, { 0x3803, 0x00 },
	{ 0x3804, 0x06 }, { 0x3805, 0x4f }, { 0x3806, 0x05 }, { 0x3807, 0x23 },
	{ 0x3808, 0x06 }, { 0x3809, 0x40 }, { 0x380a, 0x05 }, { 0x380b, 0x14 },
	{ 0x380c, 0x03 }, { 0x380d, 0x88 }, { 0x380e, 0x05 }, { 0x380f, 0xc2 },
	{ 0x3810, 0x00 }, { 0x3811, 0x08 }, { 0x3812, 0x00 }, { 0x3813, 0x08 },
	{ 0x3814, 0x11 }, { 0x3815, 0x11 }, { 0x3816, 0x00 }, { 0x3817, 0x00 },
	{ 0x3818, 0x04 }, { 0x3819, 0x00 }, { 0x3820, 0x00 }, { 0x3821, 0x00 },
	{ 0x382b, 0x5a }, { 0x382c, 0x09 }, { 0x382d, 0x9a }, { 0x3882, 0x02 },
	{ 0x3883, 0x6c }, { 0x3885, 0x07 }, { 0x389d, 0x03 }, { 0x38a6, 0x00 },
	{ 0x38a7, 0x01 }, { 0x38b3, 0x07 }, { 0x38b1, 0x00 }, { 0x38e5, 0x02 },
	{ 0x38e7, 0x00 }, { 0x38e8, 0x00 }, { 0x3910, 0xff }, { 0x3911, 0xff },
	{ 0x3912, 0x08 }, { 0x3913, 0x00 }, { 0x3914, 0x00 }, { 0x3915, 0x00 },
	{ 0x391c, 0x00 }, { 0x3920, 0xff }, { 0x3921, 0x00 }, { 0x3922, 0x00 },
	{ 0x3923, 0x00 }, { 0x3924, 0x05 }, { 0x392d, 0x05 }, { 0x392e, 0xf2 },
	{ 0x392f, 0x40 }, { 0x4001, 0x00 }, { 0x4003, 0x40 }, { 0x4008, 0x12 },
	{ 0x4009, 0x1b }, { 0x400c, 0x0c }, { 0x400d, 0x13 }, { 0x4010, 0xf0 },
	{ 0x4011, 0x00 }, { 0x4016, 0x00 }, { 0x4017, 0x04 }, { 0x4042, 0x11 },
	{ 0x4043, 0x70 }, { 0x4045, 0x00 }, { 0x4409, 0x5f }, { 0x450b, 0x00 },
	{ 0x4600, 0x00 }, { 0x4601, 0xa0 }, { 0x4708, 0x09 }, { 0x470c, 0x81 },
	{ 0x4710, 0x06 }, { 0x4711, 0x00 }, { 0x4800, 0x00 }, { 0x481f, 0x30 },
	{ 0x4837, 0x14 }, { 0x4f00, 0x00 }, { 0x4f07, 0x00 }, { 0x4f08, 0x03 },
	{ 0x4f09, 0x08 }, { 0x4f0c, 0x06 }, { 0x4f0d, 0x02 }, { 0x4f10, 0x00 },
	{ 0x4f11, 0x00 }, { 0x4f12, 0x07 }, { 0x4f13, 0xe2 }, { 0x5000, 0x9f },
	{ 0x5001, 0x20 }, { 0x5026, 0x00 }, { 0x5c00, 0x00 }, { 0x5c01, 0x2c },
	{ 0x5c02, 0x00 }, { 0x5c03, 0x7f }, { 0x5e00, 0x00 }, { 0x5e01, 0x41 },
	{ 0x3501, 0x00 }, { 0x3502, 0x40 }, { 0x3508, 0x04 }, { 0x3006, 0x08 },
	{ 0x3925, 0x00 }, { 0x3926, 0x00 }, { 0x3927, 0x00 }, { 0x3928, 0x2e },
	{ 0x0100, 0x01 },
};

static int ov2311_ser_base_cfg(struct camera_dev *cam_dev)
{
	int i;
	int ret;
	int sensor_cfg_size;
	struct i2c_adapter *adap;
	u8 reg_value;

	if (strncmp(cam_dev->deser_parent->ctl_level, "fad-lis", 7) == 0)
		return 0;

	if (cam_dev->i2c_client == NULL)
		return -EINVAL;


	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	sensor_cfg_size = sizeof(sensor_base_settings) /
			  sizeof(struct ov2311_sensor_base_cfg);

	pr_info("sensor cfg size is %d", sensor_cfg_size);
	for (i = 0; i < 8; i++) {
		ret = bst_i2c_write_byte_data_byte_reg(
				    adap,
					cam_dev->ser_alias_id,
					template_ser[i][0],
					template_ser[i][1]);
		if (ret < 0) {
			pr_info(" serializer write error: %x ---> %x",
			       template_ser[i][0], template_ser[i][1]);
			return ret;
		}
		usleep_range(1000, 2000);
		//pr_info("camra ser is %x, one is %x,two is %x\r\n",cam_dev.ser_alias_id,template_ser[i][0],template_ser[i][1]);
	}
	usleep_range(1000, 2000);
	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			    adap,
				cam_dev->sensor_alias_id,
			    sensor_base_settings[i].reg,
			    sensor_base_settings[i].value);
		if (ret) {
			pr_info("%x,%x write error",
			       sensor_base_settings[i].reg,
			       sensor_base_settings[i].value);
			return ret;
		}
		ret = bst_i2c_read_byte_data_word_reg(
			    adap,
				cam_dev->sensor_alias_id,
			    sensor_base_settings[i].reg,
				&reg_value);
		if ((ret != 0) || (reg_value !=
		    sensor_base_settings[i].value)) {
			pr_info("readback %x error",
			       sensor_base_settings[i].reg);
			// reg 0x0103 readback is different, can't return;
			//return ret;
		}
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *sensor_ov2311;
	int ret = 0;

	if (!enable)
		return 0;


	sensor_ov2311 = container_of(sd, struct camera_dev, subdev);
	if (sensor_ov2311->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(sensor_ov2311);
	if (!ret)
		return -EINVAL;


	// select 10Mbps frequency for OV2311
	ti_serdes_bc_frequency_select(sensor_ov2311->deser_parent,
				      sensor_ov2311->index_in_serdes,
				      TI_DESER_BCC_FREQ_10MBPS);
	ret = ov2311_ser_base_cfg(sensor_ov2311);
	if (ret)
		return ret;

	sensor_ov2311->power_on = true;

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

static const struct media_entity_operations ov2311_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov2311_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *sensor_ov2311;
	struct device *dev = &client->dev;
	int ret;
	int bus_index;

	sensor_ov2311 = devm_kzalloc(&client->dev
		, sizeof(struct camera_dev)
		, GFP_KERNEL);
	if (!sensor_ov2311)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	sensor_ov2311->i2c_client = client;
	sensor_ov2311->dev = dev;
	bus_index = client->adapter->nr;
	sensor_ov2311->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	sensor_ov2311->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_WORD_REG_BYTE_DATA;
	sensor_ov2311->isp_data.sensorType = BST_SENSOR_TYPE_OV2311_RAW;

	ret = parse_camera_endpoint(sensor_ov2311, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(sensor_ov2311, &camera_ops
				, &ov2311_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int ov2311_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ov2311_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ov2311_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ov2311_of_match);
MODULE_DEVICE_TABLE(i2c, ov2311_id);

static struct i2c_driver ov2311_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ov2311_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ov2311_probe,
	.remove		= ov2311_remove,
	.id_table	= ov2311_id,
};

module_i2c_driver(ov2311_driver);

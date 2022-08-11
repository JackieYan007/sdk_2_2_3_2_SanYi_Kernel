// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ox3c raw camera for BST Camera Driver
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
#include "ox3c_config.h"
#include "camera_common_op.h"

#define MAX96717f_I2CADDR		0x40

#define MODULE_NAME "bst,ox3c"
#define MAX_SER_DEVICE_ID 0x0000
//#define GUANG_ZHEN_GPIO_ANBLE 0X12 //jinghua x8b 0x00,oufei x3c 0x00,maxieye x3c 0x00 ,sunyu x3c 0x00 ,guangzhen x3c 0x12

static int read_camera_ser_alias_id(struct camera_dev *cam_dev)
{
	int ret = -1;
	int retry_maxtimes = 10;
	u8 reg_value;

	while (retry_maxtimes-- > 0) {
		ret = bst_i2c_read_byte_data_word_reg(
			cam_dev->i2c_client->adapter, cam_dev->ser_alias_id,
			MAX_SER_DEVICE_ID, &reg_value);

		if (ret)
			usleep_range(1000, 2000);
		else if (ret == 0)
			break;
	}
	//modify ser_alias i2c address
	if (reg_value == (cam_dev->ser_alias_id << 1))
		return 0;

	return ret;
}

static int ox3c_ser_cfg(struct camera_dev *ox3c_raw)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox3c %s(), line %d\n", __func__, __LINE__);

	if (ox3c_raw == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}

	adap = ox3c_raw->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 16;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ox3c_raw->ser_alias_id,
				config_serdes_setting[i][0],
				config_serdes_setting[i][1]);
			if (ret) {
				pr_info(": write_MAX96717f_reg failed!\n");
				retry_times--;
				usleep_range(2000, 2500);
				continue;
			}
			pr_info("write MAX96717f reg:%#x, val:%#x",
			       config_serdes_setting[i][0],
			       config_serdes_setting[i][1]);
		}
	}
#ifdef GUANG_ZHEN_GPIO_ANBLE
	ret = bst_i2c_write_byte_data_word_reg(adap, ox3c_raw->ser_alias_id,
					       0x2d3, GUANG_ZHEN_GPIO_ANBLE);
	if (ret) {
		pr_info(": write_MAX96717f_reg failed!\n");
		usleep_range(2000, 2500);
	}
#endif
	return 0;
}

static int ox3c_power_on(struct camera_dev *cam_dev)
{
	int i, sensor_cfg_size;
	u8 reg_value;
	int ret;
	struct i2c_adapter *adap;

	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	sensor_cfg_size = sizeof(sensor_20fps_settings) /
			  sizeof(struct ox3c_sensor_base_cfg);

	//bst_i2c_write_byte_data_word_reg
	//bst_i2c_write_word_data_word_reg
	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			sensor_20fps_settings[i].reg,
			sensor_20fps_settings[i].value);

		if (ret) {
			dev_err(cam_dev->dev, "ox03c sensor %x,%x write failed",
				sensor_20fps_settings[i].reg,
				sensor_20fps_settings[i].value);
			continue;
		}

		usleep_range(1000, 2000);
		ret = bst_i2c_read_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			sensor_20fps_settings[i].reg, &reg_value);

		if (ret) {
			dev_err(cam_dev->dev, "ox3c sensor %x,%x read failed",
				sensor_20fps_settings[i].reg,
				sensor_20fps_settings[i].value);
			continue;
		}
		if (sensor_20fps_settings[i].value != reg_value) {
			dev_err(cam_dev->dev,
				"modify fail write reg value regadress=0x%x,orig value =0x%x,dump value=0x%x\n",
				sensor_20fps_settings[i].reg,
				sensor_20fps_settings[i].value, reg_value);
		}
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox3c_raw =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ox3c %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;

	if (!ox3c_raw->maxim_power_on) {
		ox3c_raw->power_on = false;
		return -EINVAL;
	}

	if (!is_slave_soc_model(ox3c_raw)) {
		ret = ox3c_ser_cfg(ox3c_raw);

		if (ret)
			return ret;

		ret = read_camera_ser_alias_id(ox3c_raw);

		if (ret == 0)
			ox3c_power_on(ox3c_raw);
	}
	ox3c_raw->power_on = true;

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

static const struct media_entity_operations ox3c_raw_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox3c_raw_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *ox3c_raw;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	ox3c_raw = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!ox3c_raw)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	ox3c_raw->i2c_client = client;
	ox3c_raw->dev = dev;
	bus_index = client->adapter->nr;

	//cam_dev->parent_iic_address =
	//	cam_dev->i2c_client->adapter->nr * IIC_OFFSET +
	//	IIC_BASE;

	ox3c_raw->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	ox3c_raw->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox3c_raw->isp_data.sensorType = BST_SENSOR_TYPE_OX3C_RAW;

	ret = parse_camera_endpoint(ox3c_raw, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox3c_raw, &camera_ops, &ox3c_raw_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ox3c_raw_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ox3c_raw_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox3c_raw_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ox3c_raw_of_match);
MODULE_DEVICE_TABLE(i2c, ox3c_raw_id);

static struct i2c_driver ox3c_raw_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox3c_raw_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox3c_raw_probe,
	.remove		= ox3c_raw_remove,
	.id_table	= ox3c_raw_id,
};

module_i2c_driver(ox3c_raw_driver);

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ox08b raw camera for BST Camera Driver
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
#include "ox08b_jinghua_config.h"
#include "camera_common_op.h"

#define MAX9295_ID_REG		0x0d
#define MAX9295_ID			0x91
#define MAX9295_I2CADDR		0x40
#define MODULE_NAME "bst,ox08b"

static int ox08b_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
{
	int value;

	if (of_property_read_s32(node, "clock-frequency", &value)) {
		dev_err(cam_dev->dev,
			"Invalid DT clock-frequency\n");
		return -EINVAL;
	}
	return value;
}

static int ox08b_ser_cfg(struct camera_dev *ox08b_raw)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (ox08b_raw == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}
	adap = ox08b_raw->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 12;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ox08b_raw->ser_alias_id,
				config_serdes_setting[i][0],
				config_serdes_setting[i][1]);
			if (ret) {
				pr_info(": write_max9295_reg failed!\n");
				retry_times--;
				usleep_range(2000, 2500);
				continue;
			}
			pr_info("write max9295 reg:%#x, val:%#x",
			       config_serdes_setting[i][0],
			       config_serdes_setting[i][1]);
		}
	}
	//modify sensor real i2c address
	bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0042,
					 (ox08b_raw->sensor_alias_id << 1));
	bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0043,
					 0x6c);

	return 0;
}

static int ox08b_power_on(struct camera_dev *cam_dev, struct device_node *node)
{
	int i, sensor_cfg_size;
	u8 reg_value;
	int ret;
	struct i2c_adapter *adap;
	struct ox08b_sensor_base_cfg *x8b_priv_setting;

	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	ret = ox08b_parse_dts(cam_dev, node);
	if (ret == 24) {
		x8b_priv_setting = sensor_base_settings_24m;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_24m);
	} else if (ret == 27) {
		x8b_priv_setting = sensor_base_settings_27m;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_27m);
	} else {
		pr_err("parse clock-frequency fail, error\n");
		return -EINVAL;
	}
	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x8b_priv_setting[i].reg,
			x8b_priv_setting[i].value);
		if (ret) {
			dev_err(cam_dev->dev, "ox08b sensor %x,%x write failed",
				x8b_priv_setting[i].reg,
				x8b_priv_setting[i].value);
			continue;
		}

		usleep_range(1000, 2000);
		ret = bst_i2c_read_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x8b_priv_setting[i].reg, &reg_value);

		if (ret) {
			dev_err(cam_dev->dev, "ox08b sensor %x,%x read failed",
				x8b_priv_setting[i].reg,
				x8b_priv_setting[i].value);
			continue;
		}
		if (x8b_priv_setting[i].value != reg_value) {
			pr_info("modify fail write reg value regadress=0x%x,orig value =0x%x,dump value=0x%x\n",
			       x8b_priv_setting[i].reg,
			       x8b_priv_setting[i].value, reg_value);
		}
	}
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox08b_raw =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;

	if (!ox08b_raw->maxim_power_on) {
		ox08b_raw->power_on = false;
		return -EINVAL;
	}

	if (!is_slave_soc_model(ox08b_raw)) {
		ret = ox08b_ser_cfg(ox08b_raw);
		if (ret)
			return ret;

		ox08b_power_on(ox08b_raw, ox08b_raw->dev->of_node);
	}
	ox08b_raw->power_on = true;

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

static const struct media_entity_operations ox08b_raw_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox08b_raw_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct camera_dev *ox08b_raw;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	ox08b_raw = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				 GFP_KERNEL);
	if (!ox08b_raw)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	ox08b_raw->i2c_client = client;
	ox08b_raw->dev = dev;
	bus_index = client->adapter->nr;

	//cam_dev->parent_iic_address =
	//	cam_dev->i2c_client->adapter->nr * IIC_OFFSET +
	//	IIC_BASE;

	ox08b_raw->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	ox08b_raw->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox08b_raw->isp_data.sensorType = BST_SENSOR_TYPE_OX08B_RAW;

	ret = parse_camera_endpoint(ox08b_raw, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox08b_raw, &camera_ops, &ox08b_raw_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ox08b_raw_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ox08b_raw_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox08b_raw_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ox08b_raw_of_match);
MODULE_DEVICE_TABLE(i2c, ox08b_raw_id);

static struct i2c_driver ox08b_raw_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox08b_raw_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox08b_raw_probe,
	.remove		= ox08b_raw_remove,
	.id_table	= ox08b_raw_id,
};

module_i2c_driver(ox08b_raw_driver);

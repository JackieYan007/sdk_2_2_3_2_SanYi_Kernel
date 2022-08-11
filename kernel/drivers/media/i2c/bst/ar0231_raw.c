// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ar0231 raw driver for BST Cameras Driver
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

#define MODULE_NAME "bst,ar0231"

struct ar0231_sensor_base_cfg {
	uint16_t reg;
	uint16_t value;
};

static uint8_t template_ser[6][2] = {
	{ 0x02, 0x73 }, { 0x03, 0x10 }, { 0x06, 0x41 },
	{ 0x07, 0x28 }, { 0x0e, 0x10 }, { 0x0d, 0x10 },
};

static struct ar0231_sensor_base_cfg sensor_base_settings[] = {
	{ 0x301A, 0x0018 }, { 0x3092, 0x0C24 }, { 0x30B0, 0x0902 },
	{ 0x3100, 0x4200 }, { 0x3102, 0x5000 }, { 0x337A, 0x0C80 },
	{ 0x3506, 0x3333 }, { 0x3508, 0x3333 }, { 0x350C, 0x055C },
	{ 0x3520, 0x1288 }, { 0x3522, 0x880C }, { 0x3524, 0x0C12 },
	{ 0x352C, 0x1212 }, { 0x354A, 0x007F }, { 0x3566, 0x3328 },
	{ 0x3566, 0x3328 }, { 0x32D0, 0x3A02 }, { 0x32D2, 0x3508 },
	{ 0x32D4, 0x3702 }, { 0x32D6, 0x3C04 }, { 0x32DC, 0x370A },
	{ 0x30B0, 0x0800 }, { 0x302A, 0x0008 }, { 0x302C, 0x0001 },
	{ 0x302E, 0x0003 }, { 0x3030, 0x0055 }, { 0x3036, 0x000C },
	{ 0x3038, 0x0001 }, { 0x30A2, 0x0001 }, { 0x30A6, 0x0001 },
	{ 0x3040, 0x0000 }, { 0x3082, 0x0008 }, { 0x30BA, 0x11E2 },
	{ 0x3044, 0x0400 }, { 0x3064, 0x1802 }, { 0x3180, 0x0080 },
	{ 0x33E4, 0x0080 }, { 0x33E0, 0x0C80 }, { 0x3004, 0x0004 },
	{ 0x3008, 0x0783 }, { 0x3002, 0x0040 }, { 0x3006, 0x0477 },
	{ 0x3402, 0x0780 }, { 0x3404, 0x0438 }, { 0x3082, 0x0008 },
	{ 0x30BA, 0x11E2 }, { 0x300C, 0x0BC0 }, { 0x300A, 0x057A },
	{ 0x3042, 0x0000 }, { 0x3238, 0x0222 }, { 0x3012, 0x0190 },
	{ 0x3014, 0x0C88 }, { 0x321E, 0x0C88 }, { 0x3222, 0x0C88 },
	{ 0x3226, 0x0C88 }, { 0x30B0, 0x0A00 }, { 0x32EA, 0x3C0E },
	{ 0x32EC, 0x72A1 }, { 0x31D0, 0x0000 }, { 0x31AE, 0x0204 },
	{ 0x31AC, 0x0C0C }, { 0x3342, 0x122C }, { 0x3346, 0x122C },
	{ 0x334A, 0x122C }, { 0x334E, 0x122C }, { 0x3344, 0x002c },
	{ 0x3348, 0x002c }, { 0x334C, 0x002c }, { 0x3350, 0x002c },
	{ 0x3352, 0x8000 }, { 0x31B0, 0x0053 }, { 0x31B2, 0x0039 },
	{ 0x31B4, 0x21C6 }, { 0x31B6, 0x2188 }, { 0x31B8, 0x3048 },
	{ 0x31BA, 0x0188 }, { 0x31BC, 0x8386 }, { 0x3012, 0x0218 },
	{ 0x3362, 0x0003 }, { 0x3082, 0x0018 }, { 0x3042, 0x0000 },
	{ 0x33f2, 0x0aaa }, { 0x3366, 0xaaaa }, { 0x301A, 0x001C },
	{ 0x3238, 0x8000 }, { 0x3012, 0x0200 }, { 0x3212, 0x0020 },
	{ 0x3216, 0x0002 }, { 0x3056, 0x0080 }, { 0x3058, 0x0080 },
	{ 0x305A, 0x0080 }, { 0x305C, 0x0080 }, { 0x3300, 0x0200 },
	{ 0x3302, 0x0200 }, { 0x3304, 0x0200 }, { 0x3306, 0x0200 },
	{ 0x3308, 0x0200 },
};

static int ar0231_power_on(struct camera_dev *cam_dev)
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
			  sizeof(struct ar0231_sensor_base_cfg);
	dev_info(cam_dev->dev, "%s() %d sensor cfg size is %d,ser iic address is %x",
	       __func__, __LINE__, sensor_cfg_size, cam_dev->ser_alias_id);
	for (i = 0; i < 6; i++) {
		ret = bst_i2c_write_byte_data_byte_reg(
			    adap,
			    cam_dev->ser_alias_id,
			    template_ser[i][0],
			    template_ser[i][1]);
		if (ret < 0) {
			dev_info(cam_dev->dev, " ar0231 write error: %x ---> %x",
			       template_ser[i][0], template_ser[i][1]);
			return ret;
		}
		usleep_range(1000, 2000);
		//printk("camra ser is %x, one is %x,two is %x\r\n",cam_dev.ser_alias_id,template_ser[i][0],template_ser[i][1]);
	}
	usleep_range(1000, 2000);

	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_word_data_word_reg(
			    adap, cam_dev->sensor_alias_id,
			    sensor_base_settings[i].reg,
			    sensor_base_settings[i].value);
		if (ret) {
			dev_info(cam_dev->dev, "%x,%x write error",
			       sensor_base_settings[i].reg,
			       sensor_base_settings[i].value);
			return ret;
		}
		usleep_range(1000, 2000);
		ret = bst_i2c_read_word_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			sensor_base_settings[i].reg,
			&reg_value);
		if ((ret != 0) || (reg_value != sensor_base_settings[i].value)) {
			dev_info(cam_dev->dev, "readback %x error,is %x,should %x",
			       sensor_base_settings[i].reg, ret,
			       sensor_base_settings[i].value);
			return ret;
		}
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *sensor_ar0231;
	int ret = 0;

	if (!enable)
		return 0;


	sensor_ar0231 = container_of(sd, struct camera_dev, subdev);
	if (sensor_ar0231->i2c_client == NULL)
		return -EINVAL;


	ret = is_camera_connected(sensor_ar0231);
	if (!ret)
		return -EINVAL;

	/*avoid setting sensor with double times*/
	if (!is_slave_soc_model(sensor_ar0231)) {
		ret = ar0231_power_on(sensor_ar0231);
		if (ret)
			return ret;

	}
	sensor_ar0231->power_on = true;

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
static int ar0231_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *sensor_ar0231;
	struct device *dev = &client->dev;
	int bus_index = 0;
	int ret;

	sensor_ar0231 = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!sensor_ar0231)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	sensor_ar0231->i2c_client = client;
	sensor_ar0231->dev = dev;
	bus_index = client->adapter->nr;

	ret = parse_camera_endpoint(sensor_ar0231, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(sensor_ar0231, &camera_ops
				, &ar0231_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	sensor_ar0231->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	sensor_ar0231->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	sensor_ar0231->isp_data.sensorType = BST_SENSOR_TYPE_AR0231_RAW;
	sensor_ar0231->isp_data.rawinfo.dvpDummyLines = 1;

	return 0;
}

static int ar0231_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ar0231_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ar0231_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ar0231_of_match);
MODULE_DEVICE_TABLE(i2c, ar0231_id);

static struct i2c_driver ar0231_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ar0231_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ar0231_probe,
	.remove		= ar0231_remove,
	.id_table	= ar0231_id,
};

module_i2c_driver(ar0231_driver);

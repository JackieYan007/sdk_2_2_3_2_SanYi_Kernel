// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * common camera operations for BST Cameras Driver
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

#include "../../platform/bst-a1000/cam_entity.h"
#include "camera_common_op.h"
#include "ti_deser_hub.h"
#include "maxim_deser_hub.h"

static inline int is_camera_data_raw(uint8_t type)
{
	return !((type == DT_UYVY) || (type == DT_YUYV));
}

static int read_view_dts_info(struct camera_dev *cam_dev,
				 struct device_node *node,
				 int view_id)
{
	int ret;
	u32 view_fmt;
	u32 rect_size[2] = { 0, 0 }; // width, height
	u32 crop_size[4] = { 0, 0, 0, 0 }; // top, bottom, left, right
	char prop_name[MAX_DTS_STRING_LEN];

	memset(prop_name, 0, MAX_DTS_STRING_LEN);
	snprintf(prop_name, MAX_DTS_STRING_LEN, "view%d-fmt", view_id);
	ret = of_property_read_u32(node, prop_name, &view_fmt);
	if (ret != 0) {
		// view not configured is ok, disable this view
		cam_dev->isp_data.viewinfo[view_id].viewFmt = View0_Dis;
		return 0;
	}
	pr_debug("view_id = %d, view_fmt = %d\n", view_id, view_fmt);
	if ((view_fmt < View0_YUVSep_Fmt) || (view_fmt > View0_Dis)) {
		pr_err("invalid view0_fmt, value = %d\n", view_fmt);
		cam_dev->isp_data.viewinfo[view_id].viewFmt = View0_Dis;
		return -1;
	}
	cam_dev->isp_data.viewinfo[view_id].viewFmt = view_fmt;

	memset(prop_name, 0, MAX_DTS_STRING_LEN);
	snprintf(prop_name, MAX_DTS_STRING_LEN, "view%d-size", view_id);
	pr_debug("prop_name = %s\n", prop_name);
	ret = of_property_read_u32_array(node, prop_name, rect_size, 2);
	if (ret == 0) {
		pr_debug("rect_size[0] = %d, rect_size[1] = %d\n",
			rect_size[0], rect_size[1]);
		cam_dev->isp_data.viewinfo[view_id].width = rect_size[0];
		cam_dev->isp_data.viewinfo[view_id].height = rect_size[1];
	} else {
		pr_err("read view %d size info failed\n", view_id);
		// this property is must, return error
		cam_dev->isp_data.viewinfo[view_id].viewFmt = View0_Dis;
		return -1;
	}

	memset(prop_name, 0, MAX_DTS_STRING_LEN);
	snprintf(prop_name, MAX_DTS_STRING_LEN, "view%d-crop", view_id);
	ret = of_property_read_u32_array(node, prop_name, crop_size, 4);
	if (ret == 0) {
		cam_dev->isp_data.viewinfo[view_id].topCropBefore = crop_size[0];
		cam_dev->isp_data.viewinfo[view_id].botCropBefore = crop_size[1];
		cam_dev->isp_data.viewinfo[view_id].lefCropBefore = crop_size[2];
		cam_dev->isp_data.viewinfo[view_id].rigCropBefore = crop_size[3];
	} else {
		// this property is optional, don't return error
	}

	return 0;
}

int is_slave_soc_model(struct camera_dev *camera)
{
	int ret;

	if (camera->deser_parent == NULL) {
		//pr_err("%s: camera deser is NULL",__func__);
		return 0;
	}

	if (strncmp(camera->deser_parent->ctl_level, "fad-lis", 7) == 0) {
		//pr_info("Slave Soc!\n");
		return 1;
	}
	//pr_err("Master Soc!\n");
	return 0;
}

int parse_camera_endpoint(struct camera_dev *cam_dev,
				 struct device_node *node)
{
	u32 value;
	//const char *type = NULL;
	const char *fpd3_mode = NULL;
	const char *serializer = NULL;
	const char *bin_file = NULL;
	const char *camera_name = NULL;
	u32 rect_size[2] = { 0, 0 }; // width, height
	int i;
	int ret;

	ret = of_property_read_string(node, "compatible", &camera_name);
	if (ret == 0) {
		if (camera_name != NULL) {
			strncpy(cam_dev->camera_name, camera_name,
				MAX_CAMERA_NAME_LEN);
		}
	}
	cam_dev->isp_data.rawinfo.srcSel = BST_ISP_INPUT_MIPI;
	ret = of_property_read_u32(node, "data-type", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.dataType = value;

	ret = of_property_read_u32(node, "ser-alias-id", &value);
	if (ret == 0)
		cam_dev->ser_alias_id = value;

	ret = of_property_read_u32(node, "sensor-id", &value);
	if (ret == 0)
		cam_dev->sensor_id = value;

	ret = of_property_read_u32(node, "sensor-alias-id", &value);
	if (ret == 0)
		cam_dev->sensor_alias_id = value;

	if (is_camera_data_raw(cam_dev->isp_data.rawinfo.dataType))
		cam_dev->isp_data.sensorDevID = cam_dev->sensor_alias_id;
	else
		cam_dev->isp_data.sensorDevID = 0x0;

	ret = of_property_read_u32(node, "fv-polarity-low", &value);
	if (ret == 0)
		cam_dev->fv_polarity_low = value;

	cam_dev->frame_valid_min = TI_DESER_FRAME_VALID_MIN_DEFAULT;
	ret = of_property_read_u32(node, "frame-valid-min", &value);
	if (ret == 0)
		cam_dev->frame_valid_min = value;

	ret = of_property_read_string(node, "fpd3-mode", &fpd3_mode);
	if (ret == 0) {
		if (fpd3_mode != NULL) {
			strncpy(cam_dev->fpd3_mode, fpd3_mode,
				MAX_DTS_STRING_LEN);
		}
	}
	ret = of_property_read_string(node, "serializer", &serializer);
	if (ret == 0) {
		if (serializer != NULL) {
			strncpy(cam_dev->serializer, serializer,
				MAX_DTS_STRING_LEN);
		}
	}
	ret = of_property_read_string(node, "algo-online", &bin_file);
	if (ret == 0) {
		if (bin_file != NULL) {
			strncpy(cam_dev->algo_online, bin_file,
				MAX_BIN_NAME_LEN);
		}
	}
	ret = of_property_read_string(node, "iq-online", &bin_file);
	if (ret == 0) {
		if (bin_file != NULL) {
			strncpy(cam_dev->iq_online, bin_file,
				MAX_BIN_NAME_LEN);
		}
	}
	ret = of_property_read_string(node, "algo-offline", &bin_file);
	if (ret == 0) {
		if (bin_file != NULL) {
			strncpy(cam_dev->algo_offline, bin_file,
				MAX_BIN_NAME_LEN);
		}
	}
	ret = of_property_read_string(node, "iq-offline", &bin_file);
	if (ret == 0) {
		if (bin_file != NULL) {
			strncpy(cam_dev->iq_offline, bin_file,
				MAX_BIN_NAME_LEN);
		}
	}
	ret = of_property_read_string(node, "pwl-lut", &bin_file);
	if (ret == 0) {
		if (bin_file != NULL) {
			strncpy(cam_dev->pwl_lut, bin_file,
				MAX_BIN_NAME_LEN);
		}
	}
	cam_dev->trigger_gpio = -1;
	ret = of_property_read_u32(node, "tri-gpio", &value);
	if (ret == 0)
		cam_dev->trigger_gpio = value;

	// default hdrStaggerEn is 1
	cam_dev->isp_data.rawinfo.hdrMode = 1;
	ret = of_property_read_u32(node, "hdr-stagger-en", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.hdrMode = value;

	ret = of_property_read_u32(node, "dvp-dummy", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.dvpDummyVal = value;

	// default exposure number is 1
	cam_dev->isp_data.rawinfo.expNum = 1;
	ret = of_property_read_u32(node, "exp-num", &value);
	if (ret == 0)
		if ((value >= 0) && (value <= 3))
			cam_dev->isp_data.rawinfo.expNum = value;

	ret = of_property_read_u32(node, "isp-pwl-infomat", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.ispPwlInFormat = value;

	ret = of_property_read_u32(node, "dvp-data-type", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.dvpDataType = value;

	ret = of_property_read_u32(node, "vin-data-type", &value);
	if (ret == 0)
		cam_dev->isp_data.rawinfo.vinDataType = value;

	ret = of_property_read_u32_array(node, "size", rect_size, 2);
	if (ret == 0) {
		cam_dev->isp_data.rawinfo.width = rect_size[0];
		cam_dev->isp_data.rawinfo.height = rect_size[1];
	} else {
		pr_err("read camera size info failed\n");
		return -1;
	}
	for (i = 0; i < MAX_VIEWS_PER_CAMERA; i++) {
		ret = read_view_dts_info(cam_dev, node, i);
		if (ret != 0) {
			pr_err("read_view_dts_info failed\n");
			return ret;
		}
	}
	ret = of_property_read_u32(node, "pdns-mode", &value);
	if (ret == 0) {
		if ((value >= 0) && (value <= 1)) {
			cam_dev->isp_data.pdnsinfo.pdnsMode = value;
		} else {
			pr_err("invalid pdns_mode, value = %d,  disable pdns\n", value);
			cam_dev->isp_data.pdnsinfo.pdnsMode = 0;
		}
	}

	ret = of_property_read_u32(node, "pdns-input-view", &value);
	if (ret == 0) {
		if ((value >= 0) && (value <= 1)) {
			cam_dev->isp_data.pdnsinfo.pdnsViewSel = value;
		} else {
			pr_err("invalid pdns_input_view, value = %d, disable pdns\n", value);
			cam_dev->isp_data.pdnsinfo.pdnsViewSel = 0;
			cam_dev->isp_data.pdnsinfo.pdnsMode = 0;
		}
	}
	ret = of_property_read_u32(node, "hblank", &value);
	if (ret == 0)
		cam_dev->isp_data.hblank = value;

	ret = of_property_read_u32(node, "isp-top-crop", &value);
	if (ret == 0)
		cam_dev->isp_data.ispInTopCrop = value;

	ret = of_property_read_u32(node, "isp-bot-crop", &value);
	if (ret == 0)
		cam_dev->isp_data.ispInBotCrop = value;

	ret = of_property_read_u32(node, "isp-lef-crop", &value);
	if (ret == 0)
		cam_dev->isp_data.ispInLefCrop = value;

	ret = of_property_read_u32(node, "isp-rig-crop", &value);
	if (ret == 0)
		cam_dev->isp_data.ispInRigCrop = value;

	cam_dev->configured = true;

	return 0;
}

int init_camera_dev(struct camera_dev *cam_dev
	, const struct v4l2_subdev_ops *subdev_ops
	, const struct media_entity_operations *media_ops)
{
	struct v4l2_subdev *sd;
	int ret;

	cam_dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	atomic_set(&(cam_dev->is_streaming), 0);
	sd = &(cam_dev->subdev);
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, subdev_ops);
	sd->dev = cam_dev->dev;
	v4l2_set_subdevdata(sd, cam_dev); //set the camera common data
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = media_ops;
	sd->fwnode = of_fwnode_handle(cam_dev->dev->of_node);
	snprintf(sd->name, sizeof(sd->name), "%s-%x",
		dev_name(cam_dev->dev),
		cam_dev->i2c_client->addr);
	/* Initialize media entity */
	//ret = media_entity_init(, 0);

	ret = media_entity_pads_init(&sd->entity, 1, &(cam_dev->pad));
	if (ret < 0)
		return ret;

	// :: todo ::
	// check camera connection status by camera id
	// camera_id = i2c_smbus_read_byte_data(cam_dev->i2c_client, reg);

	cam_dev->connected = true;
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		pr_err("failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

static const struct media_entity_operations default_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static inline int init_camera_dev_default(struct camera_dev *cam_dev
	, const struct v4l2_subdev_ops *subdev_ops)
{
	return init_camera_dev(cam_dev, subdev_ops, &default_media_ops);
}

// write 8bit data to 8bit register
int bst_i2c_write_byte_data_byte_reg(struct i2c_adapter *adap,
		uint8_t slave_address,
		uint8_t reg_offset, uint8_t value)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[2];

	msg.addr = slave_address; /* I2C address of chip */
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	data[0] = reg_offset; /* register num */
	data[1] = value; /* register data */
	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	return 0;
}

// read 8bit data from 8bit register
int bst_i2c_read_byte_data_byte_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint8_t reg_offset
		, uint8_t *out_value)
{
	int ret;
	uint8_t value;
	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 1,
					  .buf = &reg_offset,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 1,
					  .buf = &value,
				  } };

	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		pr_info("i2c_transfer send error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		pr_info("i2c_transfer read error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
	if (out_value)
		*out_value = value;

	return 0;
}

// write 8bit data to 16bit register
int bst_i2c_write_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t value)
{
	//u64 time_start;
	//u64 time_end;
	int ret;
	struct i2c_msg msg;
	unsigned char data[3];

	data[0] = (uint8_t)((reg_offset >> 8) & 0xff); /* register addr */
	data[1] = (uint8_t)(reg_offset & 0xff); /* register addr */
	data[2] = value;

	msg.addr = slave_address;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	//time_start = ktime_get_ns();
	ret = i2c_transfer(adap, &msg, 1);
	//time_end = ktime_get_ns();
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
	//pr_err("i2ctransfer spend time :%ld ns\n", (time_end - time_start));
	return 0;
}

// read 8bit data from 16bit register
int bst_i2c_read_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t *out_value)
{
	int ret;
	uint8_t value;
	uint8_t reg_array[2];
	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 2,
					  .buf = reg_array,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 1,
					  .buf = &value,
				  } };

	reg_array[0] = (uint8_t)((reg_offset >> 8) & 0xff);
	reg_array[1] = (uint8_t)(reg_offset & 0xff);

	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}
	if (out_value)
		*out_value = value;

	return 0;
}

// write 16bit data to 16bit register
int bst_i2c_write_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t value)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[4];

	data[0] = (uint8_t)((reg_offset >> 8) & 0xff); /* register addr */
	data[1] = (uint8_t)(reg_offset & 0xff); /* register addr */
	data[2] = (uint8_t)((value >> 8) & 0xff);
	data[3] = (uint8_t)(value & 0xff);
	msg.addr = slave_address; /* I2C address of chip */
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	return 0;
}

// read 16bit data from 16bit register
int bst_i2c_read_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t *out_value)
{
	int ret;
	uint16_t value;
	uint8_t read_value[2];
	uint8_t reg_array[2];

	struct i2c_msg msg[2] = { {
					  .addr = slave_address,
					  .flags = 0,
					  .len = 2,
					  .buf = reg_array,
				  },
					{
					  .addr = slave_address,
					  .flags = I2C_M_RD,
					  .len = 2,
					  .buf = read_value,
				  } };

	reg_array[0] = (uint8_t)((reg_offset >> 8) & 0xff);
	reg_array[1] = (uint8_t)(reg_offset & 0xff);

	ret = i2c_transfer(adap, &msg[0], 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	ret = i2c_transfer(adap, &msg[1], 1);
	if (ret != 1) {
		pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
			slave_address, reg_offset, ret);
		return -EIO;
	}

	value = (uint16_t)(read_value[0] << 8 | read_value[1]);
	if (out_value)
		*out_value = value;

	return 0;
}

static int read_camera_ser_alias_id(struct camera_dev *cam_dev)
{
	int ret = -1;
	int retry_maxtimes = 20;
	u8 reg_value;
	struct deser_hub_dev *hub = cam_dev->deser_parent;
	int index = cam_dev->index_in_serdes;
	int tmp = (1 << index | index << 4);

	while ((retry_maxtimes-- > 0) && ret) {
		ret = bst_i2c_write_byte_data_byte_reg(
			hub->i2c_client->adapter, hub->i2c_client->addr,
			TI_DESER_PORT_SELECT, tmp);
		if (ret)
			usleep_range(1000, 2000);
		else if (ret == 0)
			break;
	}
	if (ret) {
		pr_err("write deser error\n");
		return -1;
	}

	ret = -1;
	reg_value = 0; // must init the value, judge it late
	retry_maxtimes = 20; // reset retry times
	while ((retry_maxtimes-- > 0) && ret) {
		ret = bst_i2c_read_byte_data_byte_reg(
			hub->i2c_client->adapter, hub->i2c_client->addr,
			TI_DESER_SER_ID_REG, &reg_value);
		if (ret)
			usleep_range(1000, 2000);
		else if (ret == 0)
			break;
	}
	if (ret) {
		pr_err("write deser error\n");
		return -1;
	}
	if (reg_value != 0)
		return 0;
	else
		return 1;
}

int is_camera_connected(struct camera_dev *cam_dev)
{
	int index = cam_dev->index_in_serdes;
	struct deser_hub_dev *hub = cam_dev->deser_parent;
	int status = CAMERA_STATUS_UN;
	int ret;

	switch (hub->type) {
	case DESER_TYPE_TI954:
		ret = read_camera_ser_alias_id(cam_dev);
		status = ret ? CAMERA_STATUS_NG : CAMERA_STATUS_OK;
		break;
	case DESER_TYPE_TI960:
		ret = serdes_read(hub, 0x00);
		if(ret < 0) {
			dev_info(hub->dev, "Deser Device not link\n");
			status = CAMERA_STATUS_NG;
			break;
		}

		if (!strncmp(hub->ctl_level, "fad-ctl", MAX_DTS_STRING_LEN)) {
			ret = read_camera_ser_alias_id(cam_dev);
			status = ret ? CAMERA_STATUS_NG : CAMERA_STATUS_OK;
			write_camera_connect_status(hub, index, status);
		} else if (!strncmp(hub->ctl_level, "fad-lis", MAX_DTS_STRING_LEN)) {
			int retry_maxtimes = 24;

			while (retry_maxtimes > 0) {
				status = read_camera_connect_status(hub, index);
				if (status)
					break;
				/* bug 4693 */
				/* Shorten scan time and prevent slow startup, but TI964 has a RAW camera read/write failure problem */
				usleep_range(5000, 6000);

				retry_maxtimes--;
			}
		} else {
			ret = read_camera_ser_alias_id(cam_dev);
			status = ret ? CAMERA_STATUS_NG : CAMERA_STATUS_OK;
		}
		break;
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
			pr_err("%s() %d, maxim connect detect\n", __func__, __LINE__);
			struct deser_hub_dev *hub = cam_dev->deser_parent;
			ret = is_gmsl2_video_connected(hub, index);
			status = ret ? CAMERA_STATUS_OK : CAMERA_STATUS_NG;
			break;
	}

	switch (status) {
	case CAMERA_STATUS_UN:
		pr_err("port %d camera status: unknown\n", index);
		break;
	case CAMERA_STATUS_OK:
		pr_err("port %d camera status: connected\n", index);
		break;
	case CAMERA_STATUS_NG:
		pr_err("port %d camera status: disconnected\n", index);
		break;
	}

	return (status == CAMERA_STATUS_OK);
}


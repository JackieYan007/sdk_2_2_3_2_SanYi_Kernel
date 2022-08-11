/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Camera entity definitions for BST
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

#ifndef __A1000_CAM_ENTITY_H__
#define __A1000_CAM_ENTITY_H__

#include <linux/types.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "proto_isp_ipc.h"

#define MAX_BIN_NAME_LEN     128
#define MAX_CAMERA_NAME_LEN  32
#define MAX_DTS_STRING_LEN   16
#define MAX_VIEWS_PER_CAMERA 3

#define IIC_OFFSET 0x1000
#define IIC_BASE   0x20000000

struct deser_hub_dev;

enum {
	BST_SUBDEV_STATE_UNKNOWN,
	BST_SUBDEV_STATE_ENABLED,
	BST_SUBDEV_STATE_BOUND,
	BST_SUBDEV_STATE_UNBIND,
};

// 0-mipi/1-HDMI input/2-FILE2FILE HDR/3-FILE2FILE SINGLE, from proto_isp_ipc.h
enum {
	BST_ISP_INPUT_MIPI = 0,
	BST_ISP_INPUT_HDMI,
	BST_ISP_INPUT_FILE2FILE_HDR,
	BST_ISP_INPUT_FILE2FILE_SINGLE,
};

enum {
	// 16 bit register addr and 16 bit data
	BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA = 0,
	// 16 bit register addr and 8 bit data
	BST_SENSOR_RW_MODE_WORD_REG_BYTE_DATA,
	// 8 bit register addr and 8 bit data
	BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA,
};

enum {
	BST_SENSOR_TYPE_OV2770_RAW = 0,
	BST_SENSOR_TYPE_OV10640_RAW = 1,
	BST_SENSOR_TYPE_AR0231_RAW = 2,
	BST_SENSOR_TYPE_ASX340_RAW = 3,
	BST_SENSOR_TYPE_AR0144_RAW = 5,
	BST_SENSOR_TYPE_OV2311_RAW = 6,
	BST_SENSOR_TYPE_IMX390_RAW = 8,
	BST_SENSOR_TYPE_AR0233_RAW = 9,
	BST_SENSOR_TYPE_YUV422 = 15,
	BST_SENSOR_TYPE_IMX424_RAW = 16,
	BST_SENSOR_TYPE_OV10652_RAW = 128,
	BST_SENSOR_TYPE_OX3C_RAW = 136,
	BST_SENSOR_TYPE_OX08B_RAW = 137,
};

struct camera_dev;

struct camera_ops {
	int (*s_register)(struct camera_dev *cam, uint16_t regaddr,
			  uint16_t regval);
	int (*g_register)(struct camera_dev *cam, uint16_t regaddr,
			  uint16_t *regval);
	int (*s_brightness)(struct camera_dev *cam, int val);
	int (*g_brightness)(struct camera_dev *cam, int *val);
	int (*s_contrast)(struct camera_dev *cam, int val);
	int (*g_contrast)(struct camera_dev *cam, int *val);
	int (*s_saturation)(struct camera_dev *cam, int val);
	int (*g_saturation)(struct camera_dev *cam, int *val);
	int (*s_hue)(struct camera_dev *cam, int val);
	int (*g_hue)(struct camera_dev *cam, int *val);
	int (*s_aec)(struct camera_dev *cam, int val);
	int (*g_aec)(struct camera_dev *cam, int *val);
	int (*s_awb)(struct camera_dev *cam, int val);
	int (*g_awb)(struct camera_dev *cam, struct v4l2_ctrl *val);
	int (*s_gamma)(struct camera_dev *cam, int val);
	int (*g_gamma)(struct camera_dev *cam, struct v4l2_ctrl *val);
};

struct camera_dev {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct i2c_client *i2c_client;
	struct device *dev;
	ipc_reconf_t isp_data;
	char camera_name[MAX_CAMERA_NAME_LEN];
	char algo_online[MAX_BIN_NAME_LEN];
	char iq_online[MAX_BIN_NAME_LEN];
	char algo_offline[MAX_BIN_NAME_LEN];
	char iq_offline[MAX_BIN_NAME_LEN];
	char pwl_lut[MAX_BIN_NAME_LEN];
	char type_name[MAX_DTS_STRING_LEN];
	int ser_alias_id;
	int sensor_id;
	int sensor_alias_id;
	int index_in_serdes;
	u32 parent_iic_address;
	struct deser_hub_dev *deser_parent; // the deserializer that camera
					    // connected
	atomic_t is_streaming;
	bool maxim_power_on;
	bool power_on;
	bool configured;
	bool connected;
	int cap_buf_shift;
	int sd_state;
	uint32_t fv_polarity_low;
	uint32_t frame_valid_min;
	uint32_t trigger_gpio;
	char fpd3_mode[MAX_DTS_STRING_LEN];
	char serializer[MAX_DTS_STRING_LEN];
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	struct camera_ops ops;
};

#endif /* __A1000_CAM_ENTITY_H__ */

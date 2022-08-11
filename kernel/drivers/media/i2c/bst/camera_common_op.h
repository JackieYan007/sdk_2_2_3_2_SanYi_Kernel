/* SPDX-License-Identifier: GPL-2.0-or-later */
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
#ifndef __BST_CAMERA_COMMON_OP_H_

#define __BST_CAMERA_COMMON_OP_H_

#include "../../platform/bst-a1000/cam_entity.h"

// TI913, 933, 953, device id reg addr are all 0x00
#define TI_SER_DEVICE_ID 0x00

int parse_camera_endpoint(struct camera_dev *cam_dev,
				 struct device_node *node);

int init_camera_dev(struct camera_dev *cam_dev
	, const struct v4l2_subdev_ops *subdev_ops
	, const struct media_entity_operations *media_ops);

int is_camera_connected(struct camera_dev *cam_dev);
extern int isp_internal_trigger(int target_freq, int fysnc_in, int fsync_out);
extern int isp_external_trigger(int external_freq, int target_freq, int fsync_in, int fsync_out);

/*
 *Judge whether current SOC model is slave model
 */
int is_slave_soc_model(struct camera_dev *camera);

// write 8bit data to 8bit register
int bst_i2c_write_byte_data_byte_reg(
	    struct i2c_adapter *adap,
		uint8_t slave_address,
		uint8_t reg_offset,
		uint8_t value);

// read 8bit data from 8bit register
int bst_i2c_read_byte_data_byte_reg(
		struct i2c_adapter *adap
		, uint8_t slave_address
		, uint8_t reg_offset
		, uint8_t *out_value);

// write 8bit data to 16bit register
int bst_i2c_write_byte_data_word_reg(
	    struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t value);

// read 8bit data from 16bit register
int bst_i2c_read_byte_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint8_t *out_value);

// write 16bit data to 16bit register
int bst_i2c_write_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t value);

// read 16bit data from 16bit register
int bst_i2c_read_word_data_word_reg(struct i2c_adapter *adap
		, uint8_t slave_address
		, uint16_t reg_offset
		, uint16_t *out_value);

#endif // __BST_CAMERA_COMMON_OP_H_

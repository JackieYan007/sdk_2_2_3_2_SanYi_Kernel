/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ti_deser_hub for BST Deserializer Driver
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
#ifndef _BST_COMMON_DESER_H_

#define _BST_COMMON_DESER_H_

#include "../../platform/bst-a1000/cam_entity.h"
#include "../../platform/bst-a1000/csi.h"
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <linux/mutex.h>

#define MAX_CAMERAS_PER_SERDES 4
#define MAX_DESER_NAME_LEN 16

enum { DESER_TYPE_INVALID = 0,
		DESER_TYPE_TI954 = 1,
		DESER_TYPE_TI960 = 2,
		DESER_TYPE_MAX9286,
		DESER_TYPE_MAX9296,
		DESER_TYPE_MAX96712,
		DESER_TYPE_MAX96722
};

enum { SER_TYPE_INVALID = 0,
		SER_TYPE_MAX96701,
		SER_TYPE_MAX96705,
		SER_TYPE_MAX9295,
		SER_TYPE_MAX96717f,
};

enum {
	DESER_TRIGGER_MODE_NONE = 0,
	DESER_TRIGGER_MODE_INTERNAL = 1,
	DESER_TRIGGER_MODE_EXTERNAL = 2,
};

struct deser_hub_dev;

struct deser_channel {
	struct v4l2_async_subdev async_dev;
	struct deser_hub_dev *deser_dev;
	struct camera_dev *cam_dev;
	struct device_node *camera_node;
	struct fwnode_handle *camera_fwnode;
	int index;
	u32 csi_vc;
	bool camera_bound; // sub-dev bound
};

struct deser_trigger_info {
	int trigger_mode;
	int trigger_fps;
	int trigger_tx_gpio;
	int trigger_rx_gpio;
	int target_freq;
	int fsync_in;
	int fsync_out;
	int external_freq;
};

enum ctl_mode_t {
	NONE,
	FAD_CTL_MODE,
	FAD_LIS_MODE,
};

struct deser_hub_dev {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct bst_csi_device *csi_dev;
	struct v4l2_subdev subdev;
	struct v4l2_async_notifier notifier;
	struct deser_channel chn[MAX_CAMERAS_PER_SERDES];
	struct deser_trigger_info trig_info;
	struct mutex deser_mutex;
	char name[MAX_DESER_NAME_LEN];
	char ctl_level[MAX_DTS_STRING_LEN];
	enum ctl_mode_t ctl_mode;
	int type;
	int data_lanes_num;
	int speed;
	int max_port;
	int src_mask;
	int lane_speed;
	int i2c_port;
	int csi2_port;
	int num_cameras;
	int sd_state;
	bool deser_boot_flag;
	bool deser_stream_flag;
	bool frame_sync_enabled;
	bool fwd_sync_enabled;
	int (*internal_trigger_sync)(struct deser_hub_dev *hub, int camera_gpio, int fps);
	int (*external_trigger_sync)(struct deser_hub_dev *hub, int external_freq, int target_freq, int fsync_in, int fsync_out,
								 int camera_trigger_gpio, int deser_trigger_gpio);
};

#endif // _BST_COMMON_DESER_H_

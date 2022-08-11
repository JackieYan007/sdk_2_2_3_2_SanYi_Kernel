/* SPDX-License-Identifier: GPL-2.0 */

/*
 * CSI definitions for BST
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

#ifndef __BST_CSI_H__
#define __BST_CSI_H__

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "../../i2c/bst/common_deser_hub.h"
#include "cam_entity.h"

#define MAX_VC_PER_CSI 4

enum {
	CSI_CHANNEL_SINK_PAD = 0,
	CSI_CHANNEL_SOURCE_PAD = 1,
	CSI_CHANNEL_PAD_NUM,
};

enum {
	CSI_STATE_INVALID,
	CSI_STATE_INITED,
};

// csi channel as a media entity
struct bst_csi_channel {
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_entity entity;
#endif
	struct media_pad pads[CSI_CHANNEL_PAD_NUM];
	struct camera_dev *cam_dev;
	struct bst_csi_device *csi;
	bool connected;
	int state;
	int csi_dev_id;
	int index;
	int csi_chn_id;
	int sn_in_all_csi;
	atomic_t is_streaming;
};

struct bst_csi_device {
	struct device *dev;
	struct platform_device *pdev;
	struct deser_hub_dev *deser;
	char devname[32];
	struct v4l2_subdev subdev;
	struct v4l2_async_subdev async_dev;
	struct v4l2_async_notifier notifier;
	struct device_node *of_node;
	struct fwnode_handle *csi_fwnode;
	struct fwnode_handle *remote_fwnode;

	struct bst_csi_channel csi_vc[MAX_VC_PER_CSI];
	// struct fwnode_handle *cam_fwnode[MAX_VC_PER_CSI];
	struct mutex mutex; /* format and crop settings */
	int state;
	int sd_state;
	int csi_id;
	int lane_speed;
	int num_vc; // number of virtual channel
	int num_lanes;
	atomic_t refcount;
};

int update_camera_status_in_csi(struct bst_csi_device *pcsi_dev);

#endif // __BST_CSI_H__

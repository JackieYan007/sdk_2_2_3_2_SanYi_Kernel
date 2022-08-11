/* SPDX-License-Identifier: GPL-2.0 */

/*
 * ISP video userspace API definitions for BST
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

#ifndef __ISP_VIDEO_UAPI_H__
#define __ISP_VIDEO_UAPI_H__

/* for struct media_command definition */
#include <linux/coreip/proto_api_common.h>

struct isp_ipcmsg {
	struct media_command cmd;
	int reply;
};

struct isp_regcfg {
	uint16_t regaddr;
	uint16_t regval;
};

struct abnormal_info {
	uint8_t abnormal_id;
	uint8_t abnormal_type;
	uint32_t last_good_sequence;
	uint32_t total_bad_frames;
	uint32_t total_frames;
} __packed;

struct scale_size {
	uint16_t width;
	uint16_t height;
};

struct crop_size {
	uint16_t topCropBefore;
	uint16_t botCropBefore;
	uint16_t lefCropBefore;
	uint16_t rigCropBefore;
	uint16_t topCropAfter;
	uint16_t botCropAfter;
	uint16_t lefCropAfter;
	uint16_t rigCropAfter;
};

struct resolution_resize {
	struct crop_size crop_size;
	struct scale_size scale_size;
	uint8_t view_id;
};

/* v4l2 ioclt private definitions for bst isp */
/* print debug mode status info */
#define ISPIOC_PRINT_DBGINFO _IO('V', BASE_VIDIOC_PRIVATE + 0)
/* enter debug mode */
#define ISPIOC_ENTER_DBGMODE _IO('V', BASE_VIDIOC_PRIVATE + 1)
/* lease debug mode */
#define ISPIOC_LEAVE_DBGMODE _IO('V', BASE_VIDIOC_PRIVATE + 2)
/* alloc debug memory */
#define ISPIOC_ALLOC_DBGMEM  _IOW('V', BASE_VIDIOC_PRIVATE + 3, size_t)
/* release debug memory */
#define ISPIOC_FREE_DBGMEM   _IO('V', BASE_VIDIOC_PRIVATE + 4)
/* query debug memory physical address */
#define ISPIOC_QUERY_DBGMEM  _IOR('V', BASE_VIDIOC_PRIVATE + 5, uint32_t)
/* transfer isp firmware ipc message */
#define ISPIOC_TRANS_IPCMSG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct isp_ipcmsg)

/* transfer isp firmware ipc message */
#define ISPIOC_G_CAMREG _IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct isp_regcfg)
/* transfer isp firmware ipc message */
#define ISPIOC_S_CAMREG _IOWR('V', BASE_VIDIOC_PRIVATE + 11, struct isp_regcfg)
/* get abnormal info */
#define ISPIOC_G_ABNORMAL_INFO \
	_IOR('V', BASE_VIDIOC_PRIVATE + 12, struct abnormal_info)
/* set resize res info */
#define ISPIOC_RESIZE_RESOLUTION \
	_IOW('V', BASE_VIDIOC_PRIVATE + 15, struct resolution_resize)

struct isp_ctrl {
	uint32_t value; // isp_set_iqinfo_t  iqVal  ,if ctrl_addr == 0  ; use
			// value
	uint16_t aecManualExp[3];
	uint16_t aecManualGain[3];
	uint16_t manualAWBGain[3][3]; // set mannual white balance value
};

/* The base for isp v4l2 controls. */
#define V4L2_CID_USER_ISP_BASE		     (V4L2_CID_USER_BASE + 0x1090)
#define V4L2_CID_ISP_TEST		     (V4L2_CID_USER_ISP_BASE + 0)
#define V4L2_CID_ISP_MANUAL_WB		     (V4L2_CID_USER_ISP_BASE + 1)
#define V4L2_CID_ISP_MANUAL_EXPOSURE	     (V4L2_CID_USER_ISP_BASE + 2)
#define V4L2_CID_ISP_YDNS		     (V4L2_CID_USER_ISP_BASE + 3)
#define V4L2_CID_ISP_UVDNS		     (V4L2_CID_USER_ISP_BASE + 4)
/* The base for isp v4l2 controls types */
#define V4L2_CTRL_ISP_TYPE_BASE		     (V4L2_CTRL_COMPOUND_TYPES + 0x10)
#define V4L2_CTRL_ISP_TYPE_TEST		     (V4L2_CTRL_ISP_TYPE_BASE + 0)
#define V4L2_CTRL_ISP_TYPE_MANUAL_WB	     (V4L2_CTRL_ISP_TYPE_BASE + 1)
#define V4L2_CTRL_ISP_TYPE_MANUAL_EXPOSURE   (V4L2_CTRL_ISP_TYPE_BASE + 2)
#define V4L2_CTRL_ISP_TYPE_RESIZE_RESOLUTION (V4L2_CTRL_ISP_TYPE_BASE + 3)
#endif

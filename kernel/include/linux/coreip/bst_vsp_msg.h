/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VSP IPC message interface for BST
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

#ifndef __BST_VSP_MSG_H__
#define __BST_VSP_MSG_H__

#include <linux/coreip/proto_api_common.h>

typedef void (*callback)(struct media_command *);

enum vsp_core_id {
	VSP_CORE_VOUT,
	VSP_CORE_GMWARP,
	VSP_CORE_ENC,
	VSP_CORE_MAX,
};

struct vsp_msg_data {
	enum vsp_core_id core_id;
	int cmdid;
	char src[4];
	char dst[4];
	uint32_t follow_pack_num;
	uint32_t cmd_type_main;
	uint32_t cmd_type_minor;
	uint32_t user_cmd_data[32];

	struct list_head node;
	char reserved[8];
};

enum output_mode_t {
	LCD_1CLK = 0,
	LCD_3CLK,
	LCD_4CLK,
	LCD_565,
	YUV656,
	YUV601,
	D601_24,
	D601_8,
	BAYER,
	MIPI_YUV422,
	MIPI_RGB565,
	MIPI_RAW8,
	MIPI_RGB888,

	HDMI_YUV444 = 0x10,
	HDMI_RGB,
	HDMI_YC422,
};

enum osd_format_t {
	OSD_16BIT_565_VYU_RGB = 0,
	OSD_16BIT_565_UYV_RGB,
	OSD_16BIT_4444_AYUV,
	OSD_16BIT_4444_RGBA,
	OSD_16BIT_4444_BGRA,
	OSD_16BIT_4444_ABGR,
	OSD_16BIT_4444_ARGB,
	OSD_16BIT_1555_AYUV,
	OSD_16BIT_1555_YUV,
	OSD_16BIT_5551_RGBA,
	OSD_16BIT_5551_BGRA,
	OSD_16BIT_1555_ABGR,
	OSD_16BIT_1555_ARGB,

	OSD_32BIT_8888_AYUV = 27,
	OSD_32BIT_8888_RGBA,
	OSD_32BIT_8888_BGRA,
	OSD_32BIT_8888_ABGR,
	OSD_32BIT_8888_ARGB,
};

struct VoutCfg_t {
	uint32_t pixelclk;
	uint32_t hactive;
	uint32_t vactive;

	uint32_t hfront_porch;
	uint32_t hback_porch;
	uint32_t hsync_len;

	uint32_t vfront_porch;
	uint32_t vback_porch;
	uint32_t vsync_len;

	uint32_t hsync_active;
	uint32_t vsync_active;

	uint32_t output_mode;
	uint32_t osd_format;
	uint32_t reserved[3];
};

enum {
	CMD_FW_BOOT_DONE = 0x1,
	CMD_VOUT_DONE = 0x10,
	CMD_PRI_START = 0x61,
	CMD_PRI_CLOSE = 0x62,
	CMD_PRI_SET_RES = 0x63,
	CMD_PRI_SET_FMT = 0x64,
	CMD_PRI_FLIP = 0x65,
	CMD_OSD_REGION = 0x66,
	CMD_OSD_FLIP = 0x67,
	CMD_OSD_ON = 0x68,
	CMD_OSD_OFF = 0x69,
	CMD_VOUT_TIMING = 0X6a,
	CMD_VOUT_SYNC_MODE = 0x6b,
	CMD_OSD_CTRL = 0X6c,

	CMD_GWARP_FRAME_DONE = 0x14,
	CMD_GWARP_OPEN = 0x70,
	CMD_GWARP_INIT_CONFIG = 0x71,
	CMD_GWARP_CLOSE = 0x72,
	CMD_GWARP_FRAME_START = 0x73,
	CMD_GWARP_TABLE_CONFIG = 0x74,
	CMD_GWARP_PITCH_CONFIG = 0x75,
	CMD_GWARP_CROP_CONFIG = 0x76,
	CMD_GWARP_FRAME_START_PLUS = 0x77,
	CMD_GWARP_SCALER_CONFIG = 0x78,
	CMD_GWARP_SELFCHECK_START = 0x90,
	CMD_GWARP_SELFCHECK_STOP = 0x91,

	CMD_ENCODER_FRAME_DONE = 0x18,
	CMD_ENCODER_SPS_PPS_DONE,
	CMD_ENCODER_OPEN = 0x80,
	CMD_ENCODER_START,
	CMD_ENCODER_CLOSE,
	CMD_ENCODER_FRAME_START,
	CMD_ENCODER_FRAME_STOP,
	CMD_ENCODER_RES_CFG,
	CMD_ENCODER_FRAMERATE_CFG,
	CMD_ENCODER_BITRATE_CFG,
	CMD_ENCODER_QP_CFG,
	CMD_ENCODER_GOP_CFG,
	CMD_ENCODER_REF_CFG,
	CMD_ENCODER_SPS_PPS_RESEND,
	CMD_ENCODER_RC_ALG_CFG,

	CMD_ENCODER_SELFCHECK_START = 0x92,
	CMD_ENCODER_SELFCHECK_STOP = 0x93
};

enum {
	CMD_BOOT_BUF_ID,
	CMD_PRI_START_BUFF_ID,
	CMD_PRI_CLOSE_BUFF_ID,
	CMD_PRI_RES_BUFF_ID,
	CMD_PRI_FMT_BUFF_ID,
	CMD_PRI_FLIP_BUFF_ID,
	CMD_OSD_REGION_BUFF_ID,
	CMD_OSD_FLIP_BUFF_ID,
	CMD_OSD_ON_BUFF_ID,
	CMD_OSD_OFF_BUFF_ID,
	CMD_VOUT_TIMING_BUFF_ID,
	CMD_VOUT_SYNC_MODE_BUFF_ID,
	CMD_OSD_CTRL_BUFF_ID,
	CMD_GWARP_FRM_DONE_BUFF_ID,
	CMD_GWARP_OPEN_BUFF_ID,
	CMD_GWARP_START_BUFF_ID,
	CMD_GWARP_CLOSE_BUFF_ID,
	CMD_GWARP_FRM_START_BUFF_ID_0_0 = 0x14,
	CMD_GWARP_TBL_CFG_BUFF_ID_0 = 0x24,
	CMD_GWARP_INIT_CFG_BUFF_ID_0 = 0x34,
	CMD_GWARP_SCALER_CFG_BUFF_ID_0 = 0x44,
	CMD_GWARP_PITCH_CFG_BUFF_ID_0 = 0x54,
	CMD_GWARP_CROP_CFG_BUFF_ID_0 = 0x64,

	CMD_GWARP_SELFCHECK_START_BUFF_ID_0 = 0x74,
	CMD_GWARP_SELFCHECK_STOP_BUFF_ID_0 = 0x84,

	CMD_ENCODER_START_BUFF_ID = 0x94,
	CMD_ENCODER_CLOSE_BUFF_ID,
	CMD_ENCODER_FRAME_START_BUFF_ID_0_0 = 0x96,
	CMD_ENCODER_FRAME_STOP_BUFF_ID_0 = 0x9e,
	CMD_ENCODER_BITRATE_CFG_BUFF_ID_0 = 0xa6,
	CMD_ENCODER_GOP_CFG_BUFF_ID_0 = 0xae,
	CMD_ENCODER_RES_CFG_BUFF_ID_0 = 0xb6,
	CMD_ENCODER_QP_CFG_BUFF_ID_0 = 0xbe,
	CMD_ENCODER_FRAMERATE_CFG_BUFF_ID_0 = 0xc6,
	CMD_ENCODER_SPS_PPS_RESEND_BUFF_ID_0 = 0xce,
	CMD_ENCODER_RC_ALG_CFG_BUFF_ID_0 = 0xd6,
	CMD_ENCODER_REF_CFG_BUFF_ID_0 = 0xde,

	CMD_ENCODER_SELFCHECK_START_BUFF_ID = 0xfe,
	CMD_ENCODER_SELFCHECK_STOP_BUFF_ID = 0xff
};

int get_encode_payload_addr(int channel_id, uint64_t *p_vaddr,
			    phys_addr_t *p_phy_addr);
int vsp_fw_boot_done(void);
int msg_transfer(struct vsp_msg_data *msg_data);
void vsp_core_callback_register(enum vsp_core_id cid, callback func);
#endif

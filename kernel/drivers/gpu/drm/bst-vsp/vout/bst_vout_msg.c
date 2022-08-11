// SPDX-License-Identifier: GPL-2.0

/*
 * VOUT IPC message interface for BST
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

#include "../bst_vsp_fw.h"
#include "bst_drm_drv.h"
#include <linux/completion.h>
#include <linux/coreip/bst_vsp_msg.h>

#define VSP_FW_ACK 0

/*vout support format*/
#define VOUT_FMT_1080P 0x077f0437
#define VOUT_FMT_720P  0x04ff02cf

static void build_command(struct vsp_msg_data *msg_data, u64 param, u64 data,
			  int cmd_minor, int bufid)
{
	msg_data->core_id = VSP_CORE_VOUT;
	strncpy(msg_data->src, "tc00", 4);
	strncpy(msg_data->dst, "vo00", 4);
	msg_data->cmd_type_main = CMD_VOUT_DEV;
	msg_data->cmd_type_minor = cmd_minor;
	msg_data->follow_pack_num = 0;
	msg_data->cmdid = bufid;
	msg_data->user_cmd_data[1] = param;
	msg_data->user_cmd_data[2] = data;
}

int vsp_cmd_set_res(void)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	build_command(&msg_data, 0x1, 0x0, CMD_PRI_SET_RES,
		      CMD_PRI_RES_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_PRI_SET_RES);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_PRI_SET_RES);
		if (ret < 0)
			pr_err("send CMD_PRI_SET_RES msg failed\n");
	}
#endif

	return ret;
}

int vsp_cmd_set_fmt(struct vop_device *pvop)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	build_command(&msg_data, 0x0, 0x0, CMD_PRI_SET_FMT,
		      CMD_PRI_FMT_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_PRI_SET_FMT);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_PRI_SET_FMT);
		if (ret < 0)
			pr_err("send CMD_PRI_SET_FMT msg failed\n");
	}
#endif

	return ret;
}

int vsp_cmd_primary_start(struct vop_device *pvop)
{
	return 0;
}

int vsp_cmd_primary_close(void)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	build_command(&msg_data, 0x0, 0, CMD_PRI_CLOSE, CMD_PRI_CLOSE_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_PRI_CLOSE);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_PRI_CLOSE);
		if (ret < 0)
			pr_err("send cmd close failed\n");
	}
#endif

	return ret;
}

int vsp_cmd_overlay_start(struct vop_device *pvop)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	// vsp_cmd_set_res(); //1080p
	build_command(&msg_data, 0x0, 0, CMD_OSD_ON, CMD_OSD_ON_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_OSD_ON);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_OSD_ON);
		if (ret < 0)
			pr_err("send CMD_OSD_ON msg failed\n");
	}
#endif

	return ret;
}

int vsp_cmd_overlay_region(struct vop_device *pvop)
{
	return 0;
}

int vsp_cmd_primary_flip(struct vop_device *pvop)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	// dump_fw_regs();
	build_command(&msg_data, pvop->primary_paddr, pvop->osd_paddr,
		      CMD_PRI_FLIP, CMD_PRI_FLIP_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_PRI_FLIP);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_PRI_FLIP);
		if (ret < 0)
			pr_err("send CMD_PRI_FLIP msg failed\n");
	}
#endif

	return ret;
}

int vsp_cmd_overlay_flip(struct vop_device *pvop, dma_addr_t paddr)
{
	int ret = 0;
	struct vsp_msg_data msg_data;

	// dump_fw_regs();
	build_command(&msg_data, paddr, 0, CMD_OSD_FLIP, CMD_OSD_FLIP_BUFF_ID);
	msg_transfer(&msg_data);
#if VSP_FW_ACK
	ret = vsp_wait_cmd_done(CMD_OSD_FLIP);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_OSD_FLIP);
		if (ret < 0)
			pr_err("send CMD_OSD_FLIP msg failed\n");
	}
#endif

	return ret;
}

void vsp_cmd_sync_mode_set(uint32_t mode)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, mode, 0, CMD_VOUT_SYNC_MODE,
		      CMD_VOUT_SYNC_MODE_BUFF_ID);
	msg_transfer(&msg_data);
}

void vsp_cmd_display_timing_set(uint32_t timing_phyaddr)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, timing_phyaddr, 0, CMD_VOUT_TIMING,
		      CMD_VOUT_TIMING_BUFF_ID);
	msg_transfer(&msg_data);
}

void vsp_cmd_overlay_ctrl(int enable, int hsize, int vsize)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, enable, 0, CMD_OSD_CTRL, CMD_OSD_CTRL_BUFF_ID);
	msg_data.user_cmd_data[2] = 0;
	msg_data.user_cmd_data[3] = ((hsize - 1) << 16) | (vsize - 1);
	msg_transfer(&msg_data);
}

void vout_ipc_msg_recv(struct media_command *param)
{
	if (param->cmd_hdr.hdr_info.cmd_type_minor == CMD_VOUT_DONE) {
		// pr_info("recv primary_paddr = 0x%x, osd_paddr = 0x%x\n",
		//	param->user_cmd_data[1], param->user_cmd_data[2]);
		// user_cmd_data[1]  primary buffer_addr
		// user_cmd_data[2]  osd buffer_addr
		complete(&vout_completion);
	}
}

// SPDX-License-Identifier: GPL-2.0

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

#include "bst_ipc_drv.h"
#include "linux/coreip/proto_api_common.h"
#include "proto_api_ipc.h"

#include <linux/coreip/bst_vsp_msg.h>
#include <linux/ipc_interface.h>

static DECLARE_COMPLETION(vsp_cmd_done);

extern struct vsp_device *pvsp;

#define WAIT_CMD_TIMEOUT      (HZ * 5)
#define WAIT_CMD_BOOT_TIMEOUT (HZ * 1)

static int send_msg_to_fw(struct vsp_device *pvsp, ipc_msg *msg)
{
	int ret;

	ret = ipc_send(pvsp->session_id, msg, -1);
	if (ret < 0) {
		pr_info("ipc_send(IPC_CORE_ARM1) failed, ret = %d", ret);
		return ret;
	}
	return 0;
}

static void build_msg_cmd(struct media_command *cmd,
			  struct vsp_msg_data *msg_data)
{
	int i;
	struct media_command *mcmd;
	uint32_t pack_num = msg_data->follow_pack_num;

	for (i = 0; i <= pack_num; i++) {
		mcmd = (struct media_command
				*)((uint8_t *)cmd +
				   i * sizeof(struct media_command));

		memset(mcmd, 0, sizeof(struct media_command));
		memcpy(mcmd->cmd_hdr.src, msg_data->src, 4);
		memcpy(mcmd->cmd_hdr.dst, msg_data->dst, 4);
		memcpy(&mcmd->user_cmd_data[0], &msg_data->user_cmd_data[i * 4],
		       sizeof(uint32_t) * 4);
		mcmd->cmd_hdr.hdr_info.cmd_type_main = msg_data->cmd_type_main;
		mcmd->cmd_hdr.hdr_info.cmd_type_minor =
			msg_data->cmd_type_minor;
		mcmd->cmd_hdr.hdr_info.follow_pack_num = pack_num;
		mcmd->cmd_hdr.hdr_info.attachment_en = 0;
	}
}

static int send_msg(struct vsp_device *pvsp, struct vsp_msg_data *msg_data)
{
	int ret;
	ipc_msg msg;
	struct media_command *cmd;
	tSoneCmdp *cmdp;

	cmdp = (tSoneCmdp *)pvsp->parti.cmdp_vaddr;
	cmd = (struct media_command *)(&cmdp->ch[0].cqueue.c0[msg_data->cmdid]);

	build_msg_cmd(cmd, msg_data);

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.cmd = ID_CMD_VSP_CORE;
	msg.token = 0;
	msg.data = pvsp->parti.cmdp_pyaddr +
		   (uint64_t)&cmdp->ch[0].cqueue.c0[msg_data->cmdid] -
		   (uint64_t)pvsp->parti.cmdp_vaddr;

	ret = send_msg_to_fw(pvsp, &msg);
	if (ret < 0) {
		pr_info("send msg failed, ret = %d", ret);
		return ret;
	}
	return 0;
}

int msg_transfer(struct vsp_msg_data *msg_data)
{
	int ret;

	ret = send_msg(pvsp, msg_data);

	return ret;
}

int vsp_recv_msg_thread(void *arg)
{
	int ret;
	struct platform_device *pdev;
	struct vsp_device *pvsp;
	struct media_command cmd_payload;
	ipc_msg msg;
	uint32_t msg_phy_addr;
	enum vsp_core_id core_id;
	int cmd_type;
	void *pcmd;
	char src[8], dst[8];

	pr_info("enter recv\n");
	pdev = (struct platform_device *)arg;
	pvsp = platform_get_drvdata(pdev);

	while (1) {
		ret = ipc_recv(pvsp->session_id, &msg, -1);
		if (ret < 0) {
			pr_info("ipc_recv_method(IPC_CORE_VSP) failed, ret = %d\n",
				ret);
			return ret;
		}
		msg_phy_addr = msg.data;
		pcmd = (struct media_command *)(msg_phy_addr -
						pvsp->parti.cmdp_pyaddr +
						(uint8_t *)
							pvsp->parti.cmdp_vaddr);
		memset(&cmd_payload, 0, sizeof(struct media_command));
		memcpy(&cmd_payload, pcmd, sizeof(struct media_command));
		strncpy(src, cmd_payload.cmd_hdr.src, 4);
		strncpy(dst, cmd_payload.cmd_hdr.dst, 4);
		cmd_type = cmd_payload.cmd_hdr.hdr_info.cmd_type_minor;
		// pr_info("recv ack cmdtype=%d\n", cmd_type);

		if ((cmd_payload.cmd_hdr.src[1] == 'o') &&
		    (cmd_payload.cmd_hdr.dst[1] == 'c')) {
			if (cmd_type == CMD_FW_BOOT_DONE) {
				complete(&vsp_cmd_done);
				// return ret;
			}
			core_id = VSP_CORE_VOUT;
		} else if ((cmd_payload.cmd_hdr.src[1] == 'w') &&
			   (cmd_payload.cmd_hdr.dst[1] == 'c')) {
			core_id = VSP_CORE_GMWARP;
		} else if ((cmd_payload.cmd_hdr.src[1] == 'e') &&
			   (cmd_payload.cmd_hdr.dst[1] == 'c')) {
			core_id = VSP_CORE_ENC;
		}
		pvsp->msg_handle[core_id](&cmd_payload);

		cmd_type = 0xff;
	}

	pr_info("exit");
	return ret;
}

static int vsp_wait_cmd_done(int cmd)
{
	int ret;

	if (cmd == CMD_FW_BOOT_DONE) {
		ret = wait_for_completion_timeout(&vsp_cmd_done,
						  WAIT_CMD_BOOT_TIMEOUT);
		if (!ret) {
			pr_err("%s: Firmware boot failed\n", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

int vsp_cmd_boot_done(void)
{
	int ret = 0;

	ret = vsp_wait_cmd_done(CMD_FW_BOOT_DONE);
	if (ret < 0) {
		ret = vsp_wait_cmd_done(CMD_FW_BOOT_DONE);
		if (ret < 0)
			pr_err("fw boot failed\n");
	}

	return ret;
}

int vsp_fw_version_check(void)
{
	int ret = 0;
	uint32_t status;
	void *reg;
	// char drv_ver[16];
	unsigned char version, level, sublevel;

	reg = ioremap(VSP_FW_VERSION_REG, 0x4);
	status = readl_relaxed(reg);
	sublevel = (status & 0xff);
	level = ((status >> 8) & 0xff);
	version = ((status >> 16) & 0xff);

	return ret;
}

#define DEBUG_REG1 0x54160020 // 3
#define DEBUG_REG2 0x54107c10 // 2
#define DEBUG_REG3 0x33102f3c // 1
#define DEBUG_RE4  0x33102000

void dump_fw_regs(void)
{
	// uint32_t status;
	void *reg1, *reg2, *reg3, *reg4;

	reg1 = ioremap(DEBUG_REG1, 0xC);
	reg2 = ioremap(DEBUG_REG2, 0x8);
	reg3 = ioremap(DEBUG_REG3, 0x4);
	reg4 = ioremap(DEBUG_RE4, 0x4);

	pr_info("zxj 0x54160020: %x,%x,%x\n", readl_relaxed(reg1),
		readl_relaxed(reg1 + 0x4), readl_relaxed(reg1 + 0x8));
	pr_info("zxj 0x54107c10: %x,%x\n", readl_relaxed(reg2),
		readl_relaxed(reg2 + 0x4));
	pr_info("zxj 0x33102f3c: %x\n", readl_relaxed(reg3));
	pr_info("zxj 0x33102000: %x\n", readl_relaxed(reg4));
}

int vsp_fw_boot_done(void)
{
	int ret = 0;

	mutex_lock(&pvsp->fw_boot_lock);
	if (!pvsp->fw_boot_done) {
		ret = vsp_runtime_fw_setup(pvsp);
		if (ret) {
			pr_err("VSP loading fw failed!\n");
			mutex_unlock(&pvsp->fw_boot_lock);
		}

		ret = vsp_cmd_boot_done();
		if (ret) {
			pr_err("VSP fw boot done failed!\n");
			mutex_unlock(&pvsp->fw_boot_lock);
		}

		ret = vsp_fw_version_check();
		if (ret < 0) {
			pr_err("VSP read fw version failed!\n");
			mutex_unlock(&pvsp->fw_boot_lock);
		}

		pvsp->fw_boot_done = 1;
	}
	mutex_unlock(&pvsp->fw_boot_lock);

	return ret;
}

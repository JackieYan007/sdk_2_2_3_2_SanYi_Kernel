/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VSP IPC driver definitions for BST
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

#ifndef _BST_IPC_COMMON_H_

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include "bst_vsp_fw.h"
#include "linux/coreip/proto_api_common.h"

#include <linux/coreip/bst_vsp_msg.h>

/*vsp register addr*/
#define CTRL_REG0 0x33002004
//#define CTRL_REG1 0x20010000
#define CTRL_REG2 0x330010f0

/*vsp register mask*/
#define REG2_MASK  0xfff0f0f0
#define REG2_MASK1 0xf0f0f0f0
#define REG2_MASK2 0xf0f0ffff

/* vsp fw version register */
#define VSP_FW_VERSION_REG 0x9c000058

/* VOUT DRV VERSION */
#define VERSION	 0
#define LEVEL	 1
#define SUBLEVEL 0x20

/* recv fw ack, need vsp fw support */
#define VSP_FW_ACK 0

// 1KB alignment
#define VSP_PAYLOAD_ALIGN_MASK	  0x3ff
// total payload size of vout
#define VOUT_PAYLOAD_TOTAL_SIZE	  0x40000 // 256KB
// total payload size of gwarp
#define GWARP_PAYLOAD_TOTAL_SIZE  0x40000 // 256KB
// total payload size of encode
#define ENCODE_PAYLOAD_TOTAL_SIZE 0x40000 // 256KB

#define ENCODE_VIDEO_PAYLOAD_NUM  256
// payload size of each encode video
#define ENCODE_VIDEO_PAYLOAD_SIZE 0x400 // 1KB

struct vsp_device {
	struct device *dev;
	struct vsp_firmware firmware_c0;
	struct vsp_firmware firmware_c1;
	struct vsp_firmware firmware_c2;
	// const char *slab_name;
	struct vsp_parti parti;
	phys_addr_t intp_phy_addr;
	phys_addr_t fw_fbuf_phy_addr; // reserve for vsp fw internal use
	long fw_fbuf_size;
	struct vsp_reg vreg;
	int64_t session_id;
	callback msg_handle[VSP_CORE_MAX];
	void *vout_payload_vaddr;
	void *gwarp_payload_vaddr;
	void *encode_payload_vaddr;
	phys_addr_t vout_payload_paddr;
	phys_addr_t gwarp_payload_paddr;
	phys_addr_t encode_payload_paddr;

	struct mutex fw_boot_lock;
	int fw_boot_done;
	u32 format;
};

int of_device_parse(struct platform_device *pdev, struct vsp_device *pvsp);
int vsp_fw_mem_init(struct vsp_device *pvsp);
int vsp_recv_msg_thread(void *arg);
void vsp_send_msg_handler(struct work_struct *work);
int alloc_command_queue(struct vsp_device *pvsp);

int vsp_dev_init(struct vsp_device *pvsp);
int vsp_runtime_fw_setup(struct vsp_device *pvsp);

#endif

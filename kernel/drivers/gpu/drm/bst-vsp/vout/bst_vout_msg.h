/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VOUT IPC message declarations for BST
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

#ifndef __BST_VOUT_MSG_H__
#define __BST_VOUT_MSG_H__

#include <linux/coreip/bst_vsp_msg.h>
#include <linux/string.h>

int vsp_cmd_primary_start(struct vop_device *pvop);
int vsp_cmd_primary_close(void);
int vsp_cmd_primary_flip(struct vop_device *pvop);
int vsp_cmd_overlay_start(struct vop_device *pvop);
int vsp_cmd_overlay_flip(struct vop_device *pvop, dma_addr_t paddr);
int vsp_cmd_overlay_region(struct vop_device *pvop);
void vsp_cmd_sync_mode_set(uint32_t mode);
void vsp_cmd_display_timing_set(uint32_t timing_phyaddr);
void vsp_cmd_overlay_ctrl(int enable, int hsize, int vsize);

void vout_ipc_msg_recv(struct media_command *param);
#endif

/* SPDX-License-Identifier: GPL-2.0 */

/*
 * VSP Firmware definitions for BST
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

#ifndef __BST_VSP_FW_h__
#define __BST_VSP_FW_h__

#include <asm/cacheflush.h>
#include <asm/mman.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

// cmd
#define SONE_VSP_CMD_ECHO   0x00000001
#define SONE_VSP_ECHO_DSTEP "vt01"
#define SONE_VSP_ECHO_SRCEP "dv01"

struct cmd_addr_node {
	int cmdid;
	uint32_t cmdpaddr;
	void *cmdvaddr;
	struct list_head node;
};

struct vsp_firmware {
	const char *fw_name;
	const struct firmware *fw;
	phys_addr_t phy_addr;
	void *base;
	resource_size_t size;
};

struct vsp_reg {
	void __iomem *regctl;
	void __iomem *regfw;
	void __iomem *regsta;
	void *base;
};

struct vsp_parti {
	void *init_vaddr;
	void *cmdp_vaddr;
	void *slab_vaddr;
	phys_addr_t cmdp_pyaddr;
	phys_addr_t fbuf_pyaddr;
};

#endif

// SPDX-License-Identifier: GPL-2.0
/*
 * BST PCIE Interconnection Control Protocol system.
 *
 * Copyright (C) 2021 Black Sesame Technologies, Inc.
 */

#ifndef _PICP_PRIVATE_H_
#define _PICP_PRIVATE_H_

#include <linux/kernel.h>
#include <linux/picp.h>
#include "pcie_test.h"
#include "../controller/bst/pcie-bst.h"

__attribute__((unused))
static struct picp_info *info;
#define PCI_DEBUG_LOGEN  0
#if PCI_DEBUG_LOGEN
#define picp_dbg(format, ...) printk(KERN_INFO "%s: "format, info->name, ##__VA_ARGS__);
#else
#define picp_dbg(format, ...)
#endif

//if (printk_ratelimit())
#define picp_err(format, ...) printk(KERN_INFO "%s: "format, info->name, ##__VA_ARGS__);

typedef enum {
	PICP_ITYP_IDLE = 0,
	PICP_ITYP_REQ,
	PICP_ITYP_ACK,
	PICP_ITYP_DATA,
	PICP_ITYP_RWENABLE,
	PICP_ITYP_RDONE,
	PICP_ITYP_WDONE
} picp_itype;

struct task_handle {
	int single;
	u32 datasize;
	struct task_struct *rtask;
	struct task_struct *wtask;
	struct picp_private *priv;
};

extern const struct dma_map_ops bst_dma_ops3;

extern u32 get_ltssm_count(void);

#endif // _PICP_PRIVATE_H_


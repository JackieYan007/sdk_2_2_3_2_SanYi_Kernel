/* SPDX-License-Identifier: GPL-2.0 */

/*
 * ISP firmware loader definitions for BST
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

#ifndef _ISP_FW_LOADER_H_
#define _ISP_FW_LOADER_H_

#include "isp_core.h"

#define ISP_FW_IMAGE_NAME   "isp/core_isp.exe"
#define ISP_SLAB_NAME	    "isp/common/slab.bin"
#define TEXT_SECTION_NAME   (".text")
#define RODATA_SECTION_NAME (".rodata")

#define ISP_PACK_OFFSET	      0X80
#define ISP_SVN_OFFSET	      0x84
#define ISP_BUILD_DATE_OFFSET 0x88
#define ISP_SLAB_OFFSET	      0x80

#define OUT_SRC_EN       0x52030100
#define SRC1_PERIIOD     0x52030104
#define SRC2_PERIIOD	 0x52030108
#define PULSE_WIDTH      0x5203011C
#define INT_PULSE_LOW    0X5203015C
#define OUT_SRC_PERIOD   0x5203010c
#define FSYNC_SEL	 0x52030160
#define DECREASE1_PERIOD 0x5203016c
#define DECREASE2_PERIOD 0x52030170

int bst_load_dsp_fw(const char *fw_name, struct a1000_isp_device *isp);
void bst_start_dsp_fw(struct a1000_isp_device *isp);
int bst_load_isp_fw(const char *fw_name, const char *slab_name,
		    struct a1000_isp_device *isp);
void bst_start_isp_fw(struct a1000_isp_device *isp);
int isp_internal_trigger(int target_freq, int fysnc_in, int fsync_out);
int isp_external_trigger(int external_freq, int target_freq, int fsync_in, int fsync_out);
int bst_boot_isp_fw(const char *fw_name, const char *slab_name,
		    struct a1000_isp_device *isp);

#endif

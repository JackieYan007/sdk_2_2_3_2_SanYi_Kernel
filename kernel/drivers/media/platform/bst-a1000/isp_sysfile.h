/* SPDX-License-Identifier: GPL-2.0 */

/*
 * ISP sysfs entry definitions for BST
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

#ifndef _ISP_SYSFILE_H_
#define _ISP_SYSFILE_H_

#include "isp_core.h"

#define TEXT_SECTION_NAME   (".text")
#define RODATA_SECTION_NAME (".rodata")

enum {
	ISP_FW_LOAD_RUN = 0,
	ISP_ECHO_TEST,
	ISP_CATCH_TEST,
};

int isp_sysfs_init(struct a1000_isp_device *isp);
void isp_echo_test_callback(void);
#endif

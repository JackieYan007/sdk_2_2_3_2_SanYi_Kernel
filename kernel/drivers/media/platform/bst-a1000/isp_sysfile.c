// SPDX-License-Identifier: GPL-2.0

/*
 * ISP sysfs entry for BST
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ipc_interface.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <linux/coreip/proto_api_common.h>

#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_sysfile.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

static struct a1000_isp_device *s_isp;
#ifdef ECHO_TEST
static int isp_fw_echo_test(struct a1000_isp_device *isp);
struct cmd_element *echo_element;
#endif

ssize_t show_isp_fw(struct kobject *object, struct kobj_attribute *attr,
		    char *buf)
{
	pr_info("you can do echo test with isp fw\r\n"
		"please put isp firmware in /lib/firmware/updates\r\n"
		"load firmware and run :echo 0 > /sys/isp/isp_fw\r\n"
		"echo test:  echo 1 > /sys/isp/isp_fw\r\n");

	return 0;
}

ssize_t store_isp_fw(struct kobject *object, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	if (value == ISP_FW_LOAD_RUN) { /*load fw*/
		bst_boot_isp_fw(ISP_FW_IMAGE_NAME, ISP_SLAB_NAME, s_isp);
	}
#ifdef ECHO_TEST
	else if (value == ISP_ECHO_TEST) { /*run fw*/
		isp_fw_echo_test(s_isp);
	}
#endif
	else if (value == ISP_CATCH_TEST) { /*run fw*/
		// isp_cache_test(s_isp);
	} else {
		pr_info("please type in correct number\n");
	}

	return count;
}

static struct kobj_attribute s_isp_fw_attribute =
	__ATTR(isp_fw, 0664, show_isp_fw, store_isp_fw);

ssize_t isp_log_level_show(struct kobject *object, struct kobj_attribute *attr,
			   char *buf)
{
	return 0;
}

ssize_t isp_log_level_store(struct kobject *object, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	return count;
}

static struct kobj_attribute s_isp_log_level_attribute =
	__ATTR(isp_log_level, 0664, isp_log_level_show, isp_log_level_store);

static ssize_t isp_version_show(struct kobject *object,
				struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf,
		      "ISP Drv: %s, FW:pack = 0x%x svn = %d, date = 0x%x\n",
		      s_isp->revision, s_isp->fw_pack_version,
		      s_isp->fw_svn_version, s_isp->fw_build_date);

	return ret;
}

static struct kobj_attribute s_isp_version_attribute =
	__ATTR(isp_version, 0444, isp_version_show, NULL);

ssize_t isp_video_debug_show(struct kobject *object,
			     struct kobj_attribute *attr, char *buf)
{
	return 0;
}

ssize_t isp_video_debug_store(struct kobject *object,
			      struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	if (value < MAX_ISP_CHANNEL)
		dump_video_debug_info(&(s_isp->channels[value].views_video));
	else
		pr_err("error video index = %lu\n", value);

	return count;
}

static struct kobj_attribute s_isp_video_debug_attribute = __ATTR(
	isp_video_debug, 0664, isp_video_debug_show, isp_video_debug_store);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
	&s_isp_fw_attribute.attr,
	&s_isp_video_debug_attribute.attr,
	&s_isp_log_level_attribute.attr,
	&s_isp_version_attribute.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *isp_kobj;

int isp_sysfs_init(struct a1000_isp_device *isp)
{
	int ret = -1;

	s_isp = isp;

	isp_kobj = kobject_create_and_add("isp", NULL);
	if (!isp_kobj) {
		pr_err("kobject_create_and_add failed");
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(isp_kobj, &attr_group);
	if (ret) {
		pr_err("sysfs_create_group failed, error code = %d", ret);
		kobject_put(isp_kobj);
	}

#ifdef ECHO_TEST
	echo_element = get_free_media_cmd_element(isp);
#endif

	return ret;
}

#ifdef ECHO_TEST
static struct media_command *fill_media_cmdd(struct a1000_isp_video *video,
					     struct cmd_element *element)
{
	struct media_command *cmd;
	static int i;

	cmd = (struct media_command *)element->media_cmd_vaddr;
	pr_info("cmd address is %x\n", cmd);

	memcpy(cmd->cmd_hdr.src, isp_driver_enp_name[video->video_index], 4);
	return cmd;
}

int isp_index = 1;

static int isp_fw_echo_test(struct a1000_isp_device *isp)
{
	int ret;
	int i, count_a;
	struct media_command *cmd;
	ipc_msg msg;
	/*here is just a echo case,so address is a example ,0x4b0205fe0*/
	cmd = fill_media_cmdd(&isp->videos[0], echo_element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ECHO_TEST;
	for (i = 0; i < 4; i++)
		cmd->user_cmd_data[i] = isp_index;
	isp_index = isp_index + 1;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = echo_element->media_cmd_paddr;
	dev_info(isp->dev, "isp driver send\n");
	for (count_a = 0; count_a < 8; count_a++)
		dev_info(isp->dev, "%x",
			 ((uint32_t *)echo_element->media_cmd_vaddr)[count_a]);
	send_cmd_to_fw(isp, &msg);
	return ret;
}

void isp_echo_test_callback(void)
{
	isp_fw_echo_test(s_isp);
}
#endif

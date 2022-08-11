/* SPDX-License-Identifier: GPL-2.0 */

/*
 * ISP core definitions for BST
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

#ifndef __A1000_ISP_CORE_H__
#define __A1000_ISP_CORE_H__

#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>

#include <linux/ipc_interface.h>

#include "cam_entity.h"
#include "csi.h"
#include "isp_video.h"

#define ISP_SYSFS

#define SONE_MEDIA_FBUF_TOTAL_SIZE 0x8000000
#define ISP_CORE_NUM		   3
#define ISP_CHANNEL_VIEW_NUM	   3
#define ISP_CHANNEL_VIEW_NUM_PDNS  1
#define MAX_CHANNEL_PER_CORE	   4
#define HDMI_DTS_PORT_NUM	   4

// A1000 supports max 4 mipi devices
#define MAX_MIPI_DEVICE_NUM 4
#define MAX_ISP_SUB_DEVICE  5

#define MAX_ISP_CHANNEL	   12
#define A1000_ISP_REVISION "1.0.0"

#define ISP_PLATFORM ("linux-isp")
#define ISP_DDR_BASE 0xA1000000

#define LOW_32_BIT_MASK	 0xFFFFFFFF
#define HIGH_32_BIT_MASK 0xFFFFFFFF00000000

#define ISP_INIT_PARTI_SIZE sizeof(tSoneInit)
#define ISP_CMDP_PARTI_SIZE sizeof(tSoneCmdp)

#define ISP_PRE_FW_TICK_ON_KTIME_NS 500 // each fw tick spend on ktime_ns
#define ISP_CTRL_PAYLOAD_SIZE	    40 // ioctrl paload size by byte

struct trigger_info {
	int trigger_mode;
	int mipi_id;
	int internal_trigger_fps;
	int camera_trigger_gpio_port;
	int deser_trigger_gpio_port;
	int external_freq;
	int target_freq;
	int fsync_in;
	int fsync_out;
};

struct camera_info {
	char camera_name[MAX_CAMERA_NAME_LEN];
	int camera_id;
	int is_streaming;
};

struct isp_misc_device {
	struct camera_info cam_info[MAX_ISP_CHANNEL];
	unsigned int camera_num;
};

enum {
	INTERNAL_TRIGGER_MODE = 0,
	EXTERNAL_TRIGGER_MODE
};

/*
 * /dev/isp_misc ioctls
 */
#define IOC_GET_VAILED_CAMERA_INFO \
	_IOR('y', 0x41, int) /* Get vailed camera info */
#define IOC_ECHO_ISP_MISC _IOWR('y', 0x42, int) /* echo */
#define IOC_SET_MIPI_TRIGGER_MODE \
	_IOR('y', 0x43, int) /* Get vailed camera info */

enum {
	ISP_STATUS_BOOT_FW = 0,
	ISP_STATUS_FW_BOOT_DONE
};

enum {
	ISP_CHANNEL_SINK_PAD = 0,
	ISP_CHANNEL_SOURCE_NORMAL = 1,
	ISP_CHANNEL_SOURCE_PDNS = 2,
	ISP_CHANNEL_PAD_NUM,
};

enum ispdrv_state {
	ISPDRV_STATE_WAIT = 0, // wait firmware boot
	ISPDRV_STATE_ISP_DSP_SYNC, // wait ISP sync with DSP
	ISPDRV_STATE_SCFG, // send firmware config
	ISPDRV_STATE_SALG, // send firmware algo bin
	ISPDRV_STATE_SIQ, // send firmware iq bin
	ISPDRV_STATE_SET_DSP_PWL, // set DSP pwl look up table
	//ISPDRV_STATE_SET_CAPTURE_CONF, // set sensor capture mode
	ISPDRV_STATE_START, // start firmware
	ISPDRV_STATE_WORK, // work with firmware
	ISPDRV_STATE_DEBUG, // debug with firmware
};

struct a1000_isp_device;

struct bst_isp_channel {
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_entity entity;
#endif
	struct media_pad pads[ISP_CHANNEL_PAD_NUM];
	struct a1000_isp_video views_video; // video supports multi view
	struct a1000_isp_video raw_video;
	struct a1000_isp_device *isp;
	struct bst_csi_channel *csi_channel;
	struct camera_dev *cam_dev;
	int sn;
	// int sd_state;
	int camera_raw_width;
	int camera_raw_height;
	int isp_core_id;
	int index_in_core;
	int isp_chn_id;
	int remote_mipi_id; // the mipi device id of this isp channel linked
	int remote_mipi_vc_index; // mipi vc index of this isp channel linked to
	bool enable;
	bool is_hdmi;
	atomic_t is_streaming;
	struct device_node *of_node;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *remote_fwnode;
	uint32_t ctrl_payload_paddr;
	u8 *ctrl_payload_vaddr;
};

struct csi_async_dev {
	struct v4l2_async_subdev async_dev;
	struct fwnode_handle *mipi_fwnode;
	struct bst_csi_device *csi_dev;
	struct a1000_isp_device *isp_parent;
	int mipi_index;
	int mipi_connected;
};

/*
 * struct a1000_isp_device - ISP device structure.
 * @dev: Device pointer specific to the A1000 ISP.
 * @revision: Stores current ISP module revision.
 * @stat_lock: Spinlock for handling statistics
 * @isp_mutex: Mutex for serializing requests to ISP.
 * @stop_failure: Indicates that an entity failed to stop.
 * @ref_count: Reference count for handling multiple ISP requests.
 * @isp_slots: ISP supports 12 sensors totally, assume 12 virtual isp slots
 *
 * This structure is used to store the BST A1000 ISP Information.
 */
struct a1000_isp_device {
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;
	struct bst_isp_channel channels[MAX_ISP_CHANNEL];
	struct csi_async_dev csi_asd[MAX_MIPI_DEVICE_NUM];
	// struct v4l2_async_subdev async_dev[MAX_MIPI_DEVICE_NUM];
	struct device *dev;
	struct platform_device *pdev;
	char revision[16];
	struct v4l2_async_subdev hdmi_async;
	struct device_node *hdmi_node;
	struct fwnode_handle *hdmi_handle;
	struct fwnode_handle *remote_hdmi;
	struct camera_dev *hdmi_cam;
	uint32_t fw_pack_version;
	uint32_t fw_svn_version;
	uint32_t fw_build_date;
	struct isp_misc_device misc_device;
	enum ispdrv_state state;
	int hdmi_detected;
	int core0_active_cam;
	int cfg_num;
	int alg_num;
	int iq_num;
	int cfg_count;
	int alg_count;
	int iq_count;
	int pwl_lut_num;
	int pwl_lut_count;
	int capture_conf_num;
	int capture_conf_count;
	atomic_t FW_load_started;
	atomic_t FW_boot_done;
	atomic_t FW_config_done;
	struct completion FW_start_completion;
	// GlbCfg   *FW_glb_cfg;
	struct wait_queue_head ctrl_recv_wq;
	uint32_t ctrl_get_val;
	int isp_ctrl_status;
	/* ISP Obj */
	struct mutex isp_mutex; /* For handling ref_count field */
	bool stop_failure;
	int core_channel_num[ISP_CORE_NUM];
	int core_camera_num[ISP_CORE_NUM];
	int64_t ipc_session_id;
	uint64_t init_paddr;
	uint64_t cmdp_paddr;
	uint64_t slab_paddr;
	uint64_t fbuf_paddr;
	uint64_t fbuf_psize;
	void *init_vaddr;
	void *cmdp_vaddr;
	void *slab_vaddr;
	uint8_t *ctrl_reg_base;
	struct task_struct *kthread_isp; // for isp message receiving
	struct task_struct *kthread_tick;
	uint64_t ipc_rx_count;
	uint64_t ipc_tx_count;
	u8 *config_payload_vaddr;
	u8 *algobin_next_vaddr;
	u8 *payload_end_vaddr;
	struct media_command *config_media_cmd;
	uint32_t config_payload_paddr;
	int config_align_size;
	uint32_t algobin_next_paddr;
	uint32_t payload_end_paddr;
	uint32_t media_cmd_paddr;
	int kthread_status;
	// int mipi_connected[MAX_MIPI_DEVICE_NUM];
	// struct fwnode_handle *mipi_fwnode[MAX_MIPI_DEVICE_NUM];
	int total_subdev;
	int total_channel;
	int config_count;
	atomic_t streamon_count;
	bool use_dsp;
	uint8_t dsp_core_id;
	uint32_t dsp_sync_ddr_base;
};

enum {
	KTHREAD_STATUS_NONE = 0,
	KTHREAD_STATUS_RECV_MSG,
	KTHREAD_STATUS_RECV_COMPLETE,
	KTHREAD_STATUS_COPY_MSG,
	KTHREAD_STATUS_COPY_COMPLETE,
};

enum {
	ISP_SET_VIEW_STATUS_NONE = 0,
	ISP_SET_VIEW_STATUS_WAIT = 1,
	ISP_SET_VIEW_STATUS_DONE = 2,
};

// todo: add more color
// A1000_ISP_COLOR_RAW8,
// A1000_ISP_COLOR_RAW10,
// A1000_ISP_COLOR_RAW12,
// A1000_ISP_COLOR_YUV422, (YUYV and so on)
// A1000_ISP_COLOR_YUV444
enum {
	A1000_ISP_COLOR_YUV420 = 0, // Y, U, V
	A1000_ISP_COLOR_NV12,
	A1000_ISP_COLOR_NV21,
	A1000_ISP_COLOR_YUYV,
	A1000_ISP_COLOR_RGB888,
	A1000_ISP_COLOR_RAW8,
	A1000_ISP_COLOR_RAW10,
	A1000_ISP_COLOR_RAW12,
	A1000_ISP_COLOR_RAW16,
	A1000_ISP_COLOR_Y_ONLY,
	A1000_ISP_COLOR_MAX,
};

int ispdrv_enter_debug_state(struct a1000_isp_device *isp);

int ispdrv_leave_debug_state(struct a1000_isp_device *isp);

void a1000_isp_print_status(struct a1000_isp_device *isp);

int send_cmd_to_fw(struct a1000_isp_device *isp, ipc_msg *msg);

int isp_power_subdevs(struct a1000_isp_device *isp);

int isp_update_views_from_fw(struct a1000_isp_device *isp);

static inline void isp_inc_streamon_count(struct a1000_isp_device *isp)
{
	atomic_inc(&(isp->streamon_count));
}

static inline void isp_dec_streamon_count(struct a1000_isp_device *isp)
{
	atomic_dec(&(isp->streamon_count));
}

static inline int get_isp_streamon_count(struct a1000_isp_device *isp)
{
	int ret;

	ret = atomic_read(&(isp->streamon_count));

	return ret;
}

#define pack_chars_to_int(a, b, c, d) ((a) | (b << 8) | (c << 16) | (d << 24))

#endif /* __A1000_ISP_CORE_H__ */

// SPDX-License-Identifier: GPL-2.0

/*
 * ISP Core driver for BST
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

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#include <asm/cacheflush.h>
#ifdef CONFIG_ARM_DMA_USE_IOMMU
#include <asm/dma-iommu.h>
#endif

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ipc_interface.h>
#include <linux/kobject.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/vmalloc.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include <linux/coreip/proto_api_common.h>

#include "../../i2c/bst/ti_deser_hub.h"
#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_sysfile.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

#define MAX_KTHREAD_MAX_TRY 5

// video stream fps is about 30, set timeout to 300000ms for debug
#define MAX_VIDEO_STREAM_TIMEOUT  300000
#define ISP_FW_CTRL_REG		  0x520b6800
#define ISP_FW_CTRL_REG_SIZE	  0x40
// isp msg payload size, 256Byte alignment
#define ISP_MSG_PAYLOAD_SIZE	  0x100
#define ISP_MSG_PAYLOAD_SIZE_MASK 0xff

struct bin_info_t {
	char filename[MAX_BIN_NAME_LEN];
	u8 *payload_vaddr;
	u32 payload_paddr;
	int size;
	int camera_mask; // maybe used by multi cameras
};

struct isp_algo_bin_t {
	struct bin_info_t algo_bin[MAX_ISP_CHANNEL];
	int file_num;
};

static struct isp_algo_bin_t s_isp_algobin;
static struct isp_misc_device *globle_isp_misc_device;

static int send_next_cfg_to_fw(struct a1000_isp_device *isp);
static int send_next_alg_to_fw(struct a1000_isp_device *isp);
static int send_next_iq_to_fw(struct a1000_isp_device *isp);
static int send_next_pwl_lut_to_fw(struct a1000_isp_device *isp);
static int send_isp_dsp_sync_msg_to_fw(struct a1000_isp_device *isp);
static int send_start_msg_to_fw(struct a1000_isp_device *isp);

static int isp_firmware_init(struct a1000_isp_device *isp)
{
	uint32_t init_phy_base_low;
	uint32_t init_phy_base_high;
	uint64_t init_phy_base;
	uint64_t cmdp_phy_base;
	uint64_t slab_phy_base;
	tSoneInit *pInit;

	void *pSlab;
	tSoneCmdp *pCmdp;

	uint32_t traceMask = 0xffffffff;
	int cmdp_size;
	int initp_size;
	uint32_t addr_start;
	int partition_total_size;
	dma_addr_t partition_dma_addr;
	char *partition_vaddr;
	u8 *payload_vaddr;
	uint64_t payload_base;
	uint32_t payload_paddr;
	u32 addr_offset;

	addr_start = ISP_DDR_BASE;
	init_phy_base = ISP_DDR_BASE;
	init_phy_base_low = ISP_DDR_BASE;
	init_phy_base_high = 0;

	// 1KB alignment
	// to do: 4KB alignment
	initp_size = (sizeof(tSoneInit) + SONE_PART_ALIGN_MASK) &
		     (~SONE_PART_ALIGN_MASK);
	pr_debug("init start = 0x%llx, initp_size = 0x%lx, align size = 0x%x\n",
		 init_phy_base, sizeof(tSoneInit), initp_size);

	addr_start += initp_size;
	cmdp_phy_base = addr_start;
	cmdp_size = (sizeof(tSoneCmdp) + SONE_PART_ALIGN_MASK) &
		    (~SONE_PART_ALIGN_MASK);

	addr_start += cmdp_size;
	// for buf, 1MB alignment
	addr_start = (addr_start + SONE_BUF_ADDR_ALIGN_MASK) &
		     (~SONE_BUF_ADDR_ALIGN_MASK);
	slab_phy_base = addr_start;
	addr_start += SONE_MEDIA_SLAB_BUFSIZE;
	addr_start = (addr_start + SONE_BUF_ADDR_ALIGN_MASK) &
		     (~SONE_BUF_ADDR_ALIGN_MASK);

	partition_total_size = addr_start - ISP_DDR_BASE;
	init_phy_base_low = (init_phy_base & LOW_32_BIT_MASK);
	init_phy_base_high = (init_phy_base & HIGH_32_BIT_MASK) >> 32;

	isp->init_paddr = init_phy_base;
	isp->cmdp_paddr = cmdp_phy_base;
	isp->slab_paddr = slab_phy_base;
	partition_vaddr = dma_alloc_coherent(isp->dev, partition_total_size,
					     &partition_dma_addr, GFP_KERNEL);
	if (partition_vaddr == NULL) {
		pr_err("dma_alloc_coherent partition_total_size = 0x%x failed\n",
		       partition_total_size);
		return -1;
	}
	if ((unsigned long)partition_dma_addr != (unsigned long)ISP_DDR_BASE) {
		pr_err("ISP ERROR: partition_dma_addr != ISP_DDR_BASE\n");
		dma_free_coherent(isp->dev, partition_total_size,
				  partition_vaddr, partition_dma_addr);
		return -1;
	}

	// setup init partition
	pInit = (tSoneInit *)partition_vaddr;
	isp->init_vaddr = pInit;
	setup_init_parti(pInit, init_phy_base_low, init_phy_base_high,
			 ISP_PLATFORM, ISP_PLATFORM, traceMask, "A1000", 'A');

	// setup cmdp partition
	pCmdp = (tSoneCmdp *)(partition_vaddr + (cmdp_phy_base - ISP_DDR_BASE));
	isp->cmdp_vaddr = pCmdp;
	setup_cmdp_parti(pInit, pCmdp, cmdp_phy_base);

	// setup slab partition
	pSlab = (void *)(partition_vaddr + (slab_phy_base - ISP_DDR_BASE));
	isp->slab_vaddr = pSlab;
	setup_slab_parti(pInit, pSlab, slab_phy_base);

	payload_vaddr = (u8 *)pCmdp->ch[DRV_CH_INDEX].cqueue.payload_cmd;
	addr_offset = (uint64_t)(payload_vaddr) - (uint64_t)(isp->cmdp_vaddr);
	payload_paddr = (addr_offset + isp->cmdp_paddr) & LOW_32_BIT_MASK;
	isp->payload_end_vaddr = payload_vaddr + SONE_MEDIA_PAYLOAD_MEMSIZE;
	isp->payload_end_paddr = payload_paddr + SONE_MEDIA_PAYLOAD_MEMSIZE;

	payload_base = (uint64_t)payload_vaddr;
	payload_base = ((payload_base + ISP_MSG_PAYLOAD_SIZE_MASK) &
			(~ISP_MSG_PAYLOAD_SIZE_MASK));
	isp->config_payload_vaddr = (u8 *)payload_base;
	isp->config_align_size =
		(sizeof(ipc_reconf_t) + ISP_MSG_PAYLOAD_SIZE_MASK) &
		(~ISP_MSG_PAYLOAD_SIZE_MASK);

	isp->algobin_next_vaddr = isp->config_payload_vaddr +
				  (isp->config_align_size * MAX_ISP_CHANNEL);

	addr_offset = (uint64_t)(isp->config_payload_vaddr) -
		      (uint64_t)(isp->cmdp_vaddr);
	isp->config_payload_paddr = (addr_offset + isp->cmdp_paddr) &
				    LOW_32_BIT_MASK;

	addr_offset = (uint64_t)(isp->algobin_next_vaddr) -
		      (uint64_t)(isp->cmdp_vaddr);
	isp->algobin_next_paddr = (addr_offset + isp->cmdp_paddr) &
				  LOW_32_BIT_MASK;

	isp->config_media_cmd =
		(struct media_command *)&pCmdp->ch[DRV_CH_INDEX].cqueue.c0[0];
	addr_offset =
		(uint64_t)(isp->config_media_cmd) - (uint64_t)(isp->cmdp_vaddr);
	isp->media_cmd_paddr = (addr_offset + isp->cmdp_paddr) &
			       LOW_32_BIT_MASK;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_device_ops isp_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void isp_cleanup_videos(struct a1000_isp_device *isp)
{
	int i;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		a1000_isp_video_cleanup(&isp->channels[i].views_video);
		a1000_isp_video_cleanup(&isp->channels[i].raw_video);
	}
}

#pragma GCC diagnostic pop
/*
 * isp_remove - Remove ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Always returns 0.
 */
static int isp_remove(struct platform_device *pdev)
{
	struct a1000_isp_device *isp = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&isp->notifier);
	v4l2_async_notifier_cleanup(&isp->notifier);

	return 0;
}

int send_cmd_to_fw(struct a1000_isp_device *isp, ipc_msg *msg)
{
	int ret;

	ret = ipc_send(isp->ipc_session_id, msg, -1);
	if (ret < 0)
		pr_err("%s: ipc_send error, ret = %d\n", __func__, ret);
	isp->ipc_tx_count++;

	return ret;
}

static int is_channel_mode_online(struct a1000_isp_device *isp, int index)
{
	int core;
	int num;
	int i;

	core = index / 4;
	num = 0;
	for (i = 0; i < 4; i++) {
		if (isp->channels[core * 4 + i].enable)
			num++;
	}

	return (num == 1);
}

static int channel_cfg_list[MAX_ISP_CHANNEL] = { -1 };
static int channel_alg_list[MAX_ISP_CHANNEL] = { -1 };
static int channel_iq_list[MAX_ISP_CHANNEL] = { -1 };
static int channel_pwl_lut_list[MAX_ISP_CHANNEL] = { -1 };

static inline int is_channel_use_alg(struct a1000_isp_device *isp, int index)
{
	return (isp->channels[index].cam_dev->algo_online[0] ||
		isp->channels[index].cam_dev->algo_offline[0]);
}

static inline int is_channel_use_iq(struct a1000_isp_device *isp, int index)
{
	return (isp->channels[index].cam_dev->iq_online[0] ||
		isp->channels[index].cam_dev->iq_offline[0]);
}

static inline int is_channel_use_pwl_lut(struct a1000_isp_device *isp,
					 int index)
{
	return isp->channels[index].cam_dev->pwl_lut[0];
}

static inline int is_channel_cfg_left(struct a1000_isp_device *isp)
{
	// pr_info("%s, cfg_num %d, cfg_count %d\n", __func__, isp->cfg_num,
	// isp->cfg_count);
	return (isp->cfg_count < isp->cfg_num);
}

static inline int is_channel_alg_left(struct a1000_isp_device *isp)
{
	// pr_info("%s, alg_num %d, alg_count %d\n", __func__, isp->alg_num,
	// isp->alg_count);
	return (isp->alg_count < isp->alg_num);
}

static inline int is_channel_iq_left(struct a1000_isp_device *isp)
{
	// pr_info("%s, iq_num %d, iq_count %d\n", __func__, isp->iq_num,
	// isp->iq_count);
	return (isp->iq_count < isp->iq_num);
}

static inline int is_channel_pwl_lut_left(struct a1000_isp_device *isp)
{
	// pr_info("%s, pwl_lut_num %d, pwl_lut_count %d\n", __func__,
	// isp->pwl_lut_num, isp->pwl_lut_count);
	return (isp->pwl_lut_count < isp->pwl_lut_num);
}

static int next_channel_cfg_index(struct a1000_isp_device *isp)
{
	int index;
	int i;

	if (isp->cfg_count >= isp->cfg_num)
		return -1;

	if (channel_cfg_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable) {
				channel_cfg_list[index] = i;
				index++;
			}
		}
	}

	return channel_cfg_list[isp->cfg_count];
}

static int next_channel_alg_index(struct a1000_isp_device *isp)
{
	int index;
	int i;

	if (isp->alg_count >= isp->alg_num)
		return -1;

	if (channel_alg_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable &&
			    is_channel_use_alg(isp, i)) {
				channel_alg_list[index] = i;
				index++;
			}
		}
	}

	return channel_alg_list[isp->alg_count];
}

static int next_channel_iq_index(struct a1000_isp_device *isp)
{
	int index;
	int i;

	if (isp->iq_count >= isp->iq_num)
		return -1;

	if (channel_iq_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable &&
			    is_channel_use_iq(isp, i)) {
				channel_iq_list[index] = i;
				index++;
			}
		}
	}

	return channel_iq_list[isp->iq_count];
}

static int next_channel_pwl_lut_index(struct a1000_isp_device *isp)
{
	int index;
	int i;

	if (isp->pwl_lut_count >= isp->pwl_lut_num)
		return -1;

	if (channel_pwl_lut_list[0] < 0) {
		index = 0;
		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			if (isp->channels[i].enable &&
			    is_channel_use_pwl_lut(isp, i)) {
				channel_pwl_lut_list[index] = i;
				index++;
			}
		}
	}

	return channel_pwl_lut_list[isp->pwl_lut_count];
}

static int ispdrv_enter_cfg_state(struct a1000_isp_device *isp)
{
	send_next_cfg_to_fw(isp);
	isp->state = ISPDRV_STATE_SCFG;
	return 0;
}

static int ispdrv_enter_alg_state(struct a1000_isp_device *isp)
{
	send_next_alg_to_fw(isp);
	isp->state = ISPDRV_STATE_SALG;
	return 0;
}

static int ispdrv_enter_iq_state(struct a1000_isp_device *isp)
{
	send_next_iq_to_fw(isp);
	isp->state = ISPDRV_STATE_SIQ;
	return 0;
}

static int ispdrv_enter_pwl_lut_state(struct a1000_isp_device *isp)
{
	send_next_pwl_lut_to_fw(isp);
	isp->state = ISPDRV_STATE_SET_DSP_PWL;
	return 0;
}

static int ispdrv_enter_isp_dsp_sync_state(struct a1000_isp_device *isp)
{
	send_isp_dsp_sync_msg_to_fw(isp);
	isp->state = ISPDRV_STATE_ISP_DSP_SYNC;

	return 0;
}

static int ispdrv_enter_start_state(struct a1000_isp_device *isp)
{
	send_start_msg_to_fw(isp);
	isp->state = ISPDRV_STATE_START;
	return 0;
}

static int ispdrv_enter_work_state(struct a1000_isp_device *isp)
{
	atomic_set(&(isp->FW_config_done), 1);
	complete_all(&isp->FW_start_completion);
	isp->state = ISPDRV_STATE_WORK;
	return 0;
}

/* for debug mode, designed for isp video ioctl */
int ispdrv_enter_debug_state(struct a1000_isp_device *isp)
{
	isp->state = ISPDRV_STATE_DEBUG;
	return 0;
}

/* for debug mode, designed for isp video ioctl */
int ispdrv_leave_debug_state(struct a1000_isp_device *isp)
{
	isp->state = ISPDRV_STATE_WORK;
	return 0;
}

static void ispdrv_state_error(struct a1000_isp_device *isp, uint16_t message)
{
	pr_err("unexpected command %d in state %d\n", message, isp->state);
}

static int work_message_handler(struct a1000_isp_device *isp,
				struct media_command *msg, int64_t ktime)
{
	// pr_err("%s enter\n", __func__);
	int isp_entity_index;
	uint16_t cmd_main = msg->cmd_hdr.hdr_info.cmd_type_main;
	uint16_t cmd_minor = msg->cmd_hdr.hdr_info.cmd_type_minor;
	uint32_t *tick = (uint32_t *)msg->cmd_hdr.dst;

	if ((msg->cmd_hdr.src[0] == 'i') && (msg->cmd_hdr.src[1] == 'i')) {
		isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
				   (msg->cmd_hdr.src[3] - '0');
		if (isp_entity_index < 0) {
			pr_err("isp_entity_index = %d, error\n",
			       isp_entity_index);
			return -1;
		}
		isp->kthread_status = KTHREAD_STATUS_COPY_MSG;

		if (cmd_minor == MINOR_ISP_RAW_BUF_DONE) {
			copy_msg_to_video_cache(
				&(isp->channels[isp_entity_index].raw_video), cmd_main,
				cmd_minor, *tick, ktime, msg->user_cmd_data, 4);
		} else {
			copy_msg_to_video_cache(
				&(isp->channels[isp_entity_index].views_video), cmd_main,
				cmd_minor, *tick, ktime, msg->user_cmd_data, 4);
		}
		isp->kthread_status = KTHREAD_STATUS_COPY_COMPLETE;
	}

	if ((msg->cmd_hdr.src[0] == 'p') && (msg->cmd_hdr.src[1] == 'd')) {
		isp_entity_index = (msg->cmd_hdr.src[2] - '0') * 10 +
				   (msg->cmd_hdr.src[3] - '0');
		if (isp_entity_index < 0) {
			pr_err("isp_entity_index = %d, error\n",
			       isp_entity_index);
			return -1;
		}
		isp->kthread_status = KTHREAD_STATUS_COPY_MSG;

		/* change the pdns node to raw video node*/

		// copy_msg_to_video_cache(
		//	&(isp->channels[isp_entity_index].raw_video), cmd_main,
		//	cmd_minor, *tick, ktime, msg->user_cmd_data, 4);

		isp->kthread_status = KTHREAD_STATUS_COPY_COMPLETE;
	}

	return 0;
}

static int debug_message_handler(struct a1000_isp_device *isp,
				 struct media_command *cmd)
{
	isp_video_debug_isp_done(cmd);
	return 0;
}

static int ispdrv_message_handler(struct a1000_isp_device *isp,
				  struct media_command *cmd,
				  int64_t ipc_timestamp)
{
	uint16_t message = cmd->cmd_hdr.hdr_info.cmd_type_minor;
	// pr_err("recv command 0x%x in state %d\n", message, isp->state);
	int channel_id = 0;

	switch (isp->state) {
	// place ISPDRV_STATE_WORK at first to enhance efficiency
	case ISPDRV_STATE_WORK:
		if (message == MINOR_ISP_GET_IQINFO ||
		    message == MINOR_ISP_SET_IQINFO ||
		    message == MINOR_ISP_SET_VIEWINFO) {
			isp_get_iqinfo_t *ctrlinfo =
				(isp_get_iqinfo_t *)&(cmd->user_cmd_data[0]);
			// debug
			switch (message) {
			case MINOR_ISP_GET_IQINFO:
				pr_info("MINOR_ISP_GET_IQINFO RECV: sensor[%d] item[%d]: value:[%d]\n",
					ctrlinfo->sensorIndex, ctrlinfo->iqItem,
					ctrlinfo->iqVal);
				channel_id = ctrlinfo->sensorIndex;
				if (channel_id >= MAX_ISP_CHANNEL) {
					pr_err("invaild channel_id\n");
					return 1;
				}
				isp->ctrl_get_val = ctrlinfo->iqVal;
				isp->isp_ctrl_status = channel_id;
				wake_up(&isp->ctrl_recv_wq);
				break;
			case MINOR_ISP_SET_IQINFO:
				pr_info("MINOR_ISP_SET_IQINFO ACK: sensor[%d] item[%d]\n",
					ctrlinfo->sensorIndex,
					ctrlinfo->iqItem);
				break;
			case MINOR_ISP_SET_VIEWINFO:
				pr_info("MINOR_ISP_SET_VIEWINFO Frimware ACK\n");
				break;
			}
		} else {
			unsigned int follow_pack_num;
			unsigned int i;
			tSoneCmdp *cmdp;
			struct media_command *queue_tail;

			follow_pack_num = cmd->cmd_hdr.hdr_info.follow_pack_num;
			if (follow_pack_num)
				pr_debug("ISP MSG: follow_pack_num: %u\n",
					 follow_pack_num);
			i = 0;
			cmdp = (tSoneCmdp *)isp->cmdp_vaddr;
			queue_tail =
				&cmdp->ch[FW_CH_INDEX].cqueue.c0[0] +
				ARRAY_SIZE(cmdp->ch[FW_CH_INDEX].cqueue.c0);
			do {
				if (cmd >= queue_tail) {
					pr_debug(
						"ISP MSG: cmd %p exceed tail %p, rewind\n",
						cmd, queue_tail);
					cmd = &cmdp->ch[FW_CH_INDEX]
						       .cqueue.c0[0];
				}
				pr_debug(
					"ISP MSG: %d CMD SRC: %c%c%c%c, Userdata: 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
					i, cmd->cmd_hdr.src[0],
					cmd->cmd_hdr.src[1],
					cmd->cmd_hdr.src[2],
					cmd->cmd_hdr.src[3],
					cmd->user_cmd_data[0],
					cmd->user_cmd_data[1],
					cmd->user_cmd_data[2],
					cmd->user_cmd_data[3]);
				work_message_handler(isp, cmd, ipc_timestamp);
				i++;
				cmd++;
			} while (i <= follow_pack_num);
		}
		break;
	case ISPDRV_STATE_WAIT:
		if (message == MINOR_BOOT_DONE) {
			// pr_info("ISPDRV_STATE_WAIT, MINOR_BOOT_DONE\n");
			atomic_set(&(isp->FW_boot_done), 1);
			isp->ctrl_reg_base =
				ioremap(ISP_FW_CTRL_REG, ISP_FW_CTRL_REG_SIZE);
			if (isp->use_dsp)
				ispdrv_enter_isp_dsp_sync_state(isp);
			else if (is_channel_cfg_left(isp))
				ispdrv_enter_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_ISP_DSP_SYNC:
		if (message == MINOR_ISP_DSP_SYNC) {
			// pr_info("ISPDRV_STATE_ISP_DSP_SYNC,
			// MINOR_ISP_DSP_SYNC\n");
			if (is_channel_cfg_left(isp))
				ispdrv_enter_cfg_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SCFG:
		if (message == MINOR_ISP_BOOTLD_RECONF) {
			// pr_info("ISPDRV_STATE_SCFG,
			// MINOR_ISP_BOOTLD_RECONF\n");
			if (is_channel_cfg_left(isp))
				send_next_cfg_to_fw(isp);
			else if (is_channel_alg_left(isp))
				ispdrv_enter_alg_state(isp);
			else if (is_channel_iq_left(isp))
				ispdrv_enter_iq_state(isp);
			else if (is_channel_pwl_lut_left(isp))
				ispdrv_enter_pwl_lut_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SALG:
		if (message == MINOR_ISP_BOOTLD_ALGO_BIN) {
			// pr_info("ISPDRV_STATE_SALG,
			// MINOR_ISP_BOOTLD_ALGO_BIN\n");
			if (is_channel_alg_left(isp))
				send_next_alg_to_fw(isp);
			else if (is_channel_iq_left(isp))
				ispdrv_enter_iq_state(isp);
			else if (is_channel_pwl_lut_left(isp))
				ispdrv_enter_pwl_lut_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SIQ:
		if (message == MINOR_ISP_BOOTLD_IQ_BIN) {
			// pr_info("ISPDRV_STATE_SIQ,
			// MINOR_ISP_BOOTLD_IQ_BIN\n");
			if (is_channel_iq_left(isp))
				send_next_iq_to_fw(isp);
			else if (is_channel_pwl_lut_left(isp))
				ispdrv_enter_pwl_lut_state(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_SET_DSP_PWL:
		if (message == MINOR_ISP_DSP_IPC) {
			// pr_info("ISPDRV_STATE_SET_DSP_PWL,
			// MINOR_ISP_DSP_IPC\n");
			if (is_channel_pwl_lut_left(isp))
				send_next_pwl_lut_to_fw(isp);
			else
				ispdrv_enter_start_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_START:
		if (message == MINOR_ISP_START) {
			// pr_info("ISPDRV_STATE_START, MINOR_ISP_START\n");
			ispdrv_enter_work_state(isp);
		} else {
			ispdrv_state_error(isp, message);
		}
		break;
	case ISPDRV_STATE_DEBUG:
		debug_message_handler(isp, cmd);
		break;
	}

	return 0;
}

static int isp_kthread_recv_msg(void *data)
{
	struct a1000_isp_device *isp = (struct a1000_isp_device *)data;
	ipc_msg msg;
	int32_t ret;
	uint32_t msg_addr_offset;
	uint32_t msg_phy_addr;
	struct media_command *media_msg;
	int32_t timeout;

	while (1) {
		isp->kthread_status = KTHREAD_STATUS_RECV_MSG;
		timeout = -1;
		if (atomic_read(&(isp->streamon_count)) > 0)
			timeout = MAX_VIDEO_STREAM_TIMEOUT;

		ret = ipc_recv(isp->ipc_session_id, &msg, timeout);
		if (ret < 0) {
			if (get_isp_streamon_count(isp) != 0) {
				dev_err(isp->dev,
					"ipc_recv failed!!!, ret = %d\n", ret);
			}
			continue;
		}
		isp->ipc_rx_count++;
		isp->kthread_status = KTHREAD_STATUS_RECV_COMPLETE;
		/*analysis ipc message and get media msg*/
		msg_phy_addr = msg.data;
		msg_addr_offset =
			msg_phy_addr - (isp->cmdp_paddr & LOW_32_BIT_MASK);
		media_msg = (struct media_command *)(isp->cmdp_vaddr +
						     msg_addr_offset);

		ispdrv_message_handler(isp, media_msg, msg.timestamp);
	}

	return 0;
}

static int send_isp_dsp_sync_msg_to_fw(struct a1000_isp_device *isp)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_dsp_sync_t *dsp_sync_cmd;
	ipc_msg msg;

	media_cmd = isp->config_media_cmd;
	media_cmd_paddr = isp->media_cmd_paddr;
	memset(media_cmd, 0, sizeof(*media_cmd));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_DSP_SYNC;
	dsp_sync_cmd = (isp_dsp_sync_t *)(&media_cmd->user_cmd_data[0]);
	dsp_sync_cmd->dspCoreId = isp->dsp_core_id;
	dsp_sync_cmd->dspSyncDdrBase = isp->dsp_sync_ddr_base;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	return send_cmd_to_fw(isp, &msg);
}

static int send_start_msg_to_fw(struct a1000_isp_device *isp)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_start_t *start_cmd;
	ipc_msg msg;
	int ret;

	media_cmd = isp->config_media_cmd;
	media_cmd_paddr = isp->media_cmd_paddr;
	// pr_info("start media_cmd_paddr = 0x%x\n", media_cmd_paddr);
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_START;
	start_cmd = (isp_start_t *)(&media_cmd->user_cmd_data[0]);
	start_cmd->reconfDDRBase = isp->fbuf_paddr;
	start_cmd->reconfDDRSize = isp->fbuf_psize;
	start_cmd->txMsgMode = AttachMsg_TimeOut;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return 0;
}

static int send_config_to_fw(struct a1000_isp_device *isp,
			     // struct bst_isp_channel *isp_chn,
			     struct camera_dev *cam_dev, int chn_index)
{
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	u8 *payload_vaddr;
	u32 payload_paddr;
	isp_ld_reconf_t *config_cmd;
	ipc_msg msg;
	int ret;
	unsigned int i;

	media_cmd = isp->config_media_cmd;
	media_cmd_paddr = isp->media_cmd_paddr;

	payload_vaddr = isp->config_payload_vaddr +
			(isp->config_align_size * chn_index);
	payload_paddr = isp->config_payload_paddr +
			(isp->config_align_size * chn_index);

	dev_dbg(isp->dev, "%s, channel %d\n", __func__, chn_index);
	dev_dbg(isp->dev, "!!! payload_paddr = 0x%x\n", payload_paddr);
	// cam_dev->isp_data.viewinfo[0].scalerRemap =
	// isp->resize_core_id[(video->chn_index/4)];
	memcpy(payload_vaddr, &(cam_dev->isp_data), sizeof(ipc_reconf_t));
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_RECONF;
	config_cmd = (isp_ld_reconf_t *)(&media_cmd->user_cmd_data[0]);
	config_cmd->sensorIndex = chn_index;
	config_cmd->payloadAddr = payload_paddr;
	pr_debug("=== config_cmd->sensorIndex = %d\n", chn_index);
	pr_debug("payload content for chn: %d:\n", chn_index);
	for (i = 0; i < sizeof(ipc_reconf_t); i++)
		pr_debug("0x%02X\n", payload_vaddr[i]);

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);
	return 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void dump_config_data(struct camera_dev *cam_dev)
{
	ipc_reconf_t *conf;

	conf = &cam_dev->isp_data;

	pr_info("i2c = 0x%x, mipi = 0x%x, sensor = 0x%x, dev = 0x%x, rw = %d, type = %d, hblank = %d\n",
		conf->i2cRegBase, conf->mipiSensorIndex, conf->sensorIndex,
		conf->sensorDevID, conf->sensorRdWrMode, conf->sensorType,
		conf->hblank);

	pr_info("srcSel = %d, dataType = %d, width = %d, height = %d\n",
		conf->rawinfo.srcSel, conf->rawinfo.dataType,
		conf->rawinfo.width, conf->rawinfo.height);

	pr_info("hdrStaggerEn = %d, expNum = %d, dvpDummyVal = 0x%x\n",
		conf->rawinfo.hdrMode, conf->rawinfo.expNum,
		conf->rawinfo.dvpDummyVal);

	pr_info("viewFmt = %d, width = %d, height = %d\n",
		conf->viewinfo[0].viewFmt, conf->viewinfo[0].width,
		conf->viewinfo[0].height);
	pr_info("top = %d, bottom = %d, right = %d, left = %d\n",
		conf->viewinfo[0].topCropBefore,
		conf->viewinfo[0].botCropBefore,
		conf->viewinfo[0].lefCropBefore,
		conf->viewinfo[0].rigCropBefore);

	pr_info("viewFmt = %d, width = %d, height = %d\n",
		conf->viewinfo[1].viewFmt, conf->viewinfo[1].width,
		conf->viewinfo[1].height);
	pr_info("top = %d, bottom = %d, right = %d, left = %d\n",
		conf->viewinfo[1].topCropBefore,
		conf->viewinfo[1].botCropBefore,
		conf->viewinfo[1].lefCropBefore,
		conf->viewinfo[1].rigCropBefore);

	pr_info("viewFmt = %d, width = %d, height = %d\n",
		conf->viewinfo[2].viewFmt, conf->viewinfo[2].width,
		conf->viewinfo[2].height);
	pr_info("top = %d, bottom = %d, right = %d, left = %d\n",
		conf->viewinfo[2].topCropBefore,
		conf->viewinfo[2].botCropBefore,
		conf->viewinfo[2].lefCropBefore,
		conf->viewinfo[2].rigCropBefore);

	pr_info("pdnsMode = %d, pdnsViewSel = %d\n", conf->pdnsinfo.pdnsMode,
		conf->pdnsinfo.pdnsViewSel);
}

#pragma GCC diagnostic pop

static int send_next_cfg_to_fw(struct a1000_isp_device *isp)
{
	int index;

	index = next_channel_cfg_index(isp);
	if (index >= 0) {
		send_config_to_fw(isp, isp->channels[index].cam_dev, index);
		isp->cfg_count++;
	}

	return 0;
}

static struct bin_info_t *get_bin_info(int chn_index, const char *filename)
{
	int i;
	int ret;

	for (i = 0; i < s_isp_algobin.file_num; i++) {
		ret = strncmp(filename, s_isp_algobin.algo_bin[i].filename,
			      MAX_BIN_NAME_LEN);
		if (ret == 0) {
			// same, return file_info
			s_isp_algobin.algo_bin[i].camera_mask |= BIT(chn_index);
			pr_info("find, mask = 0x%x, name = %s, addr = 0x%x\n",
				s_isp_algobin.algo_bin[i].camera_mask,
				s_isp_algobin.algo_bin[i].filename,
				s_isp_algobin.algo_bin[i].payload_paddr);
			return &s_isp_algobin.algo_bin[i];
		}
	}
	// not found, return NULL
	return NULL;
}

static int add_bin_info_to_array(struct bin_info_t *bin_info, int chn_index,
				 const char *filename)
{
	struct bin_info_t *info;
	int index;

	if (s_isp_algobin.file_num >= MAX_ISP_CHANNEL) {
		pr_err("algo file exceeds MAX_ISP_CHANNEL, error\n");
		return -1;
	}
	index = s_isp_algobin.file_num;
	info = &s_isp_algobin.algo_bin[index];
	info->payload_vaddr = bin_info->payload_vaddr;
	info->payload_paddr = bin_info->payload_paddr;
	info->size = bin_info->size;
	strncpy(info->filename, filename, MAX_BIN_NAME_LEN);
	info->camera_mask |= BIT(chn_index);

	s_isp_algobin.file_num++;

	return 0;
}

static int send_algo_bin_to_fw(struct a1000_isp_device *isp, int chn_index,
			       const char *filename)
{
	int ret;
	const struct firmware *firmware_p;
	u32 media_cmd_paddr;
	struct media_command *media_cmd;
	isp_ld_algo_bin_t *config_cmd;
	ipc_msg msg;
	u32 file_start_paddr;
	u8 *file_start;
	u8 *file_end;
	int file_size;
	char full_name[MAX_BIN_NAME_LEN];

	struct bin_info_t *bin_info;

	bin_info = get_bin_info(chn_index, filename);
	if (bin_info != NULL) {
		// found it, nothing to do
		file_start_paddr = bin_info->payload_paddr;
	} else {
		struct bin_info_t info;
		// not found, load it
		memset(full_name, 0, MAX_BIN_NAME_LEN);
		snprintf(full_name, MAX_BIN_NAME_LEN, "isp/algo/%s", filename);
		pr_debug("full_name = %s\n", full_name);
		ret = request_firmware(&firmware_p, full_name, isp->dev);
		if (ret) {
			pr_err("there is no %s could be used\n", full_name);
			return -1;
		}

		file_start = isp->algobin_next_vaddr;
		file_start_paddr = isp->algobin_next_paddr;
		file_size = firmware_p->size + 1; // one byte for eof
		if ((file_start + file_size) > isp->payload_end_vaddr) {
			pr_err("bin file %s size = %d exceed\n", full_name,
			       file_size);
			return -1;
		}

		memcpy(file_start, firmware_p->data, firmware_p->size);
		file_end = file_start + firmware_p->size;
		*file_end = 0xa;

		info.payload_vaddr = file_start;
		info.payload_paddr = file_start_paddr;
		info.size = file_size;
		add_bin_info_to_array(&info, chn_index, filename);
		file_size = (file_size + ISP_MSG_PAYLOAD_SIZE_MASK) &
			    (~ISP_MSG_PAYLOAD_SIZE_MASK);
		isp->algobin_next_vaddr += file_size;
		isp->algobin_next_paddr += file_size;
	}

	media_cmd = isp->config_media_cmd;
	media_cmd_paddr = isp->media_cmd_paddr;

	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_ALGO_BIN;
	config_cmd = (isp_ld_algo_bin_t *)(&media_cmd->user_cmd_data[0]);
	config_cmd->sensorIndex = chn_index;
	config_cmd->payloadAddr = file_start_paddr;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = media_cmd_paddr;
	ret = send_cmd_to_fw(isp, &msg);

	return 0;
}

static int send_iq_bin_to_fw(struct a1000_isp_device *isp, int chn_index,
			     const char *name)
{
	int ret;
	const struct firmware *fw;
	struct media_command *media_cmd;
	isp_ld_iq_bin_t *iq_bin;
	ipc_msg msg;
	char path[MAX_BIN_NAME_LEN];

	/* step 1: open iq bin file */
	/* to do: move snprintf to camera dts parse funcs */

	memset(path, 0, MAX_BIN_NAME_LEN);
	snprintf(path, MAX_BIN_NAME_LEN, "isp/algo/%s", name);

	ret = request_firmware(&fw, path, isp->dev);
	if (ret) {
		pr_err("request firmware %s failed\n", name);
		return -1;
	}

	/* step 2: copy file to payload memory area */
	/* to do: write macro, check payload range */
	if ((isp->config_payload_vaddr + fw->size) > isp->payload_end_vaddr) {
		pr_err("%s size %ld, exceed\n", name, fw->size);
		return -1;
	}
	memcpy(isp->config_payload_vaddr, fw->data, fw->size);

	/* step 3: send ipc message to firmare */
	/* to do: abstract cmd send func/marco */
	/* send_cmd(MINOR_ISP_BOOTLD_IQ_BIN, cmd_struct, void* data, size_t
	 * size)
	 */
	media_cmd = isp->config_media_cmd;
	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_IQ_BIN;
	iq_bin = (isp_ld_iq_bin_t *)(media_cmd->user_cmd_data);
	iq_bin->sensorIndex = chn_index;
	iq_bin->payloadAddr = isp->config_payload_paddr;
	iq_bin->payloadSize = fw->size;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp->media_cmd_paddr;

	pr_info("%s, channel %d, path %s, size %ld\n", __func__, chn_index,
		path, fw->size);

	send_cmd_to_fw(isp, &msg);

	return 0;
}

static int send_next_alg_to_fw(struct a1000_isp_device *isp)
{
	int index = next_channel_alg_index(isp);

	if (index >= 0) {
		const char *filename =
			is_channel_mode_online(isp, index) ?
				isp->channels[index].cam_dev->algo_online :
				isp->channels[index].cam_dev->algo_offline;
		send_algo_bin_to_fw(isp, index, filename);
		isp->alg_count++;
	}

	return 0;
}

static int send_next_iq_to_fw(struct a1000_isp_device *isp)
{
	int index = next_channel_iq_index(isp);

	if (index >= 0) {
		const char *filename =
			is_channel_mode_online(isp, index) ?
				isp->channels[index].cam_dev->iq_online :
				isp->channels[index].cam_dev->iq_offline;
		send_iq_bin_to_fw(isp, index, filename);
		isp->iq_count++;
	}

	return 0;
}

static int send_pwl_lut_bin_to_fw(struct a1000_isp_device *isp, int chn_index,
				  const char *name)
{
	int ret;
	const struct firmware *fw;
	struct media_command *media_cmd;
	isp_dsp_ipc_t *dsp_ipc_cmd;
	ipc_msg msg;
	char path[MAX_BIN_NAME_LEN];

	/* step 1: open pwl lut bin file */
	memset(path, 0, sizeof(path));
	snprintf(path, sizeof(path), "isp/algo/%s", name);
	ret = request_firmware(&fw, path, isp->dev);
	if (ret) {
		pr_err("%s: request firmware %s failed\n", path, __func__);
		return -1;
	}

	/* step 2: copy file to payload memory area */
	/* to do: write macro, check payload range */
	if ((isp->config_payload_vaddr + fw->size) > isp->payload_end_vaddr) {
		pr_err("%s size %ld, exceed\n", name, fw->size);
		return -1;
	}
	memcpy(isp->config_payload_vaddr, fw->data, fw->size);
	// pr_info("%s: firmware: %s, size: %lu\n", __func__, path, fw->size);

	/* step 3: send ipc message to firmare */
	media_cmd = isp->config_media_cmd;
	memset(media_cmd, 0, sizeof(*media_cmd));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_DSP_IPC;
	dsp_ipc_cmd = (isp_dsp_ipc_t *)(&media_cmd->user_cmd_data[0]);
	dsp_ipc_cmd->sensorIndex = chn_index;
	;
	dsp_ipc_cmd->dspIpcItem = DSPIPC_LUT_CREATE;
	dsp_ipc_cmd->infoBase = isp->config_payload_paddr;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp->media_cmd_paddr;

	return send_cmd_to_fw(isp, &msg);
}

static int send_next_pwl_lut_to_fw(struct a1000_isp_device *isp)
{
	int index = next_channel_pwl_lut_index(isp);

	if (index >= 0) {
		const char *filename = isp->channels[index].cam_dev->pwl_lut;

		send_pwl_lut_bin_to_fw(isp, index, filename);
		isp->pwl_lut_count++;
	}

	return 0;
}

static int isp_channel_get_port_info(struct a1000_isp_device *isp_dev,
				     struct device_node *core_dt, int core_id)
{
	struct device_node *ep = NULL;
	struct device_node *remote_port = NULL;
	struct device_node *remote_node = NULL;
	struct device_node *port = NULL;
	int sn;
	int i;

	struct bst_isp_channel *channel;

	for (i = 0; i < MAX_CHANNEL_PER_CORE; i++) {
		sn = core_id * MAX_CHANNEL_PER_CORE + i;
		channel = &isp_dev->channels[sn];
		channel->sn = sn;
		channel->isp_core_id = core_id;
		channel->index_in_core = i;
		channel->isp_chn_id = ((core_id << 2) | i);
		channel->isp = isp_dev;
		port = of_graph_get_port_by_id(core_dt, i);
		if (port == NULL)
			continue;

		channel->of_node = port;
		channel->fwnode = of_fwnode_handle(port);
		dev_dbg(isp_dev->dev,
			"channel %d , port = %s, fwnode = 0x%lx\n", sn,
			port->full_name, (unsigned long)channel->fwnode);

		ep = of_get_child_by_name(port, "endpoint");
		if (ep) {
			int ret;
			int reg;
			int id;
			// of_graph_get_remote_node
			remote_port = of_graph_get_remote_port(ep);
			if (!remote_port) {
				pr_err("no valid remote port\n");
				return -EINVAL;
			}
			channel->remote_fwnode = of_fwnode_handle(remote_port);
			dev_dbg(isp_dev->dev,
				"channel %d , remote port = %s, remote_fwnode = 0x%lx\n",
				sn, remote_port->full_name,
				(unsigned long)channel->remote_fwnode);

			ret = of_property_read_u32(remote_port, "reg", &reg);
			if (ret < 0) {
				dev_err(isp_dev->dev, "remote reg error\n");
				return -EINVAL;
			}
			remote_node = of_graph_get_remote_port_parent(ep);
			// of_node_put(endpoint_node);
			if (!remote_node) {
				pr_err("no valid remote node\n");
				return -EINVAL;
			}
			dev_dbg(isp_dev->dev, "channel %d , remote_node = %s\n",
				sn, remote_node->full_name);

			ret = of_property_read_u32(remote_node, "id", &id);
			if (ret < 0) {
				dev_err(isp_dev->dev, "remote id error\n");
				return -EINVAL;
			}
			channel->enable = true;
			channel->remote_mipi_id = id;
			channel->remote_mipi_vc_index = reg;

			if (isp_dev->csi_asd[id].mipi_connected == 0) {
				isp_dev->csi_asd[id].mipi_connected++;
				isp_dev->total_subdev++;
				isp_dev->csi_asd[id].mipi_fwnode =
					of_fwnode_handle(remote_node);
			} else if (isp_dev->csi_asd[id].mipi_fwnode ==
				   of_fwnode_handle(remote_node)) {
				isp_dev->csi_asd[id].mipi_connected++;
			} else {
				pr_err("error, same id, but not same node\n");
				return -1;
			}

			isp_dev->core_channel_num[core_id]++;

			dev_dbg(isp_dev->dev,
				"remote node name = %s, remote reg = %d, id = %d, fwnode = 0x%lx\n",
				remote_node->full_name, reg, id,
				(unsigned long)isp_dev->csi_asd[id].mipi_fwnode);
		}
	}

	return isp_dev->core_channel_num[core_id];
}

static int bst_isp_parse_dt(struct a1000_isp_device *isp_dev)
{
	int ret;
	int num_channels;
	u32 core_id;
	struct device_node *node = isp_dev->dev->of_node;
	struct device_node *core_dt = NULL;

	if (!node)
		return -EINVAL;

	isp_dev->use_dsp = of_property_read_bool(node, "use-dsp");
	if (isp_dev->use_dsp) {
		ret = of_property_read_u8(node, "dsp-core-id",
					  &isp_dev->dsp_core_id);
		if (ret < 0)
			return -EINVAL;
		ret = of_property_read_u32(node, "dsp-sync-ddr-base",
					   &isp_dev->dsp_sync_ddr_base);
		if (ret < 0)
			return -EINVAL;
	}

	for_each_child_of_node(node, core_dt) {
		if (!core_dt->name || of_node_cmp(core_dt->name, "core"))
			continue;
		ret = of_property_read_u32(core_dt, "id", &core_id);
		if (ret < 0)
			return -EINVAL;

		num_channels =
			isp_channel_get_port_info(isp_dev, core_dt, core_id);
		if (num_channels < 0) {
			pr_err("isp_channel_get_port_info error\n");
			return -1;
		}

		// ret = of_property_read_u8(core_dt, "crop-core",
		// &(isp_dev->resize_core_id[core_id])); if(ret < 0){
		//	pr_info("Core %d, crop-core disabled\n", core_id);
		//	isp_dev->resize_core_id[core_id] = 0;
		// }else{
		//	pr_info("Core %d, crop-core %d\n", core_id,
		// &isp_dev->resize_core_id[core_id]);
		// }

		if (core_id == 0) {
			struct device_node *remote_node = NULL;
			struct device_node *port = NULL;

			port = of_graph_get_port_by_id(core_dt,
						       HDMI_DTS_PORT_NUM);
			if (port == NULL)
				continue;
			remote_node = of_graph_get_remote_node(
				core_dt, HDMI_DTS_PORT_NUM, 0);
			if (remote_node == NULL) {
				continue;
			} else {
				isp_dev->hdmi_node = port;
				isp_dev->hdmi_handle = of_fwnode_handle(port);
				isp_dev->remote_hdmi =
					of_fwnode_handle(remote_node);
				isp_dev->total_subdev++;
			}
		}
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Media Operations
 */
static const struct media_entity_operations isp_entity_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int isp_dev_notify_bound(struct v4l2_async_notifier *async_notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_subdev *asd)
{
	struct bst_csi_device *pcsi_dev;
	struct csi_async_dev *pcsi_async;
	struct a1000_isp_device *pisp_dev;
	struct bst_isp_channel *isp_channel;
	int i;

	pisp_dev =
		container_of(async_notifier, struct a1000_isp_device, notifier);
	if (sd->fwnode == pisp_dev->remote_hdmi) {
		pisp_dev->hdmi_cam =
			container_of(sd, struct camera_dev, subdev);
	} else {
		pcsi_dev = container_of(sd, struct bst_csi_device, subdev);
		pcsi_async = container_of(asd, struct csi_async_dev, async_dev);
		pcsi_dev->sd_state = BST_SUBDEV_STATE_BOUND;
		pcsi_async->csi_dev = pcsi_dev;

		for (i = 0; i < MAX_ISP_CHANNEL; i++) {
			isp_channel = &pisp_dev->channels[i];
			pr_debug(
				"remote_mipi_id = %d, vc_index = %d, pcsi_dev->csi_id = %d\n",
				isp_channel->remote_mipi_id,
				isp_channel->remote_mipi_vc_index,
				pcsi_dev->csi_id);

			if (isp_channel->remote_mipi_id == pcsi_dev->csi_id) {
				int vc_index;

				vc_index = isp_channel->remote_mipi_vc_index;
				isp_channel->csi_channel =
					&(pcsi_dev->csi_vc[vc_index]);
				pr_debug("isp_channel->csi_channel = 0x%p\n",
					 isp_channel->csi_channel);
			} else {
				continue;
			}
		}
	}

	return 0;
}

static void isp_dev_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void dump_channel_info(struct bst_isp_channel *isp_channel)
{
	struct bst_csi_channel *csi_chan;
	struct camera_dev *cam;

	csi_chan = isp_channel->csi_channel;
	dev_err(isp_channel->isp->dev, "====== isp channel sn = %d\n",
		isp_channel->sn);
	dev_err(isp_channel->isp->dev, "csi channel sn = %d\n",
		csi_chan->sn_in_all_csi);

	cam = isp_channel->cam_dev;
	dev_err(isp_channel->isp->dev, "camera type = %s\n", cam->type_name);
}

#pragma GCC diagnostic pop

static int isp_dev_notify_complete(struct v4l2_async_notifier *async_notifier)
{
	struct a1000_isp_device *isp_dev;
	int ret;

	ret = v4l2_device_register_subdev_nodes(async_notifier->v4l2_dev);
	isp_dev =
		container_of(async_notifier, struct a1000_isp_device, notifier);

	return 0;
};

static const struct v4l2_async_notifier_operations isp_dev_async_ops = {
	.bound = isp_dev_notify_bound,
	.unbind = isp_dev_notify_unbind,
	.complete = isp_dev_notify_complete,
};

static int init_isp_one_channel(struct bst_isp_channel *isp_channel, int index)
{
	int ret;

	isp_channel->is_hdmi = false;
	isp_channel->pads[ISP_CHANNEL_SINK_PAD].flags = MEDIA_PAD_FL_SINK;
	isp_channel->pads[ISP_CHANNEL_SOURCE_NORMAL].flags =
		MEDIA_PAD_FL_SOURCE;
	isp_channel->pads[ISP_CHANNEL_SOURCE_PDNS].flags = MEDIA_PAD_FL_SOURCE;
	atomic_set(&isp_channel->is_streaming, 0);
	isp_channel->entity.function = MEDIA_ENT_F_IO_V4L;
	isp_channel->entity.ops = &isp_entity_media_ops;

	ret = media_entity_pads_init(&isp_channel->entity, ISP_CHANNEL_PAD_NUM,
				     isp_channel->pads);
	if (ret < 0)
		return ret;

	return 0;
}

static int init_isp_channel_devs(struct a1000_isp_device *isp_dev)
{
	int i;
	struct bst_isp_channel *isp_channel;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_channel = &isp_dev->channels[i];
		if (!isp_channel->enable) {
			dev_info(isp_dev->dev,
				 "%s channel %d not enabled, skip\n", __func__,
				 i);
			continue;
		}
		init_isp_one_channel(isp_channel, i);
	}

	return 0;
}

static int init_isp_dev(struct a1000_isp_device *isp_dev)
{
	int i;
	int ret;
	int asd_index;

	isp_dev->media_dev.dev = isp_dev->dev;
	strlcpy(isp_dev->media_dev.model, "BST A1000 ISP",
		sizeof(isp_dev->media_dev.model));
	isp_dev->media_dev.ops = &isp_media_ops;
	media_device_init(&isp_dev->media_dev);
	media_device_register(&isp_dev->media_dev);

	isp_dev->v4l2_dev.mdev = &isp_dev->media_dev;

	snprintf(isp_dev->v4l2_dev.name, sizeof(isp_dev->v4l2_dev.name),
		 "a1000_isp_v4l2");

	ret = v4l2_device_register(isp_dev->dev, &isp_dev->v4l2_dev);
	if (ret < 0) {
		dev_err(isp_dev->dev,
			"%s: V4L2 device registration failed (%d)\n", __func__,
			ret);
		return -1;
	}

	v4l2_async_notifier_init(&isp_dev->notifier);
	asd_index = 0;
	isp_dev->notifier.ops = &isp_dev_async_ops;

	for (i = 0; i < MAX_MIPI_DEVICE_NUM; i++) {
		isp_dev->csi_asd[i].isp_parent = isp_dev;
		if (isp_dev->csi_asd[i].mipi_fwnode == NULL) {
			pr_debug("i = %d, mipi not connected\n", i);
			continue;
		}

		isp_dev->csi_asd[i].async_dev.match_type =
			V4L2_ASYNC_MATCH_FWNODE;
		isp_dev->csi_asd[i].async_dev.match.fwnode =
			isp_dev->csi_asd[i].mipi_fwnode;
		v4l2_async_notifier_add_subdev(
			&isp_dev->notifier, &(isp_dev->csi_asd[i].async_dev));
		asd_index++;
	}

	if (isp_dev->remote_hdmi != NULL) {
		isp_dev->hdmi_async.match_type = V4L2_ASYNC_MATCH_FWNODE;
		isp_dev->hdmi_async.match.fwnode = isp_dev->remote_hdmi;
		v4l2_async_notifier_add_subdev(&isp_dev->notifier,
					       &(isp_dev->hdmi_async));
	}
	ret = v4l2_async_notifier_register(&isp_dev->v4l2_dev,
					   &(isp_dev->notifier));
	if (ret < 0)
		dev_err(isp_dev->dev,
			"v4l2_async_notifier_register register failed\n");

	return 0;
}

static int init_isp_one_channel_videos(struct a1000_isp_device *isp_dev,
				       struct bst_isp_channel *isp_channel,
				       int chn_id)
{
	int ret;
	struct a1000_isp_video *video;

	// init views_video
	video = &(isp_channel->views_video);
	isp_channel_init_views_video(isp_dev, isp_channel, chn_id);
	ret = a1000_isp_video_register(video, &isp_dev->v4l2_dev);
	if (ret < 0)
		dev_info(isp_dev->dev, "%s register view video failed\n",
			 __func__);
	// must after video register
	// ret = media_create_pad_link(&isp_channel->subdev.entity, 1,
	//			    &(video->video.entity), 0, 0);

	// init raw video
	video = &(isp_channel->raw_video);
	isp_channel_init_raw_video(isp_dev, isp_channel, chn_id);
	ret = a1000_isp_video_register(video, &isp_dev->v4l2_dev);
	if (ret < 0) {
		dev_err(isp_dev->dev, "%s register raw video failed\n",
			__func__);
		return -1;
	}
	// must after video register
	// ret = media_create_pad_link(&isp_channel->subdev.entity, 2,
	//			    &(video->video.entity), 0, 0);

	return 0;
}

static int init_isp_videos(struct a1000_isp_device *isp_dev)
{
	int i;
	struct bst_isp_channel *isp_channel;
	int payload_size;

	payload_size = ISP_CTRL_PAYLOAD_SIZE * MAX_ISP_CHANNEL + 1;
	payload_size = (payload_size + ISP_MSG_PAYLOAD_SIZE_MASK) &
		       (~ISP_MSG_PAYLOAD_SIZE_MASK);
	if ((isp_dev->algobin_next_vaddr + payload_size) >
	    isp_dev->payload_end_vaddr) {
		pr_err("ipc_iqinfo_t size %d, exceed\n", payload_size);
		return -1;
	}

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_channel = &isp_dev->channels[i];
		isp_channel->ctrl_payload_vaddr =
			isp_dev->algobin_next_vaddr + i * ISP_CTRL_PAYLOAD_SIZE;
		isp_channel->ctrl_payload_paddr =
			isp_dev->algobin_next_paddr + i * ISP_CTRL_PAYLOAD_SIZE;

		if (!isp_channel->enable) {
			pr_debug("%s channel %d not enabled, skip\n", __func__,
				 i);
			continue;
		}
		init_isp_one_channel_videos(isp_dev, isp_channel, i);
	}

	isp_dev->algobin_next_vaddr += payload_size;
	isp_dev->algobin_next_paddr += payload_size;

	// pr_info("algo_vaddr After ctrl: 0x%hhn, paddr = 0x%x,
	// ctrl_payload_vaddr : 0x%hhn, ctrl_payload_paddr : 0x%x\n",
	//	isp_dev->algobin_next_vaddr,
	//	isp_dev->algobin_next_paddr,
	//	isp_channel->ctrl_payload_vaddr,
	//	isp_channel->ctrl_payload_paddr);

	return 0;
}

static int calc_core0_cameras(struct a1000_isp_device *isp)
{
	int i;
	int count = 0;

	for (i = 0; i < MAX_CHANNEL_PER_CORE; i++) {
		if (isp->channels[i].cam_dev) {
			if (isp->channels[i].cam_dev->power_on)
				count++;
		}
	}

	return count;
}

int isp_power_subdevs(struct a1000_isp_device *isp)
{
	v4l2_device_call_all(&isp->v4l2_dev, 0, core, s_power, 1);

	return 0;
}

static int update_isp_channel_status(struct a1000_isp_device *isp)
{
	int i;
	int count;
	struct bst_isp_channel *isp_chn;

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		isp_chn = &isp->channels[i];
		pr_debug("isp_chn->enable = %d, isp_chn->csi_channel = 0x%p\n",
			 isp_chn->enable, isp_chn->csi_channel);
		if (isp_chn->enable) {
			if ((isp_chn->csi_channel) &&
			    (isp_chn->csi_channel->connected)) {
				isp->total_channel++;
				pr_debug("=== isp->total_channel = %d\n",
					 isp->total_channel);
				isp_chn->cam_dev =
					isp_chn->csi_channel->cam_dev;

		if (isp_chn->cam_dev) {
		    isp_chn->camera_raw_width =
			isp_chn->cam_dev->isp_data.rawinfo.width;
		    isp_chn->camera_raw_height =
			isp_chn->cam_dev->isp_data.rawinfo.height;
		}

		isp_chn->cam_dev->isp_data.mipiSensorIndex =
					isp_chn->csi_channel->csi_chn_id;
				isp_chn->cam_dev->isp_data.sensorIndex =
					isp_chn->isp_chn_id;
				isp->cfg_num++;
				if (is_channel_use_alg(isp, i))
					isp->alg_num++;
				if (is_channel_use_iq(isp, i))
					isp->iq_num++;
				if (is_channel_use_pwl_lut(isp, i))
					isp->pwl_lut_num++;
			} else {
				isp_chn->enable = false;
			}
		}
	}

	count = calc_core0_cameras(isp);
	if ((count == 0) && (isp->hdmi_cam != NULL)) {
		// core0 no camera power on,
		// enable hdmi by default;
		isp->hdmi_detected = true;
	}
	isp->core0_active_cam = count;

	if (isp->hdmi_detected) {
		isp->channels[0].cam_dev = isp->hdmi_cam;
		isp->channels[0].enable = true;
		isp->channels[0].is_hdmi = true;
		isp->total_channel++;
		isp->cfg_num++;
	}

	pr_err("cfg_num %d, alg_num %d\n", isp->cfg_num, isp->alg_num);

	return 0;
}

// after v4l2_async_notifier_register, the deser_hub gets the camera status
// at first, update mipi status from deser hub,
// then update isp channel status from mipi channel
static int update_camera_status(struct a1000_isp_device *isp)
{
	int i;
	struct bst_csi_device *pcsi_dev;

	for (i = 0; i < MAX_MIPI_DEVICE_NUM; i++) {
		pcsi_dev = isp->csi_asd[i].csi_dev;
		if (pcsi_dev)
			update_camera_status_in_csi(pcsi_dev);
	}
	// update isp channel
	update_isp_channel_status(isp);

	return 0;
}

/*
 *	misc_deivce : isp info
 */

static long isp_misc_ioctl(struct file *pfile, unsigned int cmd,
			   unsigned long args)
{
	int i;
	struct isp_misc_device msg;
	struct isp_misc_device *argp = (struct isp_misc_device __user *)args;
	struct a1000_isp_device *isp = container_of(
		globle_isp_misc_device, struct a1000_isp_device, misc_device);
	struct camera_dev *c_dev;
	int camera_num = 0;

	switch (cmd) {
	case IOC_ECHO_ISP_MISC:
		{
			copy_from_user(&msg, argp,
				       sizeof(struct isp_misc_device));
			pr_err("recv user req echo %u\n", msg.camera_num);
			return 0;
		}
	case IOC_GET_VAILED_CAMERA_INFO:
		{
			pr_info("%s : IOC_GET_VAILED_CAMERA_INFO\n", __func__);
			if (isp->channels[0].cam_dev == isp->hdmi_cam) {
				pr_info("camera[0] is HDMI\n");
				i = 1;
			} else {
				i = 0;
			}
			for (; i < MAX_ISP_CHANNEL; i++) {
				if (isp->channels[i].enable) {
					c_dev = isp->channels[i].cam_dev;
					msg.cam_info[camera_num].camera_id =
						isp->channels[i].isp_chn_id;

					// write camera_name
					if (c_dev->camera_name != NULL &&
					    strcmp(c_dev->camera_name, "") != 0)
						strcpy(msg.cam_info[camera_num]
							       .camera_name,
						       c_dev->camera_name);
					else
						strcpy(msg.cam_info[camera_num]
							       .camera_name,
						       "unknown camera");

					// read  & write  camera is_streaming
					msg.cam_info[camera_num].is_streaming =
						atomic_read(
							&(c_dev->is_streaming));

					pr_info("copy over[%d]  id:[%d]  name:[%s] is_stream:[%d]\n",
						camera_num,
						msg.cam_info[camera_num]
							.camera_id,
						msg.cam_info[camera_num]
							.camera_name,
						msg.cam_info[camera_num]
							.is_streaming);
					camera_num++;
				}
			}
			msg.camera_num = camera_num;
			pr_info("copy_to_user camra_num is %d\n",
				msg.camera_num);
			return copy_to_user(argp, &msg,
					    sizeof(struct isp_misc_device)) ?
				       -EFAULT :
				       0;
		}
	case IOC_SET_MIPI_TRIGGER_MODE:
		{
			struct trigger_info info;
			struct deser_hub_dev *hub_dev;

			copy_from_user(&info,
				       (struct trigger_info __user *)args,
				       sizeof(struct trigger_info));
			pr_err("mipi_id: %d ,trigger_mode: %d,camera_trigger_gpio_port: %d\n",
			       info.mipi_id, info.trigger_mode,
			       info.camera_trigger_gpio_port);
			if (info.mipi_id > 4 || info.mipi_id < 0) {
				pr_err("invalid mipi_id\n");
				return -EINVAL;
			}
			if (info.trigger_mode == INTERNAL_TRIGGER_MODE) {
				pr_err("%s: INTERNAL_TRIGGER_MODE", __func__);
				hub_dev = isp->csi_asd[info.mipi_id]
						  .csi_dev->deser;
				if (hub_dev == NULL) {
					pr_err("hub_device isn't exist!\n");
					return -EINVAL;
				}
				hub_dev->internal_trigger_sync(
					hub_dev, info.camera_trigger_gpio_port,
					info.internal_trigger_fps);
			} else if (info.trigger_mode == EXTERNAL_TRIGGER_MODE) {
				pr_err("%s: EXTERNAL_TRIGGER_MODE", __func__);
				hub_dev = isp->csi_asd[info.mipi_id]
						  .csi_dev->deser;
				if (hub_dev == NULL) {
					pr_err("hub_device isn't exist!\n");
					return -EINVAL;
				}
				hub_dev->external_trigger_sync(
					hub_dev, info.external_freq, info.target_freq, info.fsync_in, info.fsync_out, info.camera_trigger_gpio_port,
					info.deser_trigger_gpio_port);
			} else {
				pr_err("invalid trigger_mode");
			}
			return 0;
		}
	default:
		return -EINVAL;
	}
}

static const struct file_operations isp_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = isp_misc_ioctl,
};

static struct miscdevice isp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp_misc",
	.fops = &isp_misc_fops,
};

#define A1000B_ISP_ENABLE_REG 0x33002180

static int enable_isp(void)
{
	uint32_t status;
	void *isp_enable_reg = NULL;

	isp_enable_reg = ioremap(A1000B_ISP_ENABLE_REG, 0x4);
	if (isp_enable_reg == NULL) {
		pr_err("ioremap(A1000B_VSP_ENABLE_REG failed\n");
		return -1;
	}

	status = readl_relaxed(isp_enable_reg);
	if (status != 0x1ffff) {
		status = 0xffffffff;
		writel_relaxed(status, isp_enable_reg);
	}

	status = readl_relaxed(isp_enable_reg);

	return 0;
}

/*
 * isp_probe - Probe ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Returns 0 if successful,
 *   -ENOMEM if no memory available,
 *   -ENODEV if no platform device resources found
 *     or no space for remapping registers,
 *   -EINVAL if couldn't install ISR,
 *   or clk_get return error value.
 */
static int isp_probe(struct platform_device *pdev)
{
	struct a1000_isp_device *isp;
	struct device *dev = &pdev->dev;
	int ret;
	uint32_t fbuf_addr;
	uint32_t fbuf_size;

	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	globle_isp_misc_device = &isp->misc_device;
	if (!isp)
		return -ENOMEM;

	dev_info(&pdev->dev, "%s\n", __func__);
	if (of_device_is_compatible(dev->of_node, "bst,a1000b-isp")) {
		dev_info(&pdev->dev, "%s\n", __func__);
		enable_isp();
	}

	mutex_init(&isp->isp_mutex);
	isp->dev = &pdev->dev;
	isp->ipc_session_id = 0;
	memcpy(isp->revision, A1000_ISP_REVISION, sizeof(A1000_ISP_REVISION));
	isp->ipc_rx_count = 0;
	isp->ipc_tx_count = 0;
	atomic_set(&isp->streamon_count, 0);
	atomic_set(&isp->FW_load_started, 0);
	atomic_set(&isp->FW_boot_done, 0);
	atomic_set(&isp->FW_config_done, 0);
	// atomic_set(&isp->FW_resize_id, 0);
	isp->state = ISPDRV_STATE_WAIT;
	init_completion(&isp->FW_start_completion);
	init_waitqueue_head(&isp->ctrl_recv_wq);
	isp->ctrl_get_val = 0;
	isp->isp_ctrl_status = 0;
	platform_set_drvdata(pdev, isp);
	isp->ipc_session_id = ipc_init(IPC_CORE_ISP, IPC_CORE_ARM7, &pdev->dev);
	if (isp->ipc_session_id < 0) {
		dev_err(&pdev->dev, "ipc_init(IPC_CORE_ARM0) failed\n");
		return -1;
	}

	of_property_read_u32(pdev->dev.of_node, "isp-fw-fbuf-addr", &fbuf_addr);
	of_property_read_u32(pdev->dev.of_node, "isp-fw-fbuf-size", &fbuf_size);
	isp->fbuf_paddr = fbuf_addr;
	isp->fbuf_psize = fbuf_size;

	ret = bst_isp_parse_dt(isp);
	if (ret) {
		dev_err(&pdev->dev, "bst_isp_parse_dt error\n");
		ipc_close(isp->ipc_session_id);
		// isp allocated by devm_kzalloc, needn't to free it,
		// otherwise will cause free twice error
		return -1;
	}
	init_isp_channel_devs(isp);
	init_isp_dev(isp);
	// init firmware code mem, reserved buffer used only this time
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (ret) {
		dev_err(&pdev->dev, "of_reserved_mem_device_init error\n");
		return -1;
	}
	ret = isp_firmware_init(isp);
	if (ret) {
		pr_err("isp_firmware_init error\n");
		of_reserved_mem_device_release(dev);
		return -1;
	}
	/*
	 * Disassociate the reserved memory area from the device
	 * because a device can only have one DMA memory area. This
	 * should be fine since the memory is allocated and initialized
	 * and only ever accessed by the ISP device from now on
	 */
	of_reserved_mem_device_release(dev);

	isp_power_subdevs(isp);
	// must after init_isp_dev
	// because of must after v4l2_async_notifier_register
	update_camera_status(isp);
	// must after firmware init, because comand array needs comand address
	init_isp_videos(isp);
	// init the driver cma buffer
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 1);
	if (ret) {
		pr_err("init cma buffer error\n");
		// ::todo::
		// free allocated buffer
		return -1;
	}
	// register as misc device
	ret = misc_register(&isp_misc);
	if (ret < 0) {
		pr_err("misc register failed\n");
		return -1;
	}
#ifdef ISP_SYSFS
	isp_sysfs_init(isp);
#endif
	isp->kthread_isp =
		kthread_run(isp_kthread_recv_msg, isp, "isp-recv-msg");
	if (IS_ERR(isp->kthread_isp)) {
		// v4l2_err(&dev->v4l2_dev, "kernel_thread() failed\n");
		// return PTR_ERR(dev->kthread_vid_out);
		dev_err(dev, "create kthread recv error\n");
		return -1;
	}

	return 0;

	// :: todo, need add error handler
	// isp_cleanup_videos(isp);
	// v4l2_async_notifier_cleanup(&isp->notifier);
	// ipc_close(isp->ipc_session_id);
	// mutex_destroy(&isp->isp_mutex);

	// return ret;
}

static struct platform_device_id a1000_isp_id_table[] = {
	{ "a1000_isp", 0 },
	{},
};
MODULE_DEVICE_TABLE(platform, a1000_isp_id_table);

static const struct of_device_id a1000_isp_of_table[] = {
	{ .compatible = "bst,a1000-isp" },
	{ .compatible = "bst,a1000b-isp" },
	{},
};
MODULE_DEVICE_TABLE(of, a1000_isp_of_table);

static struct platform_driver a1000_isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.id_table = a1000_isp_id_table,
	.driver = {
		.name = "a1000_isp",
		.of_match_table = a1000_isp_of_table,
	},
};

static int __init a1000_isp_init(void)
{
	platform_driver_register(&a1000_isp_driver);

	return 0;
}

static void __exit a1000_isp_exit(void)
{
	platform_driver_unregister(&a1000_isp_driver);
}

late_initcall(a1000_isp_init);
module_exit(a1000_isp_exit);

MODULE_AUTHOR("BST Corporation");
MODULE_DESCRIPTION("BST A1000 ISP driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(ISP_VIDEO_DRIVER_VERSION);

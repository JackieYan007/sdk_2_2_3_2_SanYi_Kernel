// SPDX-License-Identifier: GPL-2.0

/*
 * ISP video driver for BST
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-map-ops.h>
#include <linux/i2c.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include <linux/coreip/proto_api_common.h>

#include "cam_entity.h"
#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

#define MIN_FRAME_BUFFER_NUM 3
#define DEFAULT_IMAGE_WIDTH  1920
#define DEFAULT_IMAGE_HEIGHT 1080
#define MAX_ENTITY_NAME	     12
#define GAMMA_MAX	     (8)
#define RAW_EMBEDDED_LINES  (4)
#define ISP_EMBEDDED_LINES  (2)
#define TOTAL_EMBEDDED_LINES (0) // RAW + ISP embedded lines

static const char *isp_driver_enp_name[MAX_ENTITY_NAME] = {
	"dv00", "dv01", "dv02", "dv03", "dv04", "dv05",
	"dv06", "dv07", "dv08", "dv09", "dv10", "dv11",
};

/*gamma 1.8 ~ 2.6 from cam1690.ini, default 2.2*/
/* clang-format off */
static const u16 gamma_table[GAMMA_MAX+1][33] = {
	{ /* 0: gamma 1.8 */
		0, 1, 6, 13, 23, 35, 49, 65, 83, 103, 125, 149, 174,
		201, 230, 261, 293, 327, 363, 400, 438, 479, 521,
		564, 609, 656, 704, 753, 804, 857, 911, 966, 1023
	},
	{ /* 1: gamma 1.9 */
		0, 0, 4, 10, 19, 29, 42, 56, 73, 91, 111, 134, 158,
		184, 212, 242, 273, 307, 342, 379, 418, 459, 501,
		546, 592, 640, 689, 740, 794, 848, 905, 963, 1023
	},
	{ /* 2: gamma 2.0 */
		0, 0, 3, 8, 15, 24, 35, 48, 63, 80, 99, 120, 143,
		168, 195, 224, 255, 288, 323, 360, 399, 440, 483,
		528, 575, 624, 675, 728, 783, 840, 899, 960, 1023
	},
	{ /* 3: gamma 2.1 */
		0, 0, 2, 6, 12, 20, 29, 41, 55, 70, 88, 108, 130,
		153, 179, 208, 238, 270, 305, 342, 381, 422, 465,
		511, 559, 609, 661, 716, 773, 832, 893, 957, 1023
	},
	{ /* 4: gamma 2.2 */
		0, 0, 1, 5, 10, 16, 25, 35, 48, 62, 78, 97, 117,
		140, 165, 192, 222, 254, 288, 324, 363, 404, 448,
		494, 543, 594, 648, 704, 762, 824, 887, 954, 1023
	},
	{ /* 5: gamma 2.3 */
		0, 0, 1, 3, 8, 13, 21, 30, 41, 54, 70, 87, 106, 128,
		152, 178, 207, 238, 272, 308, 346, 388, 432, 478,
		527, 579, 634, 692, 752, 816, 882, 951, 1023
	},
	{ /* 6: gamma 2.4 */
		0, 0, 0, 2, 6, 11, 17, 26, 36, 48, 62, 78, 96, 117,
		140, 165, 193, 223, 256, 292, 330, 372, 416, 463,
		512, 565, 621, 680, 742, 808, 876, 948, 1023
	},
	{ /* 7: gamma 2.5 */
		0, 0, 0, 2, 5, 9, 15, 22, 31, 42, 55, 70, 87, 107,
		129, 153, 180, 210, 242, 277, 315, 356, 400, 447,
		498, 551, 608, 669, 732, 800, 870, 945, 1023
	},
	{ /* 8: gamma 2.6 */
		0, 0, 0, 1, 4, 7, 12, 19, 27, 37, 49, 63, 79, 97,
		118, 142, 168, 197, 228, 263, 301, 342, 386, 433,
		484, 538, 596, 657, 723, 792, 865, 942, 1023
	}
};
/* clang-format on */

/* -----------------------------------------------------------------------------
 * Helper functions
 */
static int handle_view_video_sync_msg(struct a1000_isp_video *video,
				 struct internal_msg *cache_msg);

static int handle_raw_video_sync_msg(struct a1000_isp_video *video,
				 struct internal_msg *cache_msg);

static int get_view_bytesperline(struct isp_view *pview);
static int get_view_sizeimage(struct isp_view *pview);
static int notify_fw_stop_stream(struct a1000_isp_video *video);
static void dump_debug_info(struct a1000_isp_video *video);

static inline int is_reserved_buf(struct a1000_isp_video *video, uint32_t fbuf)
{
	uint64_t fbuf_paddr_start = video->isp->fbuf_paddr;
	uint64_t fbuf_paddr_end = fbuf_paddr_start + video->isp->fbuf_psize;

	return ((fbuf >= fbuf_paddr_start) && (fbuf < fbuf_paddr_end));
}

static struct media_command *fill_media_cmd(struct a1000_isp_video *video,
					    struct cmd_element *element)
{
	struct media_command *cmd;
	int isp_chn_index;

	isp_chn_index = video->video_index % MAX_ISP_CHANNEL;
	cmd = (struct media_command *)element->media_cmd_vaddr;
	memset(cmd, 0, sizeof(struct media_command));
	// actually, isp fw don't check the message header
	cmd->cmd_hdr.magic.magic[0] = 'C';
	cmd->cmd_hdr.magic.magic[1] = 'I';
	memcpy(cmd->cmd_hdr.src, isp_driver_enp_name[isp_chn_index], 4);

	return cmd;
}

static int isp_video_check_format(struct a1000_isp_video *video,
				  struct a1000_isp_video_fh *vfh)
{
	// to do
	// pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 * this function will be called by videobuf2 when user reqbuffer,
 * call it to enture the request buffer num and plane is supported by driver
 */

static int isp_video_queue_setup(struct vb2_queue *queue, unsigned int *count,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct a1000_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct a1000_isp_video *video = vfh->video;
	unsigned int buf_num = MAX_FRAME_BUF_NUM;
	int i;

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; i++) {
		int index = video->current_views_list[i];

		sizes[i] = vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage;
		if (sizes[i] == 0) {
			pr_err("ERROR: video index = %d, plane %d, sizeimage == 0\n", video->video_index, i);
			return -EINVAL;
		}
	}

	*num_planes = vfh->format.fmt.pix_mp.num_planes;
	*count = min(*count, buf_num);

	return 0;
}

/*-----------------------------------------------------------------------------
 *this will be called by videobuf2 after vb2 prepare buffer successed.
 *vb2 will do prepare before the buffer could be used.
 *this function just need to check vb2 buffer with user setting
 */
static int isp_video_buffer_prepare(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct a1000_isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct a1000_isp_video *video = vfh->video;
	dma_addr_t addr;
	int i;

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; i++) {
		int index = video->current_views_list[i];

		if (vb2_plane_size(buf, i) <
		    vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage)
			return -EINVAL;
		vb2_set_plane_payload(
			buf, i,
			vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage);
		// to do, only need do it at first time
		addr = vb2_dma_contig_plane_dma_addr(buf, i);
		buffer->dma[video->current_views_list[i]] =
			(uint32_t)(addr & LOW_32_BIT_MASK);

		// can't run __dma_flush_area on kernel thread,
		// it can only be run in the user process context
		// because the mapped user address is not belong to kernel
		// thread so the virtual address can't be find in the kernel
		// thread and causes the "paging request" error
		dev_dbg(&video->video.dev, "%s: plane: %d, mmap_uaddr: 0x%p\n",
			__func__, i, buffer->mmap_uaddr[i]);
		if (buffer->mmap_uaddr[i]) {
			// one thread is streamoff , munmap user virtual address
			// and another thread to qbuf, will cause dma flush
			// to wrong address, so skip prepare after user stream
			// off
			int video_status;

			video_status = atomic_read(&video->status);
			if (video_status >= VIDEO_STATUS_STREAMOFFING) {
				pr_info("isp video prepare, status = %d\n",
					video_status);
				return 0;
			}
			// do_gettimeofday(&start);
			__dma_flush_area(buffer->mmap_uaddr[i],
					 buffer->plane_size[i]);
			// do_gettimeofday(&end);

			// pr_err("flush = %ld\n",
			//	(end.tv_sec - start.tv_sec) * 1000000 +
			//(end.tv_usec - start.tv_usec));
		}
	}

	return 0;
}

/*
 * isp_video_buffer_queue - Add buffer to streaming queue
 * @buf: Video buffer
 * this will be called by vb2 qbuf,vb2 add this buffer to it's free list
 * here we add buffer to driver used list ,when one buffer is done,we put
 * one buffer to fw from driver list
 */
static void isp_video_buffer_queue(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct a1000_isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct a1000_isp_video *video = vfh->video;
	unsigned int empty;

	// printk(KERN_INFO "ENTER: %s\n", __func__);
	if (unlikely(video->error)) {
		vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	mutex_lock(&video->free_queue_lock);
	empty = list_empty(&video->free_buf_queue);
	buffer->cycle_count = 0;
	list_add_tail(&buffer->node, &video->free_buf_queue);
	mutex_unlock(&video->free_queue_lock);
}

/*
 * a1000_isp_video_return_buffers - Return all queued buffers to videobuf2
 * @video: ISP video object
 * @state: new state for the returned buffers
 *
 * Return all buffers queued on the video node to videobuf2 in the given state.
 * The buffer state should be VB2_BUF_STATE_QUEUED if called due to an error
 * when starting the stream, or VB2_BUF_STATE_ERROR otherwise.
 *
 * The function must be called with the video irqlock held.
 */
static void a1000_isp_video_return_buffers(struct a1000_isp_video *video,
					   enum vb2_buffer_state state)
{
	int free_count = 0;
	int wait_count = 0;

	mutex_lock(&video->free_queue_lock);
	while (!list_empty(&video->free_buf_queue)) {
		struct isp_buffer *buf;

		free_count++;
		buf = list_first_entry(&video->free_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_info(&video->video.dev,
			 "free list count %d ,buffer address  is %x\n",
			 free_count, buf->dma[0]);
	}
	mutex_unlock(&video->free_queue_lock);

	mutex_lock(&video->wait_queue_lock);
	while (!list_empty(&video->wait_buf_queue)) {
		struct isp_buffer *buf;

		wait_count++;
		buf = list_first_entry(&video->wait_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_info(&video->video.dev,
			 "wait list count %d ,buffer address  is %x\n",
			 wait_count, buf->dma[0]);
	}
	mutex_unlock(&video->wait_queue_lock);

	dev_info(&video->video.dev, "kernel buffer number is %d",
		 free_count + wait_count);
}


static int send_raw_buf_to_fw(struct a1000_isp_video *video, uint32_t *buf_paddr,
			  int cmd_index, int reserve)
{
	struct cmd_element *element;
	struct media_command *cmd;
	isp_raw_buf_t *new_buffer;
	ipc_msg msg;
	int ret;
	int video_status;

	video->tx_buf_count++;
	/*************************************************
	 *step1 : fill media cmd message
	 *************************************************/
	video_status = atomic_read(&(video->status));
	if ((video_status == VIDEO_STATUS_STREAMOFFING) ||
	    (video_status == VIDEO_STATUS_STREAMOFF_DONE)) {
		pr_err("send_raw_buf_to_fw  video_status = %d\n", video_status);
		return -1;
	}

	element = &(video->media_cmd_array[cmd_index]);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_BUF;
	new_buffer = (isp_raw_buf_t *)(&cmd->user_cmd_data[0]);
	new_buffer->sensorId = S00RawId + video->chn_index;
	new_buffer->rawBuf = buf_paddr[0];
	new_buffer->bufFlag = reserve;

	/*************************************************
	 *step2 : fill ipc message
	 *************************************************/
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	return ret;
}


static int send_view_buf_to_fw(struct a1000_isp_video *video, uint32_t *buf_paddr,
			  int cmd_index)
{
	struct cmd_element *element;
	struct media_command *cmd;
	isp_new_frame_buf_t *new_buffer;
	ipc_msg msg;
	int ret;
	int i;
	int video_status;

	video->tx_buf_count++;
	/*************************************************
	 *step1 : fill media cmd message
	 *************************************************/
	video_status = atomic_read(&(video->status));
	if ((video_status == VIDEO_STATUS_STREAMOFFING) ||
	    (video_status == VIDEO_STATUS_STREAMOFF_DONE))
		return -1;

	element = &(video->media_cmd_array[cmd_index]);
	// pr_info("%s, element = 0x%lx, cmd_index = %d\n",
	//	__func__, element, cmd_index);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_NEW_VIEW_FRAME_BUF;
	new_buffer = (isp_new_frame_buf_t *)(&cmd->user_cmd_data[0]);

	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		new_buffer->viewId[i] = video->current_views_id[i];
		pr_debug("%s: view: %d, %u\n", __func__, i,
			 new_buffer->viewId[i]);
		if (new_buffer->viewId[i])
			new_buffer->viewbuf[i] = buf_paddr[i];
	}

	/*************************************************
	 *step2 : fill ipc message
	 *************************************************/
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);
	pr_debug(
		"SENDBUF: video_index: %02d, 0x%08X, 0x%08X, 0x%08X, cmd_index: %d\n",
		video->video_index, buf_paddr[0], buf_paddr[1], buf_paddr[2],
		cmd_index);

	return ret;
}

static int send_buf_to_fw(struct a1000_isp_video *video, uint32_t *buf_paddr,
			  int cmd_index, int reserve_flag)
{
	if (video->is_raw_video) {
		send_raw_buf_to_fw(video, buf_paddr, cmd_index, reserve_flag);
	} else {
		send_view_buf_to_fw(video, buf_paddr, cmd_index);
	}

	return 0;
}


static int send_ctrl_val_to_fw(struct a1000_isp_video *video, int item,
			       uint32_t val)
{
	struct media_command *media_cmd;
	struct cmd_element *element;
	isp_set_iqinfo_t *iqset;
	ipc_msg msg;

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	media_cmd = fill_media_cmd(video, element);
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_SET_IQINFO;

	iqset = (isp_set_iqinfo_t *)(media_cmd->user_cmd_data);
	iqset->sensorIndex = video->chn_index;
	iqset->iqItem = item;
	iqset->iqVal = val;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;

	return send_cmd_to_fw(video->isp, &msg);
}

static int send_get_ctrl_val_to_fw(struct a1000_isp_video *video, int item)
{
	struct media_command *media_cmd;
	struct cmd_element *element;
	isp_set_iqinfo_t *iqset;
	ipc_msg msg;

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	media_cmd = fill_media_cmd(video, element);
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_GET_IQINFO;

	iqset = (isp_set_iqinfo_t *)(media_cmd->user_cmd_data);
	iqset->sensorIndex = video->chn_index;
	iqset->iqItem = item;
	iqset->iqVal = 0;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;

	return send_cmd_to_fw(video->isp, &msg);
}

static int send_ctrl_msg_to_fw(struct a1000_isp_video *video, int item,
			       struct isp_ctrl *ctrl_info)
{
	struct media_command *media_cmd;
	struct cmd_element *element;
	isp_set_iqinfo_t *iqset;
	ipc_iqinfo_t iqinfo_t;
	iq_rgbgamma_t rgbgamma_t;
	ipc_msg msg;
	u8 *payload_vaddr;
	uint32_t payload_paddr;
	uint32_t payload_size = 0;
	unsigned char *p = (unsigned char *)&iqinfo_t;
	int i;
	/*************************************************
	 *step1 : fill media cmd message
	 *************************************************/
	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	media_cmd = fill_media_cmd(video, element);
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_SET_IQINFO;

	if (video->channel->ctrl_payload_vaddr == NULL) {
		pr_err("ctrl_payload_vaddr is NULL !!!");
		return -1;
	}
	payload_paddr = video->channel->ctrl_payload_paddr;
	payload_vaddr = video->channel->ctrl_payload_vaddr;

	pr_debug("ctrl_vaddr = 0x%p, ctrl_paddr = 0x%x\n",
		 video->channel->ctrl_payload_vaddr,
		 video->channel->ctrl_payload_paddr);

	switch (item) {
	case IQ_AWB:
		for (i = 0; i < 9; i++) {
			pr_debug("AWB data[%d]: %d", i,
				 ctrl_info->manualAWBGain[i / 3][i % 3]);
		}
		// copy ipc_iqinfo_t to payloadaddr
		iqinfo_t.awbInfo.bManualWB = ctrl_info->value;
		for (i = 0; i < 9; i++) {
			iqinfo_t.awbInfo.man_t.manualAWBGain[i / 3][i % 3] =
				ctrl_info->manualAWBGain[i / 3][i % 3];
		}
		payload_size = sizeof(ipc_iqinfo_t);
		memcpy(payload_vaddr, p, sizeof(ipc_iqinfo_t));
		break;
	case IQ_AECAGC:
		// copy ipc_iqinfo_t to payloadaddr
		for (i = 0; i < 3; i++) {
			pr_debug("aecExp data[%d]: %d", i,
				 ctrl_info->aecManualExp[i]);
			pr_debug("aecGain data[%d]: %d", i,
				 ctrl_info->aecManualGain[i]);
		}
		iqinfo_t.aecagcInfo.bManualAECEnable = ctrl_info->value;
		// manual mode : fill custom value
		memcpy(iqinfo_t.aecagcInfo.man_t.aecManualExp,
		       ctrl_info->aecManualExp,
		       sizeof(iqinfo_t.aecagcInfo.man_t.aecManualExp));
		memcpy(iqinfo_t.aecagcInfo.man_t.aecManualGain,
		       ctrl_info->aecManualGain,
		       sizeof(iqinfo_t.aecagcInfo.man_t.aecManualGain));
		payload_size = sizeof(ipc_iqinfo_t);
		memcpy(payload_vaddr, &iqinfo_t, sizeof(ipc_iqinfo_t));
		break;
	case IQ_GAMMA:
		pr_debug("IQ_GAMMA value : %d", ctrl_info->value);
		memcpy(rgbgamma_t.nRGBCurve, gamma_table[ctrl_info->value],
		       sizeof(rgbgamma_t.nRGBCurve));
		payload_size = sizeof(iq_rgbgamma_t);
		memcpy(payload_vaddr, &rgbgamma_t, sizeof(iq_rgbgamma_t));
		break;
	default:
		pr_err("invalid item\n");
	}
	iqset = (isp_set_iqinfo_t *)(media_cmd->user_cmd_data);
	iqset->sensorIndex = video->chn_index;
	iqset->iqItem = item;
	iqset->payloadSize = payload_size;
	iqset->payloadAddr = payload_paddr;
	iqset->iqVal = 0;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;

	return send_cmd_to_fw(video->isp, &msg);
}

static int isp_video_stream_on_subdevs(struct a1000_isp_video *video,
				       bool enable, int port)
{
	/*
	 * stream_ctrl 32bit
	 * 0~3 bit for enable 1 or 0
	 * 4~7 bit for port 0~11
	 */
	int stream_ctrl = enable;
	struct bst_isp_channel *temp_channel;
	struct camera_dev *temp_cam_dev;
	struct deser_hub_dev *temp_deser_parent;
	const struct v4l2_subdev_ops *temp_ops;
	int stream_status;

	stream_ctrl |= port << 4;
	temp_channel = video->channel;
	if (temp_channel == NULL)
		return 1;

	stream_status = atomic_read(&(temp_channel->is_streaming));
	if (stream_status) {
		pr_err("stream status = %d\n", stream_status);
		return 0;
	}

	temp_cam_dev = temp_channel->cam_dev;
	if (temp_cam_dev == NULL)
		return 1;

	temp_deser_parent = temp_cam_dev->deser_parent;
	if (temp_deser_parent == NULL)
		return 1;

	temp_ops = temp_deser_parent->subdev.ops;
	if (temp_ops != NULL) {
		temp_ops->video->s_stream(&temp_deser_parent->subdev,
					  stream_ctrl);
	} else {
		pr_info("video %d sensor stream on failed", port);
	}

	atomic_set(&temp_channel->is_streaming, 1);

	return 0;
}

static int view_video_streamon(struct a1000_isp_video *video)
{
	struct isp_buffer *buf;
	int i;
	ipc_msg msg;
	struct cmd_element *element;
	struct media_command *cmd;
	isp_cam_open_t *streamon_cmd;
	uint32_t *pview_id;
	uint32_t *pcurrent_id;
	int ret;

	for (i = 0; i < MIN_FRAME_BUFFER_NUM; i++) {
		mutex_lock(&video->free_queue_lock);
		if (!list_empty(&video->free_buf_queue)) {
			buf = list_first_entry(&video->free_buf_queue,
					       struct isp_buffer, node);
			list_del(&buf->node);
			mutex_unlock(&video->free_queue_lock);
			buf->cycle_count = 0;
			send_buf_to_fw(video, buf->dma, buf->vb.vb2_buf.index, 0);

			// add sent buf to wait queue
			mutex_lock(&video->wait_queue_lock);
			list_add_tail(&buf->node, &video->wait_buf_queue);
			mutex_unlock(&video->wait_queue_lock);
		} else {
			mutex_unlock(&video->free_queue_lock);
			dev_err(&video->video.dev,
				"ERROR video->free_buf_queue empty, count = %d\n",
				i);
			break;
		}
	}

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_OPEN;
	streamon_cmd = (isp_cam_open_t *)&(cmd->user_cmd_data[0]);
	pview_id = (uint32_t *)streamon_cmd->viewId;
	pcurrent_id = (uint32_t *)video->current_views_id;
	*pview_id = *pcurrent_id;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	isp_inc_streamon_count(video->isp);
	return 0;
}

static int raw_video_streamon(struct a1000_isp_video *video)
{
	struct isp_buffer *buf;
	int i;
	ipc_msg msg;
	struct cmd_element *element;
	struct media_command *cmd;
	isp_raw_open_t *streamon_cmd;
	int ret;

    mutex_lock(&video->free_queue_lock);
    if (!list_empty(&video->free_buf_queue)) {
	buf = list_first_entry(&video->free_buf_queue, struct isp_buffer, node);
	list_del(&buf->node);
	mutex_unlock(&video->free_queue_lock);
	buf->cycle_count = 0;
	send_raw_buf_to_fw(video, buf->dma, buf->vb.vb2_buf.index, 1);
		video->reserve_buf = buf->dma[0];

    } else {
	mutex_unlock(&video->free_queue_lock);
	dev_err(&video->video.dev,
		"ERROR video->free_buf_queue empty, count = %d\n", i);

	return -1;
    }

    for (i = 0; i < MIN_FRAME_BUFFER_NUM; i++) {
		mutex_lock(&video->free_queue_lock);
		if (!list_empty(&video->free_buf_queue)) {
			buf = list_first_entry(&video->free_buf_queue,
					       struct isp_buffer, node);
			list_del(&buf->node);
			mutex_unlock(&video->free_queue_lock);
			buf->cycle_count = 0;
			dev_dbg(&video->video.dev, "buf->dma[0] = 0x%08x\n", buf->dma[0]);
			send_raw_buf_to_fw(video, buf->dma, buf->vb.vb2_buf.index, 0);

			// add sent buf to wait queue
			mutex_lock(&video->wait_queue_lock);
			list_add_tail(&buf->node, &video->wait_buf_queue);
			mutex_unlock(&video->wait_queue_lock);
		} else {
			mutex_unlock(&video->free_queue_lock);
			dev_err(&video->video.dev,
				"ERROR video->free_buf_queue empty, count = %d\n",
				i);
			break;
		}
	}

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_OPEN;
	streamon_cmd = (isp_raw_open_t *)&(cmd->user_cmd_data[0]);
	//pview_id = (uint32_t *)streamon_cmd->viewId;
	//pcurrent_id = (uint32_t *)video->current_views_id;
	//*pview_id = *pcurrent_id;

	streamon_cmd->sensorId = S00RawId + video->chn_index;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	isp_inc_streamon_count(video->isp);

	return 0;
}

static int isp_video_start_streaming(struct vb2_queue *queue,
				     unsigned int count)
{
	struct a1000_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct a1000_isp_video *video = vfh->video;
	struct isp_buffer *buf;
	int ret = 0;

	video->tx_buf_count = 0;
	video->rx_buf_count = 0;
	video->tx_drop_count = 0;
	video->rx_reserved_count = 0;
	video->first_buf_received = false;
	video->last_timestamp = 0;
	video->last_good_sequence = 0;
	video->total_bad_frames = 0;
	memset(&video->fw_ab_info, 0, sizeof(video->fw_ab_info));
	buf = NULL;

	isp_video_stream_on_subdevs(video, true, video->video_index);

	if (video->is_raw_video)
		raw_video_streamon(video);
	else
		view_video_streamon(video);

	return ret;
}

static int notify_fw_stop_stream(struct a1000_isp_video *video)
{
	ipc_msg msg;
	struct cmd_element *element;
	struct media_command *cmd;
	isp_cam_close_t *streamoff_cmd;
	uint32_t *pview_id;
	uint32_t *pcurrent_id;
	int ret;

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_CLOSE;
	streamoff_cmd = (isp_cam_close_t *)&(cmd->user_cmd_data[0]);

	pview_id = (uint32_t *)streamoff_cmd->viewId;
	pcurrent_id = (uint32_t *)video->current_views_id;
	*pview_id = *pcurrent_id;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	return ret;
}

static int raw_video_stop_stream(struct a1000_isp_video *video)
{
	ipc_msg msg;
	struct cmd_element *element;
	struct media_command *cmd;
	isp_raw_close_t *streamoff_cmd;
	int ret;

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	cmd = fill_media_cmd(video, element);
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_CLOSE;
	streamoff_cmd = (isp_raw_close_t *)&(cmd->user_cmd_data[0]);
	streamoff_cmd->sensorId = S00RawId + video->chn_index;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	return ret;
}

/*
 *this func will be called back after vb2 framework finish off
 *step 1: set video status to VIDEO_STATUS_STREAMOFFING
 *step 2:give all buffer to vb2 done list,which have done by isp_video_streamoff
 *step 3:free video->reserve_buf alloced when streamon
 *step 4:maybe should do close disable irq ,which should cooperation with IPC
 *team step 5:sleep before FW ack
 */
static void isp_video_stop_streaming(struct vb2_queue *queue)
{
	struct a1000_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct a1000_isp_video *video = vfh->video;
	int ret;

	pr_err("%s, tx_count = %lld, rx_count = %lld, tx_drop = %lld, rx_reserved = %lld ipc_rx: %lld\n",
	       __func__, video->tx_buf_count, video->rx_buf_count,
	       video->tx_drop_count, video->rx_reserved_count,
	       video->isp->ipc_rx_count);
	pr_err("isp kthread status = %d\n", video->isp->kthread_status);

	if (video->is_raw_video) {
		ret = raw_video_stop_stream(video);
		if (ret < 0) pr_err("raw_video_stop_stream error, ret = %d\n", ret);
	} else {
		// notify fw to stop at first
		ret = notify_fw_stop_stream(video);
		if (ret < 0) pr_err("notify_fw_stop_stream error, ret = %d\n", ret);
	}
	isp_dec_streamon_count(video->isp);
}

static const struct vb2_ops isp_video_queue_ops = {
	.queue_setup = isp_video_queue_setup,
	//.wait_prepare = isp_video_wait_prepare,
	//.wait_finish = isp_video_wait_finish,
	//.buf_init = isp_video_buf_init,
	.buf_prepare = isp_video_buffer_prepare,
	//.buf_finish = isp_video_buffer_finish,
	//.buf_cleanup = isp_video_buffer_cleanup,
	.start_streaming = isp_video_start_streaming,
	.stop_streaming = isp_video_stop_streaming,
	.buf_queue = isp_video_buffer_queue,
};

/*
 * a1000_isp_video_cancel_stream - Cancel stream on a video node
 * @video: ISP video object
 *
 * Cancelling a stream returns all buffers queued on the video node to videobuf2
 * in the erroneous state and makes sure no new buffer can be queued.
 */
void a1000_isp_video_cancel_stream(struct a1000_isp_video *video)
{
	// printk(KERN_INFO "ENTER: %s\n", __func__);

	// notify isp fw to stop stream;
	// wait for isp fw complete
	// todo
	// printk("stop_streaming wait_event_interruptible\n");
	// wait_event_interruptible(video->wq_status, video->status ==
	// VIDEO_STATUS_STREAMOFF_DONE); printk("stop_streaming be wakeuped\n");

	a1000_isp_video_return_buffers(video, VB2_BUF_STATE_ERROR);
	video->error = true;
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int isp_video_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct a1000_isp_video *video = video_drvdata(file);

	// printk(KERN_INFO "ENTER: %s\n", __func__);
	strlcpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;

	return 0;
}

static int isp_video_enum_input(struct file *file, void *fh,
				struct v4l2_input *input)
{
	struct a1000_isp_video *video = video_drvdata(file);
	int i;

	if (video->is_pdns) {
		i = video->pdns_input_view;
		input->index = video->views[i].enable ? ISP_VIEW_MASK(i) : 0;
		return 0;
	}

	input->index = 0;
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->pdns_input_view == i)
			continue;
		input->index |= video->views[i].enable ? ISP_VIEW_MASK(i) : 0;
	}

	return 0;
}

static int isp_video_g_input(struct file *file, void *fh, unsigned int *views)
{
	struct a1000_isp_video *video = video_drvdata(file);
	int i;

	if (video->is_pdns) {
		i = video->pdns_input_view;
		*views = video->current_views_id[i] ? ISP_VIEW_MASK(i) : 0;
		return 0;
	}

	*views = 0;
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->pdns_input_view != i) {
			*views |= video->current_views_id[i] ?
					  ISP_VIEW_MASK(i) :
					  0;
		}
	}

	return 0;
}

static int isp_video_s_input(struct file *file, void *fh, unsigned int comb)
{
	struct a1000_isp_video_fh *vfh;
	struct a1000_isp_video *video;
	int i;

	vfh = to_a1000_isp_video_fh(fh);
	video = video_drvdata(file);
	if (comb > ISP_VIEW0_VIEW1_VIEW2) {
		pr_err("set input: invalid view option\n");
		return -EINVAL;
	}

	video->current_views_num = 0;
	// pdns case
	if (video->is_pdns) {
		// valid check
		i = video->pdns_input_view;
		if (comb != ISP_VIEW_MASK(i)) {
			pr_err("set input: invalid view %d selection for pdns\n",
			       i);
			return -EINVAL;
		}
		if (!video->views[i].enable) {
			pr_err("set input: seletced view %d not available\n",
			       i);
			return -EINVAL;
		}
		// view setting
		video->current_views_id[i] = video->views[i].cmd_view_id;
		video->current_views_list[video->current_views_num] = i;
		video->current_views_num++;
		goto done;
	}

	// normal case
	// valid check
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (comb & ISP_VIEW_MASK(i)) {
			if (i == video->pdns_input_view) {
				pr_err("set input: seletced view %d occupied by pdns\n",
				       i);
				return -EINVAL;
			}
			if (!video->views[i].enable) {
				pr_err("set input: seletced view %d not available\n",
				       i);
				return -EINVAL;
			}
		}
	}
	// view setting
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (comb & ISP_VIEW_MASK(i)) {
			video->current_views_id[i] =
				video->views[i].cmd_view_id;
			video->current_views_list[video->current_views_num] = i;
			video->current_views_num++;
		} else {
			video->current_views_id[i] = 0;
		}
	}

done:
	mutex_lock(&video->mutex);
	vfh->format.fmt.pix_mp.num_planes = video->current_views_num;
	mutex_unlock(&video->mutex);
	return 0;
}

static int isp_video_get_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct a1000_isp_video *video = video_drvdata(file);
	struct isp_view_format *f;
	int i;

	if (format->type != video->type) {
		pr_err("get format: invalid, format typeset %d video typeset %d\n",
		       format->type, video->type);
		return -EINVAL;
	}

	if (video->is_pdns) {
		i = video->pdns_input_view;
		if (video->views[i].enable) {
			f = (struct isp_view_format *)&format->fmt.pix_mp
				    .plane_fmt[i];
			f->pixelformat = video->views[i].format;
			f->width = video->views[i].width;
			f->height = video->views[i].height;
			pr_debug("get format: view %d width %d height %d\n", i,
				 f->width, f->height);
		}
		return 0;
	}

    if (video->is_raw_video) {
	f = (struct isp_view_format *)&format->fmt.pix_mp.plane_fmt[0];
	f->pixelformat = video->views[i].format;
	f->width = video->channel->camera_raw_width;
	f->height = video->views[i].height;
	pr_debug("get format: view %d width %d height %d\n", i, f->width,
		 f->height);
	return 0;
    }

    for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if ((video->pdns_input_view != i) && video->views[i].enable) {
			f = (struct isp_view_format *)&format->fmt.pix_mp
				    .plane_fmt[i];
			f->pixelformat = video->views[i].format;
			f->width = video->views[i].width;
			f->height = video->views[i].height;
			pr_debug("get format: view %d width %d height %d\n", i,
				 f->width, f->height);
		}
	}

	return 0;
}

static int isp_video_set_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct a1000_isp_video_fh *vfh;
	struct a1000_isp_video *video;
	int index;
	uint32_t linebytes;
	uint32_t imagesize;

	pr_debug("set format: view %d\n", format->fmt.pix_mp.num_planes);

	vfh = to_a1000_isp_video_fh(fh);
	video = video_drvdata(file);
	if (format->type != video->type) {
		pr_err("get format: invalid, format typeset %d video typeset %d\n",
		       format->type, video->type);
		return -EINVAL;
	}

	// interpret num_planes as view index
	index = format->fmt.pix_mp.num_planes;
	if (index < 0 || index >= MAX_VIEWS_PER_CAMERA) {
		pr_err("set format: invalid plane index %d\n", index);
		return -EINVAL;
	}

	if (!video->views[index].enable) {
		pr_err("set format: disabled plane index %d\n", index);
		return -EINVAL;
	}

	if ((format->fmt.pix_mp.width != video->views[index].width) ||
	    (format->fmt.pix_mp.height != video->views[index].height) ||
	    (format->fmt.pix_mp.pixelformat != video->views[index].format)) {
		pr_err("set format: width, height or pixelformat not supported\n");
		return -EINVAL;
	}

	linebytes = get_view_bytesperline(&(video->views[index]));
	imagesize = get_view_sizeimage(&(video->views[index]));
	if (!imagesize) {
		pr_err("set format: plane %d, sizeimage zero\n", index);
		return -EINVAL;
	}

	mutex_lock(&video->mutex);
	vfh->format.fmt.pix_mp.plane_fmt[index].bytesperline = linebytes;
	vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage = imagesize;

	mutex_unlock(&video->mutex);

	return 0;
}

static int isp_video_try_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_get_selection(struct file *file, void *fh,
				   struct v4l2_selection *sel)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_set_selection(struct file *file, void *fh,
				   struct v4l2_selection *sel)
{
	pr_info("ENTER: %s\n", __func__);

	return 0;
}

static int isp_video_get_param(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_set_param(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_reqbufs(struct file *file, void *fh,
			     struct v4l2_requestbuffers *rb)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	// printk(KERN_INFO "ENTER: %s\n", __func__);
	mutex_lock(&video->queue_lock);
	ret = vb2_reqbufs(&vfh->queue, rb);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_querybuf(struct file *file, void *fh,
			      struct v4l2_buffer *b)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	mutex_lock(&video->queue_lock);
	ret = vb2_querybuf(&vfh->queue, b);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	mutex_lock(&video->queue_lock);
	ret = vb2_qbuf(&vfh->queue, &video->isp->media_dev, b);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	mutex_lock(&video->queue_lock);
	if (video->error) {
		b->flags = V4L2_BUF_FLAG_ERROR;
		b->sequence = video->last_good_sequence;
		v4l2_buffer_set_timestamp(b, video->last_timestamp);
		b->reserved = video->total_bad_frames;
		b->reserved2 = video->rx_buf_count;
		ret = -EFAULT;
	} else {
		ret = vb2_dqbuf(&vfh->queue, b, file->f_flags & O_NONBLOCK);
	}
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_streamon(struct file *file, void *fh,
			      enum v4l2_buf_type type)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	if (type != video->type)
		return -EINVAL;

	atomic_set(&(video->status), VIDEO_STATUS_STREAMONING);
	/* Verify that the currently configured format matches the output of
	 * the connected subdev.
	 */
	ret = isp_video_check_format(video, vfh);
	if (ret < 0)
		goto err_check_format;

	video->queue = &vfh->queue;
	INIT_LIST_HEAD(&video->free_buf_queue);

	mutex_lock(&video->queue_lock);
	ret = vb2_streamon(&vfh->queue, type);
	mutex_unlock(&video->queue_lock);
	if (ret < 0)
		goto err_check_format;

	atomic_set(&(video->status), VIDEO_STATUS_STREAMON_DONE);
	return 0;

err_check_format:
	INIT_LIST_HEAD(&video->free_buf_queue);
	video->queue = NULL;

	return ret;
}

static int isp_video_streamoff(struct file *file, void *fh,
			       enum v4l2_buf_type type)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	unsigned int streaming;

	if (type != video->type)
		return -EINVAL;

	atomic_set(&(video->status), VIDEO_STATUS_STREAMOFFING);
	/* Make sure we're not streaming yet. */
	mutex_lock(&video->queue_lock);
	streaming = vb2_is_streaming(&vfh->queue);
	mutex_unlock(&video->queue_lock);

	if (!streaming)
		goto done;

	a1000_isp_video_cancel_stream(video);

	mutex_lock(&video->queue_lock);
	vb2_streamoff(&vfh->queue, type);
	mutex_unlock(&video->queue_lock);
	video->queue = NULL;
	video->error = false;

done:
	atomic_set(&(video->status), VIDEO_STATUS_STREAMOFF_DONE);
	dev_info(video->isp->dev, "EXIT: %s\n", __func__);
	return 0;
}

static int isp_video_expbuf(struct file *file, void *fh,
			    struct v4l2_exportbuffer *p)
{
	struct a1000_isp_video_fh *vfh = to_a1000_isp_video_fh(fh);
	struct a1000_isp_video *video = video_drvdata(file);
	int ret;

	mutex_lock(&video->queue_lock);
	ret = vb2_expbuf(&vfh->queue, p);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_read_camera_register(struct file *file, void *fh,
					  struct isp_regcfg *cfg)
{
	struct a1000_isp_video *video = video_drvdata(file);
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.g_register)
		return -EINVAL;

	return cam->ops.g_register(cam, cfg->regaddr, &cfg->regval);
}

static int isp_video_write_camera_register(struct file *file, void *fh,
					   struct isp_regcfg *cfg)
{
	struct a1000_isp_video *video = video_drvdata(file);
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_register)
		return -EINVAL;

	return cam->ops.s_register(cam, cfg->regaddr, cfg->regval);
}

static int isp_video_debug_print(struct file *file, void *fh)
{
	pr_err("%s\n", __func__);

	return 0;
}

static int isp_video_debug_enter(struct file *file, void *fh)
{
	struct a1000_isp_video *video = video_drvdata(file);

	pr_err("%s\n", __func__);

	// TODO: judging isp busy state
	if (video->isp->state != ISPDRV_STATE_DEBUG)
		ispdrv_enter_debug_state(video->isp);

	return 0;
}

static int isp_video_debug_leave(struct file *file, void *fh)
{
	struct a1000_isp_video *video = video_drvdata(file);

	pr_err("%s\n", __func__);

	// TODO: free memory if allocated
	if (video->isp->state == ISPDRV_STATE_DEBUG)
		ispdrv_leave_debug_state(video->isp);

	return 0;
}

static size_t memsz_debug;
static dma_addr_t paddr_debug;
static uint8_t *vaddr_debug;

static int isp_video_debug_alloc_mem(struct file *file, void *fh, size_t size)
{
	struct a1000_isp_video *video = video_drvdata(file);

	pr_err("%s, size %lu\n", __func__, size);

	if (vaddr_debug)
		return -EINVAL;

	vaddr_debug = dma_alloc_coherent(video->isp->dev, size, &paddr_debug,
					 GFP_KERNEL);
	if (!vaddr_debug) {
		pr_err("%s, alloc failed\n", __func__);
		return -ENOMEM;
	}
	memsz_debug = size;
	pr_err("%s succeeded, dma_addr 0x%llx\n", __func__, paddr_debug);

	return 0;
}

static int isp_video_debug_free_mem(struct file *file, void *fh)
{
	struct a1000_isp_video *video = video_drvdata(file);

	pr_err("%s\n", __func__);

	if (!vaddr_debug)
		return -EINVAL;

	dma_free_coherent(video->isp->dev, memsz_debug, vaddr_debug,
			  paddr_debug);
	memsz_debug = 0;
	vaddr_debug = NULL;

	return 0;
}

static int isp_video_debug_query_mem(struct file *file, void *fh,
				     uint32_t *addr)
{
	if (!vaddr_debug)
		return -EINVAL;

	*addr = paddr_debug;

	return 0;
}

static struct media_command *cmd_debug;
DECLARE_COMPLETION(isp_debug);

int isp_video_debug_isp_wait(struct media_command *cmd)
{
	// TODO: add timeout
	pr_err("%s, down\n", __func__);
	wait_for_completion(&isp_debug);
	memcpy(cmd, cmd_debug, sizeof(struct media_command));

	return 0;
}

int isp_video_debug_isp_done(struct media_command *cmd)
{
	pr_err("%s, up\n", __func__);
	cmd_debug = cmd;
	complete(&isp_debug);

	return 0;
}

static int isp_video_debug_trans_ipcmsg(struct file *file, void *fh,
					struct isp_ipcmsg *isp_msg)
{
	int ret;
	ipc_msg msg;
	struct cmd_element *element;
	struct media_command *cmd_src;
	struct media_command *cmd_dst;

	struct a1000_isp_video *video = video_drvdata(file);

	pr_err("%s\n", __func__);

	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	cmd_src = &isp_msg->cmd;
	cmd_dst = (struct media_command *)element->media_cmd_vaddr;
	memcpy(cmd_dst, cmd_src, sizeof(struct media_command));
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;

	pr_err("%s minor cmd 0x%x\n", __func__,
	       isp_msg->cmd.cmd_hdr.hdr_info.cmd_type_minor);

	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0) {
		pr_err("%s failed, ret %d\n", __func__, ret);
		return -EIO;
	}

	memset(cmd_src, 0, sizeof(struct media_command));
	if (isp_msg->reply) {
		pr_err("%s wait reply\n", __func__);
		isp_video_debug_isp_wait(cmd_src);
	}

	return 0;
}

static void dump_resize_info(struct resolution_resize *resize_t)
{
	pr_info("%s() Line %d,Resize Dump\n", __func__, __LINE__);

	pr_info("topCropBefore : %hd\n", resize_t->crop_size.topCropBefore);
	pr_info("botCropBefore : %hd\n", resize_t->crop_size.botCropBefore);
	pr_info("lefCropBefore : %hd\n", resize_t->crop_size.lefCropBefore);
	pr_info("rigCropBefore : %hd\n", resize_t->crop_size.rigCropBefore);

	pr_info("topCropAfter : %hd\n", resize_t->crop_size.topCropAfter);
	pr_info("botCropAfter : %hd\n", resize_t->crop_size.botCropAfter);
	pr_info("lefCropAfter : %hd\n", resize_t->crop_size.lefCropAfter);
	pr_info("rigCropAfter : %hd\n", resize_t->crop_size.rigCropAfter);

	pr_info("scale width: %hd\n", resize_t->scale_size.width);
	pr_info("scale height: %hd\n", resize_t->scale_size.height);
}

static void update_resize_resolution(struct a1000_isp_video *video,
				     struct resolution_resize *resize_t)
{
	struct crop_size crop_size = resize_t->crop_size;
	struct scale_size scale_size = resize_t->scale_size;
	int index = resize_t->view_id;

	pr_info("jason_debug %s() line%d\n", __func__, __LINE__);
	/*Crop before*/
	if (crop_size.lefCropBefore > 0 || crop_size.rigCropBefore > 0) {
		video->views[index].width = crop_size.rigCropBefore - crop_size.lefCropBefore;
	}
	if (crop_size.botCropBefore > 0 || crop_size.topCropBefore > 0) {
		video->views[index].height = crop_size.botCropBefore - crop_size.topCropBefore;
	}
	/*scaler*/
	if (scale_size.width > 0 && scale_size.height > 0) {
		video->views[index].width = scale_size.width;
		video->views[index].height = scale_size.height;
	}
	/*Crop After*/
	if (crop_size.lefCropAfter > 0 || crop_size.rigCropAfter > 0) {
		video->views[index].width = crop_size.rigCropAfter - crop_size.lefCropAfter;
	}
	if (crop_size.botCropAfter > 0 || crop_size.topCropAfter > 0) {
		video->views[index].height = crop_size.botCropAfter - crop_size.topCropAfter;
	}
	pr_info("resize view[%d] to w:%d, h:%d !\n", index,
	video->views[index].width, video->views[index].height);
}

static int isp_resize_resolution(struct a1000_isp_video *video,
				 struct resolution_resize *resize_t)
{
	struct cmd_element *element;
	struct media_command *media_cmd;
	isp_set_viewinfo_t *view_info;
	view_cfg_t view_cfg;
	ipc_msg msg;
	uint8_t *payload_vaddr;
	uint32_t payload_paddr;
	uint32_t payload_size = 0;
	uint32_t linebytes;
	uint32_t imagesize;
	struct a1000_isp_device *isp;
	// int timeout;
	int view_id;
	int cmd_view_id;
	/*checked in ISP SDK*/

	/*************************************************
	 *step1 : fill media cmd message
	 *************************************************/
	view_id = resize_t->view_id;
	element = &(video->media_cmd_array[CTRL_CMD_INDEX]);
	media_cmd = fill_media_cmd(video, element);
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_SET_VIEWINFO;

	dump_resize_info(resize_t);

	if (video->channel->ctrl_payload_vaddr == NULL) {
		pr_err("ctrl_payload_vaddr is NULL !!!");
		return -1;
	}
	payload_paddr = video->channel->ctrl_payload_paddr;
	payload_vaddr = video->channel->ctrl_payload_vaddr;

	pr_info("ctrl_vaddr = 0x%p, ctrl_paddr = 0x%x\n",
		video->channel->ctrl_payload_vaddr,
		video->channel->ctrl_payload_paddr);

	/*************************************************
	 *step2 : fill payload mesg
	 *************************************************/
	if (view_id > 2 || view_id < 0) {
		pr_err("resize_t->view_id is invalid !!!");
		return -1;
	}
	view_cfg.viewFmt = video->views[view_id].isp_view_fmt;
	view_cfg.scalerRemap = NO_SCALER_REMAP;
	view_cfg.topCropBefore = resize_t->crop_size.topCropBefore;
	view_cfg.botCropBefore = resize_t->crop_size.botCropBefore;
	view_cfg.lefCropBefore = resize_t->crop_size.lefCropBefore;
	view_cfg.rigCropBefore = resize_t->crop_size.rigCropBefore;
	view_cfg.topCropAfter = resize_t->crop_size.topCropAfter;
	view_cfg.botCropAfter = resize_t->crop_size.botCropAfter;
	view_cfg.lefCropAfter = resize_t->crop_size.lefCropAfter;
	view_cfg.rigCropAfter = resize_t->crop_size.rigCropAfter;

	view_cfg.width = resize_t->scale_size.width;
	view_cfg.height = resize_t->scale_size.height;

	memset(payload_vaddr, 0, ISP_CTRL_PAYLOAD_SIZE);
	memcpy(payload_vaddr, &view_cfg, sizeof(struct _view_cfg_t));

	/*************************************************
	 *step3 : fill ipc mesg
	 *************************************************/
	view_info = (isp_set_viewinfo_t *)(media_cmd->user_cmd_data);
	if (view_id == 0)
		cmd_view_id = S00View0;
	else if (view_id == 1)
		cmd_view_id = S00View1;
	else if (view_id == 2)
		cmd_view_id = S00View2;

	view_info->viewid =
		((video->chn_index * VIEW_ID_NUM_PER_CAMERA) + cmd_view_id);
	view_info->payloadSize = payload_size;
	view_info->payloadAddr = payload_paddr;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = element->media_cmd_paddr;
	pr_err("send cmd_viewid : %d\n", view_info->viewid);

	mutex_lock(&(video->mutex));
	if (send_cmd_to_fw(video->isp, &msg) < 0) {
		pr_err(" send_cmd_to_fw failed !!!");
		mutex_unlock(&(video->mutex));
		return -1;
	}
	/*************************************************
	 *step4 : wait for FW Done: It'll spend double frame's time
	 *************************************************/

	isp = video->isp;
	isp->isp_ctrl_status = ISP_SET_VIEW_STATUS_WAIT;
	msleep(100);
	isp->isp_ctrl_status = ISP_SET_VIEW_STATUS_NONE;
	/*************************************************
	 *step5 : Update resolution
	 *************************************************/

	update_resize_resolution(video, resize_t);

	linebytes = get_view_bytesperline(&(video->views[view_id]));
	imagesize = get_view_sizeimage(&(video->views[view_id]));
	if (!imagesize) {
		pr_err("set format: plane %d, sizeimage zero\n", view_id);
		mutex_unlock(&(video->mutex));
		return -EINVAL;
	}

	if (video->v4l2_handle != NULL) {
		video->v4l2_handle->format.fmt.pix_mp.plane_fmt[view_id]
			.bytesperline = linebytes;
		video->v4l2_handle->format.fmt.pix_mp.plane_fmt[view_id]
			.sizeimage = imagesize;
	}

	mutex_unlock(&(video->mutex));
	return 0;
}

static long isp_video_default(struct file *file, void *fh, bool valid_prio,
			      unsigned int cmd, void *arg)
{
	struct a1000_isp_video *video;

	switch (cmd) {
	/* isp normal mode ioctl impls */
	case ISPIOC_G_CAMREG:
		return isp_video_read_camera_register(
			file, fh, ((struct isp_regcfg *)arg));

	case ISPIOC_S_CAMREG:
		return isp_video_write_camera_register(
			file, fh, ((struct isp_regcfg *)arg));

	/* isp debug mode ioctl impls */
	case ISPIOC_PRINT_DBGINFO:
		return isp_video_debug_print(file, fh);

	case ISPIOC_ENTER_DBGMODE:
		return isp_video_debug_enter(file, fh);

	case ISPIOC_LEAVE_DBGMODE:
		return isp_video_debug_leave(file, fh);

	case ISPIOC_ALLOC_DBGMEM:
		return isp_video_debug_alloc_mem(file, fh, *((size_t *)arg));

	case ISPIOC_FREE_DBGMEM:
		return isp_video_debug_free_mem(file, fh);

	case ISPIOC_QUERY_DBGMEM:
		return isp_video_debug_query_mem(file, fh, (uint32_t *)arg);

	case ISPIOC_TRANS_IPCMSG:
		return isp_video_debug_trans_ipcmsg(file, fh,
						    (struct isp_ipcmsg *)arg);

	case ISPIOC_G_ABNORMAL_INFO:
		{
			struct a1000_isp_video *video = video_drvdata(file);
			struct abnormal_info *ab_info =
				(struct abnormal_info *)arg;

			ab_info->abnormal_id = video->fw_ab_info.abnormalId;
			ab_info->abnormal_type = video->fw_ab_info.abnormalType;
			ab_info->last_good_sequence = video->last_good_sequence;
			ab_info->total_bad_frames = video->total_bad_frames;
			ab_info->total_frames = video->rx_buf_count;
			video->error = false;
			return 0;
		}

	case ISPIOC_RESIZE_RESOLUTION:
		video = video_drvdata(file);
		return isp_resize_resolution(video,
					     (struct resolution_resize *)arg);
	default:
		return -ENOTTY;
	}
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap = isp_video_querycap,
	.vidioc_try_fmt_vid_cap_mplane = isp_video_try_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane = isp_video_set_format_mplane,
	.vidioc_g_fmt_vid_cap_mplane = isp_video_get_format_mplane,
	.vidioc_g_selection = isp_video_get_selection,
	.vidioc_s_selection = isp_video_set_selection,
	.vidioc_g_parm = isp_video_get_param,
	.vidioc_s_parm = isp_video_set_param,
	.vidioc_reqbufs = isp_video_reqbufs,
	.vidioc_querybuf = isp_video_querybuf,
	.vidioc_qbuf = isp_video_qbuf,
	.vidioc_dqbuf = isp_video_dqbuf,
	.vidioc_streamon = isp_video_streamon,
	.vidioc_streamoff = isp_video_streamoff,
	.vidioc_enum_input = isp_video_enum_input,
	.vidioc_g_input = isp_video_g_input,
	.vidioc_s_input = isp_video_s_input,
	.vidioc_expbuf = isp_video_expbuf,
	.vidioc_default = isp_video_default,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int get_view_bytesperline(struct isp_view *pview)
{
	int format = pview->format;

	switch (format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return (pview->width);
	case V4L2_PIX_FMT_RGB24:
		return (pview->width * 3);
	case V4L2_PIX_FMT_GREY:
		return (pview->width);
	default:
		pr_err("%s ERROR: unknown view format = %d\n", __func__,
		       format);
		return 0;
	}
}

static int get_view_sizeimage(struct isp_view *pview)
{
	int format = pview->format;

	switch (format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return (pview->width * (pview->height + TOTAL_EMBEDDED_LINES) * 3 / 2);
	case V4L2_PIX_FMT_RGB24:
		return (pview->width * (pview->height + TOTAL_EMBEDDED_LINES) * 3);
	case V4L2_PIX_FMT_GREY:
		return (pview->width * (pview->height + TOTAL_EMBEDDED_LINES));
	default:
		pr_err("%s ERROR: unknown view format = %d\n", __func__,
		       format);
		return 0;
	}
}

static int isp_video_open(struct file *file)
{
	struct a1000_isp_video *video = video_drvdata(file);
	struct a1000_isp_video_fh *handle;
	struct vb2_queue *queue;
	int ret = 0;
	char work_queue_name[64];
	int default_view = -1;
	int video_status;
	int i;

	if (!video->enabled) {
		dev_err(video->isp->dev, "%s video %d disabled\n", __func__,
			 video->video_index);
		return -ENOENT;
	}
	mutex_lock(&(video->mutex));
	video_status = atomic_read(&(video->status));
	if ((video_status >= VIDEO_STATUS_OPENING) &&
	    (video_status < VIDEO_STATUS_CLOSE_DONE)) {
		// video opened last time, but not closed properly
		mutex_unlock(&(video->mutex));
		pr_err("video busy\n");
		return -EBUSY;
	}
	atomic_set(&(video->status), VIDEO_STATUS_OPENING);
	mutex_unlock(&(video->mutex));
	/* maybe use v4l2_fh_open is better,
	 * here also ok.
	 */
	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL) {
		return -ENOMEM;
	}

	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);

	queue = &handle->queue;
	queue->type = video->type;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->drv_priv = handle;
	queue->ops = &isp_video_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct isp_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->dev = video->isp->dev;
	/* min_buffers_needed will be used at reqbuf and streamon of vb2
	 * framework
	 */
	queue->min_buffers_needed = MIN_FRAME_BUFFER_NUM;

	// printk(KERN_INFO "%s: queue success\n", __func__);
	ret = vb2_queue_init(&handle->queue);
	if (ret < 0) {
		dev_info(video->isp->dev,
			 "%s: vb2_queue_init failed, ret = %d\n", __func__,
			 ret);
		goto done;
	}
	if (atomic_read(&(video->isp->FW_config_done)) == 0) {
		int timeout = 0;

		if (atomic_read(&(video->isp->FW_load_started)) == 0) {
			mutex_lock(&(video->isp->isp_mutex));
			if (atomic_read(&(video->isp->FW_load_started)) == 0) {
				if (video->isp->use_dsp) {
					ret = bst_load_dsp_fw(
						"bst_pwl_dsp_rt.rbf",
						video->isp);
					if (ret != 0) {
						dev_err(video->isp->dev,
							"Failed to load dsp fw\n");
					}
					dev_info(video->isp->dev,
						 "Load dsp fw done\n");
					bst_start_dsp_fw(video->isp);
				}
				ret = bst_load_isp_fw(ISP_FW_IMAGE_NAME,
						      ISP_SLAB_NAME,
						      video->isp);
				if (ret != 0) {
					pr_err("bst_load_isp_fw failed\n");
					atomic_set(
						&(video->isp->FW_load_started),
						-1);
					mutex_unlock(&(video->isp->isp_mutex));
					return ret;
				}
				bst_start_isp_fw(video->isp);
				atomic_set(&(video->isp->FW_load_started), 1);
			}
			mutex_unlock(&(video->isp->isp_mutex));
		}
		timeout = wait_for_completion_io_timeout(
			&(video->isp->FW_start_completion),
			msecs_to_jiffies(3000));
		if (timeout == 0) {
			dev_info(video->isp->dev, "open timeout");
			atomic_set(&(video->status), VIDEO_STATUS_INVALID);
			atomic_set(&(video->isp->FW_load_started), 0);
			v4l2_fh_del(&handle->vfh);
			v4l2_fh_exit(&handle->vfh);
			kfree(handle);
			return -EPIPE;
		}
	}

	// already boot done
	atomic_set(&(video->status), VIDEO_STATUS_FW_DONE);
	// pr_info("video %d: FW_boot_completion success\n",
	// video->video_index);
	memset(&handle->format, 0, sizeof(handle->format));
	handle->format.type = video->type;
	handle->timeperframe.denominator = 1;

	// default to first enabled view
	for (i = 0; i < MAX_VIEWS_PER_CAMERA; i++) {
		if (video->views[i].enable) {
			default_view = i;
			break;
		}
	}

	if (default_view < 0) {
		pr_err("%s failed: no view enabled\n", __func__);
		return -ENOENT;
	}

	handle->format.fmt.pix_mp.num_planes = 1;
	handle->format.fmt.pix_mp.width = video->views[default_view].width;
	handle->format.fmt.pix_mp.height = video->views[default_view].height;
	handle->format.fmt.pix_mp.pixelformat =
		video->views[default_view].format;
	handle->format.fmt.pix_mp.field = V4L2_FIELD_ANY;
	handle->format.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;

	if (video->is_raw_video) {
		// hard code
		//handle->format.fmt.pix_mp.plane_fmt[0].bytesperline = 1920 * 1286;
		//handle->format.fmt.pix_mp.plane_fmt[0].sizeimage = 1920 * 1286 * 2;

		handle->format.fmt.pix_mp.plane_fmt[0].bytesperline =
			video->channel->camera_raw_width * 2;
		handle->format.fmt.pix_mp.plane_fmt[0].sizeimage =
			video->channel->camera_raw_width * video->channel->camera_raw_height * 2;
	} else {
		handle->format.fmt.pix_mp.plane_fmt[0].bytesperline =
			get_view_bytesperline(&(video->views[default_view]));
		handle->format.fmt.pix_mp.plane_fmt[0].sizeimage =
			get_view_sizeimage(&(video->views[default_view]));
	}
	video->current_views_num = 1;
	video->current_views_list[0] = 0;
	video->current_views_list[1] = 0;
	video->current_views_list[2] = 0;
	video->current_views_id[0] = video->views[default_view].cmd_view_id;
	video->current_views_id[1] = 0;
	video->current_views_id[2] = 0;
	video->current_views_id[3] = 0;
	video->drv_sequence = 0;

	handle->video = video;
	file->private_data = &handle->vfh;
	memset(work_queue_name, 0, 64);
	snprintf(work_queue_name, 64, "%s%d", "msg_work_queue",
		 video->video_index);

	video->msg_work_queue = create_singlethread_workqueue(work_queue_name);
	if (video->msg_work_queue == NULL) {
		dev_info(video->isp->dev, "could not create msg_work_queue\n");
		ret = -ENOMEM;
		return ret;
	}

	// to do
	// printk("ISP OPEN wait_event_interruptible\n");
	// wait_event_interruptible(video->wq_status, video->status =
	// VIDEO_STATUS_OPEN_DONE); printk("ISP OPEN be wakeuped\n");
	// printk(KERN_INFO "%s: exit success\n", __func__);
	atomic_set(&(video->status), VIDEO_STATUS_OPEN_DONE);
done:
	if (ret < 0) {
		v4l2_fh_del(&handle->vfh);
		v4l2_fh_exit(&handle->vfh);
		kfree(handle);
	} else {
		video->v4l2_handle = handle;
	}

	return ret;
}

static int isp_video_release(struct file *file)
{
	struct a1000_isp_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct a1000_isp_video_fh *handle = to_a1000_isp_video_fh(vfh);
	int video_status;

	// printk(KERN_INFO "ENTER: %s\n", __func__);
	video_status = atomic_read(&(video->status));
	if (video_status == VIDEO_STATUS_STREAMON_DONE) {
		/* Disable streaming and free the buffers queue resources. */
		isp_video_streamoff(file, vfh, video->type);
	}

	flush_workqueue(video->msg_work_queue);
	mutex_lock(&video->msg_queue_lock);
	atomic_set(&(video->status), VIDEO_STATUS_CLOSING);
	destroy_workqueue(video->msg_work_queue);
	mutex_unlock(&video->msg_queue_lock);
	mutex_lock(&video->queue_lock);
	vb2_queue_release(&handle->queue);
	mutex_unlock(&video->queue_lock);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	kfree(handle);
	file->private_data = NULL;
	atomic_set(&(video->status), VIDEO_STATUS_CLOSE_DONE);
	return 0;
}

static __poll_t isp_video_poll(struct file *file, poll_table *wait)
{
	struct a1000_isp_video_fh *vfh =
		to_a1000_isp_video_fh(file->private_data);
	struct a1000_isp_video *video = video_drvdata(file);
	__poll_t ret;

	// printk(KERN_INFO "ENTER: %s\n", __func__);
	mutex_lock(&video->queue_lock);
	ret = vb2_poll(&vfh->queue, file, wait);
	if (video->error)
		ret = EPOLLERR | EPOLLNVAL;
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_mmap_work(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	unsigned int buffer_index;
	unsigned int plane_index;
	struct a1000_isp_video_fh *vfh;
	struct vb2_queue *q;

	vfh = to_a1000_isp_video_fh(file->private_data);
	q = &vfh->queue;

	dev_dbg(q->dev,
		"%s: O: vm_start: 0x%08lX, vm_pgoff: 0x%08lX, size: 0x%08lX, dma_coherent: %d\n",
		__func__, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, dev_is_dma_coherent(q->dev));
	/* We first find the buffer_index and plane_index for this vma, since
	 * the vm_pgoff will be cleared after mmap
	 */
	ret = find_plane_by_vma(q, vma, &buffer_index, &plane_index);
	if (ret != 0) {
		dev_err(q->dev, "Can't find plane for vm_start\n");
		return ret;
	}

	ret = vb2_mmap(&vfh->queue, vma);
	if (ret == 0 && dev_is_dma_coherent(q->dev)) {
		struct vb2_buffer *vb;
		struct vb2_v4l2_buffer *vbuf;
		struct isp_buffer *buffer;

		dev_dbg(q->dev,
			"%s: save mmap_uaddr to flush dma for buffer_index: %u, plane_index: %u\n",
			__func__, buffer_index, plane_index);
		vb = q->bufs[buffer_index];
		vbuf = to_vb2_v4l2_buffer(vb);
		buffer = to_isp_buffer(vbuf);
		buffer->mmap_uaddr[plane_index] = (void *)(vma->vm_start);
		buffer->plane_size[plane_index] =
			vb->planes[plane_index].length;
	}
	dev_dbg(q->dev,
		"%s: N: vm_start: 0x%08lX, vm_pgoff: 0x%08lX, size: 0x%08lX, dma_coherent: %d\n",
		__func__, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, dev_is_dma_coherent(q->dev));

	return ret;
}

static int isp_video_mmap_debug(struct file *file, struct vm_area_struct *vma)
{
	struct a1000_isp_video *video = video_drvdata(file);
	int ret = dma_mmap_attrs(video->isp->dev, vma, vaddr_debug, paddr_debug,
				 memsz_debug, 0);

	if (ret) {
		pr_err("%s failed, error: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct a1000_isp_video *video = video_drvdata(file);

	// for normal
	if (video->isp->state == ISPDRV_STATE_WORK)
		return isp_video_mmap_work(file, vma);

	// for debug
	if (video->isp->state == ISPDRV_STATE_DEBUG)
		return isp_video_mmap_debug(file, vma);

	return 0;
}

static const struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = isp_video_open,
	.release = isp_video_release,
	.poll = isp_video_poll,
	.mmap = isp_video_mmap,
};

static int isp_set_brightness_raw(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_BRIGHTNESS, val);
}

static int isp_set_contrast_raw(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_CONTRAST, val);
}

static int isp_set_saturation_raw(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_SATURATION, val);
}

static int isp_set_hue_raw(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_HUE, val);
}

static int isp_set_gamma_raw(struct a1000_isp_video *video, int val)
{
	struct isp_ctrl ictrl;

	ictrl.value = val;

	return send_ctrl_msg_to_fw(video, IQ_GAMMA, &ictrl);
}

static int isp_set_awb_raw(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_AWB, val);
}

static int isp_set_brightness_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_brightness)
		return -EINVAL;

	return cam->ops.s_brightness(cam, val);
}

static int isp_set_contrast_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_contrast)
		return -EINVAL;

	return cam->ops.s_contrast(cam, val);
}

static int isp_set_saturation_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_saturation)
		return -EINVAL;

	return cam->ops.s_saturation(cam, val);
}

static int isp_set_hue_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_hue)
		return -EINVAL;

	return cam->ops.s_hue(cam, val);
}

static int isp_set_brightness(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_brightness_raw(video, val) :
			       isp_set_brightness_yuv(video, val);
}

static int isp_set_contrast(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_contrast_raw(video, val) :
			       isp_set_contrast_yuv(video, val);
}

static int isp_set_saturation(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_saturation_raw(video, val) :
			       isp_set_saturation_yuv(video, val);
}

static int isp_set_hue(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_hue_raw(video, val) :
			       isp_set_hue_yuv(video, val);
}

static int isp_set_awb_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_awb)
		return -EINVAL;

	return cam->ops.s_awb(cam, val);
}

static int isp_set_awb(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_awb_raw(video, val) :
			       isp_set_awb_yuv(video, val);
}

static int isp_set_aec_raw(struct a1000_isp_video *video, int val)
{
	// struct isp_ctrl *ictrl = (struct isp_ctrl *)(ctrl->p_new.p);
	return send_ctrl_val_to_fw(video, IQ_AECAGC, val);
}

static int isp_set_aec_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_aec)
		return -EINVAL;

	return cam->ops.s_aec(cam, val);
}

static int isp_set_aec(struct a1000_isp_video *video,
		       enum v4l2_exposure_auto_type auto_exposure)
{
	if (auto_exposure == V4L2_EXPOSURE_AUTO) {
		video->is_raw_camera ? isp_set_aec_raw(video, 1) :
				isp_set_aec_yuv(video, 1);
	} else {
		video->is_raw_camera ? isp_set_aec_raw(video, 0) :
				isp_set_aec_yuv(video, 0);
	}
	return 0;
}

static int isp_set_ydns(struct a1000_isp_video *video, int val)
{
	// struct isp_ctrl* ctrl_info = (struct isp_ctrl*)val->p_new.p;
	return send_ctrl_val_to_fw(video, IQ_YDNS, val);
}

static int isp_set_uvdns(struct a1000_isp_video *video, int val)
{
	// struct isp_ctrl* ctrl_info = (struct isp_ctrl*)val->p_new.p;
	return send_ctrl_val_to_fw(video, IQ_UVDNS, val);
}

static int isp_set_sharpen(struct a1000_isp_video *video, int val)
{
	return send_ctrl_val_to_fw(video, IQ_SHARPEN, val);
}

static int isp_set_gamma_yuv(struct a1000_isp_video *video, int val)
{
	struct camera_dev *cam = video->channel->cam_dev;

	if (!cam->ops.s_gamma)
		return -EINVAL;

	return cam->ops.s_gamma(cam, val);
}

static int isp_set_gamma(struct a1000_isp_video *video, int val)
{
	return video->is_raw_camera ? isp_set_gamma_raw(video, val) :
			       isp_set_gamma_yuv(video, val);
}

static int isp_set_test(struct a1000_isp_video *video, struct v4l2_ctrl *ctrl)
{
	struct isp_ctrl *ictrl = (struct isp_ctrl *)(ctrl->p_new.p);
	int i = 0;

	for (i = 0; i < 9; i++) {
		pr_debug("manualAWBGain[%d][%d]: %d\n", i / 3, i % 3,
			 ictrl->manualAWBGain[i / 3][i % 3]);
	}
	return 0;
}

static int isp_set_manual_wb_raw(struct a1000_isp_video *video,
				 struct v4l2_ctrl *ctrl)
{
	int i = 0;
	struct isp_ctrl *ictrl = (struct isp_ctrl *)(ctrl->p_new.p);

	for (i = 0; i < 9; i++) {
		pr_debug("AWBGain[%d]: %d\n", i,
			 ictrl->manualAWBGain[i / 3][i % 3]);
	}
	return send_ctrl_msg_to_fw(video, IQ_AWB, ictrl);
}

static int isp_set_manual_wb_yuv(struct a1000_isp_video *video,
				 struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int isp_set_manual_wb(struct a1000_isp_video *video,
			     struct v4l2_ctrl *ctrl)
{
	return video->is_raw_camera ? isp_set_manual_wb_raw(video, ctrl) :
			       isp_set_manual_wb_yuv(video, ctrl);
}

static int isp_set_manual_aec_raw(struct a1000_isp_video *video,
				  struct v4l2_ctrl *ctrl)
{
	int i;
	struct isp_ctrl *ictrl = (struct isp_ctrl *)(ctrl->p_new.p);

	for (i = 0; i < 3; i++) {
		pr_debug("aecExp[%d]: %d\n", i, ictrl->aecManualExp[i]);
		pr_debug("aecGain[%d]: %d\n", i, ictrl->aecManualGain[i]);
	}
	return send_ctrl_msg_to_fw(video, IQ_AECAGC, ictrl);
}

static int isp_set_manual_aec_yuv(struct a1000_isp_video *video,
				  struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int isp_set_manual_aec(struct a1000_isp_video *video,
			      struct v4l2_ctrl *ctrl)
{
	return video->is_raw_camera ? isp_set_manual_aec_raw(video, ctrl) :
			       isp_set_manual_aec_yuv(video, ctrl);
}

static int isp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
	struct a1000_isp_video *video = container_of(
		ctrl->handler, struct a1000_isp_video, ctrl_handler);

	mutex_lock(&video->ctrl_lock);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = isp_set_brightness(video, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = isp_set_contrast(video, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = isp_set_saturation(video, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = isp_set_hue(video, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = isp_set_awb(video, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = isp_set_aec(video, ctrl->val);
		break;
	case V4L2_CID_ISP_YDNS:
		ret = isp_set_ydns(video, ctrl->val);
		break;
	case V4L2_CID_ISP_UVDNS:
		ret = isp_set_uvdns(video, ctrl->val);
		break;
	case V4L2_CID_SHARPNESS:
		ret = isp_set_sharpen(video, ctrl->val);
		break;
	/*Controls with compound data*/
	case V4L2_CID_ISP_TEST:
		ret = isp_set_test(video, ctrl);
		break;
	case V4L2_CID_GAMMA:
		ret = isp_set_gamma(video, ctrl->val);
		break;
	case V4L2_CID_ISP_MANUAL_WB:
		ret = isp_set_manual_wb(video, ctrl);
		break;
	case V4L2_CID_ISP_MANUAL_EXPOSURE:
		ret = isp_set_manual_aec(video, ctrl);
		break;
	/* CIDs below are not implemented now */
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&video->ctrl_lock);

	return ret;
}

uint32_t get_ctrl_val_by_fw(struct a1000_isp_video *video, int item)
{
	int timeout;
	struct a1000_isp_device *isp = video->isp;
	int index = video->video_index;

	// send get ctrl msg to fw
	send_get_ctrl_val_to_fw(video, item);
	// wait for recv the ictrl value from fw
	timeout = wait_event_timeout(isp->ctrl_recv_wq,
				     (isp->isp_ctrl_status == index),
				     msecs_to_jiffies(3000));

	if (timeout == 0) {
		pr_err("%s timeout", __func__);
		return -EINVAL;
	} else {
		return video->isp->ctrl_get_val;
	}
}

static int isp_g_ctrl(struct v4l2_ctrl *ctrl)
{
	int retval = 0;
	struct a1000_isp_video *video = container_of(
		ctrl->handler, struct a1000_isp_video, ctrl_handler);
	mutex_lock(&video->ctrl_lock);
	switch (ctrl->id) {
	case V4L2_CID_SHARPNESS:
		retval = get_ctrl_val_by_fw(video, IQ_SHARPEN);
		break;
	case V4L2_CID_BRIGHTNESS:
		retval = get_ctrl_val_by_fw(video, IQ_BRIGHTNESS);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		retval = get_ctrl_val_by_fw(video, IQ_AECAGC);
		break;
	case V4L2_CID_ISP_YDNS:
		retval = get_ctrl_val_by_fw(video, IQ_YDNS);
		break;
	case V4L2_CID_ISP_UVDNS:
		retval = get_ctrl_val_by_fw(video, IQ_UVDNS);
		break;
	}
	mutex_unlock(&video->ctrl_lock);
	return retval;
}
static const struct v4l2_ctrl_ops isp_ctrl_ops = {
	.s_ctrl = isp_s_ctrl,
	.g_volatile_ctrl = isp_g_ctrl,
};

static bool isp_ctrl_type_equal(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr1,
				union v4l2_ctrl_ptr ptr2)
{
	idx *= ctrl->elem_size;
	return !memcmp(ptr1.p + idx, ptr2.p + idx, ctrl->elem_size);
}

static void isp_ctrl_type_init(const struct v4l2_ctrl *ctrl, u32 idx,
			       union v4l2_ctrl_ptr ptr)
{
	idx *= ctrl->elem_size;
	memset(ptr.p + idx, 0, ctrl->elem_size);
}

static void isp_ctrl_type_log(const struct v4l2_ctrl *ctrl)
{
}

static int isp_ctrl_type_validate(const struct v4l2_ctrl *ctrl, u32 idx,
				  union v4l2_ctrl_ptr ptr)
{
	// int i;
	// isp_ctrl_t* ctrl_info = (isp_ctrl_t*)ptr.p;
	// switch (ctrl->type) {
	//	case V4L2_CTRL_ISP_TYPE_MANUAL_EXPOSURE:
	//		if(ctrl_info->aecParam.aecManualExp[0] > 1326)
	//			return -ERANGE;
	//		if(ctrl_info->aecParam.aecManualExp[1] > 62)
	//			return -ERANGE;
	//		if(ctrl_info->aecParam.aecManualExp[2] > 5)
	//			return -ERANGE;
	//		for(i = 0 ; i < 3; i++)
	//		{
	//			if(ctrl_info->aecParam.aecManualGain[0] > 128 ||
	// ctrl_info->aecParam.aecManualGain[0] < 16)
	// return -ERANGE;
	//		}
	//		break;
	//	default:
	//		return 0;
	// }
	return 0;
}

static const struct v4l2_ctrl_type_ops isp_ctrl_type_ops = {
	.equal = isp_ctrl_type_equal,
	.init = isp_ctrl_type_init,
	.log = isp_ctrl_type_log,
	.validate = isp_ctrl_type_validate,
};

static void ipc_msg_handler(struct work_struct *work)
{
	struct a1000_isp_video *video;
	struct internal_msg *cache_msg;

	cache_msg = container_of(work, struct internal_msg, msg_work);
	video = cache_msg->video_ptr;
	switch (cache_msg->cmd_minor) {
	case MINOR_SYNC_ISP_VIEW_FRAME_DONE:
		{
			// printk("ipc_msg_handler:cache_msg->cmd_minor is
			// %x",cache_msg->cmd_minor);
			if (VIDEO_STATUS_STREAMON_DONE ==
			    atomic_read(&video->status)) {
				video->rx_buf_count++;
				handle_view_video_sync_msg(video, cache_msg);
			} else {
				pr_err("not stream on status, return;\n");
			}
			// received new frame buffer, clear error flag
			video->error = false;
			break;
		}

	case MINOR_ISP_RAW_BUF_DONE:
		{
			// printk("ipc_msg_handler:cache_msg->cmd_minor is
			// %x",cache_msg->cmd_minor);
			if (VIDEO_STATUS_STREAMON_DONE ==
			    atomic_read(&video->status)) {
				video->rx_buf_count++;
				handle_raw_video_sync_msg(video, cache_msg);
			} else {
				pr_err("not stream on status, return;\n");
			}
			// received new frame buffer, clear error flag
			video->error = false;
			break;
		}

	case MINOR_ABNORMAL:
		{
			int i;
			int buf_matched = 0;
			struct isp_buffer *last_wait_buf;
			struct list_head *pos;
			abnormal_t *fw_ab_info =
				(abnormal_t *)&(cache_msg->user_data[0]);

			video->fw_ab_info = *fw_ab_info;
			if (VIDEO_STATUS_STREAMON_DONE !=
			    atomic_read(&video->status))
				break;

			dev_err(&video->video.dev,
				"ABNORMAL: 0x%02X 0x%02X 0x%02X 0x%02X 0x%08X 0x%08X 0x%08X\n",
				fw_ab_info->abnormalId,
				fw_ab_info->abnormalType, fw_ab_info->rsv0[0],
				fw_ab_info->rsv0[1], fw_ab_info->rsv[0],
				fw_ab_info->rsv[1], fw_ab_info->rsv[2]);
			mutex_lock(&video->wait_queue_lock);
			list_for_each(pos, &video->wait_buf_queue) {
				last_wait_buf = list_entry(
					pos, struct isp_buffer, node);
				if (last_wait_buf == NULL)
					continue;
				for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
					if (video->current_views_id[i]) {
						buf_matched +=
							(fw_ab_info->rsv[i] ==
							 last_wait_buf->dma[i]) ?
								1 :
								0;
					}
				}
				if (buf_matched) {
					list_del(&last_wait_buf->node);
					break;
				}
			}
			mutex_unlock(&video->wait_queue_lock);

			video->total_bad_frames++;
			video->error = true;
			wake_up(&video->queue->done_wq);
			break;
		}
	default:
		dev_err(&video->video.dev, "ERROR: Unknow cmd cmd_minor %d\n",
			cache_msg->cmd_minor);
		break;
	}
	// done, put cache msg to free cache
	cache_msg->cmd_main = 0;
	cache_msg->cmd_minor = 0;
	cache_msg->tick = 0;
	cache_msg->user_data[0] = 0;
	cache_msg->user_data[1] = 0;
	cache_msg->user_data[2] = 0;
	cache_msg->user_data[3] = 0;
	mutex_lock(&video->cache_queue_lock);
	list_add_tail(&cache_msg->node, &video->free_cache_queue);
	mutex_unlock(&video->cache_queue_lock);
}

static void dump_debug_info(struct a1000_isp_video *video)
{
	struct isp_buffer *buf;
	int wait_count = 0;
	int free_count = 0;

	mutex_lock(&video->wait_queue_lock);
	list_for_each_entry(buf, &video->wait_buf_queue, node) {
		wait_count++;
		pr_debug(
			"%s: video_index: %02d, wait: 0x%08X, 0x%08X, 0x%08X, wait_count: %2d\n",
			__func__, video->video_index, buf->dma[0], buf->dma[1],
			buf->dma[2], wait_count);
	}
	mutex_unlock(&video->wait_queue_lock);

	mutex_lock(&video->free_queue_lock);
	list_for_each_entry(buf, &video->free_buf_queue, node) {
		free_count++;
		pr_debug(
			"%s: video_index: %02d, free: 0x%08X, 0x%08X, 0x%08X, free_count: %2d\n",
			__func__, video->video_index, buf->dma[0], buf->dma[1],
			buf->dma[2], free_count);
	}
	mutex_unlock(&video->free_queue_lock);
}

static int send_next_buf_to_fw(struct a1000_isp_video *video)
{
	int empty;
	struct isp_buffer *buf;
	int wait_count;
	int flag;

	// send next framebuffer to fw
	mutex_lock(&video->free_queue_lock);
	empty = list_empty(&video->free_buf_queue);
	if (empty) {
		mutex_unlock(&video->free_queue_lock);
		flag = 0;
		wait_count = 0;
		// free queue is empty, check wait queue
		mutex_lock(&video->wait_queue_lock);
		list_for_each_entry(buf, &video->wait_buf_queue, node) {
			wait_count++;
			if (buf->cycle_count > MIN_FRAME_BUFFER_NUM) {
				flag = 1;
				// delete this node from head and insert it to
				// tail
				list_del(&buf->node);
				list_add_tail(&buf->node,
					      &video->wait_buf_queue);
				break;
			}
		}
		mutex_unlock(&video->wait_queue_lock);

		if (flag) {
			pr_err("resend buf 0x%x, cycle = %d\n", buf->dma[0],
			       buf->cycle_count);
			buf->cycle_count = 0;
			send_buf_to_fw(video, buf->dma, buf->vb.vb2_buf.index, 0);
		} else {
			if (video->tx_drop_count == 0) {
				// driver received fw reserved buf at first
				// because the fw delayed to fill the sent buf
				// reset the count here
				video->rx_reserved_count = 0;
			}
			video->tx_drop_count++;
			return -1;
		}
	} else {
		buf = list_first_entry(&video->free_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		mutex_unlock(&video->free_queue_lock);
		buf->cycle_count = 0;
		mutex_lock(&video->wait_queue_lock);
		list_add_tail(&buf->node, &video->wait_buf_queue);
		mutex_unlock(&video->wait_queue_lock);
		send_buf_to_fw(video, buf->dma, buf->vb.vb2_buf.index, 0);
	}

	return 0;
}

static int handle_view_video_sync_msg(struct a1000_isp_video *video,
				 struct internal_msg *cache_msg)
{
	int i;
	int duplicate_msg;
	struct isp_buffer *buf;
	int buf_reserved = 0;
	u64 isp_capture_ktime = 0;
	uint32_t *fbuf;
	uint32_t tick;
	int64_t ktime;
	u32 sequence;
	new_frame_done_t *frame;

	duplicate_msg = 0;

	frame = (new_frame_done_t *)&(cache_msg->user_data[0]);
	fbuf = frame->viewBuf;
	tick = cache_msg->tick;
	ktime = cache_msg->ktime;

	++video->drv_sequence;
	if (video->hdmi_video) {
		// hdmi has no vsync, so using driver time and count
		isp_capture_ktime = (uint64_t)ktime;
		sequence = video->drv_sequence;
	} else {
		isp_capture_ktime = (uint64_t)ktime - tick * ISP_TICK_TO_NS;
		sequence = frame->viewReply.vsyncCnt;
	}
	video->last_timestamp = isp_capture_ktime;

	pr_debug(
		"RECVBUF: video_index: %02d, 0x%08X, 0x%08X, 0x%08X, tick: 0x%08X\n",
		video->video_index, fbuf[0], fbuf[1], fbuf[2], tick);

	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->current_views_id[i])
			buf_reserved += is_reserved_buf(video, fbuf[i]) ? 1 : 0;
	}

	if (buf_reserved) {
		pr_debug("%s: video_index: %02d, receive reserved buf: %d\n",
			__func__, video->video_index, buf_reserved);
		dump_debug_info(video);
		// sanity check
		if (buf_reserved != video->current_views_num) {
			pr_err("reserved buf abnormal\n");
			// TODO: error handler
		}
		video->rx_reserved_count++;

		mutex_lock(&video->wait_queue_lock);
		list_for_each_entry(buf, &video->wait_buf_queue, node) {
			buf->cycle_count++;
		}
		mutex_unlock(&video->wait_queue_lock);
	} else {
		int buf_matched = 0;
		struct isp_buffer *last_wait_buf;
		struct list_head *pos;

		if (!video->first_buf_received)
			video->first_buf_received = true;

		// compare the returned buf address with the wait buffer
		mutex_lock(&video->wait_queue_lock);
		list_for_each(pos, &video->wait_buf_queue) {
			last_wait_buf =
				list_entry(pos, struct isp_buffer, node);
			for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
				if (video->current_views_id[i]) {
					buf_matched += (fbuf[i] ==
							last_wait_buf->dma[i]) ?
							       1 :
							       0;
					// buf_match_view_bitmap |= (1<< i);
				}
			}
			if (buf_matched) {
				list_del(&last_wait_buf->node);
				break;
			}
			last_wait_buf->cycle_count++;
			pr_debug("%s: video_index: %02d, receive wrong order buf: 0x%08X 0x%08X 0x%08X, wait: 0x%08X 0x%08X 0x%08X, tick: 0x%08X\n",
			       __func__, video->video_index, fbuf[0], fbuf[1],
			       fbuf[2], last_wait_buf->dma[0],
			       last_wait_buf->dma[1], last_wait_buf->dma[2],
			       tick);
			// TODO: error handler
		}
		mutex_unlock(&video->wait_queue_lock);
		if (buf_matched) {
			// sanity check
			if (buf_matched != video->current_views_num) {
				pr_err("matched buf abnormal\n");
				// TODO: error handler
			}
			video->last_done_buf[0] = fbuf[0];
			last_wait_buf->cycle_count = 0;
			/*save timestamp to timecode*/
			last_wait_buf->vb.vb2_buf.timestamp = isp_capture_ktime;
			last_wait_buf->vb.sequence = sequence;
			vb2_buffer_done(&last_wait_buf->vb.vb2_buf,
					VB2_BUF_STATE_DONE);
			if (!video->error)
				video->last_good_sequence = sequence;
		} else {
			// not found buffer in wait list
			if (video->last_done_buf[0] == fbuf[0])
				duplicate_msg = 1;
			pr_err("returned buf not matched, i = %d, last = 0x%x, buf =0x%x\n",
			       video->video_index, video->last_done_buf[0],
			       fbuf[0]);
		}
	}

	if (video->first_buf_received && (duplicate_msg == 0))
		send_next_buf_to_fw(video);

	return 0;
}


static int handle_raw_video_sync_msg(struct a1000_isp_video *video,
				 struct internal_msg *cache_msg)
{
	int duplicate_msg;
	struct isp_buffer *buf;
	int buf_reserved = 0;
	u64 isp_capture_ktime = 0;
	uint32_t fbuf;
	uint32_t tick;
	int64_t ktime;
	u32 sequence;
	new_rawframe_done_t *frame;

	duplicate_msg = 0;

	frame = (new_rawframe_done_t *)&(cache_msg->user_data[0]);
	fbuf = frame->rawBuf;
	tick = cache_msg->tick;
	ktime = cache_msg->ktime;

	++video->drv_sequence;
	if (video->hdmi_video) {
		// hdmi has no vsync, so using driver time and count
		isp_capture_ktime = (uint64_t)ktime;
		sequence = video->drv_sequence;
	} else {
		isp_capture_ktime = (uint64_t)ktime - tick * ISP_TICK_TO_NS;
		sequence = frame->vsyncCnt;
	}
	video->last_timestamp = isp_capture_ktime;

	pr_err(
		"RECVBUF: raw video_index: %02d, 0x%08X,, tick: 0x%08X\n",
		video->video_index, fbuf, tick);

	//buf_reserved += is_reserved_buf(video, fbuf) ? 1 : 0;
	if (fbuf == video->reserve_buf) {
		buf_reserved = 1;
	}

#if 1
	if (buf_reserved) {
		pr_info("%s: video_index: %02d, receive reserved buf: %d\n",
			__func__, video->video_index, buf_reserved);
		dump_debug_info(video);
		// sanity check
		video->rx_reserved_count++;

		mutex_lock(&video->wait_queue_lock);
		list_for_each_entry(buf, &video->wait_buf_queue, node) {
			buf->cycle_count++;
		}
		mutex_unlock(&video->wait_queue_lock);
	} else {
		int buf_matched = 0;
		struct isp_buffer *last_wait_buf;
		struct list_head *pos;

		if (!video->first_buf_received)
			video->first_buf_received = true;

		// compare the returned buf address with the wait buffer
		mutex_lock(&video->wait_queue_lock);
		list_for_each(pos, &video->wait_buf_queue) {
			last_wait_buf =
				list_entry(pos, struct isp_buffer, node);

			buf_matched = (fbuf ==
							last_wait_buf->dma[0]) ?
							       1 :
							       0;
			if (buf_matched) {
				list_del(&last_wait_buf->node);
				break;
			}
			last_wait_buf->cycle_count++;
			//pr_err("%s: video_index: %02d, receive wrong order buf: 0x%08X 0x%08X 0x%08X, wait: 0x%08X 0x%08X 0x%08X, tick: 0x%08X\n",
			//       __func__, video->video_index, fbuf[0], fbuf[1],
			//       fbuf[2], last_wait_buf->dma[0],
			//       last_wait_buf->dma[1], last_wait_buf->dma[2],
			//       tick);
			// TODO: error handler
		}
		mutex_unlock(&video->wait_queue_lock);
		if (buf_matched) {
			video->last_done_buf[0] = fbuf;
			last_wait_buf->cycle_count = 0;
			/*save timestamp to timecode*/
			last_wait_buf->vb.vb2_buf.timestamp = isp_capture_ktime;
			last_wait_buf->vb.sequence = sequence;
			vb2_buffer_done(&last_wait_buf->vb.vb2_buf,
					VB2_BUF_STATE_DONE);
			if (!video->error)
				video->last_good_sequence = sequence;
		} else {
			// not found buffer in wait list
			if (video->last_done_buf[0] == fbuf)
				duplicate_msg = 1;
			pr_err("returned buf not matched, i = %d, last = 0x%x, buf =0x%x\n",
			       video->video_index, video->last_done_buf[0],
			       fbuf);
		}
	}

	if (video->first_buf_received && (duplicate_msg == 0))
		send_next_buf_to_fw(video);
#endif

	return 0;
}

// get element from free cache queue, copy data
int copy_msg_to_video_cache(struct a1000_isp_video *video, uint16_t main,
			    uint16_t minor, uint32_t tick, int64_t ktime,
			    uint32_t *data, int num)
{
	struct internal_msg *cache_msg;
	int empty;
	int video_status;

	video_status = atomic_read(&video->status);
	if ((video_status < VIDEO_STATUS_OPEN_DONE) ||
	    (video_status >= VIDEO_STATUS_CLOSING)) {
		pr_debug(
			"video %d: status = %d, can't handle FW message, main: %d, minor = %d, 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
			video->video_index, video_status, main, minor, data[0],
			data[1], data[2], data[3]);
		return -1;
	}

	mutex_lock(&video->cache_queue_lock);
	empty = list_empty(&video->free_cache_queue);
	if (empty) {
		mutex_unlock(&video->cache_queue_lock);
		pr_err("ERROR: video_index: %02d NO free cache msg element\n",
		       video->video_index);
		return -1;
	}
	cache_msg = list_first_entry(&video->free_cache_queue,
				     struct internal_msg, node);
	list_del(&cache_msg->node);
	mutex_unlock(&video->cache_queue_lock);

	// copy msg to cache
	cache_msg->cmd_main = main;
	cache_msg->cmd_minor = minor;
	cache_msg->tick = tick;
	cache_msg->ktime = ktime;
	memcpy(&cache_msg->user_data[0], data,
	       num * sizeof(uint32_t)); // 4 byte per data
	mutex_lock(&video->msg_queue_lock);
	if (video_status < VIDEO_STATUS_CLOSING)
		queue_work(video->msg_work_queue, &cache_msg->msg_work);
	mutex_unlock(&video->msg_queue_lock);

	return 0;
}

static int init_video_media_command_array(struct a1000_isp_device *isp,
					  struct a1000_isp_video *video,
					  int video_index)
{
	int i;
	int cmd_index_start;
	tSoneCmdp *cmdp;

	volatile uint64_t addr_offset;

	cmdp = (tSoneCmdp *)isp->cmdp_vaddr;
	// printk(KERN_ERR
	//        "===== init_video_media_command_array, video_index = %d, cmdp
	//        = 0x%lx\n", video_index, (uint64_t)cmdp);

	cmd_index_start = video_index * MAX_MEDIA_COMMAND_PER_VIDEO;
	for (i = 0; i < MAX_MEDIA_COMMAND_PER_VIDEO; i++) {
		addr_offset =
			(uint64_t)(&cmdp->ch[DRV_CH_INDEX]
					    .cqueue.c0[cmd_index_start + i]) -
			(uint64_t)(isp->cmdp_vaddr);
		video->media_cmd_array[i].media_cmd_paddr =
			(addr_offset + isp->cmdp_paddr) & LOW_32_BIT_MASK;
		video->media_cmd_array[i].media_cmd_vaddr =
			&cmdp->ch[DRV_CH_INDEX].cqueue.c0[cmd_index_start + i];
		video->media_cmd_array[i].index = i;
		// printk(KERN_ERR
		//        "i = %d, addr_offset = 0x%x, paddr = 0x%x, vaddr =
		//        0x%lx, \n", i, addr_offset,
		//        video->media_cmd_array[i].media_cmd_paddr,
		//        video->media_cmd_array[i].media_cmd_vaddr);
	}

	return 0;
}

// 0/1/2/3/4/5 YUV420(3 channels,Y/U/V)/NV12(2 channels,Y/UV)/
// NV21(2 channels,Y/VU)/YUV422(1 channel,YUYV)/RGB888(1 channel)
static int convert_view_format_to_v4l2(int view_id, int view_format)
{
	if (view_id == 0) {
		switch (view_format) {
		case 0:
			return V4L2_PIX_FMT_YUV420;
		case 1:
			return V4L2_PIX_FMT_NV12;
		case 2:
			return V4L2_PIX_FMT_NV21;
		case 3:
			return V4L2_PIX_FMT_YUYV;
		case 4:
			return V4L2_PIX_FMT_RGB24;
		default:
			pr_err("ERROR: unsupported format %d\n", view_format);
			return -1;
		}
	} else if (view_id == 1) {
		// 0/1/2/3/4/5 YUV420(3 channels,Y/U/V)/NV12(2 channels,Y/UV)
		//  /NV21(2 channels,Y/VU)/YUV422(1 channel,YUYV)/Raw(1
		//  channel)/disabled
		switch (view_format) {
		case 0:
			return V4L2_PIX_FMT_YUV420;
		case 1:
			return V4L2_PIX_FMT_NV12;
		case 2:
			return V4L2_PIX_FMT_NV21;
		case 3:
			return V4L2_PIX_FMT_YUYV;
		case 4:
			// to do: should return RAW
			return -1;
		default:
			pr_err("ERROR: unsupported format %d\n", view_format);
			return -1;
		}
	} else if (view_id == 2) {
		return V4L2_PIX_FMT_GREY;
	}

	pr_err("ERROR: wrong view id %d\n", view_id);
	return -1;
}

static int update_views_from_camera_config(struct a1000_isp_video *video,
					   struct camera_dev *cam_dev)
{
	int view_fmt;

	view_fmt = cam_dev->isp_data.viewinfo[0].viewFmt;
	if (view_fmt != View0_Dis) {
		video->views[0].enable = true;
		video->views[0].is_pdns_input = false; // false by default
		video->views[0].index = 0;
		video->views[0].video = video;
		video->views[0].width = cam_dev->isp_data.viewinfo[0].width;
		video->views[0].height = cam_dev->isp_data.viewinfo[0].height;
		video->views[0].format =
			convert_view_format_to_v4l2(0, view_fmt);
		video->views[0].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View0);
		// pr_info("v0_w = %d, v0_h = %d\n",
		//	video->views[0].width, video->views[0].height);
	}
	view_fmt = cam_dev->isp_data.viewinfo[1].viewFmt;
	if (view_fmt != View1_Dis) {
		video->views[1].enable = true;
		video->views[1].is_pdns_input = false; // false by default
		video->views[1].index = 1;
		video->views[1].video = video;
		video->views[1].width = cam_dev->isp_data.viewinfo[1].width;
		video->views[1].height = cam_dev->isp_data.viewinfo[1].height;
		video->views[1].format =
			convert_view_format_to_v4l2(1, view_fmt);
		video->views[1].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View1);
		// pr_info("v1_w = %d, v1_h = %d\n",
		//	video->views[1].width, video->views[1].height);
	}

	view_fmt = cam_dev->isp_data.viewinfo[2].viewFmt;
	if (view_fmt != View2_Dis) {
		video->views[2].enable = true;
		video->views[2].is_pdns_input = false; // false by default
		video->views[2].index = 2;
		video->views[2].video = video;
		video->views[2].width = cam_dev->isp_data.viewinfo[2].width;
		video->views[2].height = cam_dev->isp_data.viewinfo[2].height;
		video->views[2].format =
			convert_view_format_to_v4l2(2, view_fmt);
		video->views[2].cmd_view_id =
				((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
				 S00View2);
		// pr_info("v2_w = %d, v2_h = %d\n",
		//	video->views[2].width, video->views[2].height);
	}
	video->pdns_input_view = -1; // default to invalid value;
	if (cam_dev->isp_data.pdnsinfo.pdnsMode) {
		int inputview;

		inputview = cam_dev->isp_data.pdnsinfo.pdnsViewSel;
		if ((inputview >= 0) && (inputview < MAX_VIEWS_PER_CAMERA)) {
			video->views[inputview].is_pdns_input = true;
			video->pdns_input_view = inputview;
		}
	}

	return 0;
}

static int update_rawvideo_from_camera_config(struct a1000_isp_video *video,
					      struct camera_dev *cam_dev)
{
	int view_fmt;

	view_fmt = cam_dev->isp_data.viewinfo[0].viewFmt;
	if (view_fmt != View0_Dis) {
		video->views[0].enable = true;
		video->views[0].is_pdns_input = false; // false by default
		video->views[0].index = 0;
		video->views[0].video = video;
		video->views[0].width = cam_dev->isp_data.viewinfo[0].width;
		video->views[0].height = cam_dev->isp_data.viewinfo[0].height;
		video->views[0].format =
			convert_view_format_to_v4l2(0, view_fmt);
		video->views[0].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View0);
		// pr_info("v0_w = %d, v0_h = %d\n",
		//	video->views[0].width, video->views[0].height);
	}
	view_fmt = cam_dev->isp_data.viewinfo[1].viewFmt;
	if (view_fmt != View1_Dis) {
		video->views[1].enable = true;
		video->views[1].is_pdns_input = false; // false by default
		video->views[1].index = 1;
		video->views[1].video = video;
		video->views[1].width = cam_dev->isp_data.viewinfo[1].width;
		video->views[1].height =
				cam_dev->isp_data.viewinfo[1].height;
		video->views[1].format =
			convert_view_format_to_v4l2(1, view_fmt);
		video->views[1].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View1);
		// pr_info("v1_w = %d, v1_h = %d\n",
		//	video->views[1].width, video->views[1].height);
	}

	view_fmt = cam_dev->isp_data.viewinfo[2].viewFmt;
	if (view_fmt != View2_Dis) {
		video->views[2].enable = true;
		video->views[2].is_pdns_input = false; // false by default
		video->views[2].index = 2;
		video->views[2].video = video;
		video->views[2].width = cam_dev->isp_data.viewinfo[2].width;
		video->views[2].height = cam_dev->isp_data.viewinfo[2].height;
		video->views[2].format =
			convert_view_format_to_v4l2(2, view_fmt);
		video->views[2].cmd_view_id =
				((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
				 S00View2);
		// pr_info("v2_w = %d, v2_h = %d\n",
		//	video->views[2].width, video->views[2].height);
	}
	video->pdns_input_view = -1; // default to invalid value;

	return 0;
}

static int video_ctrl_init(struct a1000_isp_video *video)
{
	/* v4l2 ctrl support */
	static const struct v4l2_ctrl_config ctrl_test = {
		.id = V4L2_CID_ISP_TEST,
		.type = V4L2_CTRL_ISP_TYPE_TEST,
		.type_ops = &isp_ctrl_type_ops,
		.ops = &isp_ctrl_ops,
		.name = "test",
		.elem_size = sizeof(struct isp_ctrl),
	};

	static const struct v4l2_ctrl_config ctrl_manual_white_balance = {
		.id = V4L2_CID_ISP_MANUAL_WB,
		.type = V4L2_CTRL_ISP_TYPE_MANUAL_WB,
		.type_ops = &isp_ctrl_type_ops,
		.ops = &isp_ctrl_ops,
		.name = "MWB",
		.elem_size = sizeof(struct isp_ctrl),
	};

	static const struct v4l2_ctrl_config ctrl_manual_exposure = {
		.id = V4L2_CID_ISP_MANUAL_EXPOSURE,
		.type = V4L2_CTRL_ISP_TYPE_MANUAL_EXPOSURE,
		.type_ops = &isp_ctrl_type_ops,
		.ops = &isp_ctrl_ops,
		.name = "MEC",
		.elem_size = sizeof(struct isp_ctrl),
	};

	static const struct v4l2_ctrl_config ctrl_ydns = {
		.id = V4L2_CID_ISP_YDNS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		//.flags = V4L2_CTRL_FLAG_SLIDER,
		.ops = &isp_ctrl_ops,
		.name = "YDNS",
		.max = 10,
		.min = 0,
		.def = 0,
		.step = 1,
	};

	static const struct v4l2_ctrl_config ctrl_uvdns = {
		.id = V4L2_CID_ISP_UVDNS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		//.flags = V4L2_CTRL_FLAG_SLIDER,
		.ops = &isp_ctrl_ops,
		.name = "UVDNS",
		.max = 10,
		.min = 0,
		.def = 0,
		.step = 1,
	};

	struct v4l2_ctrl_handler *hdl = &video->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 15);

	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 255, 1,
			  128);
	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_CONTRAST, 0, 255, 1,
			  128);
	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_SATURATION, 0, 255, 1,
			  128);
	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_HUE, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_SHARPNESS, 0, 10, 1, 0);
	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1,
			  1, 1);
	v4l2_ctrl_new_std_menu(hdl, &isp_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
			       V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);

	v4l2_ctrl_new_std(hdl, &isp_ctrl_ops, V4L2_CID_GAMMA, 0, GAMMA_MAX, 1,
			  4); /*gamma value range1.8~2.6 default 2.2*/

	v4l2_ctrl_new_custom(hdl, &ctrl_test, NULL);
	v4l2_ctrl_new_custom(hdl, &ctrl_manual_white_balance, NULL);
	v4l2_ctrl_new_custom(hdl, &ctrl_manual_exposure, NULL);
	v4l2_ctrl_new_custom(hdl, &ctrl_ydns, NULL);
	v4l2_ctrl_new_custom(hdl, &ctrl_uvdns, NULL);

	video->video.ctrl_handler = &video->ctrl_handler;

	if (hdl->error) {
		pr_err("v4l2 control init error %d\n", hdl->error);
		v4l2_ctrl_handler_free(&video->ctrl_handler);
	}

	return 0;
}

static int a1000_isp_video_init(struct a1000_isp_device *isp,
				struct a1000_isp_video *video, int index)
{
	int i;

	// default entity to isp view
	atomic_set(&(video->status), VIDEO_STATUS_INVALID);
	video->error = false;
	video->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	INIT_LIST_HEAD(&video->free_buf_queue);
	INIT_LIST_HEAD(&video->wait_buf_queue);
	INIT_LIST_HEAD(&video->free_cache_queue);
	init_video_media_command_array(isp, video, index);

	for (i = 0; i < MAX_MEDIA_COMMAND_PER_VIDEO; i++) {
		INIT_WORK(&video->cache_msg[i].msg_work, ipc_msg_handler);
		video->cache_msg[i].video_ptr = video;
		list_add_tail(&video->cache_msg[i].node,
			      &video->free_cache_queue);
	}

	mutex_init(&video->mutex);
	mutex_init(&video->msg_queue_lock);
	mutex_init(&video->queue_lock);
	mutex_init(&video->ctrl_lock);
	mutex_init(&video->cache_queue_lock);
	mutex_init(&video->free_queue_lock);
	mutex_init(&video->wait_queue_lock);
	video->video.fops = &isp_video_fops;
	video->video.vfl_type = VFL_TYPE_VIDEO;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &isp_video_ioctl_ops;
	video->video.device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				   V4L2_CAP_STREAMING;
	video_set_drvdata(&video->video, video);
	video_ctrl_init(video);

	return 0;
}

int isp_channel_init_views_video(struct a1000_isp_device *isp,
				 struct bst_isp_channel *isp_channel,
				 int chn_sn)
{
	int video_sn;
	int ret;
	struct camera_dev *cam;
	struct a1000_isp_video *video;

	cam = isp_channel->cam_dev;
	video = &(isp_channel->views_video);
	video->is_pdns = false;
	if (isp_channel->is_hdmi)
		video->hdmi_video = true;
	else
		video->hdmi_video = false;
	// default entity to isp view
	video->isp = isp;
	snprintf(video->video.name, sizeof(video->video.name),
		 "isp-channel-%d-views-video", chn_sn);

	video_sn = chn_sn; // 0 ~ 11
	video->enabled = true;
	video->chn_index = chn_sn;
	video->video_index = video_sn;
	video->channel = isp_channel;
	if (cam != NULL) {
		uint8_t type = cam->isp_data.rawinfo.dataType;

		video->is_raw_camera = ((type != DT_UYVY) && (type != DT_YUYV));
		update_views_from_camera_config(video, cam);
	}
	a1000_isp_video_init(isp, video, video_sn);
	video->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&video->video.entity, 1, &video->pad);

	return 0;
}

int isp_channel_init_raw_video(struct a1000_isp_device *isp,
			       struct bst_isp_channel *isp_channel, int chn_sn)
{
	int video_sn;
	int ret;
	struct camera_dev *cam;
	struct a1000_isp_video *video;

	cam = isp_channel->cam_dev;
	video = &(isp_channel->raw_video);
	video->is_pdns = false;
	video->is_raw_video = true; // raw video means not processed by ISP, include yuyv input
	//if (isp_channel->is_hdmi)
	//	video->hdmi_video = true;
	//else
	//	video->hdmi_video = false;
	// default entity to isp view
	video->isp = isp;
	snprintf(video->video.name, sizeof(video->video.name),
		 "isp-channel-%d-raw-video", chn_sn);

	video_sn = chn_sn + MAX_ISP_CHANNEL; // 12 ~ 23
	video->enabled = true; // will be updated from config
	video->chn_index = chn_sn;
	video->video_index = video_sn;
	video->channel = isp_channel;

	update_rawvideo_from_camera_config(video, cam);
	a1000_isp_video_init(isp, video, video_sn);
	video->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&video->video.entity, 1, &video->pad);

	return 0;
}

void a1000_isp_video_cleanup(struct a1000_isp_video *video)
{
	mutex_destroy(&video->queue_lock);
	mutex_destroy(&video->msg_queue_lock);
	mutex_destroy(&video->mutex);
	mutex_destroy(&video->ctrl_lock);
}

int a1000_isp_video_register(struct a1000_isp_video *video,
			     struct v4l2_device *vdev)
{
	int ret;

	video->video.v4l2_dev = vdev;

	ret = video_register_device(&video->video, VFL_TYPE_VIDEO,
				    video->video_index);
	if (ret < 0)
		dev_err(video->isp->dev,
			"%s: could not register video device (%d)\n", __func__,
			ret);

	dev_info(video->isp->dev, "%s() line %d , registered /dev/video%d\n",
		 __func__, __LINE__, video->video_index);

	return ret;
}

void a1000_isp_video_unregister(struct a1000_isp_video *video)
{
	if (video_is_registered(&video->video))
		video_unregister_device(&video->video);
}

void dump_video_debug_info(struct a1000_isp_video *video)
{
	uint32_t *prx_cnt;
	uint32_t *prx_error_cnt;
	uint32_t *poverflow_cnt;
	uint32_t *pprocess_cnt;
	uint32_t *ptx_cnt;
	uint32_t *ptx_error_cnt;
	uint32_t *pwait_int_cnt;
	uint32_t *prx_pack_cnt;

	dev_err(&video->video.dev, "===== video %d debug info =====\n",
		video->video_index);
	dev_err(&video->video.dev,
		"video tx_count = %lld,video rx_count = %lld, isp rx count = %lld, isp tx count = %lld\n",
		video->tx_buf_count, video->rx_buf_count,
		video->isp->ipc_rx_count, video->isp->ipc_tx_count);
	dev_err(&video->video.dev, "isp kthread status = %d\n",
		video->isp->kthread_status);

	dev_err(&video->video.dev, "~~~~~ debug info end ~~~~~\n");

	prx_cnt = (uint32_t *)(video->isp->ctrl_reg_base);
	prx_error_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x4);
	poverflow_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x8);
	pprocess_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x10);

	ptx_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x20);
	ptx_error_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x24);
	pwait_int_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x28);
	prx_pack_cnt = (uint32_t *)(video->isp->ctrl_reg_base + 0x30);

	dev_err(&video->video.dev,
		"video index = %d, tx buf = %lld, rx buf = %lld\n",
		video->video_index, video->tx_buf_count, video->rx_buf_count);

	dev_err(&video->video.dev,
		"rx_cnt = %d, rx_error = %d, overflow = %d, process = %d, tx = %d, tx_error = %d, wait_int = %d, pack = %d\n",
		*prx_cnt, *prx_error_cnt, *poverflow_cnt, *pprocess_cnt,
		*ptx_cnt, *ptx_error_cnt, *pwait_int_cnt, *prx_pack_cnt);
}

/*******************************************************************************
 * Copyright(c) Black Sesame Technologies Inc., All right reserved.
 *
 * No part of this file may be distributed, duplicated or transmitted in any
 *form or by any means, without prior consent of Black Sesame Technologies Inc.
 *
 * This file is  property. It contains BST's trade secret, proprietary
 * and confidential information.
 *
 *********************************************************************************/

#include "bst_gmwarp_msg.h"
#include "bst_gmwarp_drv.h"
#include <media/v4l2-mem2mem.h>
#include <linux/coreip/bst_vsp_msg.h>

extern struct bst_gmwarp_dev *gmwarp_dev;

static void build_command(struct vsp_msg_data *msg_data, int cmd_minor,
			  int bufid)
{
	memset(msg_data, 0, sizeof(struct vsp_msg_data));
	msg_data->core_id = VSP_CORE_GMWARP;
	strncpy(msg_data->src, "tc00", 4);
	strncpy(msg_data->dst, "vw00", 4);
	msg_data->cmd_type_main = CMD_GWC_DEV;
	msg_data->cmd_type_minor = cmd_minor;
	msg_data->follow_pack_num =
		0; //default follow_pack_num = 0,  one media_command
	msg_data->cmdid = bufid;
	//memcpy(&msg_data->user_cmd_data[0], &paddr[0], sizeof(uint32_t));
}

void vsp_gmwarp_cmd_init_config(uint32_t *index, dma_addr_t *paddr)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_INIT_CONFIG,
		      CMD_GWARP_INIT_CFG_BUFF_ID_0 + *index);

	memcpy(&msg_data.user_cmd_data[0], index, sizeof(uint32_t));
	memcpy(&msg_data.user_cmd_data[1], paddr, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_table_config(uint32_t *index, dma_addr_t *paddr)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_TABLE_CONFIG,
		      CMD_GWARP_TBL_CFG_BUFF_ID_0 + *index);

	memcpy(&msg_data.user_cmd_data[0], index, sizeof(uint32_t));
	memcpy(&msg_data.user_cmd_data[1], paddr, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_scaler_config(uint32_t *index, dma_addr_t *paddr)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_SCALER_CONFIG,
		      CMD_GWARP_SCALER_CFG_BUFF_ID_0 + *index);

	memcpy(&msg_data.user_cmd_data[0], index, sizeof(uint32_t));
	memcpy(&msg_data.user_cmd_data[1], paddr, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_pitch_config(uint32_t *index, uint32_t paddr[])
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_PITCH_CONFIG,
		      CMD_GWARP_PITCH_CFG_BUFF_ID_0 + *index);

	memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 4);

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_crop_config(uint32_t *index, dma_addr_t *paddr)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_CROP_CONFIG,
		      CMD_GWARP_CROP_CFG_BUFF_ID_0 + *index);

	memcpy(&msg_data.user_cmd_data[0], index, sizeof(uint32_t));
	memcpy(&msg_data.user_cmd_data[1], paddr, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_selfcheck_start(uint32_t *index, uint32_t paddr[])
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_SELFCHECK_START,
		      CMD_GWARP_SELFCHECK_START_BUFF_ID_0 + *index);

	strncpy(msg_data.dst, "vs00", 4); //selfcheck mode dst use "vs00"
	memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 4);
	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_selfcheck_stop(uint32_t *index)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_SELFCHECK_STOP,
		      CMD_GWARP_SELFCHECK_STOP_BUFF_ID_0 + *index);

	strncpy(msg_data.dst, "vs00", 4); //selfcheck mode dst use "vs00"
	memcpy(&msg_data.user_cmd_data[0], index, sizeof(uint32_t));
	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_frame_start(uint32_t paddr[], int plane_num,
				uint32_t channel_id, uint32_t buff_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_FRAME_START,
		      CMD_GWARP_FRM_START_BUFF_ID_0_0 + channel_id);
	msg_data.follow_pack_num =
		plane_num -
		1; //follow_pack_num = channel num - 1, channel num media_command

	memcpy(&msg_data.user_cmd_data[0], paddr,
	       sizeof(uint32_t) * plane_num * 4);

	msg_transfer(&msg_data);
}

void vsp_gmwarp_cmd_frame_start_plus(uint32_t paddr[], uint32_t channel_id,
				     uint32_t buff_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_GWARP_FRAME_START_PLUS,
		      CMD_GWARP_FRM_START_BUFF_ID_0_0 + channel_id);
	msg_data.follow_pack_num = 1; //follow pack num for media_command

	memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 2 * 4);

	msg_transfer(&msg_data);
}
void gmwarp_ipc_msg_recv(struct media_command *param)
{
	unsigned long flags;
	struct gmwarp_ctx *ctx;
	struct bst_gmwarp_video *gmwarp_video;
	struct gmwarp_cache_msg *cache_msg;
	struct vb2_v4l2_buffer *src, *dst;

	if (param->cmd_hdr.hdr_info.cmd_type_minor == CMD_GWARP_FRAME_DONE) {
		if ((param->user_cmd_data[0] >= 0) &&
		    (param->user_cmd_data[0] < GWARP_MAX_CHANNEL_NUM)) {
			//mutex_lock(&gmwarp_dev->mutex);
			gmwarp_video =
				&gmwarp_dev
					 ->gmwarp_video[param->user_cmd_data[0]];
			ctx = gmwarp_video->ctx;
			spin_lock_irqsave(&gmwarp_video->irqlock, flags);
			//mutex_unlock(&bst_gmwarp_dev->mutex);
			//mutex_lock(&gmwarp_video->lock);
			if (gmwarp_video->state == GWARP_CTX_STATE_STOPPED) {
				src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
				dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

				dst->timecode = src->timecode;
				dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
				dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
				dst->flags |= src->flags &
					      V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

				v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
				v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
				v4l2_m2m_job_finish(gmwarp_video->m2m_dev,
						    ctx->fh.m2m_ctx);
				spin_unlock_irqrestore(&gmwarp_video->irqlock,
						       flags);
			} else if (gmwarp_video->state ==
				   GWARP_CTX_STATE_RUNNING) {
				//spin_lock_irqsave(&ctx->cache_queue_lock, flags);
				if (!list_empty(&ctx->cache_list)) {
					cache_msg = list_first_entry(
						&ctx->cache_list,
						struct gmwarp_cache_msg, node);
					list_del(&cache_msg->node);
				} else {
					pr_info("gwarp No msg_cache buffer\n");
					return;
				}

				//spin_unlock_irqrestore(&ctx->cache_queue_lock, flags);
				queue_work(ctx->msg_workqueue,
					   &cache_msg->msg_work);
				//mutex_unlock(&gmwarp_video->lock);
				spin_unlock_irqrestore(&gmwarp_video->irqlock,
						       flags);
			} else {
				//selfcheck mode no something to do
				spin_unlock_irqrestore(&gmwarp_video->irqlock,
						       flags);
			}
		} else {
			pr_info("FW ack gwarp channel id error");
			WARN_ON(!((param->user_cmd_data[0] >= 0) &&
				  (param->user_cmd_data[0] <
				   GWARP_MAX_CHANNEL_NUM)));
		}
	}
}

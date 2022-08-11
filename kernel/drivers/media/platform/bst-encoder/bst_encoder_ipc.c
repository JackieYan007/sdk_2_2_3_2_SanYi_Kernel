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


#include "bst_encoder_ipc.h"
#include "bst_encoder_drv.h"
#include <media/v4l2-mem2mem.h>
#include <linux/delay.h>

extern struct bst_enc_dev *benc_dev;

#define to_encode_buffer(vbuf) container_of(vbuf, struct encode_buffer, vb)

static void build_command(struct vsp_msg_data *msg_data, int cmd_minor,
			  int bufid)
{
	msg_data->core_id = VSP_CORE_ENC;
	strncpy(msg_data->src, "tc00", 4);
	strncpy(msg_data->dst, "ve00", 4);
	msg_data->cmd_type_main = CMD_H264ENC_DEV;
	msg_data->cmd_type_minor = cmd_minor;
	msg_data->follow_pack_num = 0;
	msg_data->cmdid = bufid;
	//memcpy(&msg_data->user_cmd_data[0],"ve00",4);
}

void vsp_encode_cmd_start(void)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_START, CMD_ENCODER_START_BUFF_ID);

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_close(void)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_CLOSE, CMD_ENCODER_CLOSE_BUFF_ID);

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_frame_start(uint32_t paddr[], uint32_t channel_id, uint32_t buff_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_FRAME_START,
		      CMD_ENCODER_FRAME_START_BUFF_ID_0_0 + channel_id);

    memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 4);
	msg_transfer(&msg_data);
}

void vsp_encode_cmd_frame_stop(uint32_t channel_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_FRAME_STOP, CMD_ENCODER_FRAME_STOP_BUFF_ID_0 + channel_id);

    msg_data.user_cmd_data[0] = channel_id;

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_bitrate_cfg(uint32_t *param, uint32_t channel_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_BITRATE_CFG,
		      CMD_ENCODER_BITRATE_CFG_BUFF_ID_0 + channel_id);

    msg_data.user_cmd_data[0] = channel_id;
	memcpy(&msg_data.user_cmd_data[1], param, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_gop_cfg(uint32_t *param, uint32_t channel_id)
{
	struct vsp_msg_data msg_data;

	build_command(&msg_data, CMD_ENCODER_GOP_CFG,
		      CMD_ENCODER_GOP_CFG_BUFF_ID_0 + channel_id);

    
    msg_data.user_cmd_data[0] = channel_id;
	memcpy(&msg_data.user_cmd_data[1], param, sizeof(uint32_t));

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_qp_cfg(uint32_t *param, uint32_t channel_id)
{
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_QP_CFG,
		      CMD_ENCODER_QP_CFG_BUFF_ID_0 + channel_id);

    msg_data.user_cmd_data[0] = channel_id;
	memcpy(&msg_data.user_cmd_data[1], param, sizeof(uint32_t));

	msg_transfer(&msg_data);

}

void vsp_encode_cmd_res_cfg(uint32_t width, uint32_t height, uint32_t channel_id)
{ 
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_RES_CFG,
		      CMD_ENCODER_RES_CFG_BUFF_ID_0 + channel_id);

    msg_data.user_cmd_data[0] = channel_id;
    msg_data.user_cmd_data[1] = width;
    msg_data.user_cmd_data[2] = height;

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_framerate_cfg(uint32_t framerate, uint32_t channel_id)
{
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_FRAMERATE_CFG,
		      CMD_ENCODER_FRAMERATE_CFG_BUFF_ID_0 + channel_id);
    msg_data.user_cmd_data[0] = channel_id;
    msg_data.user_cmd_data[1] = framerate;
    
	msg_transfer(&msg_data);
}

void vsp_encode_cmd_sps_pps_resend(uint32_t paddr, uint32_t channel_id)
{
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_SPS_PPS_RESEND,
		      CMD_ENCODER_SPS_PPS_RESEND_BUFF_ID_0 + channel_id);
    msg_data.user_cmd_data[0] = channel_id;
    msg_data.user_cmd_data[1] = paddr;
    
	msg_transfer(&msg_data);
}

void vsp_encode_cmd_rc_alg_cfg(uint32_t rc_alg_enable, uint32_t channel_id)
{
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_RC_ALG_CFG,
		      CMD_ENCODER_RC_ALG_CFG_BUFF_ID_0 + channel_id);
    msg_data.user_cmd_data[0] = channel_id;
    msg_data.user_cmd_data[1] = rc_alg_enable;
    
	msg_transfer(&msg_data);
}

void vsp_encode_cmd_ref_cfg(uint32_t ref_num, uint32_t channel_id)
{
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_REF_CFG,
		      CMD_ENCODER_REF_CFG_BUFF_ID_0 + channel_id);
    msg_data.user_cmd_data[0] = channel_id;
    msg_data.user_cmd_data[1] = ref_num;
    
	msg_transfer(&msg_data);
}
void vsp_encode_cmd_selfcheck_start(uint32_t paddr[])
{ 
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_SELFCHECK_START,
		      CMD_ENCODER_SELFCHECK_START_BUFF_ID);

	strncpy(msg_data.dst, "vs00", 4);              //selfcheck mode dst use "vs00"
	memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 4);

	msg_transfer(&msg_data);
}

void vsp_encode_cmd_selfcheck_stop(uint32_t paddr[])
{ 
    struct vsp_msg_data msg_data;
	build_command(&msg_data, CMD_ENCODER_SELFCHECK_STOP,
		      CMD_ENCODER_SELFCHECK_STOP_BUFF_ID);

	strncpy(msg_data.dst, "vs00", 4);              //selfcheck mode dst use "vs00"
	memcpy(&msg_data.user_cmd_data[0], paddr, sizeof(uint32_t) * 4);

	msg_transfer(&msg_data);
}

void encode_ipc_msg_recv(struct media_command *param)
{
	unsigned long flags;
	struct bst_enc_ctx *ctx;
    struct bst_enc_video *benc_video;
    struct enc_cache_msg *cache_msg;
    struct vb2_v4l2_buffer *src, *dst;
    
	if (param->cmd_hdr.hdr_info.cmd_type_minor == CMD_ENCODER_FRAME_DONE) {

        benc_video = &benc_dev->enc_video[param->user_cmd_data[0]];
        ctx = benc_video->enc_ctx;
        spin_lock_irqsave(&benc_video->irqlock, flags);

        if(benc_video->state == ENC_STREAMING_OFF)
        {
            WARN_ON(!ctx);

            src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
            dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

            WARN_ON(!src);
            WARN_ON(!dst);

            dst->timecode = src->timecode;
            dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
            dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
            dst->flags |= src->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

            dst->vb2_buf.planes[0].bytesused = param->user_cmd_data[2];
            v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
            v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
            v4l2_m2m_job_finish(benc_video->m2m_dev, ctx->fh.m2m_ctx);
            spin_unlock_irqrestore(&benc_video->irqlock, flags);

        } else if (benc_video->state == ENC_STREAMING_ON){
    
            if(!list_empty(&ctx->cache_list))
            {
                cache_msg = list_first_entry(&ctx->cache_list, struct enc_cache_msg, node);
                cache_msg->bytesize = param->user_cmd_data[2];
                list_del(&cache_msg->node);
            }
            else
            {
                dev_err(benc_dev->dev, "No msg_cache buffer to use\n");
                return ;
            }

            queue_work(ctx->msg_workqueue, &cache_msg->msg_work);
            spin_unlock_irqrestore(&benc_video->irqlock, flags);

            
        }
        else
        {
            spin_unlock_irqrestore(&benc_video->irqlock, flags);
        }
    }
    else if(param->cmd_hdr.hdr_info.cmd_type_minor == CMD_ENCODER_SPS_PPS_DONE)
    {
        benc_video = &benc_dev->enc_video[param->user_cmd_data[0]];
        benc_video->sps_pps_flags = 1;
        wake_up(&benc_video->sps_pps_queue); 
    }
}

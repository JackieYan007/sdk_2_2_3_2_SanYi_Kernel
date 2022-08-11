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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/ipc_interface.h>

#include <linux/delay.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <asm/cacheflush.h>
#include <linux/pagemap.h>


#include "bst_encoder_drv.h"
#include "bst_encoder_ipc.h"
#include "bst_encoder_ioctl.h"

struct bst_enc_dev *benc_dev;

static inline struct bst_enc_ctx *file_to_ctx(struct file *file)
{
    return container_of(file->private_data, struct bst_enc_ctx, fh);
}

static const struct benc_video_format benc_formats[] = {
    {
        .pixfmt = V4L2_PIX_FMT_NV12,
        .num_planes = 2,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
        .size = {0},
    },
    {
        .pixfmt = V4L2_PIX_FMT_H264,
        .num_planes = 1,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .size = {0},
    },

};
struct benc_video_format *bst_enc_get_format(struct bst_enc_ctx *ctx, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->fmt_out;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->fmt_cap;
	default:
		return ERR_PTR(-EINVAL);
	}
    
}


static int bst_enc_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers, 
            unsigned int *nplanes, unsigned int sizes[],
            struct device *alloc_devs[])
{

    unsigned int buffer_num;
    struct bst_enc_ctx *ctx = vb2_get_drv_priv(vq);
    struct bst_enc_video *benc_video = ctx->enc_video;
    struct benc_video_format *format = bst_enc_get_format(ctx, vq->type);

    if(IS_ERR(format))
        return PTR_ERR(format);

    if(format->num_planes > 1)
    {
        alloc_devs[0] = benc_video->enc_dev->dev;
        sizes[0] = format->size[0];
        alloc_devs[1] = benc_video->enc_dev->dev;
        sizes[1] = format->size[1];
    }
    else
    {
        alloc_devs[0] = benc_video->enc_dev->dev;
        sizes[0] = format->size[0];
    }

    *nplanes = format->num_planes;
    if(*nbuffers < MIX_BUFFER_NUM)
        *nbuffers = MIX_BUFFER_NUM;

    if(*nbuffers > MAX_BUFFER_NUM)
        return -EINVAL;
    
    buffer_num = MAX_BUFFER_NUM;
    *nbuffers = min(*nbuffers, buffer_num);

    return 0; 
}

static int bst_enc_buf_prepare(struct vb2_buffer *vb)
{
    struct bst_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
    struct benc_video_format *format = bst_enc_get_format(ctx, vb->vb2_queue->type);
    int nplane;

#ifdef ENC_MMAP_CACHE
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct v4l2_m2m_buffer *buffer = container_of(vbuf, struct v4l2_m2m_buffer, vb);

    struct bst_encoder_buffer *enc_buffer;
    enc_buffer = container_of(buffer, struct bst_encoder_buffer, m2m_buf);
#endif

    if(IS_ERR(format))
        return PTR_ERR(format);


    for(nplane = 0; nplane < format->num_planes; nplane++) {
        if(vb2_plane_size(vb, nplane) < format->size[nplane])
            return -EINVAL;
        vb2_set_plane_payload(vb, nplane, format->size[nplane]);

#ifdef ENC_MMAP_CACHE
        if (enc_buffer->mmap_uaddr[nplane]) {
            __dma_flush_area(enc_buffer->mmap_uaddr[nplane],
                    enc_buffer->plane_size[nplane]);
        }
#endif
    }

    return 0;
}

static void bst_enc_buf_queue(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct bst_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

    v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);

}

static int bst_enc_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    return 0;
}

static void bst_enc_stop_streaming(struct vb2_queue *vq)
{
    struct bst_enc_ctx *ctx = vb2_get_drv_priv(vq);
    struct bst_enc_video *benc_video = ctx->enc_video;
    struct vb2_v4l2_buffer *vbuf;

    

    for(;;) {
        if(V4L2_TYPE_IS_OUTPUT(vq->type))
            vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
        else
            vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
        if(!vbuf)
            break;
        v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
    }
    vsp_encode_cmd_frame_stop(benc_video->channel_id);
}

const struct vb2_ops bst_enc_vb2_ops = {
	.queue_setup = bst_enc_queue_setup,
	.buf_prepare = bst_enc_buf_prepare,
	.buf_queue = bst_enc_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = bst_enc_start_streaming,
	.stop_streaming = bst_enc_stop_streaming,
};


void enc_free_buf_handler(struct vb2_buffer *vbs, struct vb2_buffer *vbd, uint32_t buff_id, uint32_t channel_id)
{
    uint32_t paddr[4] = {0};

    paddr[0] = channel_id;
    paddr[1] = vb2_dma_contig_plane_dma_addr(vbs, 0);
    paddr[2] = vb2_dma_contig_plane_dma_addr(vbs, 1);
    paddr[3] = vb2_dma_contig_plane_dma_addr(vbd, 0);
    
    vsp_encode_cmd_frame_start(paddr, channel_id, buff_id);

}

static void bst_enc_device_run(void *prv)
{
    struct bst_enc_ctx *ctx;
    struct vb2_v4l2_buffer *src, *dst;
    unsigned long flags;
    struct bst_enc_video *benc_video;

    ctx = prv;
    benc_video = ctx->enc_video;

    
    src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
    dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

    if(ctx->buff_id == ENC_WORK_CACHE_NUM)
    {
        ctx->buff_id = 0;
    }
    enc_free_buf_handler(&src->vb2_buf, &dst->vb2_buf, ctx->buff_id, ctx->enc_video->channel_id);
    ctx->buff_id++;

    spin_lock_irqsave(&benc_video->irqlock, flags);
    benc_video->state = ENC_STREAMING_ON;
    spin_unlock_irqrestore(&benc_video->irqlock, flags);

    
}

static void bst_enc_job_abort(void *prv)
{
    
}

static struct v4l2_m2m_ops bst_enc_m2m_ops = {
    .device_run = bst_enc_device_run,
    .job_abort = bst_enc_job_abort,
};

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
    struct bst_enc_ctx *ctx = priv;
    int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &bst_enc_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct bst_encoder_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->enc_video->lock;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &bst_enc_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct bst_encoder_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->enc_video->lock;

	return vb2_queue_init(dst_vq);
    

}

void encoder_ipc_msg_handler(struct work_struct *work)
{
    struct enc_cache_msg *cache_msg = (struct enc_cache_msg *)container_of(work, struct enc_cache_msg, msg_work);
    struct bst_enc_ctx *ctx = cache_msg->enc_ctx;
    struct bst_enc_video *benc_video = ctx->enc_video;

    struct vb2_v4l2_buffer *src, *dst;
    unsigned long flags;
    
    WARN_ON(!ctx);

    src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
    dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

    WARN_ON(!src);
    WARN_ON(!dst);

    dst->timecode = src->timecode;
    dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
    dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
    dst->flags |= src->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

    dst->vb2_buf.planes[0].bytesused = cache_msg->bytesize;  //size for h264 data

    v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
    v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
    v4l2_m2m_job_finish(benc_video->m2m_dev, ctx->fh.m2m_ctx);
    
    spin_lock_irqsave(&benc_video->irqlock, flags);
    list_add_tail(&cache_msg->node, &ctx->cache_list);
    spin_unlock_irqrestore(&benc_video->irqlock, flags);

}

static int bst_enc_open(struct file *file)
{
    struct bst_enc_video *benc_video = video_drvdata(file);
    struct bst_enc_ctx *ctx = NULL;
    int ret = 0;
    int i;
    char workqueue_name[64];

    mutex_lock(&benc_video->lock);        
    if( benc_video->enc_ctx != NULL)
        goto open_err;
    mutex_unlock(&benc_video->lock);

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->enc_video = benc_video;
    benc_video->enc_ctx = ctx;

    if (mutex_lock_interruptible(&ctx->enc_video->lock)) {
        kfree(ctx);
        return -ERESTARTSYS;
    }

    mutex_init(&ctx->msg_lock);
    mutex_init(&ctx->ctx_lock);
    spin_lock_init(&ctx->cache_queue_lock);
    INIT_LIST_HEAD(&ctx->cache_list);

    snprintf(workqueue_name, 64, "%s%d", "bst_enc_workqueue", benc_video->channel_id);
    ctx->msg_workqueue = create_singlethread_workqueue(workqueue_name);

    for(i = 0; i < ENC_WORK_CACHE_NUM; i++)
    {
        INIT_WORK(&ctx->cache_msg[i].msg_work, encoder_ipc_msg_handler);
        list_add_tail(&ctx->cache_msg[i].node, &ctx->cache_list);
        ctx->cache_msg[i].enc_ctx = ctx;
    }

    ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(benc_video->m2m_dev, ctx, &queue_init);
    if (IS_ERR(ctx->fh.m2m_ctx)) {
        ret = PTR_ERR(ctx->fh.m2m_ctx);
        kfree(ctx);
        goto open_unlock;
    }
    v4l2_fh_init(&ctx->fh, video_devdata(file));
    file->private_data = &ctx->fh;
    v4l2_fh_add(&ctx->fh);

open_unlock:
    mutex_unlock(&benc_video->lock);
    return ret;
open_err:
    mutex_unlock(&benc_video->lock);
    return -EBUSY;
}

static int bst_enc_release(struct file *file)
{
    unsigned long flags;

    struct bst_enc_ctx *ctx = file_to_ctx(file);
    struct bst_enc_video *benc_video = ctx->enc_video;

    spin_lock_irqsave(&benc_video->irqlock, flags);
    benc_video->state = ENC_STREAMING_OFF;
    spin_unlock_irqrestore(&benc_video->irqlock, flags);

    mutex_lock(&benc_video->lock);
    destroy_workqueue(ctx->msg_workqueue);
    v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
    v4l2_fh_del(&ctx->fh);
    v4l2_fh_exit(&ctx->fh);
    kfree(ctx);
    benc_video->enc_ctx = NULL;
    mutex_unlock(&benc_video->lock);

    return 0;
}

#define ENC_DST_QUEUE_OFF_BASE (1 << 30)
static int bst_encode_m2m_fop_mmap(struct file *file, struct vm_area_struct *vma)
{
#ifdef ENC_MMAP_CACHE
    unsigned int buffer_index;
    unsigned int plane_index;
    int ret;
    struct vb2_buffer *vb;
    struct vb2_queue *q;
    struct v4l2_m2m_buffer *buffer;
    struct bst_encoder_buffer *enc_buffer;
    struct vb2_v4l2_buffer *vbuf;

    struct v4l2_fh *fh = file->private_data;
    unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

    if(offset < ENC_DST_QUEUE_OFF_BASE) {
        q = v4l2_m2m_get_src_vq(fh->m2m_ctx);
    } else {
        q = v4l2_m2m_get_dst_vq(fh->m2m_ctx);
        vma->vm_pgoff -= (ENC_DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
    }

    ret = vb2_swiotlb_mmap_cache(q, vma, &buffer_index, &plane_index);
    vb = q->bufs[buffer_index];
    vbuf = to_vb2_v4l2_buffer(vb);
    buffer = container_of(vbuf, struct v4l2_m2m_buffer, vb);
    enc_buffer = container_of(buffer, struct bst_encoder_buffer, m2m_buf);
    enc_buffer->mmap_uaddr[plane_index] = (void *)(vma->vm_start);
    enc_buffer->plane_size[plane_index] = vb->planes[plane_index].length;


    return ret;
#else
    struct v4l2_fh *fh = file->private_data;
    return v4l2_m2m_mmap(file, fh->m2m_ctx, vma);
#endif
}

static const struct v4l2_file_operations bst_enc_fops = {
    .owner = THIS_MODULE,
    .open = bst_enc_open,
    .release = bst_enc_release,
    .poll = v4l2_m2m_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    //.mmap = v4l2_m2m_fop_mmap,
    .mmap = bst_encode_m2m_fop_mmap,
};

static int bst_enc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "bst-encoder", sizeof(cap->driver));
	strlcpy(cap->card, "Bst video encoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:bst-encoder", sizeof(cap->bus_info));

	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;             //need nv12 and y data, use mplane   
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int bst_enc_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
    unsigned int size = ARRAY_SIZE(benc_formats);
    if(f->index >= size)
        return -EINVAL;

    if(benc_formats[f->index].type == f->type)
        f->pixelformat = benc_formats[f->index].pixfmt;
    
    return 0;
}

static int bst_enc_try_fmt(struct file *file, void *fh, struct v4l2_format *format)
{
    int plane;
    int i;

    struct v4l2_pix_format_mplane *pix = &format->fmt.pix_mp;
    
    if((pix->field != V4L2_FIELD_NONE) && (pix->field != V4L2_FIELD_INTERLACED))
        pix->field = V4L2_FIELD_NONE;

    if(format->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE && format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        return -EINVAL;

    for(i = 0; i < ARRAY_SIZE(benc_formats); i++)
    {
        if(pix->pixelformat == benc_formats[i].pixfmt) {
            plane = benc_formats[i].num_planes;
            break;
        }
    }
    if(i == ARRAY_SIZE(benc_formats))
        return -EINVAL;

    if(plane > 1)
    {
        pix->plane_fmt[0].bytesperline = pix->width;                                                    //width for NV12
        pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline * pix->height * 3 / 2;             //sizeimage for NV12
        pix->plane_fmt[1].bytesperline = pix->width / 4;                                                // 1/4 for Gray width
        pix->plane_fmt[1].sizeimage = pix->plane_fmt[1].bytesperline * pix->height / 4;                 // 1/16 for Gray sizeimage
    }
    else
    {
        pix->plane_fmt[0].bytesperline = pix->width;
        pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline * pix->height * 3 / 2;              //sizeimage for H264
    }

    return 0;

}


static int bst_enc_g_fmt(struct file *file, void *fh, struct v4l2_format *format)
{
    struct v4l2_pix_format_mplane *pix = &format->fmt.pix_mp;
    struct bst_enc_ctx *ctx = file_to_ctx(file);
    const struct benc_video_format *fmt;

    if(format->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        fmt = &ctx->fmt_cap;
    else if(format->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
        fmt = &ctx->fmt_out;
    else
        return -EINVAL;
    pix->width = fmt->width;
    pix->height = fmt->height;
    pix->pixelformat = fmt->pixfmt;

    return 0;
}

static int bst_enc_s_fmt(struct file *file, void *fh, struct v4l2_format *format)
{
    int ret = 0;
    struct bst_enc_ctx *ctx = file_to_ctx(file);
    struct benc_video_format *fmt;
    struct v4l2_pix_format_mplane *pix = &format->fmt.pix_mp;


    //load fw
    mutex_lock(&benc_dev->lock);
    if(!benc_dev->fw_flag)
    {
        vsp_fw_boot_done();
        benc_dev->fw_flag = 1;
    }
    mutex_unlock(&benc_dev->lock);



    ret = bst_enc_try_fmt(file, fh, format);
    if(ret)
        return ret;
    if(format->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        fmt = &ctx->fmt_cap;
    else if(format->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
        fmt = &ctx->fmt_out;
    else
        return -EINVAL;

    fmt->width = pix->width;
    fmt->height = pix->height;
    fmt->pixfmt = pix->pixelformat;
    fmt->num_planes = pix->num_planes;
    fmt->type = format->type;
    if(fmt->num_planes > 1)
    {
        fmt->size[0] = pix->plane_fmt[0].sizeimage;
        fmt->size[1] = pix->plane_fmt[1].sizeimage;
    }
    else
    {
        //set resolution 

        vsp_encode_cmd_res_cfg(fmt->width, fmt->height, ctx->enc_video->channel_id);

        fmt->size[0] = pix->plane_fmt[0].sizeimage;
    }

     

    return 0;
}

static int bst_enc_g_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *ctrls)
{
    return 0;
}

static int bst_enc_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *ctrls)
{
    struct bst_enc_ctx *ctx = file_to_ctx(file);
    struct v4l2_ext_control *ctrl;
    int i;

    if(ctrls->which == V4L2_CTRL_WHICH_DEF_VAL)
        return -EINVAL;
    
    if(ctrls->count != 1)
        return -EINVAL;

    for(i = 0; i < ctrls->count; i++)
    {
        ctrl = ctrls->controls + i;
        if(ctrl->id == V4L2_CID_MPEG_VIDEO_BITRATE)
        {
            ctx->enc_video->bitrate = ctrl->value;
            vsp_encode_cmd_bitrate_cfg(&ctx->enc_video->bitrate, ctx->enc_video->channel_id);
        }
        else if(ctrl->id == V4L2_CID_MPEG_VIDEO_GOP_SIZE)
        {
            ctx->enc_video->gop = ctrl->value;
            vsp_encode_cmd_gop_cfg(&ctx->enc_video->gop, ctx->enc_video->channel_id);
        }
        else if(ctrl->id == V4L2_CID_MPEG_VIDEO_H264_MAX_QP)
        {
            if(ctrl->value >= 0 && ctrl->value <= 51)
            {
                ctx->enc_video->qp = ctrl->value;
                vsp_encode_cmd_qp_cfg(&ctx->enc_video->qp, ctx->enc_video->channel_id);
            }
            else
                return -EINVAL;
        }
    }
    return 0;
}

static int bst_enc_try_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *ctrls)
{
    return 0;
}

long bst_enc_vidioc_default(struct file *file, void *fh, bool valid_prio, 
                unsigned int cmd, void *arg)
{
    int i, ret;
    uint32_t index;
    uint32_t paddr[4];
    struct encoder_parm *e_parm;
    struct selfcheck_parm *s_parm;
    struct bst_enc_ctx *ctx = file_to_ctx(file);

    index = ctx->enc_video->channel_id;
    vsp_fw_boot_done();

    switch (cmd) {
        case ENCODER_CID_SELFCHECK_START :
            s_parm = arg;
            ctx->enc_video->state = ENC_SELFCHECK_MODE;
            
            ctx->enc_video->selfcheck_yuv_mem[ctx->enc_video->selfcheck_flags] = dma_alloc_coherent(ctx->enc_video->enc_dev->dev,
                                s_parm->yuv_size,
                                &ctx->enc_video->selfcheck_yuv_addr[ctx->enc_video->selfcheck_flags], GFP_KERNEL);
            if(!ctx->enc_video->selfcheck_yuv_mem[ctx->enc_video->selfcheck_flags]) {
                return -ENOMEM;
            }
            ret = copy_from_user(ctx->enc_video->selfcheck_yuv_mem[ctx->enc_video->selfcheck_flags], s_parm->yuv_addr, s_parm->yuv_size);
            if(ret) {
                return -EINVAL;
            }

            ctx->enc_video->selfcheck_y_mem[ctx->enc_video->selfcheck_flags] = dma_alloc_coherent(ctx->enc_video->enc_dev->dev,
                                s_parm->y_size,
                                &ctx->enc_video->selfcheck_y_addr[ctx->enc_video->selfcheck_flags], GFP_KERNEL);
            if(!ctx->enc_video->selfcheck_y_mem[ctx->enc_video->selfcheck_flags]) {
                return -ENOMEM;
            }
            ret = copy_from_user(ctx->enc_video->selfcheck_y_mem[ctx->enc_video->selfcheck_flags], s_parm->y_addr, s_parm->y_size);
            if(ret) {
                return -EINVAL;
            }


            ctx->enc_video->selfcheck_h264_mem[ctx->enc_video->selfcheck_flags] = dma_alloc_coherent(ctx->enc_video->enc_dev->dev,
                                s_parm->yuv_size,
                                &ctx->enc_video->selfcheck_h264_addr[ctx->enc_video->selfcheck_flags], GFP_KERNEL);
            if(!ctx->enc_video->selfcheck_h264_mem[ctx->enc_video->selfcheck_flags]) {
                return -ENOMEM;
            }
            paddr[0] = ctx->enc_video->selfcheck_yuv_addr[ctx->enc_video->selfcheck_flags];
            paddr[1] = ctx->enc_video->selfcheck_y_addr[ctx->enc_video->selfcheck_flags];
            paddr[2] = ctx->enc_video->selfcheck_h264_addr[ctx->enc_video->selfcheck_flags];
            paddr[3] = s_parm->framerate;
            
            vsp_encode_cmd_selfcheck_start(paddr);
            ctx->enc_video->selfcheck_flags++;
            break;

        case ENCODER_CID_SELFCHECK_STOP :
            s_parm = arg;
            
            paddr[0] = 0;
            paddr[1] = 0;
            paddr[2] = 0;
            paddr[3] = 0;
            vsp_encode_cmd_selfcheck_stop(paddr);

            for(i=0; i < ctx->enc_video->selfcheck_flags; i++)
            {
                dma_free_coherent(ctx->enc_video->enc_dev->dev,
                    s_parm->yuv_size, ctx->enc_video->selfcheck_yuv_mem[i],
                    ctx->enc_video->selfcheck_yuv_addr[i]);
                dma_free_coherent(ctx->enc_video->enc_dev->dev,
                    s_parm->y_size, ctx->enc_video->selfcheck_y_mem[i],
                    ctx->enc_video->selfcheck_y_addr[i]);
                dma_free_coherent(ctx->enc_video->enc_dev->dev,
                    s_parm->yuv_size, ctx->enc_video->selfcheck_h264_mem[i],
                    ctx->enc_video->selfcheck_h264_addr[i]);
            }
            ctx->enc_video->selfcheck_flags = 0;
            break;
        case ENCODER_CID_FRAMERATE_CFG :
            e_parm = arg;
            if(e_parm->framerate < 0)
                return -EINVAL;
            vsp_encode_cmd_framerate_cfg(e_parm->framerate, index);
            break;
        case ENCODER_CID_RC_ALG_CFG :
            e_parm = arg;
            if(e_parm->ratectrol_enable != 0 && e_parm->ratectrol_enable != 1)
                return -EINVAL;
            vsp_encode_cmd_rc_alg_cfg(e_parm->ratectrol_enable, index);
            break;
        case ENCODER_CID_SPS_PPS_RESEND :
            e_parm = arg;
            ret = get_encode_payload_addr(index, &ctx->enc_video->p_vaddr, &ctx->enc_video->p_phy_addr); 
            if(ret < 0)
            {
                return -EINVAL;
            }
            paddr[0] = ctx->enc_video->p_phy_addr;
            vsp_encode_cmd_sps_pps_resend(paddr[0], index);
            wait_event(ctx->enc_video->sps_pps_queue, ctx->enc_video->sps_pps_flags); //wait fw to fill sps_pps
            ctx->enc_video->sps_pps_flags = 0;
            
            ret = copy_to_user(e_parm->sps_pps_addr, (uint8_t *)ctx->enc_video->p_vaddr, 32);
            if(ret) {
                return -EINVAL;
            }
            break;
        case ENCODER_CID_REF_CFG :
            e_parm = arg;
            if(e_parm->ref_num <= 0)
                return -EINVAL;
            vsp_encode_cmd_ref_cfg(e_parm->ref_num, index);
            break;

        default :
            break;
    }
    return 0;

}

static const struct v4l2_ioctl_ops bst_enc_ioctl_ops = {
	.vidioc_querycap = bst_enc_querycap,
	.vidioc_enum_fmt_vid_cap = bst_enc_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane = bst_enc_try_fmt,
	.vidioc_g_fmt_vid_cap_mplane = bst_enc_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = bst_enc_s_fmt,

	.vidioc_enum_fmt_vid_out = bst_enc_enum_fmt,
	.vidioc_try_fmt_vid_out_mplane = bst_enc_try_fmt,
	.vidioc_g_fmt_vid_out_mplane = bst_enc_g_fmt,
	.vidioc_s_fmt_vid_out_mplane = bst_enc_s_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

    .vidioc_g_ext_ctrls = bst_enc_g_ext_ctrls,
    .vidioc_s_ext_ctrls = bst_enc_s_ext_ctrls,
    .vidioc_try_ext_ctrls = bst_enc_try_ext_ctrls,
    
    .vidioc_default = bst_enc_vidioc_default,
    
};


static int bst_encode_probe(struct platform_device *pdev)
{
	struct bst_enc_dev *enc_dev;
    struct video_device *vfd;
	int ret;
    int i;

	enc_dev = devm_kzalloc(&pdev->dev, sizeof(*enc_dev), GFP_KERNEL);
	if (!enc_dev)
		return -ENOMEM;

    ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
	    dev_err(&pdev->dev,"reserve mem device init failed\n");
        return -ENODEV;
	}
    
    benc_dev = enc_dev;
	enc_dev->dev = &pdev->dev;
	mutex_init(&enc_dev->lock);
	spin_lock_init(&enc_dev->irqlock);

	ret = v4l2_device_register(enc_dev->dev, &enc_dev->v4l2_dev);
	if (ret) {
        dev_err(enc_dev->dev, "Failed to register v4l2-device %d\n", ret);
		goto err_device_init;
	}

	platform_set_drvdata(pdev, enc_dev);

    for(i = 0; i < BST_ENC_CHANNEL_NUM; i++)
    {
        enc_dev->enc_video[i].enc_dev = enc_dev;
        enc_dev->enc_video[i].channel_id = i;

        mutex_init(&enc_dev->enc_video[i].lock);
        spin_lock_init(&enc_dev->enc_video[i].irqlock);
        init_waitqueue_head(&enc_dev->enc_video[i].sps_pps_queue);
        vfd = &enc_dev->enc_video[i].video;
        snprintf(vfd->name, sizeof(vfd->name), "%s-%d", "bst_encoder", i);
        vfd->fops = &bst_enc_fops;
        vfd->ioctl_ops = &bst_enc_ioctl_ops;
        vfd->minor = -1;
        vfd->release = video_device_release;
        vfd->vfl_dir = VFL_DIR_M2M;
        vfd->lock = &enc_dev->enc_video[i].lock;
        vfd->v4l2_dev = &enc_dev->v4l2_dev;
        vfd->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

        video_set_drvdata(vfd, &enc_dev->enc_video[i]);

        enc_dev->enc_video[i].m2m_dev = v4l2_m2m_init(&bst_enc_m2m_ops);
        if(IS_ERR(enc_dev->enc_video[i].m2m_dev)) {
            v4l2_err(&enc_dev->v4l2_dev, "Failed to init m2m device\n");
            ret = PTR_ERR(enc_dev->enc_video[i].m2m_dev);
            goto unreg_video_dev;
        }

        ret = video_register_device(vfd, VFL_TYPE_VIDEO, BST_ENC_DEVICE_NR);
        if(ret)
        {
            v4l2_err(&enc_dev->v4l2_dev, "Failed to register video device\n");
            goto rel_vdev;
        }

        v4l2_info(&enc_dev->v4l2_dev, "Registerd %s as /dev/%s\n", vfd->name, video_device_node_name(vfd));
    }


	vsp_core_callback_register(VSP_CORE_ENC, encode_ipc_msg_recv);

	return 0;

rel_vdev:
    video_device_release(vfd);
unreg_video_dev:
    v4l2_device_unregister(&enc_dev->v4l2_dev);
err_device_init:
    return ret;
}

static int bst_encode_remove(struct platform_device *pdev)
{
    int i;
	struct bst_enc_dev *enc_dev = platform_get_drvdata(pdev);
	for(i = 0; i < BST_ENC_CHANNEL_NUM; i++)
    {
        v4l2_m2m_release(enc_dev->enc_video[i].m2m_dev);
        video_unregister_device(&enc_dev->enc_video[i].video);
    }

    v4l2_device_unregister(&enc_dev->v4l2_dev);
    
    enc_dev = NULL;
	return 0;
}

static const struct of_device_id bst_encode_of_id_table[] = {
	{ .compatible = "bst,bst-encoder" },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, bst_encode_of_id_table);

static struct platform_driver bst_encode_driver = {
    .driver = {
        .name = "bst-encode",
        .of_match_table = bst_encode_of_id_table,
    },
    .probe = bst_encode_probe,
    .remove = bst_encode_remove,
};

static int __init bst_encoder_init(void)
{
	return platform_driver_register(&bst_encode_driver);
}
late_initcall_sync(bst_encoder_init);
//module_platform_driver(bst_encode_driver);

MODULE_AUTHOR("kanghua.li@bst.ai");
MODULE_DESCRIPTION("bst VSP driver for encoder");
MODULE_LICENSE("GPL v2");

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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "bst_gmwarp_drv.h"
#include "bst_gmwarp_msg.h"
#include "bst_gmwarp_ioctl.h"

struct bst_gmwarp_dev *gmwarp_dev;

static void job_abort(void *prv)
{
}

static void device_run(void *prv)
{
	struct gmwarp_ctx *ctx;
	ctx = prv;
	struct bst_gmwarp_video *gmwarp_video = ctx->gmwarp_video;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned long flags;

	//spin_lock_irqsave(&gmwarp->ctrl_lock, flags);

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	//printk("<8> device run gmwarp_video lock\n");
	//mutex_lock(&gmwarp_video->lock);
	if (ctx->buff_id == GWARP_WORK_CACHE_NUM) {
		ctx->buff_id = 0;
	}

	gmwarp_free_buf_handler(&src->vb2_buf, &dst->vb2_buf, ctx->buff_id,
				ctx->gmwarp_video->channel_id, ctx);
	ctx->buff_id++;

	spin_lock_irqsave(&gmwarp_video->irqlock, flags);
	gmwarp_video->state = GWARP_CTX_STATE_RUNNING;
	//mutex_unlock(&gmwarp_video->lock);
	spin_unlock_irqrestore(&gmwarp_video->irqlock, flags);
	//spin_unlock_irqrestore(&gmwarp->ctrl_lock, flags);
}

void gmwarp_ipc_msg_handler(struct work_struct *work)
{
	struct gmwarp_cache_msg *cache_msg =
		(struct gmwarp_cache_msg *)container_of(
			work, struct gmwarp_cache_msg, msg_work);
	struct gmwarp_ctx *ctx = cache_msg->ctx;
	struct bst_gmwarp_video *gmwarp_video = ctx->gmwarp_video;

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
	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(gmwarp_video->m2m_dev, ctx->fh.m2m_ctx);

	spin_lock_irqsave(&gmwarp_video->irqlock, flags);
	//spin_lock_irqsave(&ctx->cache_queue_lock, flags);
	list_add_tail(&cache_msg->node, &ctx->cache_list);
	//spin_unlock_irqrestore(&ctx->cache_queue_lock, flags);
	spin_unlock_irqrestore(&gmwarp_video->irqlock, flags);
}

static struct v4l2_m2m_ops gmwarp_m2m_ops = {
	.device_run = device_run,
	.job_abort = job_abort,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct gmwarp_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &gmwarp_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct bst_gmwarp_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->gmwarp_video->lock;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &gmwarp_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct bst_gmwarp_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->gmwarp_video->lock;

	return vb2_queue_init(dst_vq);
}

struct gmwarp_fmt formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.depth = 12,
		.uv_factor = 4,
		.y_div = 2,
		.x_div = 1,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

struct gmwarp_fmt *gmwarp_fmt_find(struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat)
			return &formats[i];
	}
	return NULL;
}

static struct gmwarp_frame def_frame = {
	.width = DEFAULT_WIDTH,
	.height = DEFAULT_HEIGHT,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.fmt = &formats[0],
};

struct gmwarp_frame *gmwarp_get_frame(struct gmwarp_ctx *ctx,
				      enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->in;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->out;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int gmwarp_open(struct file *file)
{
	struct bst_gmwarp_video *gmwarp_video = video_drvdata(file);
	struct gmwarp_ctx *ctx = NULL;
	int ret = 0;
	int i;
	int flags;
	char workqueue_name[64];

        mutex_lock(&gmwarp_video->lock);
        if (gmwarp_video->ctx != NULL)
                goto open_err;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
        {
                mutex_unlock(&gmwarp_video->lock);
		return -ENOMEM;
        }
	ctx->gmwarp_video = gmwarp_video;
	gmwarp_video->ctx = ctx;

        mutex_unlock(&gmwarp_video->lock);
	/* Set default formats */
	ctx->in = def_frame;
	ctx->out = def_frame;

	if (mutex_lock_interruptible(&ctx->gmwarp_video->lock)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	mutex_init(&ctx->msg_lock);
	mutex_init(&ctx->ctx_lock);
	spin_lock_init(&ctx->cache_queue_lock);
	INIT_LIST_HEAD(&ctx->cache_list);

	//gmwarp->ctx[i] = ctx;
	//workqueue init
	snprintf(workqueue_name, 64, "%s%d", "bst_gmwarp_work_queue",
		 gmwarp_video->channel_id);
	ctx->msg_workqueue = create_singlethread_workqueue(workqueue_name);

	for (i = 0; i < GWARP_WORK_CACHE_NUM; i++) {
		INIT_WORK(&ctx->cache_msg[i].msg_work, gmwarp_ipc_msg_handler);
		list_add_tail(&ctx->cache_msg[i].node, &ctx->cache_list);
		ctx->cache_msg[i].ctx = ctx;
	}

	ctx->fh.m2m_ctx =
		v4l2_m2m_ctx_init(gmwarp_video->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		kfree(ctx);
		goto open_unlock;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

open_unlock:
	mutex_unlock(&gmwarp_video->lock);
	return ret;
open_err:
        mutex_unlock(&gmwarp_video->lock);
	return -EBUSY;
}

static int gmwarp_release(struct file *file)
{
	unsigned long flags;

	struct gmwarp_ctx *ctx =
		container_of(file->private_data, struct gmwarp_ctx, fh);
	struct bst_gmwarp_video *gmwarp_video = ctx->gmwarp_video;

	spin_lock_irqsave(&gmwarp_video->irqlock, flags);
	gmwarp_video->state = GWARP_CTX_STATE_STOPPED;
	spin_unlock_irqrestore(&gmwarp_video->irqlock, flags);

	mutex_lock(&gmwarp_video->lock);
	destroy_workqueue(ctx->msg_workqueue);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
        gmwarp_video->yuv_sep_flags = 0;
	gmwarp_video->ctx = NULL;
	mutex_unlock(&gmwarp_video->lock);

	return 0;
}

#define GMWARP_DST_QUEUE_OFF_BASE (1 << 30)

static int bst_gmwarp_m2m_fop_mmap(struct file *file, struct vm_area_struct *vma)
{
#ifdef GMWARP_MMAP_CACHE
	unsigned int buffer_index;
	unsigned int plane_index;
	int ret;
	struct vb2_buffer *vb;
	struct vb2_queue *q;
	struct v4l2_m2m_buffer *buffer;
	struct bst_gmwarp_buffer *gmwarp_buffer;
	struct vb2_v4l2_buffer *vbuf;

	struct v4l2_fh *fh = file->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < GMWARP_DST_QUEUE_OFF_BASE) {
		q = v4l2_m2m_get_src_vq(fh->m2m_ctx);
	} else {
		q = v4l2_m2m_get_dst_vq(fh->m2m_ctx);
		vma->vm_pgoff -= (GMWARP_DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
	}

	ret = vb2_swiotlb_mmap_cache(q, vma, &buffer_index, &plane_index);
	vb = q->bufs[buffer_index];
	vbuf = to_vb2_v4l2_buffer(vb);
	buffer = container_of(vbuf, struct v4l2_m2m_buffer, vb);
	gmwarp_buffer = container_of(buffer, struct bst_gmwarp_buffer, m2m_buf);
	gmwarp_buffer->mmap_uaddr[plane_index] = (void *)(vma->vm_start);
	gmwarp_buffer->plane_size[plane_index] = vb->planes[plane_index].length;

	return ret;
#else
	struct v4l2_fh *fh = file->private_data;
	return v4l2_m2m_mmap(file, fh->m2m_ctx, vma);
#endif
}

static const struct v4l2_file_operations gmwarp_fops = {
	.owner = THIS_MODULE,
	.open = gmwarp_open,
	.release = gmwarp_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	//.mmap = v4l2_m2m_fop_mmap,
	.mmap = bst_gmwarp_m2m_fop_mmap,
};

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, GMWARP_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "bst gmwarp", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:gmwarp", sizeof(cap->bus_info));

	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *prv, struct v4l2_fmtdesc *f)
{
	struct gmwarp_fmt *fmt;

	fmt = &formats[0];
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int vidioc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct gmwarp_fmt *fmt;
	int plane;

	fmt = gmwarp_fmt_find(f);
	if (!fmt) {
		fmt = &formats[0];
		f->fmt.pix.pixelformat = fmt->fourcc;
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;

	if (f->fmt.pix.width > MAX_WIDTH)
		f->fmt.pix.width = MAX_WIDTH;

	if (f->fmt.pix.height > MAX_HEIGHT)
		f->fmt.pix.height = MAX_HEIGHT;

	if (f->fmt.pix.width < MIN_WIDTH)
		f->fmt.pix.width = MIN_WIDTH;

	if (f->fmt.pix.height < MIN_HEIGHT)
		f->fmt.pix.height = MIN_HEIGHT;

	for (plane = 0; plane < f->fmt.pix_mp.num_planes; plane++) {
		f->fmt.pix_mp.plane_fmt[plane].bytesperline =
			f->fmt.pix_mp.width * 3 / 2;
		f->fmt.pix_mp.plane_fmt[plane].sizeimage =
			f->fmt.pix_mp.plane_fmt[plane].bytesperline *
			f->fmt.pix_mp.height;
	}

	return 0;
}

static int vidioc_g_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct gmwarp_ctx *ctx = prv;
	struct vb2_queue *vq;
	struct gmwarp_frame *frm;
	int plane;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	frm = gmwarp_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	f->fmt.pix_mp.width = frm->width;
	f->fmt.pix_mp.height = frm->height;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.pixelformat = frm->fmt->fourcc;
	f->fmt.pix_mp.colorspace = frm->colorspace;
	f->fmt.pix_mp.quantization = frm->quantization;

	for (plane = 0; plane < frm->num_planes; plane++) {
		f->fmt.pix_mp.plane_fmt[plane].bytesperline =
			frm->stride[plane];
		f->fmt.pix_mp.plane_fmt[plane].sizeimage = frm->size[plane];
	}

	return 0;
}

static int vidioc_s_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct gmwarp_ctx *ctx = prv;
	struct vb2_queue *vq;
	struct gmwarp_frame *frm;
	struct gmwarp_fmt *fmt;
	int plane;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		//v4l2_err(&gmwarp->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}

	frm = gmwarp_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	fmt = gmwarp_fmt_find(f);
	if (!fmt)
		return -EINVAL;

	frm->width = f->fmt.pix_mp.width;
	frm->height = f->fmt.pix_mp.height;
	frm->fmt = fmt;
	frm->num_planes = f->fmt.pix_mp.num_planes;
	frm->colorspace = f->fmt.pix_mp.colorspace;
	frm->quantization = f->fmt.pix_mp.colorspace;

	for (plane = 0; plane < f->fmt.pix_mp.num_planes; plane++) {
		f->fmt.pix_mp.plane_fmt[plane].bytesperline =
			f->fmt.pix_mp.width * 3 / 2;
		f->fmt.pix_mp.plane_fmt[plane].sizeimage =
			f->fmt.pix_mp.plane_fmt[plane].bytesperline *
			f->fmt.pix_mp.height;
		frm->stride[plane] =
			f->fmt.pix_mp.plane_fmt[plane].bytesperline;
		frm->size[plane] = f->fmt.pix_mp.plane_fmt[plane].sizeimage;
	}

	return 0;
}

long bst_gmwarp_vidioc_default(struct file *file, void *fh, bool valid_prio,
			       unsigned int cmd, void *arg)
{
	struct gmwarp_ctx *ctx;
	uint32_t index;
	struct file_parm *f_parm;
	struct selfcheck_parm *s_parm;
	struct pitch_parm *p_parm;
	struct yuv_parm *sep_parm;
	uint32_t paddr[4];

	long ret;
	ctx = container_of(file->private_data, struct gmwarp_ctx, fh);

	index = ctx->gmwarp_video->channel_id;
	vsp_fw_boot_done();

	switch (cmd) {
	case GWARP_CID_INIT_CONFIG:
		f_parm = arg;
		if (!ctx->gmwarp_video->init_fw_flags) {
			ctx->gmwarp_video->vir_init_mem = dma_alloc_coherent(
				ctx->gmwarp_video->gmwarp_dev->dev,
				GWARP_INIT_MEM_SIZE,
				&ctx->gmwarp_video->init_addr, GFP_KERNEL);
			if (!ctx->gmwarp_video->vir_init_mem) {
				return -ENOMEM;
			}
			ctx->gmwarp_video->init_fw_flags = 1;
		}

		ret = copy_from_user(ctx->gmwarp_video->vir_init_mem,
				     f_parm->addr, f_parm->size);
		if (ret) {
			return -EINVAL;
		}
		vsp_gmwarp_cmd_init_config(&index,
					   &ctx->gmwarp_video->init_addr);
		break;
	case GWARP_CID_TABLE_CONFIG:
		f_parm = arg;

		if (!ctx->gmwarp_video->table_fw_flags) {
			ctx->gmwarp_video->vir_table_mem = dma_alloc_coherent(
				ctx->gmwarp_video->gmwarp_dev->dev,
				GWARP_TABLE_MEM_SIZE,
				&ctx->gmwarp_video->table_addr, GFP_KERNEL);
			if (!ctx->gmwarp_video->vir_table_mem) {
				return -ENOMEM;
			}
			ctx->gmwarp_video->table_fw_flags = 1;
		}
		ret = copy_from_user(ctx->gmwarp_video->vir_table_mem,
				     f_parm->addr, f_parm->size);
		if (ret) {
			return -EINVAL;
		}
		vsp_gmwarp_cmd_table_config(&index,
					    &ctx->gmwarp_video->table_addr);

		break;

	case GWARP_CID_SCALER_CONFIG:
		f_parm = arg;

		if (!ctx->gmwarp_video->scaler_fw_flags) {
			ctx->gmwarp_video->vir_scaler_mem = dma_alloc_coherent(
				ctx->gmwarp_video->gmwarp_dev->dev,
				sizeof(struct scalerpara_t),
				&ctx->gmwarp_video->scaler_addr, GFP_KERNEL);
			if (!ctx->gmwarp_video->vir_scaler_mem) {
				return -ENOMEM;
			}
			ctx->gmwarp_video->scaler_fw_flags = 1;
		}
		ret = copy_from_user(ctx->gmwarp_video->vir_scaler_mem,
				     f_parm->addr, f_parm->size);
		if (ret) {
			return -EINVAL;
		}
		vsp_gmwarp_cmd_scaler_config(&index,
					     &ctx->gmwarp_video->scaler_addr);
		break;

	case GWARP_CID_PITCH_CONFIG:
		p_parm = arg;
		paddr[0] = index;
		paddr[1] = p_parm->input_width;
		paddr[2] = p_parm->output_width;
		vsp_gmwarp_cmd_pitch_config(&index, paddr);
		break;

	case GWARP_CID_CROP_CONFIG:
		f_parm = arg;
		if (!ctx->gmwarp_video->crop_fw_flags) {
			ctx->gmwarp_video->vir_crop_mem = dma_alloc_coherent(
				ctx->gmwarp_video->gmwarp_dev->dev,
				sizeof(struct cropcfg_t),
				&ctx->gmwarp_video->crop_addr, GFP_KERNEL);
			if (!ctx->gmwarp_video->vir_crop_mem) {
				return -ENOMEM;
			}
			ctx->gmwarp_video->crop_fw_flags = 1;
		}
		ret = copy_from_user(ctx->gmwarp_video->vir_crop_mem,
				     f_parm->addr, f_parm->size);
		if (ret) {
			return -EINVAL;
		}
		vsp_gmwarp_cmd_crop_config(&index,
					   &ctx->gmwarp_video->crop_addr);
		break;

	case GWARP_CID_SEPARATE_CONFIG:
		sep_parm = arg;
                ctx->in.x_offset = sep_parm->x_offset;
                ctx->in.y_offset = sep_parm->y_offset;
                ctx->in.valid_width = sep_parm->width;
                ctx->in.valid_height = sep_parm->height;
		ctx->gmwarp_video->yuv_sep_flags = 1;
		break;

	case GWARP_CID_SELFCHECK_START:
		s_parm = arg;
		ctx->gmwarp_video->state = GWARP_CTX_STATE_SELFCHECK;

		ctx->selfcheck_inmem =
			dma_alloc_coherent(ctx->gmwarp_video->gmwarp_dev->dev,
					   s_parm->in_size,
					   &ctx->selfcheck_inaddr, GFP_KERNEL);
		if (!ctx->selfcheck_inmem) {
			return -ENOMEM;
		}
		ret = copy_from_user(ctx->selfcheck_inmem, s_parm->addr,
				     s_parm->in_size);
		if (ret) {
			return -EINVAL;
		}

		ctx->selfcheck_outmem =
			dma_alloc_coherent(ctx->gmwarp_video->gmwarp_dev->dev,
					   s_parm->out_size,
					   &ctx->selfcheck_outaddr, GFP_KERNEL);
		if (!ctx->selfcheck_outmem) {
			return -ENOMEM;
		}
		paddr[0] = index;
		paddr[1] = ctx->selfcheck_inaddr;
		paddr[2] = ctx->selfcheck_outaddr;
		paddr[3] = s_parm->framerate;
		vsp_gmwarp_cmd_selfcheck_start(&index, paddr);
		break;

	case GWARP_CID_SELFCHECK_STOP:
		s_parm = arg;
		dma_free_coherent(ctx->gmwarp_video->gmwarp_dev->dev,
				  s_parm->in_size, ctx->selfcheck_inmem,
				  ctx->selfcheck_inaddr);
		dma_free_coherent(ctx->gmwarp_video->gmwarp_dev->dev,
				  s_parm->out_size, ctx->selfcheck_outmem,
				  ctx->selfcheck_outaddr);
		vsp_gmwarp_cmd_selfcheck_stop(&index);
		break;
	default:
		break;
	}
	return 0;
}
static const struct v4l2_ioctl_ops gmwarp_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,

	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,

	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,

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

	.vidioc_default = bst_gmwarp_vidioc_default,

};

static int gmwarp_probe(struct platform_device *pdev)
{
	struct bst_gmwarp_dev *gmwarp;
	struct video_device *vfd;
	int plane;
	int i = 0;
	int ret = 0;

	if (!pdev->dev.of_node)
		return -ENODEV;

	gmwarp = devm_kzalloc(&pdev->dev, sizeof(*gmwarp), GFP_KERNEL);
	if (!gmwarp)
		return -ENOMEM;

	gmwarp_dev = gmwarp;
	gmwarp->dev = &pdev->dev;
	spin_lock_init(&gmwarp->ctrl_lock);
	mutex_init(&gmwarp->mutex);

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		pr_err("call reserve mem device init failed!\n");
		return -ENODEV;
	}

	ret = v4l2_device_register(&pdev->dev, &gmwarp->v4l2_dev);
	if (ret)
		goto err_device_init;

	platform_set_drvdata(pdev, gmwarp);

	for (i = 0; i < GWARP_MAX_CHANNEL_NUM; i++) {
		gmwarp->gmwarp_video[i].gmwarp_dev = gmwarp;
		gmwarp->gmwarp_video[i].channel_id = i;

		mutex_init(&gmwarp->gmwarp_video[i].lock);
		spin_lock_init(&gmwarp->gmwarp_video[i].irqlock);
		vfd = &gmwarp->gmwarp_video[i].video;
		snprintf(vfd->name, sizeof(vfd->name), "%s-%d",
			 "bst_gmwarp_channel", i);
		vfd->fops = &gmwarp_fops;
		vfd->ioctl_ops = &gmwarp_ioctl_ops;
		vfd->minor = -1;
		vfd->release = video_device_release;
		vfd->vfl_dir = VFL_DIR_M2M;
		vfd->lock = &gmwarp->gmwarp_video[i].lock;
		vfd->v4l2_dev = &gmwarp->v4l2_dev;
                vfd->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;

		video_set_drvdata(vfd, &gmwarp->gmwarp_video[i]);

		gmwarp->gmwarp_video[i].m2m_dev =
			v4l2_m2m_init(&gmwarp_m2m_ops);
		if (IS_ERR(gmwarp->gmwarp_video[i].m2m_dev)) {
			v4l2_err(&gmwarp->v4l2_dev,
				 "Failed to init mem2mem device\n");
			ret = PTR_ERR(gmwarp->gmwarp_video[i].m2m_dev);
			goto unreg_video_dev;
		}
		ret = video_register_device(vfd, VFL_TYPE_VIDEO,
					    GMWARP_DEVICE_NR + i);
		if (ret) {
			v4l2_err(&gmwarp->v4l2_dev,
				 "Failed to register video device\n");
			goto rel_vdev;
		}
		v4l2_info(&gmwarp->v4l2_dev, "Registered %s as /dev/%s\n",
			  vfd->name, video_device_node_name(vfd));
	}

	for (plane = 0; plane < MAX_FRAME_BUF_NUM; plane++) {
		def_frame.stride[plane] =
			(def_frame.width * def_frame.fmt->depth) >> 3;
		def_frame.size[plane] =
			def_frame.stride[plane] * def_frame.height;
	}

	vsp_core_callback_register(VSP_CORE_GMWARP, gmwarp_ipc_msg_recv);

	v4l2_info(&gmwarp->v4l2_dev, "gmwarp probe ok!\n");

	return 0;

rel_vdev:
	video_device_release(vfd);
unreg_video_dev:
	//video_unregister_device(gmwarp->vfd);
unreg_v4l2_dev:
	v4l2_device_unregister(&gmwarp->v4l2_dev);
err_device_init:

	return ret;
}

static int gmwarp_remove(struct platform_device *pdev)
{
	int i;
	struct bst_gmwarp_dev *gmwarp = platform_get_drvdata(pdev);

	v4l2_info(&gmwarp->v4l2_dev, "Removing\n");

	for (i = 0; i < GWARP_MAX_CHANNEL_NUM; i++) {
		v4l2_m2m_release(gmwarp->gmwarp_video[i].m2m_dev);
		video_unregister_device(&gmwarp->gmwarp_video[i].video);
	}
	v4l2_device_unregister(&gmwarp->v4l2_dev);
	gmwarp_dev = NULL;

	return 0;
}

static const struct of_device_id bst_gmwarp_match[] = {
	{
		.compatible = "bst,bst-gmwarp",
	},
	{},
};

MODULE_DEVICE_TABLE(of, bst_gmwarp_match);

static struct platform_driver gmwarp_pdrv = {
	.probe = gmwarp_probe,
	.remove = gmwarp_remove,
	.driver = {
		.name = GMWARP_NAME,
		.of_match_table = of_match_ptr(bst_gmwarp_match),
	},
};

static int __init bst_gmwarp_init(void)
{
	return platform_driver_register(&gmwarp_pdrv);
}

late_initcall_sync(bst_gmwarp_init);

MODULE_AUTHOR("Jim Zheng <jim.zheng@bst.ai>");
MODULE_DESCRIPTION("BST gmwarp driver");
MODULE_LICENSE("GPL");

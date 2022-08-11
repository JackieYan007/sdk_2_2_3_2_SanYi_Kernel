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

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <asm/cacheflush.h>
#include <linux/pagemap.h>

#include "bst_gmwarp_drv.h"
#include "bst_gmwarp_msg.h"

static int gmwarp_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct gmwarp_ctx *ctx = vb2_get_drv_priv(vq);
	struct bst_gmwarp_video *gmwarp_video = ctx->gmwarp_video;
	struct gmwarp_frame *f = gmwarp_get_frame(ctx, vq->type);
	int nplane;

	if (IS_ERR(f))
		return PTR_ERR(f);

	for (nplane = 0; nplane < f->num_planes; nplane++) {
		alloc_devs[nplane] = gmwarp_video->gmwarp_dev->dev;
		sizes[nplane] = f->size[nplane];
		if (sizes[nplane] == 0) {
			pr_err("ERROR: plane %d, sizeimage == 0\n", nplane);
			return -EINVAL;
		}
	}
	*nplanes = f->num_planes;

	if (*nbuffers == 0)
		*nbuffers = 1;

	*nbuffers = min(*nbuffers, MAX_FRAME_BUF_NUM);

	return 0;
}

static int gmwarp_buf_prepare(struct vb2_buffer *vb)
{
	struct gmwarp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct gmwarp_frame *f = gmwarp_get_frame(ctx, vb->vb2_queue->type);
	int nplane;
#ifdef GMWARP_MMAP_CACHE
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_m2m_buffer *buffer =
		container_of(vbuf, struct v4l2_m2m_buffer, vb);

	struct bst_gmwarp_buffer *gmwarp_buffer;
	gmwarp_buffer = container_of(buffer, struct bst_gmwarp_buffer, m2m_buf);
#endif
	if (IS_ERR(f))
		return PTR_ERR(f);

	for (nplane = 0; nplane < f->num_planes; nplane++) {
		if (vb2_plane_size(vb, nplane) < f->size[nplane])
			return -EINVAL;
		vb2_set_plane_payload(vb, nplane, f->size[nplane]);
#ifdef GMWARP_MMAP_CACHE
		if (gmwarp_buffer->mmap_uaddr[nplane]) {
			__dma_flush_area(gmwarp_buffer->mmap_uaddr[nplane],
					 gmwarp_buffer->plane_size[nplane]);
		}
#endif
	}

	return 0;
}

static void gmwarp_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct gmwarp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int gmwarp_buf_start_streaming(struct vb2_queue *q, unsigned int count)
{
	return 0;
}

static void gmwarp_buf_stop_streaming(struct vb2_queue *q)
{
	struct gmwarp_ctx *ctx = vb2_get_drv_priv(q);
	struct bst_gmwarp_video *gmwarp_video = ctx->gmwarp_video;
	struct vb2_v4l2_buffer *vbuf;
	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

const struct vb2_ops gmwarp_qops = {
	.queue_setup = gmwarp_queue_setup,
	.buf_prepare = gmwarp_buf_prepare,
	.buf_queue = gmwarp_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = gmwarp_buf_start_streaming,
	.stop_streaming = gmwarp_buf_stop_streaming,
};

void gmwarp_free_buf_handler(struct vb2_buffer *vbs, struct vb2_buffer *vbd,
			     uint32_t buff_id, uint32_t channel_id,
			     struct gmwarp_ctx *ctx)
{
	struct gmwarp_ctx *g_ctx;
	g_ctx = ctx;

	if (!ctx->gmwarp_video->yuv_sep_flags) {
		int plane;
		uint32_t paddr[vbs->num_planes * 4];

		for (plane = 0; plane < vbs->num_planes; plane++) {
			paddr[plane * 4] = channel_id + plane;
			paddr[plane * 4 + 1] =
				vb2_dma_contig_plane_dma_addr(vbs, plane);
			paddr[plane * 4 + 2] =
				vb2_dma_contig_plane_dma_addr(vbd, plane);
		}
		vsp_gmwarp_cmd_frame_start(paddr, vbs->num_planes, channel_id,
					   buff_id);
	} else {
		uint32_t paddr[8] = { 0 };
		int plane = 0;
		paddr[0] = channel_id;
		paddr[1] = vb2_dma_contig_plane_dma_addr(vbs, plane) + g_ctx->in.y_offset * g_ctx->in.width + g_ctx->in.x_offset;
		paddr[2] = paddr[1] - g_ctx->in.y_offset * g_ctx->in.width - g_ctx->in.x_offset + g_ctx->in.width * g_ctx->in.height + 
                                g_ctx->in.y_offset / 2 * g_ctx->in.width + g_ctx->in.x_offset;

		paddr[4] = channel_id;
		paddr[5] = vb2_dma_contig_plane_dma_addr(vbd, plane);
		paddr[6] = paddr[5] +
			   g_ctx->out.width *
				   g_ctx->out.height; 
		vsp_gmwarp_cmd_frame_start_plus(paddr, channel_id, buff_id);
	}
}

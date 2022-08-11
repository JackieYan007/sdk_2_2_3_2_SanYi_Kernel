/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __GMWARP_H__
#define __GMWARP_H__

#include <linux/platform_device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/coreip/bst_vsp_msg.h>
#include <linux/firmware.h>
#include <media/v4l2-mem2mem.h>

#define GMWARP_NAME "bst-gmwarp"

#define GMWARP_TIMEOUT 500

#define MAX_WIDTH 8192
#define MAX_HEIGHT 8192

#define MIN_WIDTH 320
#define MIN_HEIGHT 240

#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 720

#define MAX_FRAME_BUF_NUM 8

#define GMWARP_DEVICE_NR 30

#define GWARP_INIT_BIN "vsp/gmwarp/gwarp_init"
#define GWARP_TABLE_BIN "vsp/gmwarp/gwarp_table"
#define GWARP_INIT_MEM_SIZE 64 * 1024 // 64kb for init file
#define GWARP_TABLE_MEM_SIZE 64 * 1024 // 64kb for table file
#define GWARP_MAX_CHANNEL_NUM 12
#define GWARP_WORK_CACHE_NUM 8
#define GWARP_CTX_STATE_STOPPED 0
#define GWARP_CTX_STATE_RUNNING 1
#define GWARP_CTX_STATE_SELFCHECK 2

//#define GMWARP_MMAP_CACHE

struct bst_gmwarp_buffer {
	struct v4l2_m2m_buffer m2m_buf;
#ifdef GMWARP_MMAP_CACHE
	void *mmap_uaddr[VIDEO_MAX_PLANES];
	uint32_t plane_size[VIDEO_MAX_PLANES];
#endif
};

struct gmwarp_fmt {
	u32 fourcc;
	int depth;
	u8 uv_factor;
	u8 y_div;
	u8 x_div;
};

struct gmwarp_frame {
	/* Original dimensions */
	u32 width;
	u32 height;
        u32 x_offset;
        u32 y_offset;
        u32 valid_width;
        u32 valid_height;
	u32 colorspace;
	u32 quantization;

	/* Image format */
	struct gmwarp_fmt *fmt;
	u32 num_planes;

	/* Variables that can calculated once and reused */
	u32 stride[MAX_FRAME_BUF_NUM];
	u32 size[MAX_FRAME_BUF_NUM];
};

struct bst_gmwarp_version {
	u32 major;
	u32 minor;
};

struct gmwarp_cache_msg {
	struct work_struct msg_work;
	struct gmwarp_ctx *ctx;
	struct list_head node;
};

struct gmwarp_ctx {
	struct v4l2_fh fh;
	struct bst_gmwarp_video *gmwarp_video;
	struct gmwarp_frame in;
	struct gmwarp_frame out;
	struct v4l2_ctrl_handler ctrl_handler;

	struct mutex msg_lock;
	uint32_t buff_id;
	struct mutex ctx_lock;
	spinlock_t cache_queue_lock;

	int state;

	struct workqueue_struct *msg_workqueue;
	struct list_head cache_list;
	struct gmwarp_cache_msg cache_msg[GWARP_WORK_CACHE_NUM];

	//selfcheck
	void *selfcheck_inmem;
	void *selfcheck_outmem;
	dma_addr_t selfcheck_inaddr;
	dma_addr_t selfcheck_outaddr;
};

struct bst_gmwarp_video {
	struct bst_gmwarp_dev *gmwarp_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device video;
	struct gmwarp_ctx *ctx;

	struct mutex lock;
	spinlock_t irqlock;

	int init_fw_flags;
	//const struct firmware *init_fw;
	dma_addr_t init_addr;
	void *vir_init_mem;
	int table_fw_flags;
	//const struct firmware *table_fw;
	dma_addr_t table_addr;
	void *vir_table_mem;
	int scaler_fw_flags;

	dma_addr_t scaler_addr;
	void *vir_scaler_mem;
	uint32_t channel_id;

	int crop_fw_flags;
	dma_addr_t crop_addr;
	void *vir_crop_mem;

	int yuv_sep_flags;

	int state;
};
struct bst_gmwarp_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct bst_gmwarp_video gmwarp_video[GWARP_MAX_CHANNEL_NUM];

	struct bst_gmwarp_version version;

	struct mutex mutex;
	spinlock_t ctrl_lock;

	int fw_flag;
};

struct gmwarp_frame *gmwarp_get_frame(struct gmwarp_ctx *ctx,
				      enum v4l2_buf_type type);

/* GMWARP Buffers Manage Part */
extern const struct vb2_ops gmwarp_qops;
void gmwarp_ipc_msg_handler(struct work_struct *work);

#endif

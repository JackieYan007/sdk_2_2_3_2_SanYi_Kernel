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


#ifndef __BST_ENCODER_DRV_H__
#define __BST_ENCODER_DRV_H__



#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <linux/workqueue.h>
#include <media/v4l2-mem2mem.h>
#include <linux/wait.h>


#define BST_ENC_DEVICE_NR 50

#define BST_ENC_CHANNEL_NUM 8

#define MIX_BUFFER_NUM 3
#define MAX_BUFFER_NUM 8
#define ENC_WORK_CACHE_NUM 6


//video state
#define ENC_STREAMING_OFF 0
#define ENC_STREAMING_ON 1
#define ENC_SELFCHECK_MODE 2

//#define ENC_MMAP_CACHE

struct bst_encoder_buffer {
    struct v4l2_m2m_buffer m2m_buf;
#ifdef ENC_MMAP_CACHE
    void *mmap_uaddr[VIDEO_MAX_PLANES];
    uint32_t plane_size[VIDEO_MAX_PLANES];
#endif
};

struct benc_video_format {
    u32 width;
    int height;
    u32 pixfmt;
    unsigned int num_planes;
    u32 type;
    u32 size[2];
};

struct enc_cache_msg {
    struct work_struct msg_work;
    struct bst_enc_ctx *enc_ctx;
    struct list_head node;
    uint32_t bytesize;                   //h264 data size
};

struct bst_enc_ctx {
    struct v4l2_fh fh;
    struct bst_enc_video *enc_video;

    struct v4l2_ctrl_handler ctrl_handler;
    struct mutex msg_lock;
    uint32_t buff_id;
    struct mutex ctx_lock;
    spinlock_t cache_queue_lock;

    struct workqueue_struct *msg_workqueue;
    struct list_head cache_list;
    struct enc_cache_msg cache_msg[ENC_WORK_CACHE_NUM];

    struct benc_video_format fmt_out;
    struct benc_video_format fmt_cap;
    
};

struct bst_enc_video {
    struct bst_enc_dev *enc_dev;
    struct video_device video;

    struct v4l2_m2m_dev *m2m_dev;
    struct bst_enc_ctx *enc_ctx;

    u32 channel_id;
    int state;
    struct list_head list;
    struct list_head buf_list;
    
    enum v4l2_buf_type type;
    
    struct mutex lock;
    struct vb2_queue queue;
    spinlock_t irqlock;
    struct vb2_v4l2_buffer *queue_buf[MAX_BUFFER_NUM];

    struct v4l2_pix_format_mplane format;

    uint32_t bitrate;
    uint32_t gop;
    uint32_t qp;
    uint32_t width;
    uint32_t height;
    uint32_t framerate;

    //sps_pps
    wait_queue_head_t sps_pps_queue;
    uint64_t p_vaddr;
    phys_addr_t p_phy_addr;
    int sps_pps_flags;
    
    //selfcheck
    int selfcheck_flags;
    void *selfcheck_yuv_mem[3];
    void *selfcheck_y_mem[3];
    void *selfcheck_h264_mem[3];
    dma_addr_t selfcheck_yuv_addr[3];
    dma_addr_t selfcheck_y_addr[3];
    dma_addr_t selfcheck_h264_addr[3];
   
};

struct bst_enc_dev {
    struct device *dev;
    struct v4l2_device v4l2_dev;
    struct bst_enc_video enc_video[BST_ENC_CHANNEL_NUM];

    spinlock_t irqlock;
    struct mutex lock;

    int fw_flag;
};


#endif /* __BST_ENCODER_DRV_H__ */

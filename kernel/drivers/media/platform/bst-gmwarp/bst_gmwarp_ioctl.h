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

#ifndef __BST_GMWARP_IOCTL__
#define __BST_GMWARP_IOCTL__

//for fw v01a0

struct file_parm {
	void *addr; //config file addr in user mem
	int size; //the size of config file
};

struct scaler_parm {
	uint32_t input_width;
	uint32_t input_height;
	uint32_t output_width;
	uint32_t output_height;
};

struct scalercfg_t {
	uint32_t zoom_facotr_x;
	uint32_t zoom_facotr_y;
	uint32_t ver_luma_initial_phase;
	uint32_t ver_chroma_initial_phase;
	uint32_t hor_luma_initial_phase;
	uint32_t hor_chroma_initial_phase;
	uint32_t input_width;
	uint32_t input_height;
	uint32_t output_width;
	uint32_t output_height;
	uint32_t chroma_422;
	uint32_t ver_black_clip;
	uint32_t hor_black_clip;
	uint32_t gmwarp_clip_y;
	uint32_t gmwarp_clip_u;
	uint32_t gmwarp_clip_v;

	uint32_t ver_luma_polyphase_shift;
	uint32_t ver_chroma_polyphase_shift;
	uint32_t hor_luma_polyphase_shift;
	uint32_t hor_chroma_polyphase_shift;
	uint32_t rsv[4];
};

struct scalerpara_t {
	struct scalercfg_t scalercfg;
	int8_t ver_luma_polyphase[96];
	int8_t ver_chroma_polyphase[96];
	int8_t hor_luma_polyphase[96];
	int8_t hor_chroma_polyphase[96];
};

struct cropcfg_t {
	uint32_t warp_enable;
	uint32_t chroma_422;
	uint32_t output_width;
	uint32_t output_height;
	uint32_t input_width;
	uint32_t input_height;
	uint32_t rsv[2];
};

struct pitch_parm {
	uint32_t input_width;
	uint32_t output_width;
};

struct yuv_parm {
	uint32_t width;          //valid width
	uint32_t height;         //valid height
	uint32_t x_offset;
	uint32_t y_offset;
        uint32_t wstride;
};

struct selfcheck_parm {
	void *addr; //input data addr
	int in_size; //input data size (width * height * 3 / 2)
	int out_size; //output data size
	int framerate; //framerate
};

#define GWARP_CID_INIT_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 1, struct file_parm)
#define GWARP_CID_TABLE_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 2, struct file_parm)
#define GWARP_CID_SCALER_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 3, struct file_parm)
#define GWARP_CID_PITCH_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 4, struct pitch_parm)
#define GWARP_CID_CROP_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 5, struct file_parm)
#define GWARP_CID_SEPARATE_CONFIG \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 6, struct yuv_parm)

#define GWARP_CID_SELFCHECK_START \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 7, struct selfcheck_parm)
#define GWARP_CID_SELFCHECK_STOP \
	_IOW('G', V4L2_CID_PRIVATE_BASE + 8, struct selfcheck_parm)

#endif

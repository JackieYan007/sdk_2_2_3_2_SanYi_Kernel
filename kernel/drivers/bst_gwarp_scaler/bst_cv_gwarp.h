#ifndef BST_CV_GWARP_H
#define BST_CV_GWARP_H

#include <linux/types.h>
#include "bst_cv.h"
//#define BSTCV_DEBUG
#define CV_GWARP_CHANNEL 3
#ifndef BSTCV_DEBUG
struct gwarp_regs {
	__u32 *gwarp_base;
	__u32 *gwarp_format; //+0x04
	__u32 *gwarp_src_resolution; //+0x08
	__u32 *gwarp_dst_resolution; //+0x0c
	__u32 *gwarp_src_ch[3]; //+0x10//+0x14//+0x18
	__u32 *gwarp_dst_ch[3]; //+0x1c//+0x20//+0x24
	__u32 *gwarp_lut_base; //+0x28
	__u32 *gwarp_src_stride; //+0x2c
	__u32 *gwarp_dst_stride; //+0x30
	__u32 *gwarp_lut_stride; //+0x34
	__u32 *gwarp_axi_parameter; //0x38
	__u32 *gwarp_intr_clear; //+0x3c
	__u32 *gwarp_intr_enable; //+0x40
};

typedef struct gwarp_channel_mem {
	__u32 gwarp_src_phy_addr[CV_GWARP_CHANNEL];
	__u32 gwarp_dst_phy_addr[CV_GWARP_CHANNEL];
} gwarp_channel_mem_t;

#else
struct gwarp_regs {
	__u32 gwarp_base;
	__u32 gwarp_format; //+0x04
	__u32 gwarp_src_resolution; //+0x08
	__u32 gwarp_dst_resolution; //+0x0c
	__u32 gwarp_src_ch0; //+0x10
	__u32 gwarp_src_ch1; //+0x14
	__u32 gwarp_src_ch2; //+0x18
	__u32 gwarp_dst_ch0; //+0x1c
	__u32 gwarp_dst_ch1; //+0x20
	__u32 gwarp_dst_ch2; //+0x24
	__u32 gwarp_lut_base; //+0x28
	__u32 gwarp_src_stride; //+0x2c
	__u32 gwarp_dst_stride; //+0x30
	__u32 gwarp_lut_stride; //+0x34
	__u32 gwarp_axi_parameter; //0x38
	__u32 gwarp_intr_clear; //+0x3c
	__u32 gwarp_intr_enable; //+0x40
};
#endif
void bst_enable_gwarp(int is_enable);
int bst_gwarp_preinit(struct bst_cv *cv_dev);
void bst_gwarp_reset(struct bst_cv *cv_dev);
void bst_gwarp_clear_intr(struct bst_cv *cv_dev);
int bst_gwarp_init(struct bst_cv *cv_dev);
int bst_gwarp_start(struct bst_cv *cv_dev, struct gwarp_param param);
int get_format_bpp(int format);
void dump_gwarp_param(struct bst_cv *cv_dev, const struct gwarp_param param);
int get_channel_size(int format, int channel_indx, int res_size);
int get_channel_num_by_pf(__u8 pf);
#endif //BST_CV_GWARP_H
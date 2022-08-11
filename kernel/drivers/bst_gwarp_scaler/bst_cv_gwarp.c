#include "bst_cv_gwarp.h"

#define BST_CV_GWARP_BASE_PHY 0x51060000
#define BST_CV_CHANNEL_OFFSET 0x1000000
#ifdef BSTCV_DEBUG
static struct gwarp_regs *global_gwarp_regs;
#else
static struct gwarp_regs global_gwarp_regs;
#endif

void bst_gwarp_reset(struct bst_cv *cv_dev)
{
}

void bst_gwarp_clear_intr(struct bst_cv *cv_dev)
{
	BST_CV_GS_TRACE_PRINTK("start");
	writel_relaxed(0x1, global_gwarp_regs.gwarp_intr_clear);
	BST_CV_GS_TRACE_PRINTK("end");
}

int check_cv_regs(struct bst_cv *cv_dev)
{
	struct device *dev = cv_dev->dev;
	int i;

	if (global_gwarp_regs.gwarp_base == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_format == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_src_resolution == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_dst_resolution == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < 3; i++) {
		if (global_gwarp_regs.gwarp_src_ch[i] == NULL) {
			BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
			return -ENOMEM;
		}
		if (global_gwarp_regs.gwarp_dst_ch[i] == NULL) {
			BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
			return -ENOMEM;
		}
	}

	if (global_gwarp_regs.gwarp_lut_base == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_src_stride == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_dst_stride == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_lut_stride == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_intr_clear == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}
	if (global_gwarp_regs.gwarp_intr_enable == NULL) {
		BST_CV_DEV_ERR(dev, "devm_ioremap failed\n");
		return -ENOMEM;
	}

	BST_CV_DEV_INFO(dev, "devm_ioremap success!\n");
	return 0;
}

int bst_gwarp_preinit(struct bst_cv *cv_dev)
{
	struct device *dev = cv_dev->dev;

	//BST_CV_GS_TRACE_PRINTK("Gwarp ioremap start");
	global_gwarp_regs.gwarp_base = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY, 4);
	global_gwarp_regs.gwarp_format = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x04, 4);
	global_gwarp_regs.gwarp_src_resolution = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x08, 4);
	global_gwarp_regs.gwarp_dst_resolution = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x0c, 4);
	global_gwarp_regs.gwarp_src_ch[0] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x10, 4);
	global_gwarp_regs.gwarp_src_ch[1] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x14, 4);
	global_gwarp_regs.gwarp_src_ch[2] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x18, 4);
	global_gwarp_regs.gwarp_dst_ch[0] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x1c, 4);
	global_gwarp_regs.gwarp_dst_ch[1] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x20, 4);
	global_gwarp_regs.gwarp_dst_ch[2] = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x24, 4);
	global_gwarp_regs.gwarp_lut_base = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x28, 4);
	global_gwarp_regs.gwarp_src_stride = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x2c, 4);
	global_gwarp_regs.gwarp_dst_stride = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x30, 4);
	global_gwarp_regs.gwarp_lut_stride = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x34, 4);
	global_gwarp_regs.gwarp_axi_parameter = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x38, 4);
	global_gwarp_regs.gwarp_intr_clear = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x3c, 4);
	global_gwarp_regs.gwarp_intr_enable = devm_ioremap(dev, BST_CV_GWARP_BASE_PHY + 0x40, 4);
	// ioremap dst phy addr
	check_cv_regs(cv_dev);
	cv_dev->gwarp_regs = &global_gwarp_regs;
	//BST_CV_GS_TRACE_PRINTK("Gwarp preinit end");
	return 0;
}

int bst_gwarp_init(struct bst_cv *cv_dev)
{
	return 0;
}

void dump_gwarp_param(struct bst_cv *cv_dev, const struct gwarp_param param)
{
	BST_CV_DEV_INFO(cv_dev->dev, "src_format: %d", param.src_format);
	BST_CV_DEV_INFO(cv_dev->dev, "dst_format: %d", param.dst_format);
	BST_CV_DEV_INFO(cv_dev->dev, "src resolution w: %d, h: %d", param.src_res.width, param.src_res.height);
	BST_CV_DEV_INFO(cv_dev->dev, "0x%x", param.mem_info.gwarp_lut_phy_addr);
	BST_CV_DEV_INFO(cv_dev->dev, "src channel address: 0x%x",
					param.mem_info.gwarp_src_phy_addr);
	BST_CV_DEV_INFO(cv_dev->dev, "dst channel address: 0x%x",
					param.mem_info.gwarp_dst_phy_addr);
	BST_CV_DEV_INFO(cv_dev->dev, "output fd %d", param.out_dma_fd);
}

int get_format_bpp(int format)
{
	int bpp;

	switch (format) {
	case GWARP_RGB888:
		bpp = 3;
		break;
	case GWARP_YUV422_YUYV:
	case GWARP_YUV422_UYVY:
		bpp = 2;
		break;
	case GWARP_NV12:
	case GWARP_NV21:
	case GWARP_RGB888_PLANAR:
	case GWARP_YUV420_PLANAR:
	case GWARP_YUV422_PLANAR:
		bpp = 1;
		break;
	}
	return bpp;
}

int get_channel_num_by_pf(__u8 pf)
{
	int channel_num = 0;

	switch (pf) {
	case GWARP_RGB888:
	case GWARP_YUV422_YUYV:
	case GWARP_YUV422_UYVY:
		channel_num = 1;
		break;
	case GWARP_NV12:
	case GWARP_NV21:
	case GWARP_YUV420_PLANAR:
	case GWARP_YUV422_PLANAR:
		channel_num = 2;
		break;
	case GWARP_RGB888_PLANAR:
		channel_num = 3;
		break;
	}
	return channel_num;
}

int get_channel_size(int format, int channel_indx, int res_size)
{
	int channel[3] = {0};
	int bpp = get_format_bpp(format);
	int i;

	switch (format) {
	case GWARP_RGB888:
		channel[0] = res_size * bpp;
		break;
	case GWARP_RGB888_PLANAR:
		for (i = 0; i < 3; i++) {
			channel[i] = res_size * bpp;
		}
		break;
	case GWARP_YUV422_YUYV:
	case GWARP_YUV422_UYVY:
		channel[0] = res_size * bpp;
		break;
	case GWARP_NV12:
	case GWARP_NV21:
		channel[0] = res_size * bpp;
		channel[1] = res_size * bpp / 2;
		break;
	case GWARP_YUV420_PLANAR:
		channel[0] = res_size * bpp;
		channel[1] = res_size * bpp / 2;
		channel[2] = res_size * bpp / 2;
		break;
	case GWARP_YUV422_PLANAR:
		channel[0] = res_size * bpp;
		channel[1] = res_size * bpp;
		break;
	}
	return channel[channel_indx];
}

void bst_enable_gwarp(int is_enable)
{
	int val;

	val = readl_relaxed(global_gwarp_regs.gwarp_base);
	if (is_enable)
		val |= 0x1;
	else
		val &= ~(0x1);
	writel_relaxed(val, global_gwarp_regs.gwarp_base);
}

int bst_gwarp_start(struct bst_cv *cv_dev, struct gwarp_param param)
{
	// dump_gwarp_param(cv_dev, param);
	int i;
	int channel_offset = 0;
	__u32 sys_ctrl = 0;
	__u32 src_res = 0;
	__u32 dst_res = 0;
	__u32 src_bpp = 0;
	__u32 dst_bpp = 0;
	__u32 src_stride = 0;
	__u32 dst_stride = 0;
	__u32 lut_stride = 0;
	gwarp_channel_mem_t channel_mem;
	/*GWC_SYS_CTL*/
	sys_ctrl |= param.src_format;
	sys_ctrl |= param.dst_format << 8;
	sys_ctrl |= param.is_bilinear << 16;
	writel_relaxed(sys_ctrl, global_gwarp_regs.gwarp_format);
	/*Resolution*/
	src_res |= param.src_res.width;
	src_res |= param.src_res.height << 16;
	dst_res |= param.dst_res.width;
	dst_res |= param.dst_res.height << 16;
	writel_relaxed(src_res, global_gwarp_regs.gwarp_src_resolution);
	writel_relaxed(dst_res, global_gwarp_regs.gwarp_dst_resolution);
	/*Enable intr*/
	writel_relaxed(0x1, global_gwarp_regs.gwarp_intr_enable);

	switch (param.src_format) {
	case GWARP_NV12:
	case GWARP_NV21:
		channel_offset = param.src_res.width * param.src_res.height;
		channel_mem.gwarp_src_phy_addr[0] = param.mem_info.gwarp_src_phy_addr;
		channel_mem.gwarp_src_phy_addr[1] = param.mem_info.gwarp_src_phy_addr + channel_offset;
		break;
	case GWARP_RGB888:
		channel_mem.gwarp_src_phy_addr[0] = param.mem_info.gwarp_src_phy_addr;
	}

	switch (param.src_format) {
	case GWARP_NV12:
	case GWARP_NV21:
		channel_offset = param.dst_res.width * param.dst_res.height;
		channel_mem.gwarp_dst_phy_addr[0] = param.mem_info.gwarp_dst_phy_addr;
		channel_mem.gwarp_dst_phy_addr[1] = param.mem_info.gwarp_dst_phy_addr + channel_offset;
		break;
	case GWARP_RGB888:
		channel_mem.gwarp_dst_phy_addr[0] = param.mem_info.gwarp_dst_phy_addr;
	}

	for (i = 0; i < param.in_channel_num; i++) {
		writel_relaxed(channel_mem.gwarp_src_phy_addr[i], global_gwarp_regs.gwarp_src_ch[i]);
	}
	for (i = 0; i < param.out_channel_num; i++) {
		writel_relaxed(channel_mem.gwarp_dst_phy_addr[i], global_gwarp_regs.gwarp_dst_ch[i]);
	}
	writel_relaxed(param.mem_info.gwarp_lut_phy_addr, global_gwarp_regs.gwarp_lut_base);

	src_bpp = get_format_bpp(param.src_format);
	dst_bpp = get_format_bpp(param.dst_format);
	src_stride = param.src_res.width * src_bpp;
	dst_stride = param.dst_res.width * dst_bpp;
	lut_stride = ((param.src_res.width + 7) / 8 + 1) * 4;

	writel_relaxed(src_stride, global_gwarp_regs.gwarp_src_stride);
	writel_relaxed(dst_stride, global_gwarp_regs.gwarp_dst_stride);
	writel_relaxed(lut_stride, global_gwarp_regs.gwarp_lut_stride);

	//Enable gwarp
	bst_enable_gwarp(1);
	return 0;
}

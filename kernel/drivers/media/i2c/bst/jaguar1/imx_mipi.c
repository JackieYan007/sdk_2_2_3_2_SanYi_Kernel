/********************************************************************************
 *
 *  Copyright (C) 2016 	NEXTCHIP Inc. All rights reserved.
 *  Module		: Jaguar1 Device Driver
 *  Description	: MIPI CSI2 & V4L2 for imx6
 *  Author		:
 *  Date         :
 *  Version		: Version 1.0
 *
 ********************************************************************************
 *  History      :
 *
 *
 ********************************************************************************/
#ifdef FOR_IMX6
#include <linux/delay.h>
#include <linux/mipi_csi2.h>
#include "jaguar1_common.h"
#include "jaguar1_video.h"
#include "imx_mipi.h"

extern struct i2c_client* jaguar1_client;

static int s_v4l2_open[JAGUAR1_MAX_CHAN_CNT];

enum jaguar1_mode {
	jaguar1_mode_MIN = 0x00,
	jaguar1_mode_SD_PAL = 0x00,
	jaguar1_mode_SD_NTSC = 0x01,
	jaguar1_mode_HD_25 = 0x02,
	jaguar1_mode_HD_30 = 0x03,
	jaguar1_mode_HD_50 = 0x04,
	jaguar1_mode_HD_60 = 0x05,
	jaguar1_mode_FHD_25 = 0x06,
	jaguar1_mode_FHD_30 = 0x07,
	jaguar1_mode_HD_960P_25 = 0x08,
	jaguar1_mode_HD_960P_30 = 0x09,
	jaguar1_mode_SD_720H_PAL = 0x0A,
	jaguar1_mode_SD_720H_NTSC = 0x0B,
	jaguar1_mode_SD_1440H_PAL = 0x0C,
	jaguar1_mode_SD_1440H_NTSC = 0x0D,
	jaguar1_mode_MAX,
	jaguar1_mode_INIT = 0xff, /*only for sensor init*/
};

enum jaguar1_frame_rate {
	jaguar1_25_fps,
	jaguar1_30_fps,
	jaguar1_50_fps,
	jaguar1_60_fps,
};

struct jaguar1_mode_info {
	enum jaguar1_mode mode;
	u32 width;
	u32 height;
	int fps;
};

static struct jaguar1_mode_info jaguar1_mode_info_data[jaguar1_mode_MAX + 1] = {
	{jaguar1_mode_SD_PAL, 960, 576/2, 25},
	{jaguar1_mode_SD_NTSC, 960, 480/2, 30},
	{jaguar1_mode_HD_25, 1280, 720, 25},
	{jaguar1_mode_HD_30, 1280, 720, 30},
	{jaguar1_mode_HD_50, 1280, 720, 50},
	{jaguar1_mode_HD_60, 1280, 720, 60},
	{jaguar1_mode_FHD_25, 1920, 1080, 25},
	{jaguar1_mode_FHD_30, 1920, 1080, 30},
	{jaguar1_mode_HD_960P_25, 1280, 960, 25},
	{jaguar1_mode_HD_960P_30, 1280, 960, 30},
	{jaguar1_mode_SD_720H_PAL, 720, 576/2, 25},
	{jaguar1_mode_SD_720H_NTSC, 720, 480/2, 30},
	{jaguar1_mode_SD_1440H_PAL, 1440, 576/2, 25},
	{jaguar1_mode_SD_1440H_NTSC, 1440, 480/2, 30},
};

struct sensor_data jaguar1_data[JAGUAR1_MAX_CHAN_CNT];

static int jaguar1_change_mode_direct(enum jaguar1_mode mode, int v_channel)
{
	int retval = 0;

	jaguar1_data[v_channel].pix.width = jaguar1_mode_info_data[mode].width;
	jaguar1_data[v_channel].pix.height = jaguar1_mode_info_data[mode].height;

	if (jaguar1_data[v_channel].pix.width == 0 || jaguar1_data[v_channel].pix.height == 0)
		return -EINVAL;

	return retval;
}

static int jaguar1_init_mode(enum jaguar1_mode mode, enum jaguar1_mode orig_mode, int vc)
{
	int lanes;
	//void *mipi_csi2_info;
    struct mipi_csi2_info *mipi_csi2_info;
	u32 mipi_reg;
	unsigned int i = 0;
    extern unsigned int lane;

	printk("%s : %d v_channel : %d\n", __func__, __LINE__, vc);

	if ((mode > jaguar1_mode_MAX || mode < jaguar1_mode_MIN)  && (mode != jaguar1_mode_INIT))
	{
		pr_err("Wrong jaguar1 mode detected!\n");
		return -1;
	}

	mipi_csi2_info = mipi_csi2_get_info();
    printk("[DRV]>>mipi_csi2_get_info[lanes = %d] \n",mipi_csi2_info->lanes);
	/* initial mipi dphy */
	if (!mipi_csi2_info)
	{
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n", __func__, __FILE__);
		return -1;
	}

	if (!mipi_csi2_get_status(mipi_csi2_info))
	{
		mipi_csi2_enable(mipi_csi2_info);
	}

	if (!mipi_csi2_get_status(mipi_csi2_info))
	{
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}
    
    mipi_csi2_info->lanes = lane;
	lanes = mipi_csi2_set_lanes(mipi_csi2_info);
    printk("[DRV]>>mipi_csi2_set_lanes[lanes = %d] \n",lanes);

	/*Only reset MIPI CSI2 HW at sensor initialize*/
	if (mode == jaguar1_mode_INIT)
	{
		mipi_csi2_reset(mipi_csi2_info, 3024 / (lanes + 1));
	}

	switch(jaguar1_data[vc].pix.pixelformat)
	{
		case V4L2_PIX_FMT_YUYV :
		case V4L2_PIX_FMT_UYVY :
			mipi_csi2_set_datatype(mipi_csi2_info, jaguar1_data[vc].v_channel, MIPI_DT_YUV422);
			break;
		case V4L2_PIX_FMT_RGB565 :
			mipi_csi2_set_datatype(mipi_csi2_info, jaguar1_data[vc].v_channel, MIPI_DT_RGB565);
			break;
		case V4L2_PIX_FMT_BGR24 :
			mipi_csi2_set_datatype(mipi_csi2_info, jaguar1_data[vc].v_channel, V4L2_PIX_FMT_BGR24);
			break;
		default :
			pr_err("currently this sensor format can not be supported!\n");
			break;
	}
	if (mipi_csi2_info)
	{
		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		printk("%s : %d : mipi_dphy status %x\n", __func__, __LINE__, mipi_reg);
		while ((mipi_reg == 0x200) && (i < 10)) {
			printk("%s : %d : mipi_dphy status %x cnt : %d\n", __func__, __LINE__,
					mipi_reg, i);
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			printk("mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		printk("%s : %d : mipi_error1 %x \n", __func__, __LINE__, mipi_reg);
		while ((mipi_reg != 0x0) && (i < 10)) {
			printk("%s : %d : mipi_error1 %x cnt : %d\n", __func__, __LINE__,
					mipi_reg, i);
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			printk("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
	}

	return 0;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	void *mipi_csi2_info;
	int ret;

	jaguar1_data[sensor->v_channel].on = true;
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info)
	{
		mipi_csi2_enable(mipi_csi2_info);
	}
	else
	{
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n", __func__, __FILE__);
		return -EPERM;
	}
	ret = jaguar1_init_mode(jaguar1_mode_INIT, jaguar1_mode_INIT, sensor->v_channel);

	return ret;
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info)
	{
		if (mipi_csi2_get_status(mipi_csi2_info))
		{
			mipi_csi2_disable(mipi_csi2_info);
		}
	}

	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor = s->priv;

	if (s == NULL) {
		printk("ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.bt_sync_correct = 1;
	p->u.bt656.clock_curr = jaguar1_data[sensor->v_channel].mclk;

	return 0;
}

static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	struct sensor_data *sensor = s->priv;

	if (fmt->index > jaguar1_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = jaguar1_data[sensor->v_channel].pix.pixelformat;

	return 0;
}

static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;
	f->fmt.pix.width = jaguar1_mode_info_data[sensor->streamcap.capturemode].width;
	f->fmt.pix.height= jaguar1_mode_info_data[sensor->streamcap.capturemode].height;

	return 0;
}

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
		/* This is the only case currently handled. */
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			memset(a, 0, sizeof(*a));
			a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			cparm->capability = sensor->streamcap.capability;
			cparm->timeperframe = sensor->streamcap.timeperframe;
			cparm->capturemode = sensor->streamcap.capturemode;
			ret = 0;
			break;
			/* These are all the possible cases. */
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		case V4L2_BUF_TYPE_VBI_CAPTURE:
		case V4L2_BUF_TYPE_VBI_OUTPUT:
		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
			ret = -EINVAL;
			break;
		default:
			printk("   type is unknown - %d\n", a->type);
			ret = -EINVAL;
			break;
	}

	printk("[%s:%d]capture mode : %x\n", __func__, __LINE__, sensor->streamcap.capturemode);

	return ret;
}

static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	enum jaguar1_mode orig_mode;
	int ret = 0;

	switch (a->type) {
		/* This is the only case currently handled. */
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			/* Check that the new frame rate is allowed. */
			if ((timeperframe->numerator == 0)
					|| (timeperframe->denominator == 0)) {
				timeperframe->denominator = 30;
				timeperframe->numerator = 1;
			}

			orig_mode = sensor->streamcap.capturemode;
			jaguar1_change_mode_direct(orig_mode, sensor->v_channel);

			printk("[%s:%d]orig_mode:%x\n", __func__, __LINE__, orig_mode);
			printk("[%s:%d]capture mode:%x\n", __func__, __LINE__, a->parm.capture.capturemode);

			//ret = jaguar1_init_mode((u32) a->parm.capture.capturemode, orig_mode, sensor->v_channel);

			if (ret < 0)
				return ret;

			sensor->streamcap.timeperframe = *timeperframe;
			sensor->streamcap.capturemode = (u32) a->parm.capture.capturemode;

			break;

			/* These are all the possible cases. */
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		case V4L2_BUF_TYPE_VBI_CAPTURE:
		case V4L2_BUF_TYPE_VBI_OUTPUT:
		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
			printk("   type is not " "V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
					a->type);
			ret = -EINVAL;
			break;

		default:
			printk("   type is unknown - %d\n", a->type);
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int ioctl_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = s->priv;

	if (fsize->index > jaguar1_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = jaguar1_data[sensor->v_channel].pix.pixelformat;
	fsize->discrete.width = jaguar1_mode_info_data[fsize->index].width;
	fsize->discrete.height =jaguar1_mode_info_data[fsize->index].height;

	printk("[%s:%d]pixel_format %x\n", __func__, __LINE__, jaguar1_data[sensor->v_channel].pix.pixelformat);
	printk("[%s:%d]discrete.width %x\n", __func__, __LINE__, jaguar1_mode_info_data[fsize->index].width);
	printk("[%s:%d]discrete.height %x\n", __func__, __LINE__,  jaguar1_mode_info_data[fsize->index].height);

	return 0;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *fival)
{
	struct sensor_data *sensor = s->priv;
	int i, count = 0;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	for (i = 0; i < (jaguar1_mode_MAX + 1); i++)
		if (fival->pixel_format == jaguar1_data[sensor->v_channel].pix.pixelformat
				&& fival->width == jaguar1_mode_info_data[i].width
				&& fival->height == jaguar1_mode_info_data[i].height
				&& fival->index == count++) {
			fival->discrete.denominator = jaguar1_mode_info_data[i].fps;
			return 0;
		}

	return -EINVAL;
}

static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *) id)->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *) id)->match.name, "nvp6324_video_decoder");

	return 0;
}

static struct v4l2_int_ioctl_desc jaguar1_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,				(v4l2_int_ioctl_func *) ioctl_dev_init },
	{ vidioc_int_dev_exit_num,				(v4l2_int_ioctl_func *) ioctl_dev_exit },
	{ vidioc_int_g_ifparm_num,				(v4l2_int_ioctl_func *) ioctl_g_ifparm },
	{ vidioc_int_init_num,					(v4l2_int_ioctl_func *) ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,			(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,				(v4l2_int_ioctl_func *) ioctl_g_fmt_cap },
	{ vidioc_int_g_parm_num,				(v4l2_int_ioctl_func *) ioctl_g_parm },
	{ vidioc_int_s_parm_num,				(v4l2_int_ioctl_func *) ioctl_s_parm },
	{ vidioc_int_enum_framesizes_num,		(v4l2_int_ioctl_func *) ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,	(v4l2_int_ioctl_func *) ioctl_enum_frameintervals },
	{ vidioc_int_g_chip_ident_num,			(v4l2_int_ioctl_func *) ioctl_g_chip_ident },
};

static struct v4l2_int_slave jaguar1_slave[JAGUAR1_MAX_CHAN_CNT] = {
	{
		.ioctls = jaguar1_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(jaguar1_ioctl_desc),
	},

	{
		.ioctls = jaguar1_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(jaguar1_ioctl_desc),
	},

	{
		.ioctls = jaguar1_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(jaguar1_ioctl_desc),
	},

	{
		.ioctls = jaguar1_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(jaguar1_ioctl_desc),
	},
};

static struct v4l2_int_device jaguar1_int_device[JAGUAR1_MAX_CHAN_CNT] = {
	{
		.module = THIS_MODULE,
		.name = "nvp6324_0",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &jaguar1_slave[0],
		},
	},

	{
		.module = THIS_MODULE,
		.name = "nvp6324_1",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &jaguar1_slave[1],
		},
	},

	{
		.module = THIS_MODULE,
		.name = "nvp6324_2",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &jaguar1_slave[2],
		},
	},

	{
		.module = THIS_MODULE,
		.name = "nvp6324_3",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &jaguar1_slave[3],
		},
	},
};

int init_imx_mipi(int ch)
{
	int ret = 0;

	jaguar1_data[ch].i2c_client = jaguar1_client;

	jaguar1_data[ch].pix.pixelformat = V4L2_PIX_FMT_UYVY;
	jaguar1_data[ch].streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	jaguar1_data[ch].is_mipi = 1;
	jaguar1_data[ch].ipu_id = ch/2;
	jaguar1_data[ch].csi = ch%2;
	jaguar1_data[ch].v_channel = ch;
	jaguar1_data[ch].mclk= 270000;

	jaguar1_int_device[ch].priv = &jaguar1_data[ch];

	ret = v4l2_int_device_register(&jaguar1_int_device[ch]);
	if(ret < 0)
	{
		printk("v4l2_int_device_register fail!! jaguar1_int_device[%d]\n", ch);
	}
	else
	{
		s_v4l2_open[ch] = true;
		printk("%s[jaguar_data_%d] : ipu[%d], csi[%d], vc[%d]\n", __func__,
				ch, jaguar1_data[ch].ipu_id, jaguar1_data[ch].csi, jaguar1_data[ch].v_channel);
	}

	return ret;
}

void close_imx_mipi(void)
{
	int i;

	for(i=0 ; i<JAGUAR1_MAX_CHAN_CNT ; i++)
	{
		if(s_v4l2_open[i])
		{
			v4l2_int_device_unregister(&jaguar1_int_device[i]);
			s_v4l2_open[i] = false;
		}
	}
}

void set_imx_video_format(video_input_init *dev_ch_info)
{
	NC_VIVO_CH_FORMATDEF fmt = dev_ch_info->format;
	int mode = 0;

	switch(fmt)
	{
		case AHD20_SD_H960_2EX_Btype_PAL :
		case AHD20_SD_H960_2EX_Btype_SP_PAL :
			// not use
		case AHD20_SD_H960_PAL :
		case AHD20_SD_H1280_PAL :
		case AHD20_SD_H960_EX_PAL :
		case AHD20_SD_H960_2EX_PAL :
			mode = jaguar1_mode_SD_PAL;
			break;
		case AHD20_SD_H960_2EX_Btype_NT :
		case AHD20_SD_H960_2EX_Btype_SP_NT :
			// not use
		case AHD20_SD_H960_NT :
		case AHD20_SD_H1280_NT :
		case AHD20_SD_H960_EX_NT :
		case AHD20_SD_H960_2EX_NT :
			mode = jaguar1_mode_SD_NTSC;
			break;
		case AHD20_720P_25P_EX_Btype :
		case AHD20_720P_25P_EX_Btype_SP :
		case TVI_HD_B_25P_EX :
		case TVI_HD_25P_EX :
		case CVI_HD_25P_EX :
			// not use
		case AHD20_720P_25P :
		case AHD20_720P_25P_EX :
		case TVI_HD_25P :
		case TVI_HD_B_25P :
		case CVI_HD_25P :
			mode = jaguar1_mode_HD_25;
			break;
		case AHD20_720P_30P_EX_Btype :
		case AHD20_720P_30P_EX_Btype_SP :
		case TVI_HD_B_30P_EX :
		case TVI_HD_30P_EX :
		case CVI_HD_30P_EX :
			// not use
		case AHD20_720P_30P :
		case AHD20_720P_30P_EX :
		case TVI_HD_30P :
		case TVI_HD_B_30P :
		case CVI_HD_30P :
			mode = jaguar1_mode_HD_30;
			break;
		case AHD20_720P_50P :
		case TVI_HD_50P :
		case CVI_HD_50P :
			mode = jaguar1_mode_HD_50;
			break;
		case AHD20_720P_60P :
		case TVI_HD_60P :
		case CVI_HD_60P :
			mode = jaguar1_mode_HD_60;
			break;
		case AHD20_1080P_25P :
		case TVI_FHD_25P :
		case CVI_FHD_25P :
			mode = jaguar1_mode_FHD_25;
			break;
		case AHD20_1080P_30P :
		case TVI_FHD_30P :
		case CVI_FHD_30P :
			mode = jaguar1_mode_FHD_30;
			break;
			// not use
		case AHD20_1080P_60P :
		case AHD20_1080P_50P :
			mode = jaguar1_mode_FHD_30;
			break;

		case AHD20_720P_960P_25P:
			mode = jaguar1_mode_HD_960P_25;
			break;

		case AHD20_720P_960P_30P:
			mode = jaguar1_mode_HD_960P_30;
			break;

		case AHD20_SD_SH720_PAL :
			mode = jaguar1_mode_SD_720H_PAL;
			break;
		case AHD20_SD_SH720_NT:
			mode = jaguar1_mode_SD_720H_NTSC;
			break;

		case AHD20_SD_H1440_PAL :
			mode = jaguar1_mode_SD_1440H_PAL;
			break;
		case AHD20_SD_H1440_NT:
			mode = jaguar1_mode_SD_1440H_NTSC;
			break;
		case NC_VIVO_CH_FORMATDEF_UNKNOWN :
		case NC_VIVO_CH_FORMATDEF_AUTO :
		case AHD30_4M_30P :
		case AHD30_4M_25P :
		case AHD30_4M_15P :
		case AHD30_3M_30P :
		case AHD30_3M_25P :
		case AHD30_3M_18P :
		case AHD30_5M_12_5P :
		case AHD30_5M_20P :
		case AHD30_5_3M_20P :
		case AHD30_6M_18P :
		case AHD30_6M_20P :
		case AHD30_8M_X_30P :
		case AHD30_8M_X_25P :
		case AHD30_8M_7_5P :
		case AHD30_8M_12_5P :
		case AHD30_8M_15P :
		case TVI_3M_18P :
		case TVI_5M_12_5P :
		case TVI_4M_30P :
		case TVI_4M_25P :
		case TVI_4M_15P :
		case CVI_4M_30P :
		case CVI_4M_25P :
		case CVI_8M_15P :
		case CVI_8M_12_5P :
		case NC_VIVO_CH_FORMATDEF_MAX :
			mode = jaguar1_mode_FHD_30;
			printk("invalid input format %d\n", fmt);
			break;
	}

	jaguar1_data[dev_ch_info->ch].streamcap.capturemode = mode;
	jaguar1_data[dev_ch_info->ch].pix.width = jaguar1_mode_info_data[mode].width;
	jaguar1_data[dev_ch_info->ch].pix.height = jaguar1_mode_info_data[mode].height;
	jaguar1_data[dev_ch_info->ch].streamcap.timeperframe.denominator = jaguar1_mode_info_data[mode].fps;
	jaguar1_data[dev_ch_info->ch].streamcap.timeperframe.numerator = 1;

	printk("%s[jaguar_data_%d] : width[%d], height[%d]\n", __func__, dev_ch_info->ch,
			jaguar1_data[dev_ch_info->ch].pix.width, jaguar1_data[dev_ch_info->ch].pix.height);
}

#endif // FOR_IMX6

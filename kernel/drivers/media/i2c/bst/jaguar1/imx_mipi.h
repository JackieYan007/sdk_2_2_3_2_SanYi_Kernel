/********************************************************************************
 *
 *  Copyright (C) 2017 	NEXTCHIP Inc. All rights reserved.
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
#include <../drivers/media/platform/mxc/capture/mxc_v4l2_capture.h>
#include <../drivers/media/platform/mxc/capture/v4l2-int-device.h>
#include <../drivers/mxc/mipi/mxc_mipi_csi2.h>


extern struct sensor_data jaguar1_data[JAGUAR1_MAX_CHAN_CNT];

extern int init_imx_mipi(int ch);
extern void set_imx_video_format(video_input_init *dev_ch_info);
extern void close_imx_mipi(void);

#endif // FOR_IMX6

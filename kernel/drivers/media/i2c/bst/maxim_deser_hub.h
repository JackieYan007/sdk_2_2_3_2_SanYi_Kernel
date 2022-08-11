/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * maxim deser hub for BST Deser Driver
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __MAXIM_DESER_HUB__
#define __MAXIM_DESER_HUB__

#include "common_deser_hub.h"

#define MAXIM_ID_REG			0x1e
#define MAX9286_ID			0x40
#define MAX96705_ID			0x41
#define MAX96705_ADDR			0x40
#define MAX96705_BROADCAST		0x45
#define MAX96701_ID			0x45
#define MAX96701_ADDR			0x40
#define MAX_SER_ADDR			0x40
#define MAXIM_DESER_I2C_RETRY_TIMES 3 /* number of read/write retries */
#define MAXIM_N_LINKS 4
#define MAXIM_STREAM_ENABLE_MASK	0x01
#define MAXIM_STREAM_SUB_PORT_MASK	0xf0
#define MAXIM_VIDEO_GSML2_LOCK_A 0x1a
#define MAXIM_VIDEO_GSML2_LOCK_B 0xa
#define MAXIM_VIDEO_GSML2_LOCK_C 0xb
#define MAXIM_VIDEO_GSML2_LOCK_D 0xc

enum {
	RGB888_DT = 0,
	RGB565_DT,
	RGB666_DT,
	YUV8_DT,	  /*  default */
	YUV10_DT,
	RAW8_DT,
	RAW10_DT,
	RAW12_DT,
	RAW14_DT,
};

//#define MAX96712_DEBUG
enum maxim_pads {
	MAXIM_SINK_LINK0,
	MAXIM_SINK_LINK1,
	MAXIM_SINK_LINK2,
	MAXIM_SINK_LINK3,
	MAXIM_SOURCE,
	MAXIM_N_PADS,
};

struct maxim_hub_sink {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev	*sd;
	struct fwnode_handle	*fwnode;
};
// 9296 link mode(single link or spliter mode(link>1))
enum link_status {
	AUTO_LINK_MODE = 0,
	SPLITER_MODE = 1
};

enum link_mode {
	MAXIM_LINK_MODE_GMSL1 = 0,
	MAXIM_LINK_MODE_GMSL2 = 1
};

struct maxim_hub_priv {
	struct deser_hub_dev hub;
	struct media_pad pads[MAXIM_N_PADS];
	struct maxim_hub_sink sinks[MAXIM_N_LINKS];
	int des_addr;
	int n_links;
	int links_mask;
	long pixel_rate;
	int fsync_period;
	int pclk;
	int him;
	int hsync;
	int vsync;
	int bws;
	int dbl;
	int dt;
	int lane_speed;
	int timeout;
	int data_type;
	int serdes;
	int serial_i2c;
	u32 linkrx_rate[4];
	u64 crossbar;
	char cb[16];
	int ser_addr[MAXIM_N_LINKS];
	int ser_alias_addr[MAXIM_N_LINKS];
	int sensor_addr[MAXIM_N_LINKS];
	int sensor_alias_addr[MAXIM_N_LINKS];
	const char *fsync_mode;
	const char *deser_type;
	int serial_type;
	enum link_mode link_mode;
	enum link_status link_status;
};

/*
 *	if connected return 1 ,otherwise return 0
 */
int is_gmsl2_video_connected(struct deser_hub_dev *hub, int index);

void config_ser_reg_group(uint16_t (*group)[2], int len, struct deser_hub_dev *hub, int reg);

int write_register(struct i2c_adapter *adap, uint8_t slave_address, uint16_t reg_offset, uint8_t value);

int ser_word_write(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t value);

int ser_word_read(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t *value);

int ser_write(struct deser_hub_dev *hub, uint8_t addr, uint8_t reg, uint8_t value);

int reg8_read(struct i2c_client *client, u8 reg, u8 *val);

int write_reg(struct i2c_client *client, uint16_t reg, int val);

int max9286_reg_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value);
/* read reg for max96712 and max96722*/
int max96712_reg_read(struct deser_hub_dev *hub, uint16_t reg, uint8_t *value);
/* write reg for max96712 and max96722*/
int max96712_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value);

int max9296_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value);

//External trigger func
int maxim_deser_hub_set_internal_frame_sync(struct deser_hub_dev *maxim_hub_dev,
						int trigger_gpio, int fps);
//Internal trigger func
int maxim_deser_hub_set_external_frame_sync(struct deser_hub_dev *deser_hub, int external_freq, int target_freq, int fsync_in, int fsync_out,
		int camera_trigger_gpio, int deser_trigger_gpio);

int maxim_hub_mipi_output(struct maxim_hub_priv *priv, bool enable);
#endif // __MAXIM_DESER_HUB__

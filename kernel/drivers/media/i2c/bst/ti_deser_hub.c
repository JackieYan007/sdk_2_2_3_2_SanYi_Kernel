// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ti_deser_hub for BST Deserializer Driver
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_gpio.h>
#include <media/media-entity.h>
#include <media/media-device.h>

#include "ti_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,ti-deser-hub"

int serdes_read(struct deser_hub_dev *ti_deser_hub
	, uint8_t reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(ti_deser_hub->i2c_client, reg);
	if (ret < 0) {
		dev_info(ti_deser_hub->dev,
			"%s: register 0x%02x read failed (%d)\n", __func__, reg,
			ret);
	}
	return ret;
}

static int serdes_write(struct deser_hub_dev *ti_deser_hub, uint8_t reg,
			uint8_t value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ti_deser_hub->i2c_client, reg, value);
	if (ret < 0) {
		dev_dbg(ti_deser_hub->dev,
			"%s: register 0x%02x write failed (%d)\n", __func__,
			reg, ret);
	}
	return ret;
}

static int ser_write(struct deser_hub_dev *ti_deser_hub, uint8_t addr, uint8_t reg, uint8_t value)
{
	int ret;
	int retry_times = 25;
	uint8_t read_value;
	struct i2c_adapter *adap;

	adap = ti_deser_hub->i2c_client->adapter;

	if (adap == NULL) {
		return -EINVAL;
	};

	do {
		ret = bst_i2c_write_byte_data_byte_reg(adap, addr, reg, value);
		ret = bst_i2c_read_byte_data_byte_reg(adap, addr, reg, &read_value);
		retry_times--;
		if (retry_times <= 0) {
			pr_info("%s() failed timeout!", __func__);
			return -1;
		}
	} while (read_value != value);

	return 0;
}

static int ser_read(struct deser_hub_dev *ti_deser_hub, uint8_t addr, uint8_t reg, uint8_t *out_value)
{
	int ret;
	int retry_times = 25;
	struct i2c_adapter *adap;

	adap = ti_deser_hub->i2c_client->adapter;
	if (adap == NULL) {
		return -EINVAL;
	};

	do {
		ret = bst_i2c_read_byte_data_byte_reg(adap, addr, reg, out_value);
		retry_times--;
		if (retry_times <= 0) {
			pr_info("%s() failed timeout!", __func__);
			return -1;
		}
	} while (ret < 0);

	return 0;
}
// freq, 0x0: 2.5Mbps, 0x2: 10Mbps, 0x6 50Mbps
int ti_serdes_bc_frequency_select(struct deser_hub_dev *ti_deser_hub, int camera_index, u8 freq)
{
	u8 reg_value;

	if (ti_deser_hub == NULL)
		return -1;

	if ((freq != 0) && (freq != 2) && (freq != 6))
		return -1;

	reg_value = ((1 << camera_index) | (camera_index << 4));
	serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, reg_value);

	reg_value = serdes_read(ti_deser_hub, TI_DESER_BCC_CONFIG);
	pr_info("%s() origin freq = 0x%x\n", __func__, reg_value);
	reg_value &= (~0x7); // clear [2:0]
	reg_value |= freq;
	serdes_write(ti_deser_hub, TI_DESER_BCC_CONFIG, reg_value);
	usleep_range(1000, 2000);
	reg_value = serdes_read(ti_deser_hub, TI_DESER_BCC_CONFIG);
	pr_info("%s() new freq = 0x%x\n", __func__, reg_value);
	reg_value &= 0x7;

	return reg_value;
}

static int parse_camera_serdes(struct deser_hub_dev *ti_deser_hub,
				 struct device_node *remote_ep, int index)
{
	u32 value;
	u32 ser_alias_id = 0;
	u32 sensor_id = 0;
	u32 sensor_alias_id = 0;
	u8 temp = 0;
	int ret;
	const char *serializer = NULL;

	if (!of_property_read_u32(remote_ep, "ser-alias-id", &value))
		ser_alias_id = value;
	if (!of_property_read_u32(remote_ep, "sensor-id", &value))
		sensor_id = value;
	if (!of_property_read_u32(remote_ep, "sensor-alias-id", &value))
		sensor_alias_id = value;
	if (!of_property_read_u32(remote_ep, "tri-gpio", &value))
		ti_deser_hub->trig_info.trigger_tx_gpio = value;
	if (!of_property_read_string(remote_ep, "serializer", &serializer)) {
		if (serializer != NULL)
			pr_info("serializer is %s\n", serializer);
	}

	if (serializer == NULL) {
		pr_err("parse camera serdes error\n");
		return -1;
	}

	if (ti_deser_hub->ctl_mode == FAD_LIS_MODE) {
		dev_info(ti_deser_hub->dev, "FAD lis mode :don't mapping camera");
		return 0;
	}

	if (ti_deser_hub->type == DESER_TYPE_TI954) {
		temp = 0;
		if (index == 0) {
			temp |= 1 << index;
		} else if (index == 1) {
			//temp |= (1<<1 | 1<<4);
			temp |= 0x12;
		}
	} else if (ti_deser_hub->type == DESER_TYPE_TI960) {
		temp = 0;
		temp |= 1 << index | index << 4;
	}
	serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, temp);

	temp = TI_DESER_BCC_DEFAULT;
	temp |= 1 << 6; // enable i2c pass through
	if (strcmp(serializer, "ti953") == 0)
		temp |= 0x6; // back channel frequency 50Mbps

	if (strcmp(serializer, "ti933") == 0)
		temp &= ~(0x7); // back channel frequency 2.5Mbps

	ret = serdes_read(ti_deser_hub, TI_DESER_DEVICE_ID);
	if (ret > 0) {
		serdes_write(ti_deser_hub, TI_DESER_BCC_CONFIG, temp);
		serdes_write(ti_deser_hub, TI_DESER_SER_ALIAS_ID,
			     ser_alias_id * 2);
		serdes_write(ti_deser_hub, TI_DESER_REMOTE_SLAVE,
			     sensor_id * 2);
		serdes_write(ti_deser_hub, TI_SLAVE_ALAIS_0_REG
				, sensor_alias_id * 2);
	}

	return 0;
}

static int ti_deser_hub_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct deser_hub_dev *ti_deser_hub;
	int is_enable;
	int subdev_port;
	int index;
	u8 reg_value;
	int ret;
	uint8_t fwd_ctl1_value = 0;

	ti_deser_hub = container_of(subdev, struct deser_hub_dev, subdev);
	is_enable = (enable & TI_DESER_STREAM_ENABLE_MASK) == 1 ? 1 : 0;
	subdev_port  = (enable & TI_DESER_STREAM_SUB_PORT_MASK) >> 4;
	mutex_lock(&ti_deser_hub->deser_mutex);
	/*----- Step1 choose deser sub port------*/
	index = subdev_port % ti_deser_hub->max_port;

	pr_info("%s() device index:[%d]\n", __func__, index);

	if (!((ti_deser_hub->src_mask) & (1 << index)))
		pr_err("%s() device index:[%d] invalid\n", __func__, index);

	if (!ti_deser_hub->chn[index].camera_bound || (ti_deser_hub->chn[index].cam_dev == NULL))
		pr_err("%s() cam_dev [%d] is NULL\n", __func__, index);

	reg_value = 1 << index | index << 4;
	serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, reg_value);

	/*----- Step2 set FWD_CTL1------*/
	ret = serdes_read(ti_deser_hub, TI_DESER_FWD_CTL1);
	if (ret < 0) {
		pr_err("ti_deser_hub_set_fwd_sync, TI_DESER_FWD_CTL1 error\n");
		mutex_unlock(&ti_deser_hub->deser_mutex);
		return -1;
	}
	fwd_ctl1_value = (uint8_t)ret;
	fwd_ctl1_value &= ~(1 << (4 + index));

	if (is_enable)
		fwd_ctl1_value |= 0 << (4 + index);
	else
		fwd_ctl1_value |= 1 << (4 + index);

	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL1, fwd_ctl1_value);

	mutex_unlock(&ti_deser_hub->deser_mutex);
	return 0;
}

int ti_deser_hub_set_internal_frame_sync(struct deser_hub_dev *ti_deser_hub,
		int trigger_gpio, int fps)
{
	int i;
	u8 reg_value;
	int frame_period;
	u32 frame_sync_time;
	u8 fs_high_time0;
	u8 fs_low_time1;
	u8 fs_low_time0;

	if (ti_deser_hub->frame_sync_enabled) {
		pr_info("frame sync already enabled\n");
		return 0;
	}

	// for back channel, frame period needs 30 bits
	// using 25MHz clock
	frame_period = 40; // 40ns/bit, for 25MHz, 50/50mode, 30bits * 40ns/bit can't work
	frame_sync_time = 1000000000 / fps / frame_period;
	// using high, low 50/50 mode, so frame_sync_time divided by 2
	frame_sync_time /= 2;
	fs_high_time0 = (u8)((frame_sync_time & 0xff0000) >> 16);
	fs_low_time1 = (u8)((frame_sync_time & 0xff00) >> 8);
	fs_low_time0 = (u8)(frame_sync_time & 0xff);

	for (i = 0; i < ti_deser_hub->max_port; i++) {
		if (!((ti_deser_hub->src_mask) & (1 << i)))
			continue;
		if (!ti_deser_hub->chn[i].camera_bound ||
		    (ti_deser_hub->chn[i].cam_dev == NULL)) {
			continue;
		}

		reg_value = 1 << i | i << 4;
		serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, reg_value);
		//serdes_write(ti_deser_hub, TI_DESER_FWD_CTL2, 0x04);
		//serdes_write(ti_deser_hub, 0x19, 0x00);
		serdes_write(ti_deser_hub, 0x1a, fs_high_time0);
		serdes_write(ti_deser_hub, 0x1b, fs_low_time1);
		serdes_write(ti_deser_hub, 0x1c, fs_low_time0);
		serdes_write(ti_deser_hub, 0x18, 0x43);

		if (trigger_gpio == 1)
			serdes_write(ti_deser_hub, TI_DESER_BC_GPIO_CTL0, 0xa9);
		else
			serdes_write(ti_deser_hub, TI_DESER_BC_GPIO_CTL0, 0x9a);
	}
	return 0;
}

int ti_deser_hub_set_external_frame_sync(struct deser_hub_dev *ti_deser_hub, int external_freq, int target_freq, int fsync_in, int fsync_out,
		int camera_trigger_gpio, int deser_trigger_gpio)
{
	int i;
	//int gpio_x;
	u8 reg_value = 0;
	u32 FS_CTL_Reg = 0;

	if (camera_trigger_gpio > 15 || deser_trigger_gpio > 15) {
		pr_info("unvailed gpio !\n");
		return 1;
	}
	if (ti_deser_hub->frame_sync_enabled) {
		pr_info("frame sync already enabled\n");
		return 1;
	}
	for (i = 0; i < ti_deser_hub->max_port; i++) {
		if (!((ti_deser_hub->src_mask) & (1 << i)))
			continue;
		if (!ti_deser_hub->chn[i].camera_bound ||
		    (ti_deser_hub->chn[i].cam_dev == NULL)) {
			continue;
		}
		reg_value = 1 << i | i << 4;
		serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, reg_value);
		/*externel GPIO_X trigger camera exposure*/
		FS_CTL_Reg = (8 + deser_trigger_gpio) << 4;
		/*enable frameSync generate*/
		FS_CTL_Reg |= 1;
		serdes_write(ti_deser_hub, 0x18, FS_CTL_Reg);
		//mapping  deser_trigger_gpio to camera_trigger
		if (camera_trigger_gpio == 1) {
			//gpio 1
			serdes_write(ti_deser_hub, TI_DESER_BC_GPIO_CTL0, (deser_trigger_gpio << 4 | 0x09));
		} else {
			serdes_write(ti_deser_hub, TI_DESER_BC_GPIO_CTL0, (deser_trigger_gpio | 0x90));
		}
	}
	if (!external_freq)
		isp_internal_trigger(target_freq, fsync_in, fsync_out);
	else
		isp_external_trigger(external_freq, target_freq, fsync_in, fsync_out);
	return 0;
}

int ti_ser_gpio_output_ctrl_set(struct camera_dev *camera, uint8_t gpio_port, uint8_t output_enable)
{
	uint8_t output_val;
	struct deser_hub_dev *deser_dev;

	if (camera == NULL)
		return -EFAULT;

	deser_dev = camera->deser_parent;

	if (deser_dev == NULL)
		return -EFAULT;
	if (ser_read(deser_dev, camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, &output_val) < 0)
		return -EFAULT;
	if (output_enable)
		output_val |= (1 << (gpio_port + 4));
	else
		output_val &= ~(1 << (gpio_port + 4));

	ser_write(deser_dev, camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, output_val);
	//pr_err("ser_write %2x : %2x: %2x",camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, output_val);
	return 0;
}

int ti_ser_gpio_input_ctrl_set(struct camera_dev *camera, uint8_t gpio_port, uint8_t input_enable)
{
	uint8_t output_val;
	struct deser_hub_dev *deser_dev;

	if (camera == NULL)
		return -EFAULT;

	deser_dev = camera->deser_parent;

	if (deser_dev == NULL)
		return -EFAULT;

	if (ser_read(deser_dev, camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, &output_val) < 0)
		return -EFAULT;

	if (input_enable)
		output_val |= (1 << (gpio_port));
	else
		output_val &= ~(1 << (gpio_port));

	ser_write(deser_dev, camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, output_val);
	//pr_err("ser_write %2x : %2x: %2x",camera->ser_alias_id, TI_SER_GPIO_INPUT_CTL, output_val);
	return 0;
}
//#define TI_DESER_FWD_CTL1 0x20
//#define TI_DESER_FWD_CTL2 0x21

int ti_deser_hub_set_fwd_sync(struct deser_hub_dev *ti_deser_hub)
{
	int ret;
	uint8_t fwd_ctl1_value;
	uint8_t fwd_ctl2_value;
	uint8_t fwd_disable_ports = 0;
	int loop;

	if (ti_deser_hub->fwd_sync_enabled) {
		pr_info("frame sync already enabled\n");
		return 0;
	}

	for (loop = 0; loop < MAX_CAMERAS_PER_SERDES; loop++) {
		if (ti_deser_hub->chn[loop].cam_dev != NULL) {
			ret = is_camera_connected(ti_deser_hub->chn[loop].cam_dev);
			if (!ret) {
				// not connected, disable it
				fwd_disable_ports |= (1 << (loop + 4)); // from bit 4
			}
		} else {
			fwd_disable_ports |= (1 << (loop + 4)); // from bit 4
		}
	}

	ret = serdes_read(ti_deser_hub, TI_DESER_FWD_CTL1);
	if (ret < 0) {
		pr_err("%s() TI_DESER_FWD_CTL1 error\n", __func__);
		return -1;
	}
	fwd_ctl1_value = (uint8_t)ret;

	ret = serdes_read(ti_deser_hub, TI_DESER_FWD_CTL2);
	if (ret < 0) {
		pr_err("%s() TI_DESER_FWD_CTL2 error\n", __func__);
		return -1;
	}
	fwd_ctl2_value = (uint8_t)ret;

	// disable all port by TI_DESER_FWD_CTL1
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL1, 0xf0);
	fwd_ctl2_value &= 0xfc; // disable round robin
	fwd_ctl2_value |= 0x14; // enable fwd sync, clear SYNC_AS_AVAIL bit
	dev_info(ti_deser_hub->dev, "%s: set FWD_CTL2 to 0x%02X\n", __func__, fwd_ctl2_value);
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL2, fwd_ctl2_value);

	fwd_ctl1_value &= 0x0f;
	fwd_ctl1_value |= 0xf0;

	dev_info(ti_deser_hub->dev, "%s: set FWD_CTL1 to 0x%02X\n", __func__, fwd_ctl1_value);
	// enable connected camera ports by TI_DESER_FWD_CTL1
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL1, fwd_ctl1_value);

	ti_deser_hub->fwd_sync_enabled = true;

	return 0;
}

int ti_deser_hub_set_frame_valid_min(struct deser_hub_dev *ti_deser_hub, uint8_t value)
{
	serdes_write(ti_deser_hub, 0xbc, value);
	return 0;
}

int write_camera_connect_status(struct deser_hub_dev *ti_deser_hub, int index, int status)
{
	uint8_t tmp;

	tmp = (1 << index | index << 4);
	serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, tmp);
	tmp = (status << 4);
	serdes_write(ti_deser_hub, TI_DESER_CAMERA_CONNECT_REG, tmp);

	return 0;
}

int read_camera_connect_status(struct deser_hub_dev *ti_deser_hub, int index)
{
	uint8_t tmp;

	tmp = (1 << index | index << 4);
	serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, tmp);
	tmp = serdes_read(ti_deser_hub, TI_DESER_SER_ID);
	if (tmp <= 0)
		return 0;
	else
		return 1;
}

static int ti_deser_hub_s_power(struct v4l2_subdev *sd, int enable)
{
	int i;
	struct deser_hub_dev *ti_deser_hub;
	u8 temp = 0;
	int ret;
	uint8_t fwd_ctl2_value = 0;
	struct deser_trigger_info *trig_info;

	ti_deser_hub = container_of(sd, struct deser_hub_dev, subdev);

	ret = serdes_read(ti_deser_hub, TI_DESER_FWD_CTL2);
	if (ret < 0) {
		pr_err("%s, read TI_DESER_FWD_CTL2 error: %d\n", __func__, ret);
		return 1;
	} else
		fwd_ctl2_value = (uint8_t)ret;

	if (strncmp(&ti_deser_hub->ctl_level[0], "fad-lis", MAX_DTS_STRING_LEN) == 0) {
		/* We enable CSI_REPLICATE for fad-lis */
		fwd_ctl2_value |= 0x80;
		dev_info(ti_deser_hub->dev, "%s: set FWD_CTL2 to 0x%02X for fad-lis\n", __func__, fwd_ctl2_value);
		serdes_write(ti_deser_hub, TI_DESER_FWD_CTL2, fwd_ctl2_value);
		return 0;
	}

	if ((ti_deser_hub->i2c_client->addr !=
	     serdes_read(ti_deser_hub, TI_DESER_DEVICE_ID) >> 1)
		|| ti_deser_hub->deser_boot_flag)
		return 0;

	/*config port info*/
	temp = serdes_read(ti_deser_hub, TI_DESER_RX_PORT_CTL);
	for (i = 0; i < ti_deser_hub->max_port; i++) {
		temp &= ~(1 << i);
		if (ti_deser_hub->src_mask & (1 << i))
			temp |= 1 << i;
	}
	/*select i2c port(0/1)*/
	if (ti_deser_hub->type == DESER_TYPE_TI960) {
		temp &= 0x0f;
		if (ti_deser_hub->i2c_port == 0)
			temp |= 0x00;
		else if (ti_deser_hub->i2c_port == 1)
			temp |= 0xf0;
	}
	serdes_write(ti_deser_hub, TI_DESER_RX_PORT_CTL, temp);
	/*config input gpio when used ,default enable all*/
	serdes_write(ti_deser_hub, TI_DESER_GPIO_INPUT_CTL, TI_DESER_GPIO_OPEN_ALL);
	/*config csi transmitter speed,default 800M
	 *TODO,configurable
	 */
	temp = 0;
	temp = serdes_read(ti_deser_hub, TI_DESER_CSI_PLL_CTL);
	temp &= ~0x3;
	temp |= 4 - (ti_deser_hub->lane_speed) / 400;
	serdes_write(ti_deser_hub, TI_DESER_CSI_PLL_CTL, temp);
	/*
	 *enable skew-calibration
	 */
	temp = TI_DESER_CSI_CALIBRATION_DEFAULT;

	//ti_deser_hub->lane_speed >= 1600?(temp|=0x01):(temp&=~(1<<0));
	temp &= ~(1 << 0);
	serdes_write(ti_deser_hub, TI_DESER_CSI_CTL2, temp);

	/*
	 *config lande num && lane mode && continue output
	 */
	temp = 0x00;

	temp |= (4 - ti_deser_hub->data_lanes_num) << 4;
	temp &= ~(3 << 2);
	temp |= 3;
	temp &= ~(1 << 6);
	if (ti_deser_hub->lane_speed >= 1600)
		temp |= 1 << 6;
	serdes_write(ti_deser_hub, TI_DESER_CSI_CTL1, temp);
	/*
	 * config FWD port
	 * TODO distinguish different serias
	 * default disable all port forward
	 * when 960,default pass to csi-2 port 0
	 * Disable forwarding when not stream_on
	 */
	temp = serdes_read(ti_deser_hub, TI_DESER_FWD_CTL1);
	if (ti_deser_hub->csi2_port == 0)
		temp &= ~(0x0f);
	else if (ti_deser_hub->csi2_port == 1)
		temp |= (0x0f);
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL1, temp);
	/*
	 *config csi transmiter mode of numbers of port
	 *set to for mode
	 */
	//serdes_write(ti_deser_hub,TI_DESER_FWD_CTL2,0x14);
	/*
	 *config every port of hub
	 *set from port 0----maxport
	 *
	 */
	fwd_ctl2_value |= 0x03;
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL2, fwd_ctl2_value);
	for (i = 0; i < ti_deser_hub->max_port; i++) {
		if (!((ti_deser_hub->src_mask) & (1 << i)))
			continue;
		if (!ti_deser_hub->chn[i].camera_bound
			|| (ti_deser_hub->chn[i].cam_dev == NULL)) {
			continue;
		}

		temp = 0;
		temp |= (1 << i | i << 4);
		serdes_write(ti_deser_hub, TI_DESER_PORT_SELECT, temp);
		serdes_write(ti_deser_hub, TI_DESER_BC_GPIO_CTL0, 0x99);
		temp = TI_DESER_BCC_DEFAULT;
		temp |= 1 << 6; // enable i2c pass through
		if (strcmp(ti_deser_hub->chn[i].cam_dev->serializer, "ti953") == 0)
			temp |= 0x6; // back channel frequency 50Mbps

		if (strcmp(ti_deser_hub->chn[i].cam_dev->serializer, "ti933") == 0)
			temp &= ~(0x7); // back channel frequency 2.5Mbps

		serdes_write(ti_deser_hub, TI_DESER_BCC_CONFIG, temp);
		if (strcmp(ti_deser_hub->chn[i].cam_dev->fpd3_mode, "csi-2") == 0) {
			temp = TI_DESER_PORT_CONFIG_DEFAULT;
			temp &= ~(3 << 0);
			serdes_write(ti_deser_hub, TI_DESER_PORT_CONFIG, temp);
			// assume camear vc id default to 0,
			// and map camera vc to different port
			temp = (CSI_VC_MAP_DEFAULT_VALUE | (ti_deser_hub->chn[i].index));
			serdes_write(ti_deser_hub, TI_DESER_PORT_CSI_VC_MAP, temp);
		}
		if (strcmp(ti_deser_hub->chn[i].cam_dev->fpd3_mode, "raw10") == 0) {
			u8 raw_data_type;

			raw_data_type = ti_deser_hub->chn[i].cam_dev->isp_data.rawinfo.dataType;
			temp = TI_DESER_PORT_CONFIG_DEFAULT;
			temp |= 3 << 0;
			serdes_write(ti_deser_hub, TI_DESER_PORT_CONFIG, temp);
			temp = TI_DESER_PROT_CFG2_DEFAULT;
			temp &= ~(1UL);
			if (ti_deser_hub->chn[i].cam_dev->fv_polarity_low)
				temp |= 1;

			serdes_write(ti_deser_hub, TI_DESER_PROT_CFG2, temp);
			serdes_write(ti_deser_hub, 0x70,
				     (ti_deser_hub->chn[i].csi_vc << 6) |
					     (raw_data_type & 0x1f));
			serdes_write(ti_deser_hub, 0xbc, TI_DESER_FRAME_VALID_MIN_DEFAULT);
			serdes_write(ti_deser_hub, 0x7d, 0xbb);
		}
		//udelay(500);
		if (strcmp(ti_deser_hub->chn[i].cam_dev->serializer, "ti953") == 0) {
			ser_write(ti_deser_hub, ti_deser_hub->chn[i].cam_dev->ser_alias_id, TI_SER_SCL_HIGH_TIME, 0x06);
			ser_write(ti_deser_hub, ti_deser_hub->chn[i].cam_dev->ser_alias_id, TI_SER_SCL_LOW_TIME, 0x0b);
		}
	}
	//config I2C Operation FastMode

	serdes_write(ti_deser_hub, TI_DESER_SCL_HIGH_TIME, 0x06);
	serdes_write(ti_deser_hub, TI_DESER_SCL_LOW_TIME, 0x0c);

	//trigger config
	trig_info = &(ti_deser_hub->trig_info);

	switch (trig_info->trigger_mode) {
	case DESER_TRIGGER_MODE_INTERNAL:
		dev_info(ti_deser_hub->dev, "%s: set Internal trigger mode\n", __func__);
		ti_deser_hub_set_internal_frame_sync(ti_deser_hub,
						     trig_info->trigger_tx_gpio, trig_info->trigger_fps);
		break;
	case DESER_TRIGGER_MODE_EXTERNAL:
		dev_info(ti_deser_hub->dev, "%s: set External trigger mode\n", __func__);
		ti_deser_hub_set_external_frame_sync(ti_deser_hub, trig_info->external_freq,
		trig_info->target_freq, trig_info->fsync_in, trig_info->fsync_out,
						     trig_info->trigger_tx_gpio, trig_info->trigger_rx_gpio);
		break;
	default:
		break;
	}

	ti_deser_hub->deser_boot_flag = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops ti_deser_hub_video_ops = {
	.s_stream = ti_deser_hub_s_stream,
};

static struct v4l2_subdev_core_ops ti_deser_hub_core_ops = {
	.s_power = ti_deser_hub_s_power,
};

static struct v4l2_subdev_ops ti_deser_hub_ops = {
	.core = &ti_deser_hub_core_ops,
	.video = &ti_deser_hub_video_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations ti_deser_hub_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int deser_notify_bound(struct v4l2_async_notifier *notifier,
				    struct v4l2_subdev *sd,
				    struct v4l2_async_subdev *asd)
{
	struct camera_dev *pcam_dev;
	struct deser_hub_dev *deser_dev;
	struct deser_channel *deser_chn;

	//pr_info("subdev name = %s, fwnode = 0x%x, asd fwnode = 0x%x\n",
	//	sd->name, sd->fwnode, asd->match.fwnode);

	pcam_dev = container_of(sd, struct camera_dev, subdev);
	deser_chn = container_of(asd, struct deser_channel, async_dev);
	deser_dev = deser_chn->deser_dev;
	deser_chn->cam_dev = pcam_dev;
	deser_chn->camera_bound = true;
	pcam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	pcam_dev->deser_parent = deser_dev;
	pcam_dev->index_in_serdes = deser_chn->index;
	//pr_info("deser channel index = %d, cam dev name = %s\n",
	//	deser_chn->index, pcam_dev->subdev.name);

	// :: todo ::
	//return media_create_pad_link(
	//	&sd->entity, 0, &csi_channel->subdev.entity, 0,
	//	MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);

	return 0;
}

static void deser_notify_unbind(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *subdev,
				      struct v4l2_async_subdev *asd)
{
	pr_info("%s()\n", __func__);
	pr_info("subdev name = %s, fwnode = 0x%x, asd fwnode = 0x%x\n",
		subdev->name, subdev->fwnode, asd->match.fwnode);
}

static const struct v4l2_async_notifier_operations deser_async_ops = {
	.bound = deser_notify_bound,
	.unbind = deser_notify_unbind,
};

static int bst_camera_get_port_info(struct deser_hub_dev *ti_deser_hub,
				    struct device_node *node)
{
	struct device_node *remote_ep = NULL;
	struct device_node *port = NULL;
	int i;
	int ret;
	int asd_index = 0;

	for (i = 0; i < ti_deser_hub->max_port; i++) {
		u32 value;
		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			pr_info("can't get port\n");
			break;
		}

		ret = of_property_read_u32(port, "virtual-channel", &value);
		if (ret == 0)
			ti_deser_hub->chn[i].csi_vc = value;
		else
			ti_deser_hub->chn[i].csi_vc = i;

		remote_ep = of_graph_get_remote_node(node, i, 0);
		if (remote_ep == NULL) {
			pr_info("can not find remote ep\n");
			break;
			//return -1;
		}
		ti_deser_hub->src_mask |= BIT(i);
		ti_deser_hub->chn[i].index = i;
		ti_deser_hub->chn[i].camera_node = remote_ep;
		ti_deser_hub->chn[i].camera_fwnode = of_fwnode_handle(remote_ep);
		ti_deser_hub->num_cameras++;
		pr_debug("remote_ep i = %d, fullname = %s, fwnode = 0x%lx\n",
			i, remote_ep->full_name,
			(unsigned long)ti_deser_hub->chn[i].camera_fwnode);
		parse_camera_serdes(ti_deser_hub, remote_ep, i);
	}
	dev_dbg(ti_deser_hub->dev, "===== ti_deser_hub->num_cameras = %d\n",
		 ti_deser_hub->num_cameras);

	if (ti_deser_hub->num_cameras == 0) {
		dev_err(ti_deser_hub->dev, "no camera under deser\n");
		return -1;
	}

	v4l2_async_notifier_init(&(ti_deser_hub->notifier));
	ti_deser_hub->notifier.ops = &deser_async_ops;
	asd_index = 0;
	for (i = 0; i < ti_deser_hub->max_port; i++) {
		if (ti_deser_hub->chn[i].camera_fwnode == NULL) {
			pr_info("i = %d, camera not connected\n", i);
			continue;
		}

		ti_deser_hub->chn[i].deser_dev = ti_deser_hub;
		ti_deser_hub->chn[i].async_dev.match_type =
			V4L2_ASYNC_MATCH_FWNODE;
		ti_deser_hub->chn[i].async_dev.match.fwnode =
			ti_deser_hub->chn[i].camera_fwnode;
		v4l2_async_notifier_add_subdev(&ti_deser_hub->notifier, &(ti_deser_hub->chn[i].async_dev));
		asd_index++;
	}
	ret = v4l2_async_subdev_notifier_register(&ti_deser_hub->subdev,
						  &(ti_deser_hub->notifier));
	if (ret < 0)
		pr_err("v4l2_async_subdev_notifier_register register failed\n");

	return 0;
}

static int parse_des_source_port(struct deser_hub_dev *ti_deser_hub,
					struct device_node *csi2_dt)
{
	int ret;
	struct device_node *csi2_port = NULL;
	struct v4l2_fwnode_endpoint src_endpoint;

	csi2_port = of_graph_get_port_by_id(csi2_dt, 0);
	/*if NULL, means this port is not set in dts*/
	if (csi2_port == NULL) {
		pr_err("get port failed\n");
		return -1;
	}

	src_endpoint.bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(csi2_port),
					 &src_endpoint);
	ti_deser_hub->data_lanes_num = src_endpoint.bus.mipi_csi2.num_data_lanes;

	//pr_info("port id is %d,bus is %d,num data lane is %d\n",
	//       src_endpoint.base.id, src_endpoint.bus_type,
	//       src_endpoint.bus.mipi_csi2.num_data_lanes);

	return 0;
}

static int parse_ti_deser_hub_dt(struct deser_hub_dev *ti_deser_hub)
{
	int err = 0;
	struct device_node *csi2_dt = NULL;
	int lane_speed = 0;
	int i2c_port = 0;
	int csi2_port = 0;
	struct v4l2_subdev *sd;
	struct device_node *node = ti_deser_hub->dev->of_node;
	const char *type_name = NULL;
	const char *ctl_level = NULL;
	int ret;
        u32 port_num;

        if (!node)
		return -EINVAL;

	ti_deser_hub->src_mask = 0x0;
	ti_deser_hub->type = DESER_TYPE_INVALID;
	if (!of_property_read_string(node, "type", &type_name)) {
		if (type_name != NULL) {
			strncpy(ti_deser_hub->name, type_name, MAX_DESER_NAME_LEN);
			if (strncmp(type_name, "ti954", 5) == 0) {
				ti_deser_hub->src_mask = 0x03;
				ti_deser_hub->max_port = 2;
				ti_deser_hub->type = DESER_TYPE_TI954;
			} else if (strncmp(type_name, "ti960", 5) == 0) {
				ti_deser_hub->src_mask = 0x0f;
				ti_deser_hub->max_port = 4;
				ti_deser_hub->type = DESER_TYPE_TI960;
			}
		}
	}
        /* For 2X2 MIPI, we only use 2 port for 2 lane */
        if (of_property_read_u32(node, "port-num", &port_num) == 0)
		ti_deser_hub->max_port = port_num;

	err = of_property_read_u32(node, "lane-speed", &lane_speed);
        if (err) {
		dev_err(ti_deser_hub->dev, " Failed to find lane-speed\n");
		lane_speed = 800;
	}

	// for single soc, ctl_level string is NULL
	err = of_property_read_string(node, "ctl-mode", &ctl_level);
	if (err)
		dev_dbg(ti_deser_hub->dev, " Failed to find ctl-mode\n");
	else
		strncpy(&ti_deser_hub->ctl_level[0], ctl_level, MAX_DTS_STRING_LEN);

	err = of_property_read_u32(node, "i2c-port", &i2c_port);
	if (err) {
		pr_info(" Failed to find i2c-port, set to 0\n");
		i2c_port = 0;
	}
	err = of_property_read_u32(node, "csi2-port", &csi2_port);
	if (err) {
		pr_info(" Failed to find csi2-port, set to 0\n");
		csi2_port = 0;
	}

	err = of_property_read_u32(node, "trigger-mode", &(ti_deser_hub->trig_info.trigger_mode));
	if (err) {
		pr_info(" Failed to find trigger-mode, set to INTERNAL_MODE\n");
		ti_deser_hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_NONE;
	}
	err = of_property_read_u32(node, "trigger-fps", &(ti_deser_hub->trig_info.trigger_fps));
	if (err) {
		pr_info(" Failed to find trigger-fps, set to 0\n");
		ti_deser_hub->trig_info.trigger_fps = 0;
	}
	err = of_property_read_u32(node, "trigger-tx-gpio", &(ti_deser_hub->trig_info.trigger_tx_gpio));
	if (err) {
		pr_info(" Failed to find trigger-tx-gpio, set to 0\n");
		ti_deser_hub->trig_info.trigger_tx_gpio = 0;
	}
	err = of_property_read_u32(node, "trigger-rx-gpio", &(ti_deser_hub->trig_info.trigger_rx_gpio));
	if (err) {
		pr_info(" Failed to find trigger-rx-gpio, set to 0\n");
		ti_deser_hub->trig_info.trigger_rx_gpio = 0;
	}
	ti_deser_hub->lane_speed = lane_speed;
	ti_deser_hub->i2c_port = i2c_port;
	ti_deser_hub->csi2_port = csi2_port;

	csi2_dt = of_get_child_by_name(node, "csi-link");
	if (csi2_dt == NULL) {
		dev_err(ti_deser_hub->dev, "get csi-link error\n");
		return -EINVAL;
	}

	sd = &ti_deser_hub->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &ti_deser_hub_ops);
	sd->dev = ti_deser_hub->dev;
	v4l2_set_subdevdata(sd, ti_deser_hub);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &ti_deser_hub_media_ops;
	sd->fwnode = of_fwnode_handle(csi2_dt); // csi device find the csi-link
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(ti_deser_hub->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		pr_err("failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	// enable i2c for sensor i2c config
	/*configure iic; open iic master mode bit[5] 1*/
	if (ti_deser_hub->ctl_mode != FAD_LIS_MODE) {
		serdes_write(ti_deser_hub, TI_DESER_GENERAL_CONFIGURE, CONFIG_IIC_MASTER);
	}
	bst_camera_get_port_info(ti_deser_hub, node);
	parse_des_source_port(ti_deser_hub, csi2_dt);

	return 0;
}

/*
 *for generate LP-11,do it before MIPI config
 */
static void ds954_config(struct deser_hub_dev *ti_deser_hub)
{
	serdes_write(ti_deser_hub, 0x20, 0x30);
	serdes_write(ti_deser_hub, 0x33, 0x03);
	mdelay(10);
}

static void ds960_config(struct deser_hub_dev *ti_deser_hub)
{
	int ret;

	ret = serdes_read(ti_deser_hub, TI_DESER_DEVICE_ID);
	if (ret < 0) {
		dev_err(ti_deser_hub->dev,
			"read 960 device id error\n");
		return;
	}

	/* Disable forwarding of all RX ports enter LP-11 state */
	serdes_write(ti_deser_hub, TI_DESER_FWD_CTL1, 0xF0);
	// config tx port 0 & 1
	serdes_write(ti_deser_hub, 0x32, 0x13);
	serdes_write(
		ti_deser_hub, 0x33,
		0x03 | (ti_deser_lanes_num_to_bits(ti_deser_hub->data_lanes_num)
			<< 4));

	mdelay(10);
}

static void power_reset(struct i2c_client *client)
{
	int pdb_gpio = -1;
	int ret = 0;
	dev_info(&client->dev, "power_reset\n");
	pdb_gpio = of_get_named_gpio(client->dev.of_node, "pdb-gpio", 0);
	if (gpio_is_valid(pdb_gpio)) {
		ret = devm_gpio_request(&client->dev, pdb_gpio,
					dev_name(&client->dev));
		if (ret) {
			dev_err(&client->dev,
				"failed to request gpio %d\n", pdb_gpio);
			return;
		}

		gpio_direction_output(pdb_gpio, 0);
		mdelay(5);
		gpio_direction_output(pdb_gpio, 1);
		mdelay(10);
	}
	devm_gpio_free(&client->dev, pdb_gpio);
}

static int ti_deser_hub_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct deser_hub_dev *ti_deser_hub;
	int ret;
	int err;
	int device_id = 0;
	const char *ctl_level = NULL;

	ti_deser_hub = devm_kzalloc(&client->dev, sizeof(struct deser_hub_dev),
				 GFP_KERNEL);
	if (!ti_deser_hub)
		return -ENOMEM;
	ti_deser_hub->i2c_client = client;
	ti_deser_hub->dev = dev;
	ti_deser_hub->deser_boot_flag = false;
	ti_deser_hub->internal_trigger_sync = ti_deser_hub_set_internal_frame_sync;
	ti_deser_hub->external_trigger_sync = ti_deser_hub_set_external_frame_sync;

	mutex_init(&ti_deser_hub->deser_mutex);
	
	// for single soc, ctl_level string is NULL
	err = of_property_read_string(ti_deser_hub->dev->of_node, "ctl-mode", &ctl_level);
	if (!err) {
		dev_info(ti_deser_hub->dev, "get ctl-mode success\n");
	
		if (strncmp(ctl_level, "fad-lis", 7) == 0) {
			dev_info(ti_deser_hub->dev, "FAD lis mode\n");
			ti_deser_hub->ctl_mode = FAD_LIS_MODE;
			ret = parse_ti_deser_hub_dt(ti_deser_hub);
			if (ret) {
				dev_err(ti_deser_hub->dev, "ERROR: parse_ti_deser_hub_dt\n");
			}
			return 0;
		}
	}
	
	power_reset(client);

	device_id = serdes_read(ti_deser_hub, 0x00);
	if(device_id != (client->addr << 1)) {
		pr_err("Device not match %x %x\n", device_id, client->addr <<1 );
	}

	ret = parse_ti_deser_hub_dt(ti_deser_hub);
	if (ret) {
		dev_err(ti_deser_hub->dev, "ERROR: parse_ti_deser_hub_dt\n");
		// :: need to cleanup if return failed, so always return success by temporary		
		//return 0;
	}

	// I2c Devices Config
	switch (ti_deser_hub->type) {
	case DESER_TYPE_TI954:
		ds954_config(ti_deser_hub);
		break;
	case DESER_TYPE_TI960:
		ds960_config(ti_deser_hub);
		break;
	default:
		dev_err(ti_deser_hub->dev, "no support deser\n");
	}
	/*configure iic; open iic master mode bit[5] 1*/
	serdes_write(ti_deser_hub, TI_DESER_GENERAL_CONFIGURE, CONFIG_IIC_MASTER);

	// :: need to cleanup if return failed, so always return success by temporary
	return 0;
}

static int ti_deser_hub_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ti_deser_hub_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
static const struct of_device_id ti_deser_hub_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ti_deser_hub_of_match);
MODULE_DEVICE_TABLE(i2c, ti_deser_hub_id);

static struct i2c_driver ti_deser_hub_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ti_deser_hub_of_match),
	},
	.probe		= ti_deser_hub_probe,
	.remove		= ti_deser_hub_remove,
	.id_table	= ti_deser_hub_id,
};

module_i2c_driver(ti_deser_hub_driver);

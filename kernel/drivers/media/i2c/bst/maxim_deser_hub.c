// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM Derser Hub Deserializer for BST Deserializer Driver
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

#include "maxim_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,maxim-deser-hub"

#define MAXIM_HUB_DEBUG
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

static int set_max96712_csi_phy_speed(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub;
	int lane_speed;

	hub = &priv->hub;
	//Set mipi dphy lane speed [4:0]
	if (priv->lane_speed == 0) {
		dev_err(hub->dev, "lane speed is 0\n");
		return -1;
	}
	lane_speed =  0x20 | (priv->lane_speed / 100);

	max96712_reg_write(hub, 0x415, lane_speed);
	max96712_reg_write(hub, 0x418, lane_speed);
	max96712_reg_write(hub, 0x41b, lane_speed);
	max96712_reg_write(hub, 0x41e, lane_speed);

	return 0;
}

static int max_gmsl2_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
	int offset = 0;
	int i = 0;
	int evb_cmp;

	write_reg(hub->i2c_client, 0x40b, 0x00); //Disable MIPI CSI-2
	write_reg(hub->i2c_client, 0x06, 0xFF);	// Enable all 4 Links in
	//write_reg(hub->i2c_client, 0x06, 0xF0);
	// mdelay(10);
	// Video Pipe Selection
	write_reg(hub->i2c_client, 0xF0, 0x62); //40/62 // pipe X in link B to video pipe 1)pipe X in link A to video pipe 0
	write_reg(hub->i2c_client, 0xF1, 0xea); //ea/c8 // pipe X in link D to video pipe 3)pipe X in link C to video pipe 2
	//register16_write(hub,0xF2,0x51);	// pipe Y in link B to video pipe 4)pipe Y in link A to video pipe 5
	//register16_write(hub,0xF3,0xD9); // pipe Y in link D to video pipe 6)pipe Y in link C to video pipe 7
	write_reg(hub->i2c_client, 0xF4, 0x0F); // Turn on 4 pipes

	// Efficiency updates (disable HEARTBEAT Mode); for image data pipes 0-3
	write_reg(hub->i2c_client, 0x0106, 0x0A);
	write_reg(hub->i2c_client, 0x0118, 0x0A);
	write_reg(hub->i2c_client, 0x012A, 0x0A);
	write_reg(hub->i2c_client, 0x013C, 0x0A);

	for (i = 0; i < hub->max_port; i++) {
		offset = 0x40 * i;
		// YUV422 8bit);video pipe 0,1,2,3);map FS/FE
		write_reg(hub->i2c_client, 0x090B + offset, 0x07);
		write_reg(hub->i2c_client, 0x092D + offset, 0x15);// map to MIPI Controller 1
		write_reg(hub->i2c_client, 0x090D + offset, 0x2d);
		write_reg(hub->i2c_client, 0x090E + offset, 0x2d|(i<<6));// map to VC0-3
		write_reg(hub->i2c_client, 0x090F + offset, 0x00);
		write_reg(hub->i2c_client, 0x0910 + offset, i<<6);
		write_reg(hub->i2c_client, 0x0911 + offset, 0x01);
		write_reg(hub->i2c_client, 0x0912 + offset, (i<<6)|0x01);
	}
	// MIPI PHY Setting
	// Set Des in 2x4 mode
	write_reg(hub->i2c_client, 0x08A0, 0x04);
	// Set Lane Mapping for 4-lane port A
	write_reg(hub->i2c_client, 0x08A3, 0xE4);
	write_reg(hub->i2c_client, 0x08A4, 0xE4);
	// Set 4 lane D-PHY
	write_reg(hub->i2c_client, 0x090A, 0xC0);
	write_reg(hub->i2c_client, 0x094A, 0xC0);
	write_reg(hub->i2c_client, 0x098A, 0xC0);
	write_reg(hub->i2c_client, 0x09CA, 0xC0);
	// Turn on MIPI PHYs
	evb_cmp = strncmp(&hub->ctl_level[0], "evb", MAX_DTS_STRING_LEN);
	// different HW board may have different setting
	if (evb_cmp == 0) {
		// evb has special HW design
		write_reg(hub->i2c_client, 0x08A2, 0xF0);
	} else {
		// FAD case
		write_reg(hub->i2c_client, 0x08A2, 0xF4);
	}
	// Hold DPLL in reset (config_soft_rst_n = 0); before changing the rate
	write_reg(hub->i2c_client, 0x1C00, 0xF4);
	write_reg(hub->i2c_client, 0x1D00, 0xF4);
	write_reg(hub->i2c_client, 0x1E00, 0xF4);
	write_reg(hub->i2c_client, 0x1F00, 0xF4);
	// Set Data rate to be 1500Mbps/lane for port A and enable software override
	set_max96712_csi_phy_speed(priv);
	// Release reset to DPLL (config_soft_rst_n = 1);
	write_reg(hub->i2c_client, 0x1C00, 0xF5);
	write_reg(hub->i2c_client, 0x1D00, 0xF5);
	write_reg(hub->i2c_client, 0x1E00, 0xF5);
	write_reg(hub->i2c_client, 0x1F00, 0xF5);
	// PHY 2 copy PHY 0 output
	//write_reg(hub->i2c_client,0x08A9,0xC8);	//C0
	return 0;
}

static int max_channel_open(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Enbale all link*/
	/*Reset one-shot*/
	max96712_reg_write(hub, 0x0018, 0x0F);
	udelay(1*1000);
	/*open GSML1 LINK A B C D*/
	max96712_reg_write(hub, 0x0006, 0xFF);
	udelay(1*1000);
	/*enable mipi output*/
	//modify csi output close
	max96712_reg_write(hub, 0x040B, 0x40);
	return 0;
}


static int modify_serdes_address(struct maxim_hub_priv *priv)
{
	int i;
	struct i2c_adapter *adap;
	struct deser_hub_dev *hub;
	uint8_t mask, value;

	hub = &priv->hub;
	adap = hub->i2c_client->adapter;
#ifdef MAXIM_HUB_DEBUG
	dev_info(hub->dev, "%s() %d\n", __func__, __LINE__);
#endif
	//disable all remote port
	max96712_reg_write(hub, 0x03, 0xff);
	mdelay(200);
	//modify serlias i2c address
	for (i = 0; i < hub->max_port; i++) {
		if (!hub->chn[i].camera_bound || (hub->chn[i].cam_dev == NULL)) {
			pr_info("%s() cam_dev [%d] is NULL, break\n", __func__, i);
			continue;
		}
		if (!is_gmsl2_video_connected(hub, i))
			continue;
		//0x03,disable remote control channel register
		if (i == 0)
			max96712_reg_write(hub, 0x03, 0xfe);//enable port0,open linka,close linkb,linkc,linkd
		else if (i == 1)
			max96712_reg_write(hub, 0x03, 0xfb);//enable port0,open linkb,close linka,linkc,linkd
		else if (i == 2)
			max96712_reg_write(hub, 0x03, 0xef);//enable port0,open linkc,close linka,linkb,linkd
		else
			max96712_reg_write(hub, 0x03, 0xbf);//enable port0,open linkd,close linka,linkb,linkc

#ifdef MAXIM_HUB_DEBUG
		dev_info(hub->dev, "===== value = %x, mask = %x\n", value, mask);
		dev_info(hub->dev, "modify test ser_addr=0x%x,ser_alias=0x%x\n",
				priv->ser_addr[i], priv->ser_alias_addr[i]);
		dev_info(hub->dev, "modify test ser_alias_addr=0x%x,sensor_alias_addr=0x%x\n",
				priv->ser_alias_addr[i], priv->sensor_alias_addr[i]);
#endif
		write_register(adap, priv->ser_addr[i], 0x0000, (priv->ser_alias_addr[i] << 1));
		write_register(adap, priv->ser_alias_addr[i], 0x0042, (priv->sensor_alias_addr[i] << 1));
		write_register(adap, priv->ser_alias_addr[i], 0x0043, (priv->sensor_addr[i] << 1));
	}
	max96712_reg_write(hub, 0x03, 0xaa);//enable port0,open all link
	mdelay(20);
	return 0;
}

static void maxim_hub_set_mipi_output(struct deser_hub_dev *hub, bool enable)
{
	uint8_t is_enable, value;

	if (enable)
		is_enable = 0x01;
	else
		is_enable = 0x00;

	switch (hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_reg_read(hub, 0x040B, &value);
		max96712_reg_write(hub, 0x040B, value | (is_enable << 1));
		break;
	default:
		dev_err(hub->dev, "%s() Not Support\n", __func__);
		break;
	}
}

static void max_open_deskew(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
	//mipi0
	max96712_reg_write(hub, 0x903, 0x80);
	max96712_reg_write(hub, 0x904, 0x91);
	//mipi1
	max96712_reg_write(hub, 0x943, 0x80);
	max96712_reg_write(hub, 0x944, 0x91);
	//mipi2
	max96712_reg_write(hub, 0x983, 0x80);
	max96712_reg_write(hub, 0x984, 0x91);
	//mipi3
	max96712_reg_write(hub, 0x9c3, 0x80);
	max96712_reg_write(hub, 0x9c4, 0x91);
}

static int config_serial_fsync_gpio(struct maxim_hub_priv *priv, struct deser_trigger_info trig_info)
{
	int i;
	struct i2c_adapter *adap;
	struct deser_hub_dev *hub;

	hub = &priv->hub;
	adap = hub->i2c_client->adapter;
	for (i = 0; i < hub->max_port; i++) {
		if (!is_gmsl2_video_connected(hub, i))
			continue;
		if (trig_info.trigger_rx_gpio == 0) {
			write_register(adap, priv->ser_alias_addr[i], 0x02be, 0x84);
			write_register(adap, priv->ser_alias_addr[i], 0x02c0, 0x00);
		} else if (trig_info.trigger_rx_gpio == 10) {
			write_register(adap, priv->ser_alias_addr[i], 0x02be, 0x84);
			write_register(adap, priv->ser_alias_addr[i], 0x02c0, 0x0a);
		}
	}
	return 0;
}
static int max96712_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;
	int fsync_period;
	uint8_t gpi_conf, gpio_id;

	if (trig_info.trigger_mode == DESER_TRIGGER_MODE_NONE)
		return 0;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	if (trig_info.trigger_tx_gpio < 0) {
		dev_err(hub->dev, "GPIO ID select Error\n");
		return 1;
	}
	/*Set Manual mode*/
	max96712_reg_write(hub, 0x04A0, 0x04);
	/*Turn off auto master link selection*/
	max96712_reg_write(hub, 0x04A2, 0x00);
	/*Disable overlap window*/
	max96712_reg_write(hub, 0x04AA, 0x00);
	max96712_reg_write(hub, 0x04AB, 0x00);

	switch (trig_info.trigger_mode) {
	case DESER_TRIGGER_MODE_NONE:
			dev_info(hub->dev, "NONE TRIGGER MODE\n");
			//Setting default fsync mode
			max96712_reg_write(hub, 0x04A0, 0x0d);
			break;
	case DESER_TRIGGER_MODE_INTERNAL:
			dev_info(hub->dev, "INTERNAL TRIGGER MODE\n");
			/*
			 * Fsync config in diff link mode
			 */
			if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
				gpi_conf = 0;
				gpi_conf = 0x31 | (trig_info.trigger_tx_gpio << 6);

				max96712_reg_write(hub, 0x04AF, 0x4F);
				max96712_reg_write(hub, 0x0B08, gpi_conf);
				max96712_reg_write(hub, 0x0C08, gpi_conf);
				max96712_reg_write(hub, 0x0D08, gpi_conf);
				max96712_reg_write(hub, 0x0E08, gpi_conf);
			} else if (priv->link_mode == MAXIM_LINK_MODE_GMSL2) {
				max96712_reg_write(hub, 0x04AF, 0xd0);
			}
			/*PCLK config*/
			/*MAXIM internal crystal oscillator is 25Mhz*/
			fsync_period = 25000000 / trig_info.trigger_fps;
			max96712_reg_write(hub, 0x04A7, fsync_period >> 16);
			max96712_reg_write(hub, 0x04A6, (fsync_period >> 8) & 0xff);
			max96712_reg_write(hub, 0x04A5, fsync_period & 0xff);
			/*
			 * gpio id used for transmitting fsync signal [7:3]
			 */
			gpio_id = 0;
			gpio_id = 0x00 | (trig_info.trigger_tx_gpio << 3);
			max96712_reg_write(hub, 0x04b1, gpio_id);

			break;
	case DESER_TRIGGER_MODE_EXTERNAL:
			dev_info(hub->dev, "EXTERNAL TRIGGER MODE\n");
			/*Set Externel mode*/

			if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
				max96712_reg_write(hub, 0x04AF, 0x1F);
				gpi_conf = 0;
				gpi_conf = 0x21 | (trig_info.trigger_rx_gpio << 6);
				max96712_reg_write(hub, 0x0B08, gpi_conf);
				max96712_reg_write(hub, 0x0C08, gpi_conf);
				max96712_reg_write(hub, 0x0D08, gpi_conf);
				max96712_reg_write(hub, 0x0E08, gpi_conf);
			} else if (priv->link_mode == MAXIM_LINK_MODE_GMSL2) {
				if (trig_info.trigger_rx_gpio == 10) {
				//set gpio10 for fadv2 soc a
					max96712_reg_write(hub, 0x0320, 0x83);
					max96712_reg_write(hub, 0x0357, 0x2a);
					max96712_reg_write(hub, 0x038d, 0x2a);
					max96712_reg_write(hub, 0x03c4, 0x2a);
				} else if (trig_info.trigger_rx_gpio == 0) {
					max96712_reg_write(hub, 0x0300, 0x83);//306
					max96712_reg_write(hub, 0x0337, 0x20);//33d
					max96712_reg_write(hub, 0x036d, 0x20);//374
					max96712_reg_write(hub, 0x03A4, 0x20);//3aa
				}
			}
			//set serial fysnc gpio
			config_serial_fsync_gpio(priv, trig_info);
			break;
	default:
			pr_err("Wrong trigger mode!");
			return 1;
	}
	return 0;
}

static int config_max_fix_rate(struct maxim_hub_priv *priv)
{
	uint8_t value;
	struct deser_hub_dev *hub =  &priv->hub;
	//csi_out disable
	max96712_reg_write(hub, 0x040b, 0x00);
	max96712_reg_write(hub, 0x0010, 0x00);
	max96712_reg_write(hub, 0x0011, 0x00);
	//linka 3gpbs or 6gpbs
	if (priv->linkrx_rate[0] == 3)
		max96712_reg_write(hub, 0x0010, 0x01);
	else if (priv->linkrx_rate[0] == 6)
		max96712_reg_write(hub, 0x0010, 0x02);
	//linkb 3gpbs or 6gpbs
	if (priv->linkrx_rate[1] == 3) {
		max96712_reg_read(hub, 0x0010, &value);
		max96712_reg_write(hub, 0x0010, value|0x10);
	} else if (priv->linkrx_rate[1] == 6) {
		max96712_reg_read(hub, 0x0010, &value);
		max96712_reg_write(hub, 0x0010, value|0x20);
	}
	//linkc 3gpbs or 6gpbs
	if (priv->linkrx_rate[2] == 3)
		max96712_reg_write(hub, 0x0011, 0x01);
	else if (priv->linkrx_rate[2] == 6)
		max96712_reg_write(hub, 0x0011, 0x02);
	//linkd 3gpbs or 6gpbs
	if (priv->linkrx_rate[3] == 3) {
		max96712_reg_read(hub, 0x0011, &value);
		max96712_reg_write(hub, 0x0011, value|0x10);
	} else if (priv->linkrx_rate[3] == 6) {
		max96712_reg_read(hub, 0x0011, &value);
		max96712_reg_write(hub, 0x0011, value|0x20);
	}
	max96712_reg_write(hub, 0x0006, 0xff);
	max96712_reg_write(hub, 0x0018, 0x0f);
	//config data_type
	if (priv->data_type == 0x1e) {
		max96712_reg_write(hub, 0x090d, 0x1e);
		max96712_reg_write(hub, 0x090e, 0x1e);
	} else if (priv->data_type == 0x2d) {
		max96712_reg_write(hub, 0x090d, 0x2d);
		max96712_reg_write(hub, 0x090e, 0x2d);
	}
	return 0;
}

static void max967XX_replicate_mode(struct deser_hub_dev *hub)
{
	dev_info(hub->dev, "%s() line:%d\n", __func__, __LINE__);
	/* open 0~3 PHY
	 * max96712_reg_write(hub, 0x08A2, 0xF0);
	 */
	//copy port A to port B
	max96712_reg_write(hub, 0x8A9, 0xC8);
}

static int max96712_gsml2_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;

	config_max_fix_rate(priv);

	max_gmsl2_config(priv);

	max967XX_replicate_mode(hub);

	max96712_fsync_config(priv, priv->hub.trig_info);

	modify_serdes_address(priv);

	max_channel_open(priv);

	if (priv->lane_speed >= 1600)
		max_open_deskew(priv);

	return 0;
}

static void maxim_hub_open_link(struct deser_hub_dev *hub)
{
	int link_bit = 0;
	int i;
	struct maxim_hub_priv *priv =
		container_of(hub, struct maxim_hub_priv, hub);

	for (i = 0; i < MAX_CAMERAS_PER_SERDES; i++) {
		if (is_gmsl2_video_connected(hub, i))
			link_bit |= 1 << i;
	}

	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
		max96712_reg_write(hub, 0x06, 0x0 | link_bit);
	else
		max96712_reg_write(hub, 0x06, 0xf0 | link_bit);
}

static int maxim_hub_s_stream(struct v4l2_subdev *subdev, int enable)
{
	int subdev_port, index;
	struct deser_hub_dev *hub;

	pr_info("%s() %d, set stream", __func__, __LINE__);
	hub = container_of(subdev, struct deser_hub_dev, subdev);
	subdev_port = (enable & MAXIM_STREAM_SUB_PORT_MASK) >> 4;
	index = subdev_port % hub->max_port;
	// is_enable = (enable & MAXIM_STREAM_ENABLE_MASK) == 1 ? 1 : 0;

	pr_info("%s() device index:[%d]\n", __func__, index);
	if (!hub->chn[index].camera_bound || (hub->chn[index].cam_dev == NULL)) {
		pr_err("%s() cam_dev [%d] is NULL\n", __func__, index);
		return 1;
	}
	mutex_lock(&hub->deser_mutex);

	/* enable CSI output*/
	if (hub->deser_stream_flag == false) {
		//open enabled Link
		maxim_hub_open_link(hub);
		mdelay(5);
		maxim_hub_set_mipi_output(hub, true);
		hub->deser_stream_flag = true;
	}

	mutex_unlock(&hub->deser_mutex);
	return 0;
}

static int max96712_preinit(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Begin preset registers*/
	/*Reset Chip*/
	max96712_reg_write(hub, 0x0013, 0x75);
	mdelay(2);
	/*Disable mipi output*/
	max96712_reg_write(hub, 0x040B, 0x00);
	/*Disable mipi input*/
	max96712_reg_write(hub, 0x0006, 0x00);
	//one-shot Reset
	max96712_reg_write(hub, 0x0018, 0x0F);
	// /*Set i2c fast plus mode*/
	// max96712_reg_write(hub, 0x0640, 0x00);
	/*Turn on HIM*/
	max96712_reg_write(hub, 0x0B06, 0xEF);
	max96712_reg_write(hub, 0x0C06, 0xEF);
	max96712_reg_write(hub, 0x0D06, 0xEF);
	max96712_reg_write(hub, 0x0E06, 0xEF);
	/*Disable HS/VS processing*/
	max96712_reg_write(hub, 0x0B0F, 0x01);
	max96712_reg_write(hub, 0x0C0F, 0x01);
	max96712_reg_write(hub, 0x0D0F, 0x01);
	max96712_reg_write(hub, 0x0E0F, 0x01);
	/*Enable HS/VS processing*/
	max96712_reg_write(hub, 0x0B07, 0x84);
	max96712_reg_write(hub, 0x0C07, 0x84);
	max96712_reg_write(hub, 0x0D07, 0x84);
	max96712_reg_write(hub, 0x0E07, 0x84);
	mdelay(5);
	return 0;
}

static int max96712_gsm1_mipi_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
	char data_type;
	char bpp;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	max96712_reg_write(hub, 0x041A, 0xF0);
	// mipi output set at 2*4
	max96712_reg_write(hub, 0x08A0, 0x04);
	// enable 2 output phy
	max96712_reg_write(hub, 0x08A2, 0x30);
	// enable 4 pipeline
	max96712_reg_write(hub, 0x00F4, 0x0F);
	//phy1 lane no. at 4
	max96712_reg_write(hub, 0x094A, 0xC0);
	//phy0/1 lane map #default:4E
	max96712_reg_write(hub, 0x08A3, 0xE4);
	//VC for pipe line 0/1 set as 0 and //all =0
	max96712_reg_write(hub, 0x040C, 0x00);
	//VC for pipe line 2/3 set as 0 and //all=0
	max96712_reg_write(hub, 0x040D, 0x00);
	/*
	 * Setting Datatype
	 */

	data_type = priv->data_type;
	max96712_reg_write(hub, 0x040E, ((data_type >> 4) << 6) | data_type);
	//dt for pipe line 1/2 set as YUV422
	max96712_reg_write(hub, 0x040F, ((data_type >> 2) << 4) | (data_type & 0xf));
	//dt for pipe line 2/3 set as YUV422
	max96712_reg_write(hub, 0x0410, (data_type << 2) | (data_type & 0x03));

	//BPP for pipe line 1/2 set as 1E(YUV422)
	if (priv->data_type == 0x1E)
		bpp = 0x8;
	else if (priv->data_type == 0x2B)
		bpp = 0xA;
	else if (priv->data_type == 0x2C)
		bpp = 0xC;
	else if (priv->data_type == 0x2D)
		bpp = 0xE;
	else if (priv->data_type == 0x2E)
		bpp = 0x10;

	max96712_reg_write(hub, 0x0411, ((bpp >> 2) << 5) | (bpp & 0x1f));
	max96712_reg_write(hub, 0x0412, (bpp << 2) | (bpp & 0x03));
	/*
	 * set CSI PHY Speed
	 */
	set_max96712_csi_phy_speed(priv);
	//following is for pipeline mapping. Needed for FCFS mode, and not needed for concatenate mode.
	max96712_reg_write(hub, 0x090B, 0x07);//enable 3 mappings for pipeline 0
	max96712_reg_write(hub, 0x092D, 0x15);// map to destination controller 1
	max96712_reg_write(hub, 0x090F, 0x00);//source dt(00,frame start) and vc(0)
	max96712_reg_write(hub, 0x0910, 0x00);//destination dt(00,frame start) and vc(0)
	max96712_reg_write(hub, 0x0911, 0x01);//source dt(01,frame end) and vc(0)
	max96712_reg_write(hub, 0x0912, 0x01);//destination dt(01,frame end) and vc(0)
	max96712_reg_write(hub, 0x094B, 0x07);//enable 3 mappings for pipeline 1
	max96712_reg_write(hub, 0x096D, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x094F, 0x00);//Source VC=0
	max96712_reg_write(hub, 0x0950, 0x40);//DES: VC=1
	max96712_reg_write(hub, 0x0951, 0x01);//Source VC=1
	max96712_reg_write(hub, 0x0952, 0x41);
	max96712_reg_write(hub, 0x098B, 0x07);//enable 3 mappings for pipeline 2
	max96712_reg_write(hub, 0x09AD, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x098F, 0x00);//SOURCE DT AND VC
	max96712_reg_write(hub, 0x0990, 0x80);//DES DT AND VC
	max96712_reg_write(hub, 0x0991, 0x01);//SOURCE DT AND VC
	max96712_reg_write(hub, 0x0992, 0x81);//DES DT AND VC
	max96712_reg_write(hub, 0x09CB, 0x07);//enable 3 mappings for pipeline 3
	max96712_reg_write(hub, 0x09ED, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x09CF, 0x00);//source dt and vc
	max96712_reg_write(hub, 0x09D0, 0xC0);//des dt and vc
	max96712_reg_write(hub, 0x09D1, 0x01);//source dt and vc
	max96712_reg_write(hub, 0x09D2, 0xC1);//des dt and vc

	max96712_reg_write(hub, 0x090D, priv->data_type);//source data type (1E) and VC(0)
	max96712_reg_write(hub, 0x090E, priv->data_type);//destination dt(1E) and vc(0)
	max96712_reg_write(hub, 0x094D, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x094E, (0x40 | priv->data_type));//DES: VC=1
	max96712_reg_write(hub, 0x098D, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x098E, (0x80 | priv->data_type));//DES DTVC=2
	max96712_reg_write(hub, 0x09CD, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x09CE, (0xC0 | priv->data_type));//des vc =3
	return 0;
}

static int max96701_config(struct maxim_hub_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	for (i = 0; i < hub->max_port; i++) {
		//open Link
		max96712_reg_write(hub, 0x0006, (1 << i));
		mdelay(5);
		//set Link
		ser_write(hub, priv->ser_addr[i], 0x47, 0x2d);
		ser_write(hub, priv->ser_addr[i], 0x43, 0x25);
		ser_write(hub, priv->ser_addr[i], 0x67, 0xc4);
		ser_write(hub, priv->ser_addr[i], 0x0f, 0xbf);
		ser_write(hub, priv->ser_addr[i], 0x00,
			  (priv->ser_alias_addr[i] << 1));
		ser_write(hub, priv->ser_alias_addr[i], 0x01, priv->des_addr << 1);
		//ser_write(hub, priv->ser_addr[i], 0x0b, MAX96701_BROADCAST << 1);
		ser_write(hub, priv->ser_alias_addr[i], 0x0C, priv->ser_alias_addr[i] << 1);
		mdelay(10);
		ser_write(hub, priv->ser_alias_addr[i], 0x04, 0x83);
		mdelay(5);
	}

	return 0;
}

static int max96705_config(struct maxim_hub_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	for (i = 0; i < hub->max_port; i++) {
		{
			//chose Link
			max96712_reg_write(hub, 0x0006, (1 << i));
			mdelay(5);

			if (ser_write(hub, priv->ser_addr[i], 0x00,
				      (priv->ser_alias_addr[i] << 1))) {
				continue;
			}
		}
	}
	return 0;
}

static int max_channel_mapping(struct maxim_hub_priv *priv)
{
#ifdef MAXIM_HUB_DEBUG
	dev_err(priv->hub.dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	if (priv->serial_type == SER_TYPE_MAX96705)
		max96705_config(priv);
	else if (priv->serial_type == SER_TYPE_MAX96701)
		max96701_config(priv);

	return 0;
}

static int max_channel_link(struct maxim_hub_priv *priv, int enable)
{
	struct deser_hub_dev *hub =  &priv->hub;
	uint8_t is_enable;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	if (enable)
		is_enable = 0xF;
	else
		is_enable = 0x0;
	/*Enable all link*/
	udelay(1 * 1000);
	/*open GSML1 LINK A B C D*/
	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
		max96712_reg_write(hub, 0x0006, 0x0 | is_enable);
	else if (priv->link_mode == MAXIM_LINK_MODE_GMSL2)
		max96712_reg_write(hub, 0x0006, 0xF0 | is_enable);

	udelay(1 * 1000);
	max96712_reg_write(hub, 0x0018, 0x0F);
	return 0;
}

static int max96712_gsml1_config(struct maxim_hub_priv *priv)
{
	int ret;
#ifdef MAXIM_HUB_DEBUG
	dev_err(priv->hub.dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*1.preinit: reset_register*/
	ret = max96712_preinit(priv);
	if (ret) {
		pr_err("%s(), line %d, max96712_preinit failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*2.mipi config*/
	ret = max96712_gsm1_mipi_config(priv);
	if (ret) {
		pr_err("%s(), line %d, max96712_gsm1_mipi_config failed!\n",
		       __func__, __LINE__);
		return -EINVAL;
	}
	// max96712_reg_write(&(priv->hub), 0x0006, 0x0f);
	// max96712_reg_write(&(priv->hub), 0x0018, 0x0f);
	/*3.fsync config*/
	ret = max96712_fsync_config(priv, priv->hub.trig_info);
	if (ret) {
		pr_err("%s(), line %d, max96712_fsync_config failed!\n",
		       __func__, __LINE__);
		return -EINVAL;
	}

	/*4. camera_mapping*/
	ret = max_channel_mapping(priv);
	if (ret) {
		pr_err("%s(), line %d, max_channel_mapping failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*5 Disable all link & enable mipi output*/
	ret = max_channel_link(priv, 1);
	if (ret) {
		pr_err("%s(), line %d, max_channel_stream failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*6. disable mipi output*/
	maxim_hub_set_mipi_output(&priv->hub, false);
	/*open camera link*/
	max96712_reg_write(&(priv->hub), 0x0006, 0x0f);
	return 0;
}

static int maxim_hub_s_power(struct v4l2_subdev *sd, int enable)
{
	int i, j;
	int ret;
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);
	struct maxim_hub_priv *priv =
		container_of(hub, struct maxim_hub_priv, hub);
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif

	if (strncmp(&hub->ctl_level[0], "fad-lis", MAX_DTS_STRING_LEN) == 0) {
		/* We enable CSI_REPLICATE for fad-lis */
		dev_info(hub->dev, "%s: Slave mode, break config\n", __func__);
		//
		mdelay(100);
		for (i = 0; i < hub->max_port; i++) {
			for (j = 0; j < 20; j++) {
				ret = is_gmsl2_video_connected(hub, i);
				mdelay(50);
				if (ret)
					break;
			}
		}
		hub->deser_boot_flag = true;
		return 0;
	}

	if (priv->link_mode == MAXIM_LINK_MODE_GMSL2) {
		dev_info(hub->dev, "%s() line:%d GMSL2\n", __func__, __LINE__);
		if (max96712_gsml2_config(priv)) {
			pr_info("%s(), line %d, max96712_gsml2_config failed!\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
		dev_info(hub->dev, "%s() line:%d GMSL1\n", __func__, __LINE__);
		if (max96712_gsml1_config(priv)) {
			pr_info("%s(), line %d, max96712_gsml1_config failed!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	hub->deser_boot_flag = true;
	pr_info("%s(), line %d, max96712 s_power success!\n", __func__,
	       __LINE__);
	return 0;
}

static struct v4l2_subdev_video_ops maxim_deser_v4l2_video_ops = {
	.s_stream = maxim_hub_s_stream,
};

static struct v4l2_subdev_core_ops maxim_deser_v4l2_core_ops = {
	.s_power = maxim_hub_s_power,
};

static struct v4l2_subdev_ops maxim_deser_v4l2_ops = {
	.core = &maxim_deser_v4l2_core_ops,
	.video = &maxim_deser_v4l2_video_ops,
};

static int deser_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_subdev *asd)
{
	struct camera_dev *cam_dev;
	struct deser_channel *deser_chn;
	//uint8_t value;

	cam_dev = container_of(sd, struct camera_dev, subdev);
	deser_chn = container_of(asd, struct deser_channel, async_dev);

	cam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	cam_dev->deser_parent = deser_chn->deser_dev;
	cam_dev->index_in_serdes = deser_chn->index;
	deser_chn->cam_dev = cam_dev;
	deser_chn->camera_bound = true;

	pr_info("%s(),line %d channel[%d]", __func__, __LINE__, cam_dev->index_in_serdes);
	return 0;
}

static void deser_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations deser_async_ops = {
	.bound = deser_notify_bound,
	.unbind = deser_notify_unbind,
};

int parse_camera_serdes(struct deser_hub_dev *maxim_deser_hub,
			struct device_node *remote_ep, int index)
{
	const char *serializer = NULL;
	struct maxim_hub_priv *priv =
		container_of(maxim_deser_hub, struct maxim_hub_priv, hub);
	if (of_property_read_s32(remote_ep, "serial-i2c",
				 &(priv->ser_addr[index]))) {
		dev_err(maxim_deser_hub->dev,
			"Invalid DT serial-i2cproperty\n");
		return -EINVAL;
	}

	if (of_property_read_u32(remote_ep, "ser-alias-id",
				 &(priv->ser_alias_addr[index]))) {
		dev_err(maxim_deser_hub->dev,
			"Invalid DT ser-alias-id property\n");
		return -EINVAL;
	}

	if (of_property_read_s32(remote_ep, "data-type", &priv->data_type)) {
		dev_err(maxim_deser_hub->dev,
			"Invalid DT data-type property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(remote_ep, "sensor-id",
				 &(priv->sensor_addr[index]))) {
		dev_err(maxim_deser_hub->dev,
			"Invalid DT sensor-id property\n");
		//return -EINVAL;
	}

	if (of_property_read_u32(remote_ep, "sensor-alias-id",
				 &(priv->sensor_alias_addr[index]))) {
		dev_err(maxim_deser_hub->dev,
			"Invalid DT sensor-alias-id cproperty\n");
		//return -EINVAL;
	}

	if (!of_property_read_string(remote_ep, "serializer", &serializer)) {

		if (strncmp(serializer, "max96705", 8) == 0)
			priv->serial_type = SER_TYPE_MAX96705;
		else if (strncmp(serializer, "max96701", 8) == 0)
			priv->serial_type = SER_TYPE_MAX96701;
		else if (strncmp(serializer, "max9295", 7) == 0)
			priv->serial_type = SER_TYPE_MAX9295;
		else if (strncmp(serializer, "max96717f", 9) == 0)
			priv->serial_type = SER_TYPE_MAX96717f;
		else
			dev_info(maxim_deser_hub->dev, "SER_TYPE_INVALID\n");
	} else {
		priv->serial_type = SER_TYPE_INVALID;
		dev_err(maxim_deser_hub->dev,
			"Invalid DT serializer  property\n");
		priv->serial_type = SER_TYPE_INVALID;
		return -EINVAL;
	}

	dev_info(
		maxim_deser_hub->dev,
		"camera index is %d,ser is %x,ser_alias is %x,sensor addr is %x, sensor_i2c_addr_alias is %x\n",
		index, priv->ser_addr[index], priv->ser_alias_addr[index],
		priv->sensor_addr[index], priv->sensor_alias_addr[index]);

	return 0;
}

static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	int i;

	for (i = 0; i < hub->max_port; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s:: input port%d not found\n ", __func__, i);
			continue;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s:: input device%d not found\n", __func__, i);
			continue;
		}
		hub->chn[i].index = i;
		hub->chn[i].camera_node = remote;
		hub->chn[i].camera_fwnode = of_fwnode_handle(remote);
		hub->num_cameras++;
		if (parse_camera_serdes(hub, remote, i))
			dev_err(hub->dev, "%s:: parse_camera_serdes [%d]\n", __func__, i);
	}

	return 0;
}

static int parse_output_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	struct device_node *csi2 = of_get_child_by_name(node, "csi-link");

	if (!csi2) {
		dev_err(hub->dev, "csi-link not found\n");
		return -EINVAL;
	}

	hub->subdev.fwnode = of_fwnode_handle(csi2);

	return 0;
}

static int maxim_hub_parse_dt(struct i2c_client *client)
{
	struct maxim_hub_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct deser_hub_dev *hub = &priv->hub;
	const char *link_mode = NULL;
	const char *ctl_level = NULL;
	int i;

	if (!np)
		return -EINVAL;

	if (of_property_read_s32(np, "reg", &priv->des_addr)) {
		dev_err(&client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	// for single soc, ctl_level string is NULL
	if (of_property_read_string(np, "ctl-mode", &ctl_level))
		dev_dbg(hub->dev, " Failed to find ctl-mode\n");
	else
		strncpy(&hub->ctl_level[0], ctl_level, MAX_DTS_STRING_LEN);

	if (!of_property_read_string(np, "maxim,link-mode", &link_mode)) {
		if (!strncmp(link_mode, "GMSL2", 5)) {
			dev_info(hub->dev, "%s() line:%d GMSL2\n", __func__, __LINE__);
			priv->link_mode = MAXIM_LINK_MODE_GMSL2;
		} else if (!strncmp(link_mode, "GMSL1", 5)) {
			dev_info(hub->dev, "%s() line:%d GMSL1\n", __func__, __LINE__);
			priv->link_mode = MAXIM_LINK_MODE_GMSL1;
		}
	} else {
		dev_err(&client->dev, "Invalid DT link-mode property\n");
		return -EINVAL;
	}

	if (of_property_read_s32(np, "lane-speed", &priv->lane_speed)) {
		dev_err(&client->dev, "Invalid DT lane-speed property\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "maxim,linkrx-rate", priv->linkrx_rate, 4)) {
		dev_err(&client->dev, "Invalid DT maxim,linkrx-rate\n");
		return -EINVAL;
	}

	hub->type = DESER_TYPE_INVALID;
	if (!of_property_read_string(np, "type", &priv->deser_type)) {
		if (priv->deser_type != NULL) {
			strncpy(hub->name, priv->deser_type,
				MAX_DESER_NAME_LEN);
			if (strncmp(priv->deser_type, "max96722", 8) == 0) {
				hub->src_mask = 0x0f;
				hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			} else if (strncmp(priv->deser_type, "max96712", 8) ==
				   0) {
				hub->src_mask = 0x0f;
				hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			}
		}
	}

	/*Optional field*/
	if (of_property_read_u32(np, "trigger-mode",
				 &hub->trig_info.trigger_mode))
		hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_NONE;
	if (of_property_read_u32(np, "trigger-fps",
				 &hub->trig_info.trigger_fps))
		hub->trig_info.trigger_fps = 25;
	if (of_property_read_u32(np, "trigger-tx-gpio",
				 &hub->trig_info.trigger_tx_gpio))
		hub->trig_info.trigger_tx_gpio = 0;
	if (of_property_read_u32(np, "trigger-rx-gpio",
				 &hub->trig_info.trigger_rx_gpio))
		hub->trig_info.trigger_rx_gpio = 0;
	if (of_property_read_u32(np, "maxim,hsync-invert", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync-invert", &priv->vsync))
		priv->vsync = 0;
	if (of_property_read_u64(np, "maxim,crossbar", &priv->crossbar))
		priv->crossbar = crossbar;

	/* parse crossbar setup */
	for (i = 0; i < 16; i++) {
		priv->cb[i] = priv->crossbar % 16;
		priv->crossbar /= 16;
	}

	if (parse_input_dt(hub, np)) {
		dev_err(hub->dev, ":parse input dt failed\n");
		return -1;
	}
	if (parse_output_dt(hub, np)) {
		dev_err(hub->dev, ":parse output dt failed\n");
		return -1;
	}
#ifdef MAXIM_HUB_DEBUG
	//TODO: dump_device_info
#endif
	return 0;
}

static int register_subdev(struct maxim_hub_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	struct v4l2_subdev *sd;

	hub = &priv->hub;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &maxim_deser_v4l2_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->pads[MAXIM_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MAXIM_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, MAXIM_N_PADS, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, ":register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct maxim_hub_priv *priv)
{
	int ret;
	int i;
	int index;
	struct deser_hub_dev *hub = &priv->hub;

	if (!hub->num_cameras) {
		dev_err(hub->dev, "%s: :no input device found\n", __func__);
		return -1;
	}

	v4l2_async_notifier_init(&hub->notifier);
	hub->notifier.ops = &deser_async_ops;
	index = 0;
	for (i = 0; i < hub->max_port; i++) {
		if (!hub->chn[i].camera_fwnode)
			continue;

		hub->chn[i].deser_dev = hub;
		hub->chn[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
		hub->chn[i].async_dev.match.fwnode = hub->chn[i].camera_fwnode;
		v4l2_async_notifier_add_subdev(&hub->notifier,
					       &(hub->chn[i].async_dev));
		index++;
	}

	ret = v4l2_async_subdev_notifier_register(&hub->subdev, &hub->notifier);
	if (ret) {
		dev_err(hub->dev, "%s: :register subdev notifier failed\n",
			__func__);
		return -1;
	}

	return 0;
}

int maxim_deser_hub_set_internal_frame_sync(struct deser_hub_dev *deser_hub,
					    int trigger_gpio, int fps)
{
	struct maxim_hub_priv *priv =
		container_of(deser_hub, struct maxim_hub_priv, hub);
	struct deser_trigger_info trig_info;

	trig_info.trigger_mode = DESER_TRIGGER_MODE_INTERNAL;
	trig_info.trigger_fps = fps;
	trig_info.trigger_tx_gpio = trigger_gpio;

	maxim_hub_set_mipi_output(deser_hub, false);
	switch (deser_hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_fsync_config(priv, trig_info);
		break;
	default:
		dev_err(priv->hub.dev, "Device type not support for now\n");
	}
	maxim_hub_set_mipi_output(deser_hub, true);
	return 0;
}

int maxim_deser_hub_set_external_frame_sync(struct deser_hub_dev *deser_hub, int external_freq, int target_freq, int fsync_in, int fsync_out,
		int camera_trigger_gpio, int deser_trigger_gpio)
{
	struct maxim_hub_priv *priv =
		container_of(deser_hub, struct maxim_hub_priv, hub);
	struct deser_trigger_info trig_info;

	trig_info.trigger_mode = DESER_TRIGGER_MODE_EXTERNAL;
	trig_info.trigger_rx_gpio = deser_trigger_gpio;
	trig_info.trigger_tx_gpio = camera_trigger_gpio;
	switch (deser_hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_fsync_config(priv, trig_info);
		break;
	default:
		dev_err(priv->hub.dev, "Device type support for now\n");
	}
	//set isp trigger mode
	if (!external_freq)
		isp_internal_trigger(target_freq, fsync_in, fsync_out);
	else
		isp_external_trigger(external_freq, target_freq, fsync_in, fsync_out);
	return 0;
}

static int maxim_detect_deserial(struct maxim_hub_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	uint8_t value;
	struct i2c_client *client;

	hub = &priv->hub;
	client = hub->i2c_client;
	ret = max96712_reg_read(hub, 0x0000, &value);
	pr_err("==== modify test value=0x%x, client->addr=0x%x\n", value, client->addr);
	if (client->addr != (value>>1)) {
		dev_err(priv->hub.dev, "=== modify test max96712 have not found\n");
		return 1;
	}
	return 0;
}

static void maxim_vdd_regulator_enable(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub;
	uint8_t value;

	hub = &priv->hub;
	value = 0;

	//Enable REG_ENABLE	
	max96712_reg_read(hub, 0x0017, &value);
	value |= 0x1;
	max96712_reg_write(hub, 0x0017, value);
	//Enable REG_MNL
	value = 0;
	max96712_reg_read(hub, 0x0017, &value);
	value |= 1 << 4;
	max96712_reg_write(hub, 0x19, value);
	
	//readback
	max96712_reg_read(hub, 0x0017, &value);
	dev_info(priv->hub.dev, "read_back REG_ENABLE : 0x%x\n", value);
	max96712_reg_read(hub, 0x0019, &value);
	dev_info(priv->hub.dev, "read_back REG_MNL : 0x%x\n", value);	
}

static int maxim_hub_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct maxim_hub_priv *priv;
	int pdb_gpio = -1;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
	priv->hub.dev = &client->dev;
	priv->hub.deser_boot_flag = false;
	priv->hub.deser_stream_flag = false;
	priv->hub.internal_trigger_sync =
		maxim_deser_hub_set_internal_frame_sync;
	priv->hub.external_trigger_sync =
		maxim_deser_hub_set_external_frame_sync;
	mutex_init(&priv->hub.deser_mutex);
	//power up deserializer at first
	if (strncmp(priv->hub.ctl_level, "fad-lis", 7) != 0) {
		pdb_gpio = of_get_named_gpio(client->dev.of_node, "pdb-gpio", 0);
		if (gpio_is_valid(pdb_gpio)) {
			ret = devm_gpio_request(&client->dev, pdb_gpio,
						dev_name(&client->dev));
			if (ret) {
				dev_err(&client->dev,
					"failed to request gpio %d\n", pdb_gpio);
				return ret;
			}

			gpio_direction_output(pdb_gpio, 0);
			mdelay(5);
			gpio_direction_output(pdb_gpio, 1);
			mdelay(20);
		}
	}

	/*
	 * Only when VDD 1.2V call this func
	 */
	maxim_vdd_regulator_enable(priv);

	ret = maxim_detect_deserial(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: not found max96712\n", __func__);
		return -EINVAL;
	}

	ret = register_subdev(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s() line %d: register subdev failed\n",
			__func__, (int)__LINE__);
		return -EINVAL;
	}

	ret = maxim_hub_parse_dt(client);
	if (ret) {
		dev_err(priv->hub.dev, "%s: :parse dt failed\n", __func__);
		return -EINVAL;
	}

	ret = register_subdev_notifier(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: register subdev notifier failed\n",
			__func__);
		return -EINVAL;
	}
	/*wait for start up GMSL Link*/
	mdelay(20);
	max96712_reg_write(&priv->hub, 0x03, 0xff);
	pr_info("maxim hub probe done\n");
	return 0;
}

static int maxim_hub_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id maxim_deser_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, maxim_deser_id);

static const struct of_device_id maxim_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, maxim_derser_hub);

static struct i2c_driver maxim_derser_hub = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(maxim_of_match),
	},
	.probe		= maxim_hub_probe,
	.remove		= maxim_hub_remove,
	.id_table	= maxim_deser_id,
};

module_i2c_driver(maxim_derser_hub);

MODULE_DESCRIPTION("GMSL driver for maxim_deser_hub");
MODULE_AUTHOR("jason.yin@bst.ai");
MODULE_LICENSE("GPL");

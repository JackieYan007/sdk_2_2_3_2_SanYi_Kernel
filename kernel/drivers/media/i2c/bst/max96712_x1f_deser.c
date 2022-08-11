// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM96712 x1f Deserializer for BST Deserializer Driver
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
#include "common_deser_hub.h"

#define MODULE_NAME "bst,max96712_x1f_deser"
#define max96712_N_LINKS 4

enum max96712_pads {
	max96712_SINK_LINK0,
	max96712_SINK_LINK1,
	max96712_SINK_LINK2,
	max96712_SINK_LINK3,
	max96712_SOURCE,
	max96712_N_PADS,
};

struct max96712_sink {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev	*sd;
	struct fwnode_handle	*fwnode;
};

struct max96712_priv {
	struct deser_hub_dev hub;
	struct media_pad pads[max96712_N_PADS];
	struct max96712_sink sinks[max96712_N_LINKS];
	int des_addr;
	int n_links;
	int links_mask;
	long pixel_rate;
	const char *fsync_mode;
	int fsync_period;
	int pclk;
	int him;
	int hsync;
	int vsync;
	int bws;
	int dbl;
	int dt;
	int timeout;
	u64 crossbar;
	char cb[16];
	int ser_addr[4];
	const char *link_mode;
};

static char fsync_mode_default[20] = "manual"; /* manual, automatic, semi-automatic, external */
static char link_mode_default[20] = "GSML1"; /* manual, automatic, semi-automatic, external */
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

static int config_max96712_regs(struct deser_hub_dev *hub)
{
	int i;
	struct i2c_client *client = hub->i2c_client;

	for (i = 0; i < ARRAY_SIZE(max96712_x1f_regs); i++) {
		if (write_reg(client, max96712_x1f_regs[i][0],
				  max96712_x1f_regs[i][1])) {
			dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
			return -1;
		}
		//pr_info(" write max96712 reg:%#x, val:%#x", max96712_regs[i][0], max96712_regs[i][1]);
	}
	return 0;
}

static int max96712_gsml2_config(struct max96712_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	return config_max96712_regs(hub);
}

static int max96712_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int max96712_preinit(struct max96712_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif

	/*device reset registers*/
	max96712_reg_write(hub, 0x0013, 0x75);
	mdelay(2);
	/*Disable mipi output*/
	max96712_reg_write(hub, 0x040B, 0x00);
	/*Disable mipi output*/
	max96712_reg_write(hub, 0x0006, 0x00);
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

static int max96712_gsm1_mipi_config(struct max96712_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*MIPI config*/
	for (i = 0; i < sizeof(max96712_x1f_gsml1_mipi_reg_list) /
				sizeof(max96712_x1f_gsml1_mipi_reg_list[0]);
		 i++) {
		max96712_reg_write(hub, max96712_x1f_gsml1_mipi_reg_list[i][0],
				   max96712_x1f_gsml1_mipi_reg_list[i][1]);
	}
	return 0;
}

static int max96712_fsync_config(struct max96712_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Set Manual mode*/
	max96712_reg_write(hub, 0x04A0, 0x04);
	/*Turn off auto master link selection*/
	max96712_reg_write(hub, 0x04A2, 0x00);
	/*Disable overlap window*/
	max96712_reg_write(hub, 0x04AA, 0x00);
	max96712_reg_write(hub, 0x04AB, 0x00);
	/*AUTO_LINK = 0 FS_USE_XTAL = 1, FS_LINK_[3:0] = 0*/
	max96712_reg_write(hub, 0x04AF, 0x4f);
	/*PCLK config*/
	/*ar0413: 78.6MHz/25FPS */

	max96712_reg_write(hub, 0x04A7, priv->fsync_period >> 16);
	max96712_reg_write(hub, 0x04A6, (priv->fsync_period >> 8) & 0xff);
	max96712_reg_write(hub, 0x04A5, priv->fsync_period & 0xff);
	//gpio id used for transmitting fsync signal
	max96712_reg_write(hub, 0x04b1, 0x10); //0x0c 30hz
	/*Enable FSYNC transmission to serializer, Select GPIO 1(MFP2)*/
	max96712_reg_write(hub, 0x0B08, 0x71);
	max96712_reg_write(hub, 0x0C08, 0x71);
	max96712_reg_write(hub, 0x0D08, 0x71);
	max96712_reg_write(hub, 0x0E08, 0x71);
	return 0;
}

static int max96705_config(struct max96712_priv *priv, uint8_t device_addr)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d, device_addr %x\n", __func__, __LINE__,
		device_addr);
#endif
	/*Close link Enable Config*/
	ser_write(hub, device_addr, 0x04, 0x43);
	/*VSYNC invert*/
	ser_write(hub, device_addr, 0x40, 0x2f);
	/*FSYNC*/
	//max96712 0x04A5, 0x04A6, 0x04A7
	ser_write(hub, device_addr, 0x47, 0x2e);
	ser_write(hub, device_addr, 0x48, 0x50);
	ser_write(hub, device_addr, 0x49, 0x00);
	ser_write(hub, device_addr, 0x4b, 0x90);
	ser_write(hub, device_addr, 0x4c, 0x00);
	/*Enable Sync gen*/
	ser_write(hub, device_addr, 0x43, 0x25);
	mdelay(5);
	//TODO
	// if(priv->is_crossbar)
	// {

	// }
	/*Set Deser address*/
	ser_write(hub, device_addr, 0x01, 0x5a);
	/*Open link disbale config*/
	ser_write(hub, device_addr, 0x04, 0x83);

	return 0;
}

static int max_channel_mapping(struct max96712_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	for (i = 0; i < priv->n_links; i++) {
		{
			//chose Link
			max96712_reg_write(hub, 0x0006, (1 << i));
			mdelay(5);

			uint8_t device_id = 0;

			if (bst_i2c_read_byte_data_byte_reg(
					hub->i2c_client->adapter, 0x40, 0x1e,
					&device_id)) {
				continue;
			}

			if (device_id != MAX96701_ID)
				continue;

			if (ser_write(hub, 0x40, 0x00,
					  (priv->ser_addr[i] << 1))) {
				continue;
			}

			int sonser_addr = (0x50 + i);

			ser_write(hub, priv->ser_addr[i], 0x09,
				  sonser_addr * 2);

			ser_write(hub, priv->ser_addr[i], 0x0A, 0x36 * 2);

			mdelay(10);
			ser_write(hub, priv->ser_addr[i], 0x04, 0x43);

			ser_write(hub, priv->ser_addr[i], 0x01, 0x5a);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x4d, 0xc0);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x07, 0x84);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x3f, 0x1c);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x40, 0x1d);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x41, 0x1b);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x42, 0x7b);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x0f, 0xBf);
			mdelay(5);
			ser_write(hub, priv->ser_addr[i], 0x04, 0x83);
			mdelay(5);
		}
	}

	return 0;
}

static int max_channel_open(struct max96712_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Enbale all link*/
	/*Reset one-shot*/
	max96712_reg_write(hub, 0x0018, 0x0F);
	udelay(1 * 1000);
	/*open GSML1 LINK A B C D*/
	max96712_reg_write(hub, 0x0006, 0x0F);
	udelay(1 * 1000);
	/*enable mipi output*/
	max96712_reg_write(hub, 0x040B, 0x42);
	return 0;
}

static int max96712_gsml1_config(struct max96712_priv *priv)
{
	int ret;
#ifdef MAX96712_DEBUG
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
	/*3.fsync config*/
	ret = max96712_fsync_config(priv);
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
	/*5 enable all link & enable mipi output*/
	ret = max_channel_open(priv);
	if (ret) {
		pr_err("%s(), line %d, max_channel_open failed!\n", __func__,
			   __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int max96712_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);
	struct max96712_priv *priv =
		container_of(hub, struct max96712_priv, hub);
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/**/
	if (!strncmp(priv->link_mode, "GMSL2", 5)) {
		dev_dbg(hub->dev, "%s() line:%d GMSL2\n", __func__, __LINE__);
		if (max96712_gsml2_config(priv)) {
			pr_info("%s(), line %d, max96712_gsml2_config failed!\n",
				   __func__, __LINE__);
			return -EINVAL;
		}
	} else if (!strncmp(priv->link_mode, "GMSL1", 5)) {
		dev_dbg(hub->dev, "%s() line:%d GMSL1\n", __func__, __LINE__);
		if (max96712_gsml1_config(priv)) {
			pr_info("%s(), line %d, max96712_gsml1_config failed!\n",
				   __func__, __LINE__);
			return -EINVAL;
		}
	}

	hub->deser_boot_flag = true;
	pr_info("%s(), line %d, max96712 s_power success!\n", __func__,
		   __LINE__);
	return 0;
}

static struct v4l2_subdev_video_ops max96712_video_ops = {
	.s_stream = max96712_s_stream,
};

static struct v4l2_subdev_core_ops max96712_core_ops = {
	.s_power = max96712_s_power,
};

static struct v4l2_subdev_ops max96712_ops = {
	.core = &max96712_core_ops,
	.video = &max96712_video_ops,
};

static int deser_notify_bound(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *sd,
				  struct v4l2_async_subdev *asd)
{
	struct camera_dev *cam_dev;
	struct deser_channel *deser_chn;

	cam_dev = container_of(sd, struct camera_dev, subdev);
	deser_chn = container_of(asd, struct deser_channel, async_dev);

	cam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	cam_dev->deser_parent = deser_chn->deser_dev;
	cam_dev->index_in_serdes = deser_chn->index;
	deser_chn->cam_dev = cam_dev;
	deser_chn->camera_bound = true;
	pr_info("%s(),line %d", __func__, __LINE__);

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

static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	int i;

	struct max96712_priv *priv =
		container_of(hub, struct max96712_priv, hub);

	for (i = 0; i < priv->n_links; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s:: input port%d not found\n ",
				__func__, i);
			break;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s:: input device%d not found\n",
				__func__, i);
			break;
		}

		hub->chn[i].index = i;
		hub->chn[i].camera_node = remote;
		hub->chn[i].camera_fwnode = of_fwnode_handle(remote);
		hub->num_cameras++;
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

static int max96712_parse_dt(struct i2c_client *client)
{
	int err, i;
	u32 addrs[4], naddrs;
	const char *type_name = NULL;
	struct max96712_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;

	struct deser_hub_dev *hub = &priv->hub;

	if (!np)
		return -EINVAL;

	if (of_property_read_s32(np, "reg", &priv->des_addr)) {
		dev_err(&client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	naddrs = of_property_count_elems_of_size(np, "regs", sizeof(u32));
	err = of_property_read_u32_array(client->dev.of_node, "regs", addrs,
					 naddrs);

	priv->n_links = naddrs;
	memcpy(priv->ser_addr, addrs, naddrs * sizeof(u32));

	if (err < 0) {
		dev_err(&client->dev, "Invalid DT regs property\n");
		return -EINVAL;
	}

	hub->type = DESER_TYPE_INVALID;
	if (!of_property_read_string(np, "type", &type_name)) {
		if (type_name != NULL) {
			strncpy(hub->name, type_name, MAX_DESER_NAME_LEN);
			if (strncmp(type_name, "max9296", 7) == 0) {
				hub->src_mask = 0x03;
				hub->max_port = 2;
				hub->type = DESER_TYPE_MAX9296;
			} else if (strncmp(type_name, "max96712", 8) == 0) {
				hub->src_mask = 0x0f;
				hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			}
		}
	}

	if (of_property_read_string(np, "maxim,fsync-mode", &priv->fsync_mode))
		priv->fsync_mode = fsync_mode_default;

	if (of_property_read_u32(np, "maxim,fsync_period", &priv->fsync_period))
		priv->fsync_period = 3072000; /* 76.8MHz/25fps */
	if (of_property_read_u32(np, "maxim,him", &priv->him))
		priv->him = 0;
	if (of_property_read_u32(np, "maxim,hsync_vert", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync_invert", &priv->vsync))
		priv->vsync = 1;
	if (of_property_read_u32(np, "maxim,bws", &priv->bws))
		priv->bws = 0;
	if (of_property_read_u32(np, "maxim,dbl", &priv->dbl))
		priv->dbl = 1;
	if (of_property_read_u32(np, "maxim,dt", &priv->dt))
		priv->dt = 3;
	if (of_property_read_u64(np, "maxim,crossbar", &priv->crossbar))
		priv->crossbar = crossbar;
	if (of_property_read_string(np, "maxim,link_mode", &priv->link_mode))
		priv->link_mode = link_mode_default;
	//	priv->crossbar = crossbar;

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

	return 0;
}

static int register_subdev(struct max96712_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	struct v4l2_subdev *sd;

	hub = &priv->hub;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &max96712_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->pads[max96712_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[max96712_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max96712_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max96712_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max96712_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, max96712_N_PADS, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, ":register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct max96712_priv *priv)
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
	for (i = 0; i < priv->n_links; i++) {
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

static int max96712_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret;
	struct max96712_priv *priv;
	int pdb_gpio = -1;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pdb_gpio = of_get_named_gpio(client->dev.of_node, "pdb-gpio", 0);
	if (gpio_is_valid(pdb_gpio)) {
		ret = devm_gpio_request(&client->dev, pdb_gpio,
					dev_name(&client->dev));
		if (ret) {
			dev_err(&client->dev, "failed to request gpio %d\n",
				pdb_gpio);
			return ret;
		}

		mdelay(5);
		gpio_direction_output(pdb_gpio, 0);
		mdelay(5);
		gpio_direction_output(pdb_gpio, 1);
		mdelay(5);
	}

	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
	priv->hub.dev = &client->dev;
	priv->hub.deser_boot_flag = false;

	ret = register_subdev(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s() line %d: register subdev failed\n",
			__func__, (int)__LINE__);
		return -EINVAL;
	}

	ret = max96712_parse_dt(client);
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

	pr_info("max96712 probe ok !!!!\n");
	return 0;
}

static int max96712_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id max96712_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, max96712_id);

static const struct of_device_id max96712_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, max96712_of_match);

static struct i2c_driver max96712_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(max96712_of_match),
	},
	.probe		= max96712_probe,
	.remove		= max96712_remove,
	.id_table	= max96712_id,
};

module_i2c_driver(max96712_driver);

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM9296 Deserializer for BST Deserializer Driver
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

#define MODULE_NAME "bst,max9296-deser"
#define max9296_N_LINKS 4
#define max9296_SER_MAX_LINK 4
#define DRIVER_DEBUG
//#define REG_DEBUG

/*MAX9296 Spliter mode*/
/*single LINK*/
static unsigned short single_link_max9296_regs[][2] = {
	{0x320, 0x2c},
	{0x313, 0x02}
};
static unsigned short single_link_max9295_regs[][2] = {
	{0x2be, 0x10},
	{0x57, 0x12},
	{0x5b, 0x11},
	{0x318, 0x5e},
};

/*multple LINK*/
static unsigned short max9295A_pipe_line_port_regs[][2] = {
	{0x330, 0x00},
	{0x332, 0xee},		//set phy1 lane map
	{0x333, 0xe4},		//set phy2 lane map
	{0x331, 0x33},		//Set 4 lanes for serializer
	{0x311, 0xf0},		//Enable all pipelines
	{0x308, 0x7f},		//port B in ser enabled
	{0x314, 0x62},		//Unused VIDEO_x
	{0x316, 0x62},		//Unused VIDEO_Y
	{0x318, 0x5e},		//Route YUV422 8bit to VIDEO_Z
	{0x31a, 0x62},		//Unused VIDEO_U
	{0x002, 0xf3},		//start all pipelines transmission (VID_TX_ENX/Y/Z/U = 1)
	{0x2be, 0x10},
};

static unsigned short max9295B_pipe_line_port_regs[][2] = {
	{0x330, 0x00},
	{0x332, 0xee},		//set phy1 lane map
	{0x333, 0xe4},		//set phy2 lane map
	{0x331, 0x33},		//Set 4 lanes for serializer
	{0x311, 0xf0},		//Enable all pipelines
	{0x308, 0x7f},		//port B in ser enabled
	{0x314, 0x5e},		//Route YUV422 8bit to VIDEO_X (MSB enable), VC = 0
	{0x316, 0x62},		//Unused VIDEO_Y
	{0x318, 0x62},		//Unused VIDEO_Z
	{0x31a, 0x62},		//Unused VIDEO_U
	{0x002, 0xf3},		//start all pipelines transmission (VID_TX_ENX/Y/Z/U = 1)
	{0x2be, 0x10},
};

static unsigned short max9296_spliter_pipe_regs[][2] = {
	{0x330, 0x04},
	{0x332, 0xf0},
	{0x333, 0x4e},		//portA use phy0 phy1
	{0x320, 0x38},		//set phy1 2.4Ghz
	{0x323, 0x38},		//set phy2 2.4Ghz
	{0x50, 0x0},		//Set 4 lanes for serializer
	{0x51, 0x1},		//Enable all pipelines
	{0x52, 0x2},		//port B in ser enabled
	{0x53, 0x3},		//Unused VIDEO_x
	{0x02, 0xf3},		//Unused VIDEO_Y
	{0x40b, 0x07},		//Route YUV422 8bit to VIDEO_Z
	{0x42d, 0x15},		//Unused VIDEO_U
	{0x40d, 0x1e},		//start all pipelines transmission (VID_TX_ENX/Y/Z/U = 1)
	{0x40e, 0x5e},
	{0x40f, 0x0},
	{0x410, 0x40},
	{0x411, 0x1},
	{0x412, 0x41},
	{0x48b, 0x07},
	{0x4ad, 0x15},
	{0x48d, 0x1e},
	{0x48e, 0x1e},
	{0x48f, 0x0},
	{0x490, 0x0},
	{0x491, 0x1},
	{0x492, 0x1},
	{0x10, 0x33}
};

static unsigned short max9295A_TX_SRC_ID_regs[][2] = {
	{0x06B, 0x16},
	{0x073, 0x17},
	{0x07B, 0x36},
	{0x083, 0x36},
	{0x093, 0x36},
	{0x09B, 0x36},
	{0x0A3, 0x36},
	{0x0AB, 0x36},
	{0x08B, 0x36}
};

enum max9296_pads {
	max9296_SINK_LINK0,
	max9296_SINK_LINK1,
	max9296_SINK_LINK2,
	max9296_SINK_LINK3,
	max9296_SOURCE,
	max9296_N_PADS,
};

struct max9296_sink {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev	*sd;
	struct fwnode_handle	*fwnode;
};

struct max9296_priv {
	struct deser_hub_dev hub;
	struct media_pad pads[max9296_N_PADS];
	struct max9296_sink sinks[max9296_N_LINKS];
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
	enum link_status link_status;
};


static char fsync_mode_default[20] = "automatic"; /* manual, automatic, semi-automatic, external */
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

static int update_link_mode(struct max9296_priv *priv)
{
	int linkA_connect = 0;
	int linkB_connect = 0;
	uint8_t out_value = 0;
	struct deser_hub_dev *hub = &priv->hub;
	/*Reset one-shot,Disbale Auto_link,Enable linkA*/
	write_reg(hub->i2c_client, 0x10, 0x21);
	usleep_range(1000, 2000);
	if (ser_word_read(hub, MAX_SER_ADDR, 0x00, &out_value))
		dev_err(hub->dev, "LINKA Not detect %d\n", out_value);
	else
		linkA_connect = 1;

	/*Reset one-shot,Disbale Auto_link,Enable linkB*/
	write_reg(hub->i2c_client, 0x10, 0x22);
	usleep_range(1000, 2000);

	if (ser_word_read(hub, MAX_SER_ADDR, 0x00, &out_value))
		dev_err(hub->dev, "LINKB Not detect %d\n", out_value);
	else
		linkB_connect = 1;

	if (linkA_connect && linkB_connect)
		priv->link_status = SPLITER_MODE;
	else
		priv->link_status = AUTO_LINK_MODE;

#ifdef DRIVER_DEBUG
	dev_err(hub->dev, "LinkA stats is %d\n", linkA_connect);
	dev_err(hub->dev, "LinkB stats is %d\n", linkB_connect);
	dev_err(hub->dev, "Link stats is %d\n", priv->link_status);
#endif
	return 0;
}

static void config_single_link_max9296_regs(struct max9296_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
	struct i2c_client *client = hub->i2c_client;

	for (i = 0; i < sizeof(single_link_max9296_regs) /
				sizeof(single_link_max9296_regs[0]);
	     i++) {
		if (write_reg(client, single_link_max9296_regs[i][0],
			      single_link_max9296_regs[i][1])) {
			usleep_range(1000, 2000);
			pr_info("%s() line %d: write_max9296_reg failed!\n",
			       (char *)__func__, (int)__LINE__);
		}
		dev_info(priv->hub.dev, "%s() line %d, write max9296 reg:%#x, val:%#x",
		       (char *)__func__, (int)__LINE__,
		       single_link_max9296_regs[i][0],
		       single_link_max9296_regs[i][1]);
	}
	/*Map LinkA*/
	write_reg(hub->i2c_client, 0x10, 0x21);
	usleep_range(1000, 2000);
	ser_word_write(hub, 0x40, 0x00, (priv->ser_addr[0] << 1));

	config_ser_reg_group(single_link_max9295_regs,
			     ARRAY_SIZE(single_link_max9295_regs), hub,
			     priv->ser_addr[0]);
}

static void config_spliter_max9296_regs(struct max9296_priv *priv)
{
	int i = 0;
	struct i2c_client *client = priv->hub.i2c_client;
	struct deser_hub_dev *hub = &priv->hub;
	int max9295A_reg = priv->ser_addr[0];
	int max9295B_reg = priv->ser_addr[1];
#ifdef DRIVER_DEBUG
	dev_err(hub->dev, "%s() line %d\n", __func__, __LINE__);
#endif
	//init
	/*Disable UART*/
	write_reg(client, 0x03, 0x40);
	/**/
	write_reg(client, 0x10, 0x21);
	mdelay(1);

	//config MAX9295A
	ser_word_write(hub, 0x40, 0x00, (max9295A_reg << 1));

	config_ser_reg_group(max9295A_TX_SRC_ID_regs,
			     ARRAY_SIZE(max9295A_TX_SRC_ID_regs), hub,
			     max9295A_reg);

	// switch to MAX9295B
	write_reg(client, 0x10, 0x22);
	mdelay(1);
	ser_word_write(hub, 0x40, 0x00, (max9295B_reg << 1));
	usleep_range(1000, 2000);
	// switch to Spliter mode
	write_reg(client, 0x10, 0x23);
	mdelay(1);

	/*config max9295A_reg*/
	config_ser_reg_group(max9295A_pipe_line_port_regs,
			     ARRAY_SIZE(max9295A_pipe_line_port_regs), hub,
			     max9295A_reg);
	/*config max9295B_reg*/
	config_ser_reg_group(max9295B_pipe_line_port_regs,
			     ARRAY_SIZE(max9295B_pipe_line_port_regs), hub,
			     max9295B_reg);

	//config_ser_reg_group(max9296_spliter_pipe_regs, hub, max9295A_reg);

	for (i = 0; i < ARRAY_SIZE(max9296_spliter_pipe_regs); i++) {
		if (write_reg(client, max9296_spliter_pipe_regs[i][0],
			      max9296_spliter_pipe_regs[i][1])) {
			usleep_range(1000, 2000);
			pr_info("%s() line %d: write_max9296_reg failed!\n",
			       (char *)__func__, (int)__LINE__);
			return;
		}
		pr_info("%s() line %d, write max9296 reg:%#x, val:%#x",
		       (char *)__func__, (int)__LINE__,
		       max9296_spliter_pipe_regs[i][0],
		       max9296_spliter_pipe_regs[i][1]);
	}

	mdelay(1);
}

static int max9296_initialize(struct max9296_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	write_reg(hub->i2c_client, 0x313, 0x00); /* disable CSI output */

	update_link_mode(priv);

	if (priv->link_status == AUTO_LINK_MODE)
		config_single_link_max9296_regs(priv);
	else if (priv->link_status == SPLITER_MODE)
		config_spliter_max9296_regs(priv);

	write_reg(hub->i2c_client, 0x313, 0x02); /* enable CSI output */

	return 0;
}

static int max9296_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int max9296_s_power(struct v4l2_subdev *sd, int enable)
{
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);
	hub->deser_boot_flag = true;
	pr_info("%s() %d\n", __func__, __LINE__);
	return 0;
}

static struct v4l2_subdev_video_ops max9296_video_ops = {
	.s_stream = max9296_s_stream,
};

static struct v4l2_subdev_core_ops max9296_core_ops = {
	.s_power = max9296_s_power,
};

static struct v4l2_subdev_ops max9296_ops = {
	.core = &max9296_core_ops,
	.video = &max9296_video_ops,
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
	pr_info("%s(),line %d ", __func__, __LINE__);

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

	struct max9296_priv *priv = container_of(hub, struct max9296_priv, hub);

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

static int max9296_parse_dt(struct i2c_client *client)
{
	int err, i;
	u32 addrs[max9296_SER_MAX_LINK], naddrs;
	const char *type_name = NULL;
	struct max9296_priv *priv = i2c_get_clientdata(client);
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

	if (of_property_read_u32(np, "maxim,fsync-period", &priv->fsync_period))
		priv->fsync_period = 3072000; /* 76.8MHz/25fps */
	if (of_property_read_u32(np, "maxim,him", &priv->him))
		priv->him = 0;
	if (of_property_read_u32(np, "maxim,hsync-vert", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync-invert", &priv->vsync))
		priv->vsync = 1;
	if (of_property_read_u32(np, "maxim,bws", &priv->bws))
		priv->bws = 0;
	if (of_property_read_u32(np, "maxim,dbl", &priv->dbl))
		priv->dbl = 1;
	if (of_property_read_u32(np, "maxim,dt", &priv->dt))
		priv->dt = 3;
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

	return 0;
}

static int register_subdev(struct max9296_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	struct v4l2_subdev *sd;

	hub = &priv->hub;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &max9296_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->pads[max9296_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[max9296_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max9296_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max9296_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
	priv->pads[max9296_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, max9296_N_PADS, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, ":register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct max9296_priv *priv)
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

static int max9296_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct max9296_priv *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

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

	ret = max9296_parse_dt(client);
	if (ret) {
		dev_err(priv->hub.dev, "%s: :parse dt failed\n", __func__);
		return -EINVAL;
	}

	max9296_initialize(priv);

	ret = register_subdev_notifier(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: register subdev notifier failed\n",
			__func__);
		return -EINVAL;
	}

	pr_info("max9296 probe ok !!!!\n");
	return 0;
}

static int max9296_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id max9296_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, max9296_id);

static const struct of_device_id max9296_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, max9296_of_match);

static struct i2c_driver max9296_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(max9296_of_match),
	},
	.probe		= max9296_probe,
	.remove		= max9296_remove,
	.id_table	= max9296_id,
};

module_i2c_driver(max9296_driver);

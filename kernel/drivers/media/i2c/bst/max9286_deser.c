// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM9286 Deserializer for BST Deserializer Driver
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
#include "maxim_deser_hub.h"

#define MODULE_NAME "bst,max9286-deser"
#define MAX9286_N_LINKS 4


enum max9286_pads {
	MAX9286_SINK_LINK0,
	MAX9286_SINK_LINK1,
	MAX9286_SINK_LINK2,
	MAX9286_SINK_LINK3,
	MAX9286_SOURCE,
	MAX9286_N_PADS,
};

struct max9286_sink {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev	*sd;
	struct fwnode_handle	*fwnode;
};

struct max9286_priv {
	struct deser_hub_dev hub;
	struct media_pad pads[MAX9286_N_PADS];
	struct max9286_sink sinks[MAX9286_N_LINKS];
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
};

static char fsync_mode_default[20] = "automatic"; /* manual, automatic, semi-automatic, external */
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

static int register_read(struct deser_hub_dev *hub, uint8_t reg)
{
	int ret = i2c_smbus_read_byte_data(hub->i2c_client, reg);

	if (ret < 0)
		dev_err(hub->dev, "%s: read 0x%02x failed\n", __func__, reg);

	return ret;
}

static int register_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value)
{
	int ret = i2c_smbus_write_byte_data(hub->i2c_client, reg, value);

	if (ret < 0)
		dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);

	return ret;
}

static void max9286_preinit(struct max9286_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	register_write(hub, 0x1c, 0x04);
	usleep_range(2000, 2500);
	ser_write(hub, MAX96705_ADDR, 0x4d, 0X0C);
	usleep_range(2000, 2500);
	register_write(hub, 0x1c, (priv->him ? 0xf0 : 0x00) | (priv->bws ? 0x05 : 0x04)); /* high-immunity/legacy mode, BWS 24bit */
	usleep_range(2000, 2500);
	ser_write(hub, MAX96705_ADDR, 0x04, 0X43); //open 96705 i2c config link
	usleep_range(5000, 5500);
	register_write(hub, 0x15, 0x03);
	ser_write(hub, MAX96705_ADDR, 0x40, 0x2f); //96705 invert vsync
	usleep_range(2000, 2500);
}

static void max9286_initial_setup(struct max9286_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	if (strcmp(priv->fsync_mode, "manual") == 0) {
		register_write(
			hub, 0x01,
			0x00); /* manual: FRAMESYNC set manually via [0x06:0x08] regs */
	} else if (strcmp(priv->fsync_mode, "automatic") == 0) {
		register_write(
			hub, 0x01,
			0x02); /* automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "semi-automatic") == 0) {
		register_write(
			hub, 0x01,
			0x01); /* semi-automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "external") == 0) {
		register_write(
			hub, 0x01,
			0xc0); /* ECU (aka MCU) based FrameSync using GPI-to-GPO */
	}

	register_write(hub, 0x63, 0); /* disable overlap window */
	register_write(hub, 0x64, 0);
	register_write(hub, 0x06, priv->fsync_period & 0xff);
	register_write(hub, 0x07, (priv->fsync_period >> 8) & 0xff);
	register_write(hub, 0x08, priv->fsync_period >> 16);

	mdelay(64);
}

static int max9286_reverse_channel_setup(struct max9286_priv *priv, int idx)
{
	int ret = 0;
	struct deser_hub_dev *hub = &priv->hub;

	register_write(hub, 0x0a,
			   0x11 << idx); /* enable reverse control for CAMx */
	/* I2C addresse change */
	ser_write(hub, MAX96705_ADDR, 0x00, priv->ser_addr[idx] << 1);
	mdelay(2);

	ser_write(hub, priv->ser_addr[idx], 0x07,
		  0x04 | (priv->dbl ? 0x80 : 0) | (priv->bws ? 0x20 : 0));
	ser_write(hub, priv->ser_addr[idx], 0x01, priv->des_addr << 1);
	ser_write(hub, priv->ser_addr[idx], 0x0B,
		  MAX96705_BROADCAST << 1); /* broadcast I2C */
	ser_write(hub, priv->ser_addr[idx], 0x0C, priv->ser_addr[idx] << 1);

	return ret;
}

static void max9286_gmsl_link_setup(struct max9286_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	switch (priv->dt) {
	case YUV8_DT:
		/* setup crossbar for YUV8/RAW8: reverse DVP bus */
		ser_write(hub, MAX96705_ADDR, 0x20, priv->cb[7]);
		ser_write(hub, MAX96705_ADDR, 0x21, priv->cb[6]);
		ser_write(hub, MAX96705_ADDR, 0x22, priv->cb[5]);
		ser_write(hub, MAX96705_ADDR, 0x23, priv->cb[4]);
		ser_write(hub, MAX96705_ADDR, 0x24, priv->cb[3]);
		ser_write(hub, MAX96705_ADDR, 0x25, priv->cb[2]);
		ser_write(hub, MAX96705_ADDR, 0x26, priv->cb[1]);
		ser_write(hub, MAX96705_ADDR, 0x27, priv->cb[0]);

		/* this is second byte if DBL=1 */
		ser_write(hub, MAX96705_ADDR, 0x30, priv->cb[7] + 16);
		ser_write(hub, MAX96705_ADDR, 0x31, priv->cb[6] + 16);
		ser_write(hub, MAX96705_ADDR, 0x32, priv->cb[5] + 16);
		ser_write(hub, MAX96705_ADDR, 0x33, priv->cb[4] + 16);
		ser_write(hub, MAX96705_ADDR, 0x34, priv->cb[3] + 16);
		ser_write(hub, MAX96705_ADDR, 0x35, priv->cb[2] + 16);
		ser_write(hub, MAX96705_ADDR, 0x36, priv->cb[1] + 16);
		ser_write(hub, MAX96705_ADDR, 0x37, priv->cb[0] + 16);
		usleep_range(5000, 5500);

		break;
	default:
		break;
	}

	ser_write(hub, MAX96705_ADDR, 0x47, 0x2e); //max9286 0x06, 0x07, 0x08
	ser_write(hub, MAX96705_ADDR, 0x48, 0x50);
	ser_write(hub, MAX96705_ADDR, 0x49, 0x00);
	ser_write(hub, MAX96705_ADDR, 0x4b, 0x90);
	ser_write(hub, MAX96705_ADDR, 0x4c, 0x00);
	ser_write(hub, MAX96705_ADDR, 0x43, 0x25);
	mdelay(5);

	register_write(hub, 0x69, 0xb0);
	register_write(hub, 0x00, 0xef);
}

static int max9286_postinit(struct max9286_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	register_write(hub, 0x12,
			   ((4 - 1) << 6) | (priv->dbl ? 0x30 : 0) |
				   (priv->dt &
				0xf)); /* setup lanes, DBL mode, DataType */

	max9286_initial_setup(priv);

	//register_write(hub, 0x0a, 0xff);

	register_write(hub, 0x62, 0x1f);
	register_write(hub, 0x61, 0xff);
	register_write(hub, 0x5f, 0x0f);

	ser_write(hub, MAX96705_BROADCAST, 0x04,
		  0x83); //enable all max96705 output
	mdelay(64);
	register_write(hub, 0x15, 0x9b); //enable max9286 output

	return 0;
}

static int max9286_initialize(struct max9286_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;

	max9286_preinit(priv);

	max9286_gmsl_link_setup(priv);
	for (i = 0; i < priv->n_links; i++)
		max9286_reverse_channel_setup(priv, i);

	register_write(hub, 0x0a, 0xff); /* enable reverse control for CAMx */

	return 0;
}

static int max9286_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int max9286_s_power(struct v4l2_subdev *sd, int enable)
{
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);
	struct max9286_priv *priv = container_of(hub, struct max9286_priv, hub);

	max9286_postinit(priv);

	hub->deser_boot_flag = true;

	return 0;
}

static struct v4l2_subdev_video_ops max9286_video_ops = {
	.s_stream = max9286_s_stream,
};

static struct v4l2_subdev_core_ops max9286_core_ops = {
	.s_power = max9286_s_power,
};

static struct v4l2_subdev_ops max9286_ops = {
	.core = &max9286_core_ops,
	.video = &max9286_video_ops,
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

	struct max9286_priv *priv = container_of(hub, struct max9286_priv, hub);

	for (i = 0; i < priv->n_links; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s: input port%d not found\n ",
				__func__, i);
			break;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s: input device%d not found\n",
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

static int max9286_parse_dt(struct i2c_client *client)
{
	int err, i;
	u32 addrs[4], naddrs;
	struct max9286_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	u8 val = 0;

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

	val = register_read(&priv->hub, 0x1e); //read max9286 ID
	if (val != MAX9286_ID)
		return -ENODEV;

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
		dev_err(hub->dev, "parse input dt failed\n");
		return -1;
	}

	if (parse_output_dt(hub, np)) {
		dev_err(hub->dev, "parse output dt failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev(struct max9286_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	struct v4l2_subdev *sd;

	hub = &priv->hub;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &max9286_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->pads[MAX9286_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MAX9286_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAX9286_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAX9286_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAX9286_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, MAX9286_N_PADS, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, "register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct max9286_priv *priv)
{
	int ret;
	int i;
	int index;
	struct deser_hub_dev *hub = &priv->hub;

	if (!hub->num_cameras) {
		dev_err(hub->dev, "%s: no input device found\n", __func__);
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
		dev_err(hub->dev, "%s: register subdev notifier failed\n",
			__func__);
		return -1;
	}

	return 0;
}

static int max9286_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	u8 val = 0;

	struct max9286_priv *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
	priv->hub.dev = &client->dev;
	priv->hub.deser_boot_flag = false;

	reg8_read(client, MAXIM_ID_REG, &val);

	if (val != MAX9286_ID) {
		dev_err(priv->hub.dev, "max9286 not connected\n");
		return -EINVAL;
	}

	ret = register_subdev(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: register subdev failed\n",
			__func__);
		return -EINVAL;
	}

	ret = max9286_parse_dt(client);
	if (ret) {
		dev_err(priv->hub.dev, "%s: parse dt failed\n", __func__);
		return -EINVAL;
	}

	max9286_initialize(priv);

	ret = register_subdev_notifier(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: register subdev notifier failed\n",
			__func__);
		return -EINVAL;
	}

	pr_info("max9286 probe ok !!!!\n");
	return 0;
}

static int max9286_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id max9286_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, max9286_id);

static const struct of_device_id max9286_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, max9286_of_match);

static struct i2c_driver max9286_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(max9286_of_match),
	},
	.probe		= max9286_probe,
	.remove		= max9286_remove,
	.id_table	= max9286_id,
};

module_i2c_driver(max9286_driver);

MODULE_DESCRIPTION("GMSL driver for MAX9286");
MODULE_AUTHOR("kanghua.li@bst.ai");
MODULE_LICENSE("GPL v2");

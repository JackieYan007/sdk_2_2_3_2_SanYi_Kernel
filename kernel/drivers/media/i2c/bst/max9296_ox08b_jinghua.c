// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM9296 ox8b jinghua Deserializer for BST Deserializer Driver
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

#define MODULE_NAME "bst,max9296-deser-jinghua"
#define max9296_N_LINKS 4

//ox08b raw14 setting have data and vysnc
unsigned short max9296_regs03[][2] = {
	{ 0x0330, 0x04 }, { 0x0333, 0x4e }, { 0x0334, 0xe4 }, { 0x0334, 0xe4 },
	{ 0x0335, 0x00 }, { 0x0335, 0x00 }, { 0x040a, 0x00 }, { 0x044a, 0xc0 },
	{ 0x048a, 0xc0 }, { 0x04ca, 0x00 }, { 0x031d, 0x00 }, { 0x0320, 0x2f },
	{ 0x0050, 0x00 }, { 0x0051, 0x01 }, { 0x0052, 0x02 }, { 0x0325, 0x80 },
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
};


static char fsync_mode_default[20] = "automatic"; /* manual, automatic, semi-automatic, external */
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

static void config_max9296_regs(struct i2c_client *client)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(max9296_regs03); i++) {
		if (write_reg(client, max9296_regs03[i][0],
			      max9296_regs03[i][1])) {
			usleep_range(2000, 2500);
			pr_info("%s() line %d: write_max9296_reg failed!\n",
			       (char *)__func__, (int)__LINE__);
		}
		pr_info("%s() line %d, write max9296 reg:%#x, val:%#x",
		       (char *)__func__, (int)__LINE__, max9296_regs03[i][0],
		       max9296_regs03[i][1]);
	}
}

static int max9296_initialize(struct max9296_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;

	//write_reg(hub->i2c_client, 0x313, 0x00);		/* disable CSI output */

	config_max9296_regs(hub->i2c_client);

	/*TODO: link A/B config*/
	// for(i=0; i < priv->n_links; i++)
	// {
	//	max9296_reverse_channel_setup(priv, i);
	// }
	//write_reg(hub->i2c_client, 0x313, 0x02);		/* enable CSI output */
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
	pr_info("%s!\n", __func__);
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
	u32 addrs[4], naddrs;
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
	int pdb_gpio = -1;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
	priv->hub.dev = &client->dev;
	priv->hub.deser_boot_flag = false;

	// power up deserializer at first
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

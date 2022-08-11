// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * n4 yuv camera for BST Camera Driver
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
#include "n4.h"
#include "jaguar1/jaguar1_video.h"

#define MODULE_NAME "bst,n4"

extern int jaguar1_init(struct i2c_client *client, video_init_all *param);

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

static int n4_lp11(struct deser_hub_dev *hub)
{
	register_write(hub, 0xff, 0x20);
	register_write(hub, 0x00, 0x00);
	return 0;
}

static int n4_lp00(struct deser_hub_dev *hub)
{
	register_write(hub, 0xff, 0x20);
	register_write(hub, 0x00, 0xff);
	return 0;
}

static int n4_config(struct deser_hub_dev *hub)
{
	video_init_all config;
	int i;

	for (i = 0; i < 4; i++) {
		u32 size_info[2] = { 0, 0 };
		struct device_node *node = hub->chn[i].camera_node;

		if (!of_property_read_u32_array(node, "size", size_info, 2)) {
			if (size_info[0] == 1920 && size_info[1] == 1080) {
				config.ch_param[i].format = AHD20_1080P_25P;
			} else if (size_info[0] == 1280 && size_info[1] == 720) {
				config.ch_param[i].format = AHD20_720P_25P_EX_Btype;
			} else {
				dev_err(hub->dev, "%s, unsupported size %dx%d\n",
					__func__, size_info[0], size_info[1]);
				return -1;
			}
		} else {
			dev_err(hub->dev, "%s, camera info not found\n", __func__);
			return -1;
		}

		config.ch_param[i].input = SINGLE_ENDED;
		config.ch_param[i].interface = YUV_422;
		config.ch_param[i].ch = i;
	}
	jaguar1_init(hub->i2c_client, &config);

	return 0;
}

static int n4_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int n4_s_power(struct v4l2_subdev *sd, int enable)
{
	struct deser_hub_dev *hub = container_of(sd, struct deser_hub_dev, subdev);

	n4_lp00(hub);
	hub->deser_boot_flag = true;

	return 0;
}

static struct v4l2_subdev_video_ops n4_video_ops = {
	.s_stream = n4_s_stream,
};

static struct v4l2_subdev_core_ops n4_core_ops = {
	.s_power = n4_s_power,
};

static struct v4l2_subdev_ops n4_ops = {
	.core = &n4_core_ops,
	.video = &n4_video_ops,
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

static int parse_device_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	return 0;
}

static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	int i;

	for (i = 0; i < 4 /* hardcode */; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s: input port%d not found\n ", __func__, i);
			break;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s: input device%d not found\n", __func__, i);
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

static int parse_dt(struct deser_hub_dev *hub)
{
	struct device_node *node = hub->dev->of_node;

	if (!node)
		return -EINVAL;

	if (parse_device_dt(hub, node)) {
		dev_err(hub->dev, "parse device dt failed\n");
		return -1;
	}

	if (parse_input_dt(hub, node)) {
		dev_err(hub->dev, "parse input dt failed\n");
		return -1;
	}

	if (parse_output_dt(hub, node)) {
		dev_err(hub->dev, "parse output dt failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev(struct deser_hub_dev *hub)
{
	int ret;
	struct v4l2_subdev *sd;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &n4_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, "register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct deser_hub_dev *hub)
{
	int ret;
	int i;
	int index;

	if (!hub->num_cameras) {
		dev_err(hub->dev, "%s: no input device found\n", __func__);
		return -1;
	}

	v4l2_async_notifier_init(&hub->notifier);
	hub->notifier.ops = &deser_async_ops;

	index = 0;
	for (i = 0; i < 4 /* hardcode */; i++) {
		if (!hub->chn[i].camera_fwnode)
			continue;

		hub->chn[i].deser_dev = hub;
		hub->chn[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
		hub->chn[i].async_dev.match.fwnode = hub->chn[i].camera_fwnode;
		v4l2_async_notifier_add_subdev(&hub->notifier, &(hub->chn[i].async_dev));
		index++;
	}

	ret = v4l2_async_subdev_notifier_register(&hub->subdev, &hub->notifier);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev notifier failed\n", __func__);
		return -1;
	}

	return 0;
}

static int enable_dev(struct deser_hub_dev *hub)
{
	int ret;
	int gpio = of_get_named_gpio(hub->dev->of_node, "reset-gpio", 0);

	ret = gpio_is_valid(gpio);
	if (!ret) {
		dev_err(hub->dev, "gpio%d invalid\n", gpio);
		return -1;
	}

	ret = devm_gpio_request(hub->dev, gpio, dev_name(hub->dev));
	if (ret) {
		dev_err(hub->dev, "gpio%d request failed\n", gpio);
		return -1;
	}

	ret = gpio_direction_output(gpio, 1);
	if (ret) {
		dev_err(hub->dev, "gpio%d output setting failed\n", gpio);
		return -1;
	}

	return 0;
}

static int n4_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int ret;
	struct deser_hub_dev *hub;

	hub = devm_kzalloc(&client->dev, sizeof(struct deser_hub_dev), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	hub->i2c_client = client;
	hub->dev = &client->dev;
	hub->deser_boot_flag = false;

	ret = register_subdev(hub);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev failed\n", __func__);
		return -EINVAL;
	}

	ret = parse_dt(hub);
	if (ret) {
		dev_err(hub->dev, "%s: parse dt failed\n", __func__);
		return -EINVAL;
	}

	ret = register_subdev_notifier(hub);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev notifier failed\n", __func__);
		return -EINVAL;
	}

	ret = enable_dev(hub);
	if (ret) {
		dev_err(hub->dev, "%s: enable device failed\n", __func__);
		return -EINVAL;
	}

	n4_config(hub);
	n4_lp11(hub);

	return 0;
}

static int n4_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id n4_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, n4_id);

static const struct of_device_id n4_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, n4_of_match);

static struct i2c_driver n4_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(n4_of_match),
	},
	.probe		= n4_probe,
	.remove		= n4_remove,
	.id_table	= n4_id,
};

module_i2c_driver(n4_driver);

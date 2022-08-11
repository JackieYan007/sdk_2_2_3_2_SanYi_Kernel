// SPDX-License-Identifier: GPL-2.0

/*
 * MAX9275 driver for BST
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

#include "../vout/bst_drm_drv.h"
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define REG16_NUM_RETRIES 1

struct max9275_priv {
	struct i2c_client *client;
	struct device *dev;
	int reset_gpio;
};

struct vout_receiver_ops max9275_ops;

int max9275_reset(void)
{
	return 0;
}

int max9275_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct device_node *np = client->dev.of_node;
	struct max9275_priv *priv;

	dev_dbg(&client->dev, "in %s\n", __func__);
	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->client = client;
	priv->dev = &client->dev;

	// max9275 reset
	ret = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(ret))
		return -ENODEV;

	priv->reset_gpio = ret;

	ret = gpio_request(priv->reset_gpio, "max9275_reset");
	if (ret) {
		gpio_free(priv->reset_gpio);
		return -ENODEV;
	}

	gpio_direction_output(priv->reset_gpio, 0);
	mdelay(1);
	gpio_direction_output(priv->reset_gpio, 1);
	mdelay(1);

	max9275_ops.reset = &max9275_reset;
	vout_register_receiver_ops(&max9275_ops);
	dev_dbg(&client->dev, "max9275 probe ok\n");

	return 0;
}

int max9275_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id max9275_of_id_table[] = {
	{
		.compatible = "bst,max9275",
	},
};
MODULE_DEVICE_TABLE(of, max9275_of_id_table);

static struct i2c_driver max9275_driver = {
	.driver = {
		.name = "bst,max9275",
		.owner = THIS_MODULE,
		.of_match_table = max9275_of_id_table,
	},
	.probe = max9275_probe,
	.remove = max9275_remove,
};
module_i2c_driver(max9275_driver);

MODULE_AUTHOR("<kanghua.li> kanghua.li@bst.ai");
MODULE_DESCRIPTION("max9275 Encoder driver");
MODULE_LICENSE("GPL");

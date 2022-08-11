// SPDX-License-Identifier: GPL-2.0

/*
 * MS7210 driver for BST
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

struct ms7210_priv {
	struct i2c_client *client;
	struct device *dev;
	int reset_gpio;
	int pclk_freq;
};

struct vout_receiver_ops ms7210_ops;

int ms7210_reset(void)
{
	return 0;
}

static int ms7210_reg16_read(struct ms7210_priv *priv, u16 reg, u8 *val)
{
	int ret, retries;
	u8 buf[2] = { reg & 0xff, reg >> 8 };

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(priv->client, buf, 2);
		if (ret == 2) {
			ret = i2c_master_recv(priv->client, buf, 1);
			if (ret == 1)
				break;
		}
	}

	if (ret < 0) {
		dev_err(priv->dev, "read fail: chip 0x%x register 0x%x: %d\n",
			priv->client->addr, reg, ret);
	} else {
		*val = buf[0];
	}
	return ret < 0 ? ret : 0;
}

static int ms7210_reg16_write(struct ms7210_priv *priv, u16 reg, u8 val)
{
	int ret, retries;
	u8 buf[3] = { reg & 0xff, reg >> 8, val };

	for (retries = REG16_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(priv->client, buf, 3);
		if (ret == 3)
			break;
	}

	if (ret < 0) {
		dev_err(priv->dev, "write fail: chip 0x%x register 0x%x: %d\n",
			priv->client->addr, reg, ret);
	} else {
#ifdef WRITE_VERIFY
		u8 val2;

		ms7210_reg16_read(priv, reg, &val2);
		if (val != val2)
			dev_err(priv->dev,
				"write verify mismatch: chip 0x%x register 0x%x: 0x%x->0x%x\n",
				priv->client->addr, reg, val, val2);
#endif
	}

	return ret < 0 ? ret : 0;
}

static int ms7210_init(struct ms7210_priv *priv)
{
	ms7210_reg16_write(priv, 0x1281, 0x04);

	// dvin enable
	ms7210_reg16_write(priv, 0x0016, 0x04);
	ms7210_reg16_write(priv, 0x0009, 0x01);

	// tx init
	ms7210_reg16_write(priv, 0x0007, 0x09);
	ms7210_reg16_write(priv, 0x0008, 0xf0);
	ms7210_reg16_write(priv, 0x000a, 0xf0);
	ms7210_reg16_write(priv, 0x0006, 0x11);
	ms7210_reg16_write(priv, 0x0531, 0x84);
	// ms7210_reg16_write(priv, 0x0080, 0x01);
	// ms7210_reg16_write(priv, 0x1201, 0x80);

	// txphy
	ms7210_reg16_write(priv, 0x0900, 0x20);
	ms7210_reg16_write(priv, 0x0901, 0x47);
	ms7210_reg16_write(priv, 0x0904, 0x09);
	ms7210_reg16_write(priv, 0x0923, 0x07);
	ms7210_reg16_write(priv, 0x0924, 0x44);
	ms7210_reg16_write(priv, 0x0925, 0x44);
	ms7210_reg16_write(priv, 0x090f, 0x80);
	ms7210_reg16_write(priv, 0x091f, 0x07);
	ms7210_reg16_write(priv, 0x0920, 0x1e);

	// tx reset
	ms7210_reg16_write(priv, 0x000b, 0x00);

	// tx mute
	ms7210_reg16_write(priv, 0x0507, 0x06);

	// txphy config
	// clk over 100Mhz
	if (priv->pclk_freq > 100000) {
		// clk over 100Mhz
		ms7210_reg16_write(priv, 0x0906, 0x04);
		ms7210_reg16_write(priv, 0x0920, 0x5e);
		ms7210_reg16_write(priv, 0x0926, 0xdd);
		ms7210_reg16_write(priv, 0x0927, 0x0d);
		ms7210_reg16_write(priv, 0x0928, 0x88);
		ms7210_reg16_write(priv, 0x0929, 0x08);
	} else {
		// clk under 100Mhz
		ms7210_reg16_write(priv, 0x0906, 0x00);
		ms7210_reg16_write(priv, 0x0920, 0x1e);
		ms7210_reg16_write(priv, 0x0926, 0x11);
		ms7210_reg16_write(priv, 0x0927, 0x01);
		ms7210_reg16_write(priv, 0x0928, 0x11);
		ms7210_reg16_write(priv, 0x0929, 0x01);
	}
	ms7210_reg16_write(priv, 0x0910, 0x01);

	ms7210_reg16_write(priv, 0x000b, 0x11);

	// avi

	ms7210_reg16_write(priv, 0x050e, 0x00);
	ms7210_reg16_write(priv, 0x050a, 0x82);
	ms7210_reg16_write(priv, 0x0509, 0x02);
	ms7210_reg16_write(priv, 0x050b, 0x0d);

	ms7210_reg16_write(priv, 0x050d, 0x06);
	ms7210_reg16_write(priv, 0x050d, 0x11);
	ms7210_reg16_write(priv, 0x050d, 0x58);
	ms7210_reg16_write(priv, 0x050d, 0x00);

	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050e, 0x40);

	ms7210_reg16_write(priv, 0x050a, 0x84);
	ms7210_reg16_write(priv, 0x0509, 0x01);
	ms7210_reg16_write(priv, 0x050b, 0x0a);
	ms7210_reg16_write(priv, 0x050d, 0x70);
	ms7210_reg16_write(priv, 0x050d, 0x01);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050d, 0x00);
	ms7210_reg16_write(priv, 0x050e, 0x60);

	ms7210_reg16_write(priv, 0x0507, 0x00);

	return 0;
}

int ms7210_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	u8 value;
	struct device_node *np = client->dev.of_node;

	struct ms7210_priv *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->client = client;
	priv->dev = &client->dev;

	// ms7210 reset
	ret = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(ret))
		return -ENODEV;

	priv->reset_gpio = ret;

	ret = gpio_request(priv->reset_gpio, "ms7210_reset");
	if (ret) {
		gpio_free(priv->reset_gpio);
		return -ENODEV;
	}

	gpio_direction_output(priv->reset_gpio, 0);
	mdelay(1);
	gpio_direction_output(priv->reset_gpio, 1);
	mdelay(1);

	// chip id for detect

	ms7210_reg16_read(priv, 0x0001, &value);
	if (value != 0x20) {
		dev_err(priv->dev, "chip id unvalid for ms7210, value = 0x%x\n",
			value);
		return -ENODEV;
	}

	ret = device_property_read_u32_array(priv->dev, "pclk_freq",
					     &priv->pclk_freq, 1);
	if (ret) {
		dev_err(priv->dev,
			"ms7210 no pclk_freq property, use default 148.5Mhz\n");
		priv->pclk_freq = 148500;
	}

	// ms7210 init

	ms7210_init(priv);

	ms7210_ops.reset = &ms7210_reset;
	vout_register_receiver_ops(&ms7210_ops);

	return 0;
}

int ms7210_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id ms7210_of_id_table[] = {
	{
		.compatible = "bst,ms7210",
	},
};
MODULE_DEVICE_TABLE(of, ms7210_of_id_table);

static struct i2c_driver ms7210_driver = {
	.driver = {
		.name = "bst,ms7210",
		.owner = THIS_MODULE,
		.of_match_table = ms7210_of_id_table,
	},
	.probe = ms7210_probe,
	.remove = ms7210_remove,
};
module_i2c_driver(ms7210_driver);

MODULE_AUTHOR("<kanghua.li> kanghua.li@bst.ai");
MODULE_DESCRIPTION("ms7210 Encoder driver");
MODULE_LICENSE("GPL");

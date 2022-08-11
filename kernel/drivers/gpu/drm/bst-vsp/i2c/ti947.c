// SPDX-License-Identifier: GPL-2.0

/*
 * TI947 driver for BST
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

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "../vout/bst_drm_drv.h"

//#define __LOG_TRACE__ 1

#define TI947_CHIP_ID 0X393437 // "_UB947"
#define TI921_CHIP_ID 0X393231 // "_UB921"

#ifndef EDID_LENGTH
#define EDID_LENGTH 0x80
#endif

#ifdef __LOG_TRACE__
#define __TRACE__ pr_info("[ HDMI info ] %s\n", __func__)
#define __MSG_TRACE__(string, args...) \
	pr_info("[ HDMI info ] "       \
		" %s : %d : " string,  \
		__func__, __LINE__, ##args)
#else
#define __TRACE__
#define __MSG_TRACE__(string, args...)
#endif

struct ti947_data {
	struct i2c_client *client;
} *ti947;

struct vout_receiver_ops ti947_ops;

static u64 supported_chip_id[] = { TI947_CHIP_ID, TI921_CHIP_ID };

static struct i2c_client *ti947_to_i2c(struct ti947_data *ti947)
{
	__TRACE__;
	return ti947->client;
}

static s32 ti947_write(const struct i2c_client *client, u8 command, u8 value)
{
	__TRACE__;
	return i2c_smbus_write_byte_data(client, command, value);
}

static s32 ti947_read(const struct i2c_client *client, u8 command)
{
	int val;

	__TRACE__;
	val = i2c_smbus_read_word_data(client, command);

	return val & 0xff;
}

static void ti947_write_i2c(struct ti947_data *ti947, int reg, int val)
{
	struct i2c_client *client = ti947_to_i2c(ti947);
	int ret, cnt;

	__TRACE__;

	for (cnt = 0; cnt < 5; cnt++) {
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
		ret = ti947_write(client, reg, val);
		if (ret < 0)
			break;
	}
	if (ret)
		dev_err(&client->dev, "could not find device\n");
}

static int ti947_id_get(struct ti947_data *ti947)
{
	u8 val;
	u64 chip_id = 0x0;
	int i;
	struct i2c_client *client = ti947_to_i2c(ti947);

	val = ti947_read(client, 0xf3);
	chip_id = (val << 16);
	val = ti947_read(client, 0xf4);
	chip_id = (val << 8) | chip_id;
	val = ti947_read(client, 0xf5);
	chip_id = (val | chip_id);

	for (i = 0; i < ARRAY_SIZE(supported_chip_id); i++)
		if (chip_id == supported_chip_id[i])
			return 0;

	return -1;
}

static void ti947_device_init(struct ti947_data *ti947)
{
	/* input bus/pixel: full pixel wide (24bit), rising edge */
	ti947_write_i2c(ti947, 0x40, 0x10);
	ti947_write_i2c(ti947, 0x41, 0x49);
	ti947_write_i2c(ti947, 0x42, 0x16);
	ti947_write_i2c(ti947, 0x41, 0x47);
	ti947_write_i2c(ti947, 0x42, 0x20);
	ti947_write_i2c(ti947, 0x42, 0xa0);
	ti947_write_i2c(ti947, 0x42, 0x20);
	ti947_write_i2c(ti947, 0x42, 0x00);
	ti947_write_i2c(ti947, 0x41, 0x49);
	ti947_write_i2c(ti947, 0x42, 0x00);
	ti947_write_i2c(ti947, 0x4f, 0x80);
}

int ti947_reset(void)
{
	// nothing to do
	return 0;
}

static int ti947_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	int ret;
	int pdb_gpio = -1;
	struct device *dev = &client->dev;

	__TRACE__;

	pr_info("zxj %s\n", __func__);
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

	ti947 = devm_kzalloc(&client->dev, sizeof(*ti947), GFP_KERNEL);
	if (!ti947)
		return -ENOMEM;

	pdb_gpio = of_get_named_gpio(client->dev.of_node, "pdb-gpio", 0);
	if (gpio_is_valid(pdb_gpio)) {
		pr_info("pdb %s\n", __func__);
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

	ti947->client = client;
	i2c_set_clientdata(client, ti947);

	ret = ti947_id_get(ti947);
	if (ret < 0)
		return ret;

	ti947_device_init(ti947);
	ti947_ops.reset = &ti947_reset;
	vout_register_receiver_ops(&ti947_ops);

	pr_info("zxj %s ok\n", __func__);

	return 0;
}

static int ti947_remove(struct i2c_client *client)
{
	__TRACE__;

	return 0;
}

static const struct i2c_device_id ti947_id[] = {
	{ "ti947", 0 },
	{ "ti921", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ti947_id);

/* clang-format off */
static const struct of_device_id ti947_dt_ids[] = {
	{ .compatible = "bst,ti947" },
	{ .compatible = "bst,ti921" },
};
/* clang-format on */
MODULE_DEVICE_TABLE(of, ti947_dt_ids);

static struct i2c_driver ti947_i2c_driver = {
	.driver = {
		.name = "ti947",
		.owner = THIS_MODULE,
		.of_match_table = ti947_dt_ids,
	},
	.probe = ti947_probe,
	.remove = ti947_remove,
	.id_table = ti947_id,
};
module_i2c_driver(ti947_i2c_driver);

MODULE_AUTHOR("<jim.zheng> jim.zheng@bst.ai");
MODULE_DESCRIPTION("ti947 serdes driver");
MODULE_LICENSE("GPL");

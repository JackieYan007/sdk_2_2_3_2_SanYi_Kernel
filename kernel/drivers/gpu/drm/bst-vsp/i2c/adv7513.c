// SPDX-License-Identifier: GPL-2.0

/*
 * ADV7513 driver for BST
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
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <video/edid.h>

#include "../vout/bst_drm_drv.h"

#define ADV7513_CHIP_ID 0x7511

//#define __LOG_TRACE__ 1

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

struct adv7513_data {
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct regmap *regmap;
	unsigned int irq;
	struct device *dev;
	const char *format;
	u8 cable_plugin;
} *adv7513;

struct vout_receiver_ops adv7513_ops;

static void adv7513_poweron(void);
static void adv7513_poweroff(void);

static struct i2c_client *adv7513_to_i2c(struct adv7513_data *adv7513)
{
	__TRACE__;
	return adv7513->client;
}

static s32 adv7513_write(const struct i2c_client *client, u8 command, u8 value)
{
	__TRACE__;
	return i2c_smbus_write_byte_data(client, command, value);
}

static s32 adv7513_read(const struct i2c_client *client, u8 command)
{
	int val;

	__TRACE__;
	val = i2c_smbus_read_word_data(client, command);

	return val & 0xff;
}

static int __adv7513_read_edid(struct i2c_adapter *adp, unsigned char *edid,
			       u8 *buf)
{
	unsigned short addr = 0x50;
	int ret;

	struct i2c_msg msg[2] = {
		{
			.addr = addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = EDID_LENGTH,
			.buf = edid,
		},
	};

	__TRACE__;

	if (adp == NULL)
		return -EINVAL;

	memset(edid, 0, EDID_LENGTH);

	ret = i2c_transfer(adp, msg, 2);
	if (ret < 0)
		return ret;

	/* If 0x50 fails, try 0x39. */
	if (edid[1] == 0x00) {
		msg[0].addr = msg[1].addr = 0x39;
		ret = i2c_transfer(adp, msg, 2);
		if (ret < 0)
			return ret;
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	return 0;
}

static int __adv7513_get_edid(struct i2c_adapter *adp)
{
	u8 *edid;
	u8 buf[2] = { 0, 0 };
	int num, ret;

	__TRACE__;

	edid = kzalloc(EDID_LENGTH, GFP_KERNEL);
	if (!edid)
		return -ENOMEM;

	ret = __adv7513_read_edid(adp, edid, buf);
	if (ret)
		return ret;

	/* need read ext block? Only support one more blk now*/
	num = edid[0x7E];
	if (num) {
		if (num > 1)
			pr_info("Edid has %d ext block, but now only support 1 ext blk\n",
				num);

		buf[0] = 0x80;
		ret = __adv7513_read_edid(adp, edid, buf);
		if (ret)
			return ret;

		//zxj fb_edid_add_monspecs(edid, &monspecs);
	}

	kfree(edid);
	return 0;
}

static void adv7513_get_edid(void)
{
	int ret;

	__TRACE__;

	/* edid reading */
	ret = __adv7513_get_edid(adv7513->client->adapter);

	//	return ret;
}

static int adv7513_chip_id(struct adv7513_data *adv7513)
{
	struct i2c_client *client = adv7513_to_i2c(adv7513);
	int val, chip_id = 0x0;

	__TRACE__;

	/* read device ID */
	val = adv7513_read(client, 0xf5);
	chip_id = (val << 8);
	val = adv7513_read(client, 0xf6);
	chip_id = (val | chip_id);
	dev_info(&client->dev, "read id = 0x%02X", chip_id);

	if (chip_id != ADV7513_CHIP_ID)
		return -1;

	return 0;
}

static void adv7513_write_i2c(struct adv7513_data *adv7513, int reg, int val)
{
	struct i2c_client *client = adv7513_to_i2c(adv7513);
	int ret, cnt;

	__TRACE__;

	for (cnt = 0; cnt < 5; cnt++) {
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
		ret = adv7513_write(client, reg, val);
		if (ret < 0)
			break;
	}
	if (ret)
		dev_err(&client->dev, "cound not find device\n");
}

static void adv7513_device_init(struct adv7513_data *adv7513)
{
	if (strcmp(adv7513->format, "rgb444") == 0) {
		adv7513_write_i2c(adv7513, 0x98, 0x03);
		adv7513_write_i2c(adv7513, 0x9a, 0xe0);
		adv7513_write_i2c(adv7513, 0x9c, 0x30);
		adv7513_write_i2c(adv7513, 0x9d, 0x01);
		adv7513_write_i2c(adv7513, 0xa2, 0xa4);
		adv7513_write_i2c(adv7513, 0xa3, 0xa4);
		adv7513_write_i2c(adv7513, 0xe0, 0xd0);
		adv7513_write_i2c(adv7513, 0xf9, 0x00);
		adv7513_write_i2c(adv7513, 0x15, 0x00);
		adv7513_write_i2c(adv7513, 0x16, 0x34);
		adv7513_write_i2c(adv7513, 0x17, 0x02);
		adv7513_write_i2c(adv7513, 0x18, 0x00);
		adv7513_write_i2c(adv7513, 0xaf, 0x06);
	} else {
		/* input bus/pixel: full pixel wide (24bit), rising edge */
		adv7513_write_i2c(adv7513, 0x15, 0x00);
		adv7513_write_i2c(adv7513, 0x16, 0x30);
		adv7513_write_i2c(adv7513, 0x98, 0x03);
		adv7513_write_i2c(adv7513, 0x9a, 0xe0);
		adv7513_write_i2c(adv7513, 0x9c, 0x30);
		adv7513_write_i2c(adv7513, 0x9d, 0x61);
		adv7513_write_i2c(adv7513, 0xa2, 0xa4);
		adv7513_write_i2c(adv7513, 0xa3, 0xa4);
		adv7513_write_i2c(adv7513, 0xe0, 0xd0);
		adv7513_write_i2c(adv7513, 0xf9, 0x00);
		adv7513_write_i2c(adv7513, 0x18, 0xf7);
		adv7513_write_i2c(adv7513, 0x55, 0x02);
		adv7513_write_i2c(adv7513, 0x56, 0x28);
		adv7513_write_i2c(adv7513, 0xd6, 0xc0);
		adv7513_write_i2c(adv7513, 0xaf, 0x06);
		adv7513_write_i2c(adv7513, 0xf9, 0x00);
	}
}

int adv7513_reset(void)
{
	if (!adv7513)
		return 0;

	adv7513_poweroff();
	mdelay(2);
	adv7513_poweron();
	mdelay(5);
	adv7513_device_init(adv7513);

	return 0;
}

static int adv7513_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);

	__TRACE__;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

	adv7513 = devm_kzalloc(&client->dev, sizeof(*adv7513), GFP_KERNEL);
	if (!adv7513)
		return -ENOMEM;

	adv7513->client = client;
	adv7513->dev = &client->dev;
	i2c_set_clientdata(client, adv7513);

	device_property_read_string(adv7513->dev, "input_format",
				    &adv7513->format);
	if (adv7513->format != NULL) {
		pr_info("input data format for adv7513 is %s\n",
			adv7513->format);
	} else {
		adv7513->format = "yuv444";
	}

	ret = adv7513_chip_id(adv7513);
	if (ret < 0)
		return ret;

	adv7513_reset();

	adv7513_ops.reset = &adv7513_reset;
	vout_register_receiver_ops(&adv7513_ops);

	// detect resolution
	// adv7513_get_edid();

	return 0;
}

static int adv7513_remove(struct i2c_client *client)
{
	__TRACE__;

	adv7513_poweroff();
	return 0;
}

static void adv7513_poweron(void)
{
	__TRACE__;

	/* Turn on DVI or HDMI */
	adv7513_write(adv7513->client, 0x41, 0x10);
}

static void adv7513_poweroff(void)
{
	__TRACE__;

	/* disable tmds before changing resolution */
	adv7513_write(adv7513->client, 0x41, 0x50);
}

static const struct i2c_device_id adv7513_id[] = {
	{ "adv7513", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7513_id);

static const struct of_device_id adv7513_dt_ids[] = { { .compatible =
								"bst,adv7513" },
						      {} };
MODULE_DEVICE_TABLE(of, adv7513_dt_ids);

static struct i2c_driver adv7513_i2c_driver = {
	.driver = {
		.name = "adv7513",
		.owner = THIS_MODULE,
		.of_match_table = adv7513_dt_ids,
	},
	.probe = adv7513_probe,
	.remove = adv7513_remove,
	.id_table = adv7513_id,
};
module_i2c_driver(adv7513_i2c_driver);

MODULE_AUTHOR("<jim.zheng> jim.zheng@bst.ai");
MODULE_DESCRIPTION("adv7513 DVI/HDMI driver");
MODULE_LICENSE("GPL");

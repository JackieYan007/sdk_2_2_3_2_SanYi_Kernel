// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM Derser Hub Tool for BST Deserializer Driver
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
#include "maxim_deser_hub.h"
#include "camera_common_op.h"
#include <linux/delay.h>
//#define MAXIM_HUB_DEBUG

int write_register(struct i2c_adapter *adap, uint8_t slave_address, uint16_t reg_offset, uint8_t value)
{
	int ret = -1;
	int retry_times = 50;

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_write_byte_data_word_reg(adap, slave_address, reg_offset, value);
		if (ret < 0) {
			retry_times--;
			usleep_range(1000, 2000);
		}
	}

	if (retry_times <= 0) {
		pr_info("%s() line:%d, write %x:[%x,%x]failed!\n",
				__func__, __LINE__, slave_address, reg_offset, value);
		return -1;
	}
	return 0;
}

/*
 *	8 bit addr
 *	16bit reg
 *	8bit value
 */
int ser_word_write(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t value)
{
	int ret = -1;
	int retry_times = MAXIM_DESER_I2C_RETRY_TIMES;
	struct i2c_adapter *adap;

	adap = hub->i2c_client->adapter;

	if (adap == NULL) {
		return -EINVAL;
	};

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_write_byte_data_word_reg(adap, addr, reg, value);
		if (ret < 0) {
			retry_times--;
			mdelay(5);
			continue;
		}
	}

	if (retry_times <= 0) {
		dev_err(hub->dev, "%s() line:%d, write %#x:[%#x,%#x]failed!\n",
				__func__, __LINE__, addr, reg, value);
		return -1;
	}
	return 0;
}

int ser_word_read(struct deser_hub_dev *hub, uint8_t addr, uint16_t reg, uint8_t *value)
{
	int ret = -1;
	int retry_times = MAXIM_DESER_I2C_RETRY_TIMES;
	struct i2c_adapter *adap;

	adap = hub->i2c_client->adapter;

	if (adap == NULL) {
		return -EINVAL;
	};

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_read_byte_data_word_reg(adap, addr, reg, value);
		if (ret < 0) {
			retry_times--;
			mdelay(5);
			continue;
		}
	}

	if (retry_times <= 0) {
		dev_err(hub->dev, "%s() line:%d, read %#x:[%#x,%#x]failed!\n",
				__func__, __LINE__, addr, reg, *value);
		return -1;
	}
	return 0;
}

int ser_write(struct deser_hub_dev *hub, uint8_t addr, uint8_t reg, uint8_t value)
{
	int ret = -1;
	int retry_times = MAXIM_DESER_I2C_RETRY_TIMES;
	struct i2c_adapter *adap;

	adap = hub->i2c_client->adapter;

	if (adap == NULL) {
		return -EINVAL;
	};

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_write_byte_data_byte_reg(adap, addr, reg, value);
		if (ret < 0) {
			retry_times--;
			mdelay(5);
			continue;
		}
	}

	if (retry_times <= 0) {
		dev_err(hub->dev, "%s() line:%d, write %x:[0x%2x,0x%2x]failed!\n",
				__func__, __LINE__, addr, reg, value);
	}
	return 0;
}

int reg8_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret, retries;

	for (retries = MAXIM_DESER_I2C_RETRY_TIMES; retries; retries--) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if ((ret < 0))
			mdelay(5);
		else
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
		*val = ret;
	}

	return ret < 0 ? ret : 0;
}

int write_reg(struct i2c_client *client, uint16_t reg, int val)
{
	int ret = -1;
	int retry_times = MAXIM_DESER_I2C_RETRY_TIMES;

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_write_byte_data_word_reg(client->adapter, client->addr, reg, val);
		if (ret < 0) {
			retry_times--;
			mdelay(5);
			continue;
		}
	}

	if (retry_times <= 0) {
		pr_info("%s() line:%d, write %x:[%x,%x]failed!\n",
				__func__, __LINE__, client->addr, reg, val);
		return -1;
	}
	return 0;
}

void config_ser_reg_group(unsigned short (*group)[2], int len, struct deser_hub_dev *hub, int reg)
{
	int i = 0;

	for (i = 0; i < len ; i++) {
		if (ser_word_write(hub, reg, group[i][0], group[i][1])) {
			usleep_range(1000, 2000);
			pr_info("%s() line %d: write_max9296_reg failed!\n", (char *)__func__, (int)__LINE__);
		}
		pr_info("%s() line %d, write max9295 reg:%#x, val:%#x",
				(char *)__func__, (int)__LINE__,
				group[i][0], group[i][1]);
	}
}

int max9286_reg_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value)
{
	int ret, retries;
	struct i2c_client *client = hub->i2c_client;

	for (retries = MAXIM_DESER_I2C_RETRY_TIMES; retries; retries--) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(hub->dev,
			"write fail: chip 0x%x register 0x%x: %d\n",
			client->addr, reg, ret);
	} else {
	#ifdef WRITE_VERIFY
		u8 val2;

		reg8_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
				"write verify mismatch: chip 0x%x reg=0x%x 0x%x->0x%x\n",
				client->addr, reg, val, val2);
	#endif
	}

	return ret < 0 ? ret : 0;
	if (ret < 0)
		dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);
	return ret;
}

int max96712_reg_read(struct deser_hub_dev *hub, uint16_t reg, uint8_t *value)
{
	int ret = -1;
	int retry_times = MAXIM_DESER_I2C_RETRY_TIMES;
	struct i2c_adapter *adap;
	struct i2c_client *client;

	client = hub->i2c_client;
	adap = hub->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	while ((retry_times != 0) && ret) {
		ret = bst_i2c_read_byte_data_word_reg(adap, client->addr, reg, value);
		if (ret < 0) {
			retry_times--;
			mdelay(5);
			continue;
		}
	}

	if (retry_times <= 0) {
		dev_err(hub->dev, "%s() line:%d, read %#x:[%#x,%#x]failed!\n",
				__func__, __LINE__, client->addr, reg, *value);
		return -1;
	}
	return 0;
}

int max96712_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value)
{
	int ret = write_reg(hub->i2c_client, reg, value);

	if (ret < 0)
		dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);
#ifdef MAXIM_HUB_DEBUG
	else
		dev_info(hub->dev, "%s: write [0x%02x,0x%02x] success\n", __func__, reg, value);
#endif
	return ret;
}

int max9296_reg_write(struct deser_hub_dev *hub, uint16_t reg, uint8_t value)
{
	int ret = write_reg(hub->i2c_client, reg, value);

	if (ret < 0)
		dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);
	return ret;
}

int is_gmsl2_video_connected(struct deser_hub_dev *hub, int index)
{
	uint8_t value = 0;
	int ret = 0;
	int retry_times = 3;

	while (value == 0 && retry_times > 0) {
		switch (index) {
		case 0:
			ret = max96712_reg_read(hub, MAXIM_VIDEO_GSML2_LOCK_A, &value);
			dev_info(hub->dev, "%s() read Reg:[%d], val:[%d]\n",
					__func__, MAXIM_VIDEO_GSML2_LOCK_A, value);
			break;
		case 1:
			ret = max96712_reg_read(hub, MAXIM_VIDEO_GSML2_LOCK_B, &value);
			dev_info(hub->dev, "%s() read Reg:[%d], val:[%d]\n",
					__func__, MAXIM_VIDEO_GSML2_LOCK_B, value);
			break;
		case 2:
			ret = max96712_reg_read(hub, MAXIM_VIDEO_GSML2_LOCK_C, &value);
			dev_info(hub->dev, "%s() read Reg:[%d], val:[%d]\n",
					__func__, MAXIM_VIDEO_GSML2_LOCK_C, value);
			break;
		case 3:
			ret = max96712_reg_read(hub, MAXIM_VIDEO_GSML2_LOCK_D, &value);
			dev_info(hub->dev, "%s() read Reg:[%d], val:[%d]\n",
					__func__, MAXIM_VIDEO_GSML2_LOCK_D, value);
			break;
		default:
			dev_err(hub->dev, "%s() video index = %d, invalid param\n",
				__func__, index);
			break;
		}
		mdelay(5);
		retry_times--;
	}

	if (ret) {
		dev_info(hub->dev, "%s() read Reg:gsml2_link_lock failed\n",
			__func__);
		return 0;
	}
	//gsml2 link bit3:  0:not lock, 1:link lock
	if ((value & (1 << 3)) == 0) {
		dev_info(hub->dev, "%s() index = %d, not linked\n",
			__func__, index);
		return 0;
	}

	hub->chn[index].cam_dev->maxim_power_on = true;
	return 1;
}

// SPDX-License-Identifier: GPL-2.0

/*
 * ADV7125 driver for BST
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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "../vout/bst_drm_drv.h"

struct vout_receiver_ops adv7125_ops;

int adv7125_reset(void)
{
	// nothing to do
	return 0;
}

int adv7125_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int gpio;
	int value;
	enum of_gpio_flags flag;

	struct device_node *psave_gpio_node;

	psave_gpio_node = pdev->dev.of_node;
	gpio = of_get_named_gpio_flags(psave_gpio_node, "psave_gpio", 0, &flag);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	ret = gpio_request(gpio, "psave_gpio");
	if (ret) {
		gpio_free(gpio);
		return -ENODEV;
	}

	value = (flag == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	gpio_direction_output(gpio, value);

	adv7125_ops.reset = &adv7125_reset;
	vout_register_receiver_ops(&adv7125_ops);

	return 0;
}

int adv7125_remove(struct platform_device *pdev)
{
	unsigned int gpio;

	enum of_gpio_flags flag;

	struct device_node *psave_gpio_node;

	psave_gpio_node = pdev->dev.of_node;

	gpio = of_get_named_gpio_flags(psave_gpio_node, "psave_gpio", 0, &flag);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	gpio_free(gpio);

	return 0;
}

static const struct of_device_id adv7125_of_id_table[] = {
	{ .compatible = "bst,adv7125" },
	{}
};
MODULE_DEVICE_TABLE(of, adv7125_of_id_table);

static struct platform_driver adv7125_driver = {
	.driver = {
		.name = "bst,adv7125",
		.owner = THIS_MODULE,
		.of_match_table = adv7125_of_id_table,
	},
	.probe = adv7125_probe,
	.remove = adv7125_remove,
};
module_platform_driver(adv7125_driver);

MODULE_AUTHOR("<kanghua.li> kanghua.li@bst.ai");
MODULE_DESCRIPTION("adv7125 DAC driver");
MODULE_LICENSE("GPL");

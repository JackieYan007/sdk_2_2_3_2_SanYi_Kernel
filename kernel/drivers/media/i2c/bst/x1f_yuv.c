// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/media-device.h>

#include "ti_deser_hub.h"
#include "maxim_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,x1f-isp"

static int max96705_is_connect(struct i2c_client *client)
{
	int ser_addr;
	u8 val = 0;

	struct camera_dev *cam = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	ser_addr = cam->ser_alias_id;

	client->addr = ser_addr;

	reg8_read(client, MAXIM_ID_REG, &val);
	dev_err(dev, "addr: %x read_reg: %x\n", client->addr, MAXIM_ID_REG);
	if (val == MAX96701_ID)
		return 0;
	else
		return -1;
}

static int x1f_isp_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *cam = container_of(sd, struct camera_dev, subdev);
	struct device *dev = cam->dev;
	int ret = 0;

	if (!enable)
		return 0;

	ret = max96705_is_connect(cam->i2c_client);
	if (!ret) {
		dev_err(dev, "camera connected\n");
	} else {
		dev_err(dev, "camera disconnected\n");
		return ret;
	}
	cam->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_core_ops x1f_isp_core_ops = {
	.s_power = x1f_isp_s_power,
};

static struct v4l2_subdev_ops x1f_isp_ops = {
	.core = &x1f_isp_core_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations x1f_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static int x1f_isp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *cam;
	struct device *dev = &client->dev;
	int ret = 0;

	cam = devm_kzalloc(&client->dev, sizeof(struct camera_dev), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	if (client->adapter == NULL) {
		dev_err(dev, "client->adapter NULL\n");
		return -ENXIO;
	}


	cam->i2c_client = client;
	cam->dev = dev;
	cam->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	i2c_set_clientdata(client, cam);

	ret = parse_camera_endpoint(cam, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(cam, &x1f_isp_ops
		, &x1f_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	pr_info("<8> ar0143 probe ok !\n");

	return 0;
}

static int x1f_isp_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id x1f_isp_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id x1f_isp_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, x1f_isp_of_match);
MODULE_DEVICE_TABLE(i2c, x1f_isp_id);

static struct i2c_driver x1f_isp_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(x1f_isp_of_match),
	},
	.probe		= x1f_isp_probe,
	.remove		= x1f_isp_remove,
	.id_table	= x1f_isp_id,
};

module_i2c_driver(x1f_isp_driver);

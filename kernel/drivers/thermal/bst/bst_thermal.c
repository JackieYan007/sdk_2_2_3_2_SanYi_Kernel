/*
* thermal driver for BST
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>

#define PVTREG_SPI_CONTROL       0x0
#define PVTREG_SPI_SLAVE         0x8
#define PVTREG_SPI_WBUFF         0x10
#define PVTREG_SENSOR0           0x20
#define PVTREG_SENSOR1           0x24
#define PVTREG_SENSOR2           0x28
#define PVTREG_SENSOR3           0x2c
#define PVTREG_SENSOR4           0x30
#define PVTREG_TEMP_THRESHOLD    0x50

#define PVT_NUM 5
#define BST_CDEV_SIZE 1

struct bst_cooling_dev_info {
    const char *name;
    struct thermal_cooling_device *cdev;
};

struct bst_thermal {
    struct thermal_zone_device *tz;
    struct bst_cooling_dev_info bst_cdev_info[BST_CDEV_SIZE];
    void __iomem *pvtreg;
};

static int bst_get_cdev_max_state(struct thermal_cooling_device *cdev,
                    unsigned long *max_state)
{
    *max_state = 1;
    return 0;
}

static int bst_get_cdev_cur_state(struct thermal_cooling_device *cdev,
                    unsigned long *cur_state)
{
    return 0;
}

static int bst_set_cdev_cur_state(struct thermal_cooling_device *cdev,
                    unsigned long cur_state)
{
    return 0;
}

static const struct thermal_cooling_device_ops bst_cooling_ops = {
    .get_max_state = bst_get_cdev_max_state,
    .get_cur_state = bst_get_cdev_cur_state,
    .set_cur_state = bst_set_cdev_cur_state,
};

static s64 transfer(u32 sensor_val)
{
    s64 temp, val;

    val = sensor_val;
    temp = -49002000 + 325120*val - 17058*val*val/100 + 60373*val*val*val/1000000
                - 92627*val*val*val*val/10000000000;
    temp /= 1000000;
    return temp;
}

static int bst_thermal_get_temp(void *data, int *temp)
{
    u32 reg;
    u32 val;
    struct bst_thermal *bt = data;

    reg = readl(bt->pvtreg + PVTREG_SENSOR0);
    val = reg>>3;
    *temp = transfer(val) * 1000;
    return 0;
}

static const struct thermal_zone_of_device_ops bst_thermal_ops = {
    .get_temp = bst_thermal_get_temp,
};

static ssize_t ddr0_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct bst_thermal *bt = platform_get_drvdata(pdev);
    u32 reg, val;
    int temp;

    reg = readl(bt->pvtreg + PVTREG_SENSOR1);
    val = reg>>3;
    temp = transfer(val) * 1000;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t ddr1_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct bst_thermal *bt = platform_get_drvdata(pdev);
    u32 reg, val;
    int temp;

    reg = readl(bt->pvtreg + PVTREG_SENSOR2);
    val = reg>>3;
    temp = transfer(val) * 1000;
    return sprintf(buf, "%d\n", temp);
}

static ssize_t net_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct bst_thermal *bt = platform_get_drvdata(pdev);
    u32 reg, val;
    int temp;

    reg = readl(bt->pvtreg + PVTREG_SENSOR3);
    val = reg>>3;
    temp = transfer(val) * 1000;
    return sprintf(buf, "%d\n", temp);
}

static ssize_t cv_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct bst_thermal *bt = platform_get_drvdata(pdev);
    u32 reg, val;
    int temp;

    reg = readl(bt->pvtreg + PVTREG_SENSOR4);
    val = reg>>3;
    temp = transfer(val) * 1000;
    return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR_RO(ddr0_temp);
static DEVICE_ATTR_RO(ddr1_temp);
static DEVICE_ATTR_RO(net_temp);
static DEVICE_ATTR_RO(cv_temp);
static struct attribute *temp_attrs[] = {
    &dev_attr_ddr0_temp.attr,
    &dev_attr_ddr1_temp.attr,
    &dev_attr_net_temp.attr,
    &dev_attr_cv_temp.attr,
    NULL
};

static const struct attribute_group temp_attribute_group = {
    .attrs = temp_attrs,
    .name = "temps"
};

static void pvt_start(struct bst_thermal *bt)
{
    u32 index;
    u32 wbuff, slave, sensor_mode;
    u32 trimg, trimo;

    writel(0x306, bt->pvtreg + PVTREG_TEMP_THRESHOLD);
    writel(0xc0, bt->pvtreg + PVTREG_SPI_CONTROL);

    for (index = 0; index < PVT_NUM; index++) {
        wbuff = 3<<14 | 0x3fff;
        slave = ~(1<<index);
        writel(slave, bt->pvtreg + PVTREG_SPI_SLAVE);
        writel(wbuff, bt->pvtreg + PVTREG_SPI_WBUFF);

        trimg = 0xf;
        trimo = 0;
        wbuff = 1<<15 | trimo<<5 | trimg;
        writel(wbuff, bt->pvtreg + PVTREG_SPI_WBUFF);

        sensor_mode = 0;
        wbuff = 1<<14 | 0<<4 | sensor_mode<<1;
        writel(sensor_mode, bt->pvtreg + PVTREG_SENSOR0 + index * 4);
        writel(wbuff, bt->pvtreg + PVTREG_SPI_WBUFF);

        wbuff = 1<<14 | 1<<4 | sensor_mode<<1;
        writel(wbuff, bt->pvtreg + PVTREG_SPI_WBUFF);
        msleep(2);
    }
}

static void init_cooling_device(struct platform_device *pdev)
{
    struct device_node *np, *child;
    struct bst_thermal *bt = platform_get_drvdata(pdev);
    int cdev_index;
    
    cdev_index = 0;
    np = of_find_node_by_name(NULL, "cooling_dev");
    if (!np) {
        dev_info(&pdev->dev, "can't find bst cooling_dev\n");
        return;
    }

    for_each_available_child_of_node(np, child) {
        struct thermal_cooling_device *tcd;

        if (cdev_index + 1 > BST_CDEV_SIZE) {
            of_node_put(child);
            of_node_put(np);
            return;
        }
        dev_info(&pdev->dev, "cooling_dev, name=%s", child->name);
        bt->bst_cdev_info[cdev_index].name = child->name;

        tcd = thermal_of_cooling_device_register(child, (char *)child->name, bt, &bst_cooling_ops);
        if (IS_ERR_OR_NULL(tcd)) {
            dev_err(&pdev->dev, "bst cooling_dev: %s: failed to register cooling device\n",child->name);
            continue;
        }
        bt->bst_cdev_info[cdev_index].cdev = tcd;
        cdev_index++;
    }
    of_node_put(np);
}

static int bst_thermal_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct bst_thermal *bt;
    int ret;

    bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
    if (!bt)
        return -ENOMEM;

    platform_set_drvdata(pdev, bt);
    init_cooling_device(pdev);

    bt->pvtreg = of_iomap(dev_of_node(dev), 0);
    if (WARN_ON(!bt->pvtreg))
        return -ENOENT;

    bt->tz = devm_thermal_zone_of_sensor_register(dev, 0,
                        bt,&bst_thermal_ops);
    if (IS_ERR(bt->tz)) {
        iounmap(bt->pvtreg);
        return PTR_ERR(bt->tz);
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &temp_attribute_group);
    if (ret)
        return ret;
    pvt_start(bt);
    return 0;
}

static int bst_thermal_remove(struct platform_device *pdev)
{
    int i;
    struct bst_thermal *bt = platform_get_drvdata(pdev);

    for (i = 0; i < BST_CDEV_SIZE; i++) {
        thermal_cooling_device_unregister(bt->bst_cdev_info[i].cdev);
    }
    iounmap(bt->pvtreg);
    sysfs_remove_group(&pdev->dev.kobj, &temp_attribute_group);
    return 0;
}

static const struct of_device_id bst_thermal_of_match[] = {
    { .compatible = "bst,bst-thermal", },
    {},
};
MODULE_DEVICE_TABLE(of, bst_thermal_of_match);

static struct platform_driver bst_thermal_driver = {
    .probe		= bst_thermal_probe,
    .remove		= bst_thermal_remove,
    .driver = {
        .name = "bst-thermal",
        .of_match_table = bst_thermal_of_match,
    },
};
module_platform_driver(bst_thermal_driver);

MODULE_AUTHOR("Yuehong Zhong <yuehong.zhong@bst.ai>");
MODULE_DESCRIPTION("BST thermal driver");
MODULE_LICENSE("GPL v2");


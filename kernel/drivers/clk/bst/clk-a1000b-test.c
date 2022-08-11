/*
* clock driver for BST A1000
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/


#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-conf.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/module.h>
#include "clk-bsta1000b.h"
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <dt-bindings/clock/bst-a1000b.h>

#define PRT_HEAD        "[BST][CLK]: "

typedef struct _clktest_struct_{
    int opsid;      //define what operation should do
    int index;      //get clk by index or clk_name[]
#define GET_CLK_RATE        (1)
#define RND_CLK_RATE        (2)
#define WRT_CLK_RATE        (3)
#define ENABLE_CLK          (4)
#define DISABLE_CLK         (5)
    unsigned long rate;
    char clk_name[50];
}clktest_struct;

static clktest_struct bst_clk_test;
static struct clk * bst_clk_ptr;
static struct device *bst_clk_test_dev;

struct clk_core { 
	const char		*name; 
}; 

struct clk {
	struct clk_core	*core;
	const char *dev_id;
	const char *con_id;
	unsigned long min_rate;
	unsigned long max_rate;
	unsigned int exclusive_count;
	struct hlist_node clks_node;
};

static const struct of_device_id of_clktest_match[] = {
	{
		.compatible = "bst,a1000b-clkc-test"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_clktest_match);

static int bst_clk_test_dev_major = 0; //kernel auto alloc

static int clktest_open(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT   PRT_HEAD"clktest_open\n");
	return 0;
}

static ssize_t clktest_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    unsigned long rate;

	if (size != sizeof(clktest_struct)) {
        printk(KERN_ALERT   PRT_HEAD"%s:%d, length error\n", __func__, __LINE__);
		return -EINVAL;
    }

    copy_from_user(&bst_clk_test, buf, size);
    printk(KERN_ALERT   PRT_HEAD"%s:%d, opsid:%d(1(get), 2(round), 3(set), 4(enable), 5(disable)\n", __func__, __LINE__, bst_clk_test.opsid);
    switch(bst_clk_test.opsid) {
        case GET_CLK_RATE:
            bst_clk_ptr = of_clk_get(bst_clk_test_dev->of_node, bst_clk_test.index);
            if(! bst_clk_ptr) {
                printk(KERN_ALERT   PRT_HEAD"%s:%d, clk get failed.\n", __func__, __LINE__);
            }
            printk(KERN_ALERT   PRT_HEAD"%s:%d, get clk name:%s.\n", __func__, __LINE__, bst_clk_ptr->core->name);
            rate = clk_get_rate(bst_clk_ptr);
            printk(KERN_ALERT   PRT_HEAD"%s:%d, current rate:%ld.\n", __func__, __LINE__, rate);
            break;
        case RND_CLK_RATE:
            bst_clk_ptr = of_clk_get(bst_clk_test_dev->of_node, bst_clk_test.index);
            if(! bst_clk_ptr) {
                printk(KERN_ALERT   PRT_HEAD"%s:%d, clk get failed.\n", __func__, __LINE__);
            }
            printk(KERN_ALERT   PRT_HEAD"%s:%d, get clk name:%s.\n", __func__, __LINE__, bst_clk_ptr->core->name);
            rate = clk_get_rate(bst_clk_ptr);
            printk(KERN_ALERT   PRT_HEAD"%s:%d, current rate:%ld.\n", __func__, __LINE__, rate);
            printk(KERN_ALERT   PRT_HEAD"%s:%d, user round rate:%ld.\n", __func__, __LINE__, bst_clk_test.rate);
            rate = clk_round_rate(bst_clk_ptr, bst_clk_test.rate);
            printk(KERN_ALERT   PRT_HEAD"%s:%d, CCF round rate:%ld.\n", __func__, __LINE__, rate);
            break;
        case WRT_CLK_RATE:
            bst_clk_ptr = of_clk_get(bst_clk_test_dev->of_node, bst_clk_test.index);
            if(! bst_clk_ptr) {
                printk(KERN_ALERT   PRT_HEAD"%s:%d, clk get failed.\n", __func__, __LINE__);
            }
            printk(KERN_ALERT   PRT_HEAD"%s:%d, get clk name:%s.\n", __func__, __LINE__, bst_clk_ptr->core->name);
            printk(KERN_ALERT   PRT_HEAD"%s:%d, user set rate:%ld.\n", __func__, __LINE__, bst_clk_test.rate);
            clk_set_rate(bst_clk_ptr, bst_clk_test.rate);
            break;
        case ENABLE_CLK:
            bst_clk_ptr = of_clk_get(bst_clk_test_dev->of_node, bst_clk_test.index);
            if(! bst_clk_ptr) {
                printk(KERN_ALERT   PRT_HEAD"%s:%d, clk get failed.\n", __func__, __LINE__);
            }
            printk(KERN_ALERT   PRT_HEAD"%s:%d, get clk name:%s.\n", __func__, __LINE__, bst_clk_ptr->core->name);
            clk_prepare_enable(bst_clk_ptr);
            break;
        case DISABLE_CLK:
            bst_clk_ptr = of_clk_get(bst_clk_test_dev->of_node, bst_clk_test.index);
            if(! bst_clk_ptr) {
                printk(KERN_ALERT   PRT_HEAD"%s:%d, clk get failed.\n", __func__, __LINE__);
            }
            printk(KERN_ALERT   PRT_HEAD"%s:%d, get clk name:%s.\n", __func__, __LINE__, bst_clk_ptr->core->name);
            clk_disable_unprepare(bst_clk_ptr);
            break;
        default:
            break;
    }
	
	return sizeof(clktest_struct);
}


/* 2.file_operations */
static struct file_operations clktest_fops = {
	.owner = THIS_MODULE,
	.open  = clktest_open,
	.read  = clktest_read,
};

#define CLKTEST_CNT   1

static struct cdev clktest_cdev;
static struct class *cls;

static int clktest_probe(struct platform_device *pdev)
{
	dev_t devid;

    bst_clk_test_dev = &pdev->dev;    
	
	if (bst_clk_test_dev_major) {
		devid = MKDEV(bst_clk_test_dev_major, 0);
		register_chrdev_region(devid, CLKTEST_CNT, "bsta1000b_clktest");
	} else {
		alloc_chrdev_region(&devid, 0, CLKTEST_CNT, "bsta1000b_clktest");
		bst_clk_test_dev_major = MAJOR(devid);                     
	}
	
	cdev_init(&clktest_cdev, &clktest_fops);
	cdev_add(&clktest_cdev, devid, CLKTEST_CNT);

	cls = class_create(THIS_MODULE, "bsta1000b_clktest");
    device_create(cls, NULL, MKDEV(bst_clk_test_dev_major, 0), NULL, "bsta1000b_clktest"); /* /dev/bsta1000b_clktest */

    printk(KERN_ALERT   PRT_HEAD"This is %s:line %d.\n", __func__, __LINE__);

    return 0;
}

static int clktest_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver bsta1000b_clktest_driver = {
	.probe		= clktest_probe,
	.remove		= clktest_remove,
	.driver		= {
		.name	= "bsta1000b_clktest",
		.of_match_table	= of_match_ptr(of_clktest_match),
	},
};

module_platform_driver(bsta1000b_clktest_driver);

MODULE_LICENSE("GPL v2");

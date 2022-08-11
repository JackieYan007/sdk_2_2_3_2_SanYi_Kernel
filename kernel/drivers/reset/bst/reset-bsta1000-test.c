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


#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/module.h>
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
#include <dt-bindings/reset/bst-a1000-resets.h>

#define PRT_HEAD        "[BST][RST]: "

typedef struct _rsttest_struct_{
    int opsid;      //define what operation should do
#define RST_RELEASE     (1)
#define RST_GET_STS     (2)
#define RST_RESET       (3)

    int index;      //get rst by index or rst_name[]
    char rst_name[50];
}rsttest_struct;


static rsttest_struct bst_rsttest;
static struct reset_control * bst_rstptr;
static struct device *bst_rst_test_dev;


static const struct of_device_id of_rsttest_match[] = {
	{
		.compatible = "bst,a1000-rstc-test"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_rsttest_match);

static int bst_rst_test_major = 0; //kernel auto alloc

static int rsttest_open(struct inode *inode, struct file *file)
{
	pr_debug( PRT_HEAD"rsttest_open\n");
	return 0;
}

static ssize_t rsttest_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size != sizeof(rsttest_struct)) {
        pr_debug(PRT_HEAD"%s:%d, length error\n", __func__, __LINE__);
		return -EINVAL;
    }

    bst_rstptr = NULL;
    copy_from_user(&bst_rsttest, buf, size);
    pr_debug( PRT_HEAD"%s:%d, opsid:%d( 1(RST_RELEASE), 2(RST_GET_STS) )\n", __func__, __LINE__, bst_rsttest.opsid);
    pr_debug( PRT_HEAD"%s:%d, index:%d\n", __func__, __LINE__, bst_rsttest.index);
    switch(bst_rsttest.opsid) {
        case RST_RELEASE:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                pr_debug( PRT_HEAD"%s:%d, rst get failed.\n", __func__, __LINE__);
                return 0;
            }
            reset_control_deassert(bst_rstptr);
            reset_control_put(bst_rstptr);
            break;
        case RST_GET_STS:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                pr_debug( PRT_HEAD"%s:%d, rst get failed.\n", __func__, __LINE__);
                if((struct reset_control *)(-EBUSY) == bst_rstptr) {
                    pr_debug( PRT_HEAD"%s:%d, not allow to get a shared reset control handler.\n", __func__, __LINE__);
                }
                return 0;
            }
            pr_debug( PRT_HEAD"%s:%d, rst status: %d(1:reset, 0:release).\n", __func__, __LINE__,
                reset_control_status(bst_rstptr));
            reset_control_put(bst_rstptr);
            break;
        case RST_RESET:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                pr_debug( PRT_HEAD"%s:%d, rst get failed.\n", __func__, __LINE__);
                if((struct reset_control *)(-EBUSY) == bst_rstptr) {
                    pr_debug( PRT_HEAD"%s:%d, not allow to get a shared reset control handler.\n", __func__, __LINE__);
                }
                return 0;
            }
            pr_debug( PRT_HEAD"%s:%d, before reset module(index %d).\n", __func__, __LINE__, bst_rsttest.index);
            reset_control_reset(bst_rstptr);
            pr_debug( PRT_HEAD"%s:%d, after reset module(index %d).\n", __func__, __LINE__, bst_rsttest.index);
            reset_control_put(bst_rstptr);
            break;
        default:
            break;
    }
	
	return sizeof(rsttest_struct);
}

static struct file_operations rsttest_fops = {
	.owner = THIS_MODULE,
	.open  = rsttest_open,
	.read  = rsttest_read,
};

#define rstTEST_CNT   1

static struct cdev rsttest_cdev;
static struct class *cls;

static int rsttest_probe(struct platform_device *pdev)
{
	dev_t devid;

    pr_debug( PRT_HEAD"This is %s:%d.\n", __func__, __LINE__);

    bst_rst_test_dev = &pdev->dev;    
	
	if (bst_rst_test_major) {
		devid = MKDEV(bst_rst_test_major, 0);
		register_chrdev_region(devid, 1, "bsta1000_rsttest");  /* (major,0) ??? rsttest_fops, (major, 1~255)???????hello_fops */
	} else {
		alloc_chrdev_region(&devid, 0, 1, "bsta1000_rsttest");
		bst_rst_test_major = MAJOR(devid);                     
	}
	
	cdev_init(&rsttest_cdev, &rsttest_fops);
	cdev_add(&rsttest_cdev, devid, 1);

	cls = class_create(THIS_MODULE, "bsta1000_rsttest");
    device_create(cls, NULL, MKDEV(bst_rst_test_major, 0), NULL, "bsta1000_rsttest"); /* /dev/bst_rsttest */

    pr_debug( PRT_HEAD"This is %s:line %d.\n", __func__, __LINE__);

    return 0;
}

static int rsttest_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver bsta1000_rsttest_driver = {
	.probe		= rsttest_probe,
	.remove		= rsttest_remove,
	.driver		= {
		.name	= "bsta1000_rsttest",
		.of_match_table	= of_match_ptr(of_rsttest_match),
	},
};

module_platform_driver(bsta1000_rsttest_driver);

MODULE_LICENSE("GPL v2");

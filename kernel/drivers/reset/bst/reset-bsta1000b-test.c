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
#include <dt-bindings/reset/bst-a1000b-resets.h>

typedef struct _rsttest_struct_{
    int opsid;      //define what operation should do
#define RST_RELEASE     (1)
#define RST_GET_STS     (2)
#define RST_RESET       (3)

    int index;      //get rst by index or rst_name[]
    char rst_name[50];
}rsttest_struct;

extern void __iomem *a1000b_rst_addr[5];
static rsttest_struct bst_rsttest;
static struct reset_control * bst_rstptr;
static struct device *bst_rst_test_dev;


static const struct of_device_id of_rsttest_match[] = {
	{
		.compatible = "bst,a1000b-rstc-test"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_rsttest_match);

static int bst_rst_test_major = 0; //kernel auto alloc

static int rsttest_open(struct inode *inode, struct file *file)
{
	printk( KERN_CRIT "rsttest_open\n");
	return 0;
}

static ssize_t rsttest_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size != sizeof(rsttest_struct)) {
        printk(KERN_CRIT "%s:%d, length error\n", __func__, __LINE__);
		return -EINVAL;
    }

    bst_rstptr = NULL;
    copy_from_user(&bst_rsttest, buf, size);
    printk( KERN_CRIT "%s:%d, opsid:%d( 1(RST_RELEASE), 2(RST_GET_STS) )\n", __func__, __LINE__, bst_rsttest.opsid);
    printk( KERN_CRIT "%s:%d, index:%d\n", __func__, __LINE__, bst_rsttest.index);
    switch(bst_rsttest.opsid) {
        case RST_RELEASE:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                printk( KERN_CRIT "%s:%d, rst get failed.\n", __func__, __LINE__);
                return 0;
            }
            
            reset_control_deassert(bst_rstptr);
            reset_control_put(bst_rstptr);
            break;
        case RST_GET_STS:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                printk( KERN_CRIT "%s:%d, rst get failed.\n", __func__, __LINE__);
                if((struct reset_control *)(-EBUSY) == bst_rstptr) {
                    printk( KERN_CRIT "%s:%d, not allow to get a shared reset control handler.\n", __func__, __LINE__);
                }
                return 0;
            }
            
            printk( KERN_CRIT "%s:%d, rst status: %d(1:reset, 0:release).\n", __func__, __LINE__,
                reset_control_status(bst_rstptr));
            reset_control_put(bst_rstptr);
            break;
        case RST_RESET:
            bst_rstptr = devm_reset_control_get_by_index(bst_rst_test_dev, bst_rsttest.index);
            if (IS_ERR(bst_rstptr)) {
                printk( KERN_CRIT "%s:%d, rst get failed.\n", __func__, __LINE__);
                if((struct reset_control *)(-EBUSY) == bst_rstptr) {
                    printk( KERN_CRIT "%s:%d, not allow to get a shared reset control handler.\n", __func__, __LINE__);
                }
                return 0;
            }
            
            printk( KERN_CRIT "%s:%d, before reset module(index %d).\n", __func__, __LINE__, bst_rsttest.index);
            reset_control_reset(bst_rstptr);
            printk( KERN_CRIT "%s:%d, after reset module(index %d).\n", __func__, __LINE__, bst_rsttest.index);
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
static void __iomem *addr;
static int rsttest_probe(struct platform_device *pdev)
{
	dev_t devid;

    bst_rst_test_dev = &pdev->dev;    
	
	if (bst_rst_test_major) {
		devid = MKDEV(bst_rst_test_major, 0);
		register_chrdev_region(devid, 1, "bsta1000b_rsttest");  /* (major,0) ��Ӧ rsttest_fops, (major, 1~255)������Ӧhello_fops */
	} else {
		alloc_chrdev_region(&devid, 0, 1, "bsta1000b_rsttest");
		bst_rst_test_major = MAJOR(devid);                     
	}
	
	cdev_init(&rsttest_cdev, &rsttest_fops);
	cdev_add(&rsttest_cdev, devid, 1);

	cls = class_create(THIS_MODULE, "bsta1000b_rsttest");
    device_create(cls, NULL, MKDEV(bst_rst_test_major, 0), NULL, "bsta1000b_rsttest"); /* /dev/bst_rsttest */

    addr=ioremap(0x33002004,4);/* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0 */
    writel(0xffffffff,addr);

	addr=ioremap(0x330020E8,4);/* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST1 */
    writel(0xffffffff,addr);

	addr=ioremap(0x70035008,4);/* SEC_SAFE_SYS_CTRL_R_SEC_SAFE_RESET_CTRL */
    writel(0xffffffff,addr);

	addr=ioremap(0x20020000,4);/* LB_LSP0_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
    writel(0xffffffff,addr);

	addr=ioremap(0x20021000,4);/* LB_LSP1_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
    writel(0xffffffff,addr);

    writel(0xc,a1000b_rst_addr[0]);
    writel(0x40,a1000b_rst_addr[1]);
    writel(0x3fffffff,a1000b_rst_addr[2]);
    writel(0x1fff,a1000b_rst_addr[3]);
    writel(0x1ffff,a1000b_rst_addr[4]);

    return 0;
}

static int rsttest_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver bsta1000b_rsttest_driver = {
	.probe		= rsttest_probe,
	.remove		= rsttest_remove,
	.driver		= {
		.name	= "bsta1000b_rsttest",
		.of_match_table	= of_match_ptr(of_rsttest_match),
	},
};

module_platform_driver(bsta1000b_rsttest_driver);

MODULE_LICENSE("GPL v2");

/*
* DUMP_REGISTER driver for BST DUMP_REGISTER
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2021: all right reserved. 
*/

/*
* ChangeLog:
* Jul 2021: v1: create by fei.jing@bst.ai
*
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/printk.h>
#include <linux/mtd/mtd.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/moduleparam.h> 
#include <linux/miscdevice.h>
#include "bst_dump_reg.h"

#define BST_DEBUG             		(1)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#define DUMP_REG_CANFD           (_IO(0XEF, 0x1))
#define DUMP_REG_QSPI            (_IO(0XEF, 0x2))
#define DUMP_REG_GMAC            (_IO(0XEF, 0x3))
#define DUMP_REG_GICV2           (_IO(0XEF, 0x4))
#define DUMP_REG_PWM             (_IO(0XEF, 0x5))
#define DUMP_REG_WDT             (_IO(0XEF, 0x6))
#define DUMP_REG_USB2_DEVICE     (_IO(0XEF, 0x7))
#define DUMP_REG_USB3_HOST       (_IO(0XEF, 0x8))
#define DUMP_REG_USB3_DEVICE     (_IO(0XEF, 0x9))

#define DUMP_REG_CNT             1        
#define DUMP_REG_NAME            "dump_reg"	 

/* dump_reg设备结构体 */
struct dump_reg_dev{
	dev_t devid;			
	struct cdev cdev;		
	struct class *class;	
	struct device *device;	
	int major;				
	int minor;				
};

struct dump_reg_dev dump_reg = {    
	.major = 0,
	.minor = 0,
};

static int dump_reg_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t dump_reg_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

static ssize_t dump_reg_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

static long dump_reg_unlocked_ioctl (struct file* files, unsigned int cmd, unsigned long arg)
{
	switch(cmd){
	case DUMP_REG_CANFD:
			canfd_dump_register();
			break;
	case DUMP_REG_QSPI:
			qspi_dump_register();
			break;
	case DUMP_REG_GMAC:
			gmac_dump_register();
			break;
	case DUMP_REG_GICV2:
			gic_dump_register();
			break;
	case DUMP_REG_PWM:
			pwm_dump_register();
			break;
	case DUMP_REG_WDT:
			wdt_dump_register();
			break;
	case DUMP_REG_USB2_DEVICE:
			usb2_device_dump_register();
			break;
	case DUMP_REG_USB3_HOST:
			usb3_host_dump_register();
			break;
	case DUMP_REG_USB3_DEVICE:
			usb3_device_dump_register();
			break;
	default:
			break;
	}
	return 0;
}

static int dump_reg_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* 设备操作函数 */
static struct file_operations dump_reg_fops = {
	.owner   = THIS_MODULE,
	.open    = dump_reg_open,
	.read    = dump_reg_read,
	.write   = dump_reg_write,
	.release = dump_reg_release,
	.unlocked_ioctl = dump_reg_unlocked_ioctl,
};

static int __init dump_reg_init(void)
{
	if (!dump_reg.major) {	
		alloc_chrdev_region(&dump_reg.devid, 0, DUMP_REG_CNT, DUMP_REG_NAME);	
		dump_reg.major = MAJOR(dump_reg.devid);	
		dump_reg.minor = MINOR(dump_reg.devid);	
	} else {						
		dump_reg.devid = MKDEV(dump_reg.major, 0);
		register_chrdev_region(dump_reg.devid, DUMP_REG_CNT, DUMP_REG_NAME);
	}
	
	dump_reg.cdev.owner = THIS_MODULE;
	cdev_init(&dump_reg.cdev, &dump_reg_fops);
	
	/* 添加cdev */
	cdev_add(&dump_reg.cdev, dump_reg.devid, DUMP_REG_CNT);

	dump_reg.class = class_create(THIS_MODULE, DUMP_REG_NAME);
	if (IS_ERR(dump_reg.class)) 
		return PTR_ERR(dump_reg.class);

	/*创建设备 */
	dump_reg.device = device_create(dump_reg.class, NULL, dump_reg.devid, NULL, DUMP_REG_NAME);
	if (IS_ERR(dump_reg.device)) 
		return PTR_ERR(dump_reg.device);
	
	return 0;
}

static void __exit dump_reg_exit(void)
{
	cdev_del(&dump_reg.cdev);    
	unregister_chrdev_region(dump_reg.devid, DUMP_REG_CNT);  
	device_destroy(dump_reg.class, dump_reg.devid);
	class_destroy(dump_reg.class);
}

module_init(dump_reg_init);
module_exit(dump_reg_exit);
MODULE_DESCRIPTION("BST DUMP_REGISTER driver");
MODULE_AUTHOR("Jing Fei <fei.jing@bst.ai>");
MODULE_LICENSE("GPL");

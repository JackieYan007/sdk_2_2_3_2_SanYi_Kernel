/*
 * DDR_PRESS_TEST driver for BST
 * Copyright (C) 2010-2011 Picochip Ltd., Jamie Iles
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
 * ChangeLog:
 * Jan 2020: v1: Create ddr_press_test driver for the first time
 * Dec 2021: v2: Cma_alloc change to vmalloc
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/page.h>
#include <linux/memblock.h>
#include <linux/err.h>
#include <linux/sizes.h>

extern void *vmalloc(unsigned long size);
extern void vfree(const void *addr);

/******************************************************************************/

#define BST_DEBUG             		(0)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif
/******************************************************************************/

#define DEVICE_NAME			"ddr_press_test"

#define DDR_PRESS_IOCTL_TYPE    	'x'
#define DDR_PRESS_IOCTL_MEMSET  	_IO(DDR_PRESS_IOCTL_TYPE, 1)
#define DDR_PRESS_IOCTL_MEMCPY  	_IO(DDR_PRESS_IOCTL_TYPE, 2)
#define DDR_PRESS_CMD_MAX_NR		(2)

#define DDR_TEST_PAGE_SIZE		(4*1024)

static int ddr_press_test_memset(unsigned long lenth)
{
	u64 *p_var_addr = NULL;
	volatile unsigned int i = 1024;

	p_var_addr = (u64 *)vmalloc(lenth);
	if (!p_var_addr)
		return -ENOMEM;

	pr_debug("vmalloc p_var_addr 0x%llx\n", (u64)p_var_addr);

	while(i--)
		memset(p_var_addr, 1, lenth);
	
	vfree(p_var_addr);

	return 0;
}

static int ddr_press_test_memcpy(unsigned long lenth)
{
	u64 *p_var_addr = NULL;
	u64 *p_dest_addr = NULL;
	volatile unsigned int i = 1024;

	p_var_addr = (u64 *)vmalloc(lenth);
	if (!p_var_addr)
		return -ENOMEM;

	p_dest_addr = (u64 *)((u64)p_var_addr + lenth / 2);

	pr_debug("vmalloc p_var_addr 0x%llx	p_dest_addr 0x%llx\n", (u64)p_var_addr, (u64)p_dest_addr);

	while(i--)
		memcpy(p_dest_addr, p_var_addr, lenth / 2);
	
	vfree(p_var_addr);

	return 0;
}

static inline int bst_ddr_press_test_open(struct inode *inode, struct file *file)
{
	pr_debug( "ddr_press_test open:\n");
	
	return 0;
}


static inline int bst_ddr_press_test_close(struct inode *inode, struct file *file)
{
	pr_debug( "ddr_press_test close:\n");

	return 0;
}

static long bst_ddr_press_test_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	
	/* TYPE check */
	if ((_IOC_TYPE(cmd) != DDR_PRESS_IOCTL_TYPE) || (_IOC_NR(cmd) > DDR_PRESS_CMD_MAX_NR))
		return -EINVAL;

	switch (cmd) {
		case DDR_PRESS_IOCTL_MEMSET:
			/* Must be smaller than 64M */
			if ((arg<0) || (arg>0x4000*DDR_TEST_PAGE_SIZE))
				return -EINVAL;
			
			ddr_press_test_memset(arg);
			break;
			
		case DDR_PRESS_IOCTL_MEMCPY:
			/* Must be smaller than 32M */
			if ((arg<0) || (arg>0x2000*DDR_TEST_PAGE_SIZE))
				return -EINVAL;

			ddr_press_test_memcpy(arg);
			break;
			
		default:
			pr_debug( "DDR_PRESS_TEST: command is invalid\n");
			ret = -EINVAL;
			break;
	}

	return ret;
}


/*******************************************************************************
 * BST MISC DDR_PRESS_TEST Initialization
 ******************************************************************************/
static struct file_operations dev_fops = {
    .owner   =   THIS_MODULE,
    .open    =   bst_ddr_press_test_open,
    .release =   bst_ddr_press_test_close, 
    .unlocked_ioctl   =   bst_ddr_press_test_ioctl,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;

	ret = misc_register(&misc);
	pr_debug( "bst "DEVICE_NAME" initialized\n");
	
	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
	pr_debug( "bst "DEVICE_NAME" exit\n");
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("xy");
MODULE_DESCRIPTION("BST ddr pressure test Driver");

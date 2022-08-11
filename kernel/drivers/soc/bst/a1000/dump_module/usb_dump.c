/*
* DUMP_USB driver for BST DUMP_USB
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/reset.h>
#include "usb_dump.h"

int usb2_device_dump_register(void)
{
	u32  k=0;
	void __iomem *usb2_addr;
	u32 *addr;
	u64 val_1, val_2, val_3, val_4;

	usb2_addr = ioremap(USB2_BASE_ADDR, SIZE_4K);
	if(!usb2_addr){
		printk(KERN_ERR "ioremap failed in fun: usb2_device_dump_register! \n");
		return -1;
	}
	printk(KERN_DEBUG  "usb2_device_dump:\n");
	for(k = 0; k < 0XFF; k+=16) {
		addr = usb2_addr+k;
		val_1 = readl(&addr[0]);
		val_2 = readl(&addr[1]);
		val_3 = readl(&addr[2]);
		val_4 = readl(&addr[3]);
		printk(KERN_DEBUG "0x%08x:	0x%08x	0x%08x	0x%08x	0x%08x\r\n", k, val_1, val_2, val_3, val_4);
	}
	iounmap(USB2_BASE_ADDR);
	return 0;
}
EXPORT_SYMBOL(usb2_device_dump_register);

int usb3_host_dump_register(void)
{
	u32  k=0;
	void __iomem *usb3_host_addr;
	u32 *addr;
	u64 val_1, val_2, val_3, val_4;

	usb3_host_addr = ioremap(USB3_BASE_ADDR, SIZE_4K);
	if(!usb3_host_addr){
		printk(KERN_ERR "ioremap failed in fun: usb3_host_dump_register!\n");
		return -1;
	}
	printk(KERN_DEBUG  "usb3_host_dump:\n");
	for(k = 0; k < 0xFF; k+=16) {
		addr = usb3_host_addr+k;
		val_1 = readl(&addr[0]);
		val_2 = readl(&addr[1]);
		val_3 = readl(&addr[2]);
		val_4 = readl(&addr[3]);
		printk(KERN_DEBUG "0x%08x:	0x%08x	0x%08x	0x%08x	0x%08x\r\n", k, val_1, val_2, val_3, val_4);
	}
	iounmap(USB3_BASE_ADDR);
	return 0;
}
EXPORT_SYMBOL(usb3_host_dump_register);

int  usb3_device_dump_register(void)
{
	u32  k=0;
	void __iomem *usb3_device_addr;
	u32 *addr;
	u64 val_1, val_2, val_3, val_4;

	usb3_device_addr = ioremap(USB3_BASE_ADDR, SIZE_4K);
	if(! usb3_device_addr){
		printk(KERN_ERR "ioremap failed in fun: usb3_device_dump_register!\n");
		return -1;
	}
	printk(KERN_DEBUG  "usb3_device_dump:\n");
	for(k = 0; k < 0xFF; k+=16) {
		addr = usb3_device_addr+k;
		val_1 = readl(&addr[0]);
		val_2 = readl(&addr[1]);
		val_3 = readl(&addr[2]);
		val_4 = readl(&addr[3]);
		printk(KERN_DEBUG "0x%08x:	0x%08x	0x%08x	0x%08x	0x%08x\r\n", k, val_1, val_2, val_3, val_4);
	}
	iounmap(USB3_BASE_ADDR);	
	return 0;
}
EXPORT_SYMBOL(usb3_device_dump_register);

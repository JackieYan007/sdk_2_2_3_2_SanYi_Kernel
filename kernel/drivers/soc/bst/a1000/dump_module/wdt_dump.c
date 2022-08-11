/*
* DUMP_WDT driver for BST DUMP_WDT
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
#include "wdt_dump.h"

int wdt_dump_register(void)
{
	unsigned char k=0;
	void __iomem* wdt_addr;
	
	printk(KERN_DEBUG "watchdog_dump:\n");
	for (k = 0; k < WDT_CNT; k++) {
		wdt_addr = ioremap(WDT_LSP_BASE(k), SIZE_256B);
		if(! wdt_addr){
			printk(KERN_ERR "ioremap failed in fun: wdt_dump_register,k = %d.\n" ,k);
			return -1;
		}
		printk(KERN_DEBUG "\n");
		printk(KERN_DEBUG "/*** WDT[%d] ***/\n", k);
		printk(KERN_DEBUG "WDT[%d]_CR             = 0x%08x \n",      k, readl(wdt_addr + WDT_CR));
		printk(KERN_DEBUG "WDT[%d]_TORR           = 0x%08x \n",  	 k, readl(wdt_addr + WDT_TORR));
		printk(KERN_DEBUG "WDT[%d]_CCVR           = 0x%08x \n",  	 k, readl(wdt_addr + WDT_CCVR));
		printk(KERN_DEBUG "WDT[%d]_STAT           = 0x%08x \n",  	 k, readl(wdt_addr + WDT_STAT));  //Interrupt Status Register   bit0: 0:inactive  1:active
		printk(KERN_DEBUG "WDT[%d]_COMP_PARAM_5   = x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_PARAM_5));
		printk(KERN_DEBUG "WDT[%d]_COMP_PARAM_4   = 0x%08x \n", 	 k, readl(wdt_addr + WDT_COMP_PARAM_4));
		printk(KERN_DEBUG "WDT[%d]_COMP_PARAM_3   = 0x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_PARAM_3));
		printk(KERN_DEBUG "WDT[%d]_COMP_PARAM_2   = 0x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_PARAM_2));
		printk(KERN_DEBUG "WDT[%d]_COMP_PARAM_1   = 0x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_PARAM_1));
		printk(KERN_DEBUG "WDT[%d]_COMP_VERSI     = 0x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_VERSI));
		printk(KERN_DEBUG "WDT[%d]_COMP_TYPE      = 0x%08x \n",  	 k, readl(wdt_addr + WDT_COMP_TYPE));
		iounmap(WDT_LSP_BASE(k));
	}
	return 0;
}
EXPORT_SYMBOL(wdt_dump_register);

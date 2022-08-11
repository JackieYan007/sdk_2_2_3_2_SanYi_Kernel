/*
* DUMP_PWM driver for BST DUMP_PWM
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
#include "pwm_dump.h"

int pwm_dump_register(void)
{
	u8 i=0;
	void __iomem *pwm_addr;
	
	printk(KERN_DEBUG "pwm_dump:\n");
	
	for (i = 0; i < TIMER_CNT; i+=1) {
		pwm_addr = ioremap(TIMER_BASE(i), SIZE_256B);
		if(! pwm_addr){
			printk(KERN_ERR "ioremap failed in fun: pwm_dump_register! \n");
			return -1;
		}
		printk(KERN_DEBUG "\n");
		printk(KERN_DEBUG "/*** TIMER[%d] ***/\n", i);
		
		printk(KERN_DEBUG "TIMERNLOADCOUNT(%d)     = 0x%08x \n",  i, readl(pwm_addr + TIMERNLOADCOUNT(i)));
		printk(KERN_DEBUG "TIMERNCURRENTVALUE(%d)  = 0x%08x \n",  i, readl(pwm_addr + TIMERNCURRENTVALUE(i)));
		printk(KERN_DEBUG "TIMERNCTROLREG(%d)      = 0x%08x \n",  i, readl(pwm_addr + TIMERNCTROLREG(i)));
		printk(KERN_DEBUG "TIMERNLOADCOUNT2(%d)    = 0x%08x \n",  i, readl(pwm_addr + TIMERNLOADCOUNT2(i)));	
		printk(KERN_DEBUG "TIMER_N_PROT_LEVEL(%d)  = 0x%08x \n",  i, readl(pwm_addr + TIMER_N_PROT_LEVEL(i)));
		iounmap(TIMER_BASE(i));
	}
	return 0;
}

EXPORT_SYMBOL(pwm_dump_register);

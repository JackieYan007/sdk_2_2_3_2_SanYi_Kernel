/*
* DUMP_CANFD driver for BST DUMP_CANFD
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/acpi.h>
#include "canfd_dump.h"

int canfd_dump_register(void)
{
	void __iomem *canfd0_addr;
	canfd0_addr = ioremap(BST_CANFD_0_BASE_ADDR, SIZE_256B);
	if(!canfd0_addr){
		printk(KERN_ERR "ioremap failed in fun canfd_dump_register! \n");
		return -1;
	}
	printk(KERN_DEBUG "canfd0_dump:\n");
	printk(KERN_DEBUG "CAN_GLBCTRL_REG              = 0x%08x\n",  readl(canfd0_addr + CAN_GLBCTRL_REG));
	printk(KERN_DEBUG "CAN_TXERR_CNT_REG            = 0x%08x\n",  readl(canfd0_addr + CAN_TXERR_CNT_REG));
	printk(KERN_DEBUG "CAN_TMCTRL0_REG              = 0x%08x\n",  readl(canfd0_addr + CAN_TMCTRL0_REG));
	printk(KERN_DEBUG "CAN_RXERR_CNT_REG            = 0x%08x\n",  readl(canfd0_addr + CAN_RXERR_CNT_REG));
	printk(KERN_DEBUG "CAN_TMCTRL1_REG              = 0x%08x\n",  readl(canfd0_addr + CAN_TMCTRL1_REG));
	printk(KERN_DEBUG "CAN_REC_CTRLBIT_REG          = 0x%08x\n",  readl(canfd0_addr + CAN_REC_CTRLBIT_REG));
	printk(KERN_DEBUG "CAN_ID_REG                   = 0x%08x\n",  readl(canfd0_addr + CAN_ID_REG));
	printk(KERN_DEBUG "CAN_REC_ID_REG               = 0x%08x\n",  readl(canfd0_addr + CAN_REC_ID_REG));
	printk(KERN_DEBUG "CAN_ID_MASK_REG              = 0x%08x\n",  readl(canfd0_addr + CAN_ID_MASK_REG));
	printk(KERN_DEBUG "CAN_OVERWRITE_JUDGE_REG      = 0x%08x\n",  readl(canfd0_addr + CAN_OVERWRITE_JUDGE_REG));
	printk(KERN_DEBUG "CAN_SEND_ID_REG              = 0x%08x\n",  readl(canfd0_addr + CAN_SEND_ID_REG));
	printk(KERN_DEBUG "CAN_STATUS_MASK_REG          = 0x%08x\n",  readl(canfd0_addr + CAN_STATUS_MASK_REG));
	printk(KERN_DEBUG "CAN_TX_DATA0_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_TX_DATA0_REG));
	printk(KERN_DEBUG "CAN_ARB_LOST_CAPTURE_REG     = 0x%08x\n",  readl(canfd0_addr + CAN_ARB_LOST_CAPTURE_REG));
	printk(KERN_DEBUG "CAN_TX_DATA1_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_TX_DATA1_REG));
	printk(KERN_DEBUG "CAN_STATUS_REG               = 0x%08x\n",  readl(canfd0_addr + CAN_STATUS_REG));
	printk(KERN_DEBUG "CAN_TX_DATA2_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_TX_DATA2_REG));
	printk(KERN_DEBUG "CAN_PARITY_RESIDUAL_CTRL_REG = 0x%08x\n",  readl(canfd0_addr + CAN_PARITY_RESIDUAL_CTRL_REG));
	printk(KERN_DEBUG "CAN_TX_DATA3_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_TX_DATA3_REG));
	printk(KERN_DEBUG "CAN_GLBCTRL1_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_GLBCTRL1_REG));
	printk(KERN_DEBUG "CAN_DMA_CTRL_REG             = 0x%08x\n",  readl(canfd0_addr + CAN_DMA_CTRL_REG));
	iounmap(BST_CANFD_0_BASE_ADDR);
	return 0;
}
EXPORT_SYMBOL(canfd_dump_register);

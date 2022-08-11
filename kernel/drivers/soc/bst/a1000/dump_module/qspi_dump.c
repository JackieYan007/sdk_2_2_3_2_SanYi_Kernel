/*
* DUMP_QSPI driver for BST DUMP_QSPI
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
#include "qspi_dump.h"

int qspi_dump_register(void)
{
	void __iomem *qspi0_addr;
	qspi0_addr = ioremap(BST_QSPI_0_BASE_ADDR, SIZE_256B);
	if(!qspi0_addr){
		printk(KERN_ERR "ioremap failed in fun: qspi_dump_register!\n");
		return -1;
	}
	printk(KERN_DEBUG "qspi0_dump:\n");
	printk(KERN_DEBUG "DW_QSPI0_CTRL0    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_CTRL0));
	printk(KERN_DEBUG "DW_QSPI0_CTRL1    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_CTRL1));
	printk(KERN_DEBUG "DW_QSPI0_SSIENR   = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_SSIENR));
	printk(KERN_DEBUG "DW_QSPI0_SER      = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_SER));
	printk(KERN_DEBUG "DW_QSPI0_BAUDR    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_BAUDR));
	printk(KERN_DEBUG "DW_QSPI0_TXFLTR   = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_TXFLTR));
	printk(KERN_DEBUG "DW_QSPI0_RXFLTR   = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_RXFLTR));
	printk(KERN_DEBUG "DW_QSPI0_TXFLR    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_TXFLR));
	printk(KERN_DEBUG "DW_QSPI0_RXFLR    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_RXFLR));
	printk(KERN_DEBUG "DW_QSPI0_SR       = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_SR));
	printk(KERN_DEBUG "DW_QSPI0_IMR      = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_IMR));
	printk(KERN_DEBUG "DW_QSPI0_ISR      = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_ISR));
	printk(KERN_DEBUG "DW_QSPI0_DMACR    = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_DMACR));
	printk(KERN_DEBUG "DW_QSPI0_DMATDLR  = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_DMATDLR));
	printk(KERN_DEBUG "DW_QSPI0_DMARDLR  = 0x%08x  \n",  readl(qspi0_addr + DW_QSPI_DMARDLR));
	iounmap(BST_QSPI_0_BASE_ADDR);
	return 0;
}


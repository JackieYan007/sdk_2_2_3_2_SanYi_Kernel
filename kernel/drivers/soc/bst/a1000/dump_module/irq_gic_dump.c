/*
* DUMP_GIC driver for BST DUMP_GIC
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
#include <linux/irqdomain.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/cputype.h>
#include <asm/exception.h>
#include <asm/smp_plat.h>
#include <asm/virt.h>
#include "irq_gic_dump.h"

#ifdef CONFIG_ARM64
#include <asm/cpufeature.h>
#endif

int gic_dump_register(void)
{
	void __iomem *gic_dis_addr;
	void __iomem *gic_cpu_addr;
	gic_dis_addr = ioremap(GIC_DISTRIBUTOR_ADDR,   SIZE_4K);
	if(!gic_dis_addr){
		printk(KERN_ERR "gic_dis_addr ioremap failed in fun: gic_dump_register! \n");
		return -1;
	}
	gic_cpu_addr = ioremap(GIC_CPU_INTERCACE_ADDR, SIZE_4K);
	if(!gic_cpu_addr){
		printk(KERN_ERR "gic_cpu_addr ioremap failed in fun: gic_dump_register! \n");
		iounmap(GIC_DISTRIBUTOR_ADDR);
		return -1;
	}
	printk(KERN_DEBUG "GIC Distributor registers:\n");
	printk(KERN_DEBUG "GICD_CTLR_REG   = 0x%08x\n",             readl(gic_dis_addr + GICD_CTLR));
	printk(KERN_DEBUG "GICD_TYPER_REG  = 0x%08x\n",             readl(gic_dis_addr + GICD_TYPER));
	printk(KERN_DEBUG "GICD_IIDR_REG   = 0x%08x\n\n",           readl(gic_dis_addr + GICD_IIDR));

	printk(KERN_DEBUG "GIC CPU Interface registers:\n");
	printk(KERN_DEBUG "GICC_CTLR_REG   = 0x%08x\n",             readl(gic_cpu_addr + GICC_CTLR));
	printk(KERN_DEBUG "GICC_PMR_REG    = 0x%08x\n",             readl(gic_cpu_addr + GICC_PMR));
	printk(KERN_DEBUG "GICC_BPR_REG    = 0x%08x\n",             readl(gic_cpu_addr + GICC_BPR));
	printk(KERN_DEBUG "GICC_IAR_REG    = 0x%08x\n",             readl(gic_cpu_addr + GICC_IAR));
	printk(KERN_DEBUG "GICC_EOIR_REG   = 0x%08x\n",             readl(gic_cpu_addr + GICC_EOIR));
	printk(KERN_DEBUG "GICC_RPR_REG    = 0x%08x\n",             readl(gic_cpu_addr + GICC_RPR));
	printk(KERN_DEBUG "GICC_HPPIR_REG  = 0x%08x\n",             readl(gic_cpu_addr + GICC_HPPIR));
	printk(KERN_DEBUG "GICC_ABPR_REG   = 0x%08x\n",             readl(gic_cpu_addr + GICC_ABPR));
	printk(KERN_DEBUG "GICC_IIDR_REG   = 0x%08x\n",             readl(gic_cpu_addr + GICC_IIDR));
	iounmap(GIC_DISTRIBUTOR_ADDR);
	iounmap(GIC_CPU_INTERCACE_ADDR);
	return 0;
}
EXPORT_SYMBOL(gic_dump_register);

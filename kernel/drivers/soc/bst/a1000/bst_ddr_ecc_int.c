/*
 * DDR_ECC_INT driver for BST
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
 * Jan 2020: v1: Create ddr_ecc_int driver for the first time
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/memblock.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <bst/bst_a1000.h>

#define DDR_LOCAL_INTR_OFFSET					(0x28)
#define DDR_MONITOR_CTRL_OFFSET					(0x2C)
#define DDR_PLL_MONITOR_INTR_OFFSET				(0xBC)

#define DDR_OCPARSTAT0_OFFSET					(0x338)
#define DDR_OCPARSTAT1_OFFSET					(0x33C)
#define DDR_OCPARSTAT2_OFFSET					(0x340)
#define DDR_ECCSTAT_OFFSET						(0x1078)
#define DDR_ECC_CLEAR_OFFSET					(0x107C)

#define A55_SYSNOC_PARITY_INTR_STATE0_OFFSET	(0x90)
#define A55_SYSNOC_PARITY_INTR_STATE1_OFFSET	(0x94)
#define A55_SYSNOC_PARITY_INTR_STATE2_OFFSET	(0x98)
#define A55_CORENOC_PARITY_INTR_STATE0_OFFSET	(0x9C)
#define A55_CORENOC_PARITY_INTR_STATE1_OFFSET	(0x100)
#define A55_CORENOC_PARITY_INTR_STATE2_OFFSET	(0x104)
#define A55_ECU_ERR_STATE_OFFSET				(0x130)

#define AXI0_MONITOR_INT						(0x1)
#define DEVICE_NAME								"ddr_ecc_int"

struct bst_ddr_ecc_int
{
	int ddr0_irq;
	int ddr1_irq;
};

static void __iomem *ddr0_base = NULL;
static void __iomem *ddr1_base = NULL;
static void __iomem *a55_ctrl_base = NULL;

/******************************************************************************/
#define BST_DEBUG             		(1)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif
/******************************************************************************/

static inline int bst_ddr_ecc_int_open(struct inode *inode, struct file *file)
{
	bst_pr("ddr_ecc_int open:\n");
	
	return 0;
}

static inline int bst_ddr_ecc_int_close(struct inode *inode, struct file *file)
{
	bst_pr("ddr_ecc_int close:\n");
	
	return 0;
}

/*******************************************************************************
 * BST MISC DDR_ECC_INT Initialization
 ******************************************************************************/
static struct file_operations dev_fops = {
    .owner   =   THIS_MODULE,
    .open    =   bst_ddr_ecc_int_open,
    .release =   bst_ddr_ecc_int_close, 
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static irqreturn_t bst_ddr_ecc_irq_handle(int irq, void *dev_id)
{
	struct bst_ddr_ecc_int *p_data = (struct bst_ddr_ecc_int *)dev_id;
	void __iomem *ddr_base = NULL;
	unsigned int number = 0;
	static int irq_handled_flag[2] = {0,};
	
	if (irq == p_data->ddr0_irq) {
		ddr_base = ddr0_base;
		number = 0;
	} else if (irq == p_data->ddr1_irq) {
		ddr_base = ddr1_base;
		number = 1;
	} else {
		return IRQ_HANDLED;
	}

	bst_pr("ecc irq %d trigger, ECCSTAT 0x%x, ECCCTL 0x%x\r\n", irq, readl(ddr_base + DDR_ECCSTAT_OFFSET), readl(ddr_base + DDR_ECC_CLEAR_OFFSET));

	/* only send once wrong message */
	if (0 == irq_handled_flag[number]) {		
		if ((readl(ddr_base+DDR_ECCSTAT_OFFSET)>>16) & 0xff) {
			send_safety_usrmsg(0xA00C01 + number, 0x2);	/* ddr_0/1 ecc error && RESTARTSYSTEM */
			irq_handled_flag[number] = 1;
			pr_info("bst ddr_%d ecc error!!!\n", number);
		}
	}
		
	return IRQ_HANDLED;
}

static int ddr_ecc_int_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct bst_ddr_ecc_int *pbst_ecc_int = NULL;

	/* dev_info(dev, "int-bst %s, %d\n", __func__,__LINE__); */
	pbst_ecc_int = devm_kzalloc(dev, sizeof(*pbst_ecc_int), GFP_KERNEL);
	if (!pbst_ecc_int)
		return -ENOMEM;

	pbst_ecc_int->ddr0_irq = platform_get_irq_byname(pdev, "ddr0_ecc_irq");
	if (pbst_ecc_int->ddr0_irq < 0) {
		dev_err(&pdev->dev,
					"ddr0_ecc_irq configuration information not found\n");
		return pbst_ecc_int->ddr0_irq;
	}

	pbst_ecc_int->ddr1_irq = platform_get_irq_byname(pdev, "ddr1_ecc_irq");
	if (pbst_ecc_int->ddr1_irq < 0) {
		dev_err(&pdev->dev,
					"ddr1_ecc_irq configuration information not found\n");
		return pbst_ecc_int->ddr1_irq;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ddr0");
	ddr0_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ddr0_base)) {
		return PTR_ERR(ddr0_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ddr1");
	ddr1_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ddr1_base)) {
		return PTR_ERR(ddr1_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "a55_ctrl");
	a55_ctrl_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(a55_ctrl_base)) {
		return PTR_ERR(a55_ctrl_base);
	}

	/* enable ddr_ecc int, only ecc_uncorrected_err_intr_en, mask ecc_corrected_err_intr_en */
	writel((readl(ddr0_base+DDR_ECC_CLEAR_OFFSET) | (1<<9)) & (~(1<<8)), ddr0_base+DDR_ECC_CLEAR_OFFSET);
	writel((readl(ddr1_base+DDR_ECC_CLEAR_OFFSET) | (1<<9)) & (~(1<<8)), ddr1_base+DDR_ECC_CLEAR_OFFSET);
	
	ret = devm_request_irq(dev, pbst_ecc_int->ddr0_irq, bst_ddr_ecc_irq_handle, 0, "ddr0_ecc", pbst_ecc_int);
	if (ret < 0) {
		dev_warn(dev, "failed to request ECC-ddr0 IRQ\n");
	}

	ret = devm_request_irq(dev, pbst_ecc_int->ddr1_irq, bst_ddr_ecc_irq_handle, 0, "ddr1_ecc", pbst_ecc_int);
	if (ret < 0) {
		dev_warn(dev, "failed to request ECC-ddr1 IRQ\n");
		devm_free_irq(dev, pbst_ecc_int->ddr0_irq, pbst_ecc_int);
	}			

	platform_set_drvdata(pdev, pbst_ecc_int);

	ret = misc_register(&misc);
	if (ret != 0) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Cannot register miscdev\n");	
	}
	
	return ret;
}

static int ddr_ecc_int_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_ddr_ecc_int *p_data = platform_get_drvdata(pdev);

	devm_free_irq(dev, p_data->ddr0_irq, p_data); 
	devm_free_irq(dev, p_data->ddr1_irq, p_data);
	misc_deregister(&misc);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ddr_ecc_of_match[] = {
	{ .compatible = "bst,a1000_ddr_ecc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ddr_ecc_of_match);
#endif

static struct platform_driver ddr_ecc_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(ddr_ecc_of_match),
	},
	.probe = ddr_ecc_int_probe,
	.remove = ddr_ecc_int_remove,
};
module_platform_driver(ddr_ecc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("xy");
MODULE_DESCRIPTION("BST ddr_ecc_int Driver");

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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/delay.h>


#define MAX_NOC_PARAM_NUM      (64)

static int bsta1000_noc_probe(struct platform_device *pdev)
{
	void __iomem *addr;
    u32 args[MAX_NOC_PARAM_NUM*2]; //one pair param means: reg_addr+reg_val
    u32 para_num;
    u32 echo_para;
    int ret, i;

    printk("%s,%d\n", __func__, __LINE__);
    
    ret = fwnode_property_read_u32_array(dev_fwnode(&(pdev->dev)), "noc-arg-num", &para_num, 1);
    if (ret < 0)
		return ret;
    
    if(MAX_NOC_PARAM_NUM < para_num) {
        dev_err(&(pdev->dev), "para_num(%d) exceeds MAX_NOC_PARAMS_NUM(%d).\n", para_num, MAX_NOC_PARAM_NUM);
        return -EINVAL;
    }
    printk("%s,%d,para_num:%d\n", __func__, __LINE__, para_num);

    ret = fwnode_property_read_u32_array(dev_fwnode(&(pdev->dev)), "echo-args", &echo_para, 1);
    if (ret < 0)
		return ret;

    ret = fwnode_property_read_u32_array(dev_fwnode(&(pdev->dev)), "noc-args", args, para_num<<1);
    if (ret < 0)
		return ret;

    for(i = 0; i < para_num; i++) {
        addr = ioremap(args[(i<<1)], 4); //i<<1 (i*2)
        
    	writel(args[(i<<1)+1], addr);
        if(echo_para)
            printk("BST NOC echo: [0x%x] 0x%x\n", args[i<<1], readl(addr));
    	iounmap(addr);
    }

    printk("%s,%d\n", __func__, __LINE__);
    
	return 0;
}

static const struct of_device_id bsta1000_noc_dt_ids[] = {
	{ .compatible = "bst,a1000-noc", },
	{ /* sentinel */ },
};

static struct platform_driver bsta1000_noc_driver = {
	.probe	= bsta1000_noc_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= bsta1000_noc_dt_ids,
	},
};

builtin_platform_driver(bsta1000_noc_driver);

MODULE_DESCRIPTION("BST A1000 noc driver");
MODULE_AUTHOR("Kun Niu <kun.niu@bst.ai>");
MODULE_LICENSE("GPL v2");

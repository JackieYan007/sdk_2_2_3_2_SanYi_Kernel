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
#include <linux/reset-controller.h>
#include <linux/types.h>
#include <dt-bindings/reset/bst-a1000-resets.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/delay.h>

#define RST_HOLD_TIME           (10)

/*
 * total 5 registers contol A1000 top level reset
 * TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
 * TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST1
 * SEC_SAFE_SYS_CTRL_R_SEC_SAFE_RESET_CTRL
 * LB_LSP0_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG
 * LB_LSP1_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG
 */
static void __iomem *rst_addr[5];
static struct reset_controller_dev rst_ctl;
static struct mutex rst_lock;

static int bst_reset(struct reset_controller_dev *rcdev, unsigned long id);
static int bst_deassert(struct reset_controller_dev *rcdev, unsigned long id);
static int bst_status(struct reset_controller_dev *rcdev, unsigned long id);
static int bst_assert(struct reset_controller_dev *rcdev, unsigned long id);

/**
 * struct _bst_reset_map_: reset maps
 *                     change mux output to osc and after PLL stable change back to pll output
 * @addr:		    register addr
 * @mux_oft:	    mux start bit offset in mux_addr
 * @bit_idx:	    reset bit index
 * @flags:          reset flags
 */
typedef struct _bst_reset_map_ {
	void __iomem *addr;
    u32 bit_idx;
    u32 flags;
    
/* reset flags define */    
#define ZERO_ASSERT_ONE_DEASSERT            (1<<0)
#define RESET_LONG_HOLD_TIME                (1<<2)
}bst_reset_map;

/* reset resource id enum */
enum RST_RES_ID {
    TOP_CRM_BLOCK_SW_RST0 = 0,
    TOP_CRM_BLOCK_SW_RST1,
    SEC_SAFE_RESET_CTRL,
    LSP0_RST_CTRL_REG,
    LSP1_RST_CTRL_REG
};

static bst_reset_map bst_rst_maps[] = {
    /* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0 */
    [RST_MIPI0_SW]          = { NULL, 17, ZERO_ASSERT_ONE_DEASSERT },
    [RST_MIPI1_SW]          = { NULL, 16, ZERO_ASSERT_ONE_DEASSERT },
    [RST_MIPI2_SW]          = { NULL, 15, ZERO_ASSERT_ONE_DEASSERT },
    [RST_GMAC0_SW]          = { NULL, 14, ZERO_ASSERT_ONE_DEASSERT },
    [RST_GMAC1_SW]          = { NULL, 13, ZERO_ASSERT_ONE_DEASSERT },
    [RST_SDEMMC0_SW]        = { NULL, 12, ZERO_ASSERT_ONE_DEASSERT },
    [RST_SDEMMC1_SW]        = { NULL, 11, ZERO_ASSERT_ONE_DEASSERT },
    [RST_VSP_SW]            = { NULL, 10, ZERO_ASSERT_ONE_DEASSERT },
    [RST_ISP_SW]            = { NULL,  9, ZERO_ASSERT_ONE_DEASSERT },
    [RST_NET_SW]            = { NULL,  8, ZERO_ASSERT_ONE_DEASSERT },
    [RST_CV_SW]             = { NULL,  7, ZERO_ASSERT_ONE_DEASSERT },
    [RST_CPU_SW]            = { NULL,  6, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LPDDR0_SW]         = { NULL,  5, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LPDDR1_SW]         = { NULL,  4, ZERO_ASSERT_ONE_DEASSERT },

    /* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST1 */
    [RST_GDMA_SW]           = { NULL, 12, ZERO_ASSERT_ONE_DEASSERT },
    [RST_GPU_SW]            = { NULL, 11, ZERO_ASSERT_ONE_DEASSERT },
    [RST_USB2_SW]           = { NULL, 10, ZERO_ASSERT_ONE_DEASSERT },
    [RST_USB3_SW]           = { NULL,  9, ZERO_ASSERT_ONE_DEASSERT },
    [RST_PCIE_SW]           = { NULL,  8, ZERO_ASSERT_ONE_DEASSERT },

    /* SEC_SAFE_SYS_CTRL_R_SEC_SAFE_RESET_CTRL */
    [RST_QSPI1_SW]          = { NULL, 15, ZERO_ASSERT_ONE_DEASSERT },
    [RST_QSPI0_SW]          = { NULL, 16, ZERO_ASSERT_ONE_DEASSERT },

    /* LB_LSP0_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
    [RST_LSP0_UART1_SW]     = { NULL, 12, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C0_SW]           = { NULL,  0, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C1_SW]           = { NULL,  1, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C2_SW]           = { NULL,  2, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2SM_SW]           = { NULL,  3, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP0_GPIO_SW]      = { NULL,  4, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP0_UART0_SW]     = { NULL,  5, ZERO_ASSERT_ONE_DEASSERT }, 
    [RST_SPI0_SW]           = { NULL,  6, ZERO_ASSERT_ONE_DEASSERT },
    [RST_WDT0_SW]           = { NULL,  7, ZERO_ASSERT_ONE_DEASSERT },
    [RST_WDT1_SW]           = { NULL,  8, ZERO_ASSERT_ONE_DEASSERT },
    [RST_CAN_FD0_SW]        = { NULL,  9, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP0_TIMER1_SW]    = { NULL, 10, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP0_TIMER0_SW]    = { NULL, 11, ZERO_ASSERT_ONE_DEASSERT },

    /* LB_LSP1_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
    [RST_LSP1_UART1_SW]     = { NULL, 15, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C5_SW]           = { NULL, 14, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP1_GPIO_SW]      = { NULL,  5, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2SS_SW]           = { NULL,  6, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP1_UART0_SW]     = { NULL,  7, ZERO_ASSERT_ONE_DEASSERT },
    [RST_CAN_FD1_SW]        = { NULL,  8, ZERO_ASSERT_ONE_DEASSERT },
    [RST_SPI1_SW]           = { NULL,  9, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C3_SW]           = { NULL, 10, ZERO_ASSERT_ONE_DEASSERT },
    [RST_I2C4_SW]           = { NULL, 11, ZERO_ASSERT_ONE_DEASSERT },
    [RST_WDT2_SW]           = { NULL,  3, ZERO_ASSERT_ONE_DEASSERT },
    [RST_WDT3_SW]           = { NULL,  4, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP1_TIMER0_SW]    = { NULL,  0, ZERO_ASSERT_ONE_DEASSERT },
    [RST_LSP1_TIMER1_SW]    = { NULL,  1, ZERO_ASSERT_ONE_DEASSERT },
};

static const struct reset_control_ops bst_reset_ops = {
    .reset      = bst_reset,
	.assert		= bst_assert,
	.deassert	= bst_deassert,
	.status		= bst_status,
};

/**
 * bst_reset - reset the index id module
 * @rcdev: reset controller struct pointer
 * @id: reset id
 * return 0 when success and otherwise error number
 */
static int bst_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
    u32 reg;
    
    if(id >= RST_LSP1_TOP_END) {
        return -EINVAL;
    }

    mutex_lock(&rst_lock);

    /* assert */
    reg  = readl(bst_rst_maps[id].addr);
    if(bst_rst_maps[id].flags & ZERO_ASSERT_ONE_DEASSERT) {
        reg &= ~(1<<bst_rst_maps[id].bit_idx);
    } else {
        reg |= (1<<bst_rst_maps[id].bit_idx);
    }
    writel(reg, bst_rst_maps[id].addr);
    
    /* hold some time after assert */
    msleep(RST_HOLD_TIME);
    if(bst_rst_maps[id].flags & RESET_LONG_HOLD_TIME) {
        msleep(RST_HOLD_TIME);
    }

    /* deassert */
    if(bst_rst_maps[id].flags & ZERO_ASSERT_ONE_DEASSERT) {
        reg |= (1<<bst_rst_maps[id].bit_idx);
    } else {
        reg &= ~(1<<bst_rst_maps[id].bit_idx);
    }
    writel(reg, bst_rst_maps[id].addr);

    mutex_unlock(&rst_lock);
    /* hold some time after deassert */
    msleep(RST_HOLD_TIME);
    if(bst_rst_maps[id].flags & RESET_LONG_HOLD_TIME) {
        msleep(RST_HOLD_TIME);
    }
        
    return 0;
}

/**
 * bst_assert - set the module to reset state
 * @rcdev: reset controller struct pointer
 * @id: reset id
 * return 0 when success and otherwise error number
 */
static int bst_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
    u32 reg;
    
    if(id > RST_LSP1_TOP_END) {
        return -EINVAL;
    }

    mutex_lock(&rst_lock);
    
    reg  = readl(bst_rst_maps[id].addr);
    if(bst_rst_maps[id].flags & ZERO_ASSERT_ONE_DEASSERT) {        
        reg &= ~(1<<bst_rst_maps[id].bit_idx);
    } else {
        reg |= (1<<bst_rst_maps[id].bit_idx);
    }
    writel(reg, bst_rst_maps[id].addr);
    
    mutex_unlock(&rst_lock);
    
    return 0;
}

/**
 * bst_deassert - set the module to de-reset state
 * @rcdev: reset controller struct pointer
 * @id: reset id
 * return 0 when success and otherwise error number
 */
static int bst_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
    u32 reg;

    if(id > RST_LSP1_TOP_END) {
        return -EINVAL;
    }

    mutex_lock(&rst_lock);
    
    reg  = readl(bst_rst_maps[id].addr);
    if(bst_rst_maps[id].flags & ZERO_ASSERT_ONE_DEASSERT) {
        reg |= (1<<bst_rst_maps[id].bit_idx);
    } else {
        reg &= ~(1<<bst_rst_maps[id].bit_idx);        
    }
    writel(reg, bst_rst_maps[id].addr);
    
    mutex_unlock(&rst_lock);

    return 0;
}

/**
 * bst_deassert - set the module to de-reset state
 * @rcdev: reset controller struct pointer
 * @id: reset id
 * return 1 when the module is reset state and return 0 when de-reset state
 */
static int bst_status(struct reset_controller_dev *rcdev, unsigned long id)
{
    u32 reg;

    if(id > RST_LSP1_TOP_END) {
        return -EINVAL;
    }

    mutex_lock(&rst_lock);
    reg  = readl(bst_rst_maps[id].addr);
    mutex_unlock(&rst_lock);
    
    if(bst_rst_maps[id].flags & ZERO_ASSERT_ONE_DEASSERT) {
        return !(reg & (1<<bst_rst_maps[id].bit_idx));
    } else {
        return !(!(reg & (1<<bst_rst_maps[id].bit_idx)));
    }
}

static int bsta1000_reset_probe(struct platform_device *pdev)
{
    struct resource *res;
    u32 i;

    for(i = TOP_CRM_BLOCK_SW_RST0; i <= LSP1_RST_CTRL_REG; i++) {
    	res = platform_get_resource(pdev, IORESOURCE_MEM, i);
    	if (!res) {
    		dev_err(&pdev->dev, "missing IO resource\n");
    		return -ENODEV;
    	}

        rst_addr[i] = ioremap(res->start, resource_size(res));
        if (! rst_addr[i]) {
    		dev_err(&pdev->dev, "could not remap register memory\n");
    		return -ENOMEM;
	    }
    }

    for(i = RST_TOP_SW_RST0_START; i <= RST_TOP_SW_RST0_END; i++) {
        bst_rst_maps[i].addr = rst_addr[0];
    }
    
    for(i = RST_TOP_SW_RST1_START; i <= RST_TOP_SW_RST1_END; i++) {
        bst_rst_maps[i].addr = rst_addr[1];
    }

    for(i = RST_SEC_SAFE_START; i <= RST_SEC_SAFE_END; i++) {
        bst_rst_maps[i].addr = rst_addr[2];
    }
    
    for(i = RST_LSP0_TOP_START; i <= RST_LSP0_TOP_END; i++) {
        bst_rst_maps[i].addr = rst_addr[3];
    }
    
    for(i = RST_LSP1_TOP_START; i <= RST_LSP1_TOP_END; i++) {
        bst_rst_maps[i].addr = rst_addr[4];
    }
    
	rst_ctl.owner = THIS_MODULE;
	rst_ctl.nr_resets = ARRAY_SIZE(bst_rst_maps);
	rst_ctl.ops = &bst_reset_ops;
	rst_ctl.of_node = pdev->dev.of_node;

    mutex_init(&rst_lock);
    
	return devm_reset_controller_register(&pdev->dev, &rst_ctl);
}

static const struct of_device_id bsta1000_reset_dt_ids[] = {
	{ .compatible = "bst,a1000-rstc", },
	{ /* sentinel */ },
};

static struct platform_driver bsta1000_reset_driver = {
	.probe	= bsta1000_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= bsta1000_reset_dt_ids,
	},
};

//builtin_platform_driver(bsta1000_reset_driver);
static int __init bsta1000_reset_init(void)
{
	return platform_driver_register(&bsta1000_reset_driver);
}
arch_initcall(bsta1000_reset_init);


MODULE_DESCRIPTION("BST A1000 clock driver");
MODULE_AUTHOR("Kun Niu <kun.niu@bst.ai>");
MODULE_LICENSE("GPL v2");

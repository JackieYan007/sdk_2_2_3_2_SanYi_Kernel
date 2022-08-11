/*
 * Generic DWMAC platform driver
 *
 * Copyright (C) 2007-2011  STMicroelectronics Ltd
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "bstgmac.h"
#include "dwmac_platform.h"

#if defined(CONFIG_ARCH_BSTA1000A)
static int bstgmac_select_phy(int id)
{
    void __iomem *top_crm = NULL;
	void __iomem *pmm_reg = NULL;
	u32 reg_value ;

    /* select PHY */
	top_crm = ioremap(0x33000000, 0x1000);
	if(!top_crm){
        return (-EPROBE_DEFER);
	}

	reg_value = readl(top_crm + 0x54);
	reg_value = (reg_value|(1<<(id))); //bit 0: 0(GMII/MII), 1(RGMII)
	writel(reg_value,top_crm + 0x54);
	iounmap(top_crm);

	/* TRIG_INxx and PPSx pinmux */
	pmm_reg = ioremap(0x33001004, 32);
	if(!pmm_reg)
        return (-EPROBE_DEFER);
	reg_value = readl(pmm_reg);
	if (id == 0) { //gmac0: IN00:bit4 IN01:bit5  pps0:bit8
		reg_value = (reg_value & (~((1<<4)|(1<<8))));
	} else if (id == 1) {//gmac1: IN10:bit6 IN11:bit7  pps1:bit9
		reg_value = (reg_value & (~((1<<6)|(1<<9))));
	}
	writel(reg_value, pmm_reg);
	iounmap(pmm_reg);

    return 0;
}
#endif
#if defined(CONFIG_ARCH_BSTA1000B)
static int bstgmac_select_phy(int id)
{
    void __iomem *top_crm = NULL;
	void __iomem *pmm_reg = NULL;
	u32 reg_value ;

    /* select PHY */
	top_crm = ioremap(0x33000000, 0x4000);
	if(!top_crm){
        return (-EPROBE_DEFER);
	}

	reg_value = readl(top_crm + 0x54);
	reg_value = (reg_value|(1<<(id))); //bit 0: 0(GMII/MII), 1(RGMII)
	writel(reg_value,top_crm + 0x54);	
	iounmap(top_crm);

	/* TRIG_INxx and PPSx pinmux */
	pmm_reg = ioremap(0x33001008, 32);
	if(!pmm_reg)
        return (-EPROBE_DEFER);
	reg_value = readl(pmm_reg);
	if (id == 0) { 
		//gmac0: IN00:bit11 IN01:bit12  pps0:bit15
		reg_value = (reg_value & (~((1<<11)|(1<<15))));
	} else if (id == 1) {
		//gmac1: IN10:bit13 IN11:bit14  pps1:bit16
		reg_value = (reg_value & (~((1<<13)|(1<<16))));
	}
	writel(reg_value, pmm_reg);
	iounmap(pmm_reg);

    return 0;
}

#endif



static int bstgmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct bstgmac_resources bstgmac_res;
	int ret;

	ret = bstgmac_get_platform_resources(pdev, &bstgmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = bstgmac_probe_config_dt(pdev, &bstgmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}
	
    ret = bstgmac_select_phy(plat_dat->bus_id);
    if (ret < 0) {
        dev_err(&pdev->dev, "select phy failed\n");
        goto err_remove_config_dt;
    }
	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

	ret = bstgmac_dvr_probe(pdev, plat_dat, &bstgmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		bstgmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id bst_gmac_match[] = {
	{ .compatible = "bst,dw-eqos-eth"},
	{ }
};
MODULE_DEVICE_TABLE(of, bst_gmac_match);

static struct platform_driver dwmac_generic_driver = {
	.probe  = bstgmac_probe,
	.remove = bstgmac_pltfr_remove,
	.driver = {
		.name           = BSTGMAC_RESOURCE_NAME,
		.pm		= &bstgmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(bst_gmac_match),
	},
};
module_platform_driver(dwmac_generic_driver);

MODULE_DESCRIPTION("Bst Gmac driver");
MODULE_LICENSE("GPL v2");

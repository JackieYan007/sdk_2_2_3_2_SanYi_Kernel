#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/bst_boardconfig.h>
#include "pcie_vnet_plat.h"

static int bst_pcievnet_probe(struct platform_device *pdev)
{
	struct pcie_vnet_plat *plat_dat;
	int ret;

    plat_dat = bst_pcievnet_probe_config_dt(pdev);
    if (IS_ERR(plat_dat)) {
        dev_err(&pdev->dev, "dt configuration failed\n");
        return PTR_ERR(plat_dat);
    }
	
	ret = bst_pcievnet_dvr_probe(pdev, plat_dat);
	if (!ret) {
		bst_init_gpio4_5();
	}
	return ret;
}

static const struct of_device_id pcie_vnet_match[] = {
	{ .compatible = "bst,pcie-vnet"},
	{ }
};
MODULE_DEVICE_TABLE(of, pcie_vnet_match);

static struct platform_driver pcie_vnet_driver = {
	.probe  = bst_pcievnet_probe,
	.remove = bst_pcievnet_pltfr_remove,
	.driver = {
		.name           = BST_PCIEVNET_NAME,
		.of_match_table = of_match_ptr(pcie_vnet_match),
	},
};
module_platform_driver(pcie_vnet_driver);

MODULE_DESCRIPTION("Bst pcievnet driver");
MODULE_LICENSE("GPL v2");
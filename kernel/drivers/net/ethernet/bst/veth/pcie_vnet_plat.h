#ifndef __BST_PCIEVNET_PLATFORM_H__
#define __BST_PCIEVNET_PLATFORM_H__

#include <linux/of.h>
#include "pcievnet.h"

int bst_pcievnet_dvr_remove(struct device *dev);
int bst_pcievnet_dvr_probe(struct platform_device *pdev, struct pcie_vnet_plat *plat_dat);
struct pcie_vnet_plat *bst_pcievnet_probe_config_dt(struct platform_device *pdev);
int bst_pcievnet_pltfr_remove(struct platform_device *pdev);

int of_address_to_resource(struct device_node *node, int index,
			   struct resource *r);
#endif /* __BST_PCIEVNET_PLATFORM_H__ */

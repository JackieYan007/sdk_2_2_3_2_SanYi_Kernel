#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>

#include "pcievnet.h"
#include "pcie_vnet_common.h"
#include "pcie_vnet_plat.h"

struct pcie_vnet_plat *
bst_pcievnet_probe_config_dt(struct platform_device *pdev) 
{
	struct device_node *np = pdev->dev.of_node;
	struct pcie_vnet_plat *plat;
    struct device_node *mem;
	struct resource reg;
    int ret;

	plat = devm_kzalloc(&pdev->dev, sizeof(struct pcie_vnet_plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

    of_property_read_u32(np, "vnet-id", &plat->id);

	/* Get max speed of operation from device tree */
	if (of_property_read_u32(np, "dma-chan-num", &plat->dma_chan_num))
		plat->dma_chan_num = 2;
    
    of_property_read_u32(np, "rx_queues", &plat->rx_queues_to_use);
    of_property_read_u32(np, "tx_queues", &plat->tx_queues_to_use);
    if ((plat->rx_queues_to_use > plat->dma_chan_num) || (plat->tx_queues_to_use > plat->dma_chan_num)) {
        printk("Queue cannot be greater than dma chan num\n");
        return ERR_PTR(-ENODEV);
    }
    of_property_read_u32(np, "extend-op", &plat->board_type);

	of_property_read_u32(np, "tx-fifo-depth", &plat->tx_fifo_size);

	of_property_read_u32(np, "rx-fifo-depth", &plat->rx_fifo_size);

	/* Set the maxmtu to a default of JUMBO_LEN in case the
	 * parameter is not present in the device tree.
	 */
	plat->maxmtu = JUMBO_LEN;

    mem = of_parse_phandle(np, "memory-region", 0);
	if (!mem) {
		printk("missing \"memory-region\" property\n");
		return ERR_PTR(-ENODEV);
	}

	ret = of_address_to_resource(mem, 0, &reg);
	if (ret < 0) {
		printk("missing \"reg\" property\n");
		return ERR_PTR(-ENODEV);
	}

	plat->ctrl_mem_base = reg.start;
    plat->ctrl_mem_size = resource_size(&reg);

	return plat;
}
EXPORT_SYMBOL(bst_pcievnet_probe_config_dt);

/**
 * bstgmac_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
int bst_pcievnet_pltfr_remove(struct platform_device *pdev)
{
	int ret = bst_pcievnet_dvr_remove(&pdev->dev);

	return ret;
}
EXPORT_SYMBOL_GPL(bst_pcievnet_pltfr_remove);

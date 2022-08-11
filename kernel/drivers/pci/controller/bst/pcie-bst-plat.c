// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Synopsys DesignWare Core
 *
 * Copyright (C) 2015-2016 Synopsys, Inc. (www.synopsys.com)
 *
 * Authors: Joao Pinto <Joao.Pinto@synopsys.com>
 */
#define pr_fmt(fmt) KBUILD_MODNAME" " fmt
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/kthread.h>
#include <linux/of_gpio.h>
#include <linux/picp.h>
#include <linux/module.h>

#include "pcie-bst.h"
#include "pcie-bst-phy.h"

struct dw_plat_pcie {
	struct dw_pcie			*pci;
	struct regmap			*regmap;
	enum dw_pcie_device_mode	mode;
};

struct dw_plat_pcie_of_data {
	enum dw_pcie_device_mode	mode;
};

static struct dw_pcie *dw_ep;
static struct dw_pcie *dw_rc;
static u32 ltssm_count;
static const struct of_device_id dw_plat_pcie_of_match[];
struct task_struct *task;

struct dw_pcie *get_dw_pcie(int mode)
{
	if (mode == DW_PCIE_EP_TYPE)
		return dw_ep;
	else if (mode == DW_PCIE_RC_TYPE)
		return dw_rc;

	return NULL;
}
EXPORT_SYMBOL(get_dw_pcie);

static int dw_plat_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_setup_rc(pp);

	//pcie_ltssm_enable(pci);
	//dw_pcie_wait_for_link(pci);

	dw_pcie_msi_init(pp);

	return 0;
}

static void dw_plat_set_num_vectors(struct pcie_port *pp)
{
	pp->num_vectors = MAX_MSI_IRQS;
}

static const struct dw_pcie_host_ops dw_plat_pcie_host_ops = {
	.host_init = dw_plat_pcie_host_init,
	.set_num_vectors = dw_plat_set_num_vectors,
};

static int dw_plat_pcie_establish_link(struct dw_pcie *pci)
{
	pcie_ltssm_enable(pci);
	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = dw_plat_pcie_establish_link,
};

static void dw_plat_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
	//pcie_ltssm_enable(pci);
}

static int dw_plat_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features dw_plat_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	//.msix_capable = true,
};

static const struct pci_epc_features*
dw_plat_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &dw_plat_pcie_epc_features;
}

static void pcie_reset_bus(struct device *dev)
{
	int ret;
	u32 gpio;
	struct device_node *np = dev->of_node;

	//PCIE0_RSTB / PCIE1_RSTB
	gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(gpio)) {
		pr_err("Unable to read reset-gpio property!\n");
		return;
	}

	ret = devm_gpio_request(dev, gpio, "pcie-reset");
	if (ret) {
		pr_err("Unable to request reset-gpio %d!\n", gpio);
		return;
	}

	gpio_direction_output(gpio, 0);
	udelay(20);
	gpio_direction_output(gpio, 1);
	udelay(20);
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = dw_plat_pcie_ep_init,
	.raise_irq = dw_plat_pcie_ep_raise_irq,
	.get_features = dw_plat_pcie_get_features,
};

static int dw_plat_add_pcie_port(struct dw_plat_pcie *dw_plat_pcie,
				 struct platform_device *pdev)
{
	struct dw_pcie *pci = dw_plat_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq[PCIE_DMA_INT] = platform_get_irq_byname(pdev, "dma");
	if (pp->irq[PCIE_DMA_INT] < 0)
		return pp->irq[PCIE_DMA_INT];
	pp->irq[PCIE_CORR_INT] = platform_get_irq_byname(pdev, "correctable");
	if (pp->irq[PCIE_CORR_INT] < 0)
		return pp->irq[PCIE_CORR_INT];
	pp->irq[PCIE_UNCORR_INT] = platform_get_irq_byname(pdev, "uncorrectable");
	if (pp->irq[PCIE_UNCORR_INT] < 0)
		return pp->irq[PCIE_UNCORR_INT];
	pp->irq[PCIE_OTHER_INT] = platform_get_irq_byname(pdev, "other");
	if (pp->irq[PCIE_OTHER_INT] < 0)
		return pp->irq[PCIE_OTHER_INT];

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0)
			return pp->msi_irq;
	}

	pp->ops = &dw_plat_pcie_host_ops;

	pcie_reset_bus(dev);
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int dw_plat_add_pcie_ep(struct dw_plat_pcie *dw_plat_pcie,
			       struct platform_device *pdev)
{
	int ret;
	struct dw_pcie_ep *ep;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = dw_plat_pcie->pci;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	pci->dbi_base2 = devm_platform_ioremap_resource_byname(pdev, "dbi2");
	if (IS_ERR(pci->dbi_base2))
		return PTR_ERR(pci->dbi_base2);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	pcie_reset_bus(dev);
	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "Failed to initialize endpoint\n");
		return ret;
	}
	return 0;
}

static void set_ltssm_count(void)
{
	ltssm_count++;
}

u32 get_ltssm_count(void)
{
	return ltssm_count;
}
EXPORT_SYMBOL(get_ltssm_count);

static int pcie_ltssm_detect(void *kkk)
{
	int devnum;
	u8 offset;
	u32 tmp, err_count = 0;
	u32 pcie_genx;
	u32 pcie_link_width;
	static u32 link_state;
	struct dw_pcie *pci = (struct dw_pcie *)kkk;
	struct pci_dev *pci_root_dev = NULL;

	for (devnum = 0; devnum < 32; devnum++) {
		pci_root_dev = pci_get_device(0x16C3, 0xABCD, pci_root_dev);
		if(!pci_root_dev) {
			pr_err("fail: No root PCI device found\n");
			return -ENODEV;
		}
		if (pci_is_root_bus(pci_root_dev->bus))
		 	break;
	}

	offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	pcie_ltssm_enable(pci);
	pr_info("%s start...\n", __func__);
	while (!kthread_should_stop()) {
		msleep(1000);
		if (pcie_is_link(pci)) { /* detect link states */
			err_count = 0;
			if (link_state)
				continue; /* Loop detection */
		} else {
			if (link_state && ++err_count >= 2) {
				pr_info("PCIe Link down!!! LTSSM:%#x\n", pci->phy->ltssm);
				/* Remove PCI bus devices from the device lists. */
				pci_stop_and_remove_bus_device_locked(pci_root_dev);
				msleep(100);

				pcie_ltssm_disable(pci);
				dw_pcie_setup_rc(&pci->pp);
				pcie_ltssm_enable(pci);

				tmp = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
				tmp &= ~PCI_EXP_LNKCAP_SLS;
				tmp |= PCI_EXP_LNKCAP_SLS_2_5GB;
				dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, tmp);
				err_count = 0;
				link_state = 0;
			}
			continue;
		}
		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
		tmp &= ~PCI_EXP_LNKCAP_SLS;
		tmp |= PCI_EXP_LNKCTL2_TLS_8_0GT;
		dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);
		msleep(1000);

		pr_info("PCIe Link up!!! LTSSM:%#x\n", pci->phy->ltssm);
		pci_lock_rescan_remove();
		/* Re-scan PCI parent buses for devices. */
		pci_rescan_bus(pci_root_dev->bus);
		/* Release a mutex lock. */
		pci_unlock_rescan_remove();

		tmp = dw_pcie_readl_dbi(pci, 0x80);
		pcie_genx = (tmp & 0xf0000) >> 16; /* bit[19:16] */
		pcie_link_width = (tmp & 0x3f00000)>>20; /* bit[25:20] */

		if (pcie_genx == 0x1)
			pr_info("PCIe Link Speed is GEN1\n");
		else if (pcie_genx == 0x2)
			pr_info("PCIe Link Speed is GEN2\n");
		else if (pcie_genx == 0x3)
			pr_info("PCIe Link Speed is GEN3\n");
		else
			pr_info("PCIe link speed 0x%x\n", pcie_genx);

		if (pcie_link_width == 0x2)
			pr_info("PCIe Link Width is x2\n");
		else if (pcie_link_width == 0x4)
			pr_info("PCIe Link Width is x4\n");
		else
			pr_info("PCIe link Width:0x%x\n", pcie_link_width);

		link_state = 1;
		set_ltssm_count();
	}

	return 0;
}

static void pcie_dscan_create(struct dw_pcie *pci)
{
	task = kthread_run(pcie_ltssm_detect, pci, "pcie_ltssm_detect");
	if (IS_ERR(task))
		pr_err("%s create thread fail\n", __func__);
}

static void pcie_dscan_exit(void)
{
	if (task) {
		kthread_stop(task);
		task = NULL;
	}
}

#ifdef PCIE_MEM_TEST
static void memory_rdwr_test(void)
{
	long start, curr, datasize = 1024*4;
	void *addr;
	int i = 0;
	u32 val;

	pr_err("*** %s, %d\n", __func__, __LINE__);
	addr = ioremap(0x47000000, 0x1000); /* 4M */
	if (!addr) {
		pr_err("*** ioremap fail%s, %d\n", __func__, __LINE__);
		return;
	}

	start = ktime_get_boottime();
	for (i = 0; i < 1024; i = i+4)
		dw_pcie_write(addr + i, 4, 0xff);

	curr = ktime_get_boottime();
	pr_err("mem write: time:%ld size:%ld speed:%ld MB/s\n\n",
		curr-start, datasize, ((datasize*1000000000)/(curr-start))/(1024*1024));

	start = ktime_get_boottime();
	for (i = 0; i < 1024; i = i+4)
		dw_pcie_read(addr + i, 4, &val);

	curr = ktime_get_boottime();
	pr_err("mem read: time:%ld size:%ld speed:%ld MB/s\n\n",
		curr-start, datasize, ((datasize*1000000000)/(curr-start))/(1024*1024));
}
#endif

static int dw_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_plat_pcie *dw_plat_pcie;
	struct dw_pcie *pci;
	struct resource *res;  /* Resource from DT */
	int ret;
	const struct of_device_id *match;
	const struct dw_plat_pcie_of_data *data;
	enum dw_pcie_device_mode mode;

	pr_err("pcie probe start...\n");
	match = of_match_device(dw_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = (struct dw_plat_pcie_of_data *)match->data;
	mode = (enum dw_pcie_device_mode)data->mode;

	dw_plat_pcie = devm_kzalloc(dev, sizeof(*dw_plat_pcie), GFP_KERNEL);
	if (!dw_plat_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;
	pci->mode = mode;

	dw_plat_pcie->pci = pci;
	dw_plat_pcie->mode = mode;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	ret = of_property_read_u32(dev->of_node, "controller-id", &pci->ctrl_id);
	if (ret || (pci->ctrl_id > 1) || (pci->ctrl_id < 0)) {
		pr_err("controller-id Undefined. set to 0\n");
		pci->ctrl_id = 0;
	}
	pr_err("pcie controller: %d, type: %s\n",
		pci->ctrl_id, (mode == DW_PCIE_RC_TYPE) ? "RC":"EP");

	pci->dma_base = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(pci->dma_base))
		return PTR_ERR(pci->dma_base);
	// pr_info("pci->dbi_base:%#lx dma:%#lx\n",
	// 	(uintptr_t)pci->dbi_base, (uintptr_t)pci->dma_base);

	platform_set_drvdata(pdev, dw_plat_pcie);
	bst_pcie_phyinit(pci);

	switch (dw_plat_pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_HOST))
			return -ENODEV;

		ret = dw_plat_add_pcie_port(dw_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		dw_rc = pci;
		pcie_dscan_create(pci);
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_EP))
			return -ENODEV;

		ret = dw_plat_add_pcie_ep(dw_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		pcie_ltssm_enable(pci);
		dw_ep = pci;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", dw_plat_pcie->mode);
	}
	pr_err("pcie probe completed.\n");

	return 0;
}

static int dw_plat_pcie_remove(struct platform_device *pdev)
{
	struct dw_plat_pcie *dw_plat_pcie = platform_get_drvdata(pdev);
	struct dw_pcie *pci = dw_plat_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct dw_pcie_ep *ep = &pci->ep;

	switch (dw_plat_pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_HOST))
			return -ENODEV;

		pcie_dscan_exit();
		dw_pcie_host_deinit(pp);
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_EP))
			return -ENODEV;

		dw_pcie_ep_exit(ep);
		devm_pci_epc_destroy(pci->dev, ep->epc);
		pci_epc_destroy(ep->epc);
		break;
	default:
		break;
	}
	pcie_ltssm_disable(pci);
	//rockchip_pcie_deinit_phys(rockchip);
	//rockchip_pcie_disable_clocks(rockchip);
	pr_err("pcie remove completed.\n");

	return 0;
}

static const struct dw_plat_pcie_of_data dw_plat_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct dw_plat_pcie_of_data dw_plat_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id dw_plat_pcie_of_match[] = {
	{
		.compatible = "bst,dw-pcie-rc",
		.data = &dw_plat_pcie_rc_of_data,
	},
	{
		.compatible = "bst,dw-pcie-ep",
		.data = &dw_plat_pcie_ep_of_data,
	},
	{},
};

static struct platform_driver dw_plat_pcie_driver = {
	.driver = {
		.name	= "dw-pcie",
		.of_match_table = dw_plat_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = dw_plat_pcie_probe,
	.remove = dw_plat_pcie_remove,
};
//builtin_platform_driver(dw_plat_pcie_driver);
module_platform_driver(dw_plat_pcie_driver);

MODULE_LICENSE("GPL");

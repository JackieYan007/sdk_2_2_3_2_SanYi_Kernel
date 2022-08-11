/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>

#include "mve_rsrc_mem_frontend.h"
#include "mve_rsrc_driver.h"
#include "mve_rsrc_irq.h"
#include "mve_rsrc_log.h"
#include "mve_rsrc_register.h"
#include "mve_rsrc_scheduler.h"
#include "mve_rsrc_pm.h"

#include "machine/mve_port_attributes.h"

static int mve_dev_major;
struct mve_rsrc_driver_data mve_rsrc_data;

extern struct mve_config_attribute *mve_device_get_config(void);

#if defined(DEVICETREE)
/* Mali-V500 can handle physical addresses 40 bits wide */
static uint64_t mv500_dma_mask = DMA_BIT_MASK(40);
#endif

static int mver_driver_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int mver_driver_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct of_device_id mve_rsrc_of_match[] =
{
    {
        .compatible = "arm,mali-v500",
    },
    {
        .compatible = "arm,mali-v550",
    },
    {
        .compatible = "arm,mali-mve",
    },
    {}
};
MODULE_DEVICE_TABLE(of, mve_rsrc_of_match);

static struct file_operations rsrc_fops =
{
    .owner = THIS_MODULE,
    .open = mver_driver_open,
    .release = mver_driver_release,
};
static u32 mali_read_phys(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
    void *mem_mapped = ioremap(phys_addr_page, map_size);
#else
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);
#endif
	if (NULL != mem_mapped) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}

static void mve_write_phys(u32 phys_addr, u32 value)
{
    u32 phys_addr_page = phys_addr & 0xFFFFE000;
    u32 phys_offset    = phys_addr & 0x00001FFF;
    u32 map_size       = phys_offset + sizeof(u32);
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
    void *mem_mapped = ioremap(phys_addr_page, map_size);
#else
    void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);
#endif
    if (NULL != mem_mapped) {
        iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
        iounmap(mem_mapped);
    }
}
static void reset_soft(int reset)
{
    struct mve_rsrc_driver_data *private = &mve_rsrc_data;
    if(reset)
    {
        //mve_write_phys(0X33002174,mali_read_phys(0X33002174)|(0x3<<30));
        mve_write_phys(0X3300217c,mali_read_phys(0X3300217c)|(1<<3));
    }
    else
    {
        //mve_write_phys(0X33002174,mali_read_phys(0X33002174)&(~(0x3<<30));
        mve_write_phys(0X3300217c,mali_read_phys(0X3300217c)&(~(1<<3)));
    }
}

static void reset_hw(void)
{
    tCS *regs;
    uint32_t ncore;
    uint32_t corelsid_mask;
    regs = mver_reg_get_coresched_bank();    

    reset_soft(1);
    ncore = mver_reg_read32(&regs->NCORES);
    corelsid_mask = 0;
    for (; ncore > 0; --ncore)
    {
        corelsid_mask = (corelsid_mask << 4) | 0xF;
    }
    mver_reg_write32(&regs->RESET, 1);
    while (corelsid_mask !=  mver_reg_read32(&regs->CORELSID))
        ;

    /* Let the hardware power down */
    mver_reg_write32(&regs->CLKFORCE, 0);

    mver_reg_put_coresched_bank(&regs);
}

static int mver_driver_probe(struct platform_device *pdev)
{
    struct mve_rsrc_driver_data *private = &mve_rsrc_data;
    int ret = 0;
    struct resource *res;
    struct resource *irq_res;
    tCS *regs;
    mve_port_attributes_callback_fptr attr_fptr;

#ifdef DEVICETREE
    const struct of_device_id *of_match =
        of_match_device(mve_rsrc_of_match, &pdev->dev);

    if (NULL == of_match)
    {
        /* Driver doesn't support this device */
        printk(KERN_ERR "MVE: No matching device to Mali-MVE of_node: %p.\n", pdev->dev.of_node);
        return -EINVAL;
    }
#endif

    mve_dev_major = register_chrdev(0, MVE_RSRC_DRIVER_NAME, &rsrc_fops);
    if (0 > mve_dev_major)
    {
        printk(KERN_ERR "MVE: Failed to register the driver \'%s\'.\n", MVE_RSRC_DRIVER_NAME);
        ret = mve_dev_major;
        goto error;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (NULL == res)
    {
        printk(KERN_ERR "MVE: No Mali-MVE I/O registers defined.\n");
        ret = -ENXIO;
        goto error;
    }

    irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (NULL == irq_res)
    {
        printk(KERN_ERR "MVE: No IRQ defined for Mali-MVE.\n");
        ret = -ENODEV;
        goto error;
    }
    private->irq_nr = irq_res->start;
    private->irq_flags = irq_res->flags;

    private->mem_res = request_mem_region(res->start, resource_size(res), MVE_RSRC_DRIVER_NAME);
    if (!private->mem_res)
    {
        printk(KERN_ERR "MVE: Failed to request Mali-MVE memory region.\n");
        ret = -EBUSY;
        goto error;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
    private->regs = ioremap(res->start, resource_size(res));
#else
    private->regs = ioremap_nocache(res->start, resource_size(res));
#endif
    if (NULL == private->regs)
    {
        printk(KERN_ERR "MVE: Failed to map Mali-MVE registers.\n");
        ret = -ENXIO;
        goto error;
    }

    private->pdev = pdev;
    private->irq_enable_count = 0;

#if defined(DEVICETREE)
    /* There is no way to set the dma_mask using device trees.  */
    pdev->dev.dma_mask = &mv500_dma_mask;
    pdev->dev.coherent_dma_mask = mv500_dma_mask;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
    of_dma_configure(&pdev->dev, pdev->dev.of_node,1);
#endif
#endif

    private->config = mve_device_get_config();
    if (NULL == private->config)
    {
        printk(KERN_ERR "MVE: Failed to request Mali-MVE driver configuration.\n");
        ret = -ENXIO;
        goto error;
    }

    attr_fptr = (mve_port_attributes_callback_fptr)
                mve_config_get_value(private->config, MVE_CONFIG_DEVICE_ATTR_BUS_ATTRIBUTES);
    if (NULL == attr_fptr)
    {
        printk(KERN_ERR "MVE: Failed to request MVE_CONFIG_DEVICE_ATTR_BUS_ATTRIBUTES.\n");
        ret = -ENXIO;
        goto error;
    }

    /* Fetch the AXI memory access settings */
    private->port_attributes[0] = attr_fptr(0);
    private->port_attributes[1] = attr_fptr(1);
    private->port_attributes[2] = attr_fptr(2);
    private->port_attributes[3] = attr_fptr(3);

	private->mve_clk = devm_clk_get(&pdev->dev, "clk_v500");
	if (!IS_ERR(private->mve_clk))
    {       
		clk_prepare_enable(private->mve_clk);
    }

    private->hw_interaction = true;

    mver_reg_init();
    mve_rsrc_mem_init(&pdev->dev);
    mve_rsrc_log_init();
    mver_irq_handler_init(&pdev->dev);
    mver_pm_init(&pdev->dev);
    mver_scheduler_init(&pdev->dev);

    pm_runtime_set_suspended(&pdev->dev);
    pm_runtime_enable(&pdev->dev);

    pm_runtime_get_sync(&pdev->dev);

    reset_hw();
    regs = mver_reg_get_coresched_bank();
    private->nlsid = mver_reg_read32(&regs->NLSID);
    private->ncore = mver_reg_read32(&regs->NCORES);
    private->fuse = mver_reg_read32(&regs->FUSE);
    private->hw_version = mver_reg_read32(&regs->VERSION);

    mver_reg_put_coresched_bank(&regs);
    pm_runtime_put_sync(&pdev->dev);

    platform_set_drvdata(pdev, private);

    printk("MVE resource driver loaded successfully (nlsid=%u, cores=%u, version=0x%X)\n",
           private->nlsid, private->ncore, private->hw_version);

    return ret;

error:
    printk(KERN_ERR "Failed to load the driver \'%s\'.\n", MVE_RSRC_DRIVER_NAME);
    if (NULL != private->regs)
    {
        iounmap(private->regs);
    }

    if (0 < mve_dev_major)
    {
        unregister_chrdev(mve_dev_major, MVE_RSRC_DRIVER_NAME);
    }

    return ret;
}

static int mver_driver_remove(struct platform_device *pdev)
{
    mver_scheduler_deinit(&pdev->dev);
    mver_irq_handler_deinit(&pdev->dev);
    mve_rsrc_log_destroy();
    mve_rsrc_mem_deinit(&pdev->dev);
    mver_pm_deinit(&pdev->dev);
    pm_runtime_disable(&pdev->dev);

    iounmap(mve_rsrc_data.regs);
    release_mem_region(mve_rsrc_data.mem_res->start, resource_size(mve_rsrc_data.mem_res));
    unregister_chrdev(mve_dev_major, MVE_RSRC_DRIVER_NAME);

    printk("MVE resource driver unloaded successfully\n");

    return 0;
}

static const struct dev_pm_ops rsrc_pm =
{
#ifdef CONFIG_PM
    .suspend = mver_pm_suspend,
    .resume = mver_pm_resume,
    .runtime_suspend = mver_pm_runtime_suspend,
    .runtime_resume = mver_pm_runtime_resume,
    .runtime_idle = mver_pm_runtime_idle,
#endif
};

static struct platform_driver mv500_rsrc_driver =
{
    .probe = mver_driver_probe,
    .remove = mver_driver_remove,

    .driver = {
        .name = MVE_RSRC_DRIVER_NAME,
        .owner = THIS_MODULE,
        .pm = &rsrc_pm,
        .of_match_table =of_match_ptr(mve_rsrc_of_match),
    },
};

#ifndef DEVICETREE
extern void mver_init_machine(void);
extern void mver_deinit_machine(void);
#endif

int  mver_driver_init(void)
{
    platform_driver_register(&mv500_rsrc_driver);

#ifndef DEVICETREE
    /* Driver built without device tree (DT) support */
    mver_init_machine();
#endif

    return 0;
}

void  mver_driver_exit(void)
{
    platform_driver_unregister(&mv500_rsrc_driver);

#ifndef DEVICETREE
    /* Driver built without device tree (DT) support */
    mver_deinit_machine();
#endif

    MVE_LOG_PRINT(&mve_rsrc_log, MVE_LOG_INFO, "MVE resource driver unregistered");
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mali-V500 resource driver");

module_init(mver_driver_init);
module_exit(mver_driver_exit);

EXPORT_SYMBOL(mve_rsrc_data);

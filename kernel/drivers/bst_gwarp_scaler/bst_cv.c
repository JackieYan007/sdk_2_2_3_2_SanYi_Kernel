/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: jason.Yin@bst.ai
 *
 * @file    bst_cv.c
 * @brief   This file is the top source code file of bst_cv driver. It contains
 *          function definitions of driver setup and interface.
 */
#include <linux/interrupt.h>
#include <linux/mman.h>
#include "bst_cv.h"

#define BSTCV_CTL_BASE_PHY		0x51030000
#define BSTCV_CTL_GWARP_IRQ_REG		0x5106003c
#define BSTCV_CTL_SCALER_IRQ_REG	0x51050008
#define SYS_CTRL_R_CORENOC_PARITY_ENABLE (0x33000084)
#define CV_PARITY_EN			 (0x2)

#define BSTCV_CTL_GWARP_CLK_BIT	14
#define BSTCV_CTL_SCLR_CLK_BIT		15
#define BSTCV_CTL_DMA_CLK_BIT		16
#define BSTCV_CTL_SOFT_RST_DMA_BIT	17
#define BSTCV_CTL_SOFT_RST_SCLR_BIT	18
#define BSTCV_CTL_SOFT_RST_GWARP_BIT	19

/******************************************************/
static int bst_cv_probe(struct platform_device *pdev);
static int bst_cv_remove(struct platform_device *pdev);
static void bst_cv_shutdown(struct platform_device *pdev);

// globle data define
int bst_cv_gs_print_level = BST_CV_NO_PRINT;

static const struct of_device_id bst_cv_of_match[] = {
    {
	.compatible = "bst,bst_gwarp_scaler,cma",
    },
};

static struct platform_driver bst_cv_driver = {
    .probe   = bst_cv_probe,
    .remove  = bst_cv_remove,
    .shutdown = bst_cv_shutdown,
    .driver  = {
	.name = BST_CV_DRIVER_NAME,
	.of_match_table = of_match_ptr(bst_cv_of_match),
    },
};

static int bst_cv_preinit(struct bst_cv *pbst_cv)
{
	void __iomem *cv_ctl_base = NULL;
	__u32 reg;
	__u32 cv_parity_en;
    void __iomem *crm_base = NULL;
    void __iomem *corenoc_parity_enable = NULL;
    void __iomem *cv_sys_ctl_base = NULL;
	BST_CV_GS_TRACE_PRINTK("BST_CV Enable Clock and Soft reset");

    crm_base = ioremap(0x33002000, 0x200); // SYS_CRM base address
    // set up CRM register for CV block, or else OS hangs after vector reset
    reg = readl_relaxed(crm_base + 0x4); // TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
    reg |= (1UL << 7); // cv block soft reset
    writel_relaxed(reg, crm_base + 0x4);

    // set CV freq to 800MHz
    reg = readl_relaxed(crm_base + 0x15c); // CLKMUX_SEL2
    // CLK_800_CV_CORE_CLK_SEL, bit[3:2] = 01: 800MHz
    reg &= ~(1UL << 3);
    reg |= (1UL << 2);
    writel_relaxed(reg, crm_base + 0x15c);

    corenoc_parity_enable =
	devm_ioremap(&pbst_cv->pdev->dev, SYS_CTRL_R_CORENOC_PARITY_ENABLE, 4);
    if (corenoc_parity_enable == NULL) {
	BST_CV_DEV_ERR(pbst_cv->dev,
	    "IOREMAP SYS_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
	    SYS_CTRL_R_CORENOC_PARITY_ENABLE);
	return -1;
    }
    cv_parity_en = readl_relaxed(corenoc_parity_enable) & CV_PARITY_EN;

	cv_sys_ctl_base = devm_ioremap(pbst_cv->dev, BSTCV_CTL_BASE_PHY + 0x50, 4);
    reg = readl_relaxed(cv_sys_ctl_base);
    if (cv_parity_en) {
	reg |= 0x3; // cv block ecc and parity enable
	writel_relaxed(reg, cv_sys_ctl_base);
    }

	cv_ctl_base = devm_ioremap(pbst_cv->dev, BSTCV_CTL_BASE_PHY, 4);
	reg = readl_relaxed(cv_ctl_base);
	/*clk enable*/
	reg |= 1UL << BSTCV_CTL_GWARP_CLK_BIT;
	reg |= 1UL << BSTCV_CTL_SCLR_CLK_BIT;
	reg |= 1UL << BSTCV_CTL_DMA_CLK_BIT;
	/*soft reset*/
	reg |= 1UL << BSTCV_CTL_SOFT_RST_DMA_BIT;
	reg |= 1UL << BSTCV_CTL_SOFT_RST_SCLR_BIT;
	reg |= 1UL << BSTCV_CTL_SOFT_RST_GWARP_BIT;
	writel_relaxed(reg, cv_ctl_base);

	iounmap(crm_base);
	devm_iounmap(pbst_cv->dev, corenoc_parity_enable);
	devm_iounmap(pbst_cv->dev, cv_sys_ctl_base);
	devm_iounmap(pbst_cv->dev, cv_ctl_base);
	return 0;
}

static void gwarp_work_handler(struct work_struct *work)
{
	int ret;
	struct bst_cv *pbst_cv;
	struct bst_cv_msg_manager *msg_manager;
	struct bst_cv_irq_manager *irq_manager;
	struct gwarp_param *g_param;

	BST_CV_GS_TRACE_PRINTK("start");

	pbst_cv = container_of(work, struct bst_cv, gwarp_work);
	if (pbst_cv == NULL) {
		BST_CV_GS_TRACE_PRINTK("pbst_cv is NULL");
	}

	msg_manager = &pbst_cv->msg_manager;
	irq_manager = &pbst_cv->irq_manager;
	g_param = &msg_manager->cache_msg.req.g_param;

	//Gwarp CV model ARM Reg Config
	mutex_lock(&msg_manager->gwarp_lock);
	bst_gwarp_start(pbst_cv, *g_param);
	mutex_unlock(&msg_manager->gwarp_lock);

	ret = wait_for_completion_timeout(&irq_manager->gwarp_irq_complete,
							msecs_to_jiffies(500));
	if (!ret)
		BST_CV_GS_TRACE_PRINTK("wait for irq_complete time out");

	complete(&msg_manager->gwarp_cv_complete);
	BST_CV_GS_TRACE_PRINTK("end");
}

static void scaler_work_handler(struct work_struct *work)
{

}

static irqreturn_t bst_cv_hardirq(int irq, void *dev_id)
{
	struct bst_cv *pbst_cv = dev_id;
	__u32 reg = 0;

	// BST_CV_DEV_INFO(pbst_cv->dev, "receive IRQ");

	if (pbst_cv->irq_manager.gwarp_irq_reg == NULL) {
		BST_CV_DEV_ERR(pbst_cv->dev, "gawrp_irq_reg is NULL");
		goto exit;
	}

	if (pbst_cv->irq_manager.scaler_irq_reg == NULL) {
		BST_CV_DEV_ERR(pbst_cv->dev, "scaler_irq_reg is NULL");
		goto exit;
	}
	/*
	 * Check IRQ Source
	 * Gwarp: 0x5106003c bit0
	 */
	reg = readl_relaxed(pbst_cv->irq_manager.gwarp_irq_reg);
	if (reg & 0x1) {
		set_bit(0, &pbst_cv->irq_manager.gwarp_irq_flag);
		bst_gwarp_clear_intr(pbst_cv);
		bst_enable_gwarp(0);
		goto thread;
	}

	/*
	 * Check IRQ Source
	 * Scaler: 0x51050008 bit0
	 */
	reg = readl_relaxed(pbst_cv->irq_manager.scaler_irq_reg);
	if (reg & 0x1) {
		set_bit(0, &pbst_cv->irq_manager.scaler_irq_flag);
		goto thread;
	}
exit:
	return IRQ_NONE;
thread:
	return IRQ_WAKE_THREAD;
}

static irqreturn_t bst_cv_irq(int irq, void *dev_id)
{
	struct bst_cv *pbst_cv = (struct bst_cv *)dev_id;
	struct bst_cv_irq_manager *irq_manager = &pbst_cv->irq_manager;

	BST_CV_GS_TRACE_PRINTK("start");

	if (test_bit(0, &irq_manager->gwarp_irq_flag)) {
		/*Gwarp*/
		// BST_CV_DEV_INFO(pbst_cv->dev, "Gwarp save output");
		clear_bit(0, &irq_manager->gwarp_irq_flag);
	} else {
		// BST_CV_DEV_INFO(pbst_cv->dev, "Scaler save output");
		clear_bit(0, &irq_manager->scaler_irq_flag);
	}
	BST_CV_GS_TRACE_PRINTK("end");
	complete(&irq_manager->gwarp_irq_complete);
	return IRQ_HANDLED;
}

/*******************************************************************************
 * BST CV Driver Interface
 ******************************************************************************/
/*
 * @func    bst_cv_probe
 * @brief   This is the probe callback function of bst_cv driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0 for success and error code otherwise
 */
static int bst_cv_probe(struct platform_device *pdev)
{
	int ret;
	__u32 id;
	char work_queue_name[64];
	struct bst_cv *pbst_cv;

	BST_CV_GS_TRACE_PRINTK("BST_CV driver initializing ...");
	pbst_cv = devm_kzalloc(&pdev->dev, sizeof(struct bst_cv), GFP_KERNEL);
	if (pbst_cv == NULL) {
		return -ENOMEM;
	}
	BST_CV_DEV_INFO(&pdev->dev, "pbst_cv : %p", pbst_cv);
	pbst_cv->pdev = pdev;
	pbst_cv->dev = &pdev->dev;
	platform_set_drvdata(pdev, pbst_cv);
	pbst_cv->gwarp_state = BST_CV_INIT;
	pbst_cv->scaler_state = BST_CV_INIT;

	mutex_init(&pbst_cv->mutex);
	bst_cv_gwarp_miscdev_init(pbst_cv);

	//get device id from the device tree
	ret = device_property_read_u32_array(&pdev->dev, "id", &id, 1);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pdev->dev, "no id property, ret %d", ret);
		return ret;
	} else {
		BST_CV_DEV_INFO(&pdev->dev, "bst_cv_device_id : %d", id);
	}

	/*msg_manager*/
	memset(work_queue_name, 0, 64);
	snprintf(work_queue_name, 64, "%s", "gwarp_work_queue");
	pbst_cv->msg_manager.gwarp_work_queue = create_singlethread_workqueue(work_queue_name);
	if (pbst_cv->msg_manager.gwarp_work_queue == NULL) {
		BST_CV_DEV_ERR(&pdev->dev, "could not create msg_work_queue\n");
		ret = -ENOMEM;
		return ret;
	}
	snprintf(work_queue_name, 64, "%s", "scaler_work_queue");
	pbst_cv->msg_manager.scaler_work_queue = create_singlethread_workqueue(work_queue_name);
	if (pbst_cv->msg_manager.gwarp_work_queue == NULL) {
		BST_CV_DEV_ERR(&pdev->dev, "could not create msg_work_queue\n");
		ret = -ENOMEM;
		return ret;
	}

	INIT_WORK(&pbst_cv->gwarp_work, gwarp_work_handler);
	INIT_WORK(&pbst_cv->scaler_work, scaler_work_handler);

	init_completion(&pbst_cv->msg_manager.gwarp_complete);
	init_completion(&pbst_cv->msg_manager.gwarp_cv_complete);
	init_completion(&pbst_cv->irq_manager.gwarp_irq_complete);

	init_completion(&pbst_cv->msg_manager.scaler_complete);
	init_completion(&pbst_cv->irq_manager.scaler_irq_complete);

	mutex_init(&pbst_cv->msg_manager.gwarp_lock);
	mutex_init(&pbst_cv->msg_manager.scaler_lock);

    /*IRQ manager*/
	pbst_cv->irq_manager.gwarp_irq_reg = devm_ioremap(&pdev->dev, BSTCV_CTL_GWARP_IRQ_REG, 4);
	if (pbst_cv->irq_manager.gwarp_irq_reg == NULL) {
		BST_CV_DEV_ERR(pbst_cv->dev, "ioremap failed");
		ret = -ENOMEM;
	}
	pbst_cv->irq_manager.scaler_irq_reg = devm_ioremap(&pdev->dev, BSTCV_CTL_GWARP_IRQ_REG, 4);
	if (pbst_cv->irq_manager.scaler_irq_reg == NULL) {
		BST_CV_DEV_ERR(pbst_cv->dev, "ioremap failed");
		ret = -ENOMEM;
	}

	pbst_cv->irq_manager.irq = platform_get_irq_byname(pdev, "bst_cv_irq");
	if (pbst_cv->irq_manager.irq < 0) {
		BST_CV_DEV_ERR(&pdev->dev,
				"bst_cv_irq configuration information not found\n");
		return -EFAULT;
	} else {
		BST_CV_DEV_INFO(&pdev->dev, "bst_cv_irq : %d\n", pbst_cv->irq_manager.irq);
	}

	ret = devm_request_threaded_irq(&pdev->dev, pbst_cv->irq_manager.irq, bst_cv_hardirq, bst_cv_irq,
				IRQF_TRIGGER_HIGH | IRQF_SHARED | IRQF_ONESHOT, dev_name(pbst_cv->dev), pbst_cv);
	if (ret) {
		BST_CV_DEV_ERR(pbst_cv->dev, "Couldn't request CV IRQ");
		return -EFAULT;
	}

	/*mem manager init*/
	bst_cv_gs_mem_manager_init(pbst_cv);
	/*Enable CV moudle*/
	bst_cv_preinit(pbst_cv);
	/*remap gwarp register*/
	bst_gwarp_preinit(pbst_cv);
	//bst_scaler_preinit(pbst_cv);

	pbst_cv->gwarp_state = BST_CV_IDLE;
	pbst_cv->scaler_state = BST_CV_IDLE;

	return 0;
}


/*
 * @func    bst_cv_remove
 * @brief   This is the remove callback function of bst_cv driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0
 */
static int bst_cv_remove(struct platform_device *pdev)
{
	struct bst_cv *pbst_cv;

	BST_CV_GS_TRACE_PRINTK("%s", "cv_remove");
	pbst_cv = platform_get_drvdata(pdev);

	flush_workqueue(pbst_cv->msg_manager.gwarp_work_queue);
	flush_workqueue(pbst_cv->msg_manager.scaler_work_queue);
	destroy_workqueue(pbst_cv->msg_manager.gwarp_work_queue);
	destroy_workqueue(pbst_cv->msg_manager.scaler_work_queue);
	devm_free_irq(pbst_cv->dev, pbst_cv->irq_manager.irq, pbst_cv);
	bst_cv_gwarp_miscdev_exit(pbst_cv);
	bst_cv_mem_manager_exit(pbst_cv);
	//bst_cv_deinit(pbst_cv);
	return 0;
}

static void bst_cv_shutdown(struct platform_device *pdev)
{
    BST_CV_GS_TRACE_PRINTK("%s", "this is a shutdown test");
    return;
}

// register the BST_CV driver on platform bus
static int __init bst_cv_driver_init(void)
{
    return platform_driver_register(&bst_cv_driver);
}
// initialize the BST_CV driver after the BST_IPC driver which is in device_initcall_sync
late_initcall(bst_cv_driver_init);

static void __exit bst_cv_driver_exit(void)
{
    platform_driver_unregister(&bst_cv_driver);
    return;
}
module_exit(bst_cv_driver_exit);

MODULE_AUTHOR("Jason Yin");
MODULE_DESCRIPTION("bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP");
MODULE_LICENSE("GPL");


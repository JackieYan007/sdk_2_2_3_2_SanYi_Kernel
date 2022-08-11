/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file    bst_cv.c
 * @brief   This file is the top source code file of bst_cv driver. It contains
 *          function definitions of driver setup and interface. 
 */

#include "bst_cv.h"
#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/moduleparam.h>  

/******************************************************/

static int bst_cv_probe(struct platform_device *pdev);
static int bst_cv_remove(struct platform_device *pdev);
static void bst_cv_shutdown(struct platform_device *pdev);

// global data define
static const struct of_device_id bst_cv_of_match[] = {
    {
        .compatible = "bst,bare_cv,cma",
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
    int ret = 0;
    unsigned int id = 0;
    struct bst_cv *pbst_cv;


    printk("BST_CV driver initializing ...");
    // printk("reset_cores= %s\n", reset_cores);
    
    pbst_cv = devm_kzalloc(&pdev->dev, sizeof(*pbst_cv), GFP_KERNEL);
    if (pbst_cv == NULL) {
        return -ENOMEM;
    }
    
    //init bst_cv device
    pbst_cv->pdev = pdev;
    platform_set_drvdata(pdev, pbst_cv);
    mutex_init(&pbst_cv->mutex);

    //init sysfile
    ret = bare_cv_sysfile_init(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bare_cv_sysfile_init failed, ret %d", ret);
        goto err_sysfile_init;
    }
    BST_CV_STAGE_PRINTK("bst_sysfile_init OK, /sys/kernel/%s registered", BST_CV_DRIVER_NAME);
    
    //init bst_cv memory manager
    ret = bare_cv_mem_manager_init(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bare_cv_mem_manager_init failed, ret %d", ret);
        goto err_mem_manager;
    }
    BST_CV_STAGE_PRINTK("bare_cv_mem_manager_init OK");

    //init bst_cv firmware manager
    ret = bare_cv_fw_manager_init(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bare_cv_fw_manager_init all failed");
        goto err_fw_manager;
    }
    BST_CV_STAGE_PRINTK("bare_cv_fw_manager_init OK");

    //init misc device
    ret = bare_cv_miscdev_init(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_misc_init failed, ret %d", ret);
        goto err_misc_init;
    }
    BST_CV_STAGE_PRINTK("bst_cv_misc_init OK, /dev/%s registered", pbst_cv->miscdev.name);

err_misc_init:
err_msg_manager:
    if (ret == 0) {
        pbst_cv->state = BST_CV_INIT;
        BST_CV_STAGE_PRINTK("%s", "bst_cv probe completed!");
        return 0;
    }
err_fw_manager: 
    bare_cv_mem_manager_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bare_cv_mem_manager_exit OK");
err_mem_manager:
    bare_cv_sysfile_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bare_cv_sysfile_exit OK");
err_sysfile_init:
    devm_kfree(&pdev->dev, pbst_cv);
    BST_CV_STAGE_PRINTK("%s", "probe exit");
    return ret;
}

/*
 * @func    bst_cv_remove
 * @brief   This is the remove callback function of bst_cv driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0
 */
static int bst_cv_remove(struct platform_device *pdev)
{
    struct bst_cv *pbst_cv = platform_get_drvdata(pdev);

    BST_CV_STAGE_PRINTK("%s","remove enter");

    bare_cv_miscdev_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bst_cv_misc_exit OK");
    bare_cv_sysfile_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bare_cv_sysfile_exit OK");
    bare_cv_fw_rt_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bare_cv_fw_rt_exit OK");
    bare_cv_fw_manager_exit(pbst_cv);
    BST_CV_STAGE_PRINTK("bare_cv_fw_manager_exit OK");
    if (bare_cv_check_online(pbst_cv)) {
        bare_cv_mem_manager_exit(pbst_cv);
        BST_CV_STAGE_PRINTK("bare_cv_mem_manager_exit OK");
    }
    devm_kfree(&pdev->dev, pbst_cv);
    
    BST_CV_STAGE_PRINTK("%s", "remove completed");
    return 0;
}

static void bst_cv_shutdown(struct platform_device *pdev)
{
    BST_CV_STAGE_PRINTK("%s", "this is a shutdown test");
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

MODULE_AUTHOR("Gou Wenjian");
MODULE_DESCRIPTION("bare_cv: Linux device driver for Black Sesame Technologies Computer Vision DSP");
MODULE_LICENSE("GPL");


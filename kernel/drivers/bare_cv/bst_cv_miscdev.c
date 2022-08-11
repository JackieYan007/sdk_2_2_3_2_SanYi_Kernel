/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file    bst_cv_misc.c
 * @brief   This file is the source code file of misc device interface of bst_cv
 *          driver. It contains function definitions of ioctl callbacks,
 *          initialization and exit of the misc device.
 * @note    Currently, model related calls should not be used.
 */

#include "bst_cv.h"

static const char bst_cv_dev_name_prefix[] = "bare_cv";

/*******************************************************************************
 * bst_cv MISC IOCTL INTERFACE
 ******************************************************************************/
/*
 * @func    bst_cv_ioctl_alloc
 * @brief   This function allocates a continuous memory buffer requested by the
 *          user.
 * @params  filp - the file pointer to the misc device
 *          pbst_cv - the pointer to the bst_cv device
 *          palloc - the user pointer to the memory buffer metadata prefilled
 *          with requested size
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_ioctl_alloc(struct file *filp, struct bst_cv *pbst_cv,
    struct xrp_ioctl_alloc __user *palloc)
{
    struct xrp_ioctl_alloc alloc;
    int ret;

    BST_CV_TRACE_PRINTK("enter, user ptr: %px", palloc);

    ret = copy_from_user(&alloc, palloc, sizeof(alloc));
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed!");
        return ret;
    }
    if (alloc.size == 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "zero size buffer!");
        return -EINVAL;
    }

    ret = bare_cv_user_buffer_alloc(filp, pbst_cv, &alloc);

    ret = copy_to_user(palloc, &alloc, sizeof(alloc));
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_to_user failed!");
        bare_cv_user_buffer_free(filp, pbst_cv, &alloc);
    }

    BST_CV_TRACE_PRINTK("exit");
    return ret;
}

/*
 * @func    bst_cv_ioctl_free
 * @brief   This function frees the allocated continuous memory buffer.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          palloc - the user pointer to the metadata of the memory buffer to be
 *          freed
 * @return  0 for success and linux error code otherwise
 */
static int bst_cv_ioctl_free(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc __user *palloc)
{   
    struct xrp_ioctl_alloc alloc;
    int ret;

    BST_CV_TRACE_PRINTK("enter, user ptr: %px", palloc);

    ret = copy_from_user(&alloc, palloc, sizeof(alloc));
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed!");
        return ret;
    }
    BST_CV_TRACE_PRINTK("buffer addr: 0x%x", alloc.addr);

    ret = bare_cv_user_buffer_free(filp, pbst_cv, &alloc);

    BST_CV_TRACE_PRINTK("exit");
    return ret;
}

/*
 * @func    bst_cv_ioctl_sync
 * @brief   This function synchronizes the allocated continuous memory buffer.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          palloc - the user pointer to the metadata of the memory buffer to be
 *          freed
 * @return  0 for success and linux error code otherwise
 */
static int bst_cv_ioctl_sync(struct bst_cv *pbst_cv,
    struct xrp_ioctl_alloc __user *palloc)
{
    struct xrp_ioctl_alloc alloc;

    if (copy_from_user(&alloc, palloc, sizeof(alloc))){
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed");
        return -EFAULT;
    }
    
    // BST_CV_STAGE_PRINTK("before flush           size = %d,    align = 0x%x,     addr = 0x%x,    ptr = 0x%x,     kaddr = 0x%lx\n", alloc.size, alloc.align, alloc.addr, alloc.ptr, alloc.kaddr);

    __flush_dcache_area(alloc.kaddr, alloc.size);

    BST_CV_STAGE_PRINTK("after flush            size = %d,    align = 0x%x,     addr = 0x%x,    ptr = 0x%x,     kaddr = 0x%lx\n", alloc.size, alloc.align, alloc.addr, alloc.ptr, alloc.kaddr);
    BST_CV_TRACE_PRINTK("exit");
    
    return 0;
}

int bst_cv_ioctl_inv_cache(struct bst_cv *pbst_cv,
    struct xrp_ioctl_alloc __user *palloc)
{
    struct xrp_ioctl_alloc alloc;

    BST_CV_TRACE_PRINTK("enter, user ptr: %px", palloc);

    if (copy_from_user(&alloc, palloc, sizeof(alloc))){
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed");
        return -EFAULT;
    }
    __inval_dcache_area(alloc.kaddr, alloc.size);

    return 0;
}


/*
 * @func    bst_cv_ioctl
 * @brief   This function is a wrapper bst_cv ioctl interface.
 * @params  filp - file pointer to the misc device
 *          cmd - ioctl command
 *          arg - pointer to the argument of the ioctl command
 * @return  void
 */
static long bst_cv_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
    int ret = 0;
    struct bst_cv *pbst_cv;

    BST_CV_TRACE_PRINTK("%s, cmd: 0x%x", "enter", cmd);

    // check the device driver
    if (filp == NULL) {
        printk(KERN_ERR "invalid file!");
        return -EFAULT;
    }
    if (filp->private_data == NULL) {
        printk(KERN_ERR "not bst_cv device!");
        return -EFAULT;
    }
    if (!try_module_get(THIS_MODULE)) {
        printk(KERN_ERR "try_module_get failed!");
        return -EFAULT;
    }

    pbst_cv = container_of(filp->private_data, struct bst_cv, miscdev);
    if (pbst_cv->state != BST_CV_ONLINE) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid device state!");
        return -EFAULT;
    }

    switch(cmd) {
        case XRP_IOCTL_ALLOC:
            ret = bst_cv_ioctl_alloc(filp, pbst_cv, (void __user *)args);
            break;
        case XRP_IOCTL_FREE:
            ret = bst_cv_ioctl_free(filp, pbst_cv, (void __user *)args);
            break;
        case XRP_IOCTL_SYNC:
            ret = bst_cv_ioctl_sync(pbst_cv, (void __user *)args);
            break;
         case XRP_IOCTL_INV_CACHE:
            ret = bst_cv_ioctl_inv_cache(pbst_cv, (void __user *)args);
            break;
        default:
            ret = -EINVAL;
    }

    module_put(THIS_MODULE);
    BST_CV_TRACE_PRINTK("%s, ret: %d", "exit", ret);
    return ret;
}


/*******************************************************************************
 * bst_cv MISC FILE INTERFACE
 ******************************************************************************/
/*
 * @func    bst_cv_open
 * @brief   This is the open callback function of the bst_cv Misc Device.
 * @params  inode - the inode pointer of the bst_cv misc device
 *          filp - the file pointer of the bst_cv misc device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_open(struct inode *inode, struct file *filp)
{
    struct bst_cv * pbst_cv;
    int ret;

    BST_CV_TRACE_PRINTK("enter");

    if (inode == NULL || filp == NULL) {
        printk(KERN_ERR "invalid file!");
        return -EFAULT;
    }
    if (filp->private_data == NULL) {
        printk(KERN_ERR "not bst_cv device!");
        return -EFAULT;
    }
    pbst_cv = container_of(filp->private_data, struct bst_cv, miscdev);

    BST_CV_STAGE_PRINTK("pbst_cv->state  = %d", pbst_cv->state );

    mutex_lock(&pbst_cv->mutex);
    if (pbst_cv->state == BST_CV_ONLINE) {
        ret = 0;
    }
    else if (pbst_cv->state == BST_CV_INIT) {
        //setup rt fw
        ret = bare_cv_fw_rt_setup(pbst_cv);
        if (ret < 0) {
            pbst_cv->state = BST_CV_ERROR;
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bare_cv_fw_rt_setup all failed");
            // bare_cv_mem_manager_exit(pbst_cv);
            BST_CV_STAGE_PRINTK("bst_cv_open bare_cv_mem_manager_exit OK");
        }
        else {
            pbst_cv->state = BST_CV_ONLINE;
            BST_CV_STAGE_PRINTK("bare_cv_fw_rt_setup OK");
        }
    }
    else {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid device state!");
        ret = -EFAULT;
    }

    if (ret == 0) {
        ret = bare_cv_mem_ctx_add(pbst_cv, filp);
    }
    mutex_unlock(&pbst_cv->mutex);

    if (ret == 0) {
        if (!try_module_get(THIS_MODULE)) {
            printk(KERN_ERR "try_module_get failed!");
            ret = -EFAULT;
        }
    }

    BST_CV_TRACE_PRINTK("exit");
    return ret;
}

/*
 * @func    bst_cv_close
 * @brief   This is the close callback function of the bst_cv Misc Device.
 * @params  inode - the inode pointer of the bst_cv misc device
 *          filp - the file pointer of the bst_cv misc device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_close(struct inode *inode, struct file *filp)
{
    struct bst_cv *pbst_cv;
    int ret;

    BST_CV_TRACE_PRINTK("enter");

    if (inode == NULL || filp == NULL) {
        printk(KERN_ERR "invalid file!");
        return -EFAULT;
    }
    if (filp->private_data == NULL) {
        printk(KERN_ERR "not bst_cv device!");
        return -EFAULT;
    }
    pbst_cv = container_of(filp->private_data, struct bst_cv, miscdev);

    mutex_lock(&pbst_cv->mutex);
    if (pbst_cv->state != BST_CV_ONLINE) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid device state!");
        ret = -EFAULT;
    }
    else {
        ret = bare_cv_mem_ctx_remove(pbst_cv, filp);
    }
    mutex_unlock(&pbst_cv->mutex);

    module_put(THIS_MODULE);
    BST_CV_TRACE_PRINTK("exit");
    return ret;
}

/*
 * @func    bst_cv_mmap
 * @brief   This is the mmap callback function of the bst_cv Misc Device.
 * @params  inode - the inode pointer of the bst_cv misc device
 *          vma - the vma pointer of the target vm area
 * @return  0 for success and error code otherwise
 */
static int bst_cv_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret = 0;

    if (filp == NULL) {
        printk(KERN_ERR "invalid file!");
        return -EFAULT;
    }
    if (vma == NULL) {
        printk(KERN_ERR "invalid vma area");
        return -EFAULT;
    }

    BST_CV_TRACE_PRINTK("enter, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx",
        vma->vm_start, vma->vm_end, vma->vm_pgoff);

    ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
        vma->vm_end - vma->vm_start, vma->vm_page_prot);

    BST_CV_TRACE_PRINTK("exit, ret: %d", ret);
    return ret;
}


/*******************************************************************************
 * bst_cv MISC Initialization
 ******************************************************************************/
static const struct file_operations bst_cv_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .unlocked_ioctl = bst_cv_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = bst_cv_ioctl,
#endif
    .mmap = bst_cv_mmap,
    .open = bst_cv_open,
    .release = bst_cv_close,
};

/*
 * @func    bst_cv_misc_init
 * @brief   This is the initialization function of the bst_cv misc device.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_miscdev_init(struct bst_cv *pbst_cv)
{
    int i;
    int ret;
    char bst_cv_dev_name[sizeof(BST_CV_DRIVER_NAME) + BST_CV_DEV_ID_LEN];

    BST_CV_TRACE_PRINTK("bst_cv struct ptr: %px", pbst_cv);
    snprintf(bst_cv_dev_name, sizeof(BST_CV_DRIVER_NAME) + BST_CV_DEV_ID_LEN,
        "%s", BST_CV_DRIVER_NAME);
    BST_CV_TRACE_PRINTK("probe device name: %s", bst_cv_dev_name);

    // init & register bst_cv miscdev
    pbst_cv->miscdev.minor = MISC_DYNAMIC_MINOR;
    pbst_cv->miscdev.fops = &bst_cv_fops;
    pbst_cv->miscdev.name = devm_kstrdup(&pbst_cv->pdev->dev, bst_cv_dev_name, GFP_KERNEL);
    pbst_cv->miscdev.nodename = devm_kstrdup(&pbst_cv->pdev->dev, bst_cv_dev_name, GFP_KERNEL);

    ret = misc_register(&pbst_cv->miscdev);
    if (ret < 0) {
        for (i=0; i<BST_CV_DSP_NUM; i++) {
            pbst_cv->dsp_online[i] = 0;
        }
    }
    return ret;
}

/*
 * @func    bst_cv_misc_exit
 * @brief   This is the exit function of the bst_cv misc device.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
void bare_cv_miscdev_exit(struct bst_cv *pbst_cv)
{
    misc_deregister(&pbst_cv->miscdev);
    BST_CV_STAGE_PRINTK("misc_deregister OK");
    return;
}

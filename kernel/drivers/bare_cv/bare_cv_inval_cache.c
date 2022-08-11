#include "bst_cv.h"
#include <linux/init.h>
#include <linux/export.h>
#include <linux/linkage.h>

int bst_cv_ioctl_inv_cache(struct bst_cv *pbst_cv,
    struct xrp_ioctl_alloc __user *palloc)
{
    struct xrp_ioctl_alloc alloc;

    BST_CV_TRACE_PRINTK("enter, user ptr: %px", palloc);

    if (copy_from_user(&alloc, palloc, sizeof(alloc))){
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed");
        return -EFAULT;
    }

    __inval_dcache_area((void *)alloc.ptr, alloc.size);
    BST_CV_TRACE_PRINTK("exit");

    return 0;
}
EXPORT_SYMBOL(bst_cv_ioctl_inv_cache);

/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Jason Yin (jason.yin@bst.ai)
 *
 * @file    bst_cv_misc.c
 * @brief   This file is the source code file of misc device interface of bst_cv
 *          driver. It contains function definitions of ioctl callbacks,
 *          initialization and exit of the misc device.
 * @note    Currently, model related calls should not be used.
 */

#include "bst_cv.h"

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

    BST_CV_GS_TRACE_PRINTK("enter bst_cv_ioctl_alloc, user ptr: %px", palloc);

    ret = copy_from_user(&alloc, palloc, sizeof(alloc));
    if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed!");
		return ret;
    }
    if (alloc.size == 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "zero size buffer!");
		return -EINVAL;
    }

    ret = bst_cv_gs_user_buffer_alloc(filp, pbst_cv, &alloc);
    ret = copy_to_user(palloc, &alloc, sizeof(alloc));
    if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_to_user failed!");
		bst_cv_gs_user_buffer_free(filp, pbst_cv, &alloc);
    }

    BST_CV_GS_TRACE_PRINTK("exit");
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

    BST_CV_GS_TRACE_PRINTK("enter bst_cv_ioctl_free, user ptr: %px", palloc);

    ret = copy_from_user(&alloc, palloc, sizeof(alloc));
    if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed!");
		return ret;
    }
    BST_CV_GS_TRACE_PRINTK("buffer addr: 0x%x", alloc.addr);

    ret = bst_cv_gs_user_buffer_free(filp, pbst_cv, &alloc);

    BST_CV_GS_TRACE_PRINTK("exit");
    return ret;
}

static int bst_cv_input_check(struct file *filp, struct bst_cv *pbst_cv, struct bst_cv_req *request)
{
    int ret;
    int phy_addr;

    BST_CV_GS_TRACE_PRINTK("enter");
    phy_addr = 0;
    request->g_param.out_channel_num = get_channel_num_by_pf(request->g_param.dst_format);
    request->g_param.in_channel_num = get_channel_num_by_pf(request->g_param.src_format);

    /*physical address check*/
    if (request->out_type == BST_CV_BUFFER_DMA) {
		phy_addr = bst_cv_get_phy_addr_by_exported(pbst_cv, request->g_param.out_dma_fd);
		if (phy_addr == 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px, fd : %d not found",
				filp, request->g_param.out_dma_fd);
			return -ENOENT;
		}
		request->g_param.mem_info.gwarp_dst_phy_addr = phy_addr;
    } else {
		ret = bst_mem_check_mem_invalid(filp, pbst_cv, request->g_param.mem_info.gwarp_dst_phy_addr);
		if (ret) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px, phy_addr : 0x%x not found",
				filp, request->g_param.mem_info.gwarp_dst_phy_addr);
			return -ENOENT;
		}
    }

    BST_CV_GS_TRACE_PRINTK("exit");
    return 0;
}

/*******************************************************************************
 * bst_cv MISC FILE INTERFACE
 ******************************************************************************/
/*
 * @func    bst_cv_ioctl_wait
 * @brief   This function sends the bst_cv message to the CV. It is a
 *          blocking function which have to wait CV returns until timeout
 * @params  filp - the pointer to the file
 *	    pbst_cv - the pointer to the bst_cv device
 *          cv_msg - the user pointer to the preallocated message cv_msg
 *          structure
 * @return  0 - success
 *          error code  - failure
 */
static int bst_cv_ioctl_wait(struct file *filp, struct bst_cv *pbst_cv, struct bst_cv_msg __user *cv_msg)
{
    int ret;
    int i;
    int format;
    __u64 user_addr;
    struct bst_cv_msg req;
    struct bst_cv_msg_manager *msg_manager;
    struct gwarp_param *g_param;
    struct resolution *dst_res;

    bst_cv_user_buffer_t user_buffer;

    // BST_CV_DEV_INFO(pbst_cv->dev, "enter");
    msg_manager = &pbst_cv->msg_manager;

    ret = copy_from_user(&req, cv_msg, sizeof(req));
    if (ret < 0) {
		BST_CV_DEV_ERR(pbst_cv->dev, "copy_from_user failed!");
		return ret;
    }

    BST_CV_GS_TRACE_PRINTK("cookie is %d", req.cookie);
    BST_CV_GS_TRACE_PRINTK("start");
    /*Check work status*/
    do {
	switch (req.mission) {
	case BST_CV_GWARP:
	case BST_CV_GWARP_SCALER:
	    /* Gwarp */
	    if (pbst_cv->gwarp_state != BST_CV_IDLE) {
			BST_CV_GS_TRACE_PRINTK("wait for prev work completion");
			//wait for work done
			ret = wait_for_completion_timeout(&msg_manager->gwarp_complete, msecs_to_jiffies(1000));
			if (!ret) {
				BST_CV_GS_TRACE_PRINTK("wait for prev work timeout");
				return -EFAULT;
			}
	    }
	    pbst_cv->gwarp_state = BST_CV_BUSY;
	    memcpy(&msg_manager->cache_msg, &req, sizeof(struct bst_cv_msg));
	    /*Input param handle*/
	    //dump for debug
	    //dump_gwarp_param(pbst_cv, msg_manager->cache_msg.req.g_param);
	    g_param = &msg_manager->cache_msg.req.g_param;
	    //check request info
	    if (bst_cv_input_check(filp, pbst_cv, &msg_manager->cache_msg.req)) {
			BST_CV_DEV_ERR(pbst_cv->dev, "Input param error");
			return -EFAULT;
	    }
	    //schedule work
	    //BST_CV_DEV_INFO(pbst_cv->dev, "cv_handler start");
	    queue_work(msg_manager->gwarp_work_queue, &pbst_cv->gwarp_work);
	    //wait for cv work done
	    ret = wait_for_completion_timeout(&msg_manager->gwarp_cv_complete, msecs_to_jiffies(1000));
	    if (ret == 0)
			BST_CV_DEV_ERR(pbst_cv->dev, "cv gwarp work timeout!");

	    //output user addr
	    format = msg_manager->cache_msg.req.g_param.dst_format;
	    dst_res = &msg_manager->cache_msg.req.g_param.dst_res;

	    ret = bst_get_user_addr_by_bus(filp, pbst_cv,
					   msg_manager->cache_msg.req.g_param.mem_info.gwarp_dst_phy_addr,
					   &user_addr);
	    if (!ret) {
			user_buffer.uaddr = (char *)user_addr;
			user_buffer.byte_used = 0;
			for (i = 0; i < msg_manager->cache_msg.req.g_param.out_channel_num; i++) {
				user_buffer.byte_used +=
				get_channel_size(format, i, dst_res->height * dst_res->width);
			}
			user_buffer.ts_usec = ktime_get_ns()/1000;
			BST_CV_GS_TRACE_PRINTK("byte_used: %d", user_buffer.byte_used);
		} else {
			BST_CV_DEV_ERR(pbst_cv->dev, "Failed to find user ptr by phy");
	    }
	    //return ACK cookie
	    ret = copy_to_user(&cv_msg->rsp.cookie, &msg_manager->cache_msg.cookie, sizeof(cv_msg->rsp.cookie));
	    ret = copy_to_user(&cv_msg->rsp.cv_buffer, &user_buffer, sizeof(user_buffer));
	    pbst_cv->gwarp_state = BST_CV_IDLE;
	    //wakeup another process
	    complete(&msg_manager->gwarp_complete);
	    break;
	case BST_CV_SCALER:
	case BST_CV_SCALER_GWARP:
	    /* Scaler */
	    BST_CV_GS_TRACE_PRINTK("BST_CV_SCALER");
	    break;
	default:
	    break;
	}
    } while (0);

    // BST_CV_DEV_INFO(pbst_cv->dev, "exit");
    return ret;
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

    BST_CV_GS_TRACE_PRINTK("%s, cmd: 0x%x", "enter", cmd);

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

    switch (cmd) {
    case CV_IOCTL_ALLOC:
		ret = bst_cv_ioctl_alloc(filp, pbst_cv, (void __user *)args);
		break;
    case CV_IOCTL_FREE:
		ret = bst_cv_ioctl_free(filp, pbst_cv, (void __user *)args);
		break;
    case CV_IOCTL_WAIT:
		ret = bst_cv_ioctl_wait(filp, pbst_cv, (void __user *)args);
		break;
    case BST_CV_IOCTL_DMA_BUF_IMPORT:
		ret = bst_cv_ioctl_dma_buf_import(pbst_cv, (void __user *)args);
		break;
    case BST_CV_IOCTL_DMA_BUF_EXPORT:
		ret = bst_cv_ioctl_dma_buf_export(filp, pbst_cv, (void __user *)args);
		break;
    case BST_CV_IOCTL_DMA_BUF_FREE:
		ret = bst_cv_ioctl_dma_buf_return(pbst_cv, (void __user *)args);
		break;
    default:
		ret = -EINVAL;
	break;
    }

    module_put(THIS_MODULE);
    BST_CV_GS_TRACE_PRINTK("%s, ret: %d", "exit", ret);
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
    struct bst_cv *pbst_cv;
    int ret = 0;

    BST_CV_GS_TRACE_PRINTK("enter");

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
    if (ret == 0)
		ret = bst_cv_gs_mem_ctx_add(pbst_cv, filp);
    mutex_unlock(&pbst_cv->mutex);

    if (ret == 0) {
		if (!try_module_get(THIS_MODULE)) {
			printk(KERN_ERR "try_module_get failed!");
			ret = -EFAULT;
		}
    }

    BST_CV_GS_TRACE_PRINTK("exit");
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

    BST_CV_GS_TRACE_PRINTK("enter");

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
    if (pbst_cv->gwarp_state != BST_CV_IDLE) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid device state!");
		ret = -EFAULT;
    } else {
		ret = bst_cv_gs_mem_ctx_remove(pbst_cv, filp);
    }
    mutex_unlock(&pbst_cv->mutex);

    module_put(THIS_MODULE);
    BST_CV_GS_TRACE_PRINTK("exit");
    return ret;
}

/*
 * @func    bst_cv_mmap
 * @brief   This is the open callback function of the bst_cv Misc Device.
 * @params
 *          filp - the file pointer of the bst_cv misc device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct bst_cv *pbst_cv;
    int ret;

    BST_CV_GS_TRACE_PRINTK("enter");

    if (filp->private_data == NULL) {
		printk(KERN_ERR "not bst_cv device!");
		return -EFAULT;
    }

    pbst_cv = container_of(filp->private_data, struct bst_cv, miscdev);
    if (pbst_cv == NULL)

		BST_CV_DEV_ERR(pbst_cv->dev, "pbst_cv is NULL!");


    BST_CV_GS_TRACE_PRINTK("enter, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx",
			vma->vm_start, vma->vm_end, vma->vm_pgoff);

    /*cpu wait for write done*/
    vma->vm_page_prot = pgprot_writethrough(vma->vm_page_prot);

    ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			  vma->vm_end - vma->vm_start, vma->vm_page_prot);

    BST_CV_GS_TRACE_PRINTK("exit, ret: %d", ret);
    return 0;
}

/*******************************************************************************
 * bst_cv MISC Initialization
 ******************************************************************************/
static const struct file_operations bst_cv_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = bst_cv_ioctl,
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
int bst_cv_gwarp_miscdev_init(struct bst_cv *pbst_cv)
{
    int ret;
    // init & register bst_cv miscdev
    pbst_cv->miscdev.minor = MISC_DYNAMIC_MINOR;
    pbst_cv->miscdev.fops = &bst_cv_fops;
    pbst_cv->miscdev.name = devm_kstrdup(pbst_cv->dev, BST_CV_DRIVER_NAME, GFP_KERNEL);
    pbst_cv->miscdev.nodename = devm_kstrdup(pbst_cv->dev, BST_CV_DRIVER_NAME, GFP_KERNEL);

    ret = misc_register(&pbst_cv->miscdev);
    if (ret) {
		BST_CV_DEV_ERR(pbst_cv->dev, "cannot register misc device.\n");
		return -EFAULT;
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
void bst_cv_gwarp_miscdev_exit(struct bst_cv *pbst_cv)
{
    misc_deregister(&pbst_cv->miscdev);
    BST_CV_GS_TRACE_PRINTK("misc_deregister OK");
    return;
}

/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Jason Yin (jason.yin@bst.ai)
 *
 * @file    bst_cv.h
 * @brief   This file is the top header file of the bst_cv driver. It contains
 * function definitions of driver setup and interface.
 */
#ifndef BST_CV_GS_H
#define BST_CV_GS_H

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/ipc_interface.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/dma-direct.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/hashtable.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>

#define BST_CV_DRIVER_NAME          "bst_gwarp_scaler"
//device id cannot be greater or equal to 10
#define BST_CV_DEV_ID_LEN  			1

struct bst_cv;
struct gwarp_param;

#include "bst_cv_gwarp.h"
#include "bst_cv_interface.h"
#include "bst_cv_sg_mem_manager.h"
#include "bst_cv_gwarp_miscdev.h"

#define BST_CV_DEBUG_PRINT          2
#define BST_CV_LOG_PRINT            1
#define BST_CV_NO_PRINT             0

#define BST_CV_GS_TRACE_PRINTK(format, ...)       do { \
    if (bst_cv_gs_print_level >= BST_CV_LOG_PRINT) \
		printk(KERN_INFO "[%s]: %s %d: " format "\n", BST_CV_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BST_CV_DEV_ERR(dev, format, ...)       dev_err(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define BST_CV_DEV_INFO(dev, format, ...)      dev_info(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)


enum work_status {
	BST_CV_BUSY = 0,
	BST_CV_INIT,
	BST_CV_IDLE,
	BST_CV_ERROR
};

struct bst_cv_msg_manager {
	struct bst_cv_msg cache_msg;
	struct workqueue_struct *gwarp_work_queue;
	struct workqueue_struct *scaler_work_queue;
	struct mutex gwarp_lock;
	struct mutex scaler_lock;
	struct completion gwarp_complete;
	struct completion gwarp_cv_complete;
	struct completion scaler_complete;
};

struct bst_cv_irq_manager {
	int irq;
	void __iomem *gwarp_irq_reg;
	void __iomem *scaler_irq_reg;
	unsigned long gwarp_irq_flag;
	unsigned long scaler_irq_flag;
	struct completion gwarp_irq_complete;
	struct completion scaler_irq_complete;
};

struct bst_cv {
	enum work_status gwarp_state;   	//device state
	enum work_status scaler_state;	 	//device state
	struct platform_device *pdev;
	struct mutex mutex; //mutex to protect the driver state
	struct bst_cv_msg_manager msg_manager;
	struct bst_cv_irq_manager irq_manager;
	struct bst_cv_mem_manager mem_manager;
	struct work_struct gwarp_work;
	struct work_struct scaler_work;
	struct miscdevice miscdev;
	struct kobject kobj;
	struct device *dev;
	struct gwarp_regs *gwarp_regs;
};

extern int bst_cv_gs_print_level;

#endif //BST_CV_GS_H


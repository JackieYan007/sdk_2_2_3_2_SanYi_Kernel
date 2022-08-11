/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_cv.h
 * @brief   This file is the top header file of the bst_cv driver. It contains
 *          function definitions of driver setup and interface. 
 */
#ifndef BST_CV_H
#define BST_CV_H

#include <linux/module.h>
#include <linux/slab.h>
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
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>

#define BST_CV_DRIVER_NAME          "bare_cv"
#define BST_CV_DSP_NUM              4
//device id cannot be greater or equal to 10
#define BST_CV_DEV_ID_LEN           1

struct bst_cv;

#include "bst_cv_interface.h"
#include "bst_cv_mem_manager.h"
// #include "bst_cv_msg_manager.h"
#include "bst_cv_fw_manager.h"
#include "bst_cv_sysfile.h"
#include "bst_cv_miscdev.h"

#define BST_CV_DEBUG_PRINT          2
#define BST_CV_LOG_PRINT            1
#define BST_CV_NO_PRINT             0

#define COLOR_BLUE      "\033[34m"
#define COLOR_RED       "\033[31m"
#define COLOR_PURPLE    "\033[35m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"
#define COLOR_RECOVER   "\033[0m"

#define BST_CV_TRACE_PRINTK(format,...)       do { \
    if (bare_cv_print_level >= BST_CV_DEBUG_PRINT) \
        printk(KERN_DEBUG "[%s]: %s %d: " format "\n", BST_CV_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BST_CV_STAGE_PRINTK(format,...)       do { \
    if (bare_cv_print_level >= BST_CV_LOG_PRINT) \
        printk(KERN_INFO COLOR_PURPLE"[%s]: %s %d: " format "\n"COLOR_RECOVER, BST_CV_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BST_CV_DEV_ERR(dev, format,...)       dev_err(dev, COLOR_RED"%s %d: " format "\n"COLOR_RECOVER, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define BST_CV_DEV_INFO(dev, format,...)      dev_info(dev, COLOR_GREEN"%s %d: " format "\n"COLOR_RECOVER, __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define BST_CV_DSP_STATE_OFFLINE    0
#define BST_CV_DSP_STATE_INIT       1
#define BST_CV_DSP_STATE_ONLINE     2
#define BST_CV_DSP_STATE_ERROR      3

struct bst_cv {
    struct platform_device *pdev;
    struct mutex mutex; //mutex to protect the driver state
    enum {
        BST_CV_OFFLINE = 0,
        BST_CV_INIT,
        BST_CV_ONLINE,
        BST_CV_ERROR
    } state;   //device state
    uint8_t dsp_online[BST_CV_DSP_NUM];

    struct bst_cv_mem_manager mem_manager;
    struct bst_cv_fw_manager fw_manager;
    struct miscdevice miscdev;
    struct kobject kobj;
};

// online at least on DSP, return true
static inline bool bare_cv_check_online(struct bst_cv *pbst_cv) {
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            return true;
        }
    }
    return false;
}

extern int bare_cv_print_level;

#endif


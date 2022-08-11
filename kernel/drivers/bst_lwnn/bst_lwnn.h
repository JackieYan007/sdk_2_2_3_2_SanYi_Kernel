/*!
 * bst_lwnn:    Linux device driver for Blck Sesame Technologies Light Weighted
 *              Neural Network Processor
 * @author      Shichen Lin (shichen.lin@bst.ai)
 *
 * @file        bst_lwnn.h
 * @brief       This file is the top header file of the bst_lwnn driver. It
 *              includes other needed header files and defines the driver
 *              metadata structure definition. It also includes definitions or
 *              declarations of kernel driver interface functions, global
 *              constants and variables as well as helper macros. 
 */
#ifndef BST_LWNN_H
#define BST_LWNN_H

#include <linux/mman.h>
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
#include <linux/dma-buf.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>

#define BST_LWNN_DRIVER_NAME            "bst_lwnn"

#define BST_LWNN_VER_MAJOR              0
#define BST_LWNN_VER_MINOR              4
#define BST_LWNN_VER_PATCH              0
#define BST_LWNN_RELEASE_MONTH          2
#define BST_LWNN_RELEASE_DATE           15
#define BST_LWNN_RELEASE_YEAR           2022

#define BST_LWNN_FW_VER_MAJOR           0
#define BST_LWNN_FW_VER_MINOR           4

struct bst_lwnn;

#include <uapi/linux/lwnn_shared_interface.h>
#include <uapi/linux/bst_lwnn_uapi.h>
#include <uapi/linux/sched/types.h>
#include "lwnn_rt_fw_kapi.h"
#include "bst_lwnn_mem_manager.h"
#include "bst_lwnn_msg_manager.h"
#include "bst_lwnn_fw_manager.h"
#include "bst_lwnn_sysfile.h"
#include "bst_lwnn_miscdev.h"

#define BST_LWNN_DEBUG_PRINT          2
#define BST_LWNN_LOG_PRINT            1
#define BST_LWNN_NO_PRINT             0

#define BST_LWNN_TRACE_PRINTK(format,...)       do { \
    if (bst_lwnn_print_level >= BST_LWNN_DEBUG_PRINT) \
        printk(KERN_INFO "[%s]: %s %d: " format "\n", BST_LWNN_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BST_LWNN_STAGE_PRINTK(format,...)       do { \
    if (bst_lwnn_print_level >= BST_LWNN_LOG_PRINT) \
        printk(KERN_INFO "[%s]: %s %d: " format "\n", BST_LWNN_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BST_LWNN_DEV_ERR(dev, format,...)       dev_err(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define BST_LWNN_DEV_INFO(dev, format,...)      dev_info(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define BST_LWNN_DSP_STATE_OFFLINE    0
#define BST_LWNN_DSP_STATE_INIT       1
#define BST_LWNN_DSP_STATE_ONLINE     2
#define BST_LWNN_DSP_STATE_ERROR      3

struct bst_lwnn {
    struct platform_device *pdev;
    int32_t dsp_num;
    struct mutex mutex; //mutex to protect state
    enum {
        BST_LWNN_OFFLINE = 0,
        BST_LWNN_INIT,
        BST_LWNN_ONLINE,
        BST_LWNN_ERROR
    } state;   //device state

    /*
        We do not dynamically allocate space for DSP specific metadata for two
        reasons. First, it is not a big waste here to directly allocate maximum
        possible space and it saves some coding efforts. Second, the dynamically
        allocated memory is not spatially local to the other fields in our
        driver structures. It should be noticed that the index used to access
        DSP specific metadata array like dsp_online field here or those in
        substructures is not the same as the CV DSP index inside the device
        tree. The CV DSP index is the index of the DSP among four CV DSPs while
        the array index here is only the index among used CV DSPs (probably 1 or
        2).
    */

    uint8_t dsp_online[BST_LWNN_MAX_DSP_NUM];
    uint32_t dsp_indices[BST_LWNN_MAX_DSP_NUM];

    struct bst_lwnn_mem_manager mem_manager;
    struct bst_lwnn_msg_manager msg_manager;
    struct bst_lwnn_fw_manager fw_manager;
    struct miscdevice miscdev;
    struct kobject kobj;
};

static inline bool bst_lwnn_check_online(struct bst_lwnn *pbst_lwnn) {
    int i;

    for (i=0; i<pbst_lwnn->dsp_num; i++) {
        if (pbst_lwnn->dsp_online[i]) {
            return true;
        }
    }
    return false;
}

extern int bst_lwnn_print_level;

#endif


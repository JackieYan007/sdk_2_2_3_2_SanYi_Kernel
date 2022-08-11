/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bstn.h
 * @brief   This file is the top header file of BSTN driver. It contains
 *          function definitions of driver setup and interface. 
 */

#ifndef BSTN_H
#define BSTN_H

#define IPC

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/of_reserved_mem.h>
#include <linux/rcupdate.h>
#include <linux/dma-direct.h>
#include <linux/dma-buf.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/hashtable.h>
#ifdef IPC
#include <linux/ipc_interface.h>
#endif

#include <uapi/linux/sched/types.h>

#include <asm/mman.h>
#include <asm/cacheflush.h>

#include "variable_queue.h"
#include "bstn_user.h"

#define BSTN_DRIVER_NAME                    "bstn"

#define BSTN_DEBUG_PRINT                    2
#define BSTN_LOG_PRINT                      1
#define BSTN_NO_PRINT                       0

#define BSTN_TRACE_PRINTK(format,...)       do { \
    if (bstn_print_level >= BSTN_DEBUG_PRINT) \
        printk(KERN_DEBUG "[%s]: %s %d: " format "\n", BSTN_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BSTN_STAGE_PRINTK(format,...)       do { \
    if (bstn_print_level >= BSTN_LOG_PRINT) \
        printk(KERN_INFO "[%s]: %s %d: " format "\n", BSTN_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define BSTN_DEV_ERR(dev, format,...)       dev_err(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define BSTN_DEV_INFO(dev, format,...)      dev_info(dev, "%s %d: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

struct bstn_device;

#include "bstn_mem_manager.h"
#ifndef IPC
#include "bstn_channel_manager.h"
#include "bstn_netreg.h"
#endif
#include "bstn_msg_manager.h"
#include "bstn_fw_manager.h"
#include "bstn_sysfile.h"
#include "bstn_misc.h"

#define BSTN_VER_MAJOR                      2
#define BSTN_VER_MINOR                      5
#define BSTN_VER_PATCH                      3
#define BSTN_RELEASE_MONTH                  6
#define BSTN_RELEASE_DATE                   7
#define BSTN_RELEASE_YEAR                   2022

extern int bstn_print_level;

/******************************************************************************/

struct bstn_device {
    struct platform_device *pdev;

    struct mutex mutex; //mutex to protect rc and state
    uint32_t rc; //device reference count, +1 when open and -1 when close
    uint32_t id; //self-id, /dev/bstn(id)
    enum {
        BSTN_OFFLINE = 0,
        BSTN_INIT,
        BSTN_ONLINE,
        BSTN_ERROR
    } state;   //device state
    
    struct miscdevice miscdev;
    struct kobject kobj;
    struct bstn_fw_manager fw_manager;
    struct bstn_msg_manager msg_manager;
    struct bstn_mem_manager mem_manager;
};

#endif

/*!
 * BST_LWNN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * @author Shichen Lin (shichen.lin@bst.ai)
 *
 * @file 	bst_lwnn_sysfile.h
 * @brief This file is the header file of sysfs file interface of BST_LWNN
 *			  driver. It contains the declarations of misc device initialization
 *			  and cleanup functions.
 */

#ifndef BST_LWNN_SYSFILE_H
#define BST_LWNN_SYSFILE_H

#include <linux/kobject.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>

int bst_lwnn_sysfile_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_sysfile_exit(struct bst_lwnn *pbst_lwnn);

#endif /* BST_LWNN_SYSFILE_H */
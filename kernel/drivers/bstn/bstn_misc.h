/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file 	bstn_misc.h
 * @brief 	This file is the header file of misc device interface of BSTN
 *			driver. It contains the declarations of misc device initialization
 *			and cleanup functions.
 */

#ifndef BSTN_MISC_H
#define BSTN_MISC_H

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <bstn_netreg.h>

int bstn_misc_init(struct bstn_device *pbstn);
int bstn_misc_exit(struct bstn_device *pbstn);

#endif

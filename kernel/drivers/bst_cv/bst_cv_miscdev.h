/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_cv_misc.h
 * @brief   This file is the header file of misc device interface of bst_cv
 *          driver. It contains the declarations of misc device initialization
 *          and exit functions.
 */

#ifndef BST_CV_MISCDEV_H
#define BST_CV_MISCDEV_H

int bst_cv_miscdev_init(struct bst_cv *pbst_cv);
void bst_cv_miscdev_exit(struct bst_cv *pbst_cv);

#endif
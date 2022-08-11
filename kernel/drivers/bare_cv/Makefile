# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux CV DSP drivers.

# loadable module
# export symbol swiotlb_dma_ops in dma-mapping.c

ifneq ($(KERNELRELEASE),)

obj-$(CONFIG_BARE_CV) += bare_cv.o
bare_cv-objs = bst_cv_miscdev.o bst_cv_fw_manager.o bst_cv_mem_manager.o  \
	 bst_cv_sysfile.o bst_cv.o #fw_file.o

else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../..
OUT_DIR=$(KDIR)/build_out

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

endif
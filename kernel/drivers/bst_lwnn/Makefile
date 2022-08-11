# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Bluetooth HCI device drivers.

# loadable module
# export symbol swiotlb_dma_ops in dma-mapping.c

ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_BST_LWNN) += bst_lwnn_driver.o
bst_lwnn_driver-y = bst_lwnn_fw_manager.o bst_lwnn_mem_manager.o bst_lwnn_miscdev.o \
	bst_lwnn_msg_manager.o bst_lwnn_sysfile.o bst_lwnn.o
else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../..
OUT_DIR=$(KDIR)/build

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

endif

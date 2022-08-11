# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Bluetooth HCI device drivers.

# loadable module
# export symbol swiotlb_dma_ops in dma-mapping.c

ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_BSTN) += bstn_driver.o
bstn_driver-y = bstn_sysfile.o bstn.o \
	bstn_fw_manager.o bstn_mem_manager.o \
	bstn_misc.o bstn_msg_manager.o
else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../..
OUT_DIR=$(KDIR)/build_out

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

endif

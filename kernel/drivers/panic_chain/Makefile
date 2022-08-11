obj-$(CONFIG_BST_ERROR_NOTIFIER)		+= panic_chain.o bst_error_handle.o
obj-$(CONFIG_BST_ERROR_TEST):=panic_module.o

#kernel源代码路
KERNELDIR:=$(shell pwd)/../

default:
	$(MAKE) -C $(KERNELDIR) M=`pwd` modules ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

clean:
	rm -rf *.o *.ko *.mod.c *.symvers  modul* .*.cmd
# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ION_BST), y)
obj-$(CONFIG_ION_BST)	+= bst/ion.o
obj-$(CONFIG_ION_BST)	+= bst/ion_buffer.o
else
obj-$(CONFIG_ION)	+= ion.o ion_buffer.o
endif
obj-$(CONFIG_ION)	+= ion_dma_buf.o ion_heap.o
CFLAGS_ion_buffer.o 	= -I$(src)
obj-y			+= heaps/

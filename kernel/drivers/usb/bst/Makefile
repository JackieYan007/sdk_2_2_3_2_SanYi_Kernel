# SPDX-License-Identifier: GPL-2.0
# define_trace.h needs to know how to find our header
CFLAGS_trace.o				:= -I$(src)

obj-$(CONFIG_USB_BST_DWC3)			+= dwc3.o

dwc3-y					:= core.o

ifneq ($(CONFIG_TRACING),)
	dwc3-y				+= trace.o
endif

ifneq ($(filter y,$(CONFIG_USB_BST_DWC3_HOST) $(CONFIG_USB_BST_DWC3_DUAL_ROLE)),)
	dwc3-y				+= host.o
endif

ifneq ($(filter y,$(CONFIG_USB_BST_DWC3_GADGET) $(CONFIG_USB_BST_DWC3_DUAL_ROLE)),)
	dwc3-y				+= gadget.o ep0.o
endif

ifneq ($(CONFIG_USB_BST_DWC3_DUAL_ROLE),)
	dwc3-y				+= drd.o
endif

ifneq ($(CONFIG_USB_BST_DWC3_ULPI),)
	dwc3-y				+= ulpi.o
endif

ifneq ($(CONFIG_DEBUG_FS),)
	dwc3-y				+= debugfs.o
endif

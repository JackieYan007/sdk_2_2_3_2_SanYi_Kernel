# SPDX-License-Identifier: GPL-2.0
uvcvideo-objs  := uvc_queue.o uvc_v4l2.o uvc_video.o uvc_ctrl.o \
		  uvc_status.o uvc_isight.o uvc_debugfs.o uvc_metadata.o

ifeq ($(CONFIG_ARCH_BST), y)
uvcvideo-objs  += bst/uvc_driver.o
else
uvcvideo-objs  += uvc_driver.o
endif


ifeq ($(CONFIG_MEDIA_CONTROLLER),y)
uvcvideo-objs  += uvc_entity.o
endif
obj-$(CONFIG_USB_VIDEO_CLASS) += uvcvideo.o


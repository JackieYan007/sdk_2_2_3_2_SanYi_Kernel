#
#
# (C) COPYRIGHT ARM Limited. All rights reserved.
#
# This program is free software and is provided to you under the terms of the
# GNU General Public License version 2 as published by the Free Software
# Foundation, and any use by you of this program is subject to the terms
# of such GNU licence.
#
# A copy of the licence is included with the program, and can also be obtained
# from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
# Boston, MA  02110-1301, USA.
#
#

ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_MALI_VPU) += base/
obj-$(CONFIG_MALI_VPU) += resource/  
obj-$(CONFIG_DMA_SHARED_BUFFER_TEST_EXPORTER) += dma_buf_test_exporter/
else

CROSS_COMPILE ?= arm-eabi-
DEBUG ?= 0

all: android

android:
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) OS=android DEBUG=$(DEBUG) HW=1

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) HW=1 clean

.PHONY: all android

endif

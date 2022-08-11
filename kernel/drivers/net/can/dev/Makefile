# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_CAN_DEV)		+= can-dev.o
ifeq ($(CONFIG_ARCH_BST),y)
can-dev-y			+= ../bst/dev.o
else
can-dev-y			+= dev.o
endif
can-dev-y			+= rx-offload.o

can-dev-$(CONFIG_CAN_LEDS)	+= led.o

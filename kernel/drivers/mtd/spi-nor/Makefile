# SPDX-License-Identifier: GPL-2.0

spi-nor-objs			:= sfdp.o

ifeq ($(CONFIG_ARCH_BST), y)
spi-nor-objs			+= bst/core.o bst/gigadevice.o bst/micron-st.o bst/winbond.o bst/issi.o
else
spi-nor-objs			+= core.o gigadevice.o micron-st.o winbond.o issi.o
endif

spi-nor-objs			+= atmel.o
spi-nor-objs			+= catalyst.o
spi-nor-objs			+= eon.o
spi-nor-objs			+= esmt.o
spi-nor-objs			+= everspin.o
spi-nor-objs			+= fujitsu.o
spi-nor-objs			+= intel.o
spi-nor-objs			+= macronix.o
spi-nor-objs			+= spansion.o
spi-nor-objs			+= sst.o
spi-nor-objs			+= xilinx.o
spi-nor-objs			+= xmc.o
obj-$(CONFIG_MTD_SPI_NOR)	+= spi-nor.o

obj-$(CONFIG_MTD_SPI_NOR)	+= controllers/

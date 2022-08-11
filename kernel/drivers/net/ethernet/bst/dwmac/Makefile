# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_BST_DWMAC_ETH) += bstmac.o
bstmac-objs:= dwmac_main.o dwmac_ethtool.o dwmac_mdio.o ring_mode.o	\
	      chain_mode.o dwmac_lib.o enh_desc.o norm_desc.o	\
	      mmc_core.o dwmac_hwtstamp.o dwmac_ptp.o dwmac4_descs.o	\
	      dwmac4_dma.o dwmac4_lib.o dwmac4_core.o dwmac5.o hwif.o \
	      dwmac_tc.o \
	      $(bstmac-y)

# Ordering matters. Generic driver must be last.

obj-$(CONFIG_BST_GMAC_PLATFORM)	+= bst-gmac-platform.o
bst-gmac-platform-objs:= of_bstgmac.o dwmac_platform.o

KDIR:/lib/modules/'uname -r'/build


default:
	$(MAKE) -C $(KDIR) M='pwd' modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions *.order *sysvers *Module.markers


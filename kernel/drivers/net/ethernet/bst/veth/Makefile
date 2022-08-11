obj-$(CONFIG_BST_PCIE_VETH)	+= bst_pcie_veth.o
bst_pcie_veth-objs:=pcie_vnet_main.o pcie_vnet_plat.o of_pcievnet.o pcie_vnet_ethtool.o

KDIR:/lib/modules/'uname -r'/build


default:
	$(MAKE) -C $(KDIR) M='pwd' modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions *.order *sysvers *Module.markers
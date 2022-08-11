# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_PCIE_BST) += bst_pcie.o
bst_pcie-objs = pcie-bst.o pcie-bst-phy.o pcie-bst-plat.o pcie-bst-host.o pcie-bst-ep.o pcie-bst-dma.o
#obj-$(CONFIG_PCIE_BST_PLAT) += pcie-bst-plat.o
#obj-$(CONFIG_PCIE_BST_HOST) += pcie-bst-host.o
#obj-$(CONFIG_PCIE_BST_EP) += pcie-bst-ep.o
#obj-$(CONFIG_PCI_EPDMA) += bst_hal_dma.o bst_dma.o

# The following drivers are for devices that use the generic ACPI
# pci_root.c driver but don't support standard ECAM config access.
# They contain MCFG quirks to replace the generic ECAM accessors with
# device-specific ones that are shared with the DT driver.

# The ACPI driver is generic and should not require driver-specific
# config options to be enabled, so we always build these drivers on
# ARM64 and use internal ifdefs to only build the pieces we need
# depending on whether ACPI, the DT driver, or both are enabled.



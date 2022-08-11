#ifndef _BST_PCIE_DRV_H
#define _BST_PCIE_DRV_H

enum dw_pcie_device_mode {
	DW_PCIE_UNKNOWN_TYPE,
	DW_PCIE_EP_TYPE,
	DW_PCIE_LEG_EP_TYPE,
	DW_PCIE_RC_TYPE,
};

extern int bst_pcie_link_is_ok(void);
extern int bst_pcie_get_role(void);
extern int pcie_get_hw_mode(void);
extern void bst_pcie_write_outmem(u32 offset, u64 val);
extern void bst_pcie_read_outmem(u32 offset, u64 *val);
extern int ep_pcie_dma_read(u32 dma_ch_num, u32 dma_trans_size, u64 dma_read_sa_addr, u64 dma_read_da_addr);
extern int ep_pcie_dma_write(u32 dma_ch_num, u32 dma_trans_size, u64 dma_write_sa_addr, u64 dma_write_da_addr);

#endif //_BST_PCIE_DRV_H
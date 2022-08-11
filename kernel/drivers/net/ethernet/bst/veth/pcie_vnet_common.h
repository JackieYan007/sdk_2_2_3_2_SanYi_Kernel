#ifndef __COMMON_H__
#define __COMMON_H__

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/bitops.h>

#define JUMBO_LEN   9000

#define DMA_TX_SIZE 256
#define DMA_RX_SIZE 256

#define PCIE_VNET_GET_ENTRY(x, size)	((x + 1) & (size - 1))

/* Tx coalesce parameters */
#define BST_PCIEVNET_COAL_TX_TIMER	1000
#define BST_PCIEVNET_TX_MAX_FRAMES  128
#define BST_PCIEVNET_TX_FRAMES	    16

#define PCIE_VNET_TXCHAN_NUM        1
#define PCIE_VNET_RXCHAN_NUM        1
#define PCIE_VNET_CORE_NUM          2
#define PCIE_VNET_RXMEM_THRE        (DMA_RX_SIZE)
#define PCIE_VNET_RXMEM_MAX         (PCIE_VNET_RXMEM_THRE*2) /* must *2 */
#define PCIE_VNET_RXMEM_MASK        (PCIE_VNET_RXMEM_MAX-1)

#define PCIE_VNET_RX_POLL_WEIGHT NAPI_POLL_WEIGHT
#define PCIE_VENT_TX_POLL_WEIGHT NAPI_POLL_WEIGHT

#define PCIE_VNET_DMA_CHAN0 0
#define PCIE_VNET_DMA_CHAN1 1

/* GMAC TX FIFO is 8K, Rx FIFO is 16K */
#define BUF_SIZE_16KiB 11264
/* RX Buffer size must be < 8191 and multiple of 4/8/16 bytes */
#define BUF_SIZE_8KiB 8188
#define BUF_SIZE_4KiB 4096
#define BUF_SIZE_2KiB 2048

/* Extra statistic and debug information exposed by ethtool */
struct pcie_vnet_extra_stats {
    /* statu error */
	unsigned long link_not_ok ____cacheline_aligned;
	unsigned long netif_not_ok;
	unsigned long carrier_not_ok;
    unsigned long intf_poll;

    /* rx errors */
	unsigned long rx_alloc_skb_fail;
	unsigned long rx_dma_map_fail;
    unsigned long rxproc_poll;
    unsigned long rxpicp_cnt;
    unsigned long rxstack_cnt;
    unsigned long rx_read_error;
    unsigned long rx_mem_poll;

    /* Transmit errors */
	unsigned long xmit_cnt;
    unsigned long txproc_poll;
    unsigned long tx_clean;
    unsigned long tx_alloc_skb_fail;
	unsigned long tx_dma_map_fail;
    unsigned long tx_busy;
	
};

extern void bst_pcievnet_set_ethtool_ops(struct net_device *netdev);
#endif
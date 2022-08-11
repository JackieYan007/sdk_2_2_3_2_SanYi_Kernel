#ifndef __PCIEVNET_H__
#define __PCIEVNET_H__

#include "pcie_vnet_common.h"

#define BST_PCIEVNET_NAME   "bst-pcievnet"

struct pcievnet_laddr_ctrl {
    union ctrl_u {
        u64 ctrl;
        struct ctrl_s {
            // u16 cur_rx;
            u32 dirty_rx;
            u16 cur_tx;
            u16 dirty_tx;
        } s;
   } u;
};

struct pcievnet_laddr_info {
    union map {
        u64 flag;
        struct info_s {
            u64 magic : 4;
            u64 len : 20;
            u64 buf : 40;
        } s;
    } u;
};

struct pcie_vnet_tx_info {
	dma_addr_t buf;
	bool map_as_page;
	unsigned len;
	bool last_segment;
	bool is_jumbo;
};

/* Frequently used values are kept adjacent for cache effect */
struct pcie_vnet_tx_queue {
    u32 clean_run;
    spinlock_t que_lock ____cacheline_aligned_in_smp;
	u32 tx_count_frames;
	struct timer_list txtimer;
	u32 queue_index;
	struct pcie_vnet_priv *priv_data;
	struct sk_buff **tx_skbuff;
	struct pcie_vnet_tx_info *tx_skbuff_dma;
	unsigned int cur_tx;
	unsigned int dirty_tx;
   // void *laddr_ctrl_va;
    void *laddr_info_va;
};

struct pcie_vnet_rx_queue {
	u16 run;
    u16 queue_index;
	struct pcie_vnet_priv *priv_data;
    spinlock_t que_lock ____cacheline_aligned_in_smp;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	u32 rx_zeroc_thresh;
   // void *laddr_ctrl_va;
    void *laddr_info_va;
};

struct pcie_vnet_channel {
	struct pcie_vnet_priv *priv_data;
	u32 index;
    struct work_struct tx_work;
};

struct pcie_vnet_plat {
    int id;
    int board_type;
    int dma_chan_num;
    int tx_fifo_size;
    int rx_fifo_size;
	int maxmtu;
    int rx_queues_to_use;
    int tx_queues_to_use;
    u32 ctrl_mem_base;
    u32 ctrl_mem_size;
};

struct pcie_vnet_priv {
    /* Frequently used values are kept adjacent for cache effect */
	u32 tx_coal_frames;
    u32 tx_coal_timer;
    unsigned int dma_buf_sz;
    struct net_device       *dev;
	struct device           *device;
    struct pcie_vnet_plat   *plat;
    int id;
    int chan_num;
    int tx_fifo_size;
    int rx_fifo_size;
	int maxmtu;
    int extend_op;
    int rx_queues_to_use;
    int tx_queues_to_use;
    u32 msg_enable;
    u32 ctrl_mem_base;
    u32 ctrl_mem_size;

    struct mutex lock;

	/* RX Queue */
	struct pcie_vnet_rx_queue rx_queue[PCIE_VNET_RXCHAN_NUM];
	/* TX Queue */
	struct pcie_vnet_tx_queue tx_queue[PCIE_VNET_TXCHAN_NUM];

	/* Generic channel for NAPI */
    struct pcie_vnet_channel rx_channel[PCIE_VNET_RXCHAN_NUM];
    struct pcie_vnet_channel tx_channel[PCIE_VNET_TXCHAN_NUM];

    struct pcie_vnet_extra_stats xstats ____cacheline_aligned_in_smp;

    unsigned long state;
    struct workqueue_struct *wq;
    struct workqueue_struct *rxmem;
    struct workqueue_struct *tx_wq;
    struct work_struct rxmem_work;
	struct work_struct service_task;
    struct work_struct intf_task;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_rings_status;
	struct dentry *dbgfs_dma_cap;
#endif

};

struct pcievnet_trans {
    u32 len;
    dma_addr_t buf;
    int queue;
};

enum pcie_vnet_state {
	PCIE_VNET_DOWN = 0,
	PCIE_VNET_RESET_REQUESTED,
	PCIE_VNET_RESETING,
	PCIE_VNET_SERVICE_SCHED,
    PCIE_VNET_UP,
    PCIE_VNET_RXMEM_WORK_RUN,

    PCIE_VNET_STATE_MAX = 63
};

struct task_handle {
    struct pcie_vnet_priv *priv;
	struct task_struct *tsk;
    int id;
};

extern void print_hex_dump(const char *level, const char *prefix_str, int prefix_type,
		    int rowsize, int groupsize,
		    const void *buf, size_t len, bool ascii);
#endif
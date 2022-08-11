/*******************************************************************************
  Copyright (C) 2007-2009  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#ifndef __BSTGMAC_H__
#define __BSTGMAC_H__

#define BSTGMAC_RESOURCE_NAME   "bstgmaceth"
#define DRV_MODULE_VERSION	"Jan_2016"

#include <linux/clk.h>
#include <linux/if_vlan.h>
#include <linux/stmmac.h>
#include <linux/phylink.h>
#include <linux/pci.h>
#include "common.h"
#include <linux/ptp_clock_kernel.h>
#include <linux/reset.h>

struct bstgmac_resources {
	void __iomem *addr;
	const char *mac;
	int sfty_ce_irq;
	int sfty_uc_irq;
	int wol_irq;
	int perch_rx_irq[MTL_MAX_RX_QUEUES];
	int perch_tx_irq[MTL_MAX_TX_QUEUES];
	int lpi_irq;
	int irq;
};

struct bstgmac_tx_info {
	dma_addr_t buf;
	bool map_as_page;
	unsigned len;
	bool last_segment;
	bool is_jumbo;
};

/* Frequently used values are kept adjacent for cache effect */
struct bstgmac_tx_queue {
	u32 tx_count_frames;
	struct timer_list txtimer;
	u32 queue_index;
	struct bstgmac_priv *priv_data;
	struct dma_extended_desc *dma_etx ____cacheline_aligned_in_smp;
	struct dma_desc *dma_tx;
	struct sk_buff **tx_skbuff;
	struct bstgmac_tx_info *tx_skbuff_dma;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	dma_addr_t dma_tx_phy;
	u32 tx_tail_addr;
	u32 mss;
};

struct bstgmac_rx_queue {
	u32 queue_index;
	struct bstgmac_priv *priv_data;
	struct dma_extended_desc *dma_erx;
	struct dma_desc *dma_rx ____cacheline_aligned_in_smp;
    spinlock_t que_lock ____cacheline_aligned_in_smp;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	u32 rx_zeroc_thresh;
	dma_addr_t dma_rx_phy;
	u32 rx_tail_addr;
};

struct bstgmac_channel {
	struct napi_struct napi ____cacheline_aligned_in_smp;
	struct napi_struct rnapi ____cacheline_aligned_in_smp;
	struct napi_struct tnapi ____cacheline_aligned_in_smp;
	struct bstgmac_priv *priv_data;
	u32 index;
	int has_rx;
	int has_tx;
	int int_mode;
    struct work_struct rx_work;
    struct work_struct tx_work;
};

struct bstgmac_tc_entry {
	bool in_use;
	bool in_hw;
	bool is_last;
	bool is_frag;
	void *frag_ptr;
	unsigned int table_pos;
	u32 handle;
	u32 prio;
	struct {
		u32 match_data;
		u32 match_en;
		u8 af:1;
		u8 rf:1;
		u8 im:1;
		u8 nc:1;
		u8 res1:4;
		u8 frame_offset;
		u8 ok_index;
		u8 dma_ch_no;
		u32 res2;
	} __packed val;
};

#define BSTGMAC_PPS_MAX		4
struct bstgmac_pps_cfg {
	bool available;
	struct timespec64 start;
	struct timespec64 period;
};

#define BSTGMAC_FLOW_ACTION_DROP		BIT(0)
struct bstgmac_flow_entry {
	unsigned long cookie;
	unsigned long action;
	u8 ip_proto;
	int in_use;
	int idx;
	int is_l4;
};

struct bstptp_ctl {
	int ppssta;
	int ppsfix;
	int multip;
	int extintr;
	void __iomem *ptp0_reg;
	void __iomem *ptp1_reg;
	struct timespec64 utc;
};

struct bstgmac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	u32 tx_coal_frames;
	u32 tx_coal_timer;

	int tx_coalesce;
	int hwts_tx_en;
	bool tx_path_in_lpi_mode;
	bool rx_path_in_lpi_mode;
	bool tso;

	unsigned int dma_buf_sz;
	unsigned int rx_copybreak;
	u32 rx_riwt;
	int hwts_rx_en;

	void __iomem *ioaddr;
	struct net_device *dev;
	struct device *device;
	struct mac_device_info *hw;
	int (*hwif_quirks)(struct bstgmac_priv *priv);
	struct mutex lock;

	/* RX Queue */
	struct bstgmac_rx_queue rx_queue[MTL_MAX_RX_QUEUES];

	/* TX Queue */
	struct bstgmac_tx_queue tx_queue[MTL_MAX_TX_QUEUES];

	/* Generic channel for NAPI */
    struct bstgmac_channel rx_channel[STMMAC_CH_MAX];
    struct bstgmac_channel tx_channel[STMMAC_CH_MAX];
    
	bool oldlink;
	int speed;
	int oldduplex;
	unsigned int flow_ctrl;
	unsigned int pause;
	struct mii_bus *mii;
	int mii_irq[PHY_MAX_ADDR];

    struct phylink_config phylink_config;
	struct phylink *phylink;

	struct bstgmac_extra_stats xstats ____cacheline_aligned_in_smp;
	struct bstgmac_safety_stats sstats;
	struct plat_stmmacenet_data *plat;
	struct dma_features dma_cap;
	struct bstgmac_counters mmc;
	int hw_cap_support;
	int synopsys_id;
	u32 msg_enable;
	int wolopts;
	int wol_irq;
	int clk_csr;
	struct timer_list eee_ctrl_timer;
	int lpi_irq;
	int sfty_ce_irq;
	int sfty_uc_irq;
	int perch_rx_irq[MTL_MAX_RX_QUEUES];
	int perch_tx_irq[MTL_MAX_TX_QUEUES]; 
	int eee_enabled;
	int eee_active;
	int tx_lpi_timer;
    int tx_lpi_enabled;
	unsigned int mode;
	unsigned int chain_mode;
	int extend_desc;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_ops;
	unsigned int default_addend;
	u32 sub_second_inc;
	u32 systime_flags;
	u32 adv_ts;
	int use_riwt;
	int irq_wake;
	spinlock_t ptp_lock;
	void __iomem *mmcaddr;
	void __iomem *ptpaddr;
	unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];

#ifdef CONFIG_DEBUG_FS
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_rings_status;
	struct dentry *dbgfs_dma_cap;
#endif

	unsigned long state;
	struct workqueue_struct *wq;
    struct workqueue_struct *rxmem;
    struct workqueue_struct *tx_wq;
	struct work_struct service_task;
    struct work_struct mem_mgmt_work;

	/* TC Handling */
	unsigned int tc_entries_max;
	unsigned int tc_off_max;
	struct bstgmac_tc_entry *tc_entries;
	unsigned int flow_entries_max;
	struct bstgmac_flow_entry *flow_entries;

	/* Pulse Per Second output */
	struct bstgmac_pps_cfg pps[BSTGMAC_PPS_MAX];
    int extend_op;

	struct bstptp_ctl *ptpctl;
};

enum bstgmac_state {
	BSTGMAC_DOWN,
	BSTGMAC_RESET_REQUESTED,
	BSTGMAC_RESETING,
	BSTGMAC_SERVICE_SCHED,
	BSTGMAC_RXMEM_WORK_RUN,
};

struct bstgmac_mem_t { 
    struct sk_buff *skb;
    dma_addr_t buf;
};

int bstgmac_mdio_unregister(struct net_device *ndev);
int bstgmac_mdio_register(struct net_device *ndev);
int bstgmac_mdio_reset(struct mii_bus *mii);
void bstgmac_set_ethtool_ops(struct net_device *netdev);

void bstgmac_ptp_register(struct bstgmac_priv *priv);
void bstgmac_ptp_unregister(struct bstgmac_priv *priv);
int bstgmac_resume(struct device *dev);
int bstgmac_suspend(struct device *dev);
int bstgmac_dvr_remove(struct device *dev);
int bstgmac_dvr_probe(struct platform_device *pdev,
		     struct plat_stmmacenet_data *plat_dat,
		     struct bstgmac_resources *res);
void bstgmac_disable_eee_mode(struct bstgmac_priv *priv);
bool bstgmac_eee_init(struct bstgmac_priv *priv);
int bstgmac_config_mv88e6352(struct net_device *ndev);
void bcm89881_extend_op(struct phy_device *phydev);
struct timespec64 bstgmac_calc_tas_basetime(ktime_t old_base_time,
					   ktime_t current_time,
					   u64 cycle_time);
#endif /* __BSTGMAC_H__ */

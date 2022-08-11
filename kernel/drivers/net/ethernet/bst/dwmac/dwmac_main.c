/*******************************************************************************
  This is the driver for the ST MAC 10/100/1000 on-chip Ethernet controllers.
  ST Ethernet IPs are built around a Synopsys IP Core.

	Copyright(C) 2007-2011 STMicroelectronics Ltd

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

  Documentation available at:
	http://www.stlinux.com
  Support available at:
	https://bugzilla.stlinux.com/
*******************************************************************************/

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <linux/pinctrl/consumer.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#include <linux/net_tstamp.h>
#include <net/pkt_cls.h>
#include "dwmac_ptp.h"
#include "dwmac4_dma.h"
#include "bstgmac.h"
#include <linux/reset.h>
#include <linux/of_mdio.h>
#include <linux/bst_boardconfig.h>
#include <linux/phylink.h>
#include "dwmac_main.h"
#include "hwif.h"

#define	BSTGMAC_ALIGN(x)		__ALIGN_KERNEL(x, SMP_CACHE_BYTES)
#define	TSO_MAX_BUFF_SIZE	(SZ_16K - 1)

/* Module parameters */
#define TX_TIMEO	5000
static int watchdog = TX_TIMEO;
module_param(watchdog, int, 0644);
MODULE_PARM_DESC(watchdog, "Transmit timeout in milliseconds (default 5s)");

static int debug = -1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static int phyaddr = -1;
module_param(phyaddr, int, 0444);
MODULE_PARM_DESC(phyaddr, "Physical device address");

#define BSTGMAC_TX_THRESH	(DMA_TX_SIZE / 4)
#define BSTGMAC_RX_THRESH	(DMA_RX_SIZE / 4)

static int flow_ctrl = FLOW_OFF;
module_param(flow_ctrl, int, 0644);
MODULE_PARM_DESC(flow_ctrl, "Flow control ability [on/off]");

static int pause = PAUSE_TIME;
module_param(pause, int, 0644);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

#define TC_DEFAULT 64
static int tc = TC_DEFAULT;
module_param(tc, int, 0644);
MODULE_PARM_DESC(tc, "DMA threshold control value");

#define	DEFAULT_BUFSIZE	1536
static int buf_sz = DEFAULT_BUFSIZE;
module_param(buf_sz, int, 0644);
MODULE_PARM_DESC(buf_sz, "DMA buffer size");

#define	BSTGMAC_RX_COPYBREAK	256

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
				      NETIF_MSG_LINK | NETIF_MSG_IFUP |
				      NETIF_MSG_IFDOWN | NETIF_MSG_TIMER);

#define BSTGMAC_DEFAULT_LPI_TIMER	1000
static int eee_timer = BSTGMAC_DEFAULT_LPI_TIMER;
module_param(eee_timer, int, 0644);
MODULE_PARM_DESC(eee_timer, "LPI tx expiration time in msec");
#define BSTGMAC_LPI_T(x) (jiffies + msecs_to_jiffies(x))
#define BSTGMAC_TRANS_TIMEOUT_MS	3000

/* By default the driver will use the ring mode to manage tx and rx descriptors,
 * but allow user to force to use the chain instead of the ring
 */
static unsigned int chain_mode;
module_param(chain_mode, int, 0444);
MODULE_PARM_DESC(chain_mode, "To use chain instead of ring mode");

static u32 t1_phyrole = 0;
module_param(t1_phyrole, uint, 0644);
MODULE_PARM_DESC(t1_phyrole, "T1 PHY ROLE(1:master, 2:slave)");

static u32 perf_debug = 0;
module_param(perf_debug, uint, 0644);
MODULE_PARM_DESC(perf_debug, "udp performance test");

unsigned int gmac0_phyrole = 0;
unsigned int gmac1_phyrole = 0;

spinlock_t gmac0_irqbits_lock ____cacheline_aligned_in_smp;
spinlock_t gmac1_irqbits_lock ____cacheline_aligned_in_smp;

static struct sk_buff_head gmac_delivery_skblist[BSTGMAC_CORE_NUM * BSTGMAC_RXCHAN_NUM];
static struct bstgmac_priv *gmac_priv_g[2] = {0};
struct bstgmac_board_para bstgmac_para[2];

static irqreturn_t bstgmac_interrupt(int irq, void *dev_id);
static irqreturn_t bstgmac_sbd_interrupt(int irq, void *dev_id);
static irqreturn_t bstgmac_safety_interrupt(int irq, void *dev_id);
static irqreturn_t bstgmac_rx_interrupt(int irq, void *dev_id);
static irqreturn_t bstgmac_tx_interrupt(int irq, void *dev_id);
static irqreturn_t bstgmac_lpi_interrupt(int irq, void *dev_id);

static irqreturn_t bstgmac_rx_interrupt_threading_null(int irq, void *dev_id);
#ifdef CONFIG_DEBUG_FS
static int bstgmac_init_fs(struct net_device *dev);
static void bstgmac_exit_fs(struct net_device *dev);
#endif

#define BSTGMAC_COAL_TIMER(x) (jiffies + usecs_to_jiffies(x))
#define BSTGMAC_RX_NAPI 1
#define BSTGMAC_TX_NAPI 0
#define BSTGMAC_RX_INTERRUPT_THREAD 0
/**
 * bstgmac_verify_args - verify the driver parameters.
 * Description: it checks the driver parameters and set a default in case of
 * errors.
 */
static void bstgmac_verify_args(void)
{
	if (unlikely(watchdog < 0))
		watchdog = TX_TIMEO;
	if (unlikely((buf_sz < DEFAULT_BUFSIZE) || (buf_sz > BUF_SIZE_16KiB)))
		buf_sz = DEFAULT_BUFSIZE;
	if (unlikely(flow_ctrl > 1))
		flow_ctrl = FLOW_AUTO;
	else if (likely(flow_ctrl < 0))
		flow_ctrl = FLOW_OFF;
	if (unlikely((pause < 0) || (pause > 0xffff)))
		pause = PAUSE_TIME;
	if (eee_timer < 0)
		eee_timer = BSTGMAC_DEFAULT_LPI_TIMER;
}

/**
 * bstgmac_disable_all_queues - Disable all queues
 * @priv: driver private structure
 */
static void bstgmac_disable_all_queues(struct bstgmac_priv *priv)
{
	u32 rx_queues_cnt = priv->plat->rx_queues_to_use;
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_queues_cnt; queue++) {
		struct bstgmac_channel *ch = &priv->rx_channel[queue];
		if(ch->int_mode != DMA_INT_M_0 ){
#if BSTGMAC_RX_NAPI
			if(queue < rx_queues_cnt){
				napi_disable(&ch->rnapi);
			}
#endif			
		}else{
			napi_disable(&ch->napi);
		}
	}
    for (queue = 0; queue < tx_queues_cnt; queue++) {
		struct bstgmac_channel *ch = &priv->tx_channel[queue];
		if(ch->int_mode != DMA_INT_M_0 ){
#if BSTGMAC_TX_NAPI
			if(queue < tx_queues_cnt){
				napi_disable(&ch->tnapi);
			}
#endif
		}
	}
}

/**
 * bstgmac_enable_all_queues - Enable all queues
 * @priv: driver private structure
 */
static void bstgmac_enable_all_queues(struct bstgmac_priv *priv)
{
	u32 rx_queues_cnt = priv->plat->rx_queues_to_use;
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_queues_cnt; queue++) {
		struct bstgmac_channel *ch = &priv->rx_channel[queue];
		if(ch->int_mode != DMA_INT_M_0 ) {
#if BSTGMAC_RX_NAPI
			if(queue < rx_queues_cnt){
				napi_enable(&ch->rnapi);
			}
#endif
		}else{
			napi_enable(&ch->napi);
		}
	}
    for (queue = 0; queue < tx_queues_cnt; queue++) {
		struct bstgmac_channel *ch = &priv->tx_channel[queue];
		if(ch->int_mode != DMA_INT_M_0 ) {
#if BSTGMAC_TX_NAPI
			if(queue < tx_queues_cnt){
				napi_enable(&ch->tnapi);
			}
#endif
		}
	}
}

static void bstgmac_service_event_schedule(struct bstgmac_priv *priv)
{
	if (!test_bit(BSTGMAC_DOWN, &priv->state) &&
	    !test_and_set_bit(BSTGMAC_SERVICE_SCHED, &priv->state))
		queue_work(priv->wq, &priv->service_task);
}

static void bstgmac_global_err(struct bstgmac_priv *priv)
{
	netif_carrier_off(priv->dev);
	set_bit(BSTGMAC_RESET_REQUESTED, &priv->state);
	bstgmac_service_event_schedule(priv);
}

/**
 * bstgmac_clk_csr_set - dynamically set the MDC clock
 * @priv: driver private structure
 * Description: this is to dynamically set the MDC clock according to the csr
 * clock input.
 * Note:
 *	If a specific clk_csr value is passed from the platform
 *	this means that the CSR Clock Range selection cannot be
 *	changed at run-time and it is fixed (as reported in the driver
 *	documentation). Viceversa the driver will try to set the MDC
 *	clock dynamically according to the actual clock input.
 */
static void bstgmac_clk_csr_set(struct bstgmac_priv *priv)
{
	u32 clk_rate;

	clk_rate = clk_get_rate(priv->plat->stmmac_clk);

	/* Platform provided default clk_csr would be assumed valid
	 * for all other cases except for the below mentioned ones.
	 * For values higher than the IEEE 802.3 specified frequency
	 * we can not estimate the proper divider as it is not known
	 * the frequency of clk_csr_i. So we do not change the default
	 * divider.
	 */
	if (!(priv->clk_csr & MAC_CSR_H_FRQ_MASK)) {
		if (clk_rate < CSR_F_35M)
			priv->clk_csr = STMMAC_CSR_20_35M;
		else if ((clk_rate >= CSR_F_35M) && (clk_rate < CSR_F_60M))
			priv->clk_csr = STMMAC_CSR_35_60M;
		else if ((clk_rate >= CSR_F_60M) && (clk_rate < CSR_F_100M))
			priv->clk_csr = STMMAC_CSR_60_100M;
		else if ((clk_rate >= CSR_F_100M) && (clk_rate < CSR_F_150M))
			priv->clk_csr = STMMAC_CSR_100_150M;
		else if ((clk_rate >= CSR_F_150M) && (clk_rate < CSR_F_250M))
			priv->clk_csr = STMMAC_CSR_150_250M;
		else if ((clk_rate >= CSR_F_250M) && (clk_rate < CSR_F_300M))
			priv->clk_csr = STMMAC_CSR_250_300M;
	}

	if (priv->plat->has_sun8i) {
		if (clk_rate > 160000000)
			priv->clk_csr = 0x03;
		else if (clk_rate > 80000000)
			priv->clk_csr = 0x02;
		else if (clk_rate > 40000000)
			priv->clk_csr = 0x01;
		else
			priv->clk_csr = 0;
	}

	if (priv->plat->has_xgmac) {
		if (clk_rate > 400000000)
			priv->clk_csr = 0x5;
		else if (clk_rate > 350000000)
			priv->clk_csr = 0x4;
		else if (clk_rate > 300000000)
			priv->clk_csr = 0x3;
		else if (clk_rate > 250000000)
			priv->clk_csr = 0x2;
		else if (clk_rate > 150000000)
			priv->clk_csr = 0x1;
		else
			priv->clk_csr = 0x0;
	}
}

static void print_pkt(unsigned char *buf, int len)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1,
		       buf, len, true);
}

static inline u32 bstgmac_tx_avail(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 avail;

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx - 1;
	else
		avail = DMA_TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx - 1;

	return avail;
}

/**
 * bstgmac_rx_dirty - Get RX queue dirty
 * @priv: driver private structure
 * @queue: RX queue index
 */
static inline u32 bstgmac_rx_dirty(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];
	u32 dirty;

	if (rx_q->dirty_rx <= rx_q->cur_rx)
		dirty = rx_q->cur_rx - rx_q->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_q->dirty_rx + rx_q->cur_rx;

	return dirty;
}

/**
 * bstgmac_hw_fix_mac_speed - callback for speed selection
 * @priv: driver private structure
 * Description: on some platforms (e.g. ST), some HW system configuration
 * registers have to be set according to the link speed negotiated.
 */
static inline void bstgmac_hw_fix_mac_speed(struct bstgmac_priv *priv)
{
	struct net_device *ndev = priv->dev;
	struct phy_device *phydev = ndev->phydev;

	if (likely(priv->plat->fix_mac_speed))
		priv->plat->fix_mac_speed(priv->plat->bsp_priv, phydev->speed);
}

/**
 * bstgmac_enable_eee_mode - check and enter in LPI mode
 * @priv: driver private structure
 * Description: this function is to verify and enter in LPI mode in case of
 * EEE.
 */
static void bstgmac_enable_eee_mode(struct bstgmac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	/* check if all TX queues have the work finished */
	for (queue = 0; queue < tx_cnt; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		if (tx_q->dirty_tx != tx_q->cur_tx)
			return; /* still unfinished work */
	}

	/* Check and enter in LPI mode */
	if (!priv->tx_path_in_lpi_mode)
		bstgmac_set_eee_mode(priv, priv->hw,
				priv->plat->en_tx_lpi_clockgating);
}

/**
 * bstgmac_disable_eee_mode - disable and exit from LPI mode
 * @priv: driver private structure
 * Description: this function is to exit and disable EEE in case of
 * LPI state is true. This is called by the xmit.
 */
void bstgmac_disable_eee_mode(struct bstgmac_priv *priv)
{
	bstgmac_reset_eee_mode(priv, priv->hw);
	del_timer_sync(&priv->eee_ctrl_timer);
	priv->tx_path_in_lpi_mode = false;
}

/**
 * bstgmac_eee_ctrl_timer - EEE TX SW timer.
 * @arg : data hook
 * Description:
 *  if there is no data transfer and if we are not in LPI state,
 *  then MAC Transmitter can be moved to LPI state.
 */
static void bstgmac_eee_ctrl_timer(struct timer_list *t)
{
	struct bstgmac_priv *priv = from_timer(priv, t, eee_ctrl_timer);

	bstgmac_enable_eee_mode(priv);
	mod_timer(&priv->eee_ctrl_timer, BSTGMAC_LPI_T(eee_timer));
}

/**
 * bstgmac_eee_init - init EEE
 * @priv: driver private structure
 * Description:
 *  if the GMAC supports the EEE (from the HW cap reg) and the phy device
 *  can also manage EEE, this function enable the LPI state and start related
 *  timer.
 */
bool bstgmac_eee_init(struct bstgmac_priv *priv)
{
	struct net_device *ndev = priv->dev;
	int interface = priv->plat->interface;
	bool ret = false;

	if ((interface != PHY_INTERFACE_MODE_MII) &&
	    (interface != PHY_INTERFACE_MODE_GMII) &&
	    !phy_interface_mode_is_rgmii(interface))
		goto out;

	/* Using PCS we cannot dial with the phy registers at this stage
	 * so we do not support extra feature like EEE.
	 */
	if ((priv->plat->bus_id == 1) || (priv->hw->pcs == BSTGMAC_PCS_TBI) ||
	    (priv->hw->pcs == BSTGMAC_PCS_RTBI))
		goto out;

	/* MAC core supports the EEE feature. */
	if (priv->dma_cap.eee) {
		int tx_lpi_timer = priv->tx_lpi_timer;

		/* Check if the PHY supports EEE */
		if (phy_init_eee(ndev->phydev, 1)) {
			/* To manage at run-time if the EEE cannot be supported
			 * anymore (for example because the lp caps have been
			 * changed).
			 * In that case the driver disable own timers.
			 */
			mutex_lock(&priv->lock);
			if (priv->eee_active) {
				netdev_dbg(priv->dev, "disable EEE\n");
				del_timer_sync(&priv->eee_ctrl_timer);
				bstgmac_set_eee_timer(priv, priv->hw, 0,
						tx_lpi_timer);
			}
			priv->eee_active = 0;
			mutex_unlock(&priv->lock);
			goto out;
		}
		/* Activate the EEE and start timers */
		mutex_lock(&priv->lock);
		if (!priv->eee_active) {
			priv->eee_active = 1;
			timer_setup(&priv->eee_ctrl_timer,
				    bstgmac_eee_ctrl_timer, 0);
			mod_timer(&priv->eee_ctrl_timer,
				  BSTGMAC_LPI_T(eee_timer));

			bstgmac_set_eee_timer(priv, priv->hw,
					BSTGMAC_DEFAULT_LIT_LS, tx_lpi_timer);
		}
		/* Set HW EEE according to the speed */
		bstgmac_set_eee_pls(priv, priv->hw, ndev->phydev->link);

		ret = true;
		mutex_unlock(&priv->lock);

		netdev_dbg(priv->dev, "Energy-Efficient Ethernet initialized\n");
	}
out:
	return ret;
}

/* bstgmac_get_tx_hwtstamp - get HW TX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read timestamp from the descriptor & pass it to stack.
 * and also perform some sanity checks.
 */
static void bstgmac_get_tx_hwtstamp(struct bstgmac_priv *priv,
				   struct dma_desc *p, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns = 0;

	if (!priv->hwts_tx_en)
		return;

	/* exit if skb doesn't support hw tstamp */
	if (likely(!skb || !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		return;

	/* check tx tstamp status */
	if (bstgmac_get_tx_timestamp_status(priv, p)) {
		/* get the valid tstamp */
		bstgmac_get_timestamp(priv, p, priv->adv_ts, &ns);

		memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp.hwtstamp = ns_to_ktime(ns);

		netdev_dbg(priv->dev, "get valid TX hw timestamp %llu\n", ns);
		/* pass tstamp to stack */
		skb_tstamp_tx(skb, &shhwtstamp);
	}

	return;
}

/* bstgmac_get_rx_hwtstamp - get HW RX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @np : next descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read received packet's timestamp from the descriptor
 * and pass it to stack. It also perform some sanity checks.
 */
static void bstgmac_get_rx_hwtstamp(struct bstgmac_priv *priv, struct dma_desc *p,
				   struct dma_desc *np, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	struct dma_desc *desc = p;
	u64 ns = 0;

	if (!priv->hwts_rx_en)
		return;
	/* For GMAC4, the valid timestamp is from CTX next desc. */
	if (priv->plat->has_gmac4 || priv->plat->has_xgmac)
		desc = np;

	/* Check if timestamp is available */
	if (bstgmac_get_rx_timestamp_status(priv, p, np, priv->adv_ts)) {
		bstgmac_get_timestamp(priv, desc, priv->adv_ts, &ns);
		netdev_dbg(priv->dev, "get valid RX hw timestamp %llu\n", ns);
		shhwtstamp = skb_hwtstamps(skb);
		memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp->hwtstamp = ns_to_ktime(ns);
	} else  {
		netdev_dbg(priv->dev, "cannot get RX hw timestamp\n");
	}
}

/**
 *  bstgmac_hwtstamp_ioctl - control hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specific structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function configures the MAC to enable/disable both outgoing(TX)
 *  and incoming(RX) packets time stamping based on user input.
 *  Return Value:
 *  0 on success and an appropriate -ve integer on failure.
 */
static int bstgmac_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	struct timespec64 now;
	u64 temp = 0;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 sec_inc = 0;
	u32 value = 0;
	bool xmac;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
		netdev_alert(priv->dev, "No support for HW time stamping\n");
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;

		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	netdev_dbg(priv->dev, "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
		   __func__, config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	if (config.tx_type != HWTSTAMP_TX_OFF &&
	    config.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	if (priv->adv_ts) {
		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			/* time stamp no incoming packet at all */
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
			/* PTP v1, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			/* 'xmac' hardware can support Sync, Pdelay_Req and
			 * Pdelay_resp by setting bit14 and bits17/16 to 01
			 * This leaves Delay_Req timestamps out.
			 * Enable all events *and* general purpose message
			 * timestamping
			 */
			snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			/* PTP v1, UDP, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			/* PTP v1, UDP, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
			/* PTP v2, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for all event messages */
			snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
			/* PTP v2, UDP, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			/* PTP v2, UDP, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			/* PTP v2/802.AS1 any layer, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			/* PTP v2/802.AS1, any layer, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for SYNC messages only */
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			/* PTP v2/802.AS1, any layer, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			/* take time stamp for Delay_Req messages only */
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;

			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_NTP_ALL:
		case HWTSTAMP_FILTER_ALL:
			/* time stamp any incoming packet */
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			tstamp_all = PTP_TCR_TSENALL;
			break;

		default:
			return -ERANGE;
		}
	} else {
		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;
		default:
			/* PTP v1, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			break;
		}
	}
	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
	priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		bstgmac_config_hw_tstamping(priv, priv->ptpaddr, 0);
	else {
		value = (PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR |
			 tstamp_all | ptp_v2 | ptp_over_ethernet |
			 ptp_over_ipv6_udp | ptp_over_ipv4_udp |
			 ts_master_en | snap_type_sel);
		bstgmac_config_hw_tstamping(priv, priv->ptpaddr, value);

		/* program Sub Second Increment reg */
		bstgmac_config_sub_second_increment(priv,
				priv->ptpaddr, priv->plat->clk_ptp_rate,
				xmac, &sec_inc);
		temp = div_u64(1000000000ULL, sec_inc);

		/* Store sub second increment and flags for later use */
		priv->sub_second_inc = sec_inc;
		priv->systime_flags = value;

		/* calculate default added value:
		 * formula is :
		 * addend = (2^32)/freq_div_ratio;
		 * where, freq_div_ratio = 1e9ns/sec_inc
		 */
		temp = (u64)(temp << 32);
		priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
		bstgmac_config_addend(priv, priv->ptpaddr, priv->default_addend);

		/* initialize system time */
		ktime_get_real_ts64(&now);

		/* lower 32 bits of tv_sec are safe until y2106 */
		bstgmac_init_systime(priv, priv->ptpaddr,
				(u32)now.tv_sec, now.tv_nsec);
		//priv->ptp_clock_ops.status = PTP_STA_INITED;
	}

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(struct hwtstamp_config)) ? -EFAULT : 0;
}

/**
 * bstgmac_init_ptp - init PTP
 * @priv: driver private structure
 * Description: this is to verify if the HW supports the PTPv1 or PTPv2.
 * This is done by looking at the HW cap. register.
 * This function also registers the ptp driver.
 */
static int bstgmac_init_ptp(struct bstgmac_priv *priv)
{
	bool xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	if (!(priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp))
		return -EOPNOTSUPP;

	priv->adv_ts = 0;
	/* Check if adv_ts can be enabled for dwmac 4.x / xgmac core */
	if (xmac && priv->dma_cap.atime_stamp)
		priv->adv_ts = 1;
	/* Dwmac 3.x core with extend_desc can support adv_ts */
	else if (priv->extend_desc && priv->dma_cap.atime_stamp)
		priv->adv_ts = 1;

	if (priv->dma_cap.time_stamp)
		netdev_info(priv->dev, "IEEE 1588-2002 Timestamp supported\n");

	if (priv->adv_ts)
		netdev_info(priv->dev,
			    "IEEE 1588-2008 Advanced Timestamp supported\n");

	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;

	bstgmac_ptp_register(priv);

	return 0;
}

static void bstgmac_release_ptp(struct bstgmac_priv *priv)
{
	//if (priv->plat->clk_ptp_ref)
		//clk_disable_unprepare(priv->plat->clk_ptp_ref);
	bstgmac_ptp_unregister(priv);
}

/**
 *  bstgmac_mac_flow_ctrl - Configure flow control in all queues
 *  @priv: driver private structure
 *  Description: It is used for configuring the flow control in all queues
 */
static void bstgmac_mac_flow_ctrl(struct bstgmac_priv *priv, u32 duplex)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;

	bstgmac_flow_ctrl(priv, priv->hw, duplex, priv->flow_ctrl,
			priv->pause, tx_cnt);
}

/**
 * bstgmac_adjust_link - adjusts the link parameters
 * @dev: net device structure
 * Description: this is the helper called by the physical abstraction layer
 * drivers to communicate the phy link status. According the speed and duplex
 * this driver can invoke registered glue-logic as well.
 * It also invoke the eee initialization because it could happen when switch
 * on different networks (that are eee capable).
 */
static void bstgmac_adjust_link(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	bool new_state = false;

	if (!phydev)
		return;

	mutex_lock(&priv->lock);

	if (phydev->link) {
		u32 ctrl = readl(priv->ioaddr + MAC_CTRL_REG);

		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode. */
		if (phydev->duplex != priv->oldduplex) {
			new_state = true;
			if (!phydev->duplex)
				ctrl &= ~priv->hw->link.duplex;
			else
				ctrl |= priv->hw->link.duplex;
			priv->oldduplex = phydev->duplex;
		}
		/* Flow Control operation */
		if (phydev->pause)
			bstgmac_mac_flow_ctrl(priv, phydev->duplex);

		if (phydev->speed != priv->speed) {
			new_state = true;
			ctrl &= ~priv->hw->link.speed_mask;
			switch (phydev->speed) {
			case SPEED_1000:
				ctrl |= priv->hw->link.speed1000;
				break;
			case SPEED_100:
				ctrl |= priv->hw->link.speed100;
				break;
			case SPEED_10:
				ctrl |= priv->hw->link.speed10;
				break;
			default:
				netif_warn(priv, link, priv->dev, 
					   "broken speed: %d\n", phydev->speed);
				phydev->speed = SPEED_UNKNOWN;
				break;
			}
			if (phydev->speed != SPEED_UNKNOWN)
				bstgmac_hw_fix_mac_speed(priv);
			priv->speed = phydev->speed;
		}

		writel(ctrl, priv->ioaddr + MAC_CTRL_REG);

		if (!priv->oldlink) {
			new_state = true;
			priv->oldlink = true;
		}
	} else if (priv->oldlink) {
		new_state = true;
		priv->oldlink = false;
		priv->speed = SPEED_UNKNOWN;
		priv->oldduplex = DUPLEX_UNKNOWN;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

	mutex_unlock(&priv->lock);

	if (phydev->is_pseudo_fixed_link)
		/* Stop PHY layer to call the hook to adjust the link in case
		 * of a switch is attached to the bstgmac driver.
		 */
		phydev->irq = PHY_IGNORE_INTERRUPT;
	else
		/* At this stage, init the EEE if supported.
		 * Never called in case of fixed_link.
		 */
		priv->eee_enabled = bstgmac_eee_init(priv);
}

/**
 * bstgmac_check_pcs_mode - verify if RGMII/SGMII is supported
 * @priv: driver private structure
 * Description: this is to verify if the HW supports the PCS.
 * Physical Coding Sublayer (PCS) interface that can be used when the MAC is
 * configured for the TBI, RTBI, or SGMII PHY interface.
 */
static void bstgmac_check_pcs_mode(struct bstgmac_priv *priv)
{
	int interface = priv->plat->interface;

	if (priv->dma_cap.pcs) {
		if ((interface == PHY_INTERFACE_MODE_RGMII) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_ID) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_RXID) ||
		    (interface == PHY_INTERFACE_MODE_RGMII_TXID)) {
			netdev_dbg(priv->dev, "PCS RGMII support enabled\n");
			priv->hw->pcs = BSTGMAC_PCS_RGMII;
		} else if (interface == PHY_INTERFACE_MODE_SGMII) {
			netdev_dbg(priv->dev, "PCS SGMII support enabled\n");
			priv->hw->pcs = BSTGMAC_PCS_SGMII;
		}
	}
}

/**
 * bstgmac_init_phy - PHY initialization
 * @dev: net device structure
 * Description: it initializes the driver's PHY state, and attaches the PHY
 * to the mac driver.
 *  Return value:
 *  0 on success
 */

static int bstgmac_init_phy(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	struct device_node *node;
	int ret;
    struct phy_device *phydev;
	
    node = priv->plat->phylink_node;
	if (node) {
		phydev = of_phy_find_device(priv->plat->phy_node);
		if (phydev) {
			if (bstgmac_para[priv->plat->bus_id].phyrole == 1) {
				phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_FORCE;
			} else if (bstgmac_para[priv->plat->bus_id].phyrole == 2) {
				phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_FORCE;
			}
		}
		ret = phylink_of_phy_connect(priv->phylink, node, 0);
	}

	if (!priv->plat->pmt) {
		struct ethtool_wolinfo wol = { .cmd = ETHTOOL_GWOL };

		phylink_ethtool_get_wol(priv->phylink, &wol);
		device_set_wakeup_capable(priv->device, !!wol.supported);
	}

    if ((priv->extend_op == BSTA1000_BOARD_EVB) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
        bcm89881_extend_op(phydev);
    }
	return ret;
}

static void bstgmac_validate(struct phylink_config *config,
			    unsigned long *supported,
			    struct phylink_link_state *state)
{
	struct bstgmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mac_supported) = { 0, };
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };
	int tx_cnt = priv->plat->tx_queues_to_use;
	int max_speed = priv->plat->max_speed;

	phylink_set(mac_supported, 10baseT_Half);
	phylink_set(mac_supported, 10baseT_Full);
	phylink_set(mac_supported, 100baseT_Half);
	phylink_set(mac_supported, 100baseT_Full);
	phylink_set(mac_supported, 1000baseT_Half);
	phylink_set(mac_supported, 1000baseT_Full);
	phylink_set(mac_supported, 1000baseKX_Full);

	phylink_set(mac_supported, Autoneg);
	phylink_set(mac_supported, Pause);
	phylink_set(mac_supported, Asym_Pause);
	phylink_set_port_modes(mac_supported);

	/* Cut down 1G if asked to */
	if ((max_speed > 0) && (max_speed < 1000)) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseX_Full);
	} else if (priv->plat->has_xgmac) {
		if (!max_speed || (max_speed >= 2500)) {
			phylink_set(mac_supported, 2500baseT_Full);
			phylink_set(mac_supported, 2500baseX_Full);
		}
		if (!max_speed || (max_speed >= 5000)) {
			phylink_set(mac_supported, 5000baseT_Full);
		}
		if (!max_speed || (max_speed >= 10000)) {
			phylink_set(mac_supported, 10000baseSR_Full);
			phylink_set(mac_supported, 10000baseLR_Full);
			phylink_set(mac_supported, 10000baseER_Full);
			phylink_set(mac_supported, 10000baseLRM_Full);
			phylink_set(mac_supported, 10000baseT_Full);
			phylink_set(mac_supported, 10000baseKX4_Full);
			phylink_set(mac_supported, 10000baseKR_Full);
		}
		if (!max_speed || (max_speed >= 25000)) {
			phylink_set(mac_supported, 25000baseCR_Full);
			phylink_set(mac_supported, 25000baseKR_Full);
			phylink_set(mac_supported, 25000baseSR_Full);
		}
		if (!max_speed || (max_speed >= 40000)) {
			phylink_set(mac_supported, 40000baseKR4_Full);
			phylink_set(mac_supported, 40000baseCR4_Full);
			phylink_set(mac_supported, 40000baseSR4_Full);
			phylink_set(mac_supported, 40000baseLR4_Full);
		}
		if (!max_speed || (max_speed >= 50000)) {
			phylink_set(mac_supported, 50000baseCR2_Full);
			phylink_set(mac_supported, 50000baseKR2_Full);
			phylink_set(mac_supported, 50000baseSR2_Full);
			phylink_set(mac_supported, 50000baseKR_Full);
			phylink_set(mac_supported, 50000baseSR_Full);
			phylink_set(mac_supported, 50000baseCR_Full);
			phylink_set(mac_supported, 50000baseLR_ER_FR_Full);
			phylink_set(mac_supported, 50000baseDR_Full);
		}
		if (!max_speed || (max_speed >= 100000)) {
			phylink_set(mac_supported, 100000baseKR4_Full);
			phylink_set(mac_supported, 100000baseSR4_Full);
			phylink_set(mac_supported, 100000baseCR4_Full);
			phylink_set(mac_supported, 100000baseLR4_ER4_Full);
			phylink_set(mac_supported, 100000baseKR2_Full);
			phylink_set(mac_supported, 100000baseSR2_Full);
			phylink_set(mac_supported, 100000baseCR2_Full);
			phylink_set(mac_supported, 100000baseLR2_ER2_FR2_Full);
			phylink_set(mac_supported, 100000baseDR2_Full);
		}
	}

	/* Half-Duplex can only work with single queue */
	if (tx_cnt > 1) {
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 1000baseT_Half);
	}
	
	linkmode_and(supported, supported, mac_supported);
	linkmode_andnot(supported, supported, mask);

	linkmode_and(state->advertising, state->advertising, mac_supported);
	linkmode_andnot(state->advertising, state->advertising, mask);

	/* If PCS is supported, check which modes it supports. 
	bstgmac_xpcs_validate(priv, &priv->hw->xpcs_args, supported, state);*/
}

static void bstgmac_mac_pcs_get_state(struct phylink_config *config,
				     struct phylink_link_state *state)
{
	struct bstgmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	state->link = 0;
	bstgmac_xpcs_get_state(priv, &priv->hw->xpcs_args, state);
}

static void bstgmac_mac_config(struct phylink_config *config, unsigned int mode,
			      const struct phylink_link_state *state)
{
	struct bstgmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	bstgmac_xpcs_config(priv, &priv->hw->xpcs_args, state);
}

static void bstgmac_mac_an_restart(struct phylink_config *config)
{
	/* Not Supported */
}

static void bstgmac_mac_link_down(struct phylink_config *config,
				 unsigned int mode, phy_interface_t interface)
{
	struct bstgmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	bstgmac_mac_set(priv, priv->ioaddr, false);
	priv->eee_active = false;
	priv->tx_lpi_enabled = false;
	#if 0
	if (priv->dma_cap.eee) {
		bstgmac_eee_init(priv);
		bstgmac_set_eee_pls(priv, priv->hw, false);
	}
	#endif
}

static void bstgmac_mac_link_up(struct phylink_config *config,
			       struct phy_device *phy,
			       unsigned int mode, phy_interface_t interface,
			       int speed, int duplex,
			       bool tx_pause, bool rx_pause)
{
	struct bstgmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	u32 ctrl;

	bstgmac_xpcs_link_up(priv, &priv->hw->xpcs_args, speed, interface);

	ctrl = readl(priv->ioaddr + MAC_CTRL_REG);
	ctrl &= ~priv->hw->link.speed_mask;

	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		switch (speed) {
		case SPEED_10000:
			ctrl |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_5000:
			ctrl |= priv->hw->link.xgmii.speed5000;
			break;
		case SPEED_2500:
			ctrl |= priv->hw->link.xgmii.speed2500;
			break;
		default:
			return;
		}
	} else if (interface == PHY_INTERFACE_MODE_XLGMII) {
		switch (speed) {
		case SPEED_100000:
			ctrl |= priv->hw->link.xlgmii.speed100000;
			break;
		case SPEED_50000:
			ctrl |= priv->hw->link.xlgmii.speed50000;
			break;
		case SPEED_40000:
			ctrl |= priv->hw->link.xlgmii.speed40000;
			break;
		case SPEED_25000:
			ctrl |= priv->hw->link.xlgmii.speed25000;
			break;
		case SPEED_10000:
			ctrl |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_2500:
			ctrl |= priv->hw->link.speed2500;
			break;
		case SPEED_1000:
			ctrl |= priv->hw->link.speed1000;
			break;
		default:
			return;
		}
	} else {
		switch (speed) {
		case SPEED_2500:
			ctrl |= priv->hw->link.speed2500;
			break;
		case SPEED_1000:
			ctrl |= priv->hw->link.speed1000;
			break;
		case SPEED_100:
			ctrl |= priv->hw->link.speed100;
			break;
		case SPEED_10:
			ctrl |= priv->hw->link.speed10;
			break;
		default:
			return;
		}
	}

	priv->speed = speed;

	if (priv->plat->fix_mac_speed)
		priv->plat->fix_mac_speed(priv->plat->bsp_priv, speed);

	if (!duplex)
		ctrl &= ~priv->hw->link.duplex;
	else
		ctrl |= priv->hw->link.duplex;

	/* Flow Control operation */
	if (tx_pause && rx_pause)
		bstgmac_mac_flow_ctrl(priv, duplex);

	writel(ctrl, priv->ioaddr + MAC_CTRL_REG);

	bstgmac_mac_set(priv, priv->ioaddr, true);
	#if 0
	if (phy && priv->dma_cap.eee) {
		priv->eee_active = phy_init_eee(phy, 1) >= 0;
		priv->eee_enabled = bstgmac_eee_init(priv);
		priv->tx_lpi_enabled = priv->eee_enabled;
		bstgmac_set_eee_pls(priv, priv->hw, true);
	}
	#endif
}

static const struct phylink_mac_ops bstgmac_phylink_mac_ops = {
	.validate = bstgmac_validate,
	.mac_pcs_get_state = bstgmac_mac_pcs_get_state,
	.mac_config = bstgmac_mac_config,
	.mac_an_restart = bstgmac_mac_an_restart,
	.mac_link_down = bstgmac_mac_link_down,
	.mac_link_up = bstgmac_mac_link_up,
};

static int bstgmac_phy_setup(struct bstgmac_priv *priv)
{
	struct fwnode_handle *fwnode = of_fwnode_handle(priv->plat->phylink_node);
	int mode = priv->plat->phy_interface;
	struct phylink *phylink;

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.pcs_poll = true;

	if (!fwnode)
		fwnode = dev_fwnode(priv->device);

	phylink = phylink_create(&priv->phylink_config, fwnode,
				 mode, &bstgmac_phylink_mac_ops);
	if (IS_ERR(phylink))
		return PTR_ERR(phylink);

	priv->phylink = phylink;
	return 0;
}

static void bstgmac_display_rx_rings(struct bstgmac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	void *head_rx;
	u32 queue;

	/* Display RX rings */
	for (queue = 0; queue < rx_cnt; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		pr_info("\tRX Queue %u rings\n", queue);

		if (priv->extend_desc)
			head_rx = (void *)rx_q->dma_erx;
		else
			head_rx = (void *)rx_q->dma_rx;

		/* Display RX ring */
		bstgmac_display_ring(priv, head_rx, DMA_RX_SIZE, true);
	}
}

static void bstgmac_display_tx_rings(struct bstgmac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	void *head_tx;
	u32 queue;

	/* Display TX rings */
	for (queue = 0; queue < tx_cnt; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		pr_info("\tTX Queue %d rings\n", queue);

		if (priv->extend_desc)
			head_tx = (void *)tx_q->dma_etx;
		else
			head_tx = (void *)tx_q->dma_tx;

		bstgmac_display_ring(priv, head_tx, DMA_TX_SIZE, false);
	}
}

static void bstgmac_display_rings(struct bstgmac_priv *priv)
{
	/* Display RX ring */
	bstgmac_display_rx_rings(priv);

	/* Display TX ring */
	bstgmac_display_tx_rings(priv);
}

static int bstgmac_set_bfsize(int mtu, int bufsize)
{
	int ret = bufsize;

	if (mtu >= BUF_SIZE_4KiB)
		ret = BUF_SIZE_8KiB;
	else if (mtu >= BUF_SIZE_2KiB)
		ret = BUF_SIZE_4KiB;
	else if (mtu > DEFAULT_BUFSIZE)
		ret = BUF_SIZE_2KiB;
	else
		ret = DEFAULT_BUFSIZE;

	return ret;
}

/**
 * bstgmac_clear_rx_descriptors - clear RX descriptors
 * @priv: driver private structure
 * @queue: RX queue index
 * Description: this function is called to clear the RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void bstgmac_clear_rx_descriptors(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];
	int i;

	/* Clear the RX descriptors */
	for (i = 0; i < DMA_RX_SIZE; i++)
		if (priv->extend_desc)
			bstgmac_init_rx_desc(priv, &rx_q->dma_erx[i].basic,
					priv->use_riwt, priv->mode,
					(i == DMA_RX_SIZE - 1),
					priv->dma_buf_sz);
		else
			bstgmac_init_rx_desc(priv, &rx_q->dma_rx[i],
					priv->use_riwt, priv->mode,
					(i == DMA_RX_SIZE - 1),
					priv->dma_buf_sz);
}

/**
 * bstgmac_clear_tx_descriptors - clear tx descriptors
 * @priv: driver private structure
 * @queue: TX queue index.
 * Description: this function is called to clear the TX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void bstgmac_clear_tx_descriptors(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;

	/* Clear the TX descriptors */
	for (i = 0; i < DMA_TX_SIZE; i++)
		if (priv->extend_desc)
			bstgmac_init_tx_desc(priv, &tx_q->dma_etx[i].basic,
					priv->mode, (i == DMA_TX_SIZE - 1));
		else
			bstgmac_init_tx_desc(priv, &tx_q->dma_tx[i],
					priv->mode, (i == DMA_TX_SIZE - 1));
}

/**
 * bstgmac_clear_descriptors - clear descriptors
 * @priv: driver private structure
 * Description: this function is called to clear the TX and RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void bstgmac_clear_descriptors(struct bstgmac_priv *priv)
{
	u32 rx_queue_cnt = priv->plat->rx_queues_to_use;
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	/* Clear the RX descriptors */
	for (queue = 0; queue < rx_queue_cnt; queue++)
		bstgmac_clear_rx_descriptors(priv, queue);

	/* Clear the TX descriptors */
	for (queue = 0; queue < tx_queue_cnt; queue++)
		bstgmac_clear_tx_descriptors(priv, queue);
}

/**
 * bstgmac_init_rx_buffers - init the RX descriptor buffer.
 * @priv: driver private structure
 * @p: descriptor pointer
 * @i: descriptor index
 * @flags: gfp flag
 * @queue: RX queue index
 * Description: this function is called to allocate a receive buffer, perform
 * the DMA mapping and init the descriptor.
 */
static int bstgmac_init_rx_buffers(struct bstgmac_priv *priv, struct dma_desc *p,
				  int i, gfp_t flags, u32 queue)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct sk_buff *skb;

	skb = __netdev_alloc_skb_ip_align(priv->dev, priv->dma_buf_sz, flags);
	if (!skb) {
		netdev_err(priv->dev,
			   "%s: Rx init fails; skb is NULL\n", __func__);
		return -ENOMEM;
	}
	rx_q->rx_skbuff[i] = skb;
	rx_q->rx_skbuff_dma[i] = dma_map_single(priv->device, skb->data,
						priv->dma_buf_sz,
						DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->device, rx_q->rx_skbuff_dma[i])) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	bstgmac_set_desc_addr(priv, p, rx_q->rx_skbuff_dma[i]);

	if (priv->dma_buf_sz == BUF_SIZE_16KiB)
		bstgmac_init_desc3(priv, p);

	return 0;
}

/**
 * bstgmac_free_rx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bstgmac_free_rx_buffer(struct bstgmac_priv *priv, u32 queue, int i)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

	if (rx_q->rx_skbuff[i]) {
		dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[i],
				 priv->dma_buf_sz, DMA_FROM_DEVICE);
		dev_kfree_skb_any(rx_q->rx_skbuff[i]);
	}
	rx_q->rx_skbuff[i] = NULL;
}

/**
 * bstgmac_free_tx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bstgmac_free_tx_buffer(struct bstgmac_priv *priv, u32 queue, int i)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

	if (tx_q->tx_skbuff_dma[i].buf) {
		if (tx_q->tx_skbuff_dma[i].map_as_page)
			dma_unmap_page(priv->device,
				       tx_q->tx_skbuff_dma[i].buf,
				       tx_q->tx_skbuff_dma[i].len,
				       DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device,
					 tx_q->tx_skbuff_dma[i].buf,
					 tx_q->tx_skbuff_dma[i].len,
					 DMA_TO_DEVICE);
	}

	if (tx_q->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_q->tx_skbuff[i]);
		tx_q->tx_skbuff[i] = NULL;
		tx_q->tx_skbuff_dma[i].buf = 0;
		tx_q->tx_skbuff_dma[i].map_as_page = false;
	}
}

/**
 * init_dma_rx_desc_rings - init the RX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_rx_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	int bfsize = 0;
	int queue;
	int i;

	bfsize = bstgmac_set_16kib_bfsize(priv, dev->mtu);
	if (bfsize < 0)
		bfsize = 0;

	if (bfsize < BUF_SIZE_16KiB)
		bfsize = bstgmac_set_bfsize(dev->mtu, priv->dma_buf_sz);

	priv->dma_buf_sz = bfsize;

	/* RX INITIALIZATION */
	netif_dbg(priv, probe, priv->dev,
		  "SKB addresses:\nskb\t\tskb data\tdma data\n");

	for (queue = 0; queue < rx_count; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_rx_phy=0x%08x\n", __func__,
			  (u32)rx_q->dma_rx_phy);

		for (i = 0; i < DMA_RX_SIZE; i++) {
			struct dma_desc *p;

			if (priv->extend_desc)
				p = &((rx_q->dma_erx + i)->basic);
			else
				p = rx_q->dma_rx + i;

			ret = bstgmac_init_rx_buffers(priv, p, i, flags,
						     queue);
			if (ret)
				goto err_init_rx_buffers;

			netif_dbg(priv, probe, priv->dev, "[%p]\t[%p]\t[%x]\n",
				  rx_q->rx_skbuff[i], rx_q->rx_skbuff[i]->data,
				  (unsigned int)rx_q->rx_skbuff_dma[i]);
		}

		rx_q->cur_rx = 0;
		rx_q->dirty_rx = (unsigned int)(i - DMA_RX_SIZE);
        spin_lock_init(&rx_q->que_lock);

		bstgmac_clear_rx_descriptors(priv, queue);

		/* Setup the chained descriptor addresses */
		if (priv->mode == BSTGMAC_CHAIN_MODE) {
			if (priv->extend_desc)
				bstgmac_mode_init(priv, rx_q->dma_erx,
						rx_q->dma_rx_phy, DMA_RX_SIZE, 1);
			else
				bstgmac_mode_init(priv, rx_q->dma_rx,
						rx_q->dma_rx_phy, DMA_RX_SIZE, 0);
		}
	}

	buf_sz = bfsize;

	return 0;

err_init_rx_buffers:
	while (queue >= 0) {
		while (--i >= 0)
			bstgmac_free_rx_buffer(priv, queue, i);

		if (queue == 0)
			break;

		i = DMA_RX_SIZE;
		queue--;
	}

	return ret;
}

/**
 * init_dma_tx_desc_rings - init the TX descriptor rings
 * @dev: net device structure.
 * Description: this function initializes the DMA TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_tx_desc_rings(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	int i;

	for (queue = 0; queue < tx_queue_cnt; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_tx_phy=0x%08x\n", __func__,
			 (u32)tx_q->dma_tx_phy);

		/* Setup the chained descriptor addresses */
		if (priv->mode == BSTGMAC_CHAIN_MODE) {
			if (priv->extend_desc)
				bstgmac_mode_init(priv, tx_q->dma_etx,
						tx_q->dma_tx_phy, DMA_TX_SIZE, 1);
			else
				bstgmac_mode_init(priv, tx_q->dma_tx,
						tx_q->dma_tx_phy, DMA_TX_SIZE, 0);
		}

		for (i = 0; i < DMA_TX_SIZE; i++) {
			struct dma_desc *p;
			if (priv->extend_desc)
				p = &((tx_q->dma_etx + i)->basic);
			else
				p = tx_q->dma_tx + i;

			bstgmac_clear_desc(priv, p);

			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
			tx_q->tx_skbuff_dma[i].len = 0;
			tx_q->tx_skbuff_dma[i].last_segment = false;
			tx_q->tx_skbuff[i] = NULL;
		}

		tx_q->dirty_tx = 0;
		tx_q->cur_tx = 0;
		tx_q->mss = 0;

		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	return 0;
}

/**
 * init_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	int ret;

	ret = init_dma_rx_desc_rings(dev, flags);
	if (ret)
		return ret;

	ret = init_dma_tx_desc_rings(dev);

	bstgmac_clear_descriptors(priv);

	if (netif_msg_hw(priv))
		bstgmac_display_rings(priv);

	return ret;
}

/**
 * dma_free_rx_skbufs - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 */
static void dma_free_rx_skbufs(struct bstgmac_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_RX_SIZE; i++)
		bstgmac_free_rx_buffer(priv, queue, i);
}

/**
 * dma_free_tx_skbufs - free TX dma buffers
 * @priv: private structure
 * @queue: TX queue index
 */
static void dma_free_tx_skbufs(struct bstgmac_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_TX_SIZE; i++)
		bstgmac_free_tx_buffer(priv, queue, i);
}

static void bstgmac_free_rxmem(struct bstgmac_priv *priv)
{
    int chan, busid, index, i = 0;
    struct sk_buff_head *list;
    struct sk_buff *skb; 
    dma_addr_t *buf;

    busid = priv->plat->bus_id;
    for (chan = 0; chan < BSTGMAC_RXCHAN_NUM; chan++) {
        index = busid * BSTGMAC_RXCHAN_NUM + chan;
        list = &gmac_delivery_skblist[index];
        while (skb_queue_len(list)) {        
            skb = skb_dequeue(list);       
            if (skb) {
                buf = (dma_addr_t *)skb->cb;
                if (*buf) {
                    dma_unmap_single(priv->device, *buf, priv->dma_buf_sz, DMA_FROM_DEVICE);
                    *buf = 0;
                    dma_wmb();
		            dev_kfree_skb_any(skb);
                }
            }
            i++;
        }
    }
}

static void free_dma_rx_mem_res(struct bstgmac_priv *priv)
{
    int i;
    int it_ms = 10, cnt = (BSTGMAC_TRANS_TIMEOUT_MS+it_ms)/it_ms;

    i = 0;
    while((i < cnt) && (test_bit(BSTGMAC_RXMEM_WORK_RUN, &priv->state))) {
        msleep(it_ms);
        i++;
    }
    if (i != cnt) {
        bstgmac_free_rxmem(priv);
    } else {
        printk(KERN_ERR "An exception occurred, so rxmem buf was not released\n");
    }
}

/**
 * free_dma_rx_desc_resources - free RX dma desc resources
 * @priv: private structure
 */
static void free_dma_rx_desc_resources(struct bstgmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;

	/* Free RX queue resources */
	for (queue = 0; queue < rx_count; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		/* Release the DMA RX socket buffers */
		dma_free_rx_skbufs(priv, queue);

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc)
			dma_free_coherent(priv->device,
					  DMA_RX_SIZE * sizeof(struct dma_desc),
					  rx_q->dma_rx, rx_q->dma_rx_phy);
		else
			dma_free_coherent(priv->device, DMA_RX_SIZE *
					  sizeof(struct dma_extended_desc),
					  rx_q->dma_erx, rx_q->dma_rx_phy);

		kfree(rx_q->rx_skbuff_dma);
		kfree(rx_q->rx_skbuff);
	}
}

/**
 * free_dma_tx_desc_resources - free TX dma desc resources
 * @priv: private structure
 */
static void free_dma_tx_desc_resources(struct bstgmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;

	/* Free TX queue resources */
	for (queue = 0; queue < tx_count; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		/* Release the DMA TX socket buffers */
		dma_free_tx_skbufs(priv, queue);

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc)
			dma_free_coherent(priv->device,
					  DMA_TX_SIZE * sizeof(struct dma_desc),
					  tx_q->dma_tx, tx_q->dma_tx_phy);
		else
			dma_free_coherent(priv->device, DMA_TX_SIZE *
					  sizeof(struct dma_extended_desc),
					  tx_q->dma_etx, tx_q->dma_tx_phy);

		kfree(tx_q->tx_skbuff_dma);
		kfree(tx_q->tx_skbuff);
	}
}

/**
 * alloc_dma_rx_desc_resources - alloc RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_rx_desc_resources(struct bstgmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* RX queues buffers and DMA */
	for (queue = 0; queue < rx_count; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->queue_index = queue;
		rx_q->priv_data = priv;

		rx_q->rx_skbuff_dma = kmalloc_array(DMA_RX_SIZE,
						    sizeof(dma_addr_t),
						    GFP_KERNEL);
		if (!rx_q->rx_skbuff_dma)
			goto err_dma;

		rx_q->rx_skbuff = kmalloc_array(DMA_RX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!rx_q->rx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			rx_q->dma_erx = dma_alloc_coherent(priv->device,
							    DMA_RX_SIZE *
							    sizeof(struct
							    dma_extended_desc),
							    &rx_q->dma_rx_phy,
							    GFP_KERNEL);
			if (!rx_q->dma_erx)
				goto err_dma;

		} else {
			rx_q->dma_rx = dma_alloc_coherent(priv->device,
							   DMA_RX_SIZE *
							   sizeof(struct
							   dma_desc),
							   &rx_q->dma_rx_phy,
							   GFP_KERNEL);
			if (!rx_q->dma_rx)
				goto err_dma;
		}
	}

	return 0;

err_dma:
	free_dma_rx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_tx_desc_resources - alloc TX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_tx_desc_resources(struct bstgmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* TX queues buffers and DMA */
	for (queue = 0; queue < tx_count; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		tx_q->queue_index = queue;
		tx_q->priv_data = priv;

		tx_q->tx_skbuff_dma = kmalloc_array(DMA_TX_SIZE,
						    sizeof(*tx_q->tx_skbuff_dma),
						    GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma)
			goto err_dma;

		tx_q->tx_skbuff = kmalloc_array(DMA_TX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!tx_q->tx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			tx_q->dma_etx = dma_alloc_coherent(priv->device,
							    DMA_TX_SIZE *
							    sizeof(struct
							    dma_extended_desc),
							    &tx_q->dma_tx_phy,
							    GFP_KERNEL);
			if (!tx_q->dma_etx)
				goto err_dma;
		} else {
			tx_q->dma_tx = dma_alloc_coherent(priv->device,
							   DMA_TX_SIZE *
							   sizeof(struct
								  dma_desc),
							   &tx_q->dma_tx_phy,
							   GFP_KERNEL);
			if (!tx_q->dma_tx)
				goto err_dma;
		}
	}

	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_desc_resources - alloc TX/RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_desc_resources(struct bstgmac_priv *priv)
{
	/* RX Allocation */
	int ret = alloc_dma_rx_desc_resources(priv);

	if (ret)
		return ret;

	ret = alloc_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * free_dma_desc_resources - free dma desc resources
 * @priv: private structure
 */
static void free_dma_desc_resources(struct bstgmac_priv *priv)
{
	/* Release the DMA RX socket buffers */
	free_dma_rx_desc_resources(priv);

	/* Release the DMA TX socket buffers */
	free_dma_tx_desc_resources(priv);
}

/**
 *  bstgmac_mac_enable_rx_queues - Enable MAC rx queues
 *  @priv: driver private structure
 *  Description: It is used for enabling the rx queues in the MAC
 */
static void bstgmac_mac_enable_rx_queues(struct bstgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	int queue;
	u8 mode;

	for (queue = 0; queue < rx_queues_count; queue++) {
		mode = priv->plat->rx_queues_cfg[queue].mode_to_use;
		bstgmac_rx_queue_enable(priv, priv->hw, mode, queue);
	}
}

/**
 * bstgmac_start_rx_dma - start RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This starts a RX DMA channel
 */
static void bstgmac_start_rx_dma(struct bstgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes started in channel %d\n", chan);
	bstgmac_start_rx(priv, priv->ioaddr, chan);
}

/**
 * bstgmac_start_tx_dma - start TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This starts a TX DMA channel
 */
static void bstgmac_start_tx_dma(struct bstgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes started in channel %d\n", chan);
	bstgmac_start_tx(priv, priv->ioaddr, chan);
}

/**
 * bstgmac_stop_rx_dma - stop RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This stops a RX DMA channel
 */
static void bstgmac_stop_rx_dma(struct bstgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes stopped in channel %d\n", chan);
	bstgmac_stop_rx(priv, priv->ioaddr, chan);
}

/**
 * bstgmac_stop_tx_dma - stop TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This stops a TX DMA channel
 */
static void bstgmac_stop_tx_dma(struct bstgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes stopped in channel %d\n", chan);
	bstgmac_stop_tx(priv, priv->ioaddr, chan);
}

/**
 * bstgmac_start_all_dma - start all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This starts all the RX and TX DMA channels
 */
static void bstgmac_start_all_dma(struct bstgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_channels_count; chan++)
		bstgmac_start_rx_dma(priv, chan);

	for (chan = 0; chan < tx_channels_count; chan++)
		bstgmac_start_tx_dma(priv, chan);
}

/**
 * bstgmac_stop_all_dma - stop all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This stops the RX and TX DMA channels
 */
static void bstgmac_stop_all_dma(struct bstgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_channels_count; chan++)
		bstgmac_stop_rx_dma(priv, chan);

	for (chan = 0; chan < tx_channels_count; chan++)
		bstgmac_stop_tx_dma(priv, chan);
}

/**
 *  bstgmac_dma_operation_mode - HW DMA operation mode
 *  @priv: driver private structure
 *  Description: it is used for configuring the DMA operation mode register in
 *  order to program the tx/rx DMA thresholds or Store-And-Forward mode.
 */
static void bstgmac_dma_operation_mode(struct bstgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;
	u32 txmode = 0;
	u32 rxmode = 0;
	u32 chan = 0;
	u8 qmode = 0;

	if (rxfifosz == 0)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_channels_count;
	txfifosz /= tx_channels_count;

	if (priv->plat->force_thresh_dma_mode) {
		txmode = tc;
		rxmode = tc;
	} else if (priv->plat->force_sf_dma_mode || priv->plat->tx_coe) {
		/*
		 * In case of GMAC, SF mode can be enabled
		 * to perform the TX COE in HW. This depends on:
		 * 1) TX COE if actually supported
		 * 2) There is no bugged Jumbo frame support
		 *    that needs to not insert csum in the TDES.
		 */
		txmode = SF_DMA_MODE;
		rxmode = SF_DMA_MODE;
		priv->xstats.threshold = SF_DMA_MODE;
	} else {
		txmode = tc;
		rxmode = SF_DMA_MODE;
	}

	/* configure all channels */
	for (chan = 0; chan < rx_channels_count; chan++) {
		qmode = priv->plat->rx_queues_cfg[chan].mode_to_use;

		bstgmac_dma_rx_mode(priv, priv->ioaddr, rxmode, chan,
				rxfifosz, qmode);
		bstgmac_set_dma_bfsize(priv, priv->ioaddr, priv->dma_buf_sz,
				chan);
	}

	for (chan = 0; chan < tx_channels_count; chan++) {
		qmode = priv->plat->tx_queues_cfg[chan].mode_to_use;

		bstgmac_dma_tx_mode(priv, priv->ioaddr, txmode, chan,
				txfifosz, qmode);
	}
}

/**
 * bstgmac_tx_clean - to manage the transmission completion
 * @priv: driver private structure
 * @queue: TX queue index
 * Description: it reclaims the transmit resources after transmission completes.
 */
static int bstgmac_tx_clean(struct bstgmac_priv *priv, int budget, u32 queue)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry, count = 0;

	__netif_tx_lock_bh(netdev_get_tx_queue(priv->dev, queue));

	priv->xstats.tx_clean++;

	entry = tx_q->dirty_tx;
	while ((entry != tx_q->cur_tx) && (count < budget)) {
		struct sk_buff *skb = tx_q->tx_skbuff[entry];
		struct dma_desc *p;
		int status;

		if (priv->extend_desc)
			p = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			p = tx_q->dma_tx + entry;

		status = bstgmac_tx_status(priv, &priv->dev->stats,
				&priv->xstats, p, priv->ioaddr);
		/* Check if the descriptor is owned by the DMA */
		if (unlikely(status & tx_dma_own))
			break;

		count++;

		/* Make sure descriptor fields are read after reading
		 * the own bit.
		 */
		dma_rmb();

		/* Just consider the last segment and ...*/
		if (likely(!(status & tx_not_ls))) {
			/* ... verify the status error condition */
			if (unlikely(status & tx_err)) {
				priv->dev->stats.tx_errors++;
			} else {
				priv->dev->stats.tx_packets++;
				priv->xstats.tx_pkt_n++;
			}
			bstgmac_get_tx_hwtstamp(priv, p, skb);
		}

		if (likely(tx_q->tx_skbuff_dma[entry].buf)) {
			if (tx_q->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device,
					       tx_q->tx_skbuff_dma[entry].buf,
					       tx_q->tx_skbuff_dma[entry].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device,
						 tx_q->tx_skbuff_dma[entry].buf,
						 tx_q->tx_skbuff_dma[entry].len,
						 DMA_TO_DEVICE);
			tx_q->tx_skbuff_dma[entry].buf = 0;
			tx_q->tx_skbuff_dma[entry].len = 0;
			tx_q->tx_skbuff_dma[entry].map_as_page = false;
		}

		bstgmac_clean_desc3(priv, tx_q, p);

		tx_q->tx_skbuff_dma[entry].last_segment = false;
		tx_q->tx_skbuff_dma[entry].is_jumbo = false;

		if (likely(skb != NULL)) {
			pkts_compl++;
			bytes_compl += skb->len;
			dev_consume_skb_any(skb);
			tx_q->tx_skbuff[entry] = NULL;
		}

		bstgmac_release_tx_desc(priv, p, priv->mode);

		entry = BSTGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	}
	tx_q->dirty_tx = entry;

	netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, queue),
				  pkts_compl, bytes_compl);

	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,
								queue))) &&
	    bstgmac_tx_avail(priv, queue) > BSTGMAC_TX_THRESH) {

		netif_dbg(priv, tx_done, priv->dev,
			  "%s: restart transmit\n", __func__);
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	if ((priv->eee_enabled) && (!priv->tx_path_in_lpi_mode)) {
		bstgmac_enable_eee_mode(priv);
		mod_timer(&priv->eee_ctrl_timer, BSTGMAC_LPI_T(eee_timer));
	}

	__netif_tx_unlock_bh(netdev_get_tx_queue(priv->dev, queue));

	return count;
}

/**
 * bstgmac_tx_err - to manage the tx error
 * @priv: driver private structure
 * @chan: channel index
 * Description: it cleans the descriptors and restarts the transmission
 * in case of transmission errors.
 */
static void bstgmac_tx_err(struct bstgmac_priv *priv, u32 chan)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[chan];
	int i;

	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, chan));

	bstgmac_stop_tx_dma(priv, chan);
	dma_free_tx_skbufs(priv, chan);
	for (i = 0; i < DMA_TX_SIZE; i++)
		if (priv->extend_desc)
			bstgmac_init_tx_desc(priv, &tx_q->dma_etx[i].basic,
					priv->mode, (i == DMA_TX_SIZE - 1));
		else
			bstgmac_init_tx_desc(priv, &tx_q->dma_tx[i],
					priv->mode, (i == DMA_TX_SIZE - 1));
	tx_q->dirty_tx = 0;
	tx_q->cur_tx = 0;
	tx_q->mss = 0;
	netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, chan));
	bstgmac_start_tx_dma(priv, chan);

	priv->dev->stats.tx_errors++;
	netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, chan));
}

/**
 *  bstgmac_set_dma_operation_mode - Set DMA operation mode by channel
 *  @priv: driver private structure
 *  @txmode: TX operating mode
 *  @rxmode: RX operating mode
 *  @chan: channel index
 *  Description: it is used for configuring of the DMA operation mode in
 *  runtime in order to program the tx/rx DMA thresholds or Store-And-Forward
 *  mode.
 */
static void bstgmac_set_dma_operation_mode(struct bstgmac_priv *priv, u32 txmode,
					  u32 rxmode, u32 chan)
{
	u8 rxqmode = priv->plat->rx_queues_cfg[chan].mode_to_use;
	u8 txqmode = priv->plat->tx_queues_cfg[chan].mode_to_use;
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;

	if (rxfifosz == 0)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_channels_count;
	txfifosz /= tx_channels_count;

	bstgmac_dma_rx_mode(priv, priv->ioaddr, rxmode, chan, rxfifosz, rxqmode);
	bstgmac_dma_tx_mode(priv, priv->ioaddr, txmode, chan, txfifosz, txqmode);
}

static bool bstgmac_safety_feat_interrupt(struct bstgmac_priv *priv)
{
	int ret;

	ret = bstgmac_safety_feat_irq_status(priv, priv->dev,
			priv->ioaddr, priv->dma_cap.asp, &priv->sstats);
	if (ret && (ret != -EINVAL)) {
		return true;
		bstgmac_global_err(priv);
		return true;
	}

	return false;
}

static int bstgmac_napi_check(struct bstgmac_priv *priv, u32 chan)
{
	u32 flag = 0;
    int status;
	struct bstgmac_channel *ch;
	bool needs_work = false;

    flag = priv->plat->dma_cfg->dma_int_mode;
    ch = &priv->rx_channel[chan];
    status = bstgmac_dma_interrupt_status(priv, priv->ioaddr,
						 &priv->xstats, chan, flag);
	if ((status & handle_rx) && ch->has_rx) {
		needs_work = true;
	} else {
		status &= ~handle_rx;
	}

	if ((status & handle_tx) && ch->has_tx) {
		needs_work = true;
	} else {
		status &= ~handle_tx;
	}

	if (needs_work && napi_schedule_prep(&ch->napi)) {
		bstgmac_disable_dma_irq(priv, priv->ioaddr, chan);
		__napi_schedule(&ch->napi);
	}

	return status;
}

/**
 * bstgmac_dma_interrupt - DMA ISR
 * @priv: driver private structure
 * Description: this is the DMA ISR. It is called by the main ISR.
 * It calls the dwmac dma routine and schedule poll method in case of some
 * work can be done.
 */
static void bstgmac_dma_interrupt(struct bstgmac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	u32 rx_channel_count = priv->plat->rx_queues_to_use;
	u32 channels_to_check = tx_channel_count > rx_channel_count ?
				tx_channel_count : rx_channel_count;
	u32 chan;
	int status[max_t(u32, MTL_MAX_TX_QUEUES, MTL_MAX_RX_QUEUES)];

	/* Make sure we never check beyond our status buffer. */
	if (WARN_ON_ONCE(channels_to_check > ARRAY_SIZE(status)))
		channels_to_check = ARRAY_SIZE(status);

	for (chan = 0; chan < channels_to_check; chan++)
		status[chan] = bstgmac_napi_check(priv, chan);

	for (chan = 0; chan < tx_channel_count; chan++) {
		if (unlikely(status[chan] & tx_hard_error_bump_tc)) {
			/* Try to bump up the dma threshold on this failure */
			if (unlikely(priv->xstats.threshold != SF_DMA_MODE) &&
			    (tc <= 256)) {
				tc += 64;
				if (priv->plat->force_thresh_dma_mode)
					bstgmac_set_dma_operation_mode(priv,
								      tc,
								      tc,
								      chan);
				else
					bstgmac_set_dma_operation_mode(priv,
								    tc,
								    SF_DMA_MODE,
								    chan);
				priv->xstats.threshold = tc;
			}
		} else if (unlikely(status[chan] == tx_hard_error)) {
			bstgmac_tx_err(priv, chan);
		}
	}
}

/**
 * bstgmac_mmc_setup: setup the Mac Management Counters (MMC)
 * @priv: driver private structure
 * Description: this masks the MMC irq, in fact, the counters are managed in SW.
 */
static void bstgmac_mmc_setup(struct bstgmac_priv *priv)
{
	unsigned int mode = MMC_CNTRL_RESET_ON_READ | MMC_CNTRL_COUNTER_RESET |
			    MMC_CNTRL_PRESET | MMC_CNTRL_FULL_HALF_PRESET;

	dwmac_mmc_intr_all_mask(priv->mmcaddr);

	if (priv->dma_cap.rmon) {
		dwmac_mmc_ctrl(priv->mmcaddr, mode);
		memset(&priv->mmc, 0, sizeof(struct bstgmac_counters));
	} else
		netdev_info(priv->dev, "No MAC Management Counters available\n");
}

/**
 * bstgmac_get_hw_features - get MAC capabilities from the HW cap. register.
 * @priv: driver private structure
 * Description:
 *  new GMAC chip generations have a new register to indicate the
 *  presence of the optional feature/functions.
 *  This can be also used to override the value passed through the
 *  platform and necessary for old MAC10/100 and GMAC chips.
 */
static int bstgmac_get_hw_features(struct bstgmac_priv *priv)
{
	return bstgmac_get_hw_feature(priv, priv->ioaddr, &priv->dma_cap) == 0;
}

int gmac0_dump_register(void)
{
	void __iomem *gmac0_addr;
	gmac0_addr = ioremap(BST_GMAC_0_BASE_ADDR, SIZE_8K);
	if(!gmac0_addr){
		printk(KERN_ERR "ioremap failed in fun: gmac0_dump_register! \n");
		return -1;
	}
	printk(KERN_DEBUG "gmac0_dump:\n");
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CONTROL_REG                        = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CONTROL_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_TX_CONTROL_REG                     = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_TX_CONTROL_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RX_CONTROL_REG                     = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RX_CONTROL_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_TXDESC_LIST_HADDERSS_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_TXDESC_LIST_HADDERSS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_TXDESC_LIST_ADDERSS_REG            = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_TXDESC_LIST_ADDERSS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RXDESC_LIST_HADDERSS_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RXDESC_LIST_HADDERSS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RXDESC_LIST_ADDERSS_REG            = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RXDESC_LIST_ADDERSS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_TXDESC_TAIL_POINTERT_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_TXDESC_TAIL_POINTERT_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RXDESC_TAIL_POINTERT_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RXDESC_TAIL_POINTERT_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_TXDESC_RING_LENGTH_REG             = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_TXDESC_RING_LENGTH_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RXDESC_RING_LENGTH_REG             = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RXDESC_RING_LENGTH_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_INTERRUPT_ENABLE_REG               = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_INTERRUPT_ENABLE_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER_REG    = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS_REG   = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_TXDESC_REG             = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_TXDESC_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_RXDESC_REG             = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_RXDESC_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_TXBUFFER_H_REG         = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_H_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_TXBUFFER_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_RXBUFFER_H_REG         = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_H_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_CURRENT_APP_RXBUFFER_REG           = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_STATUS_REG                         = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_STATUS_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_MISS_FRAME_CNT_REG                 = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_MISS_FRAME_CNT_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RXP_ACCEPT_CNT_REG                 = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RXP_ACCEPT_CNT_REG));
	printk(KERN_DEBUG "GMAC0_DMA_CH0_RX_ERI_CNT_REG                     = 0x%08x\n",  readl(gmac0_addr + GMAC_DMA_CH0_RX_ERI_CNT_REG));
	iounmap(BST_GMAC_0_BASE_ADDR);
	return 0;
}

int gmac1_dump_register(void)
{
	void __iomem *gmac1_addr;
	gmac1_addr = ioremap(BST_GMAC_1_BASE_ADDR, SIZE_8K);
	if(! gmac1_addr){
		printk(KERN_ERR "ioremap failed in fun: gmac1_dump_register! \n");
		return -1;
	}
	printk(KERN_DEBUG "ggmac1_dump:\n");
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CONTROL_REG                        = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CONTROL_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_TX_CONTROL_REG                     = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_TX_CONTROL_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RX_CONTROL_REG                     = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RX_CONTROL_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_TXDESC_LIST_HADDERSS_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_TXDESC_LIST_HADDERSS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_TXDESC_LIST_ADDERSS_REG            = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_TXDESC_LIST_ADDERSS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RXDESC_LIST_HADDERSS_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RXDESC_LIST_HADDERSS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RXDESC_LIST_ADDERSS_REG            = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RXDESC_LIST_ADDERSS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_TXDESC_TAIL_POINTERT_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_TXDESC_TAIL_POINTERT_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RXDESC_TAIL_POINTERT_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RXDESC_TAIL_POINTERT_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_TXDESC_RING_LENGTH_REG             = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_TXDESC_RING_LENGTH_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RXDESC_RING_LENGTH_REG             = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RXDESC_RING_LENGTH_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_INTERRUPT_ENABLE_REG               = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_INTERRUPT_ENABLE_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER_REG    = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS_REG   = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_TXDESC_REG             = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_TXDESC_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_RXDESC_REG             = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_RXDESC_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_TXBUFFER_H_REG         = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_H_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_TXBUFFER_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_RXBUFFER_H_REG         = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_H_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_CURRENT_APP_RXBUFFER_REG           = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_STATUS_REG                         = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_STATUS_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_MISS_FRAME_CNT_REG                 = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_MISS_FRAME_CNT_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RXP_ACCEPT_CNT_REG                 = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RXP_ACCEPT_CNT_REG));
	printk(KERN_DEBUG "GMAC1_DMA_CH0_RX_ERI_CNT_REG                     = 0x%08x\n",  readl(gmac1_addr + GMAC_DMA_CH0_RX_ERI_CNT_REG));
	iounmap(BST_GMAC_1_BASE_ADDR);
	return 0;
}

void gmac_dump_register(void)
{
	//if gmac0 up
	if (gmac_priv_g[0] != NULL) {
		if (netif_carrier_ok(gmac_priv_g[0]->dev)) {
			printk(KERN_DEBUG "eth0 up,gmac0 register_dump:\n");
		    gmac0_dump_register();
		}
	}
	//if gmac1 up
	if (gmac_priv_g[1] != NULL) {
		if ((netif_running(gmac_priv_g[1]->dev)) && netif_carrier_ok(gmac_priv_g[1]->dev)) {
			printk(KERN_DEBUG "eth1 up,gmac1 register_dump:\n");
		    gmac1_dump_register();
		}
	}
	if((gmac_priv_g[0] == NULL) && (gmac_priv_g[1] == NULL)) 
		printk(KERN_DEBUG "gmac0 and gmac1 all down!\n");
}
EXPORT_SYMBOL(gmac_dump_register);

static void mactobuf(u64 data, char *buf)
{      
    if (!buf) {
        return;
    }

    buf[0] = (data >> 40) & 0xff;
    buf[1] = (data >> 32) & 0xff;
    buf[2] = (data >> 24) & 0xff;
    buf[3] = (data >> 16) & 0xff;
    buf[4] = (data >> 8) & 0xff;
    buf[5] = data & 0xff;
}
/**
 * bstgmac_check_ether_addr - check if the MAC addr is valid
 * @priv: driver private structure
 * Description:
 * it is to verify if the MAC address is valid, in case of failures it
 * generates a random MAC address
 */
static void bstgmac_check_ether_addr(struct bstgmac_priv *priv)
{
    u64 mac;

	if (!is_valid_ether_addr(priv->dev->dev_addr)) {
		bstgmac_get_umac_addr(priv, priv->hw, priv->dev->dev_addr, 0);
		if (!is_valid_ether_addr(priv->dev->dev_addr)) {
            eth_hw_addr_random(priv->dev);
        }
		netdev_info(priv->dev, "device MAC address %pM\n",
			    priv->dev->dev_addr);
	}
}

/**
 * bstgmac_init_dma_engine - DMA init.
 * @priv: driver private structure
 * Description:
 * It inits the DMA invoking the specific MAC/GMAC callback.
 * Some DMA parameters can be passed from the platform;
 * in case of these are not passed a default is kept for the MAC or GMAC.
 */
static int bstgmac_init_dma_engine(struct bstgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 dma_csr_ch = max(rx_channels_count, tx_channels_count);
	struct bstgmac_rx_queue *rx_q;
	struct bstgmac_tx_queue *tx_q;
	u32 chan = 0;
	int atds = 0;
	int ret = 0;
	if (!priv->plat->dma_cfg || !priv->plat->dma_cfg->pbl) {
		dev_err(priv->device, "Invalid DMA configuration\n");
		return -EINVAL;
	}

	if (priv->extend_desc && (priv->mode == BSTGMAC_RING_MODE))
		atds = 1;

	ret = bstgmac_reset(priv, priv->ioaddr);
	if (ret) {
		dev_err(priv->device, "Failed to reset the dma\n");
		return ret;
	}

	/* DMA Configuration */
	bstgmac_dma_init(priv, priv->ioaddr, priv->plat->dma_cfg, atds);

	if (priv->plat->axi)
		bstgmac_axi(priv, priv->ioaddr, priv->plat->axi);

	/* DMA CSR Channel configuration */
	for (chan = 0; chan < dma_csr_ch; chan++)
		bstgmac_init_chan(priv, priv->ioaddr, priv->plat->dma_cfg, chan);

	/* DMA RX Channel Configuration */
	for (chan = 0; chan < rx_channels_count; chan++) {
		rx_q = &priv->rx_queue[chan];

		bstgmac_init_rx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    rx_q->dma_rx_phy, chan);

		rx_q->rx_tail_addr = rx_q->dma_rx_phy +
			    (DMA_RX_SIZE * sizeof(struct dma_desc));
		bstgmac_set_rx_tail_ptr(priv, priv->ioaddr,
				       rx_q->rx_tail_addr, chan);
	}

	/* DMA TX Channel Configuration */
	for (chan = 0; chan < tx_channels_count; chan++) {
		tx_q = &priv->tx_queue[chan];

		bstgmac_init_tx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    tx_q->dma_tx_phy, chan);

		tx_q->tx_tail_addr = tx_q->dma_tx_phy;
		bstgmac_set_tx_tail_ptr(priv, priv->ioaddr,
				       tx_q->tx_tail_addr, chan);
	}
	return ret;
}

static void bstgmac_tx_timer_arm(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

	mod_timer(&tx_q->txtimer, BSTGMAC_COAL_TIMER(priv->tx_coal_timer));
}

/**
 * bstgmac_tx_timer - mitigation sw timer for tx.
 * @data: data pointer
 * Description:
 * This is the timer handler to directly invoke the bstgmac_tx_clean.
 */
static void bstgmac_tx_timer(struct timer_list *t)
{
	struct bstgmac_tx_queue *tx_q = from_timer(tx_q, t, txtimer);
	struct bstgmac_priv *priv = tx_q->priv_data;
	struct bstgmac_channel *ch;

	ch = &priv->tx_channel[tx_q->queue_index];
	if(ch->int_mode == DMA_INT_M_0){
		if (likely(napi_schedule_prep(&ch->napi)))
			__napi_schedule(&ch->napi);
	}else{
        queue_work(priv->tx_wq, &ch->tx_work);
    }
}
	
static void bstgmac_init_rxmem(struct bstgmac_priv *priv, int busid)
{
    int i, chan, index;
    struct sk_buff *skb; 
    dma_addr_t buf;
    int size = priv->dma_buf_sz;
    struct sk_buff_head *list;
    dma_addr_t *addr;

    if (busid > BSTGMAC_CORE_NUM) {
        return;
    }
    
    for (chan = 0; chan < BSTGMAC_RXCHAN_NUM; chan++) {
        index = busid * BSTGMAC_RXCHAN_NUM + chan;
        list = &gmac_delivery_skblist[index];
        for (i = 0; i < BSTGMAC_RXMEM_MAX; i++) {
            skb = netdev_alloc_skb_ip_align(priv->dev, size);
            if (unlikely(!skb)) {
                break;
            }
            buf = dma_map_single(priv->device, skb->data, size, DMA_FROM_DEVICE);
            if (dma_mapping_error(priv->device, buf)) {
                netdev_err(priv->dev, "Gmac map list failed\n");
                dev_kfree_skb(skb);
                break;
            }         
            addr = (dma_addr_t *)skb->cb;
            *addr = buf;                     
            dma_wmb();
            skb_queue_tail(list, skb);
        }    
    }
}

/**
 * bstgmac_init_tx_coalesce - init tx mitigation options.
 * @priv: driver private structure
 * Description:
 * This inits the transmit coalesce parameters: i.e. timer rate,
 * timer handler and default threshold used for enabling the
 * interrupt on completion bit.
 */
static void bstgmac_init_tx_coalesce(struct bstgmac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	u32 chan;

	priv->tx_coal_frames = BSTGMAC_TX_FRAMES;
	priv->tx_coal_timer = BSTGMAC_COAL_TX_TIMER;

	for (chan = 0; chan < tx_channel_count; chan++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[chan];

		timer_setup(&tx_q->txtimer, bstgmac_tx_timer, 0);
	}
}

static void bstgmac_set_rings_length(struct bstgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan;

	/* set TX ring length */
	for (chan = 0; chan < tx_channels_count; chan++)
		bstgmac_set_tx_ring_len(priv, priv->ioaddr,
				(DMA_TX_SIZE - 1), chan);

	/* set RX ring length */
	for (chan = 0; chan < rx_channels_count; chan++)
		bstgmac_set_rx_ring_len(priv, priv->ioaddr,
				(DMA_RX_SIZE - 1), chan);
}

/**
 *  bstgmac_set_tx_queue_weight - Set TX queue weight
 *  @priv: driver private structure
 *  Description: It is used for setting TX queues weight
 */
static void bstgmac_set_tx_queue_weight(struct bstgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 weight;
	u32 queue;

	for (queue = 0; queue < tx_queues_count; queue++) {
		weight = priv->plat->tx_queues_cfg[queue].weight;
		bstgmac_set_mtl_tx_queue_weight(priv, priv->hw, weight, queue);
	}
}

/**
 *  bstgmac_configure_cbs - Configure CBS in TX queue
 *  @priv: driver private structure
 *  Description: It is used for configuring CBS in AVB TX queues
 */
static void bstgmac_configure_cbs(struct bstgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 mode_to_use;
	u32 queue;

	/* queue 0 is reserved for legacy traffic */
	for (queue = 1; queue < tx_queues_count; queue++) {
		mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
		if (mode_to_use == MTL_QUEUE_DCB)
			continue;

		bstgmac_config_cbs(priv, priv->hw,
				priv->plat->tx_queues_cfg[queue].send_slope,
				priv->plat->tx_queues_cfg[queue].idle_slope,
				priv->plat->tx_queues_cfg[queue].high_credit,
				priv->plat->tx_queues_cfg[queue].low_credit,
				queue);
	}
}

/**
 *  bstgmac_rx_queue_dma_chan_map - Map RX queue to RX dma channel
 *  @priv: driver private structure
 *  Description: It is used for mapping RX queues to RX dma channels
 */
static void bstgmac_rx_queue_dma_chan_map(struct bstgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 chan;

	for (queue = 0; queue < rx_queues_count; queue++) {
		chan = priv->plat->rx_queues_cfg[queue].chan;
		bstgmac_map_mtl_to_dma(priv, priv->hw, queue, chan);
	}
}

/**
 *  bstgmac_mac_config_rx_queues_prio - Configure RX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX Queue Priority
 */
static void bstgmac_mac_config_rx_queues_prio(struct bstgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 prio;

	for (queue = 0; queue < rx_queues_count; queue++) {
		if (!priv->plat->rx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->rx_queues_cfg[queue].prio;
		bstgmac_rx_queue_prio(priv, priv->hw, prio, queue);
	}
}

/**
 *  bstgmac_mac_config_tx_queues_prio - Configure TX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the TX Queue Priority
 */
static void bstgmac_mac_config_tx_queues_prio(struct bstgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;
	u32 prio;

	for (queue = 0; queue < tx_queues_count; queue++) {
		if (!priv->plat->tx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->tx_queues_cfg[queue].prio;
		bstgmac_tx_queue_prio(priv, priv->hw, prio, queue);
	}
}

/**
 *  bstgmac_mac_config_rx_queues_routing - Configure RX Queue Routing
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX queue routing
 */
static void bstgmac_mac_config_rx_queues_routing(struct bstgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u8 packet;

	for (queue = 0; queue < rx_queues_count; queue++) {
		/* no specific packet type routing specified for the queue */
		if (priv->plat->rx_queues_cfg[queue].pkt_route == 0x0)
			continue;

		packet = priv->plat->rx_queues_cfg[queue].pkt_route;
		bstgmac_rx_queue_routing(priv, priv->hw, packet, queue);
	}
}

/**
 *  bstgmac_mtl_configuration - Configure MTL
 *  @priv: driver private structure
 *  Description: It is used for configurring MTL
 */
static void bstgmac_mtl_configuration(struct bstgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;

	if (tx_queues_count > 1)
		bstgmac_set_tx_queue_weight(priv);

	/* Configure MTL RX algorithms */
	if (rx_queues_count > 1)
		bstgmac_prog_mtl_rx_algorithms(priv, priv->hw,
				priv->plat->rx_sched_algorithm);

	/* Configure MTL TX algorithms */
	if (tx_queues_count > 1)
		bstgmac_prog_mtl_tx_algorithms(priv, priv->hw,
				priv->plat->tx_sched_algorithm);

	/* Configure CBS in AVB TX queues */
	if (tx_queues_count > 1)
		bstgmac_configure_cbs(priv);

	/* Map RX MTL to DMA channels */
	bstgmac_rx_queue_dma_chan_map(priv);

	/* Enable MAC RX Queues */
	bstgmac_mac_enable_rx_queues(priv);

	/* Set RX priorities */
	if (rx_queues_count > 1)
		bstgmac_mac_config_rx_queues_prio(priv);

	/* Set TX priorities */
	if (tx_queues_count > 1)
		bstgmac_mac_config_tx_queues_prio(priv);

	/* Set RX routing */
	if (rx_queues_count > 1)
		bstgmac_mac_config_rx_queues_routing(priv);
}

static void bstgmac_safety_feat_configuration(struct bstgmac_priv *priv)
{
	unsigned int asp = priv->dma_cap.asp;
	if(priv->plat->fix_safety != ASP_HW){
		asp = priv->plat->fix_safety;
		netdev_info(priv->dev, "Safety Features Fix to %d.Hw feature %d\n",asp,priv->dma_cap.asp);
	}
	if (asp) {
		netdev_info(priv->dev, "Enabling Safety Features\n");
		bstgmac_safety_feat_config(priv, priv->ioaddr,asp);
	} else {
		netdev_info(priv->dev, "No Safety Features support found\n");
	}
}

/**
 * bstgmac_hw_setup - setup mac in a usable state.
 *  @dev : pointer to the device structure.
 *  Description:
 *  this is the main function to setup the HW in a usable state because the
 *  dma engine is reset, the core registers are configured (e.g. AXI,
 *  Checksum features, timers). The DMA is ready to start receiving and
 *  transmitting.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstgmac_hw_setup(struct net_device *dev, bool init_ptp)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 chan;
	int ret;

	/* DMA initialization and SW reset */
	ret = bstgmac_init_dma_engine(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA engine initialization failed\n",
			   __func__);
		return ret;
	}

	/* Copy the MAC addr into the HW  */
	bstgmac_set_umac_addr(priv, priv->hw, dev->dev_addr, 0);

	/* PS and related bits will be programmed according to the speed */
	if (priv->hw->pcs) {
		int speed = priv->plat->mac_port_sel_speed;

		if ((speed == SPEED_10) || (speed == SPEED_100) ||
		    (speed == SPEED_1000)) {
			priv->hw->ps = speed;
		} else {
			dev_warn(priv->device, "invalid port speed\n");
			priv->hw->ps = 0;
		}
	}

	/* Initialize the MAC Core */
	bstgmac_core_init(priv, priv->hw, dev);

	/* Initialize MTL*/
	bstgmac_mtl_configuration(priv);

	/* Initialize Safety Features */
	bstgmac_safety_feat_configuration(priv);

	ret = bstgmac_rx_ipc(priv, priv->hw);
	if (!ret) {
		netdev_warn(priv->dev, "RX IPC Checksum Offload disabled\n");
		priv->plat->rx_coe = STMMAC_RX_COE_NONE;
		priv->hw->rx_csum = 0;
	}

	/* Enable the MAC Rx/Tx */
	bstgmac_mac_set(priv, priv->ioaddr, true);

	/* Set the HW DMA mode and the COE */
	bstgmac_dma_operation_mode(priv);

	bstgmac_mmc_setup(priv);

	if (init_ptp) {
		ret = clk_prepare_enable(priv->plat->clk_ptp_ref);
		if (ret < 0)
			netdev_warn(priv->dev, "failed to enable PTP reference clock: %d\n", ret);

		ret = bstgmac_init_ptp(priv);
		if (ret == -EOPNOTSUPP)
			netdev_warn(priv->dev, "PTP not supported by HW\n");
		else if (ret)
			netdev_warn(priv->dev, "PTP init failed\n");
	}

	priv->tx_lpi_timer = BSTGMAC_DEFAULT_TWT_LS;

	if (priv->use_riwt) {
		ret = bstgmac_rx_watchdog(priv, priv->ioaddr, MAX_DMA_RIWT, rx_cnt);
		if (!ret)
			priv->rx_riwt = MAX_DMA_RIWT;
	}

	if (priv->hw->pcs)
		bstgmac_pcs_ctrl_ane(priv, priv->hw, 1, priv->hw->ps, 0);

	/* set TX and RX rings length */
	bstgmac_set_rings_length(priv);

	/* Enable TSO */
	if (priv->tso) {
		for (chan = 0; chan < tx_cnt; chan++)
			bstgmac_enable_tso(priv, priv->ioaddr, 1, chan);
	}
 
 	/* VLAN Tag Insertion */
        if (priv->dma_cap.vlins)
                bstgmac_enable_vlan(priv, priv->hw, BSTGMAC_VLAN_INSERT);

	/* Start the ball rolling... */
	bstgmac_start_all_dma(priv);

	return 0;
}

static void bstgmac_hw_teardown(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	clk_disable_unprepare(priv->plat->clk_ptp_ref);
}

static void bstgmac_get_board_para(struct bstgmac_priv *priv, struct bstgmac_board_para *para)
{
    switch (priv->extend_op) {
    case BSTA1000_BOARD_EVB:
    case BSTA1000_BOARD_FADA:
    case BSTA1000_BOARD_FADB:
        para->portid = BSTGMAC1_BUS_ID;
        para->phyrole = gmac1_phyrole;
        para->valid = 1;
        break;
    case BSTA1000_BOARD_EC:
	case BSTA1000B_BOARD_EC:
	case BSTA1000_BOARD_ECV3:
	case BSTA1000_BOARD_ECU:
	case BSTA1000B_BOARD_EVB:
        para->valid = 0;
        break;
    case BSTA1000_BOARD_FAWA:
    case BSTA1000_BOARD_FAWB:
    case BSTA1000_BOARD_JAC20:
    case BSTA1000_BOARD_JAC21:
    case BSTA1000B_BOARD_JAC21:
        para->portid = BSTGMAC0_BUS_ID;
        para->phyrole = gmac0_phyrole;
        para->valid = 1;
        break;

    default:
        break;
    }

	if (t1_phyrole) {
		para->phyrole = t1_phyrole;
	}
}

/**
 *  bstgmac_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstgmac_open(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 chan;
	int ret;
    struct cpumask mask;
    
    memset (&bstgmac_para[priv->plat->bus_id], 0, sizeof(struct bstgmac_board_para));
    bstgmac_get_board_para(priv, &bstgmac_para[priv->plat->bus_id]);
    if (bstgmac_para[priv->plat->bus_id].valid) {
        if ((priv->plat->bus_id == bstgmac_para[priv->plat->bus_id].portid) && (bstgmac_para[priv->plat->bus_id].phyrole == 0)) {
            /* set slave flag before open to prevent IPv6 addrconf */
            dev->flags |= IFF_SLAVE;
            return -ENODEV;
        } else if ((priv->plat->bus_id == bstgmac_para[priv->plat->bus_id].portid) && (bstgmac_para[priv->plat->bus_id].phyrole != 0)) {
            dev->flags &= ~IFF_SLAVE;
        }
    }

	if (priv->hw->pcs != BSTGMAC_PCS_TBI &&
	    priv->hw->pcs != BSTGMAC_PCS_RTBI) {
		ret = bstgmac_init_phy(dev);
		if (ret) {
			netdev_err(priv->dev,
				   "%s: Cannot attach to PHY (error: %d)\n",
				   __func__, ret);
			return ret;
		}
	}

	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct bstgmac_extra_stats));
	priv->xstats.threshold = tc;

	priv->dma_buf_sz = BSTGMAC_ALIGN(buf_sz);
	priv->rx_copybreak = BSTGMAC_RX_COPYBREAK;

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

	ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}

	ret = bstgmac_hw_setup(dev, true);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Hw setup failed\n", __func__);
		goto init_error;
	}

	bstgmac_init_tx_coalesce(priv);

	phylink_start(priv->phylink);
	/* We may have called phylink_speed_down before */
	phylink_speed_up(priv->phylink);

	/* Request the IRQ lines */
	ret = request_irq(dev->irq, bstgmac_sbd_interrupt,
			  IRQF_SHARED, dev->name, dev);
	if (unlikely(ret < 0)) {
		netdev_err(priv->dev,
			   "%s: ERROR: allocating the IRQ %d (error: %d)\n",
			   __func__, dev->irq, ret);
		goto irq_error;
	}

	/* Request the  lpi IRQ lines */
	if (priv->lpi_irq > 0) {
		ret = request_irq(priv->lpi_irq, bstgmac_lpi_interrupt, IRQF_SHARED,
				  dev->name, dev);
		if (unlikely(ret < 0)) {
			netdev_err(priv->dev,
				   "%s: ERROR: allocating the LPI IRQ %d (%d)\n",
				   __func__, priv->lpi_irq, ret);
			goto lpiirq_error;
		}
	}
	/* Request the safety CE IRQ lines */
	if (priv->sfty_ce_irq > 0) {
		ret = request_irq(priv->sfty_ce_irq, bstgmac_safety_interrupt, IRQF_SHARED,
				  dev->name, dev);
		if (unlikely(ret < 0)) {
			netdev_err(priv->dev,
				   "%s: ERROR: allocating the sfty_ce_irq IRQ %d (%d)\n",
				   __func__, priv->sfty_ce_irq, ret);
			goto sftyceirq_error;
		}
	}	

	/* Request the safety UC IRQ lines */
	if (priv->sfty_uc_irq > 0) {
		ret = request_irq(priv->sfty_uc_irq, bstgmac_safety_interrupt, IRQF_SHARED,
				  dev->name, dev);
		if (unlikely(ret < 0)) {
			netdev_err(priv->dev,
				   "%s: ERROR: allocating the sfty_uc_irq IRQ %d (%d)\n",
				   __func__, priv->sfty_uc_irq, ret);
			goto sftyucirq_error;
		}
	} 

	if(priv->plat->dma_cfg->dma_int_mode != DMA_INT_M_0 ){
		for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++){
			/* Request the TX chan IRQ lines */
			if (priv->perch_tx_irq[chan] > 0) 
			{
				ret = request_irq(priv->perch_tx_irq[chan], bstgmac_tx_interrupt, IRQF_SHARED,dev->name, dev);
				if (unlikely(ret < 0)) {
					netdev_err(priv->dev,
						   "%s: ERROR: allocating the perch_tx_irq[chan] IRQ %d (%d)\n",
						   __func__, priv->perch_tx_irq[chan], ret);
					goto tx_irq_error;
				}
				netdev_info(priv->dev,
							"Request Tx chan:%d irq:%d.\n",chan,priv->perch_tx_irq[chan]);
			}
		}

		for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++){
			/* Request the RX chan IRQ lines */
			if (priv->perch_rx_irq[chan] > 0) 
			{
#if BSTGMAC_RX_INTERRUPT_THREAD
				ret = request_irq(priv->perch_rx_irq[chan], bstgmac_rx_interrupt, IRQF_SHARED,dev->name, dev);
#else
				ret = request_threaded_irq(priv->perch_rx_irq[chan], bstgmac_rx_interrupt, bstgmac_rx_interrupt_threading_null, IRQF_SHARED,dev->name, dev);
#endif
				if (unlikely(ret < 0)) {
					netdev_err(priv->dev,
						   "%s: ERROR: allocating the perch_rx_irq[chan] IRQ %d (%d)\n",
						   __func__, priv->perch_rx_irq[chan], ret);
					goto rx_irq_error;
				}
                if (priv->plat->bus_id == 1) {
                    cpumask_clear(&mask);
                    cpumask_set_cpu(1, &mask);
                    ret = irq_set_affinity_hint(priv->perch_rx_irq[chan], &mask);
                }
				netdev_info(priv->dev,
							"Request Rx chan:%d irq:%d.\n",chan,priv->perch_rx_irq[chan]);
			}
		}
	}
	bstgmac_init_rxmem(priv, priv->plat->bus_id);

	bstgmac_enable_all_queues(priv);

	netif_tx_start_all_queues(priv->dev);
	return 0;

rx_irq_error:
	while (chan-- > 0){
		if (priv->plat->bus_id == 1) {
        	irq_set_affinity_hint(priv->perch_rx_irq[chan], NULL);
		}
		free_irq(priv->perch_rx_irq[chan], dev);
	}
	chan =  priv->plat->tx_queues_to_use;
tx_irq_error:
	while (chan-- > 0){
		free_irq(priv->perch_tx_irq[chan], dev);
	}
	free_irq(priv->sfty_uc_irq, dev);
sftyucirq_error:
	free_irq(priv->sfty_ce_irq, dev);
sftyceirq_error:
	free_irq(priv->lpi_irq, dev);
lpiirq_error:
	free_irq(dev->irq, dev);
irq_error:
	if (dev->phydev)
		phy_stop(dev->phydev);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		del_timer_sync(&priv->tx_queue[chan].txtimer);

	bstgmac_hw_teardown(dev);
init_error:
	free_dma_desc_resources(priv);
dma_desc_error:
	if (dev->phydev)
		phy_disconnect(dev->phydev);

	return ret;
}

/**
 *  bstgmac_release - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int bstgmac_release(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 chan;

	if (priv->eee_enabled)
		del_timer_sync(&priv->eee_ctrl_timer);

	/* Stop and disconnect the PHY */
	if (dev->phydev) {
		phy_stop(dev->phydev);
		phy_disconnect(dev->phydev);
	}

	bstgmac_disable_all_queues(priv);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		del_timer_sync(&priv->tx_queue[chan].txtimer);

	/* Free the IRQ lines */
	free_irq(dev->irq, dev);
	if (priv->sfty_ce_irq != dev->irq)
		free_irq(priv->sfty_ce_irq, dev);
	if (priv->sfty_uc_irq != dev->irq)
		free_irq(priv->sfty_uc_irq, dev);
	if (priv->lpi_irq > 0)
		free_irq(priv->lpi_irq, dev);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++) {
		if (priv->perch_tx_irq[chan] > 0) {
			free_irq(priv->perch_tx_irq[chan], dev);
		}
	}
	for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
		if (priv->perch_rx_irq[chan] > 0) {
			if (priv->plat->bus_id == 1) {
            	irq_set_affinity_hint(priv->perch_rx_irq[chan], NULL);
			}
			free_irq(priv->perch_rx_irq[chan], dev);
		}
	}
	/* Stop TX/RX DMA and clear the descriptors */
	bstgmac_stop_all_dma(priv);

	/* Release and free the Rx/Tx resources */
	free_dma_desc_resources(priv);
	free_dma_rx_mem_res(priv);
	
	/* Disable the MAC Rx/Tx */
	bstgmac_mac_set(priv, priv->ioaddr, false);

	netif_carrier_off(dev);

	bstgmac_release_ptp(priv);

	return 0;
}

static bool bstgmac_vlan_insert(struct bstgmac_priv *priv, struct sk_buff *skb,
			       struct bstgmac_tx_queue *tx_q)
{
	u16 tag = 0x0, inner_tag = 0x0;
	u32 inner_type = 0x0;
	struct dma_desc *p;

	if (!priv->dma_cap.vlins)
		return false;
	if (!skb_vlan_tag_present(skb))
		return false;
	if (skb->vlan_proto == htons(ETH_P_8021AD)) {
		inner_tag = skb_vlan_tag_get(skb);
		inner_type = BSTGMAC_VLAN_INSERT;
	}

	tag = skb_vlan_tag_get(skb);

	p = tx_q->dma_tx + tx_q->cur_tx;
	if (bstgmac_set_desc_vlan_tag(priv, p, tag, inner_tag, inner_type))
		return false;

	bstgmac_set_tx_owner(priv, p);
	tx_q->cur_tx = BSTGMAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
	return true;
}

/**
 *  bstgmac_tso_allocator - close entry point of the driver
 *  @priv: driver private structure
 *  @des: buffer start address
 *  @total_len: total length to fill in descriptors
 *  @last_segmant: condition for the last descriptor
 *  @queue: TX queue index
 *  Description:
 *  This function fills descriptor and request new descriptors according to
 *  buffer length to fill
 */
static void bstgmac_tso_allocator(struct bstgmac_priv *priv, unsigned int des,
				 int total_len, bool last_segment, u32 queue)
{
	struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];
	struct dma_desc *desc;
	u32 buff_size;
	int tmp_len;

	tmp_len = total_len;

	while (tmp_len > 0) {
		tx_q->cur_tx = BSTGMAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
		WARN_ON(tx_q->tx_skbuff[tx_q->cur_tx]);
		desc = tx_q->dma_tx + tx_q->cur_tx;

		desc->des0 = cpu_to_le32(des + (total_len - tmp_len));
		buff_size = tmp_len >= TSO_MAX_BUFF_SIZE ?
			    TSO_MAX_BUFF_SIZE : tmp_len;

		bstgmac_prepare_tso_tx_desc(priv, desc, 0, buff_size,
				0, 1,
				(last_segment) && (tmp_len <= TSO_MAX_BUFF_SIZE),
				0, 0);

		tmp_len -= TSO_MAX_BUFF_SIZE;
	}
}

/**
 *  bstgmac_tso_xmit - Tx entry point of the driver for oversized frames (TSO)
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description: this is the transmit function that is called on TSO frames
 *  (support available on GMAC4 and newer chips).
 *  Diagram below show the ring programming in case of TSO frames:
 *
 *  First Descriptor
 *   --------
 *   | DES0 |---> buffer1 = L2/L3/L4 header
 *   | DES1 |---> TCP Payload (can continue on next descr...)
 *   | DES2 |---> buffer 1 and 2 len
 *   | DES3 |---> must set TSE, TCP hdr len-> [22:19]. TCP payload len [17:0]
 *   --------
 *	|
 *     ...
 *	|
 *   --------
 *   | DES0 | --| Split TCP Payload on Buffers 1 and 2
 *   | DES1 | --|
 *   | DES2 | --> buffer 1 and 2 len
 *   | DES3 |
 *   --------
 *
 * mss is fixed when enable tso, so w/o programming the TDES3 ctx field.
 */
static netdev_tx_t bstgmac_tso_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dma_desc *desc, *first, *mss_desc = NULL;
	struct bstgmac_priv *priv = netdev_priv(dev);
	int nfrags = skb_shinfo(skb)->nr_frags;
	u32 queue = skb_get_queue_mapping(skb);
	unsigned int first_entry, des;
	struct bstgmac_tx_queue *tx_q;
	int tmp_pay_len = 0;
	u32 pay_len, mss;
	u8 proto_hdr_len;
	bool has_vlan;
	int i;

	tx_q = &priv->tx_queue[queue];

	/* Compute header lengths */
	proto_hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);

	/* Desc availability based on threshold should be enough safe */
	if (unlikely(bstgmac_tx_avail(priv, queue) <
		(((skb->len - proto_hdr_len) / TSO_MAX_BUFF_SIZE + 1)))) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

	pay_len = skb_headlen(skb) - proto_hdr_len; /* no frags */

	mss = skb_shinfo(skb)->gso_size;

	/* set new MSS value if needed */
	if (mss != tx_q->mss) {
		mss_desc = tx_q->dma_tx + tx_q->cur_tx;
		bstgmac_set_mss(priv, mss_desc, mss);
		tx_q->mss = mss;
		tx_q->cur_tx = BSTGMAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
		WARN_ON(tx_q->tx_skbuff[tx_q->cur_tx]);
	}

	if (netif_msg_tx_queued(priv)) {
		pr_info("%s: tcphdrlen %d, hdr_len %d, pay_len %d, mss %d\n",
			__func__, tcp_hdrlen(skb), proto_hdr_len, pay_len, mss);
		pr_info("\tskb->len %d, skb->data_len %d\n", skb->len,
			skb->data_len);
	}

	/* Check if VLAN can be inserted by HW */
        has_vlan = bstgmac_vlan_insert(priv, skb, tx_q);

	first_entry = tx_q->cur_tx;
	WARN_ON(tx_q->tx_skbuff[first_entry]);

	desc = tx_q->dma_tx + first_entry;
	first = desc;
	
	if (has_vlan)
		bstgmac_set_desc_vlan(priv, first, BSTGMAC_VLAN_INSERT);

	/* first descriptor: fill Headers on Buf1 */
	des = dma_map_single(priv->device, skb->data, skb_headlen(skb),
			     DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, des))
		goto dma_map_err;

	tx_q->tx_skbuff_dma[first_entry].buf = des;
	tx_q->tx_skbuff_dma[first_entry].len = skb_headlen(skb);

	first->des0 = cpu_to_le32(des);

	/* Fill start of payload in buff2 of first descriptor */
	if (pay_len)
		first->des1 = cpu_to_le32(des + proto_hdr_len);

	/* If needed take extra descriptors to fill the remaining payload */
	tmp_pay_len = pay_len - TSO_MAX_BUFF_SIZE;

	bstgmac_tso_allocator(priv, des, tmp_pay_len, (nfrags == 0), queue);

	/* Prepare fragments */
	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		des = skb_frag_dma_map(priv->device, frag, 0,
				       skb_frag_size(frag),
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		bstgmac_tso_allocator(priv, des, skb_frag_size(frag),
				     (i == nfrags - 1), queue);

		tx_q->tx_skbuff_dma[tx_q->cur_tx].buf = des;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].len = skb_frag_size(frag);
		tx_q->tx_skbuff_dma[tx_q->cur_tx].map_as_page = true;
	}

	tx_q->tx_skbuff_dma[tx_q->cur_tx].last_segment = true;

	/* Only the last descriptor gets to point to the skb. */
	tx_q->tx_skbuff[tx_q->cur_tx] = skb;

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and bstgmac_tx_clean may clean up to this descriptor.
	 */
	tx_q->cur_tx = BSTGMAC_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);

	if (unlikely(bstgmac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	dev->stats.tx_bytes += skb->len;
	priv->xstats.tx_tso_frames++;
	priv->xstats.tx_tso_nfrags += nfrags;

	/* Manage tx mitigation */
	tx_q->tx_count_frames += nfrags + 1;
	if (likely(priv->tx_coal_frames > tx_q->tx_count_frames) &&
	    !(priv->synopsys_id >= DWMAC_CORE_4_00 &&
	    (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
	    priv->hwts_tx_en)) {
		bstgmac_tx_timer_arm(priv, queue);
	} else {
		tx_q->tx_count_frames = 0;
		bstgmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	skb_tx_timestamp(skb);

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->hwts_tx_en)) {
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		bstgmac_enable_tx_timestamp(priv, first);
	}

	/* Complete the first descriptor before granting the DMA */
	bstgmac_prepare_tso_tx_desc(priv, first, 1,
			proto_hdr_len,
			pay_len,
			1, tx_q->tx_skbuff_dma[first_entry].last_segment,
			tcp_hdrlen(skb) / 4, (skb->len - proto_hdr_len));

	/* If context desc is used to change MSS */
	if (mss_desc) {
		/* Make sure that first descriptor has been completely
		 * written, including its own bit. This is because MSS is
		 * actually before first descriptor, so we need to make
		 * sure that MSS's own bit is the last thing written.
		 */
		dma_wmb();
		bstgmac_set_tx_owner(priv, mss_desc);
	}

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	if (netif_msg_pktdata(priv)) {
		pr_info("%s: curr=%d dirty=%d f=%d, e=%d, f_p=%p, nfrags %d\n",
			__func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			tx_q->cur_tx, first, nfrags);

		bstgmac_display_ring(priv, (void *)tx_q->dma_tx, DMA_TX_SIZE, 0);

		pr_info(">>> frame to be transmitted: ");
		print_pkt(skb->data, skb_headlen(skb));
	}

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	tx_q->tx_tail_addr = tx_q->dma_tx_phy + (tx_q->cur_tx * sizeof(*desc));
	bstgmac_set_tx_tail_ptr(priv, priv->ioaddr, tx_q->tx_tail_addr, queue);

	return NETDEV_TX_OK;

dma_map_err:
	dev_err(priv->device, "Tx dma map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

/**
 *  bstgmac_xmit - Tx entry point of the driver
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description : this is the tx entry point of the driver.
 *  It programs the chain or the ring and supports oversized frames
 *  and SG feature.
 */
static netdev_tx_t bstgmac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i, csum_insertion = 0, is_jumbo = 0;
	u32 queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry;
	unsigned int first_entry;
	struct dma_desc *desc, *first;
	struct bstgmac_tx_queue *tx_q;
	unsigned int enh_desc;
	dma_addr_t des;
	bool has_vlan;

	tx_q = &priv->tx_queue[queue];

	if (priv->tx_path_in_lpi_mode)
		bstgmac_disable_eee_mode(priv);

	/* Manage oversized TCP frames for GMAC4 device */
	if (skb_is_gso(skb) && priv->tso) {
		if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6))
			return bstgmac_tso_xmit(skb, dev);
	}

	if (unlikely(bstgmac_tx_avail(priv, queue) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}
	
	/* Check if VLAN can be inserted by HW */
	has_vlan = bstgmac_vlan_insert(priv, skb, tx_q);

	entry = tx_q->cur_tx;
	first_entry = entry;
	WARN_ON(tx_q->tx_skbuff[first_entry]);

	csum_insertion = (skb->ip_summed == CHECKSUM_PARTIAL);

	if (likely(priv->extend_desc))
		desc = (struct dma_desc *)(tx_q->dma_etx + entry);
	else
		desc = tx_q->dma_tx + entry;

	first = desc;

	if (has_vlan)
		bstgmac_set_desc_vlan(priv, first, BSTGMAC_VLAN_INSERT);

	enh_desc = priv->plat->enh_desc;
	/* To program the descriptors according to the size of the frame */
	if (enh_desc)
		is_jumbo = bstgmac_is_jumbo_frm(priv, skb->len, enh_desc);

	if (unlikely(is_jumbo)) {
		entry = bstgmac_jumbo_frm(priv, tx_q, skb, csum_insertion);
		if (unlikely(entry < 0) && (entry != -EINVAL))
			goto dma_map_err;
	}

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = BSTGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		WARN_ON(tx_q->tx_skbuff[entry]);

		if (likely(priv->extend_desc))
			desc = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			desc = tx_q->dma_tx + entry;

		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err; /* should reuse desc w/o issues */

		tx_q->tx_skbuff_dma[entry].buf = des;

		bstgmac_set_desc_addr(priv, desc, des);

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;

		/* Prepare the descriptor and set the own bit too */
		bstgmac_prepare_tx_desc(priv, desc, 0, len, csum_insertion,
				priv->mode, 1, last_segment, skb->len);
	}

	/* Only the last descriptor gets to point to the skb. */
	tx_q->tx_skbuff[entry] = skb;

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and bstgmac_tx_clean may clean up to this descriptor.
	 */
	entry = BSTGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_q->cur_tx = entry;

	if (netif_msg_pktdata(priv)) {
		void *tx_head;

		netdev_dbg(priv->dev,
			   "%s: curr=%d dirty=%d f=%d, e=%d, first=%p, nfrags=%d",
			   __func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			   entry, first, nfrags);

		if (priv->extend_desc)
			tx_head = (void *)tx_q->dma_etx;
		else
			tx_head = (void *)tx_q->dma_tx;

		netdev_dbg(priv->dev, ">>> frame to be transmitted: ");
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(bstgmac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
  		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	dev->stats.tx_bytes += skb->len;

	/* According to the coalesce parameter the IC bit for the latest
	 * segment is reset and the timer re-started to clean the tx status.
	 * This approach takes care about the fragments: desc is the first
	 * element in case of no SG.
	 */
	tx_q->tx_count_frames += nfrags + 1;
	if (likely(priv->tx_coal_frames > tx_q->tx_count_frames) &&
	    !(priv->synopsys_id >= DWMAC_CORE_4_00 &&
	    (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
	    priv->hwts_tx_en)) {       
        bstgmac_tx_timer_arm(priv, queue);      
	} else {
		tx_q->tx_count_frames = 0;
		bstgmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	skb_tx_timestamp(skb);

	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
	if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;
       
		tx_q->tx_skbuff_dma[first_entry].buf = des;

		bstgmac_set_desc_addr(priv, first, des);

		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = last_segment;

		if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
			     priv->hwts_tx_en)) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			bstgmac_enable_tx_timestamp(priv, first);
		}

		/* Prepare the first descriptor setting the OWN bit too */
		bstgmac_prepare_tx_desc(priv, first, 1, nopaged_len,
				csum_insertion, priv->mode, 1, last_segment,
				skb->len);              
	} else {
		bstgmac_set_tx_owner(priv, first);
	}

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);

	bstgmac_enable_dma_transmission(priv, priv->ioaddr);

	tx_q->tx_tail_addr = tx_q->dma_tx_phy + (tx_q->cur_tx * sizeof(*desc));
	bstgmac_set_tx_tail_ptr(priv, priv->ioaddr, tx_q->tx_tail_addr, queue);

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static void bstgmac_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct vlan_ethhdr *veth;
	__be16 vlan_proto;
	u16 vlanid;

	veth = (struct vlan_ethhdr *)skb->data;
	vlan_proto = veth->h_vlan_proto;

	if ((vlan_proto == htons(ETH_P_8021Q) &&
	     dev->features & NETIF_F_HW_VLAN_CTAG_RX) ||
	    (vlan_proto == htons(ETH_P_8021AD) &&
	     dev->features & NETIF_F_HW_VLAN_STAG_RX)) {
		/* pop the vlan tag */
		vlanid = ntohs(veth->h_vlan_TCI);
		memmove(skb->data + VLAN_HLEN, veth, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, vlan_proto, vlanid);
	}
}


static inline int bstgmac_rx_threshold_count(struct bstgmac_rx_queue *rx_q)
{
	if (rx_q->rx_zeroc_thresh < BSTGMAC_RX_THRESH)
		return 0;

	return 1;
}

/**
 * bstgmac_rx_refill - refill used skb preallocated buffers
 * @priv: driver private structure
 * @queue: RX queue index
 * Description : this is to reallocate the skb for the reception process
 * that is based on zero-copy.
 */
static inline void bstgmac_rx_refill(struct bstgmac_priv *priv, u32 queue)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];
	int dirty = bstgmac_rx_dirty(priv, queue);
	unsigned int entry = rx_q->dirty_rx;

	int bfsize = priv->dma_buf_sz;
    int bus_id = priv->plat->bus_id, cpuid;
    struct dma_desc *p;
    int index;
    struct sk_buff_head *list;
    struct sk_buff *skb;
    dma_addr_t buf;
    dma_addr_t *addr;

    index = bus_id * BSTGMAC_RXCHAN_NUM + queue;
    list = &gmac_delivery_skblist[index];
	while (dirty-- > 0) {
		p = rx_q->dma_rx + entry;

		if (likely(!rx_q->rx_skbuff[entry])) {
            skb = skb_dequeue(list);       
            if (unlikely(!skb)) {         
                skb = netdev_alloc_skb_ip_align(priv->dev, bfsize);
                if (unlikely(!skb)) {
                  if (unlikely(net_ratelimit()))
                        dev_err(priv->device, "fail to alloc skb entry %d\n", entry);               
                    break;
                }
                buf = dma_map_single(priv->device, skb->data, bfsize, DMA_FROM_DEVICE);
                if (dma_mapping_error(priv->device, buf)) {
                    netdev_err(priv->dev, "Rx DMA map failed\n");
                    dev_kfree_skb(skb);
                    break;
                }     
            } else {
                addr = (dma_addr_t *)skb->cb;
                buf = *addr;
                *addr = 0;
                dma_wmb();
            }

            rx_q->rx_skbuff[entry] = skb;
            rx_q->rx_skbuff_dma[entry] = buf;
            
			bstgmac_set_desc_addr(priv, p, buf);
			bstgmac_refill_desc3(priv, rx_q, p);
		}
		dma_wmb();

		bstgmac_set_rx_owner(priv, p, priv->use_riwt);

		dma_wmb();

		entry = BSTGMAC_GET_ENTRY(entry, DMA_RX_SIZE);
	}
	rx_q->dirty_rx = entry;
	bstgmac_set_rx_tail_ptr(priv, priv->ioaddr, rx_q->rx_tail_addr, queue);
    if (skb_queue_len(list) < BSTGMAC_RXMEM_THRE) {
		cpuid = (bus_id == BSTGMAC0_BUS_ID) ? BSTGMAC1_BUS_ID : BSTGMAC0_BUS_ID;
        queue_work_on(cpuid, priv->rxmem, &priv->mem_mgmt_work); //eth0:cpu1 eth1:cpu0
    }
}

/**
 * bstgmac_rx - manage the receive process
 * @priv: driver private structure
 * @limit: napi bugget
 * @queue: RX queue index.
 * Description :  this the function called by the napi poll method.
 * It gets all the frames inside the ring.
 */
static int bstgmac_rx_func(struct bstgmac_priv *priv, int limit, u32 queue)
{
	struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct bstgmac_channel *ch = &priv->rx_channel[queue];
	unsigned int next_entry = rx_q->cur_rx;
	int coe = priv->hw->rx_csum;
	unsigned int count = 0;

	while (count < limit) {
		int entry, status;
		struct dma_desc *p;
		struct dma_desc *np;

		entry = next_entry;

		p = rx_q->dma_rx + entry;
		/* read the status of the incoming frame */
		status = bstgmac_rx_status(priv, &priv->dev->stats,
				&priv->xstats, p);
		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & dma_own))
			break;
		if (perf_debug) {	
			udelay(perf_debug);
		}
		rx_q->cur_rx = BSTGMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
		next_entry = rx_q->cur_rx;

		np = rx_q->dma_rx + next_entry;
		prefetch(np);

		if (unlikely(status == discard_frame)) {
			priv->dev->stats.rx_errors++;
			if (!priv->extend_desc) {
				/* DESC2 & DESC3 will be overwritten by device
				 * with timestamp value, hence reinitialize
				 * them in bstgmac_rx_refill() function so that
				 * device can reuse it.
				 */
				dev_kfree_skb_any(rx_q->rx_skbuff[entry]);
				rx_q->rx_skbuff[entry] = NULL;
				dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);
			}
		} else {
			struct sk_buff *skb;
			int frame_len;

			frame_len = bstgmac_get_rx_frame_len(priv, p, coe);

			/*  If frame length is greater than skb buffer size
			 *  (preallocated during init) then the packet is
			 *  ignored
			 */
			if (frame_len > priv->dma_buf_sz) {
				if (net_ratelimit())
					netdev_err(priv->dev, "len %d larger than size (%d)\n", frame_len, priv->dma_buf_sz);
                dev_kfree_skb_any(rx_q->rx_skbuff[entry]);
				rx_q->rx_skbuff[entry] = NULL;
				dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);
				priv->dev->stats.rx_length_errors++;
				continue;
			}

			/* ACS is set; GMAC core strips PAD/FCS for IEEE 802.3
			 * Type frames (LLC/LLC-SNAP)
			 *
			 * llc_snap is never checked in GMAC >= 4, so this ACS
			 * feature is always disabled and packets need to be
			 * stripped manually.
			 */
			if (unlikely(status != llc_snap))
				frame_len -= ETH_FCS_LEN;

            skb = rx_q->rx_skbuff[entry];
            if (unlikely(!skb)) {
                if (net_ratelimit())
                    netdev_err(priv->dev,"%s: Inconsistent Rx chain\n", priv->dev->name);
                priv->dev->stats.rx_dropped++;
                break;
            }
            prefetch(skb->data - NET_IP_ALIGN);
            rx_q->rx_skbuff[entry] = NULL;

            skb_put(skb, frame_len);
            dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);

			bstgmac_get_rx_hwtstamp(priv, p, np, skb);

			bstgmac_rx_vlan(priv->dev, skb);
			
			skb->protocol = eth_type_trans(skb, priv->dev);
            skb->ip_summed = CHECKSUM_UNNECESSARY;
#if BSTGMAC_RX_NAPI
            napi_gro_receive(&ch->rnapi, skb);
#else
			netif_receive_skb(skb);
#endif
			priv->dev->stats.rx_packets++;
			priv->dev->stats.rx_bytes += frame_len;
		}
		count++;
	}

    bstgmac_rx_refill(priv, queue);

	priv->xstats.rx_pkt_n += count;

	return count;
}

/**
 *  bstgmac_poll - bstgmac poll method (NAPI)
 *  @napi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *  To look at the incoming frames and clear the tx resources.
 */
static int bstgmac_napi_poll(struct napi_struct *napi, int budget)
{
	struct bstgmac_channel *ch =
		container_of(napi, struct bstgmac_channel, napi);
	struct bstgmac_priv *priv = ch->priv_data;
	int work_done, rx_done = 0, tx_done = 0;
	u32 chan = ch->index;

	priv->xstats.napi_poll++;

	if (ch->has_tx)
		tx_done = bstgmac_tx_clean(priv, budget, chan);
	if (ch->has_rx)
		rx_done = bstgmac_rx_func(priv, budget, chan);

	work_done = max(rx_done, tx_done);
	work_done = min(work_done, budget);

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		int stat;

		bstgmac_enable_dma_irq(priv, priv->ioaddr, chan);
		stat = bstgmac_dma_interrupt_status(priv, priv->ioaddr,
						   &priv->xstats, chan ,0);
		if (stat && napi_reschedule(napi))
			bstgmac_disable_dma_irq(priv, priv->ioaddr, chan);
	}

	return work_done;
}

/**
 *  bstgmac_rpoll - bstgmac rx poll method (NAPI)
 *  @napi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *  To look at the incoming frames.
 */
static int bstgmac_rx_napi_poll(struct napi_struct *rnapi, int budget)
{
	struct bstgmac_channel *ch = container_of(rnapi, struct bstgmac_channel, rnapi);
	struct bstgmac_priv *priv = ch->priv_data;
    int bus_id = priv->plat->bus_id;
	int work_done, rx_done = 0;
	u32 chan = ch->index;
    uint flags;

	priv->xstats.rnapi_poll++;

	if (ch->has_rx)
		rx_done = bstgmac_rx_func(priv, budget, chan);

	work_done = min(rx_done, budget);

	if (work_done < budget && napi_complete_done(rnapi, work_done)) {
		int stat;
        if (bus_id == BSTGMAC0_BUS_ID) {
            spin_lock_irqsave(&gmac0_irqbits_lock, flags);
        } else {
            spin_lock_irqsave(&gmac1_irqbits_lock, flags);
        }
        bstgmac_enable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_RIE);
        if (bus_id == BSTGMAC0_BUS_ID) {
            spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
        } else {
            spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
        }
		
		stat = bstgmac_dma_ri_interrupt_status(priv, priv->ioaddr,
						   &priv->xstats, chan);
		if (stat && napi_reschedule(rnapi)) {
            if (bus_id == BSTGMAC0_BUS_ID) {
                spin_lock_irqsave(&gmac0_irqbits_lock, flags);
            } else {
                spin_lock_irqsave(&gmac1_irqbits_lock, flags);
            }
			bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_RIE);
            if (bus_id == BSTGMAC0_BUS_ID) {
                spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
            } else {
                spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
            }
        }
	}

	return work_done;    
}
static void bstgmac_refill_rxmem(struct bstgmac_priv *priv, int chan)
{
    int cnt, busid, index;
    struct sk_buff *skb; 
    dma_addr_t buf;
    int size = priv->dma_buf_sz;
    dma_addr_t *addr;
    struct sk_buff_head *list;

    busid = priv->plat->bus_id;
    index = busid * BSTGMAC_RXCHAN_NUM + chan;
    list = &gmac_delivery_skblist[index];
    cnt = 0;

    while (cnt < (BSTGMAC_RXMEM_THRE - 1)) {    
        skb = netdev_alloc_skb_ip_align(priv->dev, size);
        if (unlikely(!skb)) {                
            break;
        }
        buf = dma_map_single(priv->device, skb->data, size, DMA_FROM_DEVICE);
        if (dma_mapping_error(priv->device, buf)) {
            netdev_err(priv->dev, "Gmac map list failed\n");
            dev_kfree_skb(skb);
            break;
        }
        addr = (dma_addr_t *)skb->cb;
        *addr = buf;                     
        dma_wmb();
        skb_queue_tail(list, skb);
        cnt++;
    }
}

static void bstgmac_mem_mgmt_work(struct work_struct *work)
{
    struct bstgmac_priv *priv = container_of(work, struct bstgmac_priv, mem_mgmt_work);
    int chan, busid, index;
    struct sk_buff_head *list;
     
    priv->xstats.napi_poll++;
    busid = priv->plat->bus_id;
	set_bit(BSTGMAC_RXMEM_WORK_RUN, &priv->state);
    for (chan = 0; chan < BSTGMAC_RXCHAN_NUM; chan++) {
        index = busid * BSTGMAC_RXCHAN_NUM + chan;
        list = &gmac_delivery_skblist[index];
        if (skb_queue_len(list) < BSTGMAC_RXMEM_THRE) {        
            bstgmac_refill_rxmem(priv, chan);
        }
    }
	clear_bit(BSTGMAC_RXMEM_WORK_RUN, &priv->state);
}

/**
 *  bstgmac_tpoll - bstgmac tx poll method (NAPI)
 *  @tnapi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *  To  clear the tx resources.
 */
static int bstgmac_tx_napi_poll(struct napi_struct *tnapi, int budget)
{
	struct bstgmac_channel *ch = container_of(tnapi, struct bstgmac_channel, tnapi);
	struct bstgmac_priv *priv = ch->priv_data;
	int work_done, tx_done = 0;
	u32 chan = ch->index;
    int bus_id = priv->plat->bus_id;
    unsigned long flags;
    
	priv->xstats.tnapi_poll++;

	if (ch->has_tx)
		tx_done = bstgmac_tx_clean(priv, budget, chan);

	work_done = min(tx_done, budget);
	if (work_done < budget && napi_complete_done(tnapi, work_done)) {
		int stat;
		if (bus_id == BSTGMAC0_BUS_ID) {
            spin_lock_irqsave(&gmac0_irqbits_lock, flags);
        } else {
            spin_lock_irqsave(&gmac1_irqbits_lock, flags);
        }
		bstgmac_enable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_TIE);
		if (bus_id == BSTGMAC0_BUS_ID) {
			spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
		} else {
			spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
		}

		stat = bstgmac_dma_ti_interrupt_status(priv, priv->ioaddr,
						   &priv->xstats, chan);
		if (stat && napi_reschedule(tnapi)) {
			if (bus_id == BSTGMAC0_BUS_ID) {
				spin_lock_irqsave(&gmac0_irqbits_lock, flags);
			} else {
				spin_lock_irqsave(&gmac1_irqbits_lock, flags);
			}
			bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_TIE);
			if (bus_id == BSTGMAC0_BUS_ID) {
				spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
			} else {
				spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
			}
		}
	}

	return work_done;    
}

static void bstgmac_tx_work(struct work_struct *tx_work)
{
	struct bstgmac_channel *ch = container_of(tx_work, struct bstgmac_channel, tx_work);
	struct bstgmac_priv *priv = ch->priv_data;
	int work_done, tx_done = 0;
	u32 chan = ch->index;
    int stat;
    int bus_id = priv->plat->bus_id;
    unsigned long flags;
    int budget = DMA_RX_SIZE/2;

	priv->xstats.txwork_poll++;

	if (ch->has_tx)
		tx_done = bstgmac_tx_clean(priv, budget, chan);

	work_done = min(tx_done, budget);
	if (work_done <= budget) {
		stat = bstgmac_dma_ti_interrupt_status(priv, priv->ioaddr, &priv->xstats, chan);
		if (stat) {    
            queue_work(priv->tx_wq, &ch->tx_work);
        } else {
            if (bus_id == BSTGMAC0_BUS_ID) {
                spin_lock_irqsave(&gmac0_irqbits_lock, flags);
            } else {
                spin_lock_irqsave(&gmac1_irqbits_lock, flags);
            }
            bstgmac_enable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_TIE);
            if (bus_id == BSTGMAC0_BUS_ID) {
                spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
            } else {
                spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
            }
        }
	}
}

/**
 *  bstgmac_tx_timeout
 *  @dev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable time. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void bstgmac_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	bstgmac_global_err(priv);
}

/**
 *  bstgmac_set_rx_mode - entry point for multicast addressing
 *  @dev : pointer to the device structure
 *  Description:
 *  This function is a driver entry point which gets called by the kernel
 *  whenever multicast addresses must be enabled/disabled.
 *  Return value:
 *  void.
 */
static void bstgmac_set_rx_mode(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	bstgmac_set_filter(priv, priv->hw, dev);
}

/**
 *  bstgmac_change_mtu - entry point to change MTU size for the device.
 *  @dev : device pointer.
 *  @new_mtu : the new MTU size for the device.
 *  Description: the Maximum Transfer Unit (MTU) is used by the network layer
 *  to drive packet transmission. Ethernet has an MTU of 1500 octets
 *  (ETH_DATA_LEN). This value can be changed with ifconfig.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstgmac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (netif_running(dev)) {
		netdev_err(priv->dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}
    if (new_mtu > 1500) {
        netdev_err(priv->dev, "cannot more than 1500\n");
        return -EINVAL;
    }
    
	dev->mtu = new_mtu;

	netdev_update_features(dev);

	return 0;
}

static netdev_features_t bstgmac_fix_features(struct net_device *dev,
					     netdev_features_t features)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (priv->plat->rx_coe == STMMAC_RX_COE_NONE)
		features &= ~NETIF_F_RXCSUM;

	if (!priv->plat->tx_coe)
		features &= ~NETIF_F_CSUM_MASK;

	/* Some GMAC devices have a bugged Jumbo frame support that
	 * needs to have the Tx COE disabled for oversized frames
	 * (due to limited buffer sizes). In this case we disable
	 * the TX csum insertion in the TDES and not use SF.
	 */
	if (priv->plat->bugged_jumbo && (dev->mtu > ETH_DATA_LEN))
		features &= ~NETIF_F_CSUM_MASK;

	/* Disable tso if asked by ethtool */
	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		if (features & NETIF_F_TSO)
			priv->tso = true;
		else
			priv->tso = false;
	}

	return features;
}

static int bstgmac_set_features(struct net_device *netdev,
			       netdev_features_t features)
{
	struct bstgmac_priv *priv = netdev_priv(netdev);

	/* Keep the COE Type in case of csum is supporting */
	if (features & NETIF_F_RXCSUM)
		priv->hw->rx_csum = priv->plat->rx_coe;
	else
		priv->hw->rx_csum = 0;
	/* No check needed because rx_coe has been set before and it will be
	 * fixed in case of issue.
	 */
	bstgmac_rx_ipc(priv, priv->hw);

	return 0;
}

static int bstgmac_match_ch_irq(struct bstgmac_priv *priv,int irq,int istx)
{
	int i;
	int maxq;
	int* perch_irqs;
	if(istx){
		perch_irqs = priv->perch_tx_irq;
		maxq = priv->plat->tx_queues_to_use;
	}else{
		perch_irqs = priv->perch_rx_irq;
		maxq = priv->plat->rx_queues_to_use;
	}
	for(i = 0;i< maxq;i++){
		if(irq == perch_irqs[i]){
			return i;
		}
	}
	return -1;
}

/**
 *  bstgmac_interrupt - main ISR
 *  @irq: interrupt number.
 *  @dev_id: to pass the net device pointer.
 *  Description: this is the main driver interrupt service routine.
 *  It can call:
 *  o DMA service routine (to manage incoming frame reception and transmission
 *    status)
 *  o Core interrupts to manage: remote wake-up, management counter, LPI
 *    interrupts.
 */
static irqreturn_t bstgmac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queues_count;
	u32 queue;
	bool xmac;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;
	queues_count = (rx_cnt > tx_cnt) ? rx_cnt : tx_cnt;

	if (priv->irq_wake)
		pm_wakeup_event(priv->device, 0);

	if (unlikely(!dev)) {
		netdev_err(priv->dev, "%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}

	/* Check if adapter is up */
	if (test_bit(BSTGMAC_DOWN, &priv->state))
		return IRQ_HANDLED;
	/* Check if a fatal error happened */
	if (bstgmac_safety_feat_interrupt(priv))
		return IRQ_HANDLED;

	/* To handle GMAC own interrupts */
	if ((priv->plat->has_gmac) || xmac) {
		int status = bstgmac_host_irq_status(priv, priv->hw, &priv->xstats);
		int mtl_status;

		if (unlikely(status)) {
			/* For LPI we need to save the tx status */
			if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
				priv->tx_path_in_lpi_mode = true;
			if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
				priv->tx_path_in_lpi_mode = false;
		}

		for (queue = 0; queue < queues_count; queue++) {
			struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

			mtl_status = bstgmac_host_mtl_irq_status(priv, priv->hw,
								queue);
			if (mtl_status != -EINVAL)
				status |= mtl_status;

			if (status & CORE_IRQ_MTL_RX_OVERFLOW)
				bstgmac_set_rx_tail_ptr(priv, priv->ioaddr,
						       rx_q->rx_tail_addr,
						       queue);
		}
		
		/* PCS link status */
		if (!priv->plat->phylink_node) {
			if (priv->hw->pcs) {
				if (priv->xstats.pcs_link)
					netif_carrier_on(dev);
				else
					netif_carrier_off(dev);
			}
		}
	}

	/* To handle DMA interrupts */
	bstgmac_dma_interrupt(priv);

	return IRQ_HANDLED;
}

/*
interrupt form gmac intr signal:sbd_intr_o
*/
static irqreturn_t bstgmac_sbd_interrupt(int irq, void *dev_id)
{

	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queues_count;
	u32 queue;
	bool xmac;

	/* if Timestamp Interrupt status active, do this*/
	if (readl(priv->ioaddr+0xb0) & (1<<12))
		bstptp_extts_interrupt(irq, priv);

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;
	queues_count = (rx_cnt > tx_cnt) ? rx_cnt : tx_cnt;

	if (priv->irq_wake){
		pm_wakeup_event(priv->device, 0);
	}

	if (unlikely(!dev)) {
		netdev_err(priv->dev, "%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}

	/* Check if adapter is up */
	if (test_bit(BSTGMAC_DOWN, &priv->state)){
		return IRQ_HANDLED;
	}
	
	/* To handle GMAC own interrupts */
	if ((priv->plat->has_gmac) || xmac) {
		int status = bstgmac_host_irq_status(priv, priv->hw, &priv->xstats);
		int mtl_status;
		//pr_emerg("[%s]%d.",__func__,__LINE__);

		if (unlikely(status)) {
			/* For LPI we need to save the tx status */
			//pr_emerg("[%s]%d.",__func__,__LINE__);
			if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
				priv->tx_path_in_lpi_mode = true;
			if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
				priv->tx_path_in_lpi_mode = false;
			
			//pr_emerg("[%s]%d.tx_path_in_lpi_mode:%d",__func__,__LINE__,priv->tx_path_in_lpi_mode);
		}

		for (queue = 0; queue < queues_count; queue++) {
			struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

			mtl_status = bstgmac_host_mtl_irq_status(priv, priv->hw,
								queue);
			if (mtl_status != -EINVAL)
				status |= mtl_status;

			if (status & CORE_IRQ_MTL_RX_OVERFLOW){
				bstgmac_set_rx_tail_ptr(priv, priv->ioaddr,
						       rx_q->rx_tail_addr,
						       queue);
			  // pr_emerg("[%s]%d. status & CORE_IRQ_MTL_RX_OVERFLOW",__func__,__LINE__);
			}
			//if(mtl_status){
			//	pr_emerg("[%s]%d. mtl_status:%x",__func__,__LINE__,mtl_status);
			//}
		}

		/* PCS link status 
		if (priv->hw->pcs) {
			if (priv->xstats.pcs_link)
				netif_carrier_on(dev);
			else
				netif_carrier_off(dev);
		}*/
	}
	if(priv->plat->dma_cfg->dma_int_mode == DMA_INT_M_0){
	/* To handle DMA interrupts */
		bstgmac_dma_interrupt(priv);
	}
	return IRQ_HANDLED;
}


/*
interrupt form gmac intr signal:sbd_sfty_ce_intr_o&sbd_sfty_ue_intr_o
*/
static irqreturn_t bstgmac_safety_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);

	//pr_emerg("[%s]%d irq:%d.",__func__,__LINE__,irq);
	//if (priv->irq_wake)
	//	pm_wakeup_event(priv->device, 0);

	if (unlikely(!dev)) {
		netdev_err(priv->dev, "%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	
	/* Check if a fatal error happened */
	bstgmac_safety_feat_interrupt(priv);
	return IRQ_HANDLED;
}


/*
interrupt form gmac intr signal:sbd_perch_rx_intr_o[3:0]
*/
static irqreturn_t bstgmac_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 chan;
	int status;
	struct bstgmac_channel *ch ;
	bool needs_work;
    unsigned long flags;
    int bus_id = priv->plat->bus_id;

	chan = bstgmac_match_ch_irq(priv,irq,0);
	if(chan < 0 || chan >= MTL_MAX_RX_QUEUES){
		return IRQ_NONE;
	}
	
	status = bstgmac_dma_ri_interrupt_status(priv, priv->ioaddr,
						 &priv->xstats, chan);
	
	ch = &priv->rx_channel[chan];
	needs_work = false;

	if ((status & handle_rx) && ch->has_rx) {
		needs_work = true;
	}
    
	if (needs_work && napi_schedule_prep(&ch->rnapi)) {
		if (bus_id == BSTGMAC0_BUS_ID) {
            spin_lock_irqsave(&gmac0_irqbits_lock, flags);
        } else {
            spin_lock_irqsave(&gmac1_irqbits_lock, flags);
        }
        bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_RIE);
        if (bus_id == BSTGMAC0_BUS_ID) {
            spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
        } else {
            spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
        }
		__napi_schedule(&ch->rnapi);
    }
        
    return IRQ_HANDLED;
}

static irqreturn_t bstgmac_rx_interrupt_threading_null(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/*
interrupt form gmac intr signal:sbd_perch_tx_intr_o[3:0]
*/
static irqreturn_t bstgmac_tx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);

	u32 chan;
	int status;
	struct bstgmac_channel *ch;
	bool needs_work;
    int bus_id = priv->plat->bus_id;
    unsigned long flags;

	chan = bstgmac_match_ch_irq(priv,irq,1);
	if(chan < 0 || chan >= MTL_MAX_TX_QUEUES){
		return IRQ_NONE;
	}

	status = bstgmac_dma_ti_interrupt_status(priv, priv->ioaddr,
						 &priv->xstats, chan);
	
	ch = &priv->tx_channel[chan];
	needs_work = false;

	if ((status & handle_tx) && ch->has_tx) {
		needs_work = true;
	} 
#if BSTGMAC_TX_NAPI
	if (needs_work && napi_schedule_prep(&ch->tnapi)) {
		if (bus_id == BSTGMAC0_BUS_ID) {
            spin_lock_irqsave(&gmac0_irqbits_lock, flags);
        } else {
            spin_lock_irqsave(&gmac1_irqbits_lock, flags);
        }
        bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_TIE);
         if (bus_id == BSTGMAC0_BUS_ID) {
            spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
        } else {
            spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
        }
        __napi_schedule(&ch->tnapi);
	}
#else
    if (needs_work) {
        if (bus_id == BSTGMAC0_BUS_ID) {
            spin_lock_irqsave(&gmac0_irqbits_lock, flags);
        } else {
            spin_lock_irqsave(&gmac1_irqbits_lock, flags);
        }
        bstgmac_disable_dma_irq_bits(priv, priv->ioaddr, chan, DMA_CHAN_INTR_ENA_TIE);
         if (bus_id == BSTGMAC0_BUS_ID) {
            spin_unlock_irqrestore(&gmac0_irqbits_lock, flags);
        } else {
            spin_unlock_irqrestore(&gmac1_irqbits_lock, flags);
        }
        queue_work(priv->tx_wq, &ch->tx_work);
    }
#endif
	return IRQ_HANDLED;
}


/*
interrupt form gmac intr signal:lpi_intr_o
*/
static irqreturn_t bstgmac_lpi_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstgmac_priv *priv = netdev_priv(dev);
	//u32 rx_cnt = priv->plat->rx_queues_to_use;
	//u32 tx_cnt = priv->plat->tx_queues_to_use;
	int status ;
	//pr_emerg("[%s]%d irq:%d.",__func__,__LINE__,irq);
	if (unlikely(!dev)) {
		netdev_err(priv->dev, "%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	/* Check if adapter is up */
	if (test_bit(BSTGMAC_DOWN, &priv->state))
		return IRQ_HANDLED;
	
	status = bstgmac_host_lpi_irq_status(priv, priv->hw, &priv->xstats);
	
	if (unlikely(status)) {
		/* For LPI we need to save the tx status */
		if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
			priv->tx_path_in_lpi_mode = true;
		if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
			priv->tx_path_in_lpi_mode = false;
		
		if (status & CORE_IRQ_RX_PATH_IN_LPI_MODE)
			priv->rx_path_in_lpi_mode = true;
		if (status & CORE_IRQ_RX_PATH_EXIT_LPI_MODE)
			priv->rx_path_in_lpi_mode = false;
	}
	
	return IRQ_HANDLED;
}


#ifdef CONFIG_NET_POLL_CONTROLLER
/* Polling receive - used by NETCONSOLE and other diagnostic tools
 * to allow network I/O with interrupts disabled.
 */
static void bstgmac_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	bstgmac_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif

/**
 *  bstgmac_ioctl - Entry point for the Ioctl
 *  @dev: Device pointer.
 *  @rq: An IOCTL specefic structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  @cmd: IOCTL command
 *  Description:
 *  Currently it supports the phy_mii_ioctl(...) and HW time stamping.
 */
static int bstgmac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	int ret = -EOPNOTSUPP;

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		if (!dev->phydev)
			return -EINVAL;
		ret = phy_mii_ioctl(dev->phydev, rq, cmd);
		break;
	case SIOCSHWTSTAMP:
		ret = bstgmac_hwtstamp_ioctl(dev, rq);
		break;
	default:
		break;
	}

	return ret;
}

static int bstgmac_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				    void *cb_priv)
{
	struct bstgmac_priv *priv = cb_priv;
	int ret = -EOPNOTSUPP;

	if (!tc_cls_can_offload_and_chain0(priv->dev, type_data))
		return ret;

	bstgmac_disable_all_queues(priv);

	switch (type) {
		case TC_SETUP_CLSU32:
			ret = bstgmac_tc_setup_cls_u32(priv, priv, type_data);
            break;
        case TC_SETUP_CLSFLOWER:
            ret = bstgmac_tc_setup_cls(priv, priv, type_data);
			break;
		default:
			break;
	}

	bstgmac_enable_all_queues(priv);

	return ret;
}

static LIST_HEAD(bstgmac_block_cb_list);

static int bstgmac_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data)
{
	struct bstgmac_priv *priv = netdev_priv(ndev);

	switch (type) {
	case TC_SETUP_BLOCK:
        return flow_block_cb_setup_simple(type_data,
						  &bstgmac_block_cb_list,
						  bstgmac_setup_tc_block_cb,
						  priv, priv, true);
	case TC_SETUP_QDISC_CBS:
		return bstgmac_tc_setup_cbs(priv, priv, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return bstgmac_tc_setup_taprio(priv, priv, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static u16 bstgmac_select_queue(struct net_device *dev, struct sk_buff *skb,
			       struct net_device *sb_dev)
{
	if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6 | SKB_GSO_UDP_L4)) {
		/*
		 * There is no way to determine the number of TSO
		 * capable Queues. Let's use always the Queue 0
		 * because if TSO is supported then at least this
		 * one will be capable.
		 */
		return 0;
	}

	return netdev_pick_tx(dev, skb, NULL) % dev->real_num_tx_queues;
}

static int bstgmac_set_mac_address(struct net_device *ndev, void *addr)
{
	struct bstgmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;

	bstgmac_set_umac_addr(priv, priv->hw, ndev->dev_addr, 0);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *bstgmac_fs_dir;

static void sysfs_display_ring_phy(void *head, void *head_phy, int size, int extend_desc,
			       struct seq_file *seq)
{
	int i;
	struct dma_extended_desc *ep = (struct dma_extended_desc *)head;
	struct dma_desc *p = (struct dma_desc *)head;
	struct dma_desc *p1 = (struct dma_desc *)head_phy;
	for (i = 0; i < size; i++) {
		if (extend_desc) {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(ep),
				   le32_to_cpu(ep->basic.des0),
				   le32_to_cpu(ep->basic.des1),
				   le32_to_cpu(ep->basic.des2),
				   le32_to_cpu(ep->basic.des3));
			ep++;
		} else {
			seq_printf(seq, "%d [0x%llx]: 0x%x 0x%x 0x%x 0x%x\n",
					i, (dma_addr_t)p1,
					le32_to_cpu(p->des0), le32_to_cpu(p->des1),
					le32_to_cpu(p->des2), le32_to_cpu(p->des3));
			p++;
			p1++;
		}
		seq_printf(seq, "\n");
	}
}
static void sysfs_display_ring(void *head, int size, int extend_desc,
			       struct seq_file *seq)
{
	int i;
	struct dma_extended_desc *ep = (struct dma_extended_desc *)head;
	struct dma_desc *p = (struct dma_desc *)head;

	for (i = 0; i < size; i++) {
		if (extend_desc) {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(ep),
				   le32_to_cpu(ep->basic.des0),
				   le32_to_cpu(ep->basic.des1),
				   le32_to_cpu(ep->basic.des2),
				   le32_to_cpu(ep->basic.des3));
			ep++;
		} else {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(p),
				   le32_to_cpu(p->des0), le32_to_cpu(p->des1),
				   le32_to_cpu(p->des2), le32_to_cpu(p->des3));
			p++;
		}
		seq_printf(seq, "\n");
	}
}

static int bstgmac_sysfs_ring_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct bstgmac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;

	if ((dev->flags & IFF_UP) == 0)
		return 0;

	for (queue = 0; queue < rx_count; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		seq_printf(seq, "RX Queue %d:\n", queue);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)rx_q->dma_erx,
					   DMA_RX_SIZE, 1, seq);
		} else {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring_phy((void *)rx_q->dma_rx, (void *)rx_q->dma_rx_phy, DMA_RX_SIZE, 0, seq);
		}
	}

	for (queue = 0; queue < tx_count; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		seq_printf(seq, "TX Queue %d:\n", queue);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)tx_q->dma_etx,
					   DMA_TX_SIZE, 1, seq);
		} else {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring_phy((void *)tx_q->dma_tx,(void *)tx_q->dma_tx_phy, DMA_TX_SIZE, 0, seq);
		}
	}

	return 0;
}

static int bstgmac_sysfs_ring_open(struct inode *inode, struct file *file)
{
	return single_open(file, bstgmac_sysfs_ring_read, inode->i_private);
}

/* Debugfs files, should appear in /sys/kernel/debug/bstgmaceth/eth0 */

static const struct file_operations bstgmac_rings_status_fops = {
	.owner = THIS_MODULE,
	.open = bstgmac_sysfs_ring_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bstgmac_sysfs_dma_cap_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct bstgmac_priv *priv = netdev_priv(dev);

	if (!priv->hw_cap_support) {
		seq_printf(seq, "DMA HW features not supported\n");
		return 0;
	}

	seq_printf(seq, "==============================\n");
	seq_printf(seq, "\tDMA HW features\n");
	seq_printf(seq, "==============================\n");

	seq_printf(seq, "\t10/100 Mbps: %s\n",
		   (priv->dma_cap.mbps_10_100) ? "Y" : "N");
	seq_printf(seq, "\t1000 Mbps: %s\n",
		   (priv->dma_cap.mbps_1000) ? "Y" : "N");
	seq_printf(seq, "\tHalf duplex: %s\n",
		   (priv->dma_cap.half_duplex) ? "Y" : "N");
	seq_printf(seq, "\tHash Filter: %s\n",
		   (priv->dma_cap.hash_filter) ? "Y" : "N");
	seq_printf(seq, "\tMultiple MAC address registers: %s\n",
		   (priv->dma_cap.multi_addr) ? "Y" : "N");
	seq_printf(seq, "\tPCS (TBI/SGMII/RTBI PHY interfaces): %s\n",
		   (priv->dma_cap.pcs) ? "Y" : "N");
	seq_printf(seq, "\tSMA (MDIO) Interface: %s\n",
		   (priv->dma_cap.sma_mdio) ? "Y" : "N");
	seq_printf(seq, "\tPMT Remote wake up: %s\n",
		   (priv->dma_cap.pmt_remote_wake_up) ? "Y" : "N");
	seq_printf(seq, "\tPMT Magic Frame: %s\n",
		   (priv->dma_cap.pmt_magic_frame) ? "Y" : "N");
	seq_printf(seq, "\tRMON module: %s\n",
		   (priv->dma_cap.rmon) ? "Y" : "N");
	seq_printf(seq, "\tIEEE 1588-2002 Time Stamp: %s\n",
		   (priv->dma_cap.time_stamp) ? "Y" : "N");
	seq_printf(seq, "\tIEEE 1588-2008 Advanced Time Stamp: %s\n",
		   (priv->dma_cap.atime_stamp) ? "Y" : "N");
	seq_printf(seq, "\t802.3az - Energy-Efficient Ethernet (EEE): %s\n",
		   (priv->dma_cap.eee) ? "Y" : "N");
	seq_printf(seq, "\tAV features: %s\n", (priv->dma_cap.av) ? "Y" : "N");
	seq_printf(seq, "\tChecksum Offload in TX: %s\n",
		   (priv->dma_cap.tx_coe) ? "Y" : "N");
	if (priv->synopsys_id >= DWMAC_CORE_4_00) {
		seq_printf(seq, "\tIP Checksum Offload in RX: %s\n",
			   (priv->dma_cap.rx_coe) ? "Y" : "N");
	} else {
		seq_printf(seq, "\tIP Checksum Offload (type1) in RX: %s\n",
			   (priv->dma_cap.rx_coe_type1) ? "Y" : "N");
		seq_printf(seq, "\tIP Checksum Offload (type2) in RX: %s\n",
			   (priv->dma_cap.rx_coe_type2) ? "Y" : "N");
	}
	seq_printf(seq, "\tRXFIFO > 2048bytes: %s\n",
		   (priv->dma_cap.rxfifo_over_2048) ? "Y" : "N");
	seq_printf(seq, "\tNumber of Additional RX channel: %d\n",
		   priv->dma_cap.number_rx_channel);
	seq_printf(seq, "\tNumber of Additional TX channel: %d\n",
		   priv->dma_cap.number_tx_channel);
	seq_printf(seq, "\tEnhanced descriptors: %s\n",
		   (priv->dma_cap.enh_desc) ? "Y" : "N");
    seq_printf(seq, "delivery list0 len %d\n", skb_queue_len(&gmac_delivery_skblist[0]));
    seq_printf(seq, "delivery list4 len %d\n", skb_queue_len(&gmac_delivery_skblist[4]));
	return 0;
}

static int bstgmac_sysfs_dma_cap_open(struct inode *inode, struct file *file)
{
	return single_open(file, bstgmac_sysfs_dma_cap_read, inode->i_private);
}

static const struct file_operations bstgmac_dma_cap_fops = {
	.owner = THIS_MODULE,
	.open = bstgmac_sysfs_dma_cap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bstgmac_init_fs(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	/* Create per netdev entries */
	priv->dbgfs_dir = debugfs_create_dir(dev->name, bstgmac_fs_dir);

	if (!priv->dbgfs_dir || IS_ERR(priv->dbgfs_dir)) {
		netdev_err(priv->dev, "ERROR failed to create debugfs directory\n");

		return -ENOMEM;
	}

	/* Entry to report DMA RX/TX rings */
	priv->dbgfs_rings_status =
		debugfs_create_file("descriptors_status", 0444,
				    priv->dbgfs_dir, dev,
				    &bstgmac_rings_status_fops);

	if (!priv->dbgfs_rings_status || IS_ERR(priv->dbgfs_rings_status)) {
		netdev_err(priv->dev, "ERROR creating bstgmac ring debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

	/* Entry to report the DMA HW features */
	priv->dbgfs_dma_cap = debugfs_create_file("dma_cap", 0444,
						  priv->dbgfs_dir,
						  dev, &bstgmac_dma_cap_fops);

	if (!priv->dbgfs_dma_cap || IS_ERR(priv->dbgfs_dma_cap)) {
		netdev_err(priv->dev, "ERROR creating bstgmac MMC debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

/*
  User can configure PHY role by shell command:
  echo 1 > /sys/kernel/debug/bstgmaceth/eth0/gmac0_phyrole 
  echo 2 > /sys/kernel/debug/bstgmaceth/eth0/gmac0_phyrole 
  where 1 means master, 2 means slave.

  And, here is a flaw that node name of gmac1_phyrole in debugfs is 
  not matched with interface name of GMAC1 in system when system uses
  GMAC1 only.

  By Bob.Wang@BST.AI@17:30-2022-06-10
*/
    if (!priv->plat->bus_id) {
        debugfs_create_u32("gmac0_phyrole", 0644, priv->dbgfs_dir, &gmac0_phyrole);
    }
	
    if (priv->plat->bus_id) {
        debugfs_create_u32("gmac1_phyrole", 0644, priv->dbgfs_dir, &gmac1_phyrole);
    }
    return 0;
}

static void bstgmac_exit_fs(struct net_device *dev)
{
	struct bstgmac_priv *priv = netdev_priv(dev);

	debugfs_remove_recursive(priv->dbgfs_dir);
}
#endif /* CONFIG_DEBUG_FS */

static u32 bstgmac_vid_crc32_le(__le16 vid_le)
{
	unsigned char *data = (unsigned char *)&vid_le;
	unsigned char data_byte = 0;
	u32 crc = ~0x0;
	u32 temp = 0;
	int i, bits;

	bits = get_bitmask_order(VLAN_VID_MASK);
	for (i = 0; i < bits; i++) {
		if ((i % 8) == 0)
			data_byte = data[i / 8];

		temp = ((crc & 1) ^ data_byte) & 1;
		crc >>= 1;
		data_byte >>= 1;

		if (temp)
			crc ^= 0xedb88320;
	}

	return crc;
}

static int bstgmac_vlan_update(struct bstgmac_priv *priv, bool is_double)
{
	u32 crc, hash = 0;
	int count = 0;
	u16 vid = 0;

	for_each_set_bit(vid, priv->active_vlans, VLAN_N_VID) {
		__le16 vid_le = cpu_to_le16(vid);
		crc = bitrev32(~bstgmac_vid_crc32_le(vid_le)) >> 28;
		hash |= (1 << crc);
		count++;
	}

	if (!priv->dma_cap.vlhash) {
		if (count > 2) /* VID = 0 always passes filter */
			return -EOPNOTSUPP;

		vid = cpu_to_le16(vid);
		hash = 0;
	}

	return bstgmac_update_vlan_hash(priv, priv->hw, hash, vid, is_double);
}

static int bstgmac_vlan_rx_add_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct bstgmac_priv *priv = netdev_priv(ndev);
	bool is_double = false;
	int ret;

	if (be16_to_cpu(proto) == ETH_P_8021AD)
		is_double = true;

	set_bit(vid, priv->active_vlans);
	ret = bstgmac_vlan_update(priv, is_double);
	if (ret) {
		clear_bit(vid, priv->active_vlans);
		return ret;
	}

	if (priv->hw->num_vlan) {
		ret = bstgmac_add_hw_vlan_rx_fltr(priv, ndev, priv->hw, proto, vid);
		if (ret)
			return ret;
	}

	return 0;
}

static int bstgmac_vlan_rx_kill_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct bstgmac_priv *priv = netdev_priv(ndev);
	bool is_double = false;
	int ret;

	if (be16_to_cpu(proto) == ETH_P_8021AD)
		is_double = true;

	clear_bit(vid, priv->active_vlans);
	if (priv->hw->num_vlan) {
		ret = bstgmac_del_hw_vlan_rx_fltr(priv, ndev, priv->hw, proto, vid);
		if (ret)
			return ret;
	}

	return bstgmac_vlan_update(priv, is_double);
}

static const struct net_device_ops bstgmac_netdev_ops = {
	.ndo_open = bstgmac_open,
	.ndo_start_xmit = bstgmac_xmit,
	.ndo_stop = bstgmac_release,
	.ndo_change_mtu = bstgmac_change_mtu,
	.ndo_fix_features = bstgmac_fix_features,
	.ndo_set_features = bstgmac_set_features,
	.ndo_set_rx_mode = bstgmac_set_rx_mode,
	.ndo_tx_timeout = bstgmac_tx_timeout,
	.ndo_do_ioctl = bstgmac_ioctl,
	.ndo_setup_tc = bstgmac_setup_tc,
	.ndo_select_queue = bstgmac_select_queue,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = bstgmac_poll_controller,
#endif
	.ndo_set_mac_address = bstgmac_set_mac_address,
	.ndo_vlan_rx_add_vid = bstgmac_vlan_rx_add_vid,
    .ndo_vlan_rx_kill_vid = bstgmac_vlan_rx_kill_vid,
};

static void bstgmac_reset_subtask(struct bstgmac_priv *priv)
{
	if (!test_and_clear_bit(BSTGMAC_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(BSTGMAC_DOWN, &priv->state))
		return;

	netdev_err(priv->dev, "Reset adapter.\n");
	//pr_emerg("[%s]%d.",__func__,__LINE__);
	rtnl_lock();
	netif_trans_update(priv->dev);
	while (test_and_set_bit(BSTGMAC_RESETING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(BSTGMAC_DOWN, &priv->state);
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(BSTGMAC_DOWN, &priv->state);
	clear_bit(BSTGMAC_RESETING, &priv->state);
	rtnl_unlock();
}

static void bstgmac_service_task(struct work_struct *work)
{
	struct bstgmac_priv *priv = container_of(work, struct bstgmac_priv,
			service_task);

	bstgmac_reset_subtask(priv);
	clear_bit(BSTGMAC_SERVICE_SCHED, &priv->state);
	
	//pr_emerg("[%s]%d.",__func__,__LINE__);
}

/**
 *  bstgmac_hw_init - Init the MAC device
 *  @priv: driver private structure
 *  Description: this function is to configure the MAC device according to
 *  some platform parameters or the HW capability register. It prepares the
 *  driver to use either ring or chain modes and to setup either enhanced or
 *  normal descriptors.
 */
static int bstgmac_hw_init(struct bstgmac_priv *priv)
{
	int ret;

	/* dwmac-sun8i only work in chain mode */
	if (priv->plat->has_sun8i)
		chain_mode = 1;
	priv->chain_mode = chain_mode;

	/* Initialize HW Interface */
	ret = bstgmac_hwif_init(priv);
	if (ret)
		return ret;

	/* Get the HW capability (new GMAC newer than 3.50a) */
	priv->hw_cap_support = bstgmac_get_hw_features(priv);
	if (priv->hw_cap_support) {
		dev_info(priv->device, "DMA HW capability register supported\n");

		/* We can override some gmac/dma configuration fields: e.g.
		 * enh_desc, tx_coe (e.g. that are passed through the
		 * platform) with the values from the HW capability
		 * register (if supported).
		 */
		priv->plat->enh_desc = priv->dma_cap.enh_desc;
		priv->plat->pmt = priv->dma_cap.pmt_remote_wake_up;
		priv->hw->pmt = priv->plat->pmt;

		/* TXCOE doesn't work in thresh DMA mode */
		if (priv->plat->force_thresh_dma_mode)
			priv->plat->tx_coe = 0;
		else
			priv->plat->tx_coe = priv->dma_cap.tx_coe;

		/* In case of GMAC4 rx_coe is from HW cap register. */
		priv->plat->rx_coe = priv->dma_cap.rx_coe;

		if (priv->dma_cap.rx_coe_type2)
			priv->plat->rx_coe = STMMAC_RX_COE_TYPE2;
		else if (priv->dma_cap.rx_coe_type1)
			priv->plat->rx_coe = STMMAC_RX_COE_TYPE1;

	} else {
		dev_info(priv->device, "No HW DMA feature register supported\n");
	}

	if (priv->plat->rx_coe) {
		priv->hw->rx_csum = priv->plat->rx_coe;
		dev_info(priv->device, "RX Checksum Offload Engine supported\n");
		if (priv->synopsys_id < DWMAC_CORE_4_00)
			dev_info(priv->device, "COE Type %d\n", priv->hw->rx_csum);
	}
	if (priv->plat->tx_coe)
		dev_info(priv->device, "TX Checksum insertion supported\n");

	if (priv->plat->pmt) {
		dev_info(priv->device, "Wake-Up On Lan supported\n");
		device_set_wakeup_capable(priv->device, 1);
	}

	if (priv->dma_cap.tsoen)
		dev_info(priv->device, "TSO supported\n");

	priv->hw->vlan_fail_q_en = priv->plat->vlan_fail_q_en;
	priv->hw->vlan_fail_q = priv->plat->vlan_fail_q;

	/* Run HW quirks, if any */
	if (priv->hwif_quirks) {
		ret = priv->hwif_quirks(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void bstgmac_netif_del_napi(struct bstgmac_priv *priv)
{
	u32 queue, maxq;

	maxq = max(priv->plat->rx_queues_to_use, priv->plat->tx_queues_to_use);
	if(priv->plat->dma_cfg->dma_int_mode != DMA_INT_M_0) {
		for (queue = 0; queue < maxq; queue++) {
			struct bstgmac_channel *ch = &priv->rx_channel[queue];
#if BSTGMAC_RX_NAPI
			if (queue < priv->plat->rx_queues_to_use) {
				netif_napi_del(&ch->rnapi);
			}
#endif
#if BSTGMAC_TX_NAPI
			if (queue < priv->plat->tx_queues_to_use) {
				netif_napi_del(&ch->tnapi);
			}
#endif
		}
	}else{	
		for (queue = 0; queue < maxq; queue++) {
			struct bstgmac_channel *ch = &priv->rx_channel[queue];
			netif_napi_del(&ch->napi);
		}
	}
}

/**
 * bstgmac_dvr_probe
 * @device: device pointer
 * @plat_dat: platform data pointer
 * @res: bstgmac resource pointer
 * Description: this is the main probe function used to
 * call the alloc_etherdev, allocate the priv structure.
 * Return:
 * returns 0 on success, otherwise errno.
 */
int bstgmac_dvr_probe(struct platform_device *pdev,
		     struct plat_stmmacenet_data *plat_dat,
		     struct bstgmac_resources *res)
{
	struct net_device *ndev = NULL;
	struct bstgmac_priv *priv;
	u32 queue;
	u32 maxq;
	int ret = 0;
    struct device *device;
    struct device_node *np = pdev->dev.of_node;

	ndev = alloc_etherdev_mqs(sizeof(struct bstgmac_priv),
				  MTL_MAX_TX_QUEUES,
				  MTL_MAX_RX_QUEUES);
	if (!ndev)
		return -ENOMEM;

    device = &pdev->dev;
	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;

	bstgmac_set_ethtool_ops(ndev);
	priv->pause = pause;
	priv->plat = plat_dat;
	priv->ioaddr = res->addr;
	priv->dev->base_addr = (unsigned long)res->addr;

	priv->dev->irq = res->irq;
	priv->wol_irq = res->wol_irq;
	priv->lpi_irq = res->lpi_irq;
	priv->sfty_ce_irq = res->sfty_ce_irq;
	priv->sfty_uc_irq = res->sfty_uc_irq;
	memcpy(priv->perch_rx_irq,res->perch_rx_irq,sizeof(int)*MTL_MAX_RX_QUEUES);
	memcpy(priv->perch_tx_irq,res->perch_tx_irq,sizeof(int)*MTL_MAX_TX_QUEUES);
	of_property_read_s32(np, "extend-op", &priv->extend_op);
    
	if (!IS_ERR_OR_NULL(res->mac))
		memcpy(priv->dev->dev_addr, res->mac, ETH_ALEN);

	dev_set_drvdata(device, priv->dev);

    dma_set_mask(priv->device, DMA_BIT_MASK(64));
    dma_set_mask_and_coherent(priv->device, DMA_BIT_MASK(64));

	/* Verify driver arguments */
	bstgmac_verify_args();

	/* Allocate workqueue */
	priv->wq = create_workqueue("gmac_rx");//create_singlethread_workqueue("bstgmac_wq");
    if (!priv->wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}

    priv->rxmem = create_workqueue("gmac_rxmem");//create_singlethread_workqueue("bstgmac_wq");
    if (!priv->rxmem) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}
    priv->tx_wq = create_workqueue("gmac_tx");//create_singlethread_workqueue("bstgmac_wq");
    if (!priv->tx_wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}
    
	INIT_WORK(&priv->service_task, bstgmac_service_task);
    INIT_WORK(&priv->mem_mgmt_work, bstgmac_mem_mgmt_work);
    
	/* Override with kernel parameters if supplied XXX CRS XXX
	 * this needs to have multiple instances
	 */
	if ((phyaddr >= 0) && (phyaddr <= 31))
		priv->plat->phy_addr = phyaddr;

	if (priv->plat->stmmac_rst) {
		ret = reset_control_assert(priv->plat->stmmac_rst);
		reset_control_deassert(priv->plat->stmmac_rst);
		/* Some reset controllers have only reset callback instead of
		 * assert + deassert callbacks pair.
		 */
		if (ret == -ENOTSUPP)
			reset_control_reset(priv->plat->stmmac_rst);
	}

	/* Init MAC and get the capabilities */
	ret = bstgmac_hw_init(priv);
	if (ret)
		goto error_hw_init;

	bstgmac_check_ether_addr(priv);

	/* Configure real RX and TX queues */
	netif_set_real_num_rx_queues(ndev, priv->plat->rx_queues_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_queues_to_use);

	ndev->netdev_ops = &bstgmac_netdev_ops;

	ndev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			    NETIF_F_RXCSUM;

	ret = bstgmac_tc_init(priv, priv);
	if (!ret) {
		ndev->hw_features |= NETIF_F_HW_TC;
	}

	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		ndev->hw_features |= NETIF_F_TSO | NETIF_F_TSO6;
		priv->tso = true;
		dev_info(priv->device, "TSO feature enabled\n");
	}
    
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;
	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
#ifdef BSTGMAC_VLAN_TAG_USED
	/* Both mac100 and gmac support receive VLAN tag detection */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_STAG_RX;
	if (priv->dma_cap.vlhash) {
		ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
		ndev->features |= NETIF_F_HW_VLAN_STAG_FILTER;
	}if (priv->dma_cap.vlins) {
		ndev->features |= NETIF_F_HW_VLAN_CTAG_TX;
		if (priv->dma_cap.dvlan)
			ndev->features |= NETIF_F_HW_VLAN_STAG_TX;
	}
#endif
	priv->msg_enable = netif_msg_init(debug, default_msg_level);
	/* MTU range: 46 - hw-specific max */
	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;
	if ((priv->plat->enh_desc) || (priv->synopsys_id >= DWMAC_CORE_4_00))
		ndev->max_mtu = JUMBO_LEN;
	else
		ndev->max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);
	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if ((priv->plat->maxmtu < ndev->max_mtu) &&
	    (priv->plat->maxmtu >= ndev->min_mtu))
		ndev->max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < ndev->min_mtu)
		dev_warn(priv->device,
			 "%s: warning: maxmtu having invalid value (%d)\n",
			 __func__, priv->plat->maxmtu);

	if (flow_ctrl)
		priv->flow_ctrl = FLOW_AUTO;	/* RX/TX pause on */

	maxq = max(priv->plat->rx_queues_to_use, priv->plat->tx_queues_to_use);
	/* Setup channels NAPI */
	if(priv->plat->dma_cfg->dma_int_mode != DMA_INT_M_0){
		
		for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
			struct bstgmac_channel *ch = &priv->rx_channel[queue];
			ch->int_mode = priv->plat->dma_cfg->dma_int_mode;
			ch->priv_data = priv;
			ch->index = queue;

			if (queue < priv->plat->rx_queues_to_use)
				ch->has_rx = true;
#if BSTGMAC_RX_NAPI
			//separate rx tx napi. by well 2020/01/20
			netif_napi_add(ndev, &ch->rnapi, bstgmac_rx_napi_poll, 256); //BSTGMAC_RX_POLL_WEIGHT
 #endif           
            skb_queue_head_init(&gmac_delivery_skblist[priv->plat->bus_id*BSTGMAC_RXCHAN_NUM+queue]); 
		}
		
		for (queue = 0; queue < priv->plat->tx_queues_to_use; queue++) {
				struct bstgmac_channel *ch = &priv->tx_channel[queue];
				ch->int_mode = priv->plat->dma_cfg->dma_int_mode;
				ch->priv_data = priv;
				ch->index = queue;
		
				if (queue < priv->plat->tx_queues_to_use)
					ch->has_tx = true;
#if BSTGMAC_TX_NAPI					
				//separate rx tx napi. by well 2020/01/20
				netif_napi_add(ndev, &ch->tnapi, bstgmac_tx_napi_poll, BSTGMAC_TX_POLL_WEIGHT);
#else
                INIT_WORK(&ch->tx_work, bstgmac_tx_work);
#endif
		}
        
	}else{
		for (queue = 0; queue < maxq; queue++) {
			struct bstgmac_channel *ch = &priv->rx_channel[queue];
			ch->int_mode = priv->plat->dma_cfg->dma_int_mode;
			ch->priv_data = priv;
			ch->index = queue;
		
			if (queue < priv->plat->rx_queues_to_use)
				ch->has_rx = true;
			if (queue < priv->plat->tx_queues_to_use)
				ch->has_tx = true;
		
			netif_napi_add(ndev, &ch->napi, bstgmac_napi_poll,
					   NAPI_POLL_WEIGHT);
		}
	}
	mutex_init(&priv->lock);

	/* If a specific clk_csr value is passed from the platform
	 * this means that the CSR Clock Range selection cannot be
	 * changed at run-time and it is fixed. Viceversa the driver'll try to
	 * set the MDC clock dynamically according to the csr actual
	 * clock input.
	 */
	if (!priv->plat->clk_csr)
		bstgmac_clk_csr_set(priv);
	else
		priv->clk_csr = priv->plat->clk_csr;

	bstgmac_check_pcs_mode(priv);

	if (priv->hw->pcs != BSTGMAC_PCS_TBI &&
	    priv->hw->pcs != BSTGMAC_PCS_RTBI) {
		/* MDIO bus Registration */
		ret = bstgmac_mdio_register(ndev);
		if (ret < 0) {
			dev_err(priv->device,
				"%s: MDIO bus (id: %d) registration failed",
				__func__, priv->plat->bus_id);
			goto error_mdio_register;
		}
	}
    ret = bstgmac_phy_setup(priv);
	if (ret) {
		netdev_err(ndev, "failed to setup phy (%d)\n", ret);
		goto error_phy_setup;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->device, "%s: ERROR %i registering the device\n",
			__func__, ret);
		goto error_netdev_register;
	}
      gmac_priv_g[plat_dat->bus_id] = priv;
    if (plat_dat->bus_id == BSTGMAC0_BUS_ID) {
        spin_lock_init(&gmac0_irqbits_lock);
    } else if ((plat_dat->bus_id == BSTGMAC1_BUS_ID) && (priv->extend_op == BSTA1000_BOARD_ECU)) {
        spin_lock_init(&gmac1_irqbits_lock);
        bstgmac_config_mv88e6352(ndev);
    }
    
#ifdef CONFIG_DEBUG_FS
	ret = bstgmac_init_fs(ndev);
	if (ret < 0)
		netdev_warn(priv->dev, "%s: failed debugFS registration\n",
			    __func__);
#endif

	return ret;

error_netdev_register:
	phylink_destroy(priv->phylink);
error_phy_setup:
	if (priv->hw->pcs != BSTGMAC_PCS_TBI &&
	    priv->hw->pcs != BSTGMAC_PCS_RTBI)
		bstgmac_mdio_unregister(ndev);
error_mdio_register:
	bstgmac_netif_del_napi(priv);
error_hw_init:
	destroy_workqueue(priv->wq);
    destroy_workqueue(priv->rxmem);
    destroy_workqueue(priv->tx_wq);
error_wq:
	free_netdev(ndev);
	return ret;
}
EXPORT_SYMBOL_GPL(bstgmac_dvr_probe);

/**
 * bstgmac_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int bstgmac_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstgmac_priv *priv = netdev_priv(ndev);

	netdev_info(priv->dev, "%s: removing driver", __func__);

#ifdef CONFIG_DEBUG_FS
	bstgmac_exit_fs(ndev);
#endif
	bstgmac_stop_all_dma(priv);

	bstgmac_mac_set(priv, priv->ioaddr, false);
	netif_carrier_off(ndev);
	unregister_netdev(ndev);

	phylink_destroy(priv->phylink);
	if (priv->plat->stmmac_rst)
		reset_control_assert(priv->plat->stmmac_rst);

	clk_disable_unprepare(priv->plat->pclk);
	clk_disable_unprepare(priv->plat->stmmac_clk);

	if (priv->hw->pcs != BSTGMAC_PCS_TBI &&
	    priv->hw->pcs != BSTGMAC_PCS_RTBI)
		bstgmac_mdio_unregister(ndev);
		
	bstgmac_netif_del_napi(priv);

	destroy_workqueue(priv->wq);
    destroy_workqueue(priv->rxmem);
    destroy_workqueue(priv->tx_wq);
	mutex_destroy(&priv->lock);
	free_netdev(ndev);

	perf_debug = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(bstgmac_dvr_remove);

/**
 * bstgmac_suspend - suspend callback
 * @dev: device pointer
 * Description: this is the function to suspend the device and it is called
 * by the platform driver to stop the network queue, release the resources,
 * program the PMT register (for WoL), clean and release driver resources.
 */
int bstgmac_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstgmac_priv *priv = netdev_priv(ndev);

	if (!ndev || !netif_running(ndev))
		return 0;

	if (ndev->phydev)
		phy_stop(ndev->phydev);

	mutex_lock(&priv->lock);

	netif_device_detach(ndev);

	bstgmac_disable_all_queues(priv);

	/* Stop TX/RX DMA */
	bstgmac_stop_all_dma(priv);

	/* Enable Power down mode by programming the PMT regs */
	if (device_may_wakeup(priv->device)) {
		bstgmac_pmt(priv, priv->hw, priv->wolopts);
		priv->irq_wake = 1;
	} else {
		bstgmac_mac_set(priv, priv->ioaddr, false);
		pinctrl_pm_select_sleep_state(priv->device);
		/* Disable clock in case of PWM is off */
		clk_disable(priv->plat->pclk);
		clk_disable(priv->plat->stmmac_clk);
	}
	mutex_unlock(&priv->lock);

	priv->oldlink = false;
	priv->speed = SPEED_UNKNOWN;
	priv->oldduplex = DUPLEX_UNKNOWN;
	return 0;
}
EXPORT_SYMBOL_GPL(bstgmac_suspend);

/**
 * bstgmac_reset_queues_param - reset queue parameters
 * @dev: device pointer
 */
static void bstgmac_reset_queues_param(struct bstgmac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_cnt; queue++) {
		struct bstgmac_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->cur_rx = 0;
		rx_q->dirty_rx = 0;
	}

	for (queue = 0; queue < tx_cnt; queue++) {
		struct bstgmac_tx_queue *tx_q = &priv->tx_queue[queue];

		tx_q->cur_tx = 0;
		tx_q->dirty_tx = 0;
		tx_q->mss = 0;
	}
}

/**
 * bstgmac_resume - resume callback
 * @dev: device pointer
 * Description: when resume this function is invoked to setup the DMA and CORE
 * in a usable state.
 */
int bstgmac_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstgmac_priv *priv = netdev_priv(ndev);

	if (!netif_running(ndev))
		return 0;

	/* Power Down bit, into the PM register, is cleared
	 * automatically as soon as a magic packet or a Wake-up frame
	 * is received. Anyway, it's better to manually clear
	 * this bit because it can generate problems while resuming
	 * from another devices (e.g. serial console).
	 */
	if (device_may_wakeup(priv->device)) {
		mutex_lock(&priv->lock);
		bstgmac_pmt(priv, priv->hw, 0);
		mutex_unlock(&priv->lock);
		priv->irq_wake = 0;
	} else {
		pinctrl_pm_select_default_state(priv->device);
		/* enable the clk previously disabled */
		clk_enable(priv->plat->stmmac_clk);
		clk_enable(priv->plat->pclk);
		/* reset the phy so that it's ready */
		if (priv->mii)
			bstgmac_mdio_reset(priv->mii);
	}

	if (!device_may_wakeup(priv->device) || !priv->plat->pmt) {
		rtnl_lock();
		phylink_start(priv->phylink);
		/* We may have called phylink_speed_down before */
		phylink_speed_up(priv->phylink);
		rtnl_unlock();
	}

    rtnl_lock();
	mutex_lock(&priv->lock);

	bstgmac_reset_queues_param(priv);

	bstgmac_clear_descriptors(priv);

	bstgmac_hw_setup(ndev, false);
	bstgmac_init_tx_coalesce(priv);
	bstgmac_set_rx_mode(ndev);

	bstgmac_restore_hw_vlan_rx_fltr(priv, ndev, priv->hw);
	
	bstgmac_enable_all_queues(priv);

	mutex_unlock(&priv->lock);

	rtnl_unlock();

	phylink_mac_change(priv->phylink, true);

	netif_device_attach(ndev);

	return 0;
}
EXPORT_SYMBOL_GPL(bstgmac_resume);

#ifndef MODULE
static int __init bstgmac_cmdline_opt(char *str)
{
	char *opt;

	if (!str || !*str)
		return -EINVAL;
	while ((opt = strsep(&str, ",")) != NULL) {
		if (!strncmp(opt, "debug:", 6)) {
			if (kstrtoint(opt + 6, 0, &debug))
				goto err;
		} else if (!strncmp(opt, "phyaddr:", 8)) {
			if (kstrtoint(opt + 8, 0, &phyaddr))
				goto err;
		} else if (!strncmp(opt, "buf_sz:", 7)) {
			if (kstrtoint(opt + 7, 0, &buf_sz))
				goto err;
		} else if (!strncmp(opt, "tc:", 3)) {
			if (kstrtoint(opt + 3, 0, &tc))
				goto err;
		} else if (!strncmp(opt, "watchdog:", 9)) {
			if (kstrtoint(opt + 9, 0, &watchdog))
				goto err;
		} else if (!strncmp(opt, "flow_ctrl:", 10)) {
			if (kstrtoint(opt + 10, 0, &flow_ctrl))
				goto err;
		} else if (!strncmp(opt, "pause:", 6)) {
			if (kstrtoint(opt + 6, 0, &pause))
				goto err;
		} else if (!strncmp(opt, "eee_timer:", 10)) {
			if (kstrtoint(opt + 10, 0, &eee_timer))
				goto err;
		} else if (!strncmp(opt, "chain_mode:", 11)) {
			if (kstrtoint(opt + 11, 0, &chain_mode))
				goto err;
		}
	}
	return 0;

err:
	pr_err("%s: ERROR broken module parameter conversion", __func__);
	return -EINVAL;
}

__setup("bstgmaceth=", bstgmac_cmdline_opt);
#endif /* MODULE */

static int __init bstgmac_init(void)
{
#ifdef CONFIG_DEBUG_FS
	/* Create debugfs main directory if it doesn't exist yet */
	if (!bstgmac_fs_dir) {
		bstgmac_fs_dir = debugfs_create_dir(BSTGMAC_RESOURCE_NAME, NULL);

		if (!bstgmac_fs_dir || IS_ERR(bstgmac_fs_dir)) {
			pr_err("ERROR %s, debugfs create directory failed\n",
			       BSTGMAC_RESOURCE_NAME);

			return -ENOMEM;
		}
	}
#endif

	return 0;
}

static void __exit bstgmac_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(bstgmac_fs_dir);
#endif
}

module_init(bstgmac_init)
module_exit(bstgmac_exit)

MODULE_DESCRIPTION("BSTGMAC 1000 Ethernet device driver");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");

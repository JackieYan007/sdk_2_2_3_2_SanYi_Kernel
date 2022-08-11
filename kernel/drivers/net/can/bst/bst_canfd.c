/*
* CANFD driver for BST CANFD
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

/*
* ChangeLog:
* Jan 2020: v1: create by xing.liao@bst.ai
*
*/

#undef DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/can/led.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of_reserved_mem.h>
#include <linux/kfifo.h>
#include <linux/cpumask.h>
#include <linux/signal.h>
#include <linux/debugfs.h>

#include "bst_canfd.h"

#include "../../../dma/virt-dma.h"
#include "../../../dma/bst/bst-axi-dmac.h"

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#if BST_TXRX_DEBUG
#define bst_pr_data(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr_data(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#if BST_ARB_LOST_TEST
static void bst_canfd_tx_test(struct net_device *ndev);
static u32 g_a_isr[5] = {0};
static u32 g_flag =  0;
static u32 g_count =  0;
#endif

#if BST_DEBUG
static u32 cnt_err = 0, g_err_flag = 0;
static u32 g_a_errtype[50] = {0}, g_a_id[50] = {0};
static u32 g_a_tec[50] = {0}, g_a_rec[50] = {0}, g_a_node[50] = {0}, g_a_errisr[50] = {0};
#endif

#if BST_REG_DEBUG
static void bst_canfd_dumpreg(struct bst_canfd_priv *priv)
{
	if (priv == NULL) {
		pr_debug("priv is NULL\n");
		return;
	}
	pr_debug("CAN_GLBCTRL_REG  = 0x%08x        CAN_TXERR_CNT_REG            = 0x%08x\n", readl(priv->base + CAN_GLBCTRL_REG), readl(priv->base + CAN_TXERR_CNT_REG));
	pr_debug("CAN_TMCTRL0_REG  = 0x%08x        CAN_RXERR_CNT_REG            = 0x%08x\n", readl(priv->base + CAN_TMCTRL0_REG), readl(priv->base + CAN_RXERR_CNT_REG));
	pr_debug("CAN_TMCTRL1_REG  = 0x%08x        CAN_REC_CTRLBIT_REG          = 0x%08x\n", readl(priv->base + CAN_TMCTRL1_REG), readl(priv->base + CAN_REC_CTRLBIT_REG));
	pr_debug("CAN_ID_REG       = 0x%08x        CAN_REC_ID_REG               = 0x%08x\n", readl(priv->base + CAN_ID_REG), 	  readl(priv->base + CAN_REC_ID_REG));
	pr_debug("CAN_ID_MASK_REG  = 0x%08x        CAN_OVERWRITE_JUDGE_REG      = 0x%08x\n", readl(priv->base + CAN_ID_MASK_REG), readl(priv->base + CAN_OVERWRITE_JUDGE_REG));
	pr_debug("CAN_SEND_ID_REG  = 0x%08x        CAN_STATUS_MASK_REG          = 0x%08x\n", readl(priv->base + CAN_SEND_ID_REG), readl(priv->base + CAN_STATUS_MASK_REG));
	pr_debug("CAN_TX_DATA0_REG = 0x%08x        CAN_ARB_LOST_CAPTURE_REG     = 0x%08x\n", readl(priv->base + CAN_TX_DATA0_REG), readl(priv->base + CAN_ARB_LOST_CAPTURE_REG));
	pr_debug("CAN_TX_DATA1_REG = 0x%08x        CAN_STATUS_REG               = 0x%08x\n", readl(priv->base + CAN_TX_DATA1_REG), readl(priv->base + CAN_STATUS_REG));
	pr_debug("CAN_TX_DATA2_REG = 0x%08x        CAN_PARITY_RESIDUAL_CTRL_REG = 0x%08x\n", readl(priv->base + CAN_TX_DATA2_REG), readl(priv->base + CAN_PARITY_RESIDUAL_CTRL_REG));
	pr_debug("CAN_TX_DATA3_REG = 0x%08x        CAN_GLBCTRL1_REG             = 0x%08x\n", readl(priv->base + CAN_TX_DATA3_REG), readl(priv->base + CAN_GLBCTRL1_REG));
	pr_debug("CAN_DMA_CTRL_REG = 0x%08x\n", readl(priv->base + CAN_DMA_CTRL_REG));
	
	//pr_debug("CAN_IRQ_TYPE_REG             = 0x%08x\n", readl(priv->base + CAN_IRQ_TYPE_REG)); //读清，可能影响正常功能，所以注释掉
	//pr_debug("CAN_ERR_TYPE_REG             = 0x%08x\n", readl(priv->base + CAN_ERR_TYPE_REG)); //读清，可能影响正常功能，所以注释掉
	//pr_debug("CAN_REC_TYPE_REG             = 0x%08x\n", readl(priv->base + CAN_REC_TYPE_REG)); //读清，可能影响正常功能，所以注释掉
}
#endif
static void bst_canfd_dump_baudrate_cfg(struct bst_canfd_priv *priv)
{
	const struct can_bittiming *bt;
	const struct can_bittiming *dbt;
	u16 brp, sjw, tseg1, tseg2;
	u16 dbrp, dsjw, dtseg1, dtseg2;
	
	if (priv == NULL) {
		pr_debug("priv is NULL\n");
		return;
	}
	bt = &priv->can.bittiming;
	dbt = &priv->can.data_bittiming;
	
	pr_debug("nrate(cfg): bitrate %u, sample_point %u, tq %u, prop_seg %u, phase_seg1 %u, phase_seg2 %u, sjw %u brp %u\n",
			   bt->bitrate, bt->sample_point, bt->tq, bt->prop_seg, 
			   bt->phase_seg1, bt->phase_seg2, bt->sjw, bt->brp);
	pr_debug("drate(cfg): bitrate %u, sample_point %u, tq %u, prop_seg %u, phase_seg1 %u, phase_seg2 %u, sjw %u brp %u\n",
			   dbt->bitrate, dbt->sample_point, dbt->tq, dbt->prop_seg, 
			   dbt->phase_seg1, dbt->phase_seg2, dbt->sjw, dbt->brp);

	brp = bt->brp;
	sjw = bt->sjw ;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;
	pr_debug("nrate(reg): brp %u, sjw %u, tseg1 %u, tseg2 %u\n",
			   brp, sjw, tseg1, tseg2);

	dbrp = dbt->brp;
	dsjw = dbt->sjw;
	dtseg1 = dbt->prop_seg + dbt->phase_seg1 - 1;
	dtseg2 = dbt->phase_seg2 - 1;
	pr_debug("drate(reg): brp %u, sjw %u, tseg1 %u, tseg2 %u\n",
			   dbrp, dsjw, dtseg1, dtseg2);

	return;
}

static void bst_canfd_dump_txrx_stats(struct bst_canfd_priv *priv)
{
	if (priv == NULL) {
		pr_debug("priv is NULL\n");
		return;
	}
	/* 各种帧类型的收发 */
	pr_debug("                          TX pkts   	Tx bytes  		RX pkts   	Rx bytes\n");
	pr_debug("can   basic  data         %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.can_std_pkts, priv->tx.can_std_bytes, priv->rx.can_std_pkts, priv->rx.can_std_bytes);
	pr_debug("can   basic  remote       %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.can_std_rmt_pkts, 0lu, priv->rx.can_std_rmt_pkts, 0lu);
	pr_debug("can   extent data         %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.can_ext_pkts, priv->tx.can_ext_bytes, priv->rx.can_ext_pkts, priv->rx.can_ext_bytes);
	pr_debug("can   extent remote       %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.can_ext_rmt_pkts, 0lu, priv->rx.can_ext_rmt_pkts, 0lu);
	pr_debug("bosch canfd  basic  data  %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.bosch_fd_std_pkts, priv->tx.bosch_fd_std_bytes, priv->rx.bosch_fd_std_pkts, priv->rx.bosch_fd_std_bytes);
	pr_debug("bosch canfd  extent data  %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.bosch_fd_ext_pkts, priv->tx.bosch_fd_ext_bytes, priv->rx.bosch_fd_ext_pkts, priv->rx.bosch_fd_ext_bytes);
	pr_debug("iso   canfd  basic  data  %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.iso_fd_std_pkts, priv->tx.iso_fd_std_bytes, priv->rx.iso_fd_std_pkts, priv->rx.iso_fd_std_bytes);
	pr_debug("iso   canfd  extent data  %-10lu	%-10lu		%-10lu	%-10lu\n", 
		priv->tx.iso_fd_ext_pkts, priv->tx.iso_fd_ext_bytes, priv->rx.iso_fd_ext_pkts, priv->rx.iso_fd_ext_bytes);
	
	return;
}

static void bst_canfd_cur_state(struct bst_canfd_priv *priv)
{
	/* 当前错误状态和计数 */
	const char *cur_states[CAN_STATE_MAX] = {
		"____ERROR-ACTIVE____",
		"____ERROR-WARNING____",
		"____ERROR-PASSIVE____",
		"____BUS-OFF____",
		"____STOPPED____",
		"____SLEEPING____"
	};
		
	if (priv == NULL) {
		pr_debug("priv is NULL\n");
		return;
	}
	if (priv->can.state >= 0 && priv->can.state < CAN_STATE_MAX) {
		pr_debug("CURRENT STATE: %s,    ", cur_states[priv->can.state]);
	} else {
		pr_debug("CURRENT STATE: unknown,    ");
	}
	pr_debug("TXERR=%5u,     RXERR=%5u\n", readl(priv->base + CAN_TXERR_CNT_REG), readl(priv->base + CAN_RXERR_CNT_REG));
	
	pr_debug("#### canfd device stats ####:\n");
	pr_debug("bus_err:          %10u\n", priv->can.can_stats.bus_error);
	pr_debug("error_warning:    %10u\n", priv->can.can_stats.error_warning);
	pr_debug("error_passive:    %10u\n", priv->can.can_stats.error_passive);
	pr_debug("bus_off:          %10u\n", priv->can.can_stats.bus_off);
	pr_debug("arbitration_lost: %10u\n", priv->can.can_stats.arbitration_lost);
	pr_debug("restarts:         %10u\n", priv->can.can_stats.restarts);

	return;
}

static void bst_canfd_dump_errtype_stats(struct bst_canfd_priv *priv)
{
	if (priv == NULL) {
		pr_debug("priv is NULL\n");
		return;
	}

	/* 5种错误检测计数 */
	pr_debug("#### canfd errtype stats ####:\n");
	pr_debug("bit_err:          %10llu\n", priv->errtype_cnt.bit_err);
	pr_debug("stuff_err:        %10llu\n", priv->errtype_cnt.stuff_err);
	pr_debug("crc_err:          %10llu\n", priv->errtype_cnt.crc_err);
	pr_debug("form_err:         %10llu\n", priv->errtype_cnt.form_err);
	pr_debug("ack_err:          %10llu\n", priv->errtype_cnt.ack_err);

	return;
}

static u32 bst_canfd_get_rxtype(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 rxtype;

	rxtype = readl(priv->base + CAN_REC_TYPE_REG);

#if BST_LOOPBACK_TEST
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) { /* loopback模式时，rxtype要特殊获取 */
		if (priv->node < 2) {
			rxtype = priv->loopback_rxtype;
			priv->loopback_rxtype = 0;
			//pr_debug("loopback mode, rxtype = 0x%x\n", rxtype);
		}
	}
#endif

	return rxtype;
}

static u32 bst_canfd_get_dma_rxtype(u32 rx_dma_ctrl)
{
	u32 rxtype;
	
	if (DMA_IS_CANFD_EXTENT_DATA_FRAME(rx_dma_ctrl)) {
		rxtype = RX_CANFD_EXTENT_DATA;
	} else if (DMA_IS_CANFD_BASIC_DATA_FRAME(rx_dma_ctrl)){
		rxtype = RX_CANFD_BASIC_DATA;
	} else if (DMA_IS_CANFD_EXTENT_DATA_FRAME(rx_dma_ctrl)) {
		rxtype = RX_CAN_EXTENT_DATA;
	} else if (DMA_IS_CAN_BASIC_DATA_FRAME(rx_dma_ctrl)) {
		rxtype = RX_CAN_BASIC_DATA;
	} else if (DMA_IS_CAN_EXTENT_REMOTE_FRAME(rx_dma_ctrl)) {
		rxtype = RX_CAN_EXTENT_REMOTE;
	} else if (DMA_IS_CAN_BASIC_REMOTE_FRAME(rx_dma_ctrl)) {
		rxtype = RX_CAN_BASIC_REMOTE;
	} else {
		rxtype = 0;
	}

	return rxtype;
}

static enum can_state his_state = CAN_STATE_STOPPED;
int bst_canfd_get_state(const struct net_device *ndev, enum can_state *state)
{
	u32 i = 0;
	struct bst_canfd_priv *priv = netdev_priv(ndev);

	*state = priv->can.state;

	pr_debug("---------------- canfd[%u] ----------------\n", priv->node);	
	bst_canfd_dump_txrx_stats(priv);
	bst_canfd_cur_state(priv);
	bst_canfd_dump_errtype_stats(priv);
	
	if (his_state != priv->can.state) { /* 减少dump次数 */
		his_state = priv->can.state;
		bst_canfd_dump_baudrate_cfg(priv);
	}

#if BST_REG_DEBUG
	bst_canfd_dumpreg(priv);
#endif

#if BST_ARB_LOST_TEST
	pr_debug("g_count : %d\n", g_count);

	for(i = 0; i < g_count; i++) {
		pr_debug("canfd[%u],g_a_isr[%d]=0x%x\n", priv->node, i, g_a_isr[i]);
		g_a_isr[i] = 0;
	}
	g_count = 0;
	g_flag = 0;
#endif

#if BST_DEBUG
	for(i = 0; i < cnt_err && i < 50; i++) {
		pr_debug("%d : canfd[%d], id = 0x%x, isr=0x%x, errtype=%d, tec=%d, rec=%d\n", 
			i, g_a_node[i], g_a_id[i], g_a_errisr[i],  g_a_errtype[i], g_a_tec[i], g_a_rec[i]);
		g_a_node[i] = 0;
		g_a_id[i] = 0;
		g_a_errisr[i] = 0;
		g_a_errtype[i] = 0;
		g_a_tec[i] = 0;
		g_a_rec[i] = 0;
	}
	cnt_err = 0;
#endif

	return 0;
}

static void bst_canfd_irq_enable(struct net_device *ndev, u32 irq_type)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 enirq = 0;
	
	switch (irq_type) {
		case ENABLE_RX_INT:
			enirq = readl(priv->base + CAN_STATUS_MASK_REG);
			enirq |= ENABLE_RX_INT;
			break;
		case DISABLE_RX_INT:
			enirq = readl(priv->base + CAN_STATUS_MASK_REG);
			enirq &= DISABLE_RX_INT;
			break;
		default:
			enirq = irq_type;
	}
	writel(enirq, priv->base + CAN_STATUS_MASK_REG);
}

static int bst_canfd_get_berr_counter(const struct net_device *ndev,
				      struct can_berr_counter *bec)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);

	bec->txerr = readl(priv->base + CAN_TXERR_CNT_REG);
	bec->rxerr = readl(priv->base + CAN_RXERR_CNT_REG);
//	pr_debug("canfd[%u],txerr=%u,rxerr=%u\n", priv->node, bec->txerr, bec->rxerr);

	return 0;
}

static void bst_canfd_set_error_state(struct net_device *ndev,
				 enum can_state new_state,
				 struct can_frame *cf)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct can_berr_counter bec;
	enum can_state tx_state, rx_state;

	bst_canfd_get_berr_counter(ndev, &bec);

	tx_state = bec.txerr >= bec.rxerr ? new_state : 0;
	rx_state = bec.txerr <= bec.rxerr ? new_state : 0;

	pr_debug("canfd[%d], TEC : %d, REC : %d, tx_state : %d, rx_state : %d, new_state : %d\n",
		priv->node, bec.txerr, bec.rxerr, tx_state, rx_state, new_state);

	can_change_state(ndev, cf, tx_state, rx_state);

	if (cf) {
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
	}

	if (new_state ==  CAN_STATE_BUS_OFF) {
		can_bus_off(ndev);
//		pr_debug("canfd[%u],can_bus_off\n", priv->node);
	}
}

static enum can_state bst_canfd_get_error_state(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct can_berr_counter bec;
	u32 status;

	/* Avoid the bug of state switching in A1000 */
	status = readl(priv->base + CAN_STATUS_REG);

	if(BUS_STATUS_BUS_OFF(status)) {
		return CAN_STATE_BUS_OFF;
	}else {
		bst_canfd_get_berr_counter(ndev, &bec);
		/* err passive */
		if((bec.rxerr >= CAN_ERR_CNT_ERR_PASSIVE) ||
			(bec.txerr >= CAN_ERR_CNT_ERR_PASSIVE)) {
			return CAN_STATE_ERROR_PASSIVE;
		} else if((bec.rxerr >= CAN_ERR_CNT_ERR_WARNING) ||
			(bec.txerr >= CAN_ERR_CNT_ERR_WARNING)) {
			return CAN_STATE_ERROR_WARNING;
		} else {
			return CAN_STATE_ERROR_ACTIVE;
		}
	}
}

static void bst_canfd_update_state_after_rxtx(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	enum can_state old_state = priv->can.state;
	enum can_state new_state;
	
	if (old_state != CAN_STATE_ERROR_WARNING &&
	    old_state != CAN_STATE_ERROR_PASSIVE)
		return;
	
	new_state = bst_canfd_get_error_state(ndev);

	if (new_state != old_state) {
		struct sk_buff *skb;
		struct can_frame *cf;

		skb = alloc_can_err_skb(ndev, &cf);

		bst_canfd_set_error_state(ndev, new_state, skb ? cf : NULL);

		if (skb) {
			struct net_device_stats *stats = &ndev->stats;

			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;
			netif_rx(skb);
		}
	}
}
static void bst_canfd_update_state_in_irq(struct net_device *ndev, enum can_state new_state)
{
	struct sk_buff *skb;
	struct can_frame *cf;

	skb = alloc_can_err_skb(ndev, &cf);
	bst_canfd_set_error_state(ndev, new_state, skb ? cf : NULL);

	if (skb) {
		struct net_device_stats *stats = &ndev->stats;

		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}
}

static void bst_canfd_rx_data_count(struct bst_canfd_priv *priv, 
									struct canfd_frame *cf, u32 rxtype)
{
	/* 接收统计 */
	switch (rxtype) {
	case RX_CAN_BASIC_REMOTE:
		priv->rx.can_std_rmt_pkts++;
		break;
	case RX_CAN_BASIC_DATA:
		priv->rx.can_std_pkts++;
		priv->rx.can_std_bytes += cf->len;
		break;
	case RX_CAN_EXTENT_REMOTE:
		priv->rx.can_ext_rmt_pkts++;
		break;
	case RX_CAN_EXTENT_DATA:
		priv->rx.can_ext_pkts++;
		priv->rx.can_ext_bytes += cf->len;
		break;
	case RX_CANFD_EXTENT_DATA:
		if (priv->is_no_iso) { /* BOSCH CANFD */
			priv->rx.bosch_fd_ext_pkts++;
			priv->rx.bosch_fd_ext_bytes += cf->len;
		} else {
			priv->rx.iso_fd_ext_pkts++;
			priv->rx.iso_fd_ext_bytes += cf->len;
		}
		break;
	case RX_CANFD_BASIC_DATA:
		if (priv->is_no_iso) { /* BOSCH CANFD */
			priv->rx.bosch_fd_std_pkts++;
			priv->rx.bosch_fd_std_bytes += cf->len;
		} else {
			priv->rx.iso_fd_std_pkts++;
			priv->rx.iso_fd_std_bytes += cf->len;
		}
		break;	
	default:
		break;
	}
}

static void bst_canfd_read_rx_data(struct net_device *ndev)
{
	struct net_device_stats *stats = &ndev->stats;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 rxdlc, rxid, rxtype;
	u32 dlc, id;
	int i, j;
	u32 *ptru32;

	rxid = priv->rx_raw_data_out.rx_id;
	rxdlc = priv->rx_raw_data_out.rx_dlc;
	dlc = RX_FRAME_DLC(rxdlc);
	rxtype = priv->rx_raw_data_out.rx_type;
	pr_debug("rxid = 0x%08x, dlc = 0x%x, rxtype = 0x%x\n", rxid, dlc, rxtype);

	id = RX_FRMAE_ID(rxid);
	
	if (IS_CANFD_BASIC_DATA_FRAME(rxtype) || IS_CANFD_EXTENT_DATA_FRAME(rxtype)) { /* CANFD */
		skb = alloc_canfd_skb(ndev, &cf);
		if (!skb) {
			stats->rx_dropped++;
			return;
		}
		cf->len = can_dlc2len(dlc); //CANFD
		if (IS_CANFD_EXTENT_DATA_FRAME(rxtype))
			id |= CAN_EFF_FLAG;
		if (RX_FRAME_BRS(rxdlc))
			cf->flags |= CANFD_BRS;
		/* BOSCH or ISO 11898-1:2015,统一在bst_canfd_start中设置，不允许在接收函数中随意更改 */
	} else if (IS_CAN_EXTENT_DATA_FRAME(rxtype) || IS_CAN_EXTENT_REMOTE_FRAME(rxtype)
	               || IS_CAN_BASIC_DATA_FRAME(rxtype) || IS_CAN_BASIC_REMOTE_FRAME(rxtype)) { /* CAN */
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
		if (!skb) {
			stats->rx_dropped++;
			return;
		}
		cf->len = get_can_dlc(dlc); //CAN
		if (IS_CAN_EXTENT_DATA_FRAME(rxtype) || IS_CAN_EXTENT_REMOTE_FRAME(rxtype)) { /* EXTENT */
			id |= CAN_EFF_FLAG;
		} else { /* BASIC */
			id = RX_FRMAE_ID_BASIC(id);
		}
		if (IS_CAN_EXTENT_REMOTE_FRAME(rxtype) || IS_CAN_BASIC_REMOTE_FRAME(rxtype))
			id |= CAN_RTR_FLAG;
	} else {
		/* err rxtype */
	    return;
	}

	cf->can_id = id;

	if (RX_FRMAE_ESI(rxid)) {
		cf->flags |= CANFD_ESI;
		netdev_dbg(ndev, "ESI Error\n");
	}
	
	pr_debug("canfd[%u],can_id=0x%08x,len=%u,flags=%u,rxdata:\n", priv->node, cf->can_id, cf->len, cf->flags);
	if (!(IS_CAN_EXTENT_REMOTE_FRAME(rxtype) || IS_CAN_BASIC_REMOTE_FRAME(rxtype))) {
		/* 读接收的data */
		ptru32 = (u32 *)cf->data;
		j = 0;
		for (i = 0; i < cf->len; i += 4) {
			/* canfd控制器32bit数据接收寄存器，可以连续读同一地址（0x0016~0x0025地址的任意一个），根据DLC读，最多可以读16次，可以将写入rxfifo里边的数据读出来。 */
			*ptru32 = be32_to_cpu(priv->rx_raw_data_out.rx_data[j]);
			pr_debug(" %08x", *ptru32);
			j++;
			ptru32++;
		}
		pr_debug("\n");
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_receive_skb(skb);

	bst_canfd_rx_data_count(priv, cf, rxtype);
}

static void bst_canfd_read_dma_rx_data(struct net_device *ndev, u32 *rx_dma_data)
{
	struct net_device_stats *stats = &ndev->stats;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 dlc, id, ctrl, rxtype;
	int i, j;
	u32 *ptru32;

	ctrl = rx_dma_data[0];
	id = rx_dma_data[1];
	dlc = DMA_RX_FRAME_DLC(ctrl);
	
	if (DMA_IS_CANFD_BASIC_DATA_FRAME(ctrl) || DMA_IS_CANFD_EXTENT_DATA_FRAME(ctrl)) { /* CANFD */
		skb = alloc_canfd_skb(ndev, &cf);
		if (!skb) {
			stats->rx_dropped++;
			return;
		}
		cf->len = can_dlc2len(dlc); //CANFD
		if (DMA_IS_CANFD_EXTENT_DATA_FRAME(ctrl)) {
			id |= CAN_EFF_FLAG;
		}
		if (DMA_RX_FRAME_BRS(ctrl))
			cf->flags |= CANFD_BRS;
		/* BOSCH or ISO 11898-1:2015,统一在bst_canfd_start中设置，不允许在接收函数中随意更改 */
	} else if (DMA_IS_CAN_EXTENT_DATA_FRAME(ctrl) || DMA_IS_CAN_EXTENT_REMOTE_FRAME(ctrl)
	               || DMA_IS_CAN_BASIC_DATA_FRAME(ctrl) || DMA_IS_CAN_BASIC_REMOTE_FRAME(ctrl)) { /* CAN */
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
		if (!skb) {
			stats->rx_dropped++;
			return;
		}
		cf->len = get_can_dlc(dlc); //CAN
		if (DMA_IS_CAN_EXTENT_DATA_FRAME(ctrl) || DMA_IS_CAN_EXTENT_REMOTE_FRAME(ctrl)) { /* EXTENT */
			id |= CAN_EFF_FLAG;
		} else { /* BASIC */
			id = RX_FRMAE_ID_BASIC(id);
		}
		if (DMA_IS_CAN_EXTENT_REMOTE_FRAME(ctrl) || DMA_IS_CAN_BASIC_REMOTE_FRAME(ctrl)) {
			id |= CAN_RTR_FLAG;
		}
	} else {
		/* err rxtype */
	    return;
	}

	cf->can_id = id;

	if (DMA_RX_FRMAE_ESI(ctrl)) {
		cf->flags |= CANFD_ESI;
		netdev_dbg(ndev, "ESI Error\n");
	}
	
	pr_debug("canfd[%u],can_id=0x%08x,len=%u,flags=%u,rxdata:\n", priv->node, cf->can_id, cf->len, cf->flags);
	if (!(DMA_IS_CAN_EXTENT_REMOTE_FRAME(ctrl) || DMA_IS_CAN_BASIC_REMOTE_FRAME(ctrl))) {
		/* 读接收的data */
		ptru32 = (u32 *)cf->data;
		j = 2;
		for (i = 0; i < cf->len; i += 4) {
			/* canfd控制器32bit数据接收寄存器，可以连续读同一地址（0x0016~0x0025地址的任意一个），根据DLC读，最多可以读16次，可以将写入rxfifo里边的数据读出来。 */
			*ptru32 = be32_to_cpu(rx_dma_data[j]);
			pr_debug(" %08x", *ptru32);
			j++;
			ptru32++;
		}
		pr_debug("\n");
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_receive_skb(skb);

	rxtype = bst_canfd_get_dma_rxtype(rx_dma_data[0]);
	bst_canfd_rx_data_count(priv, cf, rxtype);
}

static int bst_canfd_do_ow_arb_poll(struct net_device *ndev, u32 irq_type)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	skb = alloc_can_err_skb(ndev, &cf);
	if (unlikely(!skb))
		return 0;

	/* rx overwrite interrupt*/
	if (IS_BIT_SET(irq_type, OVERWRITE_FIFO_IRQ)) {
		stats->rx_errors++;
		stats->rx_over_errors++;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	}
	
	/* lost arbitration interrupt */
	if (IS_BIT_SET(irq_type, NODE_LOST_ARBITRATION_IRQ)) {
		priv->can.can_stats.arbitration_lost++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}


static int bst_canfd_do_err_irq(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	enum can_state new_state;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 errtype;
	unsigned long flags;

	skb = alloc_can_err_skb(ndev, &cf);
	if (unlikely(!skb))
		return 0;

	/* err interrupt */
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

#if BST_DEBUG
	errtype = g_a_errtype[(cnt_err - 1 > 0 ? cnt_err - 1 : 0)];
#else
	errtype = readl(priv->base + CAN_ERR_TYPE_REG);
#endif
	/* ack error */
	if (CAN_ACK_ERR_FLAG(errtype)) {
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		priv->can.can_stats.bus_error++;
		priv->errtype_cnt.ack_err++;
		stats->tx_errors++;
		atomic_add(1, &priv->cont_ack_err);
	}

	/* bit error */
	if (CAN_BIT_ERR_FLAG(errtype)) {
		cf->data[2] = CAN_ERR_PROT_BIT;
		priv->can.can_stats.bus_error++;
		priv->errtype_cnt.bit_err++;
		stats->tx_errors++;
	}

	/* stuff error */
	if (CAN_STUFF_ERR_FLAG(errtype)) {
		cf->data[2] = CAN_ERR_PROT_STUFF;
		priv->can.can_stats.bus_error++;
		priv->errtype_cnt.stuff_err++;
		stats->rx_errors++;
	}

	/* form error */
	if (CAN_FORM_ERR_FLAG(errtype)) {
		cf->data[2] = CAN_ERR_PROT_FORM;
		priv->can.can_stats.bus_error++;
		priv->errtype_cnt.form_err++;
		stats->rx_errors++;
	}

	/* crc error */
	if (CAN_CRC_ERR_FLAG(errtype)) {
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		priv->can.can_stats.bus_error++;
		priv->errtype_cnt.crc_err++;
		stats->rx_errors++;
	}

#if defined(CONFIG_ARCH_BSTA1000A)
	spin_lock_irqsave(&priv->status_lock, flags);
	new_state = bst_canfd_get_error_state(ndev);
	if (new_state != priv->can.state)
		bst_canfd_set_error_state(ndev, new_state, cf);
	spin_unlock_irqrestore(&priv->status_lock, flags);
#endif

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

	return 1;
}

static void bst_canfd_rx_callback (void *dma_async_param)
{
	struct net_device *ndev = (struct net_device *)dma_async_param;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	int i = 0;
	int status = 0;
	struct axi_dma_desc *desc = NULL;

	atomic_set(&priv->rx_suc_flag, BST_CANFD_RX_SUCCESS);
	napi_schedule(&priv->napi);
	// status = dma_async_is_tx_complete(priv->chan, priv->cookie, NULL, NULL);
	// pr_err("status : %d\n", status);
}

static int bst_canfd_do_rx_poll(struct net_device *ndev, int quota)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 *rx_dma_data;
	u32 work_done = 0;
	unsigned long flags;
	struct axi_dma_desc *ad_desc;
	struct virt_dma_desc *vd;

	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) { 

		rx_dma_data = &(priv->rb.data[priv->rb.head]);
		while(rx_dma_data[0] != 0) {

			bst_canfd_read_dma_rx_data(ndev, rx_dma_data);
			memset(rx_dma_data, 0, BST_CANFD_DMA_FRAME_MAX_LEN * 4);

			priv->rb.head = (priv->rb.head + BST_CANFD_DMA_FRAME_MAX_LEN) % (priv->rb.size);

			spin_lock_irqsave(&priv->ad_chan->vc.lock, flags);
			vd = vchan_next_desc(&priv->ad_chan->vc);
			list_del(&vd->node);
			list_add_tail(&vd->node, &priv->ad_chan->vc.desc_issued);

			spin_unlock_irqrestore(&priv->ad_chan->vc.lock, flags);

			rx_dma_data = &(priv->rb.data[priv->rb.head]);
			work_done++;

		}
	} else {
		while(!kfifo_is_empty(&priv->rx_kfifo) && work_done < quota) {
			kfifo_out_spinlocked(&priv->rx_kfifo, &priv->rx_raw_data_out, sizeof(priv->rx_raw_data_out), &priv->rx_kfifo_lock);
			bst_canfd_read_rx_data(ndev);
			work_done++;
		}
	}
	if (work_done) {
		spin_lock_irqsave(&priv->status_lock, flags);
		bst_canfd_update_state_after_rxtx(ndev);
		spin_unlock_irqrestore(&priv->status_lock, flags);
	}

	return work_done;
}

static int bst_canfd_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	int work_done = 0;
	u32 irq_type;

	if (priv->irq_type_bak) { /* 之前有保存的情况，就只再用一次 */
		irq_type = priv->irq_type_bak;
		priv->irq_type_bak = 0;
	} else { /* 之前没保存，或保存的已被使用一次, 需要重新读 */
		irq_type = readl(priv->base + CAN_IRQ_TYPE_REG);
		if (IS_BIT_SET(irq_type, TX_SUCCESS_IRQ)) {
			atomic_set(&priv->tx_suc_flag, BST_CANFD_TX_SUCCESS);
			wake_up_process(priv->echo_thread);
		}
	}

	/* err irq type */
	if (IS_BIT_SET(irq_type, CAN_ERR_IRQ)) {
#if BST_DEBUG
		if (IS_BIT_SET(irq_type, CAN_ERR_IRQ)){
			if(cnt_err >= 50){
				cnt_err = 0;
			}
			if (cnt_err < 50) {
				g_a_node[cnt_err] = priv->node;
				g_a_id[cnt_err] = priv->can_id;
				g_a_errisr[cnt_err] = irq_type;
				g_a_errtype[cnt_err] = readl(priv->base + CAN_ERR_TYPE_REG);
				g_a_tec[cnt_err] = readl(priv->base + CAN_TXERR_CNT_REG);
				g_a_rec[cnt_err] = readl(priv->base + CAN_RXERR_CNT_REG);
				cnt_err++;
			}
			g_err_flag = 1;
		}
#endif
		work_done += bst_canfd_do_err_irq(ndev);
	}

	/* lost arbitration or overwrite processing */
	if (IS_BIT_SET(irq_type, NODE_LOST_ARBITRATION_IRQ) ||
		IS_BIT_SET(irq_type, OVERWRITE_FIFO_IRQ)) {
		work_done += bst_canfd_do_ow_arb_poll(ndev, irq_type);
	}

	/* Receive processing */
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) { 
		if (atomic_read(&priv->rx_suc_flag) == BST_CANFD_RX_SUCCESS) {
			work_done += bst_canfd_do_rx_poll(ndev, quota - work_done);
			atomic_set(&priv->rx_suc_flag, BST_CANFD_RX_FAILED);
		}
	} else {
		if (IS_BIT_SET(irq_type, RX_SUCCESS_IRQ)) {
			work_done += bst_canfd_do_rx_poll(ndev, quota - work_done);
		}
	}

	if (work_done < quota) {
		napi_complete_done(napi, work_done);
	}
	return work_done;
}

static irqreturn_t bst_canfd_isr(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 irq_type;
	u8 i = 0, j = 0;
	u8 frame_len = 0, dlc = 0;

	irq_type = readl(priv->base + CAN_IRQ_TYPE_REG);
	priv->irq_type_bak = irq_type;

	/* No interrupt */
	if (unlikely(irq_type == 0)) {
		return IRQ_NONE;
	}
	
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) { 
		if(IS_BIT_SET(irq_type, RX_SUCCESS_IRQ)) {
			// bst_canfd_irq_enable(ndev, DISABLE_RX_INT);
		}
		/* err irq, lost arbitration irq, overwrite irq*/
		if(IS_BITS_SET(irq_type, ERR_LA_OW_IRQ_MASK)) {
			napi_schedule(&priv->napi);
		}
	} else {
		/* rx_success or overwrite or arbitration irq type */
		if (IS_BITS_SET(irq_type, RX_AND_ERR_IRQ_MASK)) {
			if (likely(!kfifo_is_full(&priv->rx_kfifo)) &&
				IS_BIT_SET(irq_type, RX_SUCCESS_IRQ)){
				priv->rx_raw_data_in.rx_type = bst_canfd_get_rxtype(ndev);
				priv->rx_raw_data_in.rx_id = readl(priv->base + CAN_REC_ID_REG);
				priv->rx_raw_data_in.rx_dlc = readl(priv->base + CAN_REC_CTRLBIT_REG);
				dlc = RX_FRAME_DLC(priv->rx_raw_data_in.rx_dlc);
				frame_len = can_dlc2len(dlc);

				for (i = 0, j = 0; i < frame_len; i += 4, j++) {
					priv->rx_raw_data_in.rx_data[j] = readl(priv->base + CAN_RX_DATA0_REG);
				}

				kfifo_in_spinlocked(&priv->rx_kfifo, &priv->rx_raw_data_in, sizeof(priv->rx_raw_data_in), &priv->rx_kfifo_lock);
			}
			napi_schedule(&priv->napi);
		}
	}

	/* tx_success irq type */
	if (IS_BIT_SET(irq_type, TX_SUCCESS_IRQ)) {
		atomic_set(&priv->tx_suc_flag, BST_CANFD_TX_SUCCESS);
		wake_up_process(priv->echo_thread);
	}

#if defined(CONFIG_ARCH_BSTA1000A)
#else
	if (IS_BIT_SET(irq_type, NODE_WARNING_STATUS_IRQ)) {
		bst_canfd_update_state_in_irq(ndev, CAN_STATE_ERROR_WARNING);
	}

	if (IS_BIT_SET(irq_type, NODE_ERROR_PASSIVE_IRQ)) {
		bst_canfd_update_state_in_irq(ndev, CAN_STATE_ERROR_PASSIVE);
	}
		
	if (IS_BIT_SET(irq_type, NODE_BUS_OFF_IRQ)) {
		bst_canfd_update_state_in_irq(ndev, CAN_STATE_BUS_OFF);
	}
#endif
	return IRQ_HANDLED;
}

static const struct can_bittiming_const bst_can_bittiming_const = {
	.name		= KBUILD_MODNAME,
	.tseg1_min	= 1,	/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max	= 31,
	.tseg2_min	= 1,	/* Time segment 2 = phase_seg2 */
	.tseg2_max	= 31,
	.sjw_max	= 16,
	.brp_min	= 1,
	.brp_max	= 1024,
	.brp_inc	= 1,
};

static const struct can_bittiming_const bst_canfd_bittiming_const = {
	.name		= KBUILD_MODNAME,
	.tseg1_min	= 1,	/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max	= 31,
	.tseg2_min	= 1,	/* Time segment 2 = phase_seg2 */
	.tseg2_max	= 31,
	.sjw_max	= 16,
	.brp_min	= 1,
	.brp_max	= 1024,
	.brp_inc	= 1,
};

static int bst_canfd_set_bittiming(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 oldctl0, oldctl1;

	/* Configure bit timing */
	pr_debug("canfd[%u] nrate: brp %u, sjw %u, prop_seg %u, phase_seg1 %u, phase_seg2 %u\n",
			   priv->node, bt->brp, bt->sjw, bt->prop_seg, bt->phase_seg1, bt->phase_seg2);

#if BST_DEFAULT_BAUDRATE /* 默认配置125K(25MHz时钟下)，500K(100MHz时钟下)) */
	brp = 0x8; //0x4----250K(25M时钟),1M(100M时钟)
	sjw = 0x2;
	tseg1 = 0x12;
	tseg2 = 0x4;
#else
    /* 目前测试直接配波特率让can calc bittiming自动计算的配置(直接配波特率的方式)还有问题，现可通过手动配置位时间来配置 */
	/* 直接配波特率的方式,例如:ip link set can0 type can bitrate 125000 dbitrate 250000 fd on */
    /* 手动配置位时间的方式,例如:ip link set can0 type can tq 200 prop-seg 8 phase-seg1 7 phase-seg2 4 dtq 200 dprop-seg 8 dphase-seg1 7 dphase-seg2 4 fd on */
	brp = bt->brp;
	sjw = bt->sjw;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;
#endif

	pr_debug("canfd[%u] nrate: brp %u, sjw %u, tseg1 %u, tseg2 %u\n",
			   priv->node, brp, sjw, tseg1, tseg2);
	if (brp > 0x3FF || sjw > 0xF || tseg1 > 0x3F || tseg2 > 0x1F ||
		tseg1 + sjw > 0x1F || tseg2 + sjw > 0x1F) {
		pr_debug("canfd[%u] nrate: brp %u, sjw %u, tseg1 %u, tseg2 %u error, out of range!\n",
			   priv->node, brp, sjw, tseg1, tseg2);
		return -EOPNOTSUPP;
	}

    oldctl0 = readl(priv->base + CAN_TMCTRL0_REG);
	oldctl0 &= ~CANFD_NCFG_ZONE0;
	writel(oldctl0 | CANFD_NCFG_NSJW(sjw) | CANFD_NCFG_NTSEG2(tseg2) | CANFD_NCFG_NTSEG1(tseg1),
	       priv->base + CAN_TMCTRL0_REG);
	oldctl1 = readl(priv->base + CAN_TMCTRL1_REG);
	oldctl1 &= ~CANFD_NCFG_ZONE1;
	writel(oldctl1 | CANFD_NCFG_NBRP(brp), priv->base + CAN_TMCTRL1_REG);

	return 0;
}

static int bst_canfd_set_data_bittiming(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	const struct can_bittiming *dbt = &priv->can.data_bittiming;
	u16 dbrp, dsjw, dtseg1, dtseg2;
	u32 oldctl0, oldctl1;

	/* Configure data bit timing */
	pr_debug("canfd[%u] drate: brp %u, sjw %u, prop_seg %u, phase_seg1 %u, phase_seg2 %u\n",
			   priv->node, dbt->brp, dbt->sjw, dbt->prop_seg, dbt->phase_seg1, dbt->phase_seg2);

#if BST_DEFAULT_BAUDRATE /* 默认配置250K(25MHz时钟下),1M(100MHz时钟下) */
	dbrp = 0x4; //0x2----500K(25M时钟),2M(100M时钟)
	dsjw = 0x3;
	dtseg1 = 0xf;
	dtseg2 = 0x7;
#else
	/* 目前测试直接配波特率让can calc bittiming自动计算的配置(直接配波特率的方式)还有问题，现可通过手动配置位时间来配置 */
	/* 直接配波特率的方式,例如:ip link set can0 type can bitrate 125000 dbitrate 250000 fd on */
    /* 手动配置位时间的方式,例如:ip link set can0 type can tq 200 prop-seg 8 phase-seg1 7 phase-seg2 4 dtq 200 dprop-seg 8 dphase-seg1 7 dphase-seg2 4 fd on */
	dbrp = dbt->brp;
	dsjw = dbt->sjw;
	dtseg1 = dbt->prop_seg + dbt->phase_seg1 - 1;
	dtseg2 = dbt->phase_seg2 - 1;
#endif

	pr_debug("canfd[%u] drate: brp %u, sjw %u, tseg1 %u, tseg2 %u\n",
			   priv->node, dbrp, dsjw, dtseg1, dtseg2);
	if (dbrp > 0x3FF || dsjw > 0xF || dtseg1 > 0X3F || dtseg2 > 0x1F ||
		dtseg1 + dsjw > 0x1F || dtseg2 + dsjw > 0x1F) {
		pr_debug("canfd[%u] nrate: brp %u, sjw %u, tseg1 %u, tseg2 %u error, out of range!\n",
			   priv->node, dbrp, dsjw, dtseg1, dtseg2);
		return -EOPNOTSUPP;
	}

	oldctl0 = readl(priv->base + CAN_TMCTRL0_REG);
	oldctl0 &= ~CANFD_DCFG_ZONE0;
	writel(CANFD_DCFG_DSJW(dsjw) | CANFD_DCFG_DTSEG2(dtseg2) | CANFD_DCFG_DTSEG1(dtseg1) | oldctl0,
	       priv->base + CAN_TMCTRL0_REG);
	oldctl1 = readl(priv->base + CAN_TMCTRL1_REG);
	oldctl1 &= ~CANFD_DCFG_ZONE1;
	writel(CANFD_DCFG_DBRP(dbrp) | oldctl1, priv->base + CAN_TMCTRL1_REG);

	return 0;
}

void bst_canfd_set_node_id(struct net_device *ndev, u32 node_id)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
//	writel(node_id, priv->base + CAN_ID_REG);
	priv->node_id = node_id;
}

void bst_canfd_set_node_id_mask(struct net_device *ndev, u32 node_id_mask)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
//	writel(node_id_mask, priv->base + CAN_ID_MASK_REG);
	priv->node_id_mask = node_id_mask;
}

static void bst_canfd_set_send_id(struct net_device *ndev, 
				 const u32 ident)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	
	writel(ident, priv->base + CAN_SEND_ID_REG);
	pr_debug("%s %d, canfd[%u],id=0x%x\n", __FUNCTION__, __LINE__, priv->node, ident);

	return;
}

static void bst_canfd_set_filter(struct net_device *ndev, 
				 const u32 id, const u32 mask)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 glbctrl;

	/* enable filter id */
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	glbctrl |= BIT(CAN_FILTER_MODE);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);

	writel(id, priv->base + CAN_ID_REG);
	writel(mask, priv->base + CAN_ID_MASK_REG);

	pr_debug("%s %d, canfd[%u],mask=0x%x,filter_id=0x%x\n", __FUNCTION__, __LINE__,
		priv->node, mask, id);

	return;
}
#if defined(CONFIG_ARCH_BSTA1000A)

#else
static void bst_canfd_dma_init (struct bst_canfd_priv *priv)
{
	// int status = 0;
	int ret;
	void *gdma_select;
	u32 reg;
	int size;
	int i;
	int len;
	struct virt_dma_desc *vd;
	struct virt_dma_chan *vc;
	struct axi_dma_desc *ad_desc;

	gdma_select = ioremap(SAFETY_CRM_ADDR, SZ_4K);
	reg = readl(gdma_select + 0x110);
	
	if (priv->phy_base == BST_CANFD_0_BASE_ADDR) {
		reg |= BIT(11);
	}else if (priv->phy_base == BST_CANFD_1_BASE_ADDR) {
		reg |= BIT(12);
	}else if (priv->phy_base == BST_CANFD_2_BASE_ADDR) {
		reg |= BIT(16);
	}
	writel(reg, gdma_select + 0x110);

	/* init rx dma fifo */
	size = BST_CANFD_DMA_FRAME_MAX_LEN * BST_CANFD_RX_DMA_FIFO_SIZE;
	priv->rb.data = dma_alloc_coherent(priv->dev, size * 4, &priv->rb.data_phy, GFP_KERNEL | GFP_DMA);
	pr_debug("bst_canfd alloc dma, vir addr : 0x%p, phy addr : 0x%llx\n", priv->rb.data, priv->rb.data_phy);
	priv->rb.head = 0;
    priv->rb.tail = 0;
	priv->rb.data_phy_start = priv->rb.data_phy;
    priv->rb.size = size;

	dma_cap_zero(priv->mask);
	dma_cap_set(DMA_SLAVE, priv->mask);
	priv->chan = dma_request_channel(priv->mask, NULL, NULL);

	memset(&priv->cfg, 0, sizeof(priv->cfg));
	priv->cfg.src_addr = priv->phy_base + CAN_RX_DATA0_REG;
	if (priv->phy_base == BST_CANFD_0_BASE_ADDR) {
		priv->cfg.slave_id = 17;
	} else if(priv->phy_base == BST_CANFD_1_BASE_ADDR) {
		priv->cfg.slave_id = 19;
	} else if(priv->phy_base == BST_CANFD_2_BASE_ADDR){
		priv->cfg.slave_id = 21;
	}
	ret = dmaengine_slave_config(priv->chan, &priv->cfg);
	if (ret) {
		dev_err(priv->dev, "can't configure rx dmaengine slave: %d\n",
			ret);
		goto err_dma;
	}

	sg_init_table(&priv->sg, 1);
	sg_dma_address(&priv->sg) = priv->rb.data_phy;
	sg_dma_len(&priv->sg) = priv->rb.size;
	priv->dma_tx = dmaengine_prep_slave_sg(priv->chan, &priv->sg,
						1, DMA_DEV_TO_MEM, DMA_FALGS_CANFD_DEV);
	vd = container_of(priv->dma_tx, struct virt_dma_desc, tx);
	priv->desc = container_of(vd, struct axi_dma_desc, vd);

	vc = container_of(priv->chan, struct virt_dma_chan, chan);
	priv->ad_chan = container_of(vc, struct axi_dma_chan, vc);

	priv->dma_tx->callback = bst_canfd_rx_callback;
	priv->dma_tx->callback_param = priv->ndev;
	dmaengine_submit(priv->dma_tx);

	list_for_each_entry(ad_desc, &priv->desc->xfer_list, xfer_list) {
		ad_desc->vd.tx.callback = bst_canfd_rx_callback;
		ad_desc->vd.tx.callback_param = priv->ndev;
		dmaengine_submit(&ad_desc->vd.tx);
	}

	dma_async_issue_pending(priv->chan);

	// status = dma_async_is_tx_complete(priv->chan, priv->cookie, NULL, NULL);
	// pr_err("status : %d\n", status);

	return;

err_dma:
	dma_release_channel(priv->chan);
}
#endif

static void bst_canfd_start(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	unsigned long timeout;
	u32 glbctrl;

	/* Reset the IP */
	reset_control_reset(priv->rst);

	bst_canfd_set_bittiming(ndev);
	bst_canfd_set_data_bittiming(ndev);
	bst_canfd_set_filter(ndev, priv->node_id, priv->node_id_mask);

	timeout = jiffies + BST_CANFD_TIMEOUT;
	while (BUS_STATUS_BUS_OFF(readl(priv->base + CAN_STATUS_REG))) {
		if (time_after(jiffies, timeout)) {
			pr_err("canfd wait for bus on timeout.\n");
			return;
		}
		usleep_range(500, 10000);
	}
	
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	/* listen_only_mode做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		glbctrl |= BIT(LISTEN_ONLY_MODE);
		pr_debug("======== CAN_CTRLMODE_LISTENONLY on\n");
	} else {
		glbctrl &= ~(BIT(LISTEN_ONLY_MODE));
		pr_debug("======== CAN_CTRLMODE_LISTENONLY off\n");
	}
	
	/* loop_back_mode做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		glbctrl |= BIT(CAN_LOOP_BACK);
		pr_debug("======== CAN_CTRLMODE_LOOPBACK on\n");
	} else {
		glbctrl &= ~(BIT(CAN_LOOP_BACK));
		pr_debug("======== CAN_CTRLMODE_LOOPBACK off\n");
	}

	/* triple_sample_mode做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) {
		glbctrl |= BIT(CAN_TRIPLE_SAMPLE);
		pr_debug("======== CAN_CTRLMODE_3_SAMPLES on\n");
	} else {
		glbctrl &= ~(BIT(CAN_TRIPLE_SAMPLE));
		pr_debug("======== CAN_CTRLMODE_3_SAMPLES off\n");
	}
	
	/* self_test_mode做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_SELF_TEST) { 
		glbctrl |= BIT(SELF_TEST_MODE);
		pr_debug("======== CAN_CTRLMODE_BST_SELF_TEST on\n");
	} else {
		glbctrl &= ~(BIT(SELF_TEST_MODE));
		pr_debug("======== CAN_CTRLMODE_BST_SELF_TEST off\n");
	}

#if defined(CONFIG_ARCH_BSTA1000A)
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) { 
		priv->can.ctrlmode &= ~(CAN_CTRLMODE_BST_DMA_ENABLE);
		netdev_err(ndev, "This chip does not support dma !\n");
	}
	writel(0, priv->base + CAN_DMA_CTRL_REG);
#else
	/* DMA传输开关做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) { 
		writel(BIT(CAN_DMA_RX_CTRL_EN), priv->base + CAN_DMA_CTRL_REG);
		bst_canfd_dma_init(priv);
		pr_debug("======== CAN_CTRLMODE_BST_DMA_ENABLE on\n");
	} else {
		writel(0, priv->base + CAN_DMA_CTRL_REG);
		pr_debug("======== CAN_CTRLMODE_BST_DMA_ENABLE off\n");
	}
#endif

	/* BOSCH or ISO 11898-1:2015 CANFD做相应设置 */
	if ((priv->can.ctrlmode & CAN_CTRLMODE_FD) &&
	    !(priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO)){ /* ISO 11898-1:2015 CANFD */
	    glbctrl |= BIT(CANFD_ISO_SELECT_EN);
		priv->is_no_iso = 0;
		pr_debug("======== ISO 11898-1:2015 CANFD (CAN_CTRLMODE_FD_NON_ISO off)\n");
	}
	if ((priv->can.ctrlmode & CAN_CTRLMODE_FD) && (priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO)) { /* BOSCH CANFD */
		glbctrl &= ~(BIT(CANFD_ISO_SELECT_EN));
		priv->is_no_iso = 1;
		pr_debug("======== BOSCH CANFD (CAN_CTRLMODE_FD_NON_ISO on)\n");
	}

#if defined(CONFIG_ARCH_BSTA1000A)
	/* CAN or CANFD节点做相应设置 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_CAN_NODE) { /* CAN */
		glbctrl |= BIT(CAN_NODES_TYPE_SEL);
		pr_debug("======== CAN_CTRLMODE_BST_CAN_NODE on\n");
	} else {
		glbctrl &= ~(BIT(CAN_NODES_TYPE_SEL));
		pr_debug("======== CAN_CTRLMODE_BST_CAN_NODE off\n");
	}
#endif

	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_FILTER_ALL) { /* 不接收所有帧 */
		glbctrl &= ~(BIT(CAN_FILTER_MODE));   /* refuse match */
		pr_debug("======== CAN_CTRLMODE_BST_FILTER_ALL on\n");
		bst_canfd_set_filter(ndev, CANFD_REFUSE_MATCH_ID, 0);
	} else {
		glbctrl |= BIT(CAN_FILTER_MODE);     /* accept match */
		pr_debug("======== CAN_CTRLMODE_BST_FILTER_ALL off\n");
	}

	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_ERR_INJECT) { /* 错误注入，见bst_canfd_start_xmit */
		pr_debug("======== CAN_CTRLMODE_BST_ERR_INJECT on\n");
	} else {
		pr_debug("======== CAN_CTRLMODE_BST_ERR_INJECT off\n");
	}

	/* Unmask all interrupts */
	bst_canfd_irq_enable(ndev, ENABLE_ALL_INT);

#if defined(CONFIG_ARCH_BSTA1000A)
#else
	/* can bus enable */
	glbctrl |= BIT(CAN_BUS_ENABLE);
#endif
	/* active the node */
	glbctrl &= ~(BIT(RESET_MODE));
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);

	/* Set the state as ERROR_ACTIVE */
	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	atomic_set(&priv->tx_suc_flag, BST_CANFD_TX_FAILED);
	atomic_set(&priv->rx_suc_flag, BST_CANFD_RX_FAILED);
	atomic_set(&priv->cont_ack_err, 0);

	return;
}

static void bst_canfd_stop(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	u32 glbctrl;

	/* not active the node */
#if defined(CONFIG_ARCH_BSTA1000A)
#else
	/* can bus not enable */
	glbctrl &= ~BIT(CAN_BUS_ENABLE);
#endif
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);

	/* Reset the IP */
	reset_control_reset(priv->rst);
	
	/* Clear all pending interrupts，复位自动清除 */
	/* Mask all interrupts */
	bst_canfd_irq_enable(ndev, DISABLE_ALL_INT);

	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_DMA_ENABLE) {
		/* terminate all transfers on specified channels */
		dmaengine_terminate_sync(priv->chan);
		dma_release_channel(priv->chan);
		dma_free_coherent(priv->dev, priv->rb.size * 4, priv->rb.data, priv->rb.data_phy_start);
		priv->rb.head = 0;
		priv->rb.tail = 0;
		priv->rb.size = 0;
		priv->rb.data = NULL;
		priv->rb.data_phy = 0;
		priv->rb.data_phy_start = 0;
	}

	/* Set the state as STOPPED */
	priv->can.state = CAN_STATE_STOPPED;

	return;
}
static int bst_canfd_close(struct net_device *ndev);
static int bst_canfd_echo_skb_thread(void *data)
{
	struct net_device *ndev = (struct net_device *)data;
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	enum can_state new_state;
	unsigned long flags;

	while (!kthread_should_stop()) {
		set_current_state(TASK_UNINTERRUPTIBLE);

		if(atomic_read(&priv->tx_suc_flag) == BST_CANFD_TX_SUCCESS) {
			stats->tx_bytes += can_get_echo_skb(ndev, 0);
			stats->tx_packets++;
			atomic_set(&priv->tx_suc_flag, BST_CANFD_TX_FAILED);
			atomic_set(&priv->cont_ack_err, 0);
			atomic_set(&priv->cont_tx_timeout, 0);
			netif_wake_queue(ndev);

			spin_lock_irqsave(&priv->status_lock, flags);
			bst_canfd_update_state_after_rxtx(ndev);
			spin_unlock_irqrestore(&priv->status_lock, flags);
		}

		/* tx timeout, restart the device */
		if((atomic_read(&priv->cont_ack_err) > CAN_MAX_ACK_ERR) &&
			(priv->can.state != CAN_STATE_BUS_OFF)) {
			/* Mask all interrupts */
			bst_canfd_irq_enable(ndev, DISABLE_ALL_INT);

			priv->can.can_stats.restarts++;
			atomic_add(1, &priv->cont_tx_timeout);

			if (atomic_read(&priv->cont_tx_timeout) > CAN_MAX_CONT_RESTART) {
				netdev_err(ndev, "Restart more than three times, turn off the device!\n");
				can_free_echo_skb(ndev, 0);
				priv->echo_thread = NULL;
				bst_canfd_close(ndev);
				ndev->flags &= ~IFF_UP;
				do_exit(SIGKILL);
			} else {
				netdev_err(ndev, "tx data timeout, reset\n");
				netif_device_detach(ndev);
				can_free_echo_skb(ndev, 0);
				bst_canfd_stop(ndev);

				bst_canfd_start(ndev);
				netif_device_attach(ndev);
			}
		}

#if defined(CONFIG_ARCH_BSTA1000A)
		/* Avoid the bug of state switching in A1000 */
		if((priv->can.state != CAN_STATE_BUS_OFF) &&
			BUS_STATUS_BUS_OFF(readl(priv->base + CAN_STATUS_REG))) {
			netdev_err(ndev, "the node has an error, reset.\n");
			priv->can.can_stats.restarts++;
			netif_device_detach(ndev);
			can_free_echo_skb(ndev, 0);
			bst_canfd_stop(ndev);

			bst_canfd_start(ndev);
			netif_device_attach(ndev);
		}

		new_state = bst_canfd_get_error_state(ndev);
		if (new_state != priv->can.state) {
			spin_lock_irqsave(&priv->status_lock, flags);
			bst_canfd_set_error_state(ndev, new_state, NULL);
			spin_unlock_irqrestore(&priv->status_lock, flags);
		}
#endif

		schedule_timeout(BST_CANFD_TIMEOUT);
	}
	set_current_state(TASK_RUNNING);
	return 0;
}

static int bst_canfd_set_mode(struct net_device *ndev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		bst_canfd_start(ndev);
		netif_wake_queue(ndev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int bst_canfd_open(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	int ret;
	char name_th[16] = "kthread_";

	strcat(name_th, ndev->name);

	pr_debug("%s %d, canfd[%u]\n", __FUNCTION__, __LINE__, priv->node);

	ret = open_candev(ndev);
	if (ret) {
		netdev_err(ndev, "Failed to open CAN device\n");
		return ret;
	}

	/* Register interrupt handler */
	ret = request_irq(ndev->irq, bst_canfd_isr, IRQF_SHARED,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "Failed to request interrupt(irq=%d)\n", ndev->irq);
		goto err_irq;
	}

	bst_canfd_start(ndev);

	/* create tx echo thread */
	priv->echo_thread = kthread_create(bst_canfd_echo_skb_thread, ndev, name_th);
	if(IS_ERR(priv->echo_thread)){
		netdev_err(ndev, "Unable to start kernel thread.\n");
		ret = PTR_ERR(priv->echo_thread);
		priv->echo_thread = NULL;
		goto err_thread;
	}
	wake_up_process(priv->echo_thread);

	napi_enable(&priv->napi);
	if (atomic_read(&priv->cont_tx_timeout) > CAN_MAX_CONT_RESTART) {
		netif_wake_queue(ndev);
	} else {
		netif_start_queue(ndev);
	}
	atomic_set(&priv->cont_tx_timeout, 0);

	return 0;

err_thread:
	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	bst_canfd_stop(ndev);
	free_irq(ndev->irq, ndev);

err_irq:
	close_candev(ndev);
	return ret;
}

static int bst_canfd_close(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);

	pr_debug("%s %d, canfd[%u]\n", __FUNCTION__, __LINE__, priv->node);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	bst_canfd_stop(ndev);
	free_irq(ndev->irq, ndev);

	if (priv->echo_thread) {
		kthread_stop(priv->echo_thread);
		priv->echo_thread = NULL;
	}

	close_candev(ndev);

	return 0;
}

/* 循环注入不同种类的错误 */
void bst_inject_err_loop(struct bst_canfd_priv *priv)
{
	u32 inject = 0;
	static int err_kind = CAN_BIT_ERR_INJECT;	

	if (priv == NULL) {
		return;
	}

	if (err_kind >= CAN_BIT_ERR_INJECT) {
		inject = BIT(err_kind);
		writel(inject, priv->base + CAN_PARITY_RESIDUAL_CTRL_REG); /* 错误注入控制bit是W1R写1清的 */
		pr_debug("inject err_kind = %d (kind: bit, stuff, crc, ack, form)\n", err_kind);
	}
	err_kind++;
	if (err_kind > CAN_FORM_ERR_INJECT) {
		//err_kind = CAN_BIT_ERR_INJECT;
		err_kind = -5; /* 连续注入不同错误后，发送几次正常的帧 */
	}
	
	return;
}

#if BST_ARB_LOST_TEST
static void bst_canfd_tx_test(struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame cf;
	u32 txid, txdlc, txtype, glbctrl;
	u32* ptru32;
	u32 txdata;

	memset(&cf, 0, sizeof(cf));
	cf.can_id = 0x80123456;
	cf.len = 8;
	cf.flags = CANFD_BRS;
	cf.data[0] = 0xA5;
	cf.data[1] = 0xA5;
	cf.data[2] = 0xA5;
	cf.data[3] = 0xA5;
	cf.data[4] = 0xA5;
	cf.data[5] = 0xA5;
	cf.data[6] = 0xA5;
	cf.data[7] = 0xA5;

	txtype = TX_CANFD_EXTENT_DATA_FRAME;	
	/* 配置帧格式 */
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	glbctrl = CAN_SEND_STYLE_CLR(glbctrl);
	glbctrl |= CAN_SEND_STYLE(txtype);
	/* 配制DLC */
	txdlc = can_len2dlc(cf.len);
	glbctrl = CAN_SEND_DLC_CLR(glbctrl);
	glbctrl |= CAN_SEND_DLC(txdlc);
	/* 配置BRS */
	glbctrl |= BIT(CAN_SEND_BRS);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);
	/* 配制SEND ID */
	txid = cf.can_id & CAN_EFF_MASK;
	bst_canfd_set_send_id(ndev, txid);

#if 0
	/* enable filter id */
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	glbctrl |= BIT(CAN_FILTER_MODE);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);
	/* 配置FILTER ID  */
	bst_canfd_set_filter(ndev, 0, ~(0x3));  /*屏蔽ID: 0x3  */
#endif

    /* 配置发送的数据 */
	ptru32 = (u32 *)cf.data;
	txdata = cpu_to_be32(*ptru32); /* 总线使用大端发送，需要转换为大端 */
	writel(txdata, priv->base + CAN_TX_DATA0_REG);
	txdata = cpu_to_be32(*(ptru32+1)); /* 总线使用大端发送，需要转换为大端 */
	writel(txdata, priv->base + CAN_TX_DATA1_REG);
	
	/* tx request */
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG); 
	glbctrl |= BIT(TX_REQUEST);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);
	
	/* 统计 */
	priv->tx.iso_fd_ext_pkts++;
	priv->tx.iso_fd_ext_bytes += cf.len;
	
	return;
}
#endif

static netdev_tx_t bst_canfd_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct bst_canfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 txid, txdlc, txtype, glbctrl, status;
	unsigned long flags;
	int i;
	bool is_brs;
	u32* ptru32;
	u32 txdata;

#if BST_ARB_LOST_TEST 
	if(g_flag)
		g_flag = 0;
#endif
	if (can_dropped_invalid_skb(ndev, skb)) {
		pr_debug("TX can_dropped_invalid_skb!\n");
		return NETDEV_TX_OK;
	}

	/* Sending data, fifo full */
	status = readl(priv->base + CAN_STATUS_REG);
	if (!TRANSMIT_BUFFER_NEED_PREPARE(status)) {
		netdev_err(ndev, "Sending data, fifo full!\n");
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

#if defined(CONFIG_ARCH_BSTA1000A) /* TODO:(A1000上临时处理)为规避硬件bug，长度为0的数据帧不发送 */
	if ((cf->len == 0) && !(cf->can_id & CAN_RTR_FLAG)) {
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}
#endif

#if BST_DEBUG
	priv->can_id = cf->can_id;
#endif

	netif_stop_queue(ndev);
	can_put_echo_skb(skb, ndev, 0);
	spin_lock_irqsave(&priv->tx_lock, flags);

	/* 配制帧格式，can_send_style，标准/扩展，是否远程帧，CAN/CANFD, BOSCH/ISO 11898-1:2015 */
	pr_debug("cf->can_id = 0x%08x\n", cf->can_id);
	if (cf->can_id & CAN_EFF_FLAG) { /* 扩展帧 */
		if (cf->can_id & CAN_RTR_FLAG) { /* 远程帧 */
			txtype = TX_CAN_EXTENT_REMOTE_FRAME;
		} else {
			txtype = TX_CAN_EXTENT_DATA_FRAME;
		}
	} else { /* 标准帧 */
		if (cf->can_id & CAN_RTR_FLAG) { /* 远程帧 */
			txtype = TX_CAN_BASIC_REMOTE_FRAME;
		} else {
			txtype = TX_CAN_BASIC_DATA_FRAME;
		}
	}
	is_brs = 0;
	
	if ((priv->can.ctrlmode & CAN_CTRLMODE_FD) && can_is_canfd_skb(skb)) { /* CANFD BOSCH or ISO11898-1:2015 */
		if (cf->can_id & CAN_EFF_FLAG) {
			txtype = TX_CANFD_EXTENT_DATA_FRAME;
		} else {
			txtype = TX_CANFD_BASIC_DATA_FRAME;
		}
		if (cf->flags & CANFD_BRS)
			is_brs = 1;
	}
	
	/* 配置帧格式 */
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	glbctrl = CAN_SEND_STYLE_CLR(glbctrl);
	glbctrl |= CAN_SEND_STYLE(txtype);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);
	/* BOSCH or ISO11898-1:2015 CANFD 已在bst_canfd_start统一设置,不允许在发送函数中随意更改 */

#if BST_LOOPBACK_TEST
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) { /* loopback模式时，txtype要传给rxtype */
		switch (txtype) {
		case TX_CAN_BASIC_REMOTE_FRAME:
			priv->loopback_rxtype = RX_CAN_BASIC_REMOTE;
			break;
		case TX_CAN_BASIC_DATA_FRAME:
			priv->loopback_rxtype = RX_CAN_BASIC_DATA;
			break;
		case TX_CANFD_BASIC_DATA_FRAME:
			priv->loopback_rxtype = RX_CANFD_BASIC_DATA;
			break;
		case TX_CAN_EXTENT_REMOTE_FRAME:
			priv->loopback_rxtype = RX_CAN_EXTENT_REMOTE;
			break;
		case TX_CAN_EXTENT_DATA_FRAME:
			priv->loopback_rxtype = RX_CAN_EXTENT_DATA;
			break;
		case TX_CANFD_EXTENT_DATA_FRAME:
			priv->loopback_rxtype = RX_CANFD_EXTENT_DATA;
			break;
		default:
			priv->loopback_rxtype = 0;
			break;
		}
	}
#endif

	/* 配制DLC */
	txdlc = can_len2dlc(cf->len);
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
	glbctrl = CAN_SEND_DLC_CLR(glbctrl);
	glbctrl |= CAN_SEND_DLC(txdlc);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);

	/* 配置BRS */
	if (is_brs) {
		glbctrl = readl(priv->base + CAN_GLBCTRL_REG);
		glbctrl |= BIT(CAN_SEND_BRS);
		writel(glbctrl, priv->base + CAN_GLBCTRL_REG);
	}

	/* 配制SEND ID */
	if (cf->can_id & CAN_EFF_FLAG) {
		txid = cf->can_id & CAN_EFF_MASK;
	} else {
		txid = (cf->can_id & CAN_SFF_MASK) << 18; /* 发送时候把id写在send_id[28:18],读回来读receive_id[10:0] */
	}
	pr_debug("cf->can_id = 0x%08x, txid = 0x%08x\n", cf->can_id, txid);
	bst_canfd_set_send_id(ndev, txid);

	pr_debug("%s %d, txtype=%d,txdlc=%d,is_bosch_canfd=%d,is_brs=%d,txid=0x%08x,txdata:\n", 
			__FUNCTION__, __LINE__, txtype, txdlc, priv->is_no_iso, is_brs, txid);

    /* 配置发送的数据 */
	ptru32 = (u32 *)cf->data;
	for (i = 0; i < cf->len; i += 4, ptru32++) {
		txdata = cpu_to_be32(*ptru32); /* 总线使用大端发送，需要转换为大端 */
		pr_debug(" %08x", txdata);
		writel(txdata, priv->base + CAN_TX_DATA0_REG + i);
	}
	pr_debug("\n");


#if BST_INJECT_ERR_EN /* 测试错误注入 */ 
    if (priv->can.ctrlmode & CAN_CTRLMODE_BST_ERR_INJECT) {
		bst_inject_err_loop(priv);
	}
#endif

#if BST_REG_DEBUG
	pr_debug("CAN_GLBCTRL_REG        = 0x%08x\n", readl(priv->base + CAN_GLBCTRL_REG));
#endif

	/* Start the transmission: tx_request */
	atomic_set(&priv->tx_suc_flag, BST_CANFD_TX_FAILED);
	glbctrl = readl(priv->base + CAN_GLBCTRL_REG); 
	glbctrl |= BIT(TX_REQUEST);
	writel(glbctrl, priv->base + CAN_GLBCTRL_REG);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

#if BST_REG_DEBUG
	pr_debug("CAN_GLBCTRL_REG        = 0x%08x\n", readl(priv->base + CAN_GLBCTRL_REG));
#endif

	/* 发送统计 */
	switch (txtype) {
	case TX_CAN_BASIC_DATA_FRAME:
		priv->tx.can_std_pkts++;
		priv->tx.can_std_bytes += cf->len;
		break;
	case TX_CAN_BASIC_REMOTE_FRAME:
		priv->tx.can_std_rmt_pkts++;
		break;
	case TX_CAN_EXTENT_DATA_FRAME:
		priv->tx.can_ext_pkts++;
		priv->tx.can_ext_bytes += cf->len;
		break;
	case TX_CAN_EXTENT_REMOTE_FRAME:
		priv->tx.can_ext_rmt_pkts++;
		break;
	case TX_CANFD_BASIC_DATA_FRAME:
		if (priv->is_no_iso) { /* BOSCH CANFD */
			priv->tx.bosch_fd_std_pkts++;
			priv->tx.bosch_fd_std_bytes += cf->len;
		} else {
			priv->tx.iso_fd_std_pkts++;
			priv->tx.iso_fd_std_bytes += cf->len;
		}
		break;
	case TX_CANFD_EXTENT_DATA_FRAME:
		if (priv->is_no_iso) { /* BOSCH CANFD */
			priv->tx.bosch_fd_ext_pkts++;
			priv->tx.bosch_fd_ext_bytes += cf->len;
		} else {
			priv->tx.iso_fd_ext_pkts++;
			priv->tx.iso_fd_ext_bytes += cf->len;
		}
		break;	
	default:
		break;
	}

	if (priv->can.ctrlmode & CAN_CTRLMODE_BST_ARB_LOST_TEST) { /* 用来测试tx lost arbitration */
		while(!g_flag){
			bst_canfd_tx_test(ndev);
		}
	}

	return NETDEV_TX_OK;
}

static const struct net_device_ops bst_canfd_netdev_ops = {
	.ndo_open	= bst_canfd_open,
	.ndo_stop	= bst_canfd_close,
	.ndo_start_xmit	= bst_canfd_start_xmit,   //发送网络数据
	.ndo_change_mtu	= can_change_mtu,         //网络设备一次最大传输单元
};

static int bst_canfd_probe(struct platform_device *pdev)
{
	u32 node;
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct bst_canfd_priv *priv;
	struct resource *res;
	void __iomem *addr;
	int irq, ret;

	/* 判断是canfd-0还是canfd-1 */
	if (of_property_read_u32(dev->of_node, "bst-index", &node)) {
		return -EINVAL;
	}
	pr_debug("%s %d, canfd[%u]\n", __FUNCTION__, __LINE__, node);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(dev, res);
	irq = platform_get_irq(pdev, 0);
	if (IS_ERR(addr) || irq < 0)
		return -EINVAL;

	ndev = alloc_candev(sizeof(*priv), 1);
	if (!ndev)
		return -ENOMEM;

	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;	/* we support local echo */
	ndev->netdev_ops = &bst_canfd_netdev_ops;
	ndev->tx_queue_len = 1000;

	priv = netdev_priv(ndev);
	priv->phy_base = res->start;
	priv->ndev = ndev;
	priv->dev = dev;
	priv->base = addr;
	priv->node = (u16)node;
	
	spin_lock_init(&priv->rx_kfifo_lock);
	if (kfifo_alloc(&priv->rx_kfifo, BST_CANFD_RX_KFIFO_SIZE, GFP_KERNEL))
		return -ENOMEM;

	/* 时钟配置 can_pclk(APB slave clock 100MHz), can_wclk(can work clock 200MHz) */
	priv->pclk = devm_clk_get(dev, "pclk");
	if (!IS_ERR(priv->pclk)) {
		ret = clk_prepare_enable(priv->pclk);
		if (ret) {
			dev_info(dev, "Cannot enable pclk\n");
			return ret;
		}
	}

	priv->wclk = devm_clk_get(dev, "wclk");
	if (!IS_ERR(priv->wclk)) {
		ret = clk_prepare_enable(priv->wclk);
		if (ret) {
			dev_info(dev, "Cannot enable wclk\n");
			goto err_pclk;
		}
	}

	priv->can.clock.freq = clk_get_rate(priv->wclk);
	pr_debug("canfd clk frequence %ld\n", clk_get_rate(priv->wclk));

	/* 复位canfd控制器，再解复位 */
	priv->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(priv->rst)) {
			ret = PTR_ERR(priv->rst);
			goto err_wclk;
		}
	reset_control_deassert(priv->rst);

	priv->can.state = CAN_STATE_STOPPED;	

	priv->can.bittiming_const	    = &bst_can_bittiming_const;
	priv->can.data_bittiming_const	= &bst_canfd_bittiming_const;
	priv->can.do_set_mode		    = bst_canfd_set_mode;
	priv->can.do_get_berr_counter	= bst_canfd_get_berr_counter;
	priv->can.do_set_bittiming      = bst_canfd_set_bittiming;
	priv->can.do_set_data_bittiming = bst_canfd_set_data_bittiming;
	priv->can.do_set_node_id		= bst_canfd_set_node_id;
	priv->can.do_set_node_id_mask	= bst_canfd_set_node_id_mask;
//	priv->can.do_get_state			= bst_canfd_get_state;

	/* BST CANFD can do both Bosch FD and ISO FD */
	priv->can.ctrlmode = CAN_CTRLMODE_FD | CAN_CTRLMODE_BERR_REPORTING;

	/* BST CANFD can do both Bosch FD and ISO FD */
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					   CAN_CTRLMODE_LISTENONLY |
					   CAN_CTRLMODE_3_SAMPLES |
					   CAN_CTRLMODE_FD |
					   CAN_CTRLMODE_FD_NON_ISO |
					   CAN_CTRLMODE_BERR_REPORTING |
					   CAN_CTRLMODE_ONE_SHOT |
					   CAN_CTRLMODE_PRESUME_ACK |
					   CAN_CTRLMODE_BST_CAN_NODE |
					   CAN_CTRLMODE_BST_FILTER_ALL |
					   CAN_CTRLMODE_BST_SELF_TEST |
					   CAN_CTRLMODE_BST_DMA_ENABLE |
					   CAN_CTRLMODE_BST_ERR_INJECT |
					   CAN_CTRLMODE_BST_ARB_LOST_TEST;


	priv->node_id = CANFD_DEFAULT_ID;
	priv->node_id_mask = CANFD_DEFAULT_ID_MASK;

	spin_lock_init(&priv->tx_lock);
	netif_napi_add(ndev, &priv->napi, bst_canfd_poll, BST_CANFD_NAPI_WEIGHT);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(dev, "Failed to register (ret=%d)\n", ret);
		pr_debug("%s %d, Failed to register (ret=%d)\n", __FUNCTION__, __LINE__, ret);
		goto err_reg;
	}

	dev_info(dev, "Driver registered: regs=0x%llx, irq=%d, clock=%d\n",
		 (u64)priv->base, ndev->irq, priv->can.clock.freq);
	pr_debug("Driver registered: regs=0x%llx, irq=%d, clock=%d\n", 
		 (u64)priv->base, ndev->irq, priv->can.clock.freq);

	/* enable transceiver */
	priv->transceiver_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (gpio_is_valid(priv->transceiver_gpio)) {
		if (of_get_property(dev->of_node, "reset-active-low", NULL)) {
			priv->reset_active = GPIOF_OUT_INIT_LOW;
		} else if (of_get_property(dev->of_node, "reset-active-high", NULL)) {
			priv->reset_active = GPIOF_OUT_INIT_HIGH;
		} else {
			dev_err(dev, "No valid level specified.\n");
			goto err_reset;
		}

		ret = devm_gpio_request_one(dev, priv->transceiver_gpio, priv->reset_active,
					    ndev->name);
		if (ret < 0)
			goto err_reset;
	}

	priv->debug_root =debugfs_create_dir(ndev->name, NULL);
	debugfs_create_u64("bit_err",
			   0444, priv->debug_root,
			   &priv->errtype_cnt.bit_err);
	debugfs_create_u64("stuff_err",
			   0444, priv->debug_root,
			   &priv->errtype_cnt.stuff_err);
	debugfs_create_u64("crc_err",
			   0444, priv->debug_root,
			   &priv->errtype_cnt.crc_err);
	debugfs_create_u64("form_err",
			   0444, priv->debug_root,
			   &priv->errtype_cnt.form_err);
	debugfs_create_u64("ack_err",
			   0444, priv->debug_root,
			   &priv->errtype_cnt.ack_err);

#ifdef CONFIG_BST_CANFD_MEM_TEST
	 priv->dma_addr_test = NULL;
	 priv->kern_addr = NULL;
	 ret = of_reserved_mem_device_init(dev);
	 if (ret) {
		dev_err(dev, "of_reserved_mem_device_init fail, ret: %d", ret);
		return 0;
	 }
	 ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(36));
	 if (ret) {
		 dev_err(dev, "dma_set_coherent_mask fail, ret: %d", ret);
		 of_reserved_mem_device_release(&pdev->dev);
	 } else {
		 priv->kern_addr = dma_alloc_coherent(dev, 0x100, &priv->dma_addr_test, GFP_KERNEL);
		 bst_pr("************ bst_canfd alloc dma, vir addr : 0x%llx, phy addr : 0x%llx\n", (phys_addr_t)priv->kern_addr, priv->dma_addr_test);
	 }
#endif

	return 0;

err_reset:
	unregister_candev(ndev);

err_reg:
	free_candev(ndev);
	reset_control_assert(priv->rst);

err_wclk:
	if (!IS_ERR(priv->wclk))
		clk_disable_unprepare(priv->wclk);

err_pclk:
	if (!IS_ERR(priv->pclk))
		clk_disable_unprepare(priv->pclk);	

	kfifo_free(&priv->rx_kfifo);
	return ret;
}

static int bst_canfd_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct bst_canfd_priv *priv = netdev_priv(ndev);

	/* reset canfd controller */
	reset_control_reset(priv->rst);

	pr_debug("%s %d, canfd[%u]\n", __FUNCTION__, __LINE__, priv->node);
	
	/* disable interrupt */
	bst_canfd_irq_enable(ndev, DISABLE_ALL_INT);

	unregister_candev(ndev);
	reset_control_assert(priv->rst);
	clk_disable_unprepare(priv->wclk);
	clk_disable_unprepare(priv->pclk);
	kfifo_free(&priv->rx_kfifo);
	
	platform_set_drvdata(pdev, NULL);
	netif_napi_del(&priv->napi);
	free_candev(ndev);

	debugfs_remove_recursive(priv->debug_root);
#ifdef CONFIG_BST_CANFD_MEM_TEST
	if (priv->kern_addr) {
		dma_free_coherent(&pdev->dev, 0x100, priv->kern_addr, priv->dma_addr_test);
		of_reserved_mem_device_release(&pdev->dev);
	}
#endif

	return 0;
}

static int __maybe_unused bst_canfd_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused bst_canfd_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(bst_canfd_pm_ops, bst_canfd_suspend,
			 bst_canfd_resume);

static const struct of_device_id bst_canfd_of_table[] = {
	{ .compatible = "BST,bst-canfd" },  /* 与dts节点中compatible属性匹配 */
	{ }
};

MODULE_DEVICE_TABLE(of, bst_canfd_of_table);

static struct platform_driver bst_canfd_driver = {
	.driver = {
		.name = BSTCANFD_DRV_NAME,
		.of_match_table = of_match_ptr(bst_canfd_of_table),
		.pm = &bst_canfd_pm_ops,
	},
	.probe = bst_canfd_probe,
	.remove = bst_canfd_remove,
};

module_platform_driver(bst_canfd_driver);

MODULE_DESCRIPTION("BST CANFD driver");
MODULE_AUTHOR("Xing Liao <xing.liao@bst.ai>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" BSTCANFD_DRV_NAME);

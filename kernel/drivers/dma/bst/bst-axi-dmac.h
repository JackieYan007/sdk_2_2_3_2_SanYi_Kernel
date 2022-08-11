// SPDX-License-Identifier:  GPL-2.0
// (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)

/*
 * Synopsys DesignWare AXI DMA Controller driver.
 *
 * Author: Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>
 */

#ifndef _AXI_DMA_PLATFORM_H
#define _AXI_DMA_PLATFORM_H

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/types.h>

#include "../virt-dma.h"

enum{
	BST_AXI_DMA_TYPE_COMMON,
	BST_AXI_DMA_TYPE_BST
};

#define DMAC_MAX_CHANNELS	8
#define DMAC_MAX_MASTERS	2
#define DMAC_MAX_BLK_SIZE	0x200000

struct bst_axi_dma_hcfg {
	u32	nr_channels;
	u32	nr_masters;
	u32	cmn_irq;
	u32	m_data_width;
	u32	block_size[DMAC_MAX_CHANNELS];
	u32	priority[DMAC_MAX_CHANNELS];
	u32	chan_irq[DMAC_MAX_CHANNELS];
	/* maximum supported axi burst length */
	u32	axi_rw_burst_len;
	bool	restrict_axi_burst_len;
};

struct axi_dma_chan {
	struct axi_dma_chip		*chip;
	void __iomem			*chan_regs;
	u8				id;
	atomic_t			descs_allocated;

	struct virt_dma_chan		vc;

	/* these other elements are all protected by vc.lock */
	bool				is_paused;

	struct dma_slave_config		dma_sconfig;
};

struct bst_axi_dma {
	struct dma_device	dma;
	struct bst_axi_dma_hcfg	*hdata;
	struct dma_pool		*desc_pool;
	struct reset_control	*reset;

	/* channels */
	struct axi_dma_chan	*chan;
};

struct axi_dma_chip {
	struct device		*dev;
	void __iomem		*regs;
	struct clk		*core_clk;
	struct clk		*cfgr_clk;
	struct bst_axi_dma	*bst;
	int			support_slave;
};

/* LLI == Linked List Item */
struct __packed axi_dma_lli {
	__le64		sar;
	__le64		dar;
	__le32		block_ts_lo;
	__le32		block_ts_hi;
	__le64		llp;
	__le32		ctl_lo;
	__le32		ctl_hi;
	__le32		sstat;
	__le32		dstat;
	__le32		status_lo;
	__le32		ststus_hi;
	__le32		reserved_lo;
	__le32		reserved_hi;
};

struct axi_dma_desc {
	struct axi_dma_lli		lli;

	struct virt_dma_desc		vd;
	struct axi_dma_chan		*chan;
	struct list_head		xfer_list;
};

static inline struct device *dchan2dev(struct dma_chan *dchan)
{
	return &dchan->dev->device;
}

static inline struct device *chan2dev(struct axi_dma_chan *chan)
{
	return &chan->vc.chan.dev->device;
}

static inline struct axi_dma_desc *vd_to_axi_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct axi_dma_desc, vd);
}

static inline struct axi_dma_chan *vc_to_axi_dma_chan(struct virt_dma_chan *vc)
{
	return container_of(vc, struct axi_dma_chan, vc);
}

static inline struct axi_dma_chan *dchan_to_axi_dma_chan(struct dma_chan *dchan)
{
	return vc_to_axi_dma_chan(to_virt_chan(dchan));
}


#define COMMON_REG_LEN		0x100
#define CHAN_REG_LEN		0x100

/* Common registers offset */
#define DMAC_ID			0x000 /* R DMAC ID */
#define DMAC_COMPVER		0x008 /* R DMAC Component Version */
#define DMAC_CFG		0x010 /* R/W DMAC Configuration */
#define DMAC_CHEN		0x018 /* R/W DMAC Channel Enable */
#define DMAC_CHEN_L		0x018 /* R/W DMAC Channel Enable 00-31 */
#define DMAC_CHEN_H		0x01C /* R/W DMAC Channel Enable 32-63 */
#define DMAC_INTSTATUS		0x030 /* R DMAC Interrupt Status */
#define DMAC_COMMON_INTCLEAR	0x038 /* W DMAC Interrupt Clear */
#define DMAC_COMMON_INTSTATUS_ENA 0x040 /* R DMAC Interrupt Status Enable */
#define DMAC_COMMON_INTSIGNAL_ENA 0x048 /* R/W DMAC Interrupt Signal Enable */
#define DMAC_COMMON_INTSTATUS	0x050 /* R DMAC Interrupt Status */
#define DMAC_RESET		0x058 /* R DMAC Reset Register1 */

/* DMA channel registers offset */
#define CH_SAR			0x000 /* R/W Chan Source Address */
#define CH_DAR			0x008 /* R/W Chan Destination Address */
#define CH_BLOCK_TS		0x010 /* R/W Chan Block Transfer Size */
#define CH_CTL			0x018 /* R/W Chan Control */
#define CH_CTL_L		0x018 /* R/W Chan Control 00-31 */
#define CH_CTL_H		0x01C /* R/W Chan Control 32-63 */
#define CH_CFG			0x020 /* R/W Chan Configuration */
#define CH_CFG_L		0x020 /* R/W Chan Configuration 00-31 */
#define CH_CFG_H		0x024 /* R/W Chan Configuration 32-63 */
#define CH_LLP			0x028 /* R/W Chan Linked List Pointer */
#define CH_STATUS		0x030 /* R Chan Status */
#define CH_SWHSSRC		0x038 /* R/W Chan SW Handshake Source */
#define CH_SWHSDST		0x040 /* R/W Chan SW Handshake Destination */
#define CH_BLK_TFR_RESUMEREQ	0x048 /* W Chan Block Transfer Resume Req */
#define CH_AXI_ID		0x050 /* R/W Chan AXI ID */
#define CH_AXI_QOS		0x058 /* R/W Chan AXI QOS */
#define CH_SSTAT		0x060 /* R Chan Source Status */
#define CH_DSTAT		0x068 /* R Chan Destination Status */
#define CH_SSTATAR		0x070 /* R/W Chan Source Status Fetch Addr */
#define CH_DSTATAR		0x078 /* R/W Chan Destination Status Fetch Addr */
#define CH_INTSTATUS_ENA	0x080 /* R/W Chan Interrupt Status Enable */
#define CH_INTSTATUS		0x088 /* R/W Chan Interrupt Status */
#define CH_INTSIGNAL_ENA	0x090 /* R/W Chan Interrupt Signal Enable */
#define CH_INTCLEAR		0x098 /* W Chan Interrupt Clear */


/* DMAC_CFG */
#define DMAC_EN_POS			0
#define DMAC_EN_MASK			BIT(DMAC_EN_POS)

#define INT_EN_POS			1
#define INT_EN_MASK			BIT(INT_EN_POS)

#define DMAC_CHAN_EN_SHIFT		0
#define DMAC_CHAN_EN_WE_SHIFT		8

#define DMAC_CHAN_SUSP_SHIFT		16
#define DMAC_CHAN_SUSP_WE_SHIFT		24

/* CH_CTL_H */
#define CH_CTL_H_ARLEN_EN		BIT(6)
#define CH_CTL_H_ARLEN_POS		7
#define CH_CTL_H_AWLEN_EN		BIT(15)
#define CH_CTL_H_AWLEN_POS		16
#define CH_CTL_H_IOC_BlkTf_EN	BIT(26)

#define INT_CMN_UDF_REG_MASK	BIT(8)
#define INT_CMN_WOH_ERR_MASK	BIT(3)
#define INT_CMN_R2WO_ERR_MASK	BIT(2)
#define INT_CMN_W2RO_ERR_MASK	BIT(1)
#define INT_CMN_DEC_ERR_MASK	BIT(0)

#define INT_CMN_MASK (INT_CMN_UDF_REG_MASK | INT_CMN_WOH_ERR_MASK  | INT_CMN_DEC_ERR_MASK)

enum {
	BSTAXIDMAC_ARWLEN_1		= 0,
	BSTAXIDMAC_ARWLEN_2		= 1,
	BSTAXIDMAC_ARWLEN_4		= 3,
	BSTAXIDMAC_ARWLEN_8		= 7,
	BSTAXIDMAC_ARWLEN_16		= 15,
	BSTAXIDMAC_ARWLEN_32		= 31,
	BSTAXIDMAC_ARWLEN_64		= 63,
	BSTAXIDMAC_ARWLEN_128		= 127,
	BSTAXIDMAC_ARWLEN_256		= 255,
	BSTAXIDMAC_ARWLEN_MIN		= BSTAXIDMAC_ARWLEN_1,
	BSTAXIDMAC_ARWLEN_MAX		= BSTAXIDMAC_ARWLEN_256
};

#define CH_CTL_H_LLI_LAST		BIT(30)
#define CH_CTL_H_LLI_VALID		BIT(31)

/* CH_CTL_L */
#define CH_CTL_L_LAST_WRITE_EN		BIT(30)

#define CH_CTL_L_DST_MSIZE_POS		18
#define CH_CTL_L_SRC_MSIZE_POS		14

enum {
	BSTAXIDMAC_BURST_TRANS_LEN_1	= 0,
	BSTAXIDMAC_BURST_TRANS_LEN_4,
	BSTAXIDMAC_BURST_TRANS_LEN_8,
	BSTAXIDMAC_BURST_TRANS_LEN_16,
	BSTAXIDMAC_BURST_TRANS_LEN_32,
	BSTAXIDMAC_BURST_TRANS_LEN_64,
	BSTAXIDMAC_BURST_TRANS_LEN_128,
	BSTAXIDMAC_BURST_TRANS_LEN_256,
	BSTAXIDMAC_BURST_TRANS_LEN_512,
	BSTAXIDMAC_BURST_TRANS_LEN_1024
};

#define CH_CTL_L_DST_WIDTH_POS		11
#define CH_CTL_L_SRC_WIDTH_POS		8

#define CH_CTL_L_DST_INC_POS		6
#define CH_CTL_L_SRC_INC_POS		4
enum {
	BSTAXIDMAC_CH_CTL_L_INC		= 0,
	BSTAXIDMAC_CH_CTL_L_NOINC
};

#define CH_CTL_L_DST_MAST		BIT(2)
#define CH_CTL_L_SRC_MAST		BIT(0)

/* CH_CFG_H */
#define CH_CFG_H_DST_OSR_LMT_POS	27
#define CH_CFG_H_SRC_OSR_LMT_POS	23
#define CH_CFG_H_PRIORITY_POS		17
#define CH_CFG_H_HS_SEL_DST_POS		4
#define CH_CFG_H_HS_SEL_SRC_POS		3
enum {
	BSTAXIDMAC_HS_SEL_HW		= 0,
	BSTAXIDMAC_HS_SEL_SW
};

#define CH_CFG_H_TT_FC_POS		0
enum {
	BSTAXIDMAC_TT_FC_MEM_TO_MEM_DMAC	= 0,
	BSTAXIDMAC_TT_FC_MEM_TO_PER_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_MEM_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_PER_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_MEM_SRC,
	BSTAXIDMAC_TT_FC_PER_TO_PER_SRC,
	BSTAXIDMAC_TT_FC_MEM_TO_PER_DST,
	BSTAXIDMAC_TT_FC_PER_TO_PER_DST
};

/* CH_CFG_L */
#define CH_CFG_L_DST_MULTBLK_TYPE_POS	2
#define CH_CFG_L_SRC_MULTBLK_TYPE_POS	0
enum {
	BSTAXIDMAC_MBLK_TYPE_CONTIGUOUS	= 0,
	BSTAXIDMAC_MBLK_TYPE_RELOAD,
	BSTAXIDMAC_MBLK_TYPE_SHADOW_REG,
	BSTAXIDMAC_MBLK_TYPE_LL
};

/**
 * BST AXI DMA channel interrupts
 *
 * @BSTAXIDMAC_IRQ_NONE: Bitmask of no one interrupt
 * @BSTAXIDMAC_IRQ_BLOCK_TRF: Block transfer complete
 * @BSTAXIDMAC_IRQ_DMA_TRF: Dma transfer complete
 * @BSTAXIDMAC_IRQ_SRC_TRAN: Source transaction complete
 * @BSTAXIDMAC_IRQ_DST_TRAN: Destination transaction complete
 * @BSTAXIDMAC_IRQ_SRC_DEC_ERR: Source decode error
 * @BSTAXIDMAC_IRQ_DST_DEC_ERR: Destination decode error
 * @BSTAXIDMAC_IRQ_SRC_SLV_ERR: Source slave error
 * @BSTAXIDMAC_IRQ_DST_SLV_ERR: Destination slave error
 * @BSTAXIDMAC_IRQ_LLI_RD_DEC_ERR: LLI read decode error
 * @BSTAXIDMAC_IRQ_LLI_WR_DEC_ERR: LLI write decode error
 * @BSTAXIDMAC_IRQ_LLI_RD_SLV_ERR: LLI read slave error
 * @BSTAXIDMAC_IRQ_LLI_WR_SLV_ERR: LLI write slave error
 * @BSTAXIDMAC_IRQ_INVALID_ERR: LLI invalid error or Shadow register error
 * @BSTAXIDMAC_IRQ_MULTIBLKTYPE_ERR: Slave Interface Multiblock type error
 * @BSTAXIDMAC_IRQ_DEC_ERR: Slave Interface decode error
 * @BSTAXIDMAC_IRQ_WR2RO_ERR: Slave Interface write to read only error
 * @BSTAXIDMAC_IRQ_RD2RWO_ERR: Slave Interface read to write only error
 * @BSTAXIDMAC_IRQ_WRONCHEN_ERR: Slave Interface write to channel error
 * @BSTAXIDMAC_IRQ_SHADOWREG_ERR: Slave Interface shadow reg error
 * @BSTAXIDMAC_IRQ_WRONHOLD_ERR: Slave Interface hold error
 * @BSTAXIDMAC_IRQ_LOCK_CLEARED: Lock Cleared Status
 * @BSTAXIDMAC_IRQ_SRC_SUSPENDED: Source Suspended Status
 * @BSTAXIDMAC_IRQ_SUSPENDED: Channel Suspended Status
 * @BSTAXIDMAC_IRQ_DISABLED: Channel Disabled Status
 * @BSTAXIDMAC_IRQ_ABORTED: Channel Aborted Status
 * @BSTAXIDMAC_IRQ_ALL_ERR: Bitmask of all error interrupts
 * @BSTAXIDMAC_IRQ_ALL: Bitmask of all interrupts
 */
enum {
	BSTAXIDMAC_IRQ_NONE		= 0,
	BSTAXIDMAC_IRQ_BLOCK_TRF		= BIT(0),
	BSTAXIDMAC_IRQ_DMA_TRF		= BIT(1),
	BSTAXIDMAC_IRQ_SRC_TRAN		= BIT(3),
	BSTAXIDMAC_IRQ_DST_TRAN		= BIT(4),
	BSTAXIDMAC_IRQ_SRC_DEC_ERR	= BIT(5),
	BSTAXIDMAC_IRQ_DST_DEC_ERR	= BIT(6),
	BSTAXIDMAC_IRQ_SRC_SLV_ERR	= BIT(7),
	BSTAXIDMAC_IRQ_DST_SLV_ERR	= BIT(8),
	BSTAXIDMAC_IRQ_LLI_RD_DEC_ERR	= BIT(9),
	BSTAXIDMAC_IRQ_LLI_WR_DEC_ERR	= BIT(10),
	BSTAXIDMAC_IRQ_LLI_RD_SLV_ERR	= BIT(11),
	BSTAXIDMAC_IRQ_LLI_WR_SLV_ERR	= BIT(12),
	BSTAXIDMAC_IRQ_INVALID_ERR	= BIT(13),
	BSTAXIDMAC_IRQ_MULTIBLKTYPE_ERR	= BIT(14),
	BSTAXIDMAC_IRQ_DEC_ERR		= BIT(16),
	BSTAXIDMAC_IRQ_WR2RO_ERR		= BIT(17),
	BSTAXIDMAC_IRQ_RD2RWO_ERR	= BIT(18),
	BSTAXIDMAC_IRQ_WRONCHEN_ERR	= BIT(19),
	BSTAXIDMAC_IRQ_SHADOWREG_ERR	= BIT(20),
	BSTAXIDMAC_IRQ_WRONHOLD_ERR	= BIT(21),
	BSTAXIDMAC_IRQ_LOCK_CLEARED	= BIT(27),
	BSTAXIDMAC_IRQ_SRC_SUSPENDED	= BIT(28),
	BSTAXIDMAC_IRQ_SUSPENDED		= BIT(29),
	BSTAXIDMAC_IRQ_DISABLED		= BIT(30),
	BSTAXIDMAC_IRQ_ABORTED		= BIT(31),
	BSTAXIDMAC_IRQ_ALL_ERR		= (GENMASK(21, 16) | GENMASK(14, 5)),
	BSTAXIDMAC_IRQ_ALL		= GENMASK(31, 0)
};

enum {
	BSTAXIDMAC_TRANS_WIDTH_8		= 0,
	BSTAXIDMAC_TRANS_WIDTH_16,
	BSTAXIDMAC_TRANS_WIDTH_32,
	BSTAXIDMAC_TRANS_WIDTH_64,
	BSTAXIDMAC_TRANS_WIDTH_128,
	BSTAXIDMAC_TRANS_WIDTH_256,
	BSTAXIDMAC_TRANS_WIDTH_512,
	BSTAXIDMAC_TRANS_WIDTH_MAX	= BSTAXIDMAC_TRANS_WIDTH_512
};

#define CH_CFG_L_DST_PERIPHERAL_POS 11
#define CH_CFG_L_SRC_PERIPHERAL_POS 4
/*
 * BST Soc gdma slave ID reference table.(slave_id)
 */
enum {
	QSPI0_DMA_TX_REQ = 0, 	QSPI0_DMA_RX_REQ = 1,
	QSPI1_DMA_TX_REQ = 2, 	QSPI1_DMA_RX_REQ = 3,
	LSP0_DMA_I2SM_TX_REQ = 4, 	LSP0_DMA_I2SM_RX_REQ = 5,
	LSP0_DMA_UART0_TX_REQ = 6,	LSP0_DMA_UART0_RX_REQ = 7,
	LSP0_DMA_UART1_TX_REQ = 8,	LSP0_DMA_UART1_RX_REQ = 9,
	LSP1_DMA_I2SS_TX_REQ = 10,	LSP1_DMA_I2SS_RX_REQ = 11,
	LSP1_DMA_UART0_TX_REQ = 12,	LSP1_DMA_UART0_RX_REQ = 13,
	LSP1_DMA_UART1_TX_REQ = 14,	LSP1_DMA_UART1_RX_REQ = 15,
	LSP0_DMA_CAN0_TX_REQ = 16, 	LSP0_DMA_CAN0_RX_REQ = 17,
	LSP0_DMA_CAN1_TX_REQ = 18, 	LSP0_DMA_CAN1_RX_REQ = 19,
	LSP1_DMA_CAN_TX_REQ = 20, 	LSP1_DMA_CAN_RX_REQ = 21,
	CHAN_SLAVE_MAX_REQ,
};

#endif /* _AXI_DMA_PLATFORM_H */

// SPDX-License-Identifier:  GPL-2.0
// (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)

/*
 * Synopsys DesignWare AXI DMA Controller driver.
 *
 * Author: Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/types.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/reset.h>

#include "bst-axi-dmac.h"
#include "../dmaengine.h"
#include "../virt-dma.h"

/*
 * The set of bus widths supported by the DMA controller. BST AXI DMAC supports
 * master data bus width up to 512 bits (for both AXI master interfaces), but
 * it depends on IP block configurarion.
 */
#define AXI_DMA_BUSWIDTHS		  \
	(DMA_SLAVE_BUSWIDTH_1_BYTE	| \
	DMA_SLAVE_BUSWIDTH_2_BYTES	| \
	DMA_SLAVE_BUSWIDTH_4_BYTES	| \
	DMA_SLAVE_BUSWIDTH_8_BYTES	| \
	DMA_SLAVE_BUSWIDTH_16_BYTES	| \
	DMA_SLAVE_BUSWIDTH_32_BYTES	| \
	DMA_SLAVE_BUSWIDTH_64_BYTES)

static inline void
axi_dma_iowrite32(struct axi_dma_chip *chip, u32 reg, u32 val)
{
	iowrite32(val, chip->regs + reg);
}

static inline u32 axi_dma_ioread32(struct axi_dma_chip *chip, u32 reg)
{
	return ioread32(chip->regs + reg);
}

static inline void
axi_chan_iowrite32(struct axi_dma_chan *chan, u32 reg, u32 val)
{
	iowrite32(val, chan->chan_regs + reg);
}

static inline u32 axi_chan_ioread32(struct axi_dma_chan *chan, u32 reg)
{
	return ioread32(chan->chan_regs + reg);
}

static inline void
axi_chan_iowrite64(struct axi_dma_chan *chan, u32 reg, u64 val)
{
	/*
	 * We split one 64 bit write for two 32 bit write as some HW doesn't
	 * support 64 bit access.
	 */
	iowrite32(lower_32_bits(val), chan->chan_regs + reg);
	iowrite32(upper_32_bits(val), chan->chan_regs + reg + 4);
}

static inline void axi_dma_disable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val &= ~DMAC_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_enable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val |= DMAC_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_disable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val &= ~INT_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_enable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val |= INT_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_cmn_irq_disable(struct axi_dma_chip *chip)
{
	u32 val;
	
	val = axi_dma_ioread32(chip, DMAC_COMMON_INTSIGNAL_ENA);
	val &= ~INT_CMN_MASK;
	axi_dma_iowrite32(chip, DMAC_COMMON_INTSIGNAL_ENA, val);
}

static inline void axi_dma_cmn_irq_enable(struct axi_dma_chip *chip)
{
	u32 val;
	
	val = axi_dma_ioread32(chip, DMAC_COMMON_INTSIGNAL_ENA);
	val |= INT_CMN_MASK;
	axi_dma_iowrite32(chip, DMAC_COMMON_INTSIGNAL_ENA, val);
}

static inline void axi_chan_irq_disable(struct axi_dma_chan *chan, u32 irq_mask)
{
	u32 val;

	if (likely(irq_mask == BSTAXIDMAC_IRQ_ALL)) {
		axi_chan_iowrite32(chan, CH_INTSTATUS_ENA, BSTAXIDMAC_IRQ_NONE);
	} else {
		val = axi_chan_ioread32(chan, CH_INTSTATUS_ENA);
		val &= ~irq_mask;
		axi_chan_iowrite32(chan, CH_INTSTATUS_ENA, val);
	}
}

static inline void axi_chan_irq_set(struct axi_dma_chan *chan, u32 irq_mask)
{
	axi_chan_iowrite32(chan, CH_INTSTATUS_ENA, irq_mask);
}

static inline void axi_chan_irq_sig_set(struct axi_dma_chan *chan, u32 irq_mask)
{
	axi_chan_iowrite32(chan, CH_INTSIGNAL_ENA, irq_mask);
}

static inline void axi_chan_irq_clear(struct axi_dma_chan *chan, u32 irq_mask)
{
	axi_chan_iowrite32(chan, CH_INTCLEAR, irq_mask);
}

static inline void axi_cmn_irq_clear(struct axi_dma_chip	*chip, u32 irq_mask)
{
	axi_dma_iowrite32(chip, DMAC_COMMON_INTCLEAR, irq_mask);
}

static inline u32 axi_chan_irq_read(struct axi_dma_chan *chan)
{
	return axi_chan_ioread32(chan, CH_INTSTATUS);
}

static inline u32 axi_cmn_irq_read(struct axi_dma_chip		*chip)
{
	return axi_dma_ioread32(chip, DMAC_COMMON_INTSTATUS);
}

static inline void axi_chan_disable(struct axi_dma_chan *chan)
{
	u32 val;

	val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
	val &= ~(BIT(chan->id) << DMAC_CHAN_EN_SHIFT);
	val |=   BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);
}

static inline void axi_chan_enable(struct axi_dma_chan *chan)
{
	u32 val;

	val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
	val |= BIT(chan->id) << DMAC_CHAN_EN_SHIFT |
	       BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);
}

static inline bool axi_chan_is_hw_enable(struct axi_dma_chan *chan)
{
	u32 val;

	val = axi_dma_ioread32(chan->chip, DMAC_CHEN);

	return !!(val & (BIT(chan->id) << DMAC_CHAN_EN_SHIFT));
}

static void axi_dma_hw_init(struct axi_dma_chip *chip)
{
	u32 i;

	for (i = 0; i < chip->bst->hdata->nr_channels; i++) {
		axi_chan_irq_disable(&chip->bst->chan[i], BSTAXIDMAC_IRQ_ALL);
		axi_chan_disable(&chip->bst->chan[i]);
	}
}

static u32 axi_chan_get_xfer_width(struct axi_dma_chan *chan, dma_addr_t src,
				   dma_addr_t dst, size_t len)
{
	u32 max_width = chan->chip->bst->hdata->m_data_width;

	return __ffs(src | dst | len | BIT(max_width));
}

static inline const char *axi_chan_name(struct axi_dma_chan *chan)
{
	return dma_chan_name(&chan->vc.chan);
}

static struct axi_dma_desc *axi_desc_get(struct axi_dma_chan *chan)
{
	struct bst_axi_dma *bst = chan->chip->bst;
	struct axi_dma_desc *desc;
	dma_addr_t phys;

	desc = dma_pool_zalloc(bst->desc_pool, GFP_NOWAIT, &phys);
	if (unlikely(!desc)) {
		dev_err(chan2dev(chan), "%s: not enough descriptors available\n",
			axi_chan_name(chan));
		return NULL;
	}

	atomic_inc(&chan->descs_allocated);
	INIT_LIST_HEAD(&desc->xfer_list);
	desc->vd.tx.phys = phys;
	desc->chan = chan;

	return desc;
}

static void axi_desc_put(struct axi_dma_desc *desc)
{
	struct axi_dma_chan *chan = desc->chan;
	struct bst_axi_dma *bst = chan->chip->bst;
	struct axi_dma_desc *child, *_next;
	unsigned int descs_put = 0;

	if (chan->dma_sconfig.slave_id == 17 ||
		chan->dma_sconfig.slave_id == 19 ||
		chan->dma_sconfig.slave_id == 21) {
		list_del(&desc->xfer_list);
	} else {
		list_for_each_entry_safe(child, _next, &desc->xfer_list, xfer_list) {
			list_del(&child->xfer_list);
			dma_pool_free(bst->desc_pool, child, child->vd.tx.phys);
			descs_put++;
		}
	}

	dma_pool_free(bst->desc_pool, desc, desc->vd.tx.phys);
	descs_put++;

	atomic_sub(descs_put, &chan->descs_allocated);
	dev_vdbg(chan2dev(chan), "%s: %d descs put, %d still allocated\n",
		axi_chan_name(chan), descs_put,
		atomic_read(&chan->descs_allocated));
}

static void vchan_desc_put(struct virt_dma_desc *vdesc)
{
	axi_desc_put(vd_to_axi_desc(vdesc));
}

static enum dma_status
dma_chan_tx_status(struct dma_chan *dchan, dma_cookie_t cookie,
		  struct dma_tx_state *txstate)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	enum dma_status ret;

	ret = dma_cookie_status(dchan, cookie, txstate);

	if (chan->is_paused && ret == DMA_IN_PROGRESS)
		ret = DMA_PAUSED;

	return ret;
}

static void write_desc_llp(struct axi_dma_desc *desc, dma_addr_t adr)
{
	desc->lli.llp = cpu_to_le64(adr);
}

static void write_chan_llp(struct axi_dma_chan *chan, dma_addr_t adr)
{
	axi_chan_iowrite64(chan, CH_LLP, adr);
}

/* Called in chan locked context */
static void axi_chan_block_xfer_start(struct axi_dma_chan *chan,
				      struct axi_dma_desc *first)
{
	u32 priority = chan->chip->bst->hdata->priority[chan->id];
	u32 reg, irq_mask;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_err(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));

		return;
	}

	axi_dma_irq_enable(chan->chip);
	axi_dma_enable(chan->chip);

	reg = (BSTAXIDMAC_MBLK_TYPE_LL << CH_CFG_L_DST_MULTBLK_TYPE_POS |
	       BSTAXIDMAC_MBLK_TYPE_LL << CH_CFG_L_SRC_MULTBLK_TYPE_POS);

	switch (chan->dma_sconfig.direction) {
	case DMA_MEM_TO_DEV:
		reg |= (chan->dma_sconfig.slave_id & 0xff) << CH_CFG_L_DST_PERIPHERAL_POS;
		break;
	case DMA_DEV_TO_MEM:
		reg |= (chan->dma_sconfig.slave_id & 0xff) << CH_CFG_L_SRC_PERIPHERAL_POS;
		break;
	default:
		break;
	}
	axi_chan_iowrite32(chan, CH_CFG_L, reg);

	//canfd config
	if (chan->dma_sconfig.slave_id == 17 ||
		chan->dma_sconfig.slave_id == 19 ||
		chan->dma_sconfig.slave_id == 21) {

		reg = (BSTAXIDMAC_TT_FC_PER_TO_MEM_SRC << CH_CFG_H_TT_FC_POS |
			priority << CH_CFG_H_PRIORITY_POS |
			BSTAXIDMAC_HS_SEL_SW << CH_CFG_H_HS_SEL_DST_POS |
			BSTAXIDMAC_HS_SEL_HW << CH_CFG_H_HS_SEL_SRC_POS) |
			0x0 << CH_CFG_H_DST_OSR_LMT_POS |
			0x0 << CH_CFG_H_SRC_OSR_LMT_POS;

		irq_mask = BSTAXIDMAC_IRQ_BLOCK_TRF | BSTAXIDMAC_IRQ_ALL_ERR;
	} else {
		reg = (BSTAXIDMAC_TT_FC_MEM_TO_MEM_DMAC << CH_CFG_H_TT_FC_POS |
				priority << CH_CFG_H_PRIORITY_POS |
				BSTAXIDMAC_HS_SEL_HW << CH_CFG_H_HS_SEL_DST_POS |
				BSTAXIDMAC_HS_SEL_HW << CH_CFG_H_HS_SEL_SRC_POS);

		irq_mask = BSTAXIDMAC_IRQ_DMA_TRF | BSTAXIDMAC_IRQ_ALL_ERR;
	}
	axi_chan_iowrite32(chan, CH_CFG_H, reg);

	write_chan_llp(chan, first->vd.tx.phys | lms);

	axi_chan_irq_sig_set(chan, irq_mask);

	/* Generate 'suspend' status but don't generate interrupt */
	irq_mask |= BSTAXIDMAC_IRQ_SUSPENDED;
	axi_chan_irq_set(chan, irq_mask);

	axi_chan_enable(chan);
}

static void axi_chan_start_first_queued(struct axi_dma_chan *chan)
{
	struct axi_dma_desc *desc;
	struct virt_dma_desc *vd;

	vd = vchan_next_desc(&chan->vc);
	if (!vd)
		return;

	desc = vd_to_axi_desc(vd);
	dev_vdbg(chan2dev(chan), "%s: started %u\n", axi_chan_name(chan),
		vd->tx.cookie);
	axi_chan_block_xfer_start(chan, desc);
}

static void dma_chan_issue_pending(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc))
		axi_chan_start_first_queued(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static int dma_chan_alloc_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan)) {
		dev_err(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));
		return -EBUSY;
	}

	dev_vdbg(dchan2dev(dchan), "%s: allocating\n", axi_chan_name(chan));

	pm_runtime_get(chan->chip->dev);

	return 0;
}

static void dma_chan_free_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan))
		dev_err(dchan2dev(dchan), "%s is non-idle!\n",
			axi_chan_name(chan));

	axi_chan_disable(chan);
	axi_chan_irq_disable(chan, BSTAXIDMAC_IRQ_ALL);

	vchan_free_chan_resources(&chan->vc);

	dev_vdbg(dchan2dev(dchan),
		 "%s: free resources, descriptor still allocated: %u\n",
		 axi_chan_name(chan), atomic_read(&chan->descs_allocated));

	pm_runtime_put(chan->chip->dev);
}

/*
 * If BST_axi_dmac sees CHx_CTL.ShadowReg_Or_LLI_Last bit of the fetched LLI
 * as 1, it understands that the current block is the final block in the
 * transfer and completes the DMA transfer operation at the end of current
 * block transfer.
 */
static void set_desc_last(struct axi_dma_desc *desc)
{
	u32 val;

	val = le32_to_cpu(desc->lli.ctl_hi);
	val |= CH_CTL_H_LLI_LAST;
	desc->lli.ctl_hi = cpu_to_le32(val);
}

static void write_desc_sar(struct axi_dma_desc *desc, dma_addr_t adr)
{
	desc->lli.sar = cpu_to_le64(adr);
}

static void write_desc_dar(struct axi_dma_desc *desc, dma_addr_t adr)
{
	desc->lli.dar = cpu_to_le64(adr);
}

static void set_desc_src_master(struct axi_dma_desc *desc)
{
	u32 val;

	/* Select AXI0 for source master */
	val = le32_to_cpu(desc->lli.ctl_lo);
	val &= ~CH_CTL_L_SRC_MAST;
	desc->lli.ctl_lo = cpu_to_le32(val);
}

static void set_desc_dest_master(struct axi_dma_desc *desc)
{
	u32 val;

	/* Select AXI1 for source master if available */
	val = le32_to_cpu(desc->lli.ctl_lo);
	if (desc->chan->chip->bst->hdata->nr_masters > 1)
		val |= CH_CTL_L_DST_MAST;
	else
		val &= ~CH_CTL_L_DST_MAST;

	desc->lli.ctl_lo = cpu_to_le32(val);
}

static struct dma_async_tx_descriptor *
dma_chan_prep_dma_memcpy(struct dma_chan *dchan, dma_addr_t dst_adr,
			 dma_addr_t src_adr, size_t len, unsigned long flags)
{
	struct axi_dma_desc *first = NULL, *desc = NULL, *prev = NULL;
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	size_t block_ts, max_block_ts, xfer_len;
	u32 xfer_width, reg;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	dev_dbg(chan2dev(chan), "%s: memcpy: src: %pad dst: %pad length: %zd flags: %#lx",
		axi_chan_name(chan), &src_adr, &dst_adr, len, flags);

	max_block_ts = chan->chip->bst->hdata->block_size[chan->id];

	chan->dma_sconfig.direction = DMA_MEM_TO_MEM;

	while (len) {
		xfer_len = len;

		/*
		 * Take care for the alignment.
		 * Actually source and destination widths can be different, but
		 * make them same to be simpler.
		 */
		xfer_width = axi_chan_get_xfer_width(chan, src_adr, dst_adr, xfer_len);

		/*
		 * block_ts indicates the total number of data of width
		 * to be transferred in a DMA block transfer.
		 * BLOCK_TS register should be set to block_ts - 1
		 */
		block_ts = xfer_len >> xfer_width;
		if (block_ts > max_block_ts) {
			block_ts = max_block_ts;
			xfer_len = max_block_ts << xfer_width;
		}

		desc = axi_desc_get(chan);
		if (unlikely(!desc))
			goto err_desc_get;

		write_desc_sar(desc, src_adr);
		write_desc_dar(desc, dst_adr);
		desc->lli.block_ts_lo = cpu_to_le32(block_ts - 1);

		reg = CH_CTL_H_LLI_VALID;
		if (chan->chip->bst->hdata->restrict_axi_burst_len) {
			u32 burst_len = chan->chip->bst->hdata->axi_rw_burst_len;

			reg |= (CH_CTL_H_ARLEN_EN |
				burst_len << CH_CTL_H_ARLEN_POS |
				CH_CTL_H_AWLEN_EN |
				burst_len << CH_CTL_H_AWLEN_POS);
		}
		desc->lli.ctl_hi = cpu_to_le32(reg);

		reg = (BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_DST_MSIZE_POS |
		       BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_SRC_MSIZE_POS |
		       xfer_width << CH_CTL_L_DST_WIDTH_POS |
		       xfer_width << CH_CTL_L_SRC_WIDTH_POS |
		       BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
		       BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
		desc->lli.ctl_lo = cpu_to_le32(reg);

		set_desc_src_master(desc);
		set_desc_dest_master(desc);

		/* Manage transfer list (xfer_list) */
		if (!first) {
			first = desc;
		} else {
			list_add_tail(&desc->xfer_list, &first->xfer_list);
			write_desc_llp(prev, desc->vd.tx.phys | lms);
		}
		prev = desc;

		/* update the length and addresses for the next loop cycle */
		len -= xfer_len;
		dst_adr += xfer_len;
		src_adr += xfer_len;
	}

	/* Total len of src/dest sg == 0, so no descriptor were allocated */
	if (unlikely(!first))
		return NULL;

	/* Set end-of-link to the last link descriptor of list */
	set_desc_last(desc);

	return vchan_tx_prep(&chan->vc, &first->vd, flags);

err_desc_get:
	if (first)
		axi_desc_put(first);
	return NULL;
}

static void dma_chan_caps(struct dma_chan *dchan, struct dma_slave_caps *caps)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	caps->max_burst = chan->chip->bst->hdata->axi_rw_burst_len;
}

static int dma_chan_config(struct dma_chan *dchan, struct dma_slave_config *sconfig)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	memcpy(&chan->dma_sconfig, sconfig, sizeof (*sconfig));

	chan->dma_sconfig.src_maxburst =
		clamp(chan->dma_sconfig.src_maxburst, 0U,
					chan->chip->bst->hdata->axi_rw_burst_len);
	chan->dma_sconfig.dst_maxburst =
		clamp(chan->dma_sconfig.dst_maxburst, 0U,
					chan->chip->bst->hdata->axi_rw_burst_len);

	return 0;
}

static struct dma_async_tx_descriptor *
dma_chan_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct axi_dma_desc *first = NULL, *desc = NULL, *prev = NULL;
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	size_t block_ts, max_block_ts, xfer_len, per_ts;
	u32 xfer_width, reg, len;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */
	dma_addr_t reg_addr, mem_addr;
	unsigned int i;
	struct scatterlist	*sg;

	dev_dbg(chan2dev(chan), "%s: slave memcpy flags: %#lx", axi_chan_name(chan), flags);

	if (unlikely(!is_slave_direction(direction) || !sg_len))
		return NULL;

	max_block_ts = chan->chip->bst->hdata->block_size[chan->id];
	chan->dma_sconfig.direction = direction;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		reg_addr = chan->dma_sconfig.dst_addr;
		per_ts = chan->dma_sconfig.dst_addr_width;
		max_block_ts = clamp(per_ts, 1, max_block_ts);
		for_each_sg(sgl, sg, sg_len, i) {
			mem_addr = sg_dma_address(sg);
			len = sg_dma_len(sg);

			while (len) {
				xfer_len = len;

				/*
				* Take care for the alignment.
				* Actually source and destination widths can be different, but
				* make them same to be simpler.
				*/
				xfer_width = axi_chan_get_xfer_width(chan, mem_addr, reg_addr, xfer_len);

				/*
				* block_ts indicates the total number of data of width
				* to be transferred in a DMA block transfer.
				* BLOCK_TS register should be set to block_ts - 1
				*/
				block_ts = xfer_len >> xfer_width;
				if (block_ts > max_block_ts) {
					block_ts = max_block_ts;
					xfer_len = max_block_ts << xfer_width;
				}

				desc = axi_desc_get(chan);
				if (unlikely(!desc))
					goto err_desc_get;

				write_desc_sar(desc, mem_addr);
				write_desc_dar(desc, reg_addr);
				desc->lli.block_ts_lo = cpu_to_le32(block_ts - 1);

				reg = CH_CTL_H_LLI_VALID;
				if (chan->chip->bst->hdata->restrict_axi_burst_len) {
					u32 burst_len = chan->chip->bst->hdata->axi_rw_burst_len;

					reg |= (CH_CTL_H_ARLEN_EN |
						burst_len << CH_CTL_H_ARLEN_POS |
						CH_CTL_H_AWLEN_EN |
						burst_len << CH_CTL_H_AWLEN_POS);
				}
				desc->lli.ctl_hi = cpu_to_le32(reg);

				reg = (BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_DST_MSIZE_POS |
					BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_SRC_MSIZE_POS |
					xfer_width << CH_CTL_L_DST_WIDTH_POS |
					xfer_width << CH_CTL_L_SRC_WIDTH_POS |
					BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
					BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
				desc->lli.ctl_lo = cpu_to_le32(reg);

				set_desc_src_master(desc);
				set_desc_dest_master(desc);

				/* Manage transfer list (xfer_list) */
				if (!first) {
					first = desc;
				} else {
					list_add_tail(&desc->xfer_list, &first->xfer_list);
					write_desc_llp(prev, desc->vd.tx.phys | lms);
				}
				prev = desc;

				/* update the length and addresses for the next loop cycle */
				len -= xfer_len;

				mem_addr += xfer_len;
			}
		}
		break;
	case DMA_DEV_TO_MEM:
		reg_addr = chan->dma_sconfig.src_addr;
		per_ts = chan->dma_sconfig.src_addr_width;
		max_block_ts = clamp(per_ts, 1, max_block_ts);

		if (flags & (1 << 31)) {

			//canfd config
			for_each_sg(sgl, sg, sg_len, i) {
				mem_addr = sg_dma_address(sg);
				len = sg_dma_len(sg);

				while (len) {
					desc = axi_desc_get(chan);
					if (unlikely(!desc))
						goto err_desc_get;

					write_desc_sar(desc, reg_addr);
					write_desc_dar(desc, mem_addr);
					desc->lli.block_ts_lo = cpu_to_le32(block_ts - 1);

						reg = CH_CTL_H_LLI_VALID;

					reg |= (CH_CTL_H_ARLEN_EN |
						0x1 << CH_CTL_H_ARLEN_POS |
						CH_CTL_H_AWLEN_EN |
						0x0 << CH_CTL_H_AWLEN_POS |
						CH_CTL_H_IOC_BlkTf_EN |
						0x0 << CH_CTL_H_LLI_LAST);
					desc->lli.ctl_hi = cpu_to_le32(reg);

					reg = (BSTAXIDMAC_BURST_TRANS_LEN_1 << CH_CTL_L_DST_MSIZE_POS |
						BSTAXIDMAC_BURST_TRANS_LEN_1 << CH_CTL_L_SRC_MSIZE_POS |
						2 << CH_CTL_L_DST_WIDTH_POS |
						2 << CH_CTL_L_SRC_WIDTH_POS |
						BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
						BSTAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_SRC_INC_POS);
					desc->lli.ctl_lo = cpu_to_le32(reg);

					set_desc_src_master(desc);
					set_desc_dest_master(desc);

					/* Manage transfer list (xfer_list) */
					if (!first) {
						first = desc;
					} else {
						list_add_tail(&desc->xfer_list, &first->xfer_list);
						write_desc_llp(prev, desc->vd.tx.phys | lms);
					}
					prev = desc;

					/* update the length and addresses for the next loop cycle */
					len -= 18;

					mem_addr += (18 * 4);
					vchan_tx_prep(&chan->vc, &desc->vd, flags);
				}
			}
		} else {
			for_each_sg(sgl, sg, sg_len, i) {
				mem_addr = sg_dma_address(sg);
				len = sg_dma_len(sg);

				while (len) {
					xfer_len = len;

					/*
					* Take care for the alignment.
					* Actually source and destination widths can be different, but
					* make them same to be simpler.
					*/
					xfer_width = axi_chan_get_xfer_width(chan, reg_addr, mem_addr, xfer_len);

					/*
					* block_ts indicates the total number of data of width
					* to be transferred in a DMA block transfer.
					* BLOCK_TS register should be set to block_ts - 1
					*/
					block_ts = xfer_len >> xfer_width;
					if (block_ts > max_block_ts) {
						block_ts = max_block_ts;
						xfer_len = max_block_ts << xfer_width;
					}
		
					desc = axi_desc_get(chan);
					if (unlikely(!desc))
						goto err_desc_get;

					write_desc_sar(desc, reg_addr);
					write_desc_dar(desc, mem_addr);
					desc->lli.block_ts_lo = cpu_to_le32(block_ts - 1);

					reg = CH_CTL_H_LLI_VALID;
					if (chan->chip->bst->hdata->restrict_axi_burst_len) {
						u32 burst_len = chan->chip->bst->hdata->axi_rw_burst_len;

						reg |= (CH_CTL_H_ARLEN_EN |
							burst_len << CH_CTL_H_ARLEN_POS |
							CH_CTL_H_AWLEN_EN |
							burst_len << CH_CTL_H_AWLEN_POS);
					}
					desc->lli.ctl_hi = cpu_to_le32(reg);

					reg = (BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_DST_MSIZE_POS |
						BSTAXIDMAC_BURST_TRANS_LEN_16 << CH_CTL_L_SRC_MSIZE_POS |
						xfer_width << CH_CTL_L_DST_WIDTH_POS |
						xfer_width << CH_CTL_L_SRC_WIDTH_POS |
						BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
						BSTAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
					desc->lli.ctl_lo = cpu_to_le32(reg);

					set_desc_src_master(desc);
					set_desc_dest_master(desc);

					/* Manage transfer list (xfer_list) */
					if (!first) {
						first = desc;
					} else {
						list_add_tail(&desc->xfer_list, &first->xfer_list);
						write_desc_llp(prev, desc->vd.tx.phys | lms);
					}
					prev = desc;
		
					/* update the length and addresses for the next loop cycle */
					len -= xfer_len;

					mem_addr += xfer_len;
				}
			}
		}
		break;
	default:
		return NULL;
	}

	/* Total len of src/dest sg == 0, so no descriptor were allocated */
	if (unlikely(!first))
		return NULL;

	/* Set end-of-link to the last link descriptor of list */
	if (flags & (1 << 31)) {
		//canfd config
		// flags &= ~(1 << 31);
		write_desc_llp(prev, first->vd.tx.phys | lms);
		return &first->vd.tx;
	} else {
		set_desc_last(desc);
		return vchan_tx_prep(&chan->vc, &first->vd, flags);
	}

err_desc_get:
	if (first)
		axi_desc_put(first);
	return NULL;
}

static void axi_chan_dump_lli(struct axi_dma_chan *chan,
			      struct axi_dma_desc *desc)
{
	dev_err(dchan2dev(&chan->vc.chan),
		"SAR: 0x%llx DAR: 0x%llx LLP: 0x%llx BTS 0x%x CTL: 0x%x:%08x",
		le64_to_cpu(desc->lli.sar),
		le64_to_cpu(desc->lli.dar),
		le64_to_cpu(desc->lli.llp),
		le32_to_cpu(desc->lli.block_ts_lo),
		le32_to_cpu(desc->lli.ctl_hi),
		le32_to_cpu(desc->lli.ctl_lo));
}

static void axi_chan_list_dump_lli(struct axi_dma_chan *chan,
				   struct axi_dma_desc *desc_head)
{
	struct axi_dma_desc *desc;

	axi_chan_dump_lli(chan, desc_head);
	list_for_each_entry(desc, &desc_head->xfer_list, xfer_list)
		axi_chan_dump_lli(chan, desc);
}

static noinline void axi_chan_handle_err(struct axi_dma_chan *chan, u32 status)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	axi_chan_disable(chan);

	/* The bad descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	/* Remove the completed descriptor from issued list */
	list_del(&vd->node);

	/* WARN about bad descriptor */
	dev_err(chan2dev(chan),
		"Bad descriptor submitted for %s, cookie: %d, irq: 0x%08x\n",
		axi_chan_name(chan), vd->tx.cookie, status);
	axi_chan_list_dump_lli(chan, vd_to_axi_desc(vd));

	vchan_cookie_complete(vd);

	/* Try to restart the controller */
	axi_chan_start_first_queued(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void axi_chan_block_xfer_complete(struct axi_dma_chan *chan)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_err(chan2dev(chan), "BUG: %s caught BSTAXIDMAC_IRQ_DMA_TRF, but channel not idle!\n",
			axi_chan_name(chan));
		axi_chan_disable(chan);
	}

	/* The completed descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	/* Remove the completed descriptor from issued list before completing */
	list_del(&vd->node);
	vchan_cookie_complete(vd);

	/* Submit queued descriptors after processing the completed ones */
	axi_chan_start_first_queued(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void axi_chan_block_xfer_complete_bst_canfd(struct axi_dma_chan *chan)
{
	struct virt_dma_desc *vd;
	unsigned long flags;
	struct axi_dma_desc *desc, *desc_next;
	u32 reg;

	//canfd config
	spin_lock_irqsave(&chan->vc.lock, flags);

	vd = vchan_next_desc(&chan->vc);
	vd->tx.chan = &chan->vc.chan;

	vchan_cyclic_callback(vd);

	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static irqreturn_t bst_axi_dma_bst_chan_interrupt(int irq, void *dev_id)
{
	struct axi_dma_chan *chan = dev_id;
	struct axi_dma_chip *chip = chan->chip;

	u32 status, i;
	
	/* Disable DMAC inerrupts. We'll enable them after processing chanels */
	axi_dma_irq_disable(chip);

	status = axi_chan_irq_read(chan);
	axi_chan_irq_clear(chan, status);

	dev_vdbg(chip->dev, "%s %u IRQ status: 0x%08x\n",
		axi_chan_name(chan), i, status);

	if (status & BSTAXIDMAC_IRQ_ALL_ERR)
		axi_chan_handle_err(chan, status);
	else if (status & BSTAXIDMAC_IRQ_DMA_TRF) 
		axi_chan_block_xfer_complete(chan);
	else if (status & BSTAXIDMAC_IRQ_BLOCK_TRF) 
		axi_chan_block_xfer_complete_bst_canfd(chan);

	/* Re-enable interrupts */
	axi_dma_irq_enable(chip);

	return IRQ_HANDLED;
}

static irqreturn_t bst_axi_dma_bst_cmn_interrupt(int irq, void *dev_id)
{
	struct axi_dma_chip *chip = dev_id;
	u32 status;

	/* Disable DMAC inerrupts. We'll enable them after processing chanels */
	axi_dma_cmn_irq_disable(chip);

	status = axi_cmn_irq_read(chip);
	axi_cmn_irq_clear(chip, status);

	dev_vdbg(chip->dev, "CMN IRQ status: 0x%08x\n",status);

	/* Re-enable interrupts */
	axi_dma_cmn_irq_enable(chip);

	return IRQ_HANDLED;
}

static int dma_chan_terminate_all(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);

	axi_chan_disable(chan);

	vchan_get_all_descriptors(&chan->vc, &head);

	/*
	 * As vchan_dma_desc_free_list can access to desc_allocated list
	 * we need to call it in vc.lock context.
	 */
	vchan_dma_desc_free_list(&chan->vc, &head);

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	dev_vdbg(dchan2dev(dchan), "terminated: %s\n", axi_chan_name(chan));

	return 0;
}

static int dma_chan_pause(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;
	unsigned int timeout = 20; /* timeout iterations */
	u32 val;

	spin_lock_irqsave(&chan->vc.lock, flags);

	val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
	val |= BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT |
	       BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT;
	axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);

	do  {
		if (axi_chan_irq_read(chan) & BSTAXIDMAC_IRQ_SUSPENDED)
			break;

		udelay(2);
	} while (--timeout);

	axi_chan_irq_clear(chan, BSTAXIDMAC_IRQ_SUSPENDED);

	chan->is_paused = true;

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return timeout ? 0 : -EAGAIN;
}

/* Called in chan locked context */
static inline void axi_chan_resume(struct axi_dma_chan *chan)
{
	u32 val;

	val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
	val &= ~(BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT);
	val |=  (BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT);
	axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);

	chan->is_paused = false;
}

static int dma_chan_resume(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (chan->is_paused)
		axi_chan_resume(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return 0;
}

static int axi_dma_suspend(struct axi_dma_chip *chip)
{
	axi_dma_irq_disable(chip);
	axi_dma_disable(chip);

	clk_disable_unprepare(chip->core_clk);
	clk_disable_unprepare(chip->cfgr_clk);

	return 0;
}

static int axi_dma_resume(struct axi_dma_chip *chip)
{
	int ret;

	ret = clk_prepare_enable(chip->cfgr_clk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(chip->core_clk);
	if (ret < 0)
		return ret;

	axi_dma_enable(chip);
	axi_dma_irq_enable(chip);

	return 0;
}

static int __maybe_unused axi_dma_runtime_suspend(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_suspend(chip);
}

static int __maybe_unused axi_dma_runtime_resume(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_resume(chip);
}

static int parse_device_properties(struct axi_dma_chip *chip)
{
	struct device *dev = chip->dev;
	u32 tmp, carr[DMAC_MAX_CHANNELS];
	int ret;

	if(device_property_read_bool(dev, "support-slave"))
		chip->support_slave = 1;

	ret = device_property_read_u32(dev, "dma-channels", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_CHANNELS)
		return -EINVAL;

	chip->bst->hdata->nr_channels = tmp;

	ret = device_property_read_u32(dev, "snps,dma-masters", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_MASTERS)
		return -EINVAL;

	chip->bst->hdata->nr_masters = tmp;

	ret = device_property_read_u32(dev, "snps,data-width", &tmp);
	if (ret)
		return ret;
	if (tmp > BSTAXIDMAC_TRANS_WIDTH_MAX)
		return -EINVAL;

	chip->bst->hdata->m_data_width = tmp;

	ret = device_property_read_u32_array(dev, "snps,block-size", carr,
					     chip->bst->hdata->nr_channels);
	if (ret)
		return ret;
	for (tmp = 0; tmp < chip->bst->hdata->nr_channels; tmp++) {
		if (carr[tmp] == 0 || carr[tmp] > DMAC_MAX_BLK_SIZE)
			return -EINVAL;

		chip->bst->hdata->block_size[tmp] = carr[tmp];
	}

	ret = device_property_read_u32_array(dev, "snps,priority", carr,
					     chip->bst->hdata->nr_channels);
	if (ret)
		return ret;
	/* Priority value must be programmed within [0:nr_channels-1] range */
	for (tmp = 0; tmp < chip->bst->hdata->nr_channels; tmp++) {
		if (carr[tmp] >= chip->bst->hdata->nr_channels)
			return -EINVAL;

		chip->bst->hdata->priority[tmp] = carr[tmp];
	}

	/* axi-max-burst-len is optional property */
	ret = device_property_read_u32(dev, "snps,axi-max-burst-len", &tmp);
	if (!ret) {
		if (tmp > BSTAXIDMAC_ARWLEN_MAX + 1)
			return -EINVAL;
		if (tmp < BSTAXIDMAC_ARWLEN_MIN + 1)
			return -EINVAL;

		chip->bst->hdata->restrict_axi_burst_len = true;
		chip->bst->hdata->axi_rw_burst_len = tmp - 1;
	}

	return 0;
}

static int bst_chan_irq_init(struct axi_dma_chip *chip)
{
	int i;
	int ret;

	for(i = 0; i < chip->bst->hdata->nr_channels; i++) {
		ret = devm_request_irq(chip->dev,chip->bst->hdata->chan_irq[i], bst_axi_dma_bst_chan_interrupt,
				       IRQF_SHARED, KBUILD_MODNAME, &chip->bst->chan[i]);
		if (ret)
		   return ret;
   }

	return 0;
}

static int bst_probe(struct platform_device *pdev)
{
	struct axi_dma_chip *chip;
	struct resource *mem;
	struct bst_axi_dma *bst;
	struct bst_axi_dma_hcfg *hdata;
	u32 i;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	bst = devm_kzalloc(&pdev->dev, sizeof(*bst), GFP_KERNEL);
	if (!bst)
		return -ENOMEM;

	hdata = devm_kzalloc(&pdev->dev, sizeof(*hdata), GFP_KERNEL);
	if (!hdata)
		return -ENOMEM;

	chip->bst = bst;
	chip->dev = &pdev->dev;
	chip->bst->hdata = hdata;

	hdata->cmn_irq = platform_get_irq(pdev, 0);
	if (hdata->cmn_irq < 0)
		return hdata->cmn_irq;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->regs = devm_ioremap_resource(chip->dev, mem);
	if (IS_ERR(chip->regs))
		return PTR_ERR(chip->regs);

	chip->core_clk = devm_clk_get(chip->dev, "core-clk");
	if (IS_ERR(chip->core_clk))
		return PTR_ERR(chip->core_clk);

	chip->cfgr_clk = devm_clk_get(chip->dev, "cfgr-clk");
	if (IS_ERR(chip->cfgr_clk))
		return PTR_ERR(chip->cfgr_clk);

	ret = parse_device_properties(chip);
	if (ret)
		return ret;

	bst->reset = devm_reset_control_get_optional_exclusive(chip->dev, NULL);

	if (IS_ERR(bst->reset))
		return PTR_ERR(bst->reset);

	ret = reset_control_deassert(bst->reset);
	if (ret)
		return -EINVAL;

	bst->chan = devm_kcalloc(chip->dev, hdata->nr_channels,
				sizeof(*bst->chan), GFP_KERNEL);
	if (!bst->chan)
		return -ENOMEM;

	ret = platform_irq_count(pdev);
	if (ret < 0)
		return ret;

	if (ret != 1) {
		if (ret != (chip->bst->hdata->nr_channels + 1))
			return -EINVAL;

		for (i = 0; i < chip->bst->hdata->nr_channels; i++) {
			ret = platform_get_irq(pdev, i + 1);
			if (ret < 0)
				return ret;

			chip->bst->hdata->chan_irq[i] = ret;
		}
	}

	ret =bst_chan_irq_init(chip);
	if (ret)
		return ret;

	ret = devm_request_irq(chip->dev, chip->bst->hdata->cmn_irq, bst_axi_dma_bst_cmn_interrupt,
			       IRQF_SHARED, KBUILD_MODNAME, chip);
	if (ret)
		return ret;

	if (!dma_set_mask(chip->dev, DMA_BIT_MASK(64)))
		dev_info(chip->dev, "Black Sesame Team AXI DMA Controller Support 64BIT\n");
	else {
		dma_set_mask(chip->dev, DMA_BIT_MASK(32));
		dev_info(chip->dev, "Black Sesame Team AXI DMA Controller Support 32BIT\n");
	}

	/* Lli address must be aligned to a 64-byte boundary */
	bst->desc_pool = dmam_pool_create(KBUILD_MODNAME, chip->dev,
					 sizeof(struct axi_dma_desc), 64, 0);
	if (!bst->desc_pool) {
		dev_err(chip->dev, "No memory for descriptors dma pool\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&bst->dma.channels);
	for (i = 0; i < hdata->nr_channels; i++) {
		struct axi_dma_chan *chan = &bst->chan[i];

		chan->chip = chip;
		chan->id = i;
		chan->chan_regs = chip->regs + COMMON_REG_LEN + i * CHAN_REG_LEN;
		atomic_set(&chan->descs_allocated, 0);

		chan->vc.desc_free = vchan_desc_put;
		vchan_init(&chan->vc, &bst->dma);
	}

	/* Set capabilities */
	dma_cap_set(DMA_MEMCPY, bst->dma.cap_mask);

	/* DMA capabilities */
	bst->dma.chancnt = hdata->nr_channels;
	bst->dma.src_addr_widths = AXI_DMA_BUSWIDTHS;
	bst->dma.dst_addr_widths = AXI_DMA_BUSWIDTHS;
	bst->dma.directions = BIT(DMA_MEM_TO_MEM);
	bst->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;

	bst->dma.dev = chip->dev;
	bst->dma.device_tx_status = dma_chan_tx_status;
	bst->dma.device_issue_pending = dma_chan_issue_pending;
	bst->dma.device_terminate_all = dma_chan_terminate_all;
	bst->dma.device_pause = dma_chan_pause;
	bst->dma.device_resume = dma_chan_resume;

	bst->dma.device_alloc_chan_resources = dma_chan_alloc_chan_resources;
	bst->dma.device_free_chan_resources = dma_chan_free_chan_resources;

	bst->dma.device_prep_dma_memcpy = dma_chan_prep_dma_memcpy;

	bst->dma.device_caps = dma_chan_caps;
	if (chip->support_slave) {
		dma_cap_set(DMA_SLAVE, bst->dma.cap_mask);
		bst->dma.device_config = dma_chan_config;
		bst->dma.device_prep_slave_sg = dma_chan_prep_slave_sg;
	}

	platform_set_drvdata(pdev, chip);

	pm_runtime_enable(chip->dev);

	/*
	 * We can't just call pm_runtime_get here instead of
	 * pm_runtime_get_noresume + axi_dma_resume because we need
	 * driver to work also without Runtime PM.
	 */
	pm_runtime_get_noresume(chip->dev);
	ret = axi_dma_resume(chip);
	if (ret < 0)
		goto err_pm_disable;

	axi_dma_hw_init(chip);

	pm_runtime_put(chip->dev);

	ret = dma_async_device_register(&bst->dma);
	if (ret)
		goto err_pm_disable;

	dev_info(chip->dev, "Black Sesame Team AXI DMA Controller, %d channels\n",
		 bst->hdata->nr_channels);

	return 0;

err_pm_disable:
	pm_runtime_disable(chip->dev);

	return ret;
}

int dma_pr_debug_test(void)
{
	pr_debug("/****** dma: this is  pr_debug in module dma. ******/\n");
	return 0;
}
EXPORT_SYMBOL(dma_pr_debug_test);


static int bst_remove(struct platform_device *pdev)
{
	struct axi_dma_chip *chip = platform_get_drvdata(pdev);
	struct bst_axi_dma *bst = chip->bst;
	struct axi_dma_chan *chan, *_chan;
	u32 i;

	/* Enable clk before accessing to registers */
	clk_prepare_enable(chip->cfgr_clk);
	clk_prepare_enable(chip->core_clk);
	axi_dma_irq_disable(chip);
	for (i = 0; i < bst->hdata->nr_channels; i++) {
		axi_chan_disable(&chip->bst->chan[i]);
		axi_chan_irq_disable(&chip->bst->chan[i], BSTAXIDMAC_IRQ_ALL);
		devm_free_irq(chip->dev, chip->bst->hdata->chan_irq[i], chip);
	}
	axi_dma_disable(chip);

	pm_runtime_disable(chip->dev);
	axi_dma_suspend(chip);

	devm_free_irq(chip->dev, chip->bst->hdata->cmn_irq, chip);

	reset_control_assert(bst->reset);

	list_for_each_entry_safe(chan, _chan, &bst->dma.channels,
			vc.chan.device_node) {
		list_del(&chan->vc.chan.device_node);
		tasklet_kill(&chan->vc.task);
	}

	dma_async_device_unregister(&bst->dma);

	return 0;
}

static const struct dev_pm_ops bst_axi_dma_pm_ops = {
	SET_RUNTIME_PM_OPS(axi_dma_runtime_suspend, axi_dma_runtime_resume, NULL)
};

static const struct of_device_id bst_dma_of_id_table[] = {
	{ .compatible = "bst,dw-axi-gdma" },
	{}
};
MODULE_DEVICE_TABLE(of, bst_dma_of_id_table);

static struct platform_driver bst_driver = {
	.probe		= bst_probe,
	.remove		= bst_remove,
	.driver = {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_match_ptr(bst_dma_of_id_table),
		.pm = &bst_axi_dma_pm_ops,
	},
};
module_platform_driver(bst_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Synopsys DesignWare AXI DMA Controller platform driver");
MODULE_AUTHOR("Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>");

// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare PCIe DMA driver
 *
 * Copyright (C) 2022 Black Sesame Tec Co., Ltd.
 *		https://www.bst.ai
 *
 * Author: GaoQingpeng <qingpeng.gao@bst.ai>
 */
#include <linux/delay.h>
#include <linux/picp.h>
#include "../../pci.h"
#include "pcie-bst.h"
#include "pcie-bst-phy.h"

#define PCIE_DMA_USEIRQ 0

spinlock_t dma_rlock, dma_wlock;
static unsigned long dma_read_flag;
static unsigned long dma_write_flag;

static void write_pcie_dma_reg(struct dw_pcie *pci, u32 offset, u32 val)
{
	writel(val, pci->dma_base + offset);
}

static u32 read_pcie_dma_reg(struct dw_pcie *pci, u32 offset)
{
	return readl(pci->dma_base + offset);
}

bool wait_ep_dma_rw_stop(void)
{
	int i, j, cnt;

	for (i = 0; i < 4; i++) {
		cnt = 0;
		while ((cnt < 10) && (test_bit(i, &dma_read_flag))) {
			msleep(20);
			cnt++;
		}
	}
	for (j = 0; j < 4; j++) {
		cnt = 0;
		while ((cnt < 10) && (test_bit(j, &dma_write_flag))) {
			msleep(20);
			cnt++;
		}
	}

	return true;
}
EXPORT_SYMBOL(wait_ep_dma_rw_stop);

/*
 * dmc_dma_read: dm controller dma read
 * return 0 success
 */
int dmc_dma_read(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst)
{
	u32 regval;
	u32 dma_trans_ok = 0;
	int i = 0;
	int ret = 0;

	ret = pcie_is_link(pci);
	if (!ret || ch >= 4 || !size || !src || !dst) {
		pr_info("%s error: link:%#x ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
			__func__, pci->phy->ltssm, ch, size, src, dst);
		return ret ? PCIE_ERR_PARAERR : PCIE_ERR_LINKERR;
	}

	//pr_info( "DMA Read transfer begin, ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
		// ch, size, src, dst);
	spin_lock(&dma_rlock);
	write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
	/* 3. cfg DMA channel control register */
	write_pcie_dma_reg(pci, 0x300 + ch*0x200, 0x8); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_pcie_dma_reg(pci, 0x308 + ch*0x200, size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_pcie_dma_reg(pci, 0x30C + ch*0x200, src&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x310 + ch*0x200, src>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_pcie_dma_reg(pci, 0x314 + ch*0x200, dst&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x318 + ch*0x200, dst>>32); //DMA_DAR_High_OFF_RDCH_%0d
	regval = read_pcie_dma_reg(pci, 0x38);
	//pr_info( "DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",regval);
	/* 7. cfg Doorbell for start DMA transfer */
	regval = read_pcie_dma_reg(pci, 0x30); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffffff8;
	regval = regval | (ch&0x7);
	write_pcie_dma_reg(pci, 0x30, regval); //DMA_DAR_High_OFF_RDCH_%0d

	regval = read_pcie_dma_reg(pci, 0xa0);
	//pr_info( "DMA_READ_INT_STATUS_OFF: 0x%x\n",regval);
	while (regval != (0x1 << ch)) {
		regval = read_pcie_dma_reg(pci, 0xa0);
		//pr_info( "DMA_READ_INT_STATUS_OFF: 0x%x\n",regval);
		if (regval == (0x10000 << ch)) {
			pr_info("****dm_pcie_dma_read about 0xa0:%x\n", regval);
			write_pcie_dma_reg(pci, 0xac, regval);
		}
		i++;
		if (i > 100000) {
			pr_info("rc_pcie_fail when dm_pcie_dma_read 0xa0:%x\n", regval);
			write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
			spin_unlock(&dma_rlock);
			return PCIE_ERR_DMAERR;
		}
	}
	write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
	spin_unlock(&dma_rlock);

	return dma_trans_ok;
}
EXPORT_SYMBOL(dmc_dma_read);

/*
 * dmc_dma_write: dm controller dma write
 * return 0 success
 */
int dmc_dma_write(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst)
{
	u32 regval;
	u32 dma_trans_ok = 0;
	int i = 0;
	int ret = 0;

	ret = pcie_is_link(pci);
	if (!ret || ch >= 4 || !size || !src || !dst) {
		pr_info("%s error: link:%#x ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
			__func__, pci->phy->ltssm, ch, size, src, dst);
		return ret ? PCIE_ERR_PARAERR : PCIE_ERR_LINKERR;
	}

	//pr_info( "DMA Write transfer begin, ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
		//ch, size, src, dst);
	spin_lock(&dma_wlock);
	write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
	/* 3. cfg DMA channel control register */
	write_pcie_dma_reg(pci, 0x200 + ch*0x200, 0x8); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_pcie_dma_reg(pci, 0x208 + ch*0x200, size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_pcie_dma_reg(pci, 0x20C + ch*0x200, src&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x210 + ch*0x200, src>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_pcie_dma_reg(pci, 0x214 + ch*0x200, dst&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x218 + ch*0x200, dst>>32); //DMA_DAR_High_OFF_RDCH_%0d
	regval = read_pcie_dma_reg(pci, 0x18);
	//pr_info( "DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",regval);
	/* 7. cfg Doorbell for start DMA transfer */
	regval = read_pcie_dma_reg(pci, 0x10); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffffff8;
	regval = regval | (ch&0x7);
	write_pcie_dma_reg(pci, 0x10, regval); //DMA_DAR_High_OFF_RDCH_%0d

	regval = read_pcie_dma_reg(pci, 0x4c);
	//pr_info( "DMA_WRITE_INT_STATUS_OFF: 0x%x\n",regval);
	while (regval != (0x1 << ch)) {
		regval = read_pcie_dma_reg(pci, 0x4c);
		//pr_info( "DMA_WRITE_INT_STATUS_OFF: 0x%x\n",regval);
		if (regval == (0x10000 << ch)) {
			pr_info("****dm_pcie_dma_write about 0xa0:%x\n", regval);
			write_pcie_dma_reg(pci, 0x58, regval);
		}
		i++;
		if (i > 100000) {
			pr_info("rc_pcie_fail when dm_pcie_dma_write\n");
			write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
			spin_unlock(&dma_wlock);
			return PCIE_ERR_DMAERR;
		}
	}
	write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
	spin_unlock(&dma_wlock);

	return dma_trans_ok;
}
EXPORT_SYMBOL(dmc_dma_write);

/*
 * epc_dma_read: ep controller dma read
 * return 0 success
 */
int epc_dma_read(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst)
{
	u32 regval;
	u32 dma_trans_ok = 0;
	int i = 0;
	int ret = 0;

	ret = pcie_is_link(pci);
	if (!ret || ch >= 4 || !size || !src || !dst) {
		pr_info("%s error: link:%#x ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
			__func__, pci->phy->ltssm, ch, size, src, dst);
		return ret ? PCIE_ERR_PARAERR : PCIE_ERR_LINKERR;
	}

	//pr_info( "DMA Read transfer begin, ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n", ch, size, src, dst);
	spin_lock(&dma_rlock);
	write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
	set_bit(ch, &dma_read_flag);
	/* 3. cfg DMA channel control register */
	write_pcie_dma_reg(pci, 0x300 + ch*0x200, 0x8); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_pcie_dma_reg(pci, 0x308 + ch*0x200, size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_pcie_dma_reg(pci, 0x30C + ch*0x200, src&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x310 + ch*0x200, src>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_pcie_dma_reg(pci, 0x314 + ch*0x200, dst&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x318 + ch*0x200, dst>>32); //DMA_DAR_High_OFF_RDCH_%0d
	regval = read_pcie_dma_reg(pci, 0x38);
	//pr_info( "DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",regval);
	/* 7. cfg Doorbell for start DMA transfer */
	//spin_lock(&dma_rlock);
	regval = read_pcie_dma_reg(pci, 0x30); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffffff8;
	regval = regval | (ch&0x7);
	write_pcie_dma_reg(pci, 0x30, regval); //DMA_DAR_High_OFF_RDCH_%0d

#if PCIE_DMA_USEIRQ
	wait_event_interruptible(read_wq, (read_dma_reg == (0x1 << ch)));
	if (read_dma_reg & (0x10000 << ch))
		pr_info("***** DMA about int a0:%x ch:%d\n", read_dma_reg, ch);
	read_dma_reg = read_dma_reg & ~(0x10001 << ch);
#else
	regval = read_pcie_dma_reg(pci, 0xa0);
	while (regval != (0x1 << ch)) {
		regval = read_pcie_dma_reg(pci, 0xa0);
		if (regval == (0x10000 << ch)) {
			pr_info("***** DMA about int a0:%x ch:%d\n", regval, ch);
			write_pcie_dma_reg(pci, 0xac, regval);
		}

		if (i++ > 100000) {
			pr_info("***** DMA timeout ch%d\n", ch);
			write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
			clear_bit(ch, &dma_read_flag);
			spin_unlock(&dma_rlock);
			return PCIE_ERR_DMAERR; //fail
		}
	}
	write_pcie_dma_reg(pci, 0xac, (0x10001 << ch));
	clear_bit(ch, &dma_read_flag);
#endif
	spin_unlock(&dma_rlock);

	return dma_trans_ok;
}
EXPORT_SYMBOL(epc_dma_read);

/*
 * epc_dma_write: ep controller dma write
 * return 0 success
 */
int epc_dma_write(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst)
{
	u32 regval;
	u32 dma_trans_ok = 0;
	int i = 0;
	int ret = 0;

	ret = pcie_is_link(pci);
	if (!ret || ch >= 4 || !size || !src || !dst) {
		pr_info("%s error: link:%#x ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n",
			__func__, pci->phy->ltssm, ch, size, src, dst);
		return ret ? PCIE_ERR_PARAERR : PCIE_ERR_LINKERR;
	}

	//pr_info("DMA Write transfer begin, ch=0x%0x, size=0x%0x, src=0x%0llx, dst=0x%0llx\n", ch, size, src, dst);
	spin_lock(&dma_wlock);
	write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
	set_bit(ch, &dma_write_flag);
	/* 3. cfg DMA channel control register */
	write_pcie_dma_reg(pci, 0x200 + ch*0x200, 0x8); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_pcie_dma_reg(pci, 0x208 + ch*0x200, size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_pcie_dma_reg(pci, 0x20C + ch*0x200, src&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x210 + ch*0x200, src>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_pcie_dma_reg(pci, 0x214 + ch*0x200, dst&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_pcie_dma_reg(pci, 0x218 + ch*0x200, dst>>32); //DMA_DAR_High_OFF_RDCH_%0d
	regval = read_pcie_dma_reg(pci, 0x18);
	//pr_info( "DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",regval);
	/* 7. cfg Doorbell for start DMA transfer */
	//spin_lock(&dma_wlock);
	regval = read_pcie_dma_reg(pci, 0x10); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffffff8;
	regval = regval | (ch&0x7);
	write_pcie_dma_reg(pci, 0x10, regval); //DMA_DAR_High_OFF_RDCH_%0d

#if PCIE_DMA_USEIRQ
	wait_event_interruptible(write_wq, (write_dma_reg == (0x1 << ch)));
	if (write_dma_reg & (0x10000 << ch))
		pr_info("***** DMA about int ac:%x ch:%d\n", write_dma_reg, ch);
	write_dma_reg = write_dma_reg & ~(0x10001 << ch);
#else
	regval = read_pcie_dma_reg(pci, 0x4c);
	while (regval != (0x1 << ch)) {
		regval = read_pcie_dma_reg(pci, 0x4c);
		if (regval == (0x10000 << ch)) {
			pr_info("****** write about 4c:%x ch:%d\n", regval, ch);
			write_pcie_dma_reg(pci, 0x58, regval);
		}

		if (i++ > 100000) {
			pr_info("****** timeout ch%d\n", ch);
			write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
			clear_bit(ch, &dma_write_flag);
			spin_unlock(&dma_wlock);
			return PCIE_ERR_DMAERR; //fail
		}
	}
	write_pcie_dma_reg(pci, 0x58, (0x10001 << ch));
	clear_bit(ch, &dma_write_flag);
#endif
	spin_unlock(&dma_wlock);

	return dma_trans_ok;
}
EXPORT_SYMBOL(epc_dma_write);

void pcie_dma_init(struct dw_pcie *pci)
{
	u32 regval;

	/* clear write Abort and Done Interrupt mask*/
	write_pcie_dma_reg(pci, 0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_pcie_dma_reg(pci, 0xa8, 0);

	spin_lock_init(&dma_rlock);
	spin_lock_init(&dma_wlock);

	/* 1. cfg dma read enable */
	regval = read_pcie_dma_reg(pci, 0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	regval = regval & 0xfffffffe;
	regval = regval | 0x1;
	write_pcie_dma_reg(pci, 0x2c, regval);
	/* 2. cfg dma read interupt enable */
	regval = read_pcie_dma_reg(pci, 0xa8); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffefffe;
	regval = regval | 0x00010001;
	write_pcie_dma_reg(pci, 0xa8, regval);

	/* 3. cfg dma write enable */
	regval = read_pcie_dma_reg(pci, 0xc); //dma_write_ENGINE_EN_OFF bit0->1
	regval = regval & 0xfffffffe;
	regval = regval | 0x1;
	write_pcie_dma_reg(pci, 0xc, regval);
	/* 4. cfg dma write interupt enable */
	regval = read_pcie_dma_reg(pci, 0x54); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	regval = regval & 0xfffefffe;
	regval = regval | 0x00010001;
	write_pcie_dma_reg(pci, 0x54, regval);
}
EXPORT_SYMBOL(pcie_dma_init);

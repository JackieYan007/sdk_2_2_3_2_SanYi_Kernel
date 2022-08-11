/*
 * Copyright (C) 2007-2015  STMicroelectronics Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Author: Alexandre Torgue <alexandre.torgue@st.com>
 */

#include <linux/io.h>
#include <linux/delay.h>
#include "common.h"
#include "dwmac4_dma.h"
#include "dwmac4.h"

int dwmac4_dma_reset(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	int limit;
#if defined(CONFIG_ARCH_BSTA1000B)
	void __iomem *top_crm = NULL;
	u32 reg_value ;
#endif
	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);
#if defined(CONFIG_ARCH_BSTA1000B)
	top_crm = ioremap(0x33000000, 0x100);
	if(top_crm) {
        reg_value = readl(top_crm + 0x80);
		if (reg_value & (1 << 7)) {
			mdelay(100);
			writel(0x5, ioaddr+0xce0);
			mdelay(1);
			writel(0x11f, ioaddr+0xcc0);
			mdelay(1);
			writel(0x1111, ioaddr+0xcc8);
			mdelay(10);
		}
		iounmap(top_crm);
	}
#endif
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET))
			break;
		mdelay(10);
	}

	if (limit < 0)
		return -EBUSY;

	return 0;
}

void dwmac4_set_rx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	writel(tail_ptr, ioaddr + DMA_CHAN_RX_END_ADDR(chan));
}

void dwmac4_set_tx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	writel(tail_ptr, ioaddr + DMA_CHAN_TX_END_ADDR(chan));
}

void dwmac4_dma_start_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value |= DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);
}

void dwmac4_dma_stop_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value &= ~DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);
}

void dwmac4_dma_start_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value |= DMA_CONTROL_SR;

	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
	//pr_emerg("[%s]%d.",__func__,__LINE__);
}

void dwmac4_dma_stop_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value &= ~DMA_CONTROL_SR;
	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
	//pr_emerg("[%s]%d.",__func__,__LINE__);
}

void dwmac4_set_tx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	writel(len, ioaddr + DMA_CHAN_TX_RING_LEN(chan));
}

void dwmac4_set_rx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	writel(len, ioaddr + DMA_CHAN_RX_RING_LEN(chan));
}

void dwmac4_enable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	writel(DMA_CHAN_INTR_DEFAULT_MASK, ioaddr +
	       DMA_CHAN_INTR_ENA(chan));
}

void dwmac410_enable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	writel(DMA_CHAN_INTR_DEFAULT_MASK_4_10,
	       ioaddr + DMA_CHAN_INTR_ENA(chan));
}

void dwmac4_disable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	writel(0, ioaddr + DMA_CHAN_INTR_ENA(chan));
}

void dwmac4_enable_dma_irq_bits(void __iomem *ioaddr, u32 chan,u32 bits)
{
	u32 value;
	value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
	value |= bits;
	writel(value,ioaddr + DMA_CHAN_INTR_ENA(chan));

    dma_wmb();
    writel(value,ioaddr + DMA_CHAN_INTR_ENA(chan));
    dma_wmb();
         
    value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
    if (!(value & bits)) {
        value |= bits;
	    writel(value,ioaddr + DMA_CHAN_INTR_ENA(chan));
        dma_wmb();
    }
}

void dwmac4_disable_dma_irq_bits(void __iomem *ioaddr, u32 chan,u32 bits)
{
	u32 value;
	
	value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
	value &= ~bits;
	writel(value, ioaddr + DMA_CHAN_INTR_ENA(chan));
}

int dwmac4_dma_interrupt(void __iomem *ioaddr,
			 struct bstgmac_extra_stats *x, u32 chan,u32 flag)
{
	int ret = 0;

	u32 intr_status = readl(ioaddr + DMA_CHAN_STATUS(chan));

	/* ABNORMAL interrupts */
	if (unlikely(intr_status & DMA_CHAN_STATUS_AIS)) {
		if (unlikely(intr_status & DMA_CHAN_STATUS_RBU))
			x->rx_buf_unav_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_RPS))
			x->rx_process_stopped_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_RWT))
			x->rx_watchdog_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_ETI))
			x->tx_early_irq++;
		if (unlikely(intr_status & DMA_CHAN_STATUS_TPS)) {
			x->tx_process_stopped_irq++;
			ret = tx_hard_error;
		}
		if (unlikely(intr_status & DMA_CHAN_STATUS_FBE)) {
			x->fatal_bus_error_irq++;
			ret = tx_hard_error;
		}
	}
	if(!flag){
		/* TX/RX NORMAL interrupts */
		if (likely(intr_status & DMA_CHAN_STATUS_NIS)) {
			x->normal_irq_n++;
			if (likely(intr_status & DMA_CHAN_STATUS_RI)) {
				u32 value;

				value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
				/* to schedule NAPI on real RIE event. */
				if (likely(value & DMA_CHAN_INTR_ENA_RIE)) {
					x->rx_normal_irq_n++;
					ret |= handle_rx;
				}
			}
			if (likely(intr_status & DMA_CHAN_STATUS_TI)) {
				x->tx_normal_irq_n++;
				ret |= handle_tx;
			}
			if (unlikely(intr_status & DMA_CHAN_STATUS_ERI))
				x->rx_early_irq++;
		}
		
		writel((intr_status & 0x3fffc7),
		       ioaddr + DMA_CHAN_STATUS(chan));
	}else{
	/* Clear the interrupt by writing a logic 1 to the chanX interrupt
	 * status [21-0] expect reserved bits [5-3]. 
	 *Bits[15,6,0] cleard by sbd_perch_intrs.
	 */
		writel((intr_status & 0x3f7f86),
		       ioaddr + DMA_CHAN_STATUS(chan));
	}

	return ret;
}

int bstgmac_dma_ri_interrupt(void __iomem *ioaddr,
			  struct bstgmac_extra_stats *x, u32 chan)
{
	int ret = 0;
	//u32 value;

	u32 intr_status = readl(ioaddr + DMA_CHAN_STATUS(chan));
	
	//if(intr_status){
	//	pr_emerg("[%s]%d intr_status:%x",__func__,__LINE__,intr_status);
	//}
	/* RX Complete interrupts */
	if (likely(intr_status & DMA_CHAN_STATUS_RI)) {
		x->rx_normal_irq_n++;
		ret |= handle_rx;
		/* Clear the interrupt by writing a logic 1 to the chanX interrupt
		* DMA_CHAN_STATUS_RI 
		*/
		writel(DMA_CHAN_STATUS_RI|DMA_CHAN_STATUS_NIS,ioaddr + DMA_CHAN_STATUS(chan));
	}
	return ret;
}

int bstgmac_dma_ti_interrupt(void __iomem *ioaddr,
			struct bstgmac_extra_stats *x, u32 chan)
{
	int ret = 0;

	u32 intr_status = readl(ioaddr + DMA_CHAN_STATUS(chan));
	/* TX Complete interrupts */
	if (likely(intr_status & DMA_CHAN_STATUS_TI)) {
		x->tx_normal_irq_n++;
		ret |= handle_tx;
	/* Clear the interrupt by writing a logic 1 to the chanX interrupt
	* DMA_CHAN_STATUS_TI 
	*/
		writel(DMA_CHAN_STATUS_TI|DMA_CHAN_STATUS_NIS,ioaddr + DMA_CHAN_STATUS(chan));
	}

	return ret;
}

void bstgmac_dwmac4_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
				unsigned int high, unsigned int low)
{
	unsigned long data;

	data = (addr[5] << 8) | addr[4];
	/* For MAC Addr registers se have to set the Address Enable (AE)
	 * bit that has no effect on the High Reg 0 where the bit 31 (MO)
	 * is RO.
	 */
	data |= (BSTGMAC_CHAN0 << GMAC_HI_DCS_SHIFT);
	writel(data | GMAC_HI_REG_AE, ioaddr + high);
	data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	writel(data, ioaddr + low);
}

/* Enable disable MAC RX/TX */
void bstgmac_dwmac4_set_mac(void __iomem *ioaddr, bool enable)
{
	u32 value = readl(ioaddr + GMAC_CONFIG);

	if (enable)
		value |= GMAC_CONFIG_RE | GMAC_CONFIG_TE;
	else
		value &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
	//pr_emerg("[%s]%d enable:%d.",__func__,__LINE__,enable);
	writel(value, ioaddr + GMAC_CONFIG);
}

void bstgmac_dwmac4_get_mac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int high, unsigned int low)
{
	unsigned int hi_addr, lo_addr;

	/* Read the MAC address from the hardware */
	hi_addr = readl(ioaddr + high);
	lo_addr = readl(ioaddr + low);

	/* Extract the MAC address from the high and low words */
	addr[0] = lo_addr & 0xff;
	addr[1] = (lo_addr >> 8) & 0xff;
	addr[2] = (lo_addr >> 16) & 0xff;
	addr[3] = (lo_addr >> 24) & 0xff;
	addr[4] = hi_addr & 0xff;
	addr[5] = (hi_addr >> 8) & 0xff;
}

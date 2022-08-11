/*
* QSPI core controller driver for BST QSPI
* Based on a patch from spi-dw.c
* Copyright (c) 2009, Intel Corporation.
*
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*
* ChangeLog:
* Jan 2020: v1: create qspi driver for the first time
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*/


#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/sizes.h>

#include "qspi-bst.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
void __iomem *qspiregtopcfg;

static struct dw_qspi_chip def_chip_info={
	.poll_mode = 0, /* 1 for controller polling mode */
	.type = SSI_MOTO_SPI,	/* SPI/SSP/MicroWire */
	.cs_control = NULL,
};

/* Slave spi_dev related */
struct chip_data {
	u8 tmode;		/* TR/TO/RO/EEPROM */
	u8 type;		/* SPI/SSP/MicroWire */

	u8 poll_mode;		/* 1 means use poll mode */

	u16 clk_div;		/* baud rate divider */
	u32 speed_hz;		/* baud rate */
	void (*cs_control)(u32 command);
};

#ifdef CONFIG_DEBUG_FS
#define QSPI_REGS_BUFSIZE	1024
static ssize_t dw_qspi_show_regs(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct dw_qspi *dws = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(QSPI_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"%s registers:\n", dev_name(&dws->master->dev));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"CTRL0: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_CTRL0));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"CTRL1: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_CTRL1));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"SSIENR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SSIENR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"SER: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SER));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"BAUDR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_BAUDR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"TXFTLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_TXFLTR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"RXFTLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_RXFLTR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"TXFLR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_TXFLR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"RXFLR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_RXFLR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"SR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"IMR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_IMR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"ISR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_ISR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"DMACR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMACR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"DMATDLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMATDLR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"DMARDLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMARDLR));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"SPI_CTRLR0: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SPI_CTRLR0));
	len += snprintf(buf + len, QSPI_REGS_BUFSIZE - len,
			"=================================\n");

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations dw_qspi_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= dw_qspi_show_regs,
	.llseek		= default_llseek,
};

static int dw_qspi_debugfs_init(struct dw_qspi *dws)
{
	char name[32];

	snprintf(name, 32, "dw_qspi%d", dws->master->bus_num);
	dws->debugfs = debugfs_create_dir(name, NULL);
	if (!dws->debugfs)
		return -ENOMEM;

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
		dws->debugfs, (void *)dws, &dw_qspi_regs_ops);
	return 0;
}

static ssize_t dw_qspi_dump_regs(struct dw_qspi* dws)
{
	int ret;
	pr_debug("CTRL0: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_CTRL0));
	pr_debug("CTRL1: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_CTRL1));
	pr_debug("SSIENR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SSIENR));
	pr_debug("SER: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SER));
	pr_debug("BAUDR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_BAUDR));
	pr_debug("TXFTLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_TXFLTR));
	pr_debug("RXFTLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_RXFLTR));
	pr_debug("TXFLR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_TXFLR));
	pr_debug("RXFLR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_RXFLR));
	pr_debug("SR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SR));
	pr_debug("IMR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_IMR));
	pr_debug("ISR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_ISR));
	pr_debug("DMACR: \t\t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMACR));
	pr_debug("DMATDLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMATDLR));
	pr_debug("DMARDLR: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_DMARDLR));
	pr_debug("SPI_CTRLR0: \t0x%08x\n", dw_qspi_readl(dws, DW_QSPI_SPI_CTRLR0));

	return ret;
}


static void dw_qspi_debugfs_remove(struct dw_qspi *dws)
{
	debugfs_remove_recursive(dws->debugfs);
}

#else
static inline int dw_qspi_debugfs_init(struct dw_qspi *dws)
{
	return 0;
}

static inline void dw_qspi_debugfs_remove(struct dw_qspi *dws)
{
}
#endif /* CONFIG_DEBUG_FS */

static void qspi_set_cs(struct spi_device *spi, bool enable)
{	
    struct dw_qspi *dws = spi_controller_get_devdata(spi->controller);
	struct chip_data *chip = spi_get_ctldata(spi);
	if (gpio_is_valid(spi->cs_gpio)) {		
		if(spi->mode & SPI_CS_HIGH){
			enable = !enable;
		}
		gpio_set_value(spi->cs_gpio, !enable);

		/* Chip select logic is inverted from spi_set_cs() */
		if(chip && chip->cs_control)
			chip->cs_control(!enable);
		if(spi->controller->flags & SPI_MASTER_GPIO_SS && !enable){
			dw_qspi_write_io_reg(dws, DW_QSPI_SER, BIT(spi->chip_select));
		}
	}else{
		pr_emerg("[%s]%d chip_select:%d  enable%d.",__func__,__LINE__,spi->chip_select,enable);
		if(enable)
			dw_qspi_write_io_reg(dws, DW_QSPI_SER, BIT(spi->chip_select));
	}
}
EXPORT_SYMBOL_GPL(qspi_set_cs);


static inline u32 enh_is_tx_done(struct dw_qspi *dws)
{
	u32 tx_left;
	tx_left = (dws->tx_end - dws->tx) / dws->n_bytes;
	if(!tx_left){
		return !dw_qspi_readl(dws, DW_QSPI_TXFLR);
	}
	return 0;
}


/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct dw_qspi *dws)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (dws->tx_end - dws->tx) / dws->n_bytes;
	tx_room = dws->fifo_len - dw_qspi_readl(dws, DW_QSPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (dws->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap =  ((dws->rx_end - dws->rx) - (dws->tx_end - dws->tx))
			/ dws->n_bytes;

	return min3(tx_left, tx_room, (u32) (dws->fifo_len - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct dw_qspi *dws)
{
	u32 rx_left = (dws->rx_end - dws->rx) / dws->n_bytes;
	return min_t(u32, rx_left, dw_qspi_readl(dws, DW_QSPI_RXFLR));
}

/* Return the max entries we can fill into tx fifo */
static inline u32 enh_tx_max(struct dw_qspi *dws)
{
	u32 tx_left, tx_room;

	tx_left = (dws->tx_end - dws->tx) / dws->n_bytes;
	tx_room = dws->fifo_len - dw_qspi_readl(dws, DW_QSPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (dws->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	return min_t(u32, tx_left, tx_room);
}

/* Return the max entries we should read out of rx fifo */
static inline u32 enh_rx_max(struct dw_qspi *dws)
{
	return rx_max(dws);
}

static void bst_qspi_writer(struct dw_qspi *dws)
{
	u32 max = tx_max(dws);
	u16 txw = 0xffff;
	if(dws->txmode == QSPI_TMOD_RO){
		if(dws->tx_end - dws->tx){
			dw_qspi_write_io_reg(dws, DW_QSPI_DR, txw);
			dws->tx_end = dws->tx;
		}
		return;
	}
	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (dws->tx_end - dws->len) {
			if (dws->n_bytes == 1)
				txw = *(u8 *)(dws->tx);
			else
				txw = *(u16 *)(dws->tx);
		}
		dw_qspi_write_io_reg(dws, DW_QSPI_DR, txw);
		dws->tx += dws->n_bytes;
	}
}

static void bst_qspi_reader(struct dw_qspi *dws)
{
	u32 max = rx_max(dws);
	u16 rxw;
	
	while (max--) {
		rxw = dw_qspi_read_io_reg(dws, DW_QSPI_DR);
		/* Care rx only if the transfer's original "rx" is not null */
		if (dws->rx_end - dws->len) {
			if (dws->n_bytes == 1)
				*(u8 *)(dws->rx) = rxw;
			else
				*(u16 *)(dws->rx) = rxw;
		}
		dws->rx += dws->n_bytes;
	}
}
#if 0
static void bst_qspi_enh_send_head(struct dw_qspi *dws)
{
	u32 max = dws->head_len/dws->n_bytes;
	u32 txw = 0;
	void* bufp = dws->head_buf;
	if(max <= 0){
		return;
	}
	//pr_emerg("[%s]%d.send head len:%d",__func__,__LINE__,max);
	while (max--) {
		if (dws->n_bytes == 1)
			txw = *(u8 *)(bufp);
		else if (dws->n_bytes == 2)
			txw = *(u16 *)(bufp);
		else
			txw = *(u32 *)(bufp);
		bufp += dws->n_bytes;
		dw_qspi_write_io_reg(dws, DW_QSPI_DR, txw);
	}
	dws->head_len = 0;
	//pr_emerg("[%s]%d.send head Done",__func__,__LINE__);
}
#else
static void bst_qspi_enh_send_head(struct dw_qspi *dws)
{
	int i;
	if(!dws->start){
		return;
	}
	//pr_emerg("[%s]%d.send head len:%d",__func__,__LINE__,max);
	//pr_emerg("[%s]%d send cmd:%08x addr:%08x.",__func__,__LINE__,dws->cmd,dws->addr);
	dw_qspi_write_io_reg(dws, DW_QSPI_DR, dws->cmd);
	if(dws->has_addr){
		//pr_emerg("[%s]%d send cmd:%08x addr:%08x.",__func__,__LINE__,dws->cmd,dws->addr);
		if(dws->data_width != 1){
			dw_qspi_write_io_reg(dws, DW_QSPI_DR, dws->addr);
		}else{
			for(i = 0;i<dws->addr_len;i++){
				dw_qspi_write_io_reg(dws,DW_QSPI_DR,(dws->addr >> (8*(dws->addr_len-i-1)))&0xff);
			}
		}
	}
	dws->start = 0;
	//pr_emerg("[%s]%d.send head Done",__func__,__LINE__);
}
#endif

static void bst_qspi_enh_writer(struct dw_qspi *dws)
{
	u32 max = enh_tx_max(dws);
	u32 txw = 0xFFFFFFFF;

	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if(dws->tx_dummy_len){
			//pr_emerg("tx_dummy_len:%d",dws->tx_dummy_len);
			dw_qspi_write_io_reg(dws, DW_QSPI_DR, 0xffffffff);
			dws->tx_dummy_len--;
			continue;
		}
		if (dws->tx_end - dws->len) {
			if (dws->n_bytes == 1)
				txw = *(u8 *)(dws->tx);
			else if (dws->n_bytes == 2)
				txw = *(u16 *)(dws->tx);
			else
				txw = *(u32 *)(dws->tx);

		}
		dw_qspi_write_io_reg(dws, DW_QSPI_DR, txw);
		dws->tx += dws->n_bytes;
	}
	
}

static void bst_qspi_enh_reader(struct dw_qspi *dws)
{
	u32 max = enh_rx_max(dws);
	u32 rxw;
	
	while (max--) {
		rxw = dw_qspi_read_io_reg(dws, DW_QSPI_DR);
		if(dws->rx_ignor_len){
			//pr_emerg("rx_ignor_len:%d",dws->rx_ignor_len);
			dws->rx_ignor_len--;
			continue;
		}
		/* Care rx only if the transfer's original "rx" is not null */
		if (dws->rx_end - dws->len) {
			if (dws->n_bytes == 1)
				*(u8 *)(dws->rx) = rxw;
			else if (dws->n_bytes == 2)
				*(u16 *)(dws->rx) = rxw;
			else
				*(u32 *)(dws->rx) = rxw;
		}
		dws->rx += dws->n_bytes;
	}
}

static void int_error_stop(struct dw_qspi *dws, const char *msg)
{
	qspi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s\n", msg);
	dws->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(dws->master);
}
static void int_error_stop_enh(struct dw_qspi *dws, const char *msg)
{
	qspi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s\n", msg);
	dws->master->cur_msg->status = -EIO;
	dws->done = 1;
	//qspi_enable_chip(dws, 0);
	//spi_finalize_current_message(dws->master);
}

static irqreturn_t interrupt_transfer(struct dw_qspi *dws)
{
	u16 irq_status = dw_qspi_readl(dws, DW_QSPI_ISR) & 0x3f;
	/* Error handling */
	if (irq_status & (QSPI_INT_TXOI | QSPI_INT_RXOI | QSPI_INT_RXUI)) {
		dw_qspi_readl(dws, DW_QSPI_ICR);
		int_error_stop(dws, "interrupt_transfer: fifo overrun/underrun");
		return IRQ_HANDLED;
	}
	
	bst_qspi_reader(dws);
	
	if (dws->rx_end <= dws->rx) {
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		spi_finalize_current_transfer(dws->master);
		pr_emerg("[%s]%d.Done.",__func__,__LINE__);
		return IRQ_HANDLED;
	}
	if (dws->rx_end <= dws->rx) {
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		spi_finalize_current_transfer(dws->master);
		pr_emerg("[%s]%d.Done.",__func__,__LINE__);
		return IRQ_HANDLED;
	}
	if(QSPI_TMOD_TO == dws->txmode && dws->tx_end <= dws->tx && !dw_qspi_readl(dws, DW_QSPI_RXFLR)){
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		spi_finalize_current_transfer(dws->master);
		pr_emerg("[%s]%d.Done.",__func__,__LINE__);
		return IRQ_HANDLED;
	}
	pr_emerg("[%s]%d:[S:%x].RXFLR:%d TXFLR:%d rx_end:%lx  rx:%lx tx_end:%lx tx:%lx.",__func__,__LINE__,irq_status,
		dw_qspi_readl(dws, DW_QSPI_RXFLR),dw_qspi_readl(dws, DW_QSPI_TXFLR),dws->rx_end, dws->rx,dws->tx_end, dws->tx);
	if (irq_status & QSPI_INT_TXEI) {
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		bst_qspi_writer(dws);
		/* Enable TX irq always, it will be disabled when RX finished */
		qspi_umask_intr(dws, QSPI_INT_TXEI);
	}

	return IRQ_HANDLED;
}

static irqreturn_t interrupt_transfer_enhanced(struct dw_qspi *dws)
{
	u16 irq_status = dw_qspi_readl(dws, DW_QSPI_ISR) & 0x3f;

	/* Error handling */
	if (irq_status & (QSPI_INT_TXOI | QSPI_INT_RXUI)) {
		dw_qspi_readl(dws, DW_QSPI_ICR);
		int_error_stop_enh(dws, "interrupt_transfer: fifo overrun/underrun");
		return IRQ_HANDLED;
	}
	bst_qspi_enh_reader(dws);
	bst_qspi_enh_send_head(dws);
	//pr_emerg("[%s]%d:[S:%x].RXFLR:%d TXFLR:%d rx_end:%lx  rx:%lx tx_end:%lx tx:%lx.",__func__,__LINE__,irq_status,
	//	dw_qspi_readl(dws, DW_QSPI_RXFLR),dw_qspi_readl(dws, DW_QSPI_TXFLR),dws->rx_end, dws->rx,dws->tx_end, dws->tx);	
	if(dws->rx_end == dws->rx &&  dws->tx_end == dws->tx && !dw_qspi_readl(dws, DW_QSPI_TXFLR)){
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		dws->cur_msg->status = 0;
		dws->done = 1;
		return IRQ_HANDLED;
	}
	if (dws->tx_end != dws->tx && irq_status & QSPI_INT_TXEI) {
		qspi_mask_intr(dws, QSPI_INT_TXEI);
		bst_qspi_enh_writer(dws);
		/* Enable TX irq always, it will be disabled when RX finished */
		qspi_umask_intr(dws, QSPI_INT_TXEI);
	}
	return IRQ_HANDLED;
}

static irqreturn_t dw_qspi_irq(int irq, void *dev_id)
{
	struct spi_controller *master = dev_id;
	struct dw_qspi *dws = spi_controller_get_devdata(master);
	u16 irq_status = dw_qspi_readl(dws, DW_QSPI_ISR) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg) {
		qspi_mask_intr(dws, 0xff);
		//qspi_mask_intr(dws, QSPI_INT_TXEI);
		//pr_emerg("irq_status: %x *",irq_status);
		return IRQ_HANDLED;
	}
	return dws->transfer_handler(dws);
}

/* Must be called inside pump_transfers() */
static int poll_transfer_norm(struct dw_qspi *dws)
{
	do {
		bst_qspi_writer(dws);
		bst_qspi_reader(dws);
		cpu_relax();
	} while (dws->rx_end > dws->rx);

	return 0;
}

/* Must be called inside pump_transfers() */
static int bst_qspi_poll_transfer(struct dw_qspi *dws)
{
	bst_qspi_enh_send_head(dws);
	
	do {
		bst_qspi_enh_writer(dws);
		bst_qspi_enh_reader(dws);
		cpu_relax();
	}while(dws->rx_end != dws->rx ||  dws->tx_end != dws->tx || dw_qspi_readl(dws, DW_QSPI_TXFLR));
	return 0;
}

static void dw_decode_addr(struct dw_qspi *dws,char* buf,int addr_len)
{
	u32 addr = 0;
	int i ;
	for(i = 0 ;i < addr_len;i++){
		addr <<= 8;
		addr |= (buf[i]&0xff);
	}
	dws->addr_len = addr_len;
	dws->addr = addr;
	dws->has_addr = 1;
}

static int dw_qspi_transfer_one(struct spi_controller *master,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	struct dw_qspi *dws = spi_controller_get_devdata(master);
	struct chip_data *chip = spi_get_ctldata(spi);
	u8 imask = 0;
	u16 txlevel = 0;
	int nbits;
	u32 cr0 = 0;
	int ret;
	qspi_enable_chip(dws, 0);
	
	dws->dma_mapped = 0;
	dws->tx = (void *)transfer->tx_buf;
	dws->tx_end = dws->tx + transfer->len;
	dws->rx = transfer->rx_buf;
	dws->rx_end = dws->rx + transfer->len;
	dws->len = transfer->len;
	//pr_emerg("[%s]%d dws->rx:%lx dws->rx_end:%lx.",__func__,__LINE__,dws->rx,dws->rx_end);

	/* Handle per transfer options for bpw and speed */
	if (transfer->speed_hz != dws->current_freq) {
		if (transfer->speed_hz != chip->speed_hz) {
			/* clk_div doesn't support odd number */
			chip->clk_div = (DIV_ROUND_UP(dws->max_freq, transfer->speed_hz) + 1) & 0xfffe;
			chip->speed_hz = transfer->speed_hz;
		}
		dws->current_freq = transfer->speed_hz;
		pr_emerg("******** Set clk:%d clk_div:%d  max_freq:%d.*************",dws->current_freq,chip->clk_div,dws->max_freq);
		qspi_set_clk(dws, chip->clk_div);
	}
	
	dws->n_bytes = 1;
	dws->dma_width = 1;
	
	/* Default SPI mode is SCPOL = 0, SCPH = 0 */
	//pr_emerg("[%s]%d.bits_per_word:%d.",__func__,__LINE__,transfer->bits_per_word);
	cr0 = (transfer->bits_per_word - 1)
		| (chip->type << QSPI_FRF_OFFSET)
		| ((spi->mode&0x3) << QSPI_MODE_OFFSET)
		| (chip->tmode << QSPI_TMOD_OFFSET);

		if(transfer->rx_buf){
			nbits= transfer->rx_nbits;
		}else{
			nbits= transfer->tx_nbits;
		}

	switch(nbits){
		case 1:
			cr0 |= QSPI_SPI_FRF_STD << QSPI_SPI_FRF_OFFSET ;
			break;
		case 2 :
			cr0 |= QSPI_SPI_FRF_DUAL << QSPI_SPI_FRF_OFFSET ;
			break;
		case 4 :
			cr0 |= QSPI_SPI_FRF_QUAD << QSPI_SPI_FRF_OFFSET ;
			break;
		case 8 :
			cr0 |= QSPI_SPI_FRF_OCTAL << QSPI_SPI_FRF_OFFSET ;
			break;
		default:
	//		pr_emerg("[%s]%d error out.",__func__,__LINE__);
			return -1;
			break;	
	}
	/*
	 * Adjust transfer mode if necessary. Requires platform dependent
	 * chipselect mechanism.
	 */
	 if(nbits == 1){
		chip->tmode = QSPI_TMOD_TR;
	 }else if(transfer->rx_buf){
		 chip->tmode = QSPI_TMOD_RO;
		 dws->tx_end = dws->tx+1;
		 dw_qspi_writel(dws, DW_QSPI_CTRL1, dws->len);
	 }else{
		 chip->tmode = QSPI_TMOD_TO;
	 }
	dws->txmode = chip->tmode; 
	cr0 &= ~QSPI_TMOD_MASK;
	cr0 |= (chip->tmode << QSPI_TMOD_OFFSET);

	dw_qspi_writel(dws, DW_QSPI_CTRL0, cr0);

	/* Check if current transfer is a DMA transaction */
	if (master->can_dma && master->can_dma(master, spi, transfer))
		dws->dma_mapped = master->cur_msg_mapped;

	/* For poll mode just disable all interrupts */
	qspi_mask_intr(dws, 0xff);

	/*
	 * Interrupt mode
	 * we only need set the TXEI IRQ, as TX/RX always happen syncronizely
	 */
	if (dws->dma_mapped) {
		ret = dws->dma_ops->dma_setup(dws, transfer);
		if (ret < 0) {
			qspi_enable_chip(dws, 1);
			return ret;
		}
	} else if (!chip->poll_mode) {
		txlevel = min_t(u16, dws->fifo_len / 2, dws->len / dws->n_bytes);
		dw_qspi_writel(dws, DW_QSPI_TXFLTR, txlevel);
		//pr_emerg("[%s]%d Waiting Intrrupt.",__func__,__LINE__);
		
		/* Set the interrupt mask */
		imask |= QSPI_INT_TXEI | QSPI_INT_TXOI |
			 QSPI_INT_RXUI | QSPI_INT_RXOI;
		qspi_umask_intr(dws, imask);
		dws->transfer_handler = interrupt_transfer;
	}
//	pr_emerg("[%s]%d Start.",__func__,__LINE__);
	qspi_enable_chip(dws, 1);

	if (dws->dma_mapped) {
		ret = dws->dma_ops->dma_transfer(dws, transfer);
		if (ret < 0)
			return ret;
	}

	if (chip->poll_mode)
		return poll_transfer_norm(dws);

	return 1;
}

static int dw_qspi_enhanced_config(struct spi_device *spi,struct dw_qspi *dws)
{
	struct chip_data *chip = spi_get_ctldata(spi);
	dw_qspi_enhanced_msg_t  *enhanced_msg;
	dw_qspi_enhanced_msg_en_t *msg_en;
	int trans_type = 0;
	//int has_addr = 1;
	u32 cmdR = 0;
	int inst_width = 0;
	u16 inst_len = 0;
	int tmod_type = 0;
	int addr_width = 0;
	u16 addr_len = 0;
	int data_width = 1;
	int wait_cycl = 0;
	u32 spi_ctrl0 = 0;
	u32 cr0 = 0;


	enhanced_msg = &dws->enhanced_msg;
	if(enhanced_msg->msg_num == 0){
		return -1;
	}
	msg_en = &enhanced_msg->msg_en[0];
	inst_width = msg_en->width_len;
	inst_len =  msg_en->len;
	cmdR = *(u8*)msg_en->buf;
	dws->cmd = cmdR & 0xff;
	dws->has_addr = 0;
	//dws->head_buf[dws->head_len++] = msg_en->buf[0];
	switch(inst_len){
		case 1 :
			spi_ctrl0 |= QSPI_INST_L_L8 << QSPI_INST_L_OFFSET ;
			break;
		case 2 :
			spi_ctrl0 |= QSPI_INST_L_L16 << QSPI_INST_L_OFFSET ;
			break;
		default:
			pr_emerg("[%s]%d error out.",__func__,__LINE__);
			return -1;
	}	
	tmod_type = QSPI_TMOD_TO;
	msg_en++;
	switch(enhanced_msg->msg_num){
		case 4:
			addr_width = msg_en->width_len;
			addr_len =  msg_en->len;
			dw_decode_addr(dws,msg_en->buf,addr_len);
			dws->cmd = cmdR;
			msg_en++;
			wait_cycl = (msg_en->len * 8) / addr_width;
			if(addr_width != msg_en->width_len){
				pr_emerg("(wait_cycl != addr_width).",__func__,__LINE__);
				return -1;
			}
			msg_en++;
			data_width = msg_en->width_len;
			if(msg_en->istx){
				dws->tx = msg_en->buf;
				dws->tx_end = dws->tx + msg_en->len;				
			}else{
				tmod_type = QSPI_TMOD_RO;
				dws->rx = msg_en->buf;
				dws->rx_end = dws->rx + msg_en->len;
			}
			break;
		case 3:
			
			addr_width = msg_en->width_len;
			addr_len = msg_en->len;
			dw_decode_addr(dws,msg_en->buf,addr_len);
			
			msg_en++;
			data_width = msg_en->width_len;
			if(msg_en->istx){
				dws->tx =  msg_en->buf;
				dws->tx_end =  dws->tx +  msg_en->len;				
			}else{
				tmod_type = QSPI_TMOD_RO;
				dws->rx =  msg_en->buf;
				dws->rx_end =  dws->rx +  msg_en->len;
			}
			break;
		case 2:
			dws->has_addr = 0;
			data_width =  msg_en->width_len;
			if(msg_en->istx){
				dws->tx =  msg_en->buf;
				dws->tx_end =  dws->tx + msg_en->len;
			}else{
				tmod_type = QSPI_TMOD_RO;
				dws->rx =  msg_en->buf;
				dws->rx_end =  dws->rx + msg_en->len;
			}
		break;
		default:
			//dws->has_addr = 0;
		//	pr_emerg("No transfer number:%d .",enhanced_msg->msg_num);
			break;
	}
	
	if(inst_width != 1){
		trans_type++;
	}
	if(dws->tx){
		dws->len = dws->tx_end - dws->tx;
		dws->rx_end = dws->rx + dws->len;
	}else{
		dws->len = dws->rx_end - dws->rx;
	}
	
	if(dws->has_addr){
		spi_ctrl0 |= (addr_len << 1) << QSPI_ADDR_L_OFFSET;
		if(addr_width != 1){
			trans_type++;
		}
	}
	spi_ctrl0 |= 1 < QSPI_CLK_STRETCH_EN_OFFSET;
	spi_ctrl0 |= (trans_type & 3) << QSPI_TRANS_TYPE_OFFSET ;
	if(wait_cycl){
		spi_ctrl0 |= (wait_cycl & 0x1f) << QSPI_WAIT_CYCLES_OFFSET ;
	}
	dws->rx_ignor_len = 0;
	dws->tx_dummy_len = 0;
	if(data_width == 1){
		dws->rx_ignor_len = inst_len + addr_len + wait_cycl / (data_width*8);
		dws->tx_dummy_len = wait_cycl / ( data_width * 8);
		dws->tx_end = dws->tx + dws->len;
		tmod_type = QSPI_TMOD_TR;
		spi_ctrl0 = 0;
	}
	dws->data_width = data_width;
	
	dws->tmode = tmod_type;
	
	cr0 |= 7 | (dws->tmode & 0x3) << QSPI_TMOD_OFFSET;
	switch(data_width){
		case 1:
			cr0 |= QSPI_SPI_FRF_STD << QSPI_SPI_FRF_OFFSET ;
			break;
		case 2 :
			cr0 |= QSPI_SPI_FRF_DUAL << QSPI_SPI_FRF_OFFSET ;
			break;
		case 4 :
			cr0 |= QSPI_SPI_FRF_QUAD << QSPI_SPI_FRF_OFFSET ;
			break;
		case 8 :
			cr0 |= QSPI_SPI_FRF_OCTAL << QSPI_SPI_FRF_OFFSET ;
			break;
		default:
			pr_emerg("[%s]%d error out.",__func__,__LINE__);
			return -1;
			break;	
	}

	cr0 |= (spi->mode & 0x3) << QSPI_MODE_OFFSET;
	cr0 |= (chip->type & 0x3) << QSPI_FRF_OFFSET;
	//pr_emerg("[%s]%d:tmod_type:%d,data_width:%d,spi->mode:%x,chip->type:%d,head_len:%d cr0:%08x",
		//	__func__,__LINE__,tmod_type,data_width,spi->mode&3,chip->type,dws->head_len,cr0);
	pr_emerg("[%s]%d:inst_width:%d,tmod_type:%d trans_type:%d, data_width:%d, addr_len:%d, addr_width:%d, wait_cycl:%d,spi_ctrl0:%08x,cr0:%08x,NDF:%d",
			__func__,__LINE__,inst_width,tmod_type,trans_type,data_width,addr_len,addr_width,wait_cycl,spi_ctrl0,cr0, dws->rx ? dws->rx_end - dws->rx:0);
	pr_emerg("[%s]%d:rx:%lx  rx_end:%lx tx:%lx  tx_end:%lx rx_ignor_len:%d ,tx_dummy_len:%d",
			__func__,__LINE__, dws->rx , dws->rx_end ,dws->tx , dws->tx_end,dws->rx_ignor_len ,dws->tx_dummy_len );
	
	dw_qspi_writel(dws, DW_QSPI_SPI_CTRLR0, spi_ctrl0);
	dw_qspi_writel(dws, DW_QSPI_CTRL0, cr0);
	dw_qspi_writel(dws, DW_QSPI_CTRL1, dws->rx && (tmod_type != QSPI_TMOD_TR) ? dws->rx_end - dws->rx:0);	
	return 0;
}

static int dw_qspi_wait_done(struct dw_qspi *dws)
{
	int wait_time = 100;
	while(!dws->done && wait_time--){
		mdelay(10);
	}
	if(wait_time <= 0){
		return -1;
	}
	return 0;
}
static int dw_qspi_transfer_one_message(struct spi_master *master,
		struct spi_message *m)
{
	struct dw_qspi *dws = spi_master_get_devdata(master);
	struct spi_device *spi = m->spi;
	struct chip_data *chip = spi_get_ctldata(spi);
	struct spi_transfer *t;
	dw_qspi_enhanced_msg_t  *enhanced_msg;
	dw_qspi_enhanced_msg_en_t *msg_en;
	int ret = 0;
	unsigned int frame_len_words, transfer_len_words;
	u32			t_freq = 0;
	int wlen = 1;
	u8 imask = 0;
	u16 txlevel = 0;
	u16 rxlevel = 0;
	u32 cr0;
	//u32 cr1;
	u32 spi_cr0;

	m->actual_length = 0;
	dws->master = master;
	mutex_lock(&dws->list_lock);
	qspi_enable_chip(dws, 0);
	qspi_set_cs(m->spi, true);
	dws->cur_msg = m;

	enhanced_msg = &dws->enhanced_msg;
	//enhanced_msg->dws = dws;
	
	msg_en = &enhanced_msg->msg_en[0];
	
	enhanced_msg->msg_num = 0;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if(!t->rx_buf && !t->tx_buf){
			dev_err(&spi->dev,
				"Bufferless transfer has length %u\n",
				t->len);
			continue;
		}
		
		if(!t_freq){
			t_freq = t->speed_hz;
		}
		if(t->rx_buf){
			msg_en->buf = t->rx_buf;
			msg_en->istx = 0;
			msg_en->width_len = t->rx_nbits;
		}else{
			msg_en->buf = t->tx_buf;
			msg_en->istx = 1;
			msg_en->width_len = t->tx_nbits;
		}
		msg_en->len = t->len;
		m->actual_length +=  t->len;
		enhanced_msg->msg_num++;
		msg_en++;
	}
	dws->start = 0;
	dws->rx = dws->rx_end = NULL;
	dws->tx = dws->tx_end = NULL;
	dws->len = 0;
	if(dw_qspi_enhanced_config(spi,dws) < 0){
		goto err_free_master;
	}
	
	dws->cur_chip = spi_get_ctldata(spi);
		/* Handle per transfer options for bpw and speed */
	if (t_freq != dws->current_freq) {
		if (t_freq != chip->speed_hz) {
			/* clk_div doesn't support odd number */
			chip->clk_div = (DIV_ROUND_UP(dws->max_freq, t_freq) + 1) & 0xfffe;
			chip->speed_hz = t_freq;
		}
		dws->current_freq = t_freq;
		qspi_set_clk(dws, chip->clk_div);
	}
	
	if (!chip->poll_mode) {
		txlevel = min_t(u16, dws->fifo_len / 2, (dws->tx_end - dws->tx)/ dws->n_bytes);
		rxlevel = min_t(u16, dws->fifo_len / 4, (dws->rx_end - dws->rx)/ dws->n_bytes);
		dw_qspi_writel(dws, DW_QSPI_TXFLTR, txlevel);
		dw_qspi_writel(dws, DW_QSPI_RXFLTR, rxlevel);
		dws->start = 1;

		/* Set the interrupt mask */
		imask |= QSPI_INT_TXEI | QSPI_INT_TXOI |
			 QSPI_INT_RXUI ;
		qspi_umask_intr(dws, imask);
		//pr_emerg("[%s]%d:txlevel:%d rxlevel:%d rx_end:%lx,rx:%lx,tx_end:%lx,tx:%lx,",__func__,__LINE__,
		//	txlevel,rxlevel,dws->rx_end,dws->rx,dws->tx_end,dws->tx);
		dws->done = 0;
		dws->transfer_handler = interrupt_transfer_enhanced;
	}
	qspi_enable_chip(dws, 1);	
	if (chip->poll_mode){
		bst_qspi_poll_transfer(dws);
		dws->done = 1;
		m->status = 0;
	}

	if(dw_qspi_wait_done(dws)){
		m->status = -ETIMEDOUT;
	}
	qspi_set_cs(m->spi, false);
	qspi_enable_chip(dws, 0);
	spi_finalize_current_message(master);
	dws->cur_msg = NULL;
	mutex_unlock(&dws->list_lock);	
	return 0;
	
err_free_master:
	m->status = -EIO;
	qspi_set_cs(m->spi, false);
	spi_finalize_current_message(master);
	dws->cur_msg = NULL;
	mutex_unlock(&dws->list_lock);
	return 0;

}

		
static void dw_qspi_handle_err(struct spi_controller *master,
		struct spi_message *msg)
{
	struct dw_qspi *dws = spi_controller_get_devdata(master);

	if (dws->dma_mapped)
		dws->dma_ops->dma_stop(dws);

	//qspi_reset_chip(dws);
}

/* This may be called twice for each spi dev */
static int dw_qspi_setup(struct spi_device *spi)
{
	struct dw_qspi_chip *chip_info = NULL;
	struct chip_data *chip;
	int ret;
	
	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;

		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;
	}else{
		spi->controller_data = &def_chip_info;
		if (def_chip_info.cs_control)
			chip->cs_control = def_chip_info.cs_control;

		chip->poll_mode = def_chip_info.poll_mode;
		chip->type = def_chip_info.type;
	}

	chip->tmode = QSPI_TMOD_TR;

	if (gpio_is_valid(spi->cs_gpio)) {
		pr_emerg("[%s]%d Init cs_gpio:%d",__func__,__LINE__,spi->cs_gpio);
		ret = gpio_direction_output(spi->cs_gpio,
				!(spi->mode & SPI_CS_HIGH));
		if (ret)
			return ret;
	}

	return 0;
}

static void dw_qspi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void qspi_hw_init(struct device *dev, struct dw_qspi *dws)
{
	qspi_reset_chip(dws);
	
	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 8 to 256 from HW spec
	 */
	if (!dws->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			dw_qspi_writel(dws, DW_QSPI_TXFLTR, fifo);
			if (fifo != dw_qspi_readl(dws, DW_QSPI_TXFLTR))
				break;
		}
		dw_qspi_writel(dws, DW_QSPI_TXFLTR, 0);

		dws->fifo_len = (fifo == 1) ? 0 : fifo;
		dev_dbg(dev, "Detected FIFO size: %u bytes\n", dws->fifo_len);
		dev_emerg(dev, "Detected FIFO size: %u bytes\n", dws->fifo_len);
	}
}

int dw_qspi_add_host(struct device *dev, struct dw_qspi *dws)
{
	struct spi_controller *master;
	int ret;

	BUG_ON(dws == NULL);

	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	dws->master = master;
	dws->type = SSI_MOTO_SPI;
	dws->spi_type = SPI_STANDARD;
	dws->dma_inited = 0;
	dws->n_bytes = 1;
	dws->dma_addr = (dma_addr_t)(dws->paddr + DW_QSPI_DR);
	qspiregtopcfg = ioremap(QSPI_REG_TOP_CFG_R,0x1000);

	spi_controller_set_devdata(master, dws);
	pr_debug("%s %d dev-name=%s dws->irq=%d\n",__func__,__LINE__,dev_name(dev),dws->irq);
	//qspi_umask_intr(dws, QSPI_INT_TXEI);
	//master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->mode_bits = SPI_MODE_0 | SPI_LOOP | SPI_RX_DUAL | SPI_RX_QUAD |SPI_TX_DUAL |SPI_TX_QUAD|SPI_RX_OCTAL |SPI_TX_OCTAL;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	master->bus_num = dws->bus_num;
	master->num_chipselect = dws->num_cs;
	if(dws->use_gpio_cs){
		master->cs_gpios = dws->cs_gpios;
	}
	master->setup = dw_qspi_setup;
	master->cleanup = dw_qspi_cleanup;
	if( SSI_WM_NORMAL == dws->work_mode){
		master->transfer_one = dw_qspi_transfer_one;
	}else{
		master->set_cs = qspi_set_cs;
		master->transfer_one_message = dw_qspi_transfer_one_message;
	}
	master->handle_err = dw_qspi_handle_err;
	master->max_speed_hz = dws->max_freq;
	master->dev.of_node = dev->of_node;
	//master->flags = SPI_MASTER_GPIO_SS;
	//master->cs_gpios = &(dws->cs_gpio);

	if (dws->set_cs)
		master->set_cs = dws->set_cs;

	/* Basic HW init */
	qspi_hw_init(dev, dws);
	mutex_init(&dws->list_lock);
	ret = request_irq(dws->irq, dw_qspi_irq, IRQF_SHARED, dev_name(dev),
			  master);
	if (ret < 0) {
		dev_err(dev, "can not get IRQ\n");
		goto err_free_master;
	}

	if (dws->dma_ops && dws->dma_ops->dma_init) {
		ret = dws->dma_ops->dma_init(dws);
		if (ret) {
			dev_warn(dev, "DMA init failed\n");
			dws->dma_inited = 0;
		} else {
			master->can_dma = dws->dma_ops->can_dma;
		}
	}

	ret = devm_spi_register_controller(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_dma_exit;
	}
	pr_emerg("%s %d\n",__func__,__LINE__);

	dw_qspi_debugfs_init(dws);
	return 0;

err_dma_exit:
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	qspi_enable_chip(dws, 0);
	free_irq(dws->irq, master);
err_free_master:
	spi_controller_put(master);
	return ret;
}
EXPORT_SYMBOL_GPL(dw_qspi_add_host);

void dw_qspi_remove_host(struct dw_qspi *dws)
{
	dw_qspi_debugfs_remove(dws);
	iounmap(qspiregtopcfg);

	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);

	qspi_shutdown_chip(dws);

	free_irq(dws->irq, dws->master);
}
EXPORT_SYMBOL_GPL(dw_qspi_remove_host);

int dw_qspi_suspend_host(struct dw_qspi *dws)
{
	int ret;

	ret = spi_controller_suspend(dws->master);
	if (ret)
		return ret;

	qspi_shutdown_chip(dws);
	return 0;
}
EXPORT_SYMBOL_GPL(dw_qspi_suspend_host);

int dw_qspi_resume_host(struct dw_qspi *dws)
{
	int ret;

	qspi_hw_init(&dws->master->dev, dws);
	ret = spi_controller_resume(dws->master);
	if (ret)
		dev_err(&dws->master->dev, "fail to start queue (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(dw_qspi_resume_host);

MODULE_AUTHOR("Feng Tang <feng.tang@intel.com>");
MODULE_DESCRIPTION("Driver for DesignWare SPI controller core");
MODULE_LICENSE("GPL v2");

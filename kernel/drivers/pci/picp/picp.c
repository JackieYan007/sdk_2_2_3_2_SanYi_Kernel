// SPDX-License-Identifier: GPL-2.0
/*
 * BST PCIE Interconnection Control Protocol system.
 *
 * Copyright (C) 2021 Black Sesame Technologies, Inc.
 */
#define pr_fmt(fmt) "picp: " fmt
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/dma-buf.h>
#include <linux/crc16.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
//#include <linux/picp.h>
#include "pcie_test.h"
#include "picp_private.h"

#define DMA_CH_NUM  4
//#define CRC_CHECK_ENABLE

static dev_t devid;
static struct cdev pcie_cdev;
static struct class *pclass;
static struct device *pdev;

static u32 iontest_exists;
static u32 picp_sniffer;
static struct picp_record record = {0};
static u32 dma_read_channel[DMA_CH_NUM] = {0};
static u32 dma_write_channel[DMA_CH_NUM] = {0};
static struct task_handle picp_kthread[8] = {0};
struct dw_pcie *pci;
static int (*pcie_dma_read)(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst);
static int (*pcie_dma_write)(struct dw_pcie *pci, u32 ch, u32 size, u64 src, u64 dst);
static u32 picp_ctrl;
static wait_queue_head_t chipid_queue;
static struct task_struct *chipid_task;
static u32 chipid_flag;

static long get_current_time(void)
{
	return ktime_get_boottime()/1000000;
}

#ifdef PICP_DEBUG_FUNC
static void print_head(struct picp_head *head)
{
	picp_dbg("head:%d dtype:%d itype:%d dataid:%d len:%d ",
		head->head, head->dtype, head->itype, head->dataid, head->length);
}

static void record_time(int line, int print)
{
	static long rtime[5] = {0};
	int i = 0;

	rtime[line] = ktime_get_boottime();
	if (print) {
		for (i = 0; i < 5; i++)
			pr_err("** line:%d time:%ld", i, rtime[i]);
	}
}
#endif

static u32 alloc_dma_channel(u32 rwopt)
{
	u32 num, i, min = 0;
	u32 *channel = NULL;

	if (rwopt == PICP_WRITE)
		channel = dma_write_channel;
	else
		channel = dma_read_channel;

	num = channel[0];
	for (i = 0; i < DMA_CH_NUM; i++) {
		if (num > channel[i]) {
			num = channel[i];
			min = i;
		}
	}
	channel[min]++;

	return min;
}

static void free_dma_channel(u32 ch, u32 rwopt)
{
	u32 *channel;

	if (rwopt == PICP_WRITE)
		channel = dma_write_channel;
	else
		channel = dma_read_channel;

	if (channel[ch] > 0)
		channel[ch]--;
}

static void error_classify(int rwopt, int errno)
{
	if (rwopt == PICP_WRITE) {
		if (errno == PCIE_ERR_TIMEOUT)
			record.wtimeout++;
		else if (errno == PCIE_ERR_DMAERR)
			record.wdmaerr++;
		else if (errno == PCIE_ERR_LINKERR)
			record.wlinkerr++;
		else if (errno == PCIE_ERR_STOPTRF || errno == PCIE_ERR_PARAERR)
			return;
		record.werrors++;
	} else {
		if (errno == PCIE_ERR_TIMEOUT)
			record.rtimeout++;
		else if (errno == PCIE_ERR_DMAERR)
			record.rdmaerr++;
		else if (errno == PCIE_ERR_LINKERR)
			record.rlinkerr++;
		else if (errno == PCIE_ERR_STOPTRF || errno == PCIE_ERR_PARAERR)
			return;
		record.rerrors++;
	}
}

static int print_buffer(ulong addr, ulong physaddr, const void *data, uint width, uint count,
		 uint linelen)
{
	int i;

	while (count) {
		uint thislinelen = linelen;

		pr_cont("0x%08lX (0x%08lX):", physaddr, addr);

		/* check for overflow condition */
		if (count < thislinelen)
			thislinelen = count;

		for (i = 0; i < thislinelen; i++) {
			printk(KERN_CONT" %08X", *(u32 *)data);
			data += width;
		}

		/* update references */
		addr += thislinelen * width;
		physaddr += thislinelen * width;
		count -= thislinelen;

		printk(KERN_CONT "\n");
	}

	return 0;
}

static void picp_sniffer_packet(struct picp_info *info)
{
	u32 length = 0;

	if (info->hvirt) {
		length = sizeof(struct picp_head)/4;
		printk("\n** sniffer packet: head %d\n", length);
		print_buffer((ulong)info->hvirt, info->hlocal, info->hvirt, 4, length, 4);
	}

	if (info->dvirt) {
		length = info->dsize/4;
		length = (length > 128) ? 128 : length;
		printk("\n** sniffer packet: data %d\n", length);
		print_buffer((ulong)info->dvirt, info->dlocal, info->dvirt, 4, length, 4);
	}
}

void picp_stop_transfer(struct picp_info *info)
{
	info->stop_flag = 1;
}
EXPORT_SYMBOL(picp_stop_transfer);

static int picp_should_stop(struct picp_info *info)
{
	if ((current->flags & PF_KTHREAD) && kthread_should_stop())
		return 1;

	if (info->stop_flag)
		return 1;

	return 0;
}

static int picp_poll(struct picp_info *info, int itype)
{
	int ret = 0;
	u32 delay = 0;
	long start, curr;
	struct picp_head *hvirt = info->hvirt;

	start = get_current_time();
	while (delay < info->timeout) {
		curr = get_current_time();
		delay = curr - start;

		if (picp_should_stop(info))
			return PCIE_ERR_STOPTRF;

		if (info->rwside) {
			ret = pcie_dma_read(pci, info->channel, info->hsize, info->hremote, info->hlocal);
			if (ret < 0) {
				//if (ret == PCIE_ERR_LINKERR || ret == PCIE_ERR_DMAERR)
				picp_err("Fail src:0x%llx dst:0x%llx\n", info->hremote, info->hlocal);
				return ret;
			}
		}

		switch (itype) {
		case PICP_ITYP_REQ:
			if ((ioread16(&hvirt->itype) == itype) && (info->previd != hvirt->dataid)) {
				return true;
			} else if (!hvirt->cont) {
				//start = get_current_time();
			}
			break;
		case PICP_ITYP_RDONE:
			if ((ioread16(&hvirt->itype) == itype) && (hvirt->dataid == info->currid + 1)) {
				return true;
			} else if (!hvirt->cont) {
				//start = get_current_time();
			}
			break;
		case PICP_ITYP_WDONE:
			if ((ioread16(&hvirt->itype) == itype) && (hvirt->dataid == info->currid + 1)) {
				return true;
			} else if (!hvirt->cont) {
				//start = get_current_time();
			}
			break;
		case PICP_ITYP_ACK:
		case PICP_ITYP_RWENABLE:
			if ((ioread16(&hvirt->itype) == itype) && (info->previd != hvirt->dataid))
				return true;
			break;
		case PICP_ITYP_IDLE:
			if (ioread16(&hvirt->itype) == itype)
				return true;
			break;
		default:
			picp_err("** picp_poll: pollside type error\n");
			return -1;
		}
		//picp_dbg("local:itype:%d previd:%d currid:%d remote:itype:%d id:%d\n",
			//itype, info->previd, info->currid, hvirt->itype, hvirt->dataid);

		usleep_range(10, 20);
	}

	if (ret < 0) {
		picp_err("** poll fail dma or link error:%d\n", ret);
		return ret;
	} else {
		//picp_err("** poll fail Timeout lo-itype:%d re-itype:%d\n", itype, ioread16(&hvirt->itype));
		return PCIE_ERR_TIMEOUT;
	}
}

static int get_remote_addrsee(struct picp_info *info)
{
	int ret = 0;

	picp_dbg("1.get remote address\n");
	ret = picp_poll(info, PICP_ITYP_RWENABLE);

	return ret;
}

static int set_remote_done(struct picp_info *info, int itype)
{
	u32 size = 0, ret = 0;
	u64 srcaddr = 0, destaddr = 0;
	struct picp_head *hvirt = info->hvirt;

	picp_dbg("3.send done\n");

#ifdef CRC_CHECK_ENABLE
	hvirt->verify = crc16(0, info->dvirt, hvirt->length);
	picp_err("data crc16 : 0x%x\n", hvirt->verify);
#endif

	info->previd = hvirt->dataid;
	hvirt->dataid = hvirt->dataid + 1;
	hvirt->itype = itype;

	srcaddr  = info->hlocal;
	destaddr = info->hremote;
	size = sizeof(struct picp_head);

	ret = pcie_dma_write(pci, info->channel, size, srcaddr, destaddr);

	return ret;
}

static int check_remote_status(struct picp_info *info, int itype)
{
	return picp_poll(info, itype);
}

static int dma_read(struct picp_info *info, u64 dma_buf, u32 len, u32 type)
{
	int ret = 0;
	struct picp_head *hvirt = info->hvirt;

	info->record++;
	record.rtotal++;

	ret = get_remote_addrsee(info);
	if (ret < 0)
		goto fail;
	info->currid = hvirt->dataid;
	picp_dbg("2.remote:%#llx,%d id:%d local:%#llx,%d\n",
		hvirt->physaddr, hvirt->length, hvirt->dataid, dma_buf, len);

	switch (type) {
	case PICP_DTYP_DATA:
		if (hvirt->length > len) {
			ret = -1;
			goto fail;
		}

		ret = pcie_dma_read(pci, info->channel, hvirt->length, hvirt->physaddr, dma_buf);
		if (ret < 0)
			goto fail;

		hvirt->dtype = type;
		hvirt->physaddr = dma_buf;
		ret = set_remote_done(info, PICP_ITYP_RDONE);
		if (ret < 0)
			goto fail;
		break;

	case PICP_DTYP_CHIPID:
		info->custom = hvirt->physaddr;
		hvirt->dtype = type;
		hvirt->physaddr = dma_buf;
		ret = set_remote_done(info, PICP_ITYP_RDONE);
		if (ret < 0)
			goto fail;

		ret = check_remote_status(info, PICP_ITYP_IDLE);
		if (ret < 0)
			goto fail;
		break;
	default:
		break;
	}
	picp_dbg("success dataid:%d\n", info->currid);

	record.rbytes += hvirt->length;
	return hvirt->length;

fail:
	picp_dbg("Fail id:%d local %#x ret:%d\n", info->currid, dma_buf, ret);
	return ret;
}

static int mem_read(struct picp_info *info, u64 dma_buf, u32 len, u32 type)
{
	int ret = 0;
	struct picp_head *hvirt = info->hvirt;

	info->record++;
	record.rtotal++;

	picp_dbg("1.local data %#llx,%d id:%d\n", dma_buf, len, info->record);
	hvirt->dtype = type;
	hvirt->length = len;
	hvirt->physaddr = dma_buf;
	hvirt->dataid = info->record;
	hvirt->itype  = PICP_ITYP_RWENABLE;
	info->currid = hvirt->dataid;

	picp_dbg("2.waiting for done\n");
	ret = picp_poll(info, PICP_ITYP_WDONE);
	if (ret < 0)
		goto fail;
	info->custom = hvirt->physaddr;

#ifdef CRC_CHECK_ENABLE
	u16 crcver = 0;

	crcver = crc16(0, info->dvirt, len);
	if (crcver != hvirt->verify) {
		picp_err("crc16 local:0x%x remote:0x%x\n", crcver, hvirt->verify);
		goto fail;
	}
#endif
	hvirt->itype = PICP_ITYP_IDLE;
	picp_dbg("3.success id:%d,len:%d\n", info->currid, hvirt->length);

	record.rbytes += hvirt->length;
	return hvirt->length;

fail:
	picp_dbg("fail dataid:%d ret:%d\n", info->currid, ret);
	hvirt->itype = PICP_ITYP_IDLE;
	return ret;
}

int picp_read(struct picp_private *priv, u64 dma_buf, u32 len, u32 type)
{
	int ret = -EFAULT;

	if (priv && priv->read_data)
		ret = priv->read_data(&priv->rinfo, dma_buf, len, type);

	if (ret < 0 && ret != -EFAULT)
		error_classify(PICP_READ, ret);

	if (picp_sniffer)
		picp_sniffer_packet(&priv->winfo);

	return ret;
}
EXPORT_SYMBOL(picp_read);

static int dma_write(struct picp_info *info, u64 dma_buf, u32 len, u32 type)
{
	int ret = 0;
	struct picp_head *hvirt =  info->hvirt;

	info->record++;
	record.wtotal++;

	ret = get_remote_addrsee(info);
	if (ret < 0)
		goto fail;
	info->currid = hvirt->dataid;
	picp_dbg("2.remote:%#llx,%d id:%d local:%#llx,%d\n",
		hvirt->physaddr, hvirt->length, hvirt->dataid, dma_buf, len);

	switch (type) {
	case PICP_DTYP_DATA:
		if (len > hvirt->length) {
			ret = -1;
			goto fail;
		}

		ret = pcie_dma_write(pci, info->channel, len, dma_buf, hvirt->physaddr);
		if (ret < 0)
			goto fail;

		hvirt->dtype = type;
		hvirt->physaddr = dma_buf;
		hvirt->length = len;
		ret = set_remote_done(info, PICP_ITYP_WDONE);
		if (ret < 0)
			goto fail;
		break;

	case PICP_DTYP_CHIPID:
		info->custom = hvirt->physaddr;
		hvirt->dtype = type;
		hvirt->physaddr = dma_buf;
		ret = set_remote_done(info, PICP_ITYP_WDONE);
		if (ret < 0)
			goto fail;

		ret = check_remote_status(info, PICP_ITYP_IDLE);
		if (ret < 0)
			goto fail;
		break;
	default:
		break;
	}
	picp_dbg("success dataid:%d\n", info->currid);

	record.wbytes += len;
	return len;

fail:
	picp_dbg("Fail id:%d local %#x ret:%d\n", info->currid, dma_buf, ret);
	return ret;
}

static int mem_write(struct picp_info *info, u64 dma_buf, u32 len, u32 type)
{
	int ret = 0;
	struct picp_head *hvirt = info->hvirt;

	info->record++;
	record.wtotal++;

	picp_dbg("1.local data %#llx,%d id:%d\n", dma_buf, len, info->record);
	hvirt->dtype = type;
	hvirt->length = len;
	hvirt->physaddr = dma_buf;
	hvirt->dataid = info->record;
	hvirt->itype  = PICP_ITYP_RWENABLE;
	info->currid = hvirt->dataid;

	picp_dbg("2.waiting for done id:%d\n", info->currid);
	ret = picp_poll(info, PICP_ITYP_RDONE);
	if (ret < 0) {
		goto fail;
	}
	info->custom = hvirt->physaddr;

#ifdef CRC_CHECK_ENABLE
	u16 crcver = 0;

	crcver = crc16(0, info->dvirt, len);
	if (crcver != hvirt->verify) {
		picp_err("crc16 local:%#x remote:%#x\n", crcver, hvirt->verify);
		goto fail;
	}
#endif
	hvirt->itype = PICP_ITYP_IDLE;
	picp_dbg("3.success dataid:%d\n", info->currid);

	record.wbytes += len;
	return len;

fail:
	picp_dbg("fail dataid:%d ret:%d\n", info->currid, ret);
	hvirt->itype = PICP_ITYP_IDLE;
	return ret;
}

int picp_write(struct picp_private *priv, u64 dma_buf, u32 len, u32 type)
{
	int ret = -EFAULT;

	if (priv && priv->write_data)
		ret = priv->write_data(&priv->winfo, dma_buf, len, type);

	if (ret < 0 && ret != -EFAULT)
		error_classify(PICP_WRITE, ret);

	if (picp_sniffer)
		picp_sniffer_packet(&priv->winfo);

	return ret;
}
EXPORT_SYMBOL(picp_write);

static int alloc_picp_head(struct picp_private *priv, struct picp_cfg *cfg)
{
	if (!priv || !cfg)
		return -EFAULT;

	priv->vaddr = ioremap(cfg->local_head, cfg->head_physize);
	if (!priv->vaddr) {
		picp_err("Fail: %#llx ioremap\n", cfg->local_head);
		return -1;
	}
	memset_io(priv->vaddr, 0, cfg->head_physize);

	return 0;
}

static void free_picp_head(struct picp_private *priv)
{
	if (!priv->vaddr)
		return;

	iounmap(priv->vaddr);
}

static int init_picp_head(struct picp_private *priv, struct picp_cfg *cfg)
{
	int ret = 0;

	if (!priv || !cfg)
		return -1;

	ret = alloc_picp_head(priv, cfg);
	if (ret)
		return -1;

	/* 1.local virt address */
	priv->rinfo.hvirt = priv->vaddr;
	priv->winfo.hvirt = priv->rinfo.hvirt + 1;//sizeof(struct picp_head);

	/* 2.local phy address */
	priv->rinfo.hlocal = cfg->local_head;
	priv->winfo.hlocal = cfg->local_head + sizeof(struct picp_head);

	/* 3.remote phy address */
	priv->rinfo.hremote = cfg->remote_head + sizeof(struct picp_head);
	priv->winfo.hremote = cfg->remote_head;

	priv->rinfo.hsize = sizeof(struct picp_head);
	priv->winfo.hsize = sizeof(struct picp_head);

	/* local head init */
	priv->rinfo.hvirt->head = 0x12345678;
	priv->winfo.hvirt->head = 0x12345678;

	priv->winfo.hvirt->verify = 0x87654321;
	priv->rinfo.hvirt->verify = 0x11111111;

	return ret;
}

/* alloc picp resources and init */
struct picp_private *picp_init(struct picp_cfg *cfg)
{
	int ret;
	u32 rchannel = 0, wchannel = 0;
	struct picp_private *priv = NULL;

	printk(KERN_INFO "%s\n", __func__);
	priv = kmalloc(sizeof(struct picp_private), GFP_KERNEL);
	if (!priv) {
		picp_err("Fail: kmalloc picp_private\n");
		return NULL;
	}
	memset(priv, 0, sizeof(struct picp_private));

	ret = init_picp_head(priv, cfg);
	if (ret < 0)
		goto fail;

	priv->rwside = picp_ctrl;//get_pcie_mode();
	if (priv->rwside == PCIE_DMA_SIDE) {
		priv->read_data = &dma_read;
		priv->write_data = &dma_write;
	} else {
		priv->read_data = &mem_read;
		priv->write_data = &mem_write;
	}

	rchannel = alloc_dma_channel(PICP_READ);
	priv->rinfo.rwside  = priv->rwside;
	priv->rinfo.timeout = cfg->timeout;
	priv->rinfo.channel = rchannel;
	snprintf(priv->rinfo.name, 32, "%s", cfg->rname);
	//memcpy(priv->rinfo.name, cfg->name, 32);

	wchannel = alloc_dma_channel(PICP_WRITE);
	priv->winfo.rwside  = priv->rwside;
	priv->winfo.timeout = cfg->timeout;
	priv->winfo.channel = wchannel;
	snprintf(priv->winfo.name, 32, "%s", cfg->wname);
	//memcpy(priv->winfo.name, cfg->name, 32);

	printk(KERN_INFO "%s/%s Success: rch%d wch%d rwside:%d\r\n"
			 "->read  hlocal: %#llx hremote: %#llx\r\n"
			 "->write hlocal: %#llx hremote: %#llx\r\n\n",
			 cfg->rname, cfg->wname, rchannel, wchannel, priv->rwside,
			 priv->rinfo.hlocal, priv->rinfo.hremote,
			 priv->winfo.hlocal, priv->winfo.hremote);

	return priv;

fail:
	kfree(priv);
	return NULL;
}
EXPORT_SYMBOL(picp_init);

/* release picp resources */
void picp_exit(struct picp_private *priv)
{
	if (!priv)
		return;

	free_dma_channel(priv->rinfo.channel, 0);
	free_dma_channel(priv->winfo.channel, 1);
	free_picp_head(priv);

	kfree(priv);
}
EXPORT_SYMBOL(picp_exit);


/************************************** Example & Test Start **************************************/
static int read_kthread(void *param)
{
	int ret = 0, recodnum = 0;
	long start, curr, datasize = 0;
	u64 dataphys = 0;
	void *datavirt = NULL;
	struct task_handle *task = NULL;
	struct picp_private *priv = NULL;
	struct picp_info *info = NULL;
	static u32 total, errors;

	task = (struct task_handle *)param;
	priv = task->priv;
	info = &priv->rinfo;
	datasize = task->datasize;

	/* alloc data phys addr */
	datavirt = kmalloc(datasize, GFP_DMA);
	if (!datavirt) {
		picp_err("kmalloc failed size:%ld\n", datasize);
		goto exit;
	}
	memset(datavirt, 0xa, datasize);
	dataphys = virt_to_phys(datavirt);
	info->dvirt = datavirt;
	info->dsize = datasize;
	picp_err("** %s running...\n", current->comm);

	/* pcie dma read only */
	if (task->single) {
		while (!kthread_should_stop() && priv->rwside) {
			start = ktime_get_boottime();
			ret = pcie_dma_read(pci, info->channel, datasize, dataphys,
				PICP_REMOTE_HEAD_MEM);
			curr = ktime_get_boottime();
			if (ret < 0) {
				picp_err("read-only fail");
			} else {
				picp_err("read-only rch%d:%d size:%ld time:%ld speed:%ld MB/s\n\n",
					info->channel, recodnum++, datasize, curr-start,
					((datasize*1000000000)/(curr-start))/(1024*1024));
			}
			msleep(200);
			//usleep_range(500, 800);
		}
		goto exit;
	}

	/* picp read */
	while (!kthread_should_stop()) {
		start = ktime_get_boottime();
		ret = picp_read(priv, dataphys, datasize, PICP_DTYP_DATA);
		curr = ktime_get_boottime();

		if (ret == PCIE_ERR_STOPTRF)
			break;

		total++;
		if (ret < 0) {
			picp_err("Fail: Read total:%d err:%d ret:%d\n\n",
				total, ++errors, ret);
			msleep(500); continue;
		}

		if (priv->rwside) {
			picp_err("rch:%d Read total:%d err:%d time:%ld len:%ld speed:%ld MB/s\n\n",
				info->channel, total, errors, curr-start, datasize,
				((datasize*1000000000)/(curr-start))/(1024*1024));
		} else {
			picp_err("rch:%d Read total:%d err:%d\n\n",
				info->channel, total, errors);
		}

		msleep(100);
	}

exit:
	if (datavirt)
		kfree(datavirt);

	printk("** %s exit\n", current->comm);
	return 0;
}

static int write_kthread(void *param)
{
	int ret = 0, recodnum = 0;
	long start, curr, datasize = 0;
	u64 dataphys = 0;
	void *datavirt = NULL;
	struct task_handle *task = NULL;
	struct picp_private *priv = NULL;
	struct picp_info *info = NULL;
	static u32 total, errors;

	task = (struct task_handle *)param;
	priv = task->priv;
	info = &priv->winfo;
	datasize = task->datasize;

	/* alloc data phys addr */
	datavirt = kmalloc(datasize, GFP_DMA);
	if (!datavirt) {
		picp_err("kmalloc failed size:%ld\n", datasize);
		goto exit;
	}
	memset(datavirt, 0xb, datasize);
	dataphys = virt_to_phys(datavirt);
	info->dvirt = datavirt;
	info->dsize = datasize;
	picp_err("** %s running... %#llx\n", current->comm, dataphys);

	/* pcie dma write only */
	if (task->single) {
		while (!kthread_should_stop() && priv->rwside) {
			start = ktime_get_boottime();
			ret = pcie_dma_write(pci, info->channel, datasize, dataphys,
					PICP_REMOTE_HEAD_MEM);
			curr = ktime_get_boottime();
			if (ret < 0) {
				picp_err("write-only fail");
			} else {
				picp_err("write-only wch%d:%d size:%ld time:%ld speed:%ld MB/s\n\n",
					info->channel, recodnum++, datasize, curr-start,
					((datasize*1000000000)/(curr-start))/(1024*1024));
			}
			msleep(200);
			//usleep_range(500, 800);
		}
		goto exit;
	}

	/* picp write */
	while (!kthread_should_stop()) {
		start = ktime_get_boottime();
		ret = picp_write(priv, dataphys, datasize, PICP_DTYP_DATA);
		curr = ktime_get_boottime();

		if (ret == PCIE_ERR_STOPTRF)
			break;

		total++;
		if (ret < 0) {
			picp_err("Fail: Write total:%d err:%d ret:%d\n\n",
				total, ++errors, ret);
			msleep(500); continue;
		}

		if (priv->rwside) {
			picp_err("wch:%d Write total:%d err:%d time:%ld len:%ld speed:%ld MB/s\n\n",
				info->channel, total, errors, curr-start, datasize,
				((datasize*1000000000)/(curr-start))/(1024*1024));
		} else {
			picp_err("wch:%d Write total:%d err:%d\n\n",
				info->channel, total, errors);
		}
		msleep(100);
	}

exit:
	if (datavirt)
		kfree(datavirt);

	printk("** %s exit\n", current->comm);
	return 0;
}

static int create_picp_kthread(struct ioctl_info ioc)
{
	int i, num = 1;
	struct picp_cfg config;
	u32 size = 1024;
	struct picp_private *priv = NULL;

	u64 local_head_physaddr  = PICP_LOCAL_HEAD_CHIPID;
	u64 remote_head_physaddr = PICP_REMOTE_HEAD_CHIPID;
	struct task_handle *kth = picp_kthread;

	if (ioc.single && !picp_ctrl) {
		pr_err("picp ctrl type is mem, do this in dma side\n");
		return -1;
	}

	if (ioc.size)
		size = ioc.size;

	if (ioc.thread)
		num = ioc.thread;

	for (i = 0; i < num; i++) {
		if (!kth[i].priv) {
			config.timeout = 5000;
			config.local_head  = local_head_physaddr  + (i * 128);
			config.remote_head = remote_head_physaddr + (i * 128);
			config.head_physize = 128;//
			snprintf(config.rname, 32, "picp-read-kth%d", i);
			snprintf(config.wname, 32, "picp-write-kth%d", i);
			kth[i].single = ioc.single;
			kth[i].datasize = size;

			priv = picp_init(&config);
			if (!priv) {
				picp_err("picp_init failed\n");
				return -1;
			}
			kth[i].priv = priv;
		}

		if (ioc.rwopt == PICP_WRITE) {
			if (kth[i].wtask) {
				printk(KERN_INFO "kthread already exists\n");
				continue;
			}
			kth[i].wtask = kthread_run(write_kthread, &kth[i], config.wname);
			if (IS_ERR(kth[i].wtask))
				goto fail;

		} else {
			if (kth[i].rtask) {
				printk(KERN_INFO "kthread already exists\n");
				continue;
			}
			kth[i].rtask = kthread_run(read_kthread, &kth[i], config.rname);
			if (IS_ERR(kth[i].rtask))
				goto fail;
		}
	}

	return 0;
fail:
	picp_exit(priv);
	kth[i].priv = NULL;
	return -1;
}

static void stop_picp_kthread(void)
{
	int i, num = 4;
	struct task_handle *kth = picp_kthread;

	for (i = 0; i < num; i++) {
		if (kth[i].rtask && kth[i].rtask->state != TASK_DEAD) {
			kthread_stop(kth[i].rtask);
		}
		kth[i].rtask = NULL;

		if (kth[i].wtask && kth[i].wtask->state != TASK_DEAD) {
			kthread_stop(kth[i].wtask);
		}
		kth[i].wtask = NULL;

		if (kth[i].priv)
			picp_exit(kth[i].priv);
		kth[i].priv = NULL;
	}
}

static void picp_ioctl_rwtest(struct ioctl_info ioc)
{
	if (ioc.quit) {
		stop_picp_kthread();
		return;
	}

	ioc.thread = (ioc.thread > 4) ? 4 : ioc.thread;
	picp_sniffer = (ioc.sniffer == 0) ? picp_sniffer : 1;

	create_picp_kthread(ioc);
}
/************************************** Example & Test End **************************************/

static int ion_buf_import(struct ion_share_info *ion_info,
	struct picp_private *priv)
{
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *table;
	unsigned int reg_addr, reg_size;

	picp_err("ion_info len:%d type:%d typ:%d buffd:%d\r\n\n",
		ion_info->buflen, ion_info->flag_type, ion_info->heap_type, ion_info->buffd);

	dmabuf = dma_buf_get(ion_info->buffd);
	if (IS_ERR(dmabuf)) {
		return PTR_ERR(dmabuf);
	}

	attachment = dma_buf_attach(dmabuf, pdev);
	if (IS_ERR(attachment))
		return PTR_ERR(attachment);

	table = dma_buf_map_attachment(attachment, DMA_TO_DEVICE);
	if (IS_ERR(table))
		return PTR_ERR(table);

	reg_addr = sg_phys(table->sgl); //sg_dma_address(table->sgl);
	reg_size = table->sgl->length; //sg_dma_len(table->sgl);
	picp_err("reg_addr = 0x%x, reg_size = 0x%x\n", reg_addr, reg_size);

	if (ion_info->rwopt) {
		priv->winfo.dlocal = reg_addr;
		priv->winfo.dsize = reg_size;
	} else {
		priv->rinfo.dlocal = reg_addr;
		priv->rinfo.dsize = reg_size;
	}

#if 0
	vbuf = dma_buf_vmap(dmabuf);
	if (IS_ERR(vbuf)) {
		return PTR_ERR(vbuf);
	}
	picp_err("\nbuf :%s\n", vbuf);
	dma_buf_vunmap(dmabuf, vbuf);
#endif

	dma_buf_unmap_attachment(attachment, table, DMA_TO_DEVICE);
	dma_buf_detach(dmabuf, attachment);
	iontest_exists = 0;
	return 0;
}

//pcie chipid begin
static int pcie_chipid_kthread(void *ptr)
{
	struct picp_private *priv = NULL;
	struct picp_info *info = NULL;
	struct picp_cfg cfg;
	struct pcie_chipid *chipid = ptr;
	int ret;

	cfg.timeout = 5000;
	cfg.local_head = PICP_LOCAL_HEAD_CHIPID;
	cfg.remote_head = PICP_REMOTE_HEAD_CHIPID;
	cfg.head_physize = PICP_HEAD_CHIPID_SIZE;
	strncpy(cfg.rname, "picp-chipid", 32);
	strncpy(cfg.wname, "picp-chipid", 32);

	priv = picp_init(&cfg);
	if (!priv) {
		printk(KERN_ERR "picp_init failed\n");
		goto exit;
	}

	if (picp_ctrl == PCIE_DMA_SIDE)
		info = &priv->winfo;
	else if (picp_ctrl == PCIE_MEM_SIDE)
		info = &priv->rinfo;

	printk(KERN_INFO "local chipid :0x%llx\n", chipid->local_chipid);
	printk(KERN_INFO "%s running...\n", __func__);
	while(!kthread_should_stop()){
		if (picp_ctrl == PCIE_DMA_SIDE)
			ret = picp_write(priv, chipid->local_chipid, sizeof(u64), PICP_DTYP_CHIPID);
		else if (picp_ctrl == PCIE_MEM_SIDE)
			ret = picp_read(priv, chipid->local_chipid, sizeof(u64), PICP_DTYP_CHIPID);

		if (ret >= 0) {
			printk(KERN_INFO "local chipid:0x%llx remote chipid :0x%llx\n",
				chipid->local_chipid, info->custom);
			if (info->custom != 0) {
				chipid->remote_chipid = info->custom;
				chipid->state = 1;
				break;
			}
		}
		msleep(200);
	}

exit:
	chipid_flag = 1;
	wake_up_interruptible(&chipid_queue);

	printk(KERN_INFO "%s exit\n", __func__);
	picp_exit(priv);
	chipid_task = NULL;
	return 0;
}

static int pcie_chipid_init(struct pcie_chipid *chipid)
{
	printk(KERN_INFO "** chipid_pcie_init\n");
	chipid_task = kthread_run(pcie_chipid_kthread, chipid, "read_chipid_ep");
	if (IS_ERR(chipid_task))
		return -1;
	else
		return 0;
}

static void pcie_chipid_exit(void)
{
	if (chipid_task) {
		kthread_stop(chipid_task);
		chipid_task = NULL;
	}
}
//pcie chipid end

static long picp_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ion_share_info ion_info;
	struct picp_private *priv = NULL;
	struct picp_cfg config;
	struct ioctl_info ioc;
	int ret = 0, on = 0;
	static struct pcie_chipid chipid;

	if (_IOC_TYPE(cmd) != PICP_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > PICP_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case PICP_IOC_RWTEST:
		ret = copy_from_user(&ioc, (const void *)arg, _IOC_SIZE(cmd));
		if (ret)
			return -EFAULT;

		picp_ioctl_rwtest(ioc);
		break;
	case PICP_IOC_SNIFFER:
		ret = copy_from_user(&on, (const void *)arg, _IOC_SIZE(cmd));
		if (ret)
			return -EFAULT;

		picp_sniffer = on;
		printk(KERN_ERR "picp_sniffer = %d\n", picp_sniffer);
		break;

	case PICP_IOC_QUERY:
		record.ltssm_count = get_ltssm_count();
		ret = copy_to_user((void __user *)arg, &record, _IOC_SIZE(cmd));
		break;

	case PICP_IOC_CHIPID:
		memset(&chipid, 0, sizeof(chipid));
		if (copy_from_user(&chipid, (const void *)arg, _IOC_SIZE(cmd)))
		 	return -EFAULT;
		if (chipid_task)
			return -EBUSY;

		if (!pcie_chipid_init(&chipid)) {
			chipid_flag = 0;
			wait_event_interruptible_timeout(chipid_queue, chipid_flag != 0,
				msecs_to_jiffies(chipid.timeout*1000));
			chipid_flag = 0;
		}
		pcie_chipid_exit();
		 ret = copy_to_user((void __user *)arg, &chipid, _IOC_SIZE(cmd));
		break;

	case PICP_IOC_INIT:
		if (copy_from_user(&config, (const void *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

		if (iontest_exists) {
			printk(KERN_INFO "iontest already exists\n");
			return -1;
		}
		config.local_head  = PICP_LOCAL_HEAD_IONTEST;
		config.remote_head = PICP_REMOTE_HEAD_IONTEST;
		strncpy(config.rname, "picp-ion-r", 32);
		strncpy(config.wname, "picp-ion-w", 32);
		config.head_physize = PICP_HEAD_IONTEST_SIZE;

		priv = picp_init(&config);
		if (!priv) {
			return -EFAULT;
		}
		file->private_data = priv;
		iontest_exists = 1;
		break;
	case PICP_IOC_EXIT:
		if (!file->private_data)
			return -EFAULT;
		priv = file->private_data;
		picp_exit(priv);
		break;

	case PICP_IOC_SIONFD:
		if (!file->private_data)
			return -EFAULT;
		priv = file->private_data;

		if (copy_from_user(&ion_info, (const void *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
		ret = ion_buf_import(&ion_info, priv);
		break;

	default:
		picp_err("err cmd:%x\n", cmd);
		return -EEXIST;
	}

	return ret;
}

static ssize_t picp_cdev_read(struct file *file, char __user *buf,
	  size_t size, loff_t *ppos)
{
	int ret;
	struct picp_private *priv = NULL;

    if (!file->private_data)
		return -EFAULT;

	priv = file->private_data;
	priv->rinfo.dtype = PICP_DTYP_DATA;

	ret = picp_read(priv, priv->rinfo.dlocal, size, priv->rinfo.dtype);
	if (ret < 0) {
		picp_err("Fail: read total:%d errors:%d ret:%d\n\n",
			record.rtotal, record.rerrors, ret);
	}

    return ret;
}

static ssize_t picp_cdev_write(struct file *file, const char __user *buf,
	  size_t size, loff_t *ppos)
{
	int ret;
	struct picp_private *priv = NULL;

	if (!file->private_data)
		return -EFAULT;

	priv = file->private_data;
	priv->winfo.dtype = PICP_DTYP_DATA;

	ret = picp_write(priv, priv->winfo.dlocal, size, priv->winfo.dtype);
	if (ret < 0) {
		picp_err("Fail: write total:%d errors:%d ret:%d\n\n",
			record.wtotal, record.werrors, ret);
	}

    return ret;
}

static int picp_cdev_open(struct inode *inode, struct file *file)
{
	init_waitqueue_head(&chipid_queue);

    return 0;
}

struct file_operations pcip_cdev_fops = {
	.owner = THIS_MODULE,
	.open  = picp_cdev_open,
	.read  = picp_cdev_read,
	.write = picp_cdev_write,
	//.release =
	.unlocked_ioctl = picp_cdev_ioctl,
};

static int __init picp_cdev_init(void)
{
	u32 ctrl_id = 0xff;
	static u64 dmamsk;
	struct device_node *np = NULL;

	pr_info("picp init start...\n");
	for_each_node_by_type(np, "pci") {
		if (!of_device_is_available(np))
			continue;

		if (!of_find_property(np, "picp-ctl", NULL))
			continue;

		if (of_property_match_string(np, "compatible", "bst,dw-pcie-ep") >= 0) {
			pci = get_dw_pcie(DW_PCIE_EP_TYPE);
			if (!pci) {
				pr_err("pci ep not init\n");
				return -1;
			}
		} else if (of_property_match_string(np, "compatible", "bst,dw-pcie-rc") >= 0) {
			pci = get_dw_pcie(DW_PCIE_RC_TYPE);
			if (!pci) {
				pr_err("pci rc not init\n");
				return -1;
			}
		} else {
			pr_err("match string fail bst,dw-pcie-rc/ep\n");
			return -1;
		}
		ctrl_id = pci->ctrl_id;
		if (ctrl_id != 0 && ctrl_id != 1)
			ctrl_id = (pci->mode == DW_PCIE_EP_TYPE) ? 1 : 0;

		if (of_property_match_string(np, "picp-ctl", "dma") >= 0) {
			if (ctrl_id == 0) {
				pcie_dma_read = &dmc_dma_read;
				pcie_dma_write = &dmc_dma_write;
			} else if (ctrl_id == 1) {
				pcie_dma_read = &epc_dma_read;
				pcie_dma_write = &epc_dma_write;
			}
			picp_ctrl = PCIE_DMA_SIDE;
			pcie_dma_init(pci);
			pr_err("pcie controller: %d, mode: %s-DMA\n", ctrl_id,
				(pci->mode == DW_PCIE_RC_TYPE) ? "RC":"EP");
			break;
		} else if (of_property_match_string(np, "picp-ctl", "mem") >= 0) {
			pr_err("pcie controller: %d, mode: %s-MEM\n", ctrl_id,
				(pci->mode == DW_PCIE_RC_TYPE) ? "RC":"EP");
			break;
		} else {
			pr_err("picp-ctl Undefined\n");
			return -1;
		}
	}
	if (!pci) {
		pr_err("pci not init..\n");
		return -1;
	}

	alloc_chrdev_region(&devid, 0, 255, "picp");
	pclass = class_create(THIS_MODULE, "picp");
	if (IS_ERR(pclass)) {
		printk("picp: failed to allocate class\n");
		unregister_chrdev_region(devid, 255);
		return PTR_ERR(pclass);
	}

	cdev_init(&pcie_cdev, &pcip_cdev_fops);
	cdev_add(&pcie_cdev, devid, 255);

    pdev = device_create(pclass, NULL, devid, NULL, "picp");
    if (IS_ERR(pdev)) {
    	printk("picp: failed to allocate device\n");
        class_destroy(pclass);
     	unregister_chrdev_region(devid, 255);
        return -EBUSY;
    }

	dmamsk = DMA_BIT_MASK(32);
	pdev->dma_mask = &dmamsk;
	//set_dma_ops(pdev, &bst_dma_ops3);
	//set_dma_ops(pdev, &dma_noncoherent_ops);
	//set_dma_ops(pdev, &dma_direct_ops);
	pr_info("picp init completed.\n");
    return 0;
}

static void __exit picp_cdev_exit(void)
{
	picp_err("picp cdev exit\n");
	stop_picp_kthread();

	device_destroy(pclass, devid);
	cdev_del(&pcie_cdev);
	class_destroy(pclass);
	unregister_chrdev_region(devid, 255);
}

module_init(picp_cdev_init);
module_exit(picp_cdev_exit);

MODULE_VERSION("0.4");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GaoQingpeng@bst.ai");
MODULE_DESCRIPTION("PCIE Interconnection Control Protocol");


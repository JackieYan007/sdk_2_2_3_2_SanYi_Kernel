/*******************************************************************************
  PTP 1588 clock using the BSTGMAC.

  Copyright (C) 2013  Vayavya Labs Pvt Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
*******************************************************************************/
#include "bstgmac.h"
#include "dwmac_ptp.h"

//#define  DIVSEC 1000000000ULL

/**
 * bstgmac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
static int bstgmac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;
	struct bstptp_ctl *ctl = priv->ptpctl;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	if (ctl->multip) {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_config_addend(priv, ctl->ptp0_reg, addend);
		bstgmac_config_addend(priv, ctl->ptp1_reg, addend);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	} else {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_config_addend(priv, priv->ptpaddr, addend);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	return 0;
}

/**
 * bstgmac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int bstgmac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;
	bool xmac, est_rst = false;
	int ret;
	struct bstptp_ctl *ctl = priv->ptpctl;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	/* If EST is enabled, disabled it before adjust ptp time. */
	if (priv->plat->est && priv->plat->est->enable) {
		est_rst = true;
		mutex_lock(&priv->plat->est->lock);
		priv->plat->est->enable = false;
		bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
				     priv->plat->clk_ptp_rate);
		mutex_unlock(&priv->plat->est->lock);
	}

	if (ctl->multip) {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_adjust_systime(priv, ctl->ptp1_reg, sec, nsec, neg_adj, xmac);
		bstgmac_adjust_systime(priv, ctl->ptp0_reg, sec, nsec, neg_adj, xmac);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	} else {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		bstgmac_adjust_systime(priv, priv->ptpaddr, sec, nsec, neg_adj, xmac);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	/* Caculate new basetime and re-configured EST after PTP time adjust. */
	if (est_rst) {
		struct timespec64 current_time, time;
		ktime_t current_time_ns, basetime;
		u64 cycle_time;

		mutex_lock(&priv->plat->est->lock);
		priv->ptp_clock_ops.gettime64(&priv->ptp_clock_ops, &current_time);
		current_time_ns = timespec64_to_ktime(current_time);
		time.tv_nsec = priv->plat->est->btr_reserve[0];
		time.tv_sec = priv->plat->est->btr_reserve[1];
		basetime = timespec64_to_ktime(time);
		cycle_time = priv->plat->est->ctr[1] * NSEC_PER_SEC +
			     priv->plat->est->ctr[0];
		time = bstgmac_calc_tas_basetime(basetime,
						current_time_ns,
						cycle_time);

		priv->plat->est->btr[0] = (u32)time.tv_nsec;
		priv->plat->est->btr[1] = (u32)time.tv_sec;
		priv->plat->est->enable = true;
		ret = bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
					   priv->plat->clk_ptp_rate);
		mutex_unlock(&priv->plat->est->lock);
		if (ret)
			netdev_err(priv->dev, "failed to configure EST\n");
	}

	return 0;
}

/**
 * bstgmac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int bstgmac_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns = 0;

	if ((readl(priv->ptpaddr + PTP_TCR) & 1) == 0) {
		printk(KERN_ERR "ptp clock uninitialized\n");
		return -EINTR;
	}
	
	spin_lock_irqsave(&priv->ptp_lock, flags);
	bstgmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/**
 * bstgmac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int bstgmac_set_time(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	unsigned long flags;
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);

	struct bstptp_ctl *ctl = priv->ptpctl;

	if (ctl->ppsfix) {
		if (ctl->ppssta--) {
			/* Time setting will be done in interrupt */
			ctl->utc.tv_sec  = ts->tv_sec;
			ctl->utc.tv_nsec = ts->tv_nsec;
		} else {
			pr_debug( "Settime Failed, NO PPS\n");
			return -1;
		}
	} else {
		if (ctl->multip) {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, ctl->ptp0_reg, ts->tv_sec, ts->tv_nsec);
			bstgmac_init_systime(priv, ctl->ptp1_reg, ts->tv_sec, ts->tv_nsec);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		} else {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, priv->ptpaddr, ts->tv_sec, ts->tv_nsec);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		}
	}
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

	return 0;
}

void bstptp_extts_interrupt(int irq, struct bstgmac_priv *priv)
{
	u32 reg_value = 0, aux_nan, sta_nan, nan;
	unsigned long flags;
	struct bstptp_ctl *ctl = priv->ptpctl;
	//printk(KERN_ERR "bstptp_extts_interrupt\n");

	/* Timestamp Interrupt disabled */
	reg_value = readl(priv->ioaddr + 0xb4);
	reg_value = (reg_value & (~(1<<12))); 
	writel(reg_value, priv->ioaddr + 0xb4);
	aux_nan = readl(priv->ptpaddr + 0x48);
	//printk(KERN_ERR "*** %s:%d nan:%x\n", __func__, __LINE__, reg_value);

	sta_nan = readl(priv->ptpaddr + 0x0c);
	//printk(KERN_ERR "*** %s:%d nan:%x\n", __func__, __LINE__, reg_value);

	if (sta_nan >= aux_nan)
		nan = (sta_nan - aux_nan);// + 10800;
	else
		nan = (0x3B9AC9FF - aux_nan + sta_nan);// + 10800;
	//printk(KERN_ERR "*** %s:%d **:%d\n", __func__, __LINE__, nan);
	
	if (ctl->utc.tv_sec != 0) {
		ctl->utc.tv_sec = ctl->utc.tv_sec + 1;

		if (ctl->multip) {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, ctl->ptp0_reg, ctl->utc.tv_sec, nan);
			bstgmac_init_systime(priv, ctl->ptp1_reg, ctl->utc.tv_sec, nan);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		} else {
			spin_lock_irqsave(&priv->ptp_lock, flags);
			bstgmac_init_systime(priv, priv->ptpaddr, ctl->utc.tv_sec, nan);
			spin_unlock_irqrestore(&priv->ptp_lock, flags);
		}
		//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;

		ctl->utc.tv_sec  = 0;
		ctl->utc.tv_nsec = 0;
	}
	ctl->ppssta = 2;

	/* Get Timestamp_Status 0xb20*/
	reg_value = readl(priv->ptpaddr + 0x20);

	/* Auxiliary Snapshot FIFO Clear */
	reg_value = readl(priv->ptpaddr + 0x40);
	reg_value = reg_value | 1;
	writel(reg_value, priv->ptpaddr + 0x40);

	/* Timestamp Interrupt enable */
	reg_value = readl(priv->ioaddr + 0xb4);
	reg_value = (reg_value | (1<<12));
	writel(reg_value, priv->ioaddr + 0xb4);
}

static int bstptp_extts_config(struct bstgmac_priv *priv, struct ptp_extts_request extts)
{
	u32 reg_value = 0;
	void __iomem *ptp_reg = priv->ptpaddr;
	struct bstptp_ctl *ctl= priv->ptpctl;
	bool xmac;
	u32 sec_inc = 0;
	u64 temp = 0;
	struct timespec64 now;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	/* Timestamp Interrupt disabled */
	reg_value = readl(priv->ioaddr + 0xb4);
	reg_value = (reg_value & (~(1<<12)));
	writel(reg_value, priv->ioaddr + 0xb4);

	/* 初始化ptp配置 */
	reg_value = readl(ptp_reg + PTP_TCR);
	reg_value = reg_value | 0x203;
	writel(reg_value, ptp_reg + PTP_TCR);

	/* MAC_Sub_Second_Increment 0xb04*/
	/* program Sub Second Increment reg */
	bstgmac_config_sub_second_increment(priv,
			priv->ptpaddr, priv->plat->clk_ptp_rate,
			xmac, &sec_inc);
	temp = div_u64(1000000000ULL, sec_inc);

	/* Store sub second increment and flags for later use */
	priv->sub_second_inc = sec_inc;
	//priv->systime_flags = value;

	/* 开启外部中断 */
	if (extts.flags & PTP_ENABLE_FEATURE) {
		/* Auxiliary Snapshot 0 Enable */
		reg_value = BIT(4) | BIT(7);//EVB-IN00 EC-IN10
		writel(reg_value, ptp_reg + 0x40);

		/* Timestamp Interrupt enable */
		reg_value = readl(priv->ioaddr + 0xb4);
		reg_value = (reg_value | (1<<12));
		writel(reg_value, priv->ioaddr + 0xb4);
		ctl->extintr = 1;
		printk(KERN_ERR "bstptp config extts intr on\n");
	} else {
		/* Auxiliary Snapshot 0 disabled */
		writel(1, priv->ptpaddr + 0x40);
		/* Timestamp Interrupt disabled */
		reg_value = readl(priv->ioaddr + 0xb4);
		reg_value = (reg_value & (~(1<<12))); 
		writel(reg_value, priv->ioaddr + 0xb4);
		ctl->extintr = 0;
		printk(KERN_ERR "bstptp config extts intr off\n");
	}

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
	//priv->ptp_clock_ops.elapsed = ktime_get_boottime()/DIVSEC;
	//priv->ptp_clock_ops.status = PTP_STA_INITED;

	/* 开启gps-pps修正 */
	if (extts.rsv[0]) {
		ctl->ppsfix = 1;
		printk(KERN_ERR "bstptp config ppsfix on\n");
	} else {
		ctl->ppsfix = 0;
		printk(KERN_ERR "bstptp config ppsfix off\n");
	}

	/* 同时设置多个ptp(内部ptp同步) */
	if (extts.rsv[1]) {
		ctl->multip = 1;
		printk(KERN_ERR "bstptp config multi on\n");
	} else {
		ctl->multip = 0;
		printk(KERN_ERR "bstptp config multi off\n");
	}

	return 0;
}

static int bstptp_pps_config(void __iomem *ioaddr, int on)
{
	void __iomem *ptp_reg = ioaddr + PTP_GMAC4_OFFSET;
	
	if (on) {
		/* MAC_PPS_Control */
		writel(1, ptp_reg + 0x70);

		pr_debug("PPS OUT ON\n");
	} else {
		writel(0, ptp_reg + 0x70);
		pr_debug( "PPS OUT OFF\n");
	}

	return 0;
}

static int bstgmac_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	struct bstgmac_priv *priv =
	    container_of(ptp, struct bstgmac_priv, ptp_clock_ops);
	struct bstgmac_pps_cfg *cfg;
	int ret = -EOPNOTSUPP;
	unsigned long flags;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		//return -EOPNOTSUPP; //conflict with ifconfig eth up
		cfg = &priv->pps[rq->perout.index];

		cfg->start.tv_sec = rq->perout.start.sec;
		cfg->start.tv_nsec = rq->perout.start.nsec;
		cfg->period.tv_sec = rq->perout.period.sec;
		cfg->period.tv_nsec = rq->perout.period.nsec;

		spin_lock_irqsave(&priv->ptp_lock, flags);
		ret = bstgmac_flex_pps_config(priv, priv->ioaddr,
					     rq->perout.index, cfg, on,
					     priv->sub_second_inc,
					     priv->systime_flags);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		break;
	case PTP_CLK_REQ_EXTTS:
		ret = bstptp_extts_config(priv, rq->extts);
		break;
	case PTP_CLK_REQ_PPS:
		ret = bstptp_pps_config(priv->ioaddr, on);
		break;
	default:
		break;
	}

	return ret;
}

/* structure describing a PTP hardware clock */
static struct ptp_clock_info bstgmac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "bstmac_ptp_clock",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 1,
	.n_per_out = 0, /* will be overwritten in bstgmac_ptp_register */
	.n_pins = 0,
	.pps = 1,
	.adjfreq = bstgmac_adjust_freq,
	.adjtime = bstgmac_adjust_time,
	.gettime64 = bstgmac_get_time,
	.settime64 = bstgmac_set_time,
	.enable = bstgmac_enable,
};

/**
 * bstgmac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void bstgmac_ptp_register(struct bstgmac_priv *priv)
{
	int i;

	for (i = 0; i < priv->dma_cap.pps_out_num; i++) {
		if (i >= BSTGMAC_PPS_MAX)
			break;
		priv->pps[i].available = true;
	}

	bstgmac_ptp_clock_ops.n_per_out = priv->dma_cap.pps_out_num;

#if 0
	bstgmac_ptp_clock_ops.pin_config = kcalloc(bstgmac_ptp_clock_ops.n_pins,
			                          sizeof(*bstgmac_ptp_clock_ops.pin_config),GFP_KERNEL);
	if (!bstgmac_ptp_clock_ops.pin_config)
		return ;

	for (i = 0; i < bstgmac_ptp_clock_ops.n_pins; i++) {
		snprintf(bstgmac_ptp_clock_ops.pin_config[i].name,
			 sizeof(bstgmac_ptp_clock_ops.pin_config[i].name),
			 "ptp_extts%d", i);
		bstgmac_ptp_clock_ops.pin_config[i].index = i;
		bstgmac_ptp_clock_ops.pin_config[i].func = PTP_PF_EXTTS;
		bstgmac_ptp_clock_ops.pin_config[i].chan = i;
	}
#endif
	priv->ptpctl = (struct bstptp_ctl *)kmalloc(sizeof(struct bstptp_ctl), GFP_KERNEL);
	if (!priv->ptpctl) {
		netdev_err(priv->dev, "ptpctl kmalloc failed\n");
	}else {
		memset(priv->ptpctl, 0, sizeof(struct bstptp_ctl));	
		priv->ptpctl->ptp0_reg = ioremap(0x30000b00, 200);
		priv->ptpctl->ptp1_reg = ioremap(0x30100b00, 200);
	}
	
	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = bstgmac_ptp_clock_ops;

	priv->device->id = priv->plat->bus_id;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     priv->device);
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->dev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
	} else if (priv->ptp_clock)
		netdev_info(priv->dev, "registered PTP clock\n");
}

/**
 * bstgmac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void bstgmac_ptp_unregister(struct bstgmac_priv *priv)
{
	if (priv->ptpctl) {
		if (priv->ptpctl->ptp0_reg)
			iounmap(priv->ptpctl->ptp0_reg);
		if (priv->ptpctl->ptp1_reg)
			iounmap(priv->ptpctl->ptp1_reg);		
		kfree(priv->ptpctl);
	}

#if 0
	if(bstgmac_ptp_clock_ops.pin_config)
		kfree(bstgmac_ptp_clock_ops.pin_config);
#endif
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		pr_debug("Removed PTP HW clock successfully on %s\n",
			 priv->dev->name);
	}
}

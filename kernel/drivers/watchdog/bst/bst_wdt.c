/*
 * Watchdog driver for BST WDT
 * Based on a patch from dw_wdt.c
 * Copyright (C) 2010-2011 Picochip Ltd., Jamie Iles
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
 * Jan 2020: v1: Create wdt driver for the first time
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>

#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <bst/bst_a1000.h>
#include <linux/percpu.h>
#include <linux/completion.h>
#endif

#define WDOG_CONTROL_REG_OFFSET		        0x00
#define WDOG_CONTROL_REG_WDT_EN_MASK	    0x01
#define WDOG_CONTROL_REG_RESP_MODE_MASK	    0x02
#define WDOG_CONTROL_REG_PULSE_LEN_MASK     0x1c
#define WDOG_CONTROL_REG_PULSE_LEN_SHIFT    2
#define WDOG_TIMEOUT_RANGE_REG_OFFSET	    0x04
#define WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT    4
#define WDOG_CURRENT_COUNT_REG_OFFSET	    0x08
#define WDOG_COUNTER_RESTART_REG_OFFSET     0x0c
#define WDOG_COUNTER_RESTART_KICK_VALUE	    0x76
#define WDOG_EOI_REG_OFFSET		            0x14

#define BST_SEC_SAFE_SYS_CTRL_BASE_ADDR		0x70035000
#define BST_SEC_SAFE_REST_CTRL_OFFSET		0x8
#define BST_SEC_SAFE_REST_SEL_OFFSET		0xc
#define BST_SEC_SAFE_SW_RESET_SHIFT			0x0
#define BST_WDT0_RST_SHIFT					0x0
#define BST_WDT1_RST_SHIFT					0x1
#define BST_WDT2_RST_SHIFT					0x2
#define BST_WDT3_RST_SHIFT					0x3

#define BST_WDT0_NAME_STR					"lsp_wdt0"
#define BST_WDT1_NAME_STR					"lsp_wdt1"
#define BST_WDT2_NAME_STR					"lsp_wdt2"
#define BST_WDT3_NAME_STR					"lsp_wdt3"
#define BST_WDT4_NAME_STR					"a55_wdt0"
#define BST_WDT5_NAME_STR					"a55_wdt1"
#define BST_WDT6_NAME_STR					"a55_wdt2"
#define BST_WDT7_NAME_STR					"a55_wdt3"
#define BST_WDT8_NAME_STR					"a55_wdt4"
#define BST_WDT9_NAME_STR					"a55_wdt5"
#define BST_WDT10_NAME_STR					"a55_wdt6"
#define BST_WDT11_NAME_STR					"a55_wdt7"

#define BST_A55_WDT_DIV						4

enum reset_pluse_length{
    PCLK_CYCLES_2,
    PCLK_CYCLES_4,
    PCLK_CYCLES_8,
    PCLK_CYCLES_16,
    PCLK_CYCLES_32,
    PCLK_CYCLES_64,
    PCLK_CYCLES_128,
    PCLK_CYCLES_256,
};

/* The maximum TOP (timeout period) value that can be set in the watchdog. */
#define BST_WDT_MAX_TOP		15

#define BST_WDT_DEFAULT_SECONDS	60

#define BST_WDT_INTE_MODE    0x1

static bool nowayout = WATCHDOG_NOWAYOUT;

void __iomem *sec_sys_ctrl_base;

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
		 "(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct bst_wdt {
	void __iomem		*regs;
	struct clk		*wclk;
	struct clk		*pclk;
	unsigned long		rate;
	struct watchdog_device	wdd;
	struct reset_control	*rst;
    int irq;
	const char 		*name;
	/* Save/restore */
	u32			control;
	u32			timeout;
#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
	/* a55_wdt feed on kthread */
	u32 a55wdt_num;
	struct hrtimer feed_timer;
	/* kthread completion */
	struct completion wait_done;
#endif
};

static unsigned int bst_wdt_get_timeleft(struct watchdog_device *wdd);

#define to_bst_wdt(wdd)	container_of(wdd, struct bst_wdt, wdd)

static inline int bst_wdt_is_enabled(struct bst_wdt *bst_wdt)
{
	return readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET) &
		WDOG_CONTROL_REG_WDT_EN_MASK;
}

static inline int bst_wdt_top_in_seconds(struct bst_wdt *bst_wdt, unsigned top)
{
	/*
	 * There are 16 possible timeout values in 0..15 where the number of
	 * cycles is 2 ^ (16 + i) and the watchdog counts down.
	 */
	return (1U << (16 + top)) / bst_wdt->rate;
}

static int bst_wdt_get_top(struct bst_wdt *bst_wdt)
{
	int top = readl(bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET) & 0xF;

	return bst_wdt_top_in_seconds(bst_wdt, top);
}

#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
static enum hrtimer_restart bst_a55wdt_hrtimer_func(struct hrtimer *t)
{
	struct bst_wdt *bst_fwdt = container_of(t, struct bst_wdt, feed_timer);
	ktime_t now, m_kt;

	/* feed wdt */
	writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_fwdt->regs +
	       WDOG_COUNTER_RESTART_REG_OFFSET);
	
	m_kt = ktime_set(2, 0);
	now = hrtimer_cb_get_time(t);
	hrtimer_forward(t, now, m_kt);

	/* restart */
	return HRTIMER_RESTART;
}

static int feed_a55wdt_kthread(void *data)
{
	ktime_t m_kt;
	struct bst_wdt *bst_wdt = (struct bst_wdt *)data;

	hrtimer_init(&bst_wdt->feed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	bst_wdt->feed_timer.function = bst_a55wdt_hrtimer_func;

	/* two sec, zero nsec */
	m_kt = ktime_set(2, 0);
	hrtimer_start(&bst_wdt->feed_timer, m_kt, HRTIMER_MODE_REL_PINNED);

	complete(&bst_wdt->wait_done);

	return 0;
}

static DEFINE_PER_CPU(bool, bst_wdt_initialized) = false;

static int kthread_feed_wdt(struct bst_wdt *bst_wdt)
{
	struct task_struct *thread = NULL;
	
	/* already ping wdt, return */ 
	if (true == per_cpu(bst_wdt_initialized, bst_wdt->a55wdt_num)) {	
		return -EBUSY;
	}

	if (!cpu_active(bst_wdt->a55wdt_num)) {
		pr_err("Active cpus number %d, but cpu %d Inactive, don't create wdt kthread\n", num_active_cpus(), bst_wdt->a55wdt_num);
		return -EPERM;
	}

	init_completion(&bst_wdt->wait_done);

	thread = kthread_create_on_cpu(feed_a55wdt_kthread,
					   (void *)bst_wdt, bst_wdt->a55wdt_num,
					   bst_wdt->name);

	if (IS_ERR(thread)) {
		pr_err("Failed to create kthread on CPU %d\n", bst_wdt->a55wdt_num);
		return PTR_ERR(thread);
	}					   
		
	wake_up_process(thread);
	
	/* Wait until kthreadd is all set-up. */
	wait_for_completion(&bst_wdt->wait_done);

	per_cpu(bst_wdt_initialized, bst_wdt->a55wdt_num) = true;

	return 0;
}
#endif

static int bst_wdt_ping(struct watchdog_device *wdd)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	
#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
	if (strncmp(bst_wdt->name, BST_WDT4_NAME_STR, 3) == 0) {		
		/* get BST_WDT_NAME_STR last character as cpu num */
		bst_wdt->a55wdt_num = simple_strtoul((const char *)(bst_wdt->name + strlen(BST_WDT4_NAME_STR) - 1), NULL, 10);		
		kthread_feed_wdt(bst_wdt);
		return 0;
	}
#endif	
    
	writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs +
	       WDOG_COUNTER_RESTART_REG_OFFSET);
	return 0;
}

static int bst_wdt_set_timeout(struct watchdog_device *wdd, unsigned int top_s)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	int i, top_val = BST_WDT_MAX_TOP;

	/*
	 * Iterate over the timeout values until we find the closest match. We
	 * always look for >=.
	 */
	for (i = 0; i <= BST_WDT_MAX_TOP; ++i)
		if (bst_wdt_top_in_seconds(bst_wdt, i) >= top_s) {
			top_val = i;
			break;
		}

	/*
	 * Set the new value in the watchdog.  Some versions of bst_wdt
	 * have have TOPINIT in the TIMEOUT_RANGE register (as per
	 * CP_WDT_DUAL_TOP in WDT_COMP_PARAMS_1).  On those we
	 * effectively get a pat of the watchdog right here.
	 */
	writel(top_val | top_val << WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT,
	       bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
    writel(top_val, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);

	wdd->timeout = bst_wdt_top_in_seconds(bst_wdt, top_val);

	return 0;
}

static int bst_wdt_set_pulse(struct watchdog_device *wdd, unsigned int rpl)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	u32 val = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	
	val &= ~WDOG_CONTROL_REG_PULSE_LEN_MASK;
    val |= (rpl << WDOG_CONTROL_REG_PULSE_LEN_SHIFT);
	writel(val, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	return 0;
}


static void bst_wdt_arm_system_reset(struct bst_wdt *bst_wdt)
{
	u32 val = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	/* Enable watchdog. */
	val |= WDOG_CONTROL_REG_WDT_EN_MASK;
	writel(val, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
    writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs +
	       WDOG_COUNTER_RESTART_REG_OFFSET);
}

static int bst_wdt_start(struct watchdog_device *wdd)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);

#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
	if (strncmp(bst_wdt->name, BST_WDT4_NAME_STR, 3) == 0) {	
		/* over active_cpus_num watchdog shouldn't enable */
		bst_wdt->a55wdt_num = simple_strtoul((const char *)(bst_wdt->name + strlen(BST_WDT4_NAME_STR) - 1), NULL, 10);
		if (bst_wdt->a55wdt_num >= num_active_cpus()) {
			printk(KERN_ERR "when active cpus number %d, the a55_wdt number should less than %d, So don't start %s\n", num_active_cpus(), num_active_cpus(), bst_wdt->name);
			return -EINVAL;
		}			
	}
#endif

	bst_wdt_set_pulse(wdd, PCLK_CYCLES_32);
	bst_wdt_set_timeout(wdd, wdd->timeout);
	bst_wdt_arm_system_reset(bst_wdt);

	bst_wdt_ping(wdd);    
	return 0;
}

static int bst_wdt_stop(struct watchdog_device *wdd)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	
#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
	/* a55_wdt : cancel hrtimer */
	if (strncmp(bst_wdt->name, BST_WDT4_NAME_STR, 3) == 0) {
		if (true == per_cpu(bst_wdt_initialized, bst_wdt->a55wdt_num)) {
			hrtimer_cancel(&bst_wdt->feed_timer);
		}

		return 0;
	}
#endif

	if (!bst_wdt->rst) {
		set_bit(WDOG_HW_RUNNING, &wdd->status);
		return 0;
	}

	reset_control_assert(bst_wdt->rst);
	reset_control_deassert(bst_wdt->rst);

	return 0;
}

static int bst_wdt_restart(struct watchdog_device *wdd,
			  unsigned long action, void *data)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);

	writel(0, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
	if (bst_wdt_is_enabled(bst_wdt))
		writel(WDOG_COUNTER_RESTART_KICK_VALUE,
		       bst_wdt->regs + WDOG_COUNTER_RESTART_REG_OFFSET);
	else
		bst_wdt_arm_system_reset(bst_wdt);

	/* wait for reset to assert... */
	mdelay(500);

	return 0;
}

static unsigned int bst_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);

	return readl(bst_wdt->regs + WDOG_CURRENT_COUNT_REG_OFFSET) /
		bst_wdt->rate;
}

static void bst_wdt_irq_clear(void *dev_id)
{
	struct bst_wdt *bst_wdt = dev_id;

	readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
}

static irqreturn_t bst_wdt_irq_handle(int irq, void *dev_id)
{
    struct bst_wdt *bst_wdt = dev_id;
#ifdef CONFIG_BST_WATCHDOG_DEBUG  
	bst_wdt_irq_clear(dev_id);
	dev_info(bst_wdt->wdd.parent, "watchdog%d: irq trigger\r\n", bst_wdt->wdd.id);
#else
	/* reset bst a1000 soc */
	if(strncmp(bst_wdt->name, BST_WDT4_NAME_STR, 3) == 0){
		u32 val = 0;
		
#ifdef CONFIG_BST_AUTOFEED_A55_WATCHDOG
		static int irq_handled_flag[8] = {0,};

		/* get BST_WDT_NAME_STR last character as cpu num */
		bst_wdt->a55wdt_num = simple_strtoul((const char *)(bst_wdt->name + strlen(BST_WDT4_NAME_STR) - 1), NULL, 10);
		
		/* only send once wrong message */
		if (0 == irq_handled_flag[bst_wdt->a55wdt_num]) {
			send_safety_usrmsg(0xA00B01 + bst_wdt->a55wdt_num, 0x1);	/* a55_wdt timeout && IGNORE */
			irq_handled_flag[bst_wdt->a55wdt_num] = 1;
			dev_info(bst_wdt->wdd.parent, "watchdog%d: irq trigger	%s\r\n", bst_wdt->wdd.id, bst_wdt->name);
		}

		return IRQ_HANDLED;
#endif

		bst_wdt_irq_clear(dev_id);
		val = readl(sec_sys_ctrl_base + BST_SEC_SAFE_REST_CTRL_OFFSET);
		val &= (~BIT(BST_SEC_SAFE_SW_RESET_SHIFT));
		writel(val, sec_sys_ctrl_base + BST_SEC_SAFE_REST_CTRL_OFFSET);
		
	}
#endif	
    return IRQ_HANDLED;
}

#ifndef CONFIG_BST_WATCHDOG_DEBUG
static void bst_wdt_reset_sys_enable(struct bst_wdt *bst_wdt)
{
	u32 val = 0;
	const char *bst_wdt_name;

	bst_wdt_name = bst_wdt->name;
	
	/* enable wdt system restart */
    val = readl(sec_sys_ctrl_base + BST_SEC_SAFE_REST_SEL_OFFSET);
	if(strcmp(bst_wdt_name, BST_WDT0_NAME_STR) == 0){
		val &= (~BIT(BST_WDT0_RST_SHIFT)); //watchdog0
	}else if(strcmp(bst_wdt_name, BST_WDT1_NAME_STR) == 0){
		val &= (~BIT(BST_WDT1_RST_SHIFT)); //watchdog1
	}else if(strcmp(bst_wdt_name, BST_WDT2_NAME_STR) == 0){
		val &= (~BIT(BST_WDT2_RST_SHIFT)); //watchdog2
	}else if(strcmp(bst_wdt_name, BST_WDT3_NAME_STR) == 0){
		val &= (~BIT(BST_WDT3_RST_SHIFT)); //watchdog3
	}
    writel(val, sec_sys_ctrl_base + BST_SEC_SAFE_REST_SEL_OFFSET);
	
}
#endif

static const struct watchdog_info bst_wdt_ident = {
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_MAGICCLOSE,
	.identity	= "BST Watchdog",
};

static const struct watchdog_ops bst_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= bst_wdt_start,
	.stop		= bst_wdt_stop,
	.ping		= bst_wdt_ping,
	.set_timeout	= bst_wdt_set_timeout,
	.get_timeleft	= bst_wdt_get_timeleft,
	.restart	= bst_wdt_restart,
};

#ifdef CONFIG_PM_SLEEP
static int bst_wdt_suspend(struct device *dev)
{
	struct bst_wdt *bst_wdt = dev_get_drvdata(dev);

	bst_wdt->control = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	bst_wdt->timeout = readl(bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);

	clk_disable_unprepare(bst_wdt->wclk);
	clk_disable_unprepare(bst_wdt->pclk);

	return 0;
}

static int bst_wdt_resume(struct device *dev)
{
	struct bst_wdt *bst_wdt = dev_get_drvdata(dev);
	int err = clk_prepare_enable(bst_wdt->wclk);
    
	if (err)
		return err;
	
	err = clk_prepare_enable(bst_wdt->pclk);
	if (err)
		return err;

	writel(bst_wdt->timeout, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
	writel(bst_wdt->control, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

	bst_wdt_ping(&bst_wdt->wdd);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(bst_wdt_pm_ops, bst_wdt_suspend, bst_wdt_resume);

static int bst_wdt_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct bst_wdt *bst_wdt;
	struct resource *mem;
	int ret;
	u32 rmod;

	dev_info(dev, "wdt %s, %d\n",  __func__,__LINE__);

	sec_sys_ctrl_base = ioremap(BST_SEC_SAFE_SYS_CTRL_BASE_ADDR, 0x1000);
	
	bst_wdt = devm_kzalloc(dev, sizeof(*bst_wdt), GFP_KERNEL);
	if (!bst_wdt)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bst_wdt->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(bst_wdt->regs))
		return PTR_ERR(bst_wdt->regs);

	of_property_read_string(dev->of_node, "bst_wdt_name", &(bst_wdt->name));

	bst_wdt->wclk = devm_clk_get(dev, "wclk");
	if (IS_ERR(bst_wdt->wclk))
		return PTR_ERR(bst_wdt->wclk);
	
	ret = clk_prepare_enable(bst_wdt->wclk);
	if (ret)
		return ret;

	if(strncmp(bst_wdt->name, BST_WDT0_NAME_STR, 3) == 0){
		bst_wdt->pclk = devm_clk_get(dev, "pclk");
		if (IS_ERR(bst_wdt->pclk))
			return PTR_ERR(bst_wdt->pclk);

		ret = clk_prepare_enable(bst_wdt->pclk);
		if (ret)
			return ret;
	}

	if(strncmp(bst_wdt->name, BST_WDT0_NAME_STR, 3) == 0){
		bst_wdt->rate = clk_get_rate(bst_wdt->wclk);
	}else if(strncmp(bst_wdt->name, BST_WDT4_NAME_STR, 3) == 0){
		bst_wdt->rate = (clk_get_rate(bst_wdt->wclk) / BST_A55_WDT_DIV);
	}
	if (bst_wdt->rate == 0) {
		ret = -EINVAL;
		goto out_disable_clk;
	}

	/*xiangning.he 2020.4.23*/
	/*dev_info(dev, "wdt wclock frequency (bst_wdt->wclk):%ld\n", clk_get_rate(bst_wdt->wclk));
	dev_info(dev, "wdt pclock frequency (bst_wdt->pclk):%ld\n", clk_get_rate(bst_wdt->pclk));
	dev_info(dev, "wdt rate (bst_wdt->rate):%ld\n", bst_wdt->rate);*/
	
#ifdef CONFIG_BST_WATCHDOG_DEBUG  
	/* All use interrupt mode, can't reset CPU */
	bst_wdt->control = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	bst_wdt->control |= WDOG_CONTROL_REG_RESP_MODE_MASK;
	
	bst_wdt->irq = platform_get_irq(pdev, 0);
	if (bst_wdt->irq) {
		ret = devm_request_irq(dev, bst_wdt->irq, bst_wdt_irq_handle, 0, pdev->name, bst_wdt);
		if (ret < 0)
			dev_warn(dev, "failed to request IRQ\n");
	}
	writel(bst_wdt->control, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
#else
	bst_wdt->rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(bst_wdt->rst)) {
		ret = PTR_ERR(bst_wdt->rst);
		goto out_disable_clk;
	}
	
	reset_control_deassert(bst_wdt->rst);
	
	/* According to dts configuration */
	/* config response mode and request irq */
	bst_wdt_reset_sys_enable(bst_wdt);
	if(of_property_read_u32(dev->of_node, "response-mode", &rmod) == 0 ){
		bst_wdt->control = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

		if(BST_WDT_INTE_MODE == rmod){	  
			bst_wdt->control |= WDOG_CONTROL_REG_RESP_MODE_MASK;
			
			bst_wdt->irq = platform_get_irq(pdev, 0);
			if (bst_wdt->irq) {
				ret = devm_request_irq(dev, bst_wdt->irq, bst_wdt_irq_handle, 0, pdev->name, bst_wdt);
				if (ret < 0)
					dev_warn(dev, "failed to request IRQ\n");
			}
		} else
			bst_wdt->control &= ~WDOG_CONTROL_REG_RESP_MODE_MASK;		

		writel(bst_wdt->control, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	}
#endif

	wdd = &bst_wdt->wdd;
	wdd->info = &bst_wdt_ident;
	wdd->ops = &bst_wdt_ops;
	wdd->min_timeout = 1;
	wdd->max_hw_heartbeat_ms =
		bst_wdt_top_in_seconds(bst_wdt, BST_WDT_MAX_TOP) * 1000;
	wdd->parent = dev;

	watchdog_set_drvdata(wdd, bst_wdt);
	watchdog_set_nowayout(wdd, nowayout);
	watchdog_init_timeout(wdd, 0, dev);

	/*
	 * If the watchdog is already running, use its already configured
	 * timeout. Otherwise use the default or the value provided through
	 * devicetree.
	 */
	if (bst_wdt_is_enabled(bst_wdt)) {
		wdd->timeout = bst_wdt_get_top(bst_wdt);
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	} else {
		wdd->timeout = BST_WDT_DEFAULT_SECONDS;
		watchdog_init_timeout(wdd, 0, dev);
	}

	platform_set_drvdata(pdev, bst_wdt);

    watchdog_set_restart_priority(wdd, 128);

	ret = watchdog_register_device(wdd);
	if (ret)
		goto out_disable_clk;

	return 0;

out_disable_clk:
	iounmap(sec_sys_ctrl_base);
	clk_disable_unprepare(bst_wdt->pclk);
	clk_disable_unprepare(bst_wdt->wclk);
	return ret;
}

static int bst_wdt_drv_remove(struct platform_device *pdev)
{
	struct bst_wdt *bst_wdt = platform_get_drvdata(pdev);

	iounmap(sec_sys_ctrl_base);
	watchdog_unregister_device(&bst_wdt->wdd);
	reset_control_assert(bst_wdt->rst);
	clk_disable_unprepare(bst_wdt->pclk);
	clk_disable_unprepare(bst_wdt->wclk);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bst_wdt_of_match[] = {
	{ .compatible = "snps,dw-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_wdt_of_match);
#endif

static struct platform_driver bst_wdt_driver = {
	.probe		= bst_wdt_drv_probe,
	.remove		= bst_wdt_drv_remove,
	.driver		= {
		.name	= "bst_wdt",
		.of_match_table = of_match_ptr(bst_wdt_of_match),
		.pm	= &bst_wdt_pm_ops,
	},
};

module_platform_driver(bst_wdt_driver);

MODULE_AUTHOR("Xiangning He");
MODULE_DESCRIPTION("BST Watchdog Driver");
MODULE_LICENSE("GPL v2");

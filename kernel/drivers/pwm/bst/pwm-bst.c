/*
 * PWM driver for BST PWM
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
 * Jan 2020: v1: Create timer driver for the first time
 * Jan 2021: v2: pwm single channel management
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define TIMERNLOADCOUNT(_n) 		(0X00 + (_n) * 0X14)
#define TIMERNCURRENTVALUE(_n) 		(0X04 + (_n) * 0X14)
#define TIMERNCTROLREG(_n) 			(0X08 + (_n) * 0X14)
#define TIMERNEOI(_n) 				(0X0C + (_n) * 0X14)
#define TIMERNINTSTATUS(_n) 		(0X10 + (_n) * 0X14)
/*#define TIMERSINTSTATUS 			(0XA0)
#define TIMERSEOI 					(0XA4)
#define TIMERSRAWINTSTATUS 			(0XA8)
#define TIMERS_COMP_VERSI 			(0XAC)*/
#define TIMERNLOADCOUNT2_TOP(_n) 		(0X00 + (_n) * 0X04)	/* offset (0XB0 + (_n) * 0X04) */
/* #define TIMER_N_PROT_LEVEL(_n) 		(0XD0 + (_n) * 0X04) */

/* PWM is a function of the timer */
#define BST_DUTY_100_SHIFT			(0x4)
#define BST_PWM_ENABLE_SHIFT		(0x3)
#define BST_MASK_INT_SHIFT			(0x2)
#define BST_USER_MODE_SHIFT			(0x1)
#define BST_TIMER_ENABLE_SHIFT		(0x0)

#define BST_DEBUG             		(0)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

struct bst_pwm_chip {	
	void __iomem 	*base;
	void __iomem 	*top;	/* TIMERNLOADCOUNT2 BASE */
	struct clk		*wclk;	
	struct clk		*pclk;	
	struct pwm_chip chip;
};


static inline struct bst_pwm_chip *to_bst_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct bst_pwm_chip, chip);
}

static inline int bst_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_debug("pwmchip%u pwm%u request\n", chip->base, pwm->hwpwm);

	return 0;
}

static inline void bst_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	 pr_debug("pwmchip%u pwm%u free\n", chip->base, pwm->hwpwm);
}


static int bst_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int ret = 0;
	unsigned int ch_num = 0;
	unsigned int tcon =0;

	struct bst_pwm_chip *our_chip = to_bst_pwm_chip(chip);

	/* clear channel interrupt */
	for (ch_num=0; ch_num<pwm->hwpwm; ch_num++)
		readl(our_chip->base + TIMERNEOI(pwm->hwpwm));
	
	ret = clk_enable(our_chip->wclk);
	if (ret)
		return ret;
	
	tcon = readl(our_chip->base + TIMERNCTROLREG(pwm->hwpwm));
	tcon |= BIT(BST_TIMER_ENABLE_SHIFT) | BIT(BST_USER_MODE_SHIFT) | BIT(BST_MASK_INT_SHIFT) | BIT(BST_PWM_ENABLE_SHIFT);
	
	writel(tcon, our_chip->base + TIMERNCTROLREG(pwm->hwpwm));
	
	pr_debug("current_cnt:%d\n", readl(our_chip->base + TIMERNCURRENTVALUE(pwm->hwpwm)));

	return 0;
}

static void bst_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bst_pwm_chip *our_chip = to_bst_pwm_chip(chip);
	unsigned int tcon;

	tcon = readl(our_chip->base + TIMERNCTROLREG(pwm->hwpwm));
	tcon &= ~(BIT(BST_TIMER_ENABLE_SHIFT)|BIT(BST_PWM_ENABLE_SHIFT));
	writel(tcon, our_chip->base + TIMERNCTROLREG(pwm->hwpwm));
	clk_disable(our_chip->wclk);

	pr_debug("pwmchip%u pwm%u disable\n", chip->base, pwm->hwpwm);
}


static int bst_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct bst_pwm_chip *our_chip = to_bst_pwm_chip(chip);
	void __iomem *base = our_chip->base;
	void __iomem *top = our_chip->top;
	unsigned long long c;
	unsigned long period_cycles;
	unsigned long duty_cycles;
	int ret = 0;

	if (!pwm_is_enabled(pwm)) {
		ret = clk_enable(our_chip->wclk);
		if (ret)
			return ret;
	}

	/* calculation tick count */
	c = clk_get_rate(our_chip->wclk);	
	c *= period_ns;	
	do_div(c, 1000000000);
	period_cycles = c;	

	c = period_cycles;
	c *= duty_ns;	
	do_div(c, period_ns);
	duty_cycles = c;

	pr_debug("pwmchip%u pwm%u duty_cnt = %ld	 period_cnt = %ld\n", chip->base, pwm->hwpwm, duty_cycles, period_cycles);

	if (period_cycles < 0xffffffff && duty_cycles < 0xffffffff) {
		writel(period_cycles - duty_cycles, base + TIMERNLOADCOUNT(pwm->hwpwm));
		writel(duty_cycles, top + TIMERNLOADCOUNT2_TOP(pwm->hwpwm));
	} else {
		ret = -EINVAL;
	}

	if (!pwm_is_enabled(pwm))
		clk_disable(our_chip->wclk);

	return ret;	
}


static const struct pwm_ops bst_pwm_ops = {
	.request = bst_pwm_request,
	.free = bst_pwm_free,
	.config = bst_pwm_config,
	.enable = bst_pwm_enable,
	.disable = bst_pwm_disable,
	.owner = THIS_MODULE,
};

static int bst_pwm_probe(struct platform_device *pdev)
{
	struct bst_pwm_chip *bst_pwm;
	struct resource *res;
	unsigned pwm_chnum = 0;
	int ret;
	struct device *dev = &pdev->dev;

	dev_info(dev, "pwm-bst %s, %d\n",  __func__,__LINE__);

	bst_pwm = devm_kzalloc(&pdev->dev, sizeof(*bst_pwm), GFP_KERNEL);
	if (!bst_pwm)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	bst_pwm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bst_pwm->base))
		return PTR_ERR(bst_pwm->base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "top");
	bst_pwm->top = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bst_pwm->top))
		return PTR_ERR(bst_pwm->top);

	bst_pwm->wclk = devm_clk_get(&pdev->dev, "wclk");
	if (IS_ERR(bst_pwm->wclk)) {
		dev_err(dev, "failed to get timer work clk\n");
		return PTR_ERR(bst_pwm->wclk);
	}

	ret = clk_prepare_enable(bst_pwm->wclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable work clock\n");
		return ret;
	}

	/*get work clk*/
	bst_pwm->wclk = devm_clk_get(dev, "wclk");
	bst_pwm->pclk = devm_clk_get(dev, "pclk");
	dev_info(dev, "pwm wclock frequency (bst_pwm->wclk):%ld\n", clk_get_rate(bst_pwm->wclk));
	dev_info(dev, "pwm pclock frequency (bst_pwm->pclk):%ld\n", clk_get_rate(bst_pwm->pclk));

	/* get how mang channel of chip */
	if (device_property_read_u32_array(dev, "pwm-ch", &pwm_chnum, 1))	
		dev_err(dev, "missing/invalid timer-pwm ch\n");
	
	bst_pwm->chip.dev = &pdev->dev;
	bst_pwm->chip.ops = &bst_pwm_ops;	
	bst_pwm->chip.base = -1;			
	bst_pwm->chip.npwm = pwm_chnum;	

	platform_set_drvdata(pdev, bst_pwm);

	ret = pwmchip_add(&bst_pwm->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip\n");
		clk_disable_unprepare(bst_pwm->wclk);
		return ret;
	}
	
	return 0;
}

static int bst_pwm_remove(struct platform_device *pdev)
{
	struct bst_pwm_chip *bst_pwm = platform_get_drvdata(pdev);
	int ret = -1;
	
	ret = pwmchip_remove(&bst_pwm->chip);
	if (ret < 0)
		return ret;

	clk_disable_unprepare(bst_pwm->wclk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bst_pwm_of_match[] = {
	{ .compatible = "snps,bst-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_pwm_of_match);
#endif

static struct platform_driver bst_pwm_driver = {
	.driver = {
		.name = "bst_pwm",
		.of_match_table = of_match_ptr(bst_pwm_of_match),
	},
	.probe = bst_pwm_probe,
	.remove = bst_pwm_remove,
};
module_platform_driver(bst_pwm_driver);


MODULE_AUTHOR("xy");
MODULE_DESCRIPTION("BST Timer_Pwm Driver");
MODULE_LICENSE("GPL v2");
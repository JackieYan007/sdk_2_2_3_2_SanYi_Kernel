#include <linux/printk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/tee_interface.h>
#include <asm/delay.h>
#include <linux/moduleparam.h> 
#include <linux/device.h>
//#include <linux/dynamic_debug.h>
#include <linux/miscdevice.h>



//module_param(test_type, int, S_IRUSR);

#define BST_DEBUG             		(1)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

struct device	dev_jf;

#define DEVICE_NAME     "dynamic_dev_debug_test"
#define DEBUG_IOCTL_TYPE    	'x'
#define DEBUG_IOCTL_I2S_DEBUG  	_IO(DEBUG_IOCTL_TYPE, 1)
#define DEBUG_IOCTL_CLK_DBG  	_IO(DEBUG_IOCTL_TYPE, 2)
#define DEBUG_IOCTL_CLOCKSOURCE_DBG  	_IO(DEBUG_IOCTL_TYPE, 3)
#define DEBUG_IOCTL_DMA_DBG  	_IO(DEBUG_IOCTL_TYPE, 4)
#define DEBUG_IOCTL_I2C_DBG  	_IO(DEBUG_IOCTL_TYPE, 5)

#define DEBUG_CMD_MAX_NR		(5)

extern int  i2s_pr_debug_test(void);
extern int  clk_pr_debug_test(void);
extern int  clocksource_pr_debug_test(void);
extern int  dma_pr_debug_test(void);
extern int  i2c_pr_debug_test(void);


static void pr_debug_test(void)
{
	int a=1, b=2, c=0;

	c = a+b;
	bst_pr("/*********bst_pr: this is kernel_2.0 test file. follow is pr_debug********/\n");	
    pr_debug("/*******pr_debug:  test1!*******/\n");
    pr_debug("/*******pr_debug:  test2!*******/\n");
    pr_debug("/*******pr_debug:  test3!*******/\n");
    pr_debug("/*******pr_debug:  test4!*******/\n");
    pr_debug("/*******pr_debug:  test5!*******/\n");
	c = b-a;
}

static inline int dev_debug_open(struct inode* inode, struct file* file)
{
	bst_pr("dev_debug_open!\n");
	return 0;
}

static inline int dev_debug_close(struct inode* inode, struct file* file)
{
	bst_pr("dev_debug_close!\n");
	return 0;
}

static long unlocked_ioctl (struct file* files, unsigned int cmd, unsigned long arg)
{
	int ret=0;
	
	/* TYPE check */
	if ((_IOC_TYPE(cmd) != DEBUG_IOCTL_TYPE) || (_IOC_NR(cmd) > DEBUG_CMD_MAX_NR))
	{
		return -EINVAL;
	}

	switch(cmd)
		{
			case DEBUG_IOCTL_I2S_DEBUG:
				i2s_pr_debug_test();
				break;
			case DEBUG_IOCTL_CLK_DBG:
				clk_pr_debug_test();
				break;
			case DEBUG_IOCTL_CLOCKSOURCE_DBG:
				clocksource_pr_debug_test();
				break;	
			case DEBUG_IOCTL_DMA_DBG:
				dma_pr_debug_test();
				break;
			case DEBUG_IOCTL_I2C_DBG:
				i2c_pr_debug_test();
				break;			
			default:
				ret = -EINVAL;
				break;
		}
	return ret;
	
}


static struct file_operations debug_ops = {
	.owner = THIS_MODULE,
	.open  = dev_debug_open,
	.release = dev_debug_close,
	.unlocked_ioctl = unlocked_ioctl,
};


static struct miscdevice  misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = DEVICE_NAME,
	.fops  = &debug_ops,
};

#if 0
static int dynamic_init(void)
{
	int ret, a=1, b=2, c=0, d=0;
	c = a + b;
#if 1
	bst_pr("/*********bst_pr: this is kernel_2.0 test file********/\n");
    pr_debug("/*******pr_debug: dev_deb test1!*******/\n");
    pr_debug("/*******pr_debug: dev_deb test2!*******/\n");
    pr_debug("/*******pr_debug: dev_deb test3!*******/\n");
    pr_debug("/*******pr_debug: dev_deb test4!*******/\n");
    pr_debug("/*******pr_debug: dev_deb test5!*******/\n");
#endif
    
#if 1
		bst_pr("/*********bst_pr: this is kernel_2.0 test file. follow is dev_dbg...********/\n");
	
	dev_dbg(NULL,"null,/*******dev_dbg: dev_dbg test1!*******/");
    dev_dbg(NULL,"null,/*******dev_dbg: dev_dbg test2!*******/");
	dev_dbg(NULL,"null,/*******dev_dbg: dev_dbg test3!*******/");
	dev_dbg(NULL,"null,/*******dev_dbg: dev_dbg test4!*******/");
	dev_dbg(NULL,"null,/*******dev_dbg: dev_dbg test5!*******/");
#endif
   
    d = b - a;
}
#endif
 
static int __init debug_init(void)
{
    bst_pr("EXPORT_SYMBOL  Module one,Init!\n");
	misc_register(&misc);
    return 0;
}
 
static void __exit debug_exit(void)
{
	misc_deregister(&misc);
    bst_pr("EXPORT_SYMBOL  Module one,Exit!\n");
}
 
module_init(debug_init);
module_exit(debug_exit);
MODULE_LICENSE("GPL v2"); 

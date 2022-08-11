#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-conf.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/cdev.h>


#define KER_RW_R8      0
#define KER_RW_R16     1
#define KER_RW_R32     2

#define KER_RW_W8      3
#define KER_RW_W16     4
#define KER_RW_W32     5


static int reg_rw_major = 0;
static struct class *reg_rw_class;

static int kerrw_open(struct inode *inode, struct file *file)
{
	//pr_debug( "kerrw_open\n");
	return 0;
}

ssize_t ker_rw_read(struct file *file, char __user *ubuf, size_t size, loff_t *ppos)
{    

	volatile unsigned int   *p32;
	unsigned int val = 0;
	unsigned int addr;
    unsigned int ops;

	unsigned int buf[3];


	copy_from_user(buf, (const void __user *)ubuf, size);
    ops  = buf[0];
	addr = buf[1];
	val  = buf[2];
	
	p32  = (volatile unsigned int*)ioremap(addr, 4);

	switch (ops) {
		case KER_RW_R32: 
			val = *p32;
			copy_to_user((void __user *)(ubuf+8), &val, 4);
			break;
		case KER_RW_W32: 
			*p32 = val;
			break;
        default:
            break;
	}

	iounmap(p32);
	return size;
}


static long ker_rw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	volatile unsigned char  *p8;
	volatile unsigned short *p16;
	volatile unsigned int   *p32;
	unsigned int val;
	unsigned int addr;

	unsigned int buf[2];

    pr_debug( "%s:%d.\n", __func__, __LINE__);
	copy_from_user(buf, (const void __user *)arg, 8);
	addr = buf[0];
	val  = buf[1];

    pr_debug( "%s:%d, addr:0x%x.\n", __func__, __LINE__, buf[0]);
	
	p8  = (volatile unsigned char *)ioremap(addr, 4);
	p16 = (volatile unsigned short *)p8;
	p32 = (volatile unsigned int   *)p8;

	switch (cmd)
	{
		case KER_RW_R8:
		{
			val = *p8;
			copy_to_user((void __user *)(arg+4), &val, 4);
			break;
		}

		case KER_RW_R16:
		{
			val = *p16;
			copy_to_user((void __user *)(arg+4), &val, 4);
			break;
		}

		case KER_RW_R32:
		{
			val = *p32;
			copy_to_user((void __user *)(arg+4), &val, 4);
			break;
		}

		case KER_RW_W8:
		{
			*p8 = val;
			break;
		}

		case KER_RW_W16:
		{
			*p16 = val;
			break;
		}

		case KER_RW_W32:
		{
			*p32 = val;
			break;
		}
	}

	iounmap(p8);
	return 0;
}

static struct file_operations ker_rw_ops = {
	.owner   = THIS_MODULE,
    .open    = kerrw_open,
    .read    = ker_rw_read,
	.unlocked_ioctl   = ker_rw_ioctl,
	.compat_ioctl     = ker_rw_ioctl,
};

static struct cdev kerrw_cdev;
static struct class *kerrwcls;


static int ker_rw_init(void)
{
    dev_t devid;

	if (reg_rw_major) {
		devid = MKDEV(reg_rw_major, 0);
		register_chrdev_region(devid, 1, "ker_rw");  /* (major,0) 对应 clktest_fops, (major, 1~255)都不对应hello_fops */
	} else {
		alloc_chrdev_region(&devid, 0, 1, "ker_rw");
		reg_rw_major = MAJOR(devid);
	}
	
	cdev_init(&kerrw_cdev, &ker_rw_ops);
	cdev_add(&kerrw_cdev, devid, 1);

	kerrwcls = class_create(THIS_MODULE, "ker_rw");
    device_create(kerrwcls, NULL, MKDEV(reg_rw_major, 0), NULL, "ker_rw");

	pr_debug( "%s:%d.\n", __func__, __LINE__);
	return 0;
}

static void ker_rw_exit(void)
{
	class_destroy(reg_rw_class);
	unregister_chrdev(reg_rw_major, "ker_rw");
}

module_init(ker_rw_init);
module_exit(ker_rw_exit);


MODULE_LICENSE("GPL");


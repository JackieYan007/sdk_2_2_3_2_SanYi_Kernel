
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/string.h>
#include <linux/semaphore.h>

static int error_type = 0;
module_param(error_type, int, 0644);
MODULE_PARM_DESC(error_type, "error type");

static int test = 0;
module_param(test, int, 0644);
MODULE_PARM_DESC(test, "test type");

struct semaphore sem1;
struct semaphore sem2;



DEFINE_SPINLOCK(hack_spinB);
DEFINE_SPINLOCK(hack_spinA);

void hack_spinAB(void)
{
  printk("hack_lockdep:A->B\n");
  spin_lock(&hack_spinA);
  spin_lock(&hack_spinB);
}

void hack_spinBA(void)
{
  printk("hack_lockdep:B->A\n");
  spin_lock(&hack_spinB);
}


void sem1_recursion(void)
{
    down(&sem1);
    printk("down semaphore\n");
    sem1_recursion();
    up(&sem1);
}


char *stack_ptr_ret(void)
{
    char str[10] = "nihao";

    return &str[0];
}

extern int call_error_notifiers(unsigned long val, void *v);
static int __init panic_module_init(void)
{
    int i;
    char *p1;
    sema_init(&sem1, 1);

    if(test)
    {
        for(i =0 ;i<100;i++)
            call_error_notifiers(0x03,"test type\n");
    }else
    {
        switch(error_type){
            case 0:
                *(char *)0x00000000 = 'K';
                break;

            case 1:
                p1 = stack_ptr_ret();
                p1[0] = 'K';
                break;

            case 2:
                BUG_ON(1);
                break;

            case 3:
                WARN_ONCE(1,"TEST WARNING");
                break;

            case 4:
                sem1_recursion();
                break;

            case 5:
                printk("al: lockdep error test init\n");
                hack_spinAB();
                hack_spinBA();
                break;
        }
    }

    return 0;
}

static void __exit panic_module_exit(void)
{
    printk("netlink_kernel_exit\n");
}

module_init(panic_module_init);
module_exit(panic_module_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("chengyu.yang@bst.ai");
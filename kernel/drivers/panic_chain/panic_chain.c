#include "bst_error_handle.h"
#include <linux/notifier.h>

char *pErrorData;

struct miscchain_dev{
#define MISCCHAIN_NAME      "panic_chain"
#define MISCCHAIN_MINOR     144         
    struct fasync_struct *async_queue;
}miscchain;

static BLOCKING_NOTIFIER_HEAD(error_chain);

int call_error_notifiers(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&error_chain, val, v);
}
EXPORT_SYMBOL(call_error_notifiers);

int register_error_notifier(struct notifier_block *nb)
{
	int err;

	err = blocking_notifier_chain_register(&error_chain, nb);

	if(err)
		goto out;
 
out:
	return err;
}
EXPORT_SYMBOL(register_error_notifier);

int unregister_error_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&error_chain, nb);
}
EXPORT_SYMBOL(unregister_error_notifier);

unsigned int handle_status = 0;
int error_init_event(struct notifier_block *nb, unsigned long event,void *data)
{
    if(handle_status)
        return NOTIFY_DONE;

    handle_status = 1;
	switch(event){
    case EVENT_WARN:
        bst_warn_handle(data);
		break;
    case EVENT_DIE:
        bst_die_handle(data);
		break;
    case EVENT_USER_FAULTS:
        bst_user_faults_handle(data);
        break;
	case EVENT_TEST:
		bst_test_handle(data);
		break;
    case EVENT_PANIC:
        bst_panic_handle(data);
		break;
	default:
		break;
	}

    handle_status = 0;
	return NOTIFY_DONE;
}

/* 定义一个通知块 */
static struct notifier_block error_init_notifier = {
	.notifier_call = error_init_event,
};


int send_usrmsg(char *pbuf, uint16_t len)
{
    if(miscchain.async_queue)
	    kill_fasync(&miscchain.async_queue, SIGIO, POLL_IN);
    else
        pr_err("miscchain.async_queue NULL\n");
    
      return 0;
}

static int miscchain_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &miscchain; /* 设置私有数据 */

    return 0;
}

static int miscchain_fasync(int fd, struct file *filp, int on)
{
    struct miscchain_dev *dev = (struct miscchain_dev *)filp->private_data;

    return fasync_helper(fd, filp, on, &dev->async_queue);
}

DEFINE_MUTEX(read_mutex);
static ssize_t miscchain_read(struct file *filp, 
                    char __user *buf,size_t cnt, loff_t *offt)
{
    int ret;

    ret = copy_to_user(buf,pErrorData,ERROR_DATA_SIZE);
    return ret;
}

int miscchain_release (struct inode *inode, struct file *filp)
{
    return miscchain_fasync(-1, filp, 0);
}

static struct file_operations miscchain_fops = {
    .owner = THIS_MODULE,
    .open = miscchain_open,
    .read = miscchain_read,
    .fasync = miscchain_fasync,
    .release = miscchain_release,
};

static struct miscdevice misc_miscdev = {
    .minor = MISCCHAIN_MINOR,
    .name = MISCCHAIN_NAME,
    .fops = &miscchain_fops,
};

static int __init miscchain_init(void)
{
    int ret;

    register_error_notifier(&error_init_notifier);
    
    ret = misc_register(&misc_miscdev);
    if(ret < 0){
        pr_err("misc device register failed!\r\n");
        return -EFAULT;
    }

    pErrorData = (char *)kmalloc(ERROR_DATA_SIZE*sizeof(char),GFP_KERNEL);

    if(!pErrorData){
        pr_err("kmalloc error \r\n");
        return -ENOMEM;
    }

    return 0;
}

static void __exit miscchain_exit(void)
{
    /* unregister error notifier*/
    unregister_error_notifier(&error_init_notifier);
    misc_deregister(&misc_miscdev);
    kfree(pErrorData);
}

module_init(miscchain_init);
module_exit(miscchain_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("chengyu.yang@bst.ai");

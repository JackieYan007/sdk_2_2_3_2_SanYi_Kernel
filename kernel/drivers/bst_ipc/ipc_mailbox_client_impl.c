#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>

#include "ipc_mailbox_client.h"
#include "ipc_common.h"
#include "ipc_communication_manager.h"
#include "ipc_nodemanager.h"
#include "ipc_mailbox_controller.h"

#define IPC_DRIVER_NAME "ipc_mailbox_client"

/********************* extern global variables *******************/
extern struct ipc_client_info * client_map[IPC_CORE_MAX];

/********************* structures *******************/
struct ipc_driver {
	struct device *dev;
	struct miscdevice miscdev;
};

/********************* function declaration *******************/
static struct platform_device * g_pdev;

static long bst_ipc_client_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
{
	int32_t ret = 0;

	IPC_LOG_INFO("%s, cmd: 0x%x", "enter", cmd);
	switch(cmd){
		default:
			ret = -EINVAL;
			break;
	}

	IPC_LOG_INFO("%s, ret: %d", "exit", ret);
	return ret;
}

static int32_t bst_ipc_client_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int32_t ret = 0;

	IPC_LOG_INFO("%s, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx", "enter",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      vma->vm_end - vma->vm_start,
			      pgprot_writecombine(vma->vm_page_prot)); //pgprot_writecombine():MT_NORMAL_NC,no_cache

	IPC_LOG_INFO("%s, ret: %d", "exit", ret);
	return ret;
}

static int32_t bst_ipc_client_open(struct inode *inode, struct file *filp)
{
	IPC_LOG_DEBUG("%s", "enter");

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t bst_ipc_client_close(struct inode *inode, struct file *filp)
{
	IPC_LOG_DEBUG("%s", "enter");

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static const struct file_operations bst_ipc_client_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.unlocked_ioctl = bst_ipc_client_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bst_ipc_client_ioctl,
#endif
	.mmap    = bst_ipc_client_mmap,
	.open    = bst_ipc_client_open,
	.release = bst_ipc_client_close,
};

static const struct miscdevice bst_ipc_client_misc_base = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops  = &bst_ipc_client_fops,
};


static int32_t bst_ipc_mbox_client_probe(struct platform_device *pdev)
{
    int32_t ret    = 0;
        g_pdev = pdev;

	const int32_t dev_name_len = 10;
	char dev_name[dev_name_len];
	struct ipc_driver *ipc_driver = NULL;

		//create bstn device
	ipc_driver = devm_kzalloc(&pdev->dev, sizeof(struct ipc_driver), GFP_KERNEL);
	if (!ipc_driver) {
		IPC_DEV_ERR(&pdev->dev, "no enough memory!\n");
		return -ENOMEM;
	}

	//init ipc_driver device
	ipc_driver->dev = &pdev->dev;
	platform_set_drvdata(pdev, ipc_driver);

	memset(dev_name, 0, sizeof(dev_name));
	//init raw session for handshake

	//init bstn miscdev
	sprintf(dev_name, "ipccl%d",0);

	IPC_LOG_INFO("ipc_driver struct ptr addr: 0x%px", ipc_driver);
	IPC_LOG_INFO("dev_name: %s", dev_name);

	memcpy(&ipc_driver->miscdev, &bst_ipc_client_misc_base, sizeof(struct miscdevice));
	ipc_driver->miscdev.name     = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);
	ipc_driver->miscdev.nodename = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);


	//regist miscdev
	ret = misc_register(&ipc_driver->miscdev);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "misc_register(&ipc_driver->miscdev) , ret %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "device[%px] %s is registed.\n", pdev->dev, dev_name);

	IPC_LOG_DEBUG("exit");

	return ret;
}


static int32_t bst_ipc_client_mbox_remove(struct platform_device *pdev)
{

	IPC_LOG_DEBUG("enter");

	IPC_LOG_DEBUG("exit");
	return 0;
}

static const struct of_device_id bstn_mbox_of_match[] = {
	{ .compatible = "bst,ipc-mbox-client", },
	{},
};

MODULE_DEVICE_TABLE(of, bstn_mbox_of_match);

static struct platform_driver bst_ipc_client_mbox_driver = {
	.driver = {
		.name           = "ipc-mbox-client",
		.of_match_table = bstn_mbox_of_match,
	},
	.probe  = bst_ipc_mbox_client_probe,
	.remove = bst_ipc_client_mbox_remove,
};

static int32_t __init bst_ipc_client_mbox_init(void)
{
	return platform_driver_register(&bst_ipc_client_mbox_driver);
}
// subsys_initcall(bst_ipc_client_mbox_init);
late_initcall(bst_ipc_client_mbox_init);


MODULE_AUTHOR("Wenjian Gou");
MODULE_DESCRIPTION("IPC");
MODULE_LICENSE("GPL");

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#include "ipc_communication_manager.h"
#include "ipc_common.h"
#include "ipc_mailbox_controller.h"
#include "ipc_regs.h"
#include "user_head.h"
#include "ipc_msg_manager.h"
#include "ipc_session.h"
#include <linux/ipc_interface.h>
#include "ipc_nodemanager.h"

#include "ipc_top_interface.h"

/********************* structures *******************/
enum ipc_cmds{
	IPC_CMD_ADD = IPC_CMD_RESERVED_MAX,
	IPC_CMD_SUB ,
	IPC_CMD_APU_PRINT,
};


/********************* macros *******************/
#define IPC_DRIVER_NAME		"apu"
#define APU_NR          	8
#define APU0_DRIVER_NAME	"apu0"
#define APU1_DRIVER_NAME	"apu1"

/********************* extern global variables *******************/

/********************* global variables **************************/
// struct ipc_client_info* a55_client_info[APU_NR];

/********************* local variables ***************************/
static int64_t session_id;	// apu0 --> apu1
static int64_t session_id1to0;	// apu0 --> apu1
static struct task_struct *apu1_send_thread;
static struct task_struct *apu0_recv_thread;
static struct miscdevice s_miscdev;
static struct platform_device *apu_pdev;

static int32_t apu1_send_func(void *arg);

static const struct of_device_id apu0_of_match[] = {
	{
		.compatible = "bst,arm0",
	},
};

MODULE_DEVICE_TABLE(of, apu0_of_match);

static const struct of_device_id apu1_of_match[] = {
	{
		.compatible = "bst,arm1",
	},
};

MODULE_DEVICE_TABLE(of, apu1_of_match);

static int32_t apu0_open(struct inode *inode, struct file *filp)
{
	IPC_LOG_DEBUG("enter");
	//wake_up_process(apu0_recv_thread);
	apu1_send_func(NULL);
	//wake_up_process(apu1_send_thread);
	
	IPC_LOG_DEBUG("exit");
	return 0;
}

static int32_t apu0_close(struct inode *inode, struct file *filp)
{
	IPC_LOG_DEBUG("enter");

	IPC_LOG_DEBUG("exit");
	return 0;
}

static const struct file_operations apu0_fops = {
	.owner  = THIS_MODULE,
	.open = apu0_open,
	.release = apu0_close,
};

static const struct miscdevice apu0_misc_base = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &apu0_fops,
};

static void deal_each_session_of_core(struct ipc_session *session , void* args)
{
	// struct ipc_drv_msg*  ipc_drv_msg =(struct ipc_drv_msg* )args;
	// ipc_session_msg_in(session->id, ipc_drv_msg);
}

static int32_t apu_probe_common(struct platform_device *pdev,int32_t cpu_id)
{
	int32_t ret = 0;
	IPC_LOG_INFO("enter, cpu_id = %d, pdev-name = %s", cpu_id, pdev->name);

	IPC_LOG_DEBUG("%s", "exit");
	return ret;
}
#ifdef ENABLE_APU_SAMPLE

int32_t cnt = 0;
int64_t num_sum = 0;

void cb_func_addr(int32_t session_id, ipc_msg *re_msg)
{
	int32_t ret = ipc_recv_reply(session_id, re_msg, 300);

	num_sum += re_msg->data;
}

// async_func_calllback cb_func = cb_func_addr;

static int32_t apu1_send_func(void *arg)
{
	int32_t ret;
	int32_t send_cnt = 1;
	struct platform_device *pdev;
	pdev = (struct platform_device *)arg;
	int64_t num_count = 0;
	struct _usr_async_msg async_msg; 

	IPC_LOG_DEBUG("enter");

	session_id = ipc_init(IPC_CORE_R5_0, IPC_CORE_ARM5, &apu_pdev->dev);
	if (session_id<0) {
		IPC_DEV_ERR(&pdev->dev, "ipc_init(IPC_CORE_ARM1) failed, ret = %d", session_id);
		return ret;
	}
	IPC_LOG_INFO("session_id = %d", session_id);
	struct __kernel_timex txc;
	struct rtc_time tm;

	struct timespec64 ts;
	ktime_get_real_ts64(&ts);
	txc.time.tv_sec = ts.tv_sec;
	txc.time.tv_usec = ts.tv_nsec / 1000;
	//do_gettimeofday(&(txc.time));
	
	rtc_time64_to_tm(txc.time.tv_sec,&tm);
	//rtc_time_to_tm(txc.time.tv_sec,&tm);
	printk("begin UTC time :%d-%d-%d %d:%d:%d \n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);

	while (send_cnt--) {
		// ipc_msg msg = {
		// 	.type = IPC_MSG_TYPE_SIGNAL,
		// 	.cmd = IPC_CMD_ADD,
		// 	.data =send_cnt,
		// };
				
		
		// // NOTE: ipc_send and ipc_recv can be in another thread
		// ret = ipc_send(session_id, &msg, 0);
		// if (ret < 0) {
		// 	IPC_LOG_ERR(
		// 		    "ipc_send(IPC_CORE_ARM1) failed, ret = %d",
		// 		    ret);
		// 	// ipc_close(session_id);
		// 	// return ret;
		// }
		// // IPC_LOG_INFO("token of send message is %d", msg.token);
		// // IPC_LOG_INFO("data of send message is 0x%x", msg.data);

		// ret = ipc_recv(session_id, &msg, -1);
		// if (ret < 0) {
		// 	IPC_LOG_ERR(
		// 		"ipc_recv_method(IPC_CORE_ARM1) failed, ret = %d",
		// 		ret);
		// 	ipc_close(session_id);
		// 	return ret;
		// }
		// num_count += msg.data;
		// IPC_LOG_INFO("data of recv message is 0x%x", msg.data);		
		// msleep(1000);

		// async testing begin
		async_msg.send_msg.type = IPC_MSG_TYPE_METHOD;
		async_msg.send_msg.cmd = IPC_CMD_ADD;
		async_msg.send_msg.data = send_cnt;
		async_msg.send_msg.userdata = (uint64_t)cb_func;
		async_msg.session_id = session_id;

		ret = ipc_async_send_msg(&async_msg);
		if (ret < 0)
		{
			IPC_LOG_ERR("async send msg failed!");
			ipc_close(session_id);
			return -1;
		}
		msleep(100);
	}

	ktime_get_real_ts64(&ts);
	txc.time.tv_sec = ts.tv_sec;
	txc.time.tv_usec = ts.tv_nsec / 1000;
	//do_gettimeofday(&(txc.time));

	rtc_time64_to_tm(txc.time.tv_sec,&tm);
	//rtc_time_to_tm(txc.time.tv_sec,&tm);
	printk("end   UTC time :%d-%d-%d %d:%d:%d \n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
	// printk("sum = %ld\n",num_count);
	printk("sum = %lld\n", num_sum);

	ipc_close(session_id);
	IPC_LOG_DEBUG("exit");

	return ret;
}

static int32_t apu0_recv_func(void *arg)
{
	int32_t ret;
	ipc_msg msg;
	struct platform_device *pdev;

	IPC_LOG_DEBUG("enter");
	pdev = (struct platform_device *)arg;
	if(apu1_send_thread)
		wake_up_process(apu1_send_thread); // important

	while (!kthread_should_stop())
	{
		// NOTE: ipc_send and ipc_recv can be in another thread
		ret = ipc_recv(session_id, &msg, -1);
		if (ret == IPC_SEND_ERR_INVALID_PARAM) {

			IPC_LOG_INFO("invalid_param");
		}
		else if (ret == 0)
		{
			IPC_LOG_INFO("data of recv message is 0x%x", msg.data);
		}
		else
		{
			IPC_DEV_ERR(
				&pdev->dev,
				"ipc_recv_method(IPC_CORE_ARM1) failed, ret = %d",
				ret);
		}
		/*switch (msg.cmd) {
		case IPC_CMD_ADD:
			IPC_LOG_INFO("recv cmd IPC_CMD_ADD");
			msg.data += 1;
			break;

		default:
			break;
		}

		ret = ipc_send(session_id, &msg, 0);
		if (ret < 0) {
			IPC_DEV_ERR(
				&pdev->dev,
				"ipc_send(IPC_CORE_ARM1) failed, ret = %d",
				ret);
			return -1;
		}
		IPC_LOG_INFO("data of send message is 0x%x", msg.data);
		IPC_LOG_INFO("token of send message is %d", msg.token);
		*/
	}
	IPC_LOG_DEBUG("exit");
	return ret;
}
#endif

static int32_t apu0_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	char dev_name[20] = {0};

	IPC_LOG_DEBUG("%s", "enter");
	ret = apu_probe_common(pdev, 0);
	if(ret<0)
	{
		IPC_DEV_ERR(&pdev->dev, "apu_probe_common failed, ret %d\n", ret);
		return ret;
	}
	#ifdef ENABLE_APU_SAMPLE
	//session_id = ipc_init(IPC_CORE_ARM0, IPC_CORE_ARM0, &pdev->dev);
	//if (session_id<0) {
	//	IPC_DEV_ERR(&pdev->dev, "ipc_init(IPC_CORE_ARM1) failed, ret = %d", session_id);
	//	return -1;
	//}
	//IPC_LOG_INFO("session_id = %d", session_id);

	//session_id1to0 = ipc_init(IPC_CORE_ARM0, IPC_CORE_ARM0, &pdev->dev);
	//if (session_id1to0<0) {
	//	IPC_DEV_ERR(&pdev->dev, "ipc_init(IPC_CORE_ARM0) failed, ret = %d", session_id1to0);
	//	return -1;
	//}

	// IPC_LOG_INFO("session_id1to0 = %d", session_id1to0);
	apu_pdev = pdev;

	apu0_recv_thread = kthread_create(apu0_recv_func, pdev, "apu0_recv_thread");
	if (IS_ERR(apu0_recv_thread)) {
		IPC_DEV_ERR(&pdev->dev,"Failed to create apu0_recv_thread thread");
		return PTR_ERR(apu0_recv_thread);
	}

	sprintf(dev_name, "ipc-test%d", 0);
	IPC_LOG_INFO("dev_name: %s", dev_name);

	memcpy(&s_miscdev, &apu0_misc_base, sizeof(struct miscdevice));
	s_miscdev.name = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);
	s_miscdev.nodename = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);

	//register miscdev
	ret = misc_register(&s_miscdev);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "misc_register(&s_miscdev) , ret %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "device %s is registed.\n", dev_name);
	#endif
	IPC_LOG_DEBUG("%s", "exit");
	return ret;
}

static int32_t apu1_probe(struct platform_device *pdev)
{
	int32_t ret;

	IPC_LOG_DEBUG("%s", "enter");

	ret = apu_probe_common(pdev, 1);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "apu_probe_common failed, ret %d\n",
			    ret);
		return ret;
	}
	#ifdef ENABLE_APU_SAMPLE

	apu1_send_thread = kthread_create(apu1_send_func, pdev, "apu1_send_thread");
	if (IS_ERR(apu1_send_thread)) {
		IPC_DEV_ERR(&pdev->dev,"Failed to create apu1_send_thread thread");
		return PTR_ERR(apu1_send_thread);
	}
	#endif

	IPC_LOG_DEBUG("%s", "exit");
	return ret;
}




static int32_t apu_remove_common(struct platform_device *pdev)
{
	IPC_LOG_DEBUG("%s", "enter");
	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static struct platform_driver apu0_driver = {
	.probe  = apu0_probe,
	.remove = apu_remove_common,
	.driver = {
		.name           = APU0_DRIVER_NAME,
		.of_match_table = of_match_ptr(apu0_of_match),
		//.pm = &apu0_pm_ops,
	},
};

static struct platform_driver apu1_driver = {
	.probe  = apu1_probe,
	.remove = apu_remove_common,
	.driver = {
		.name           = APU1_DRIVER_NAME,
		.of_match_table = of_match_ptr(apu1_of_match),
		//.pm = &apu0_pm_ops,
	},
};

static int32_t __init apu0_init(void)
{
	return platform_driver_register(&apu0_driver);
}

static int32_t __init apu1_init(void)
{
	return platform_driver_register(&apu1_driver);
}

// module_platform_driver(apu0_driver);
// module_platform_driver(apu1_driver);

late_initcall_sync(apu0_init);
late_initcall_sync(apu1_init);

MODULE_AUTHOR("Wenjian Gou");
MODULE_DESCRIPTION("IPC");
MODULE_LICENSE("GPL");

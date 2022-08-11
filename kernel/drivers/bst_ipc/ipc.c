#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm/memory.h>
#include <linux/mman.h>

#include <linux/ipc_interface.h>

#include "ipc.h"
#include "ipc_mempool.h"
#include "ipc_session.h"
#include "ipc_msg_manager.h"
#include "ipc_mailbox_controller.h"
#include "ipc_nodemanager.h"
#include "user_head.h"
#include "ipc_mempool.h"


/********************* macros *******************/
#define IPC_INDEX_SIZE 50
#define IPC_DRIVER_NAME "ipc"

/********************* extern global variables *******************/
//this addr need remove
extern struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr;
extern struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr_uaddr;
extern struct ipc_memblock *g_ipc_memblock;
extern struct diag_info diagnose_info[SESSION_NUM];
#ifdef MSG_DUMP
extern bool save_flag[IPC_CORE_MAX];
extern uint32_t save_cnt[IPC_CORE_MAX];
#endif

/********************* local variables ***************************/
struct platform_device *g_ipc_main_pdev;

/********************* global variables ***************************/


/********************* function declaration ***************************/
static int32_t ipc_mempool_init(struct bstipc *bstipc)
{
	int32_t ret = 0;
	ret = of_reserved_mem_device_init(bstipc->dev);
	if (ret < 0) {
		IPC_LOG_ERR( "of_reserved_mem_device_init fail, ret: %d\n", ret);
		return -ENODEV;
	}
	return ipc_init_cma_mempool(&bstipc->pool, bstipc->dev);
}

//ipc memory alloc, this function is deprecated now
static int32_t ipc_ioctl_alloc(struct file *filp, struct ipc_buffer __user *p)
{
#if 0
	struct bstipc* bstipc = NULL;
	struct ipc_buffer ipc_buffer;
	int32_t ret = 0;
    struct ipc_memblock * memblock = NULL;

	IPC_LOG_INFO("%s, user ptr: %px", "enter", p);

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_WARNING("can not find bstipc dev!\n");
		return -EFAULT;
	}

	if (copy_from_user(&ipc_buffer, p, sizeof(*p))) {
		IPC_LOG_WARNING("copy_from_user fail!\n");
		return -EFAULT;
	}
	IPC_LOG_INFO("ipc_buffer: size = 0x%x, align = 0x%x",
		ipc_buffer.size, ipc_buffer.align);

	ret = bst_alloc(&ipc_buffer, true);
	if (ret) {
		IPC_LOG_WARNING( "bst_alloc fail!\n");
		return ret;
	}

	if (copy_to_user(p, &ipc_buffer, sizeof(*p))) {
		bst_free(ipc_buffer.phy_addr.ptr_64, true);
		IPC_LOG_WARNING("copy_to_user fail!\n");
		return -EFAULT;
	}
#endif
	return 0;
}

static int32_t ipc_ioctl_free(struct file *filp, struct ipc_buffer __user *p)
{

	struct ipc_buffer ipc_buffer;
	struct ipc_memblock *memblock = NULL;
	struct bstipc* bstipc = NULL;
	int32_t ret = 0;
#if 0
	IPC_LOG_INFO("%s, user ptr: %px", "enter", p);

	if (copy_from_user(&ipc_buffer, p, sizeof(*p)))
		return -EFAULT;

	ret = bst_free(ipc_buffer.phy_addr.ptr_64, true);
	if(ret<0)
	{
		return ret;
	}
#endif
	return ret;
}

static int32_t ipc_ioctl_bind_cpu(struct file *filp, uint32_t __user *p)
{
	return 0;
}

//sync call function
static int32_t ipc_ioctl_register_info(struct file *filp, struct msg_subscribe __user *p)
{
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct ipc_drv_msg* ipc_drv_msg = NULL;
	struct msg_subscribe info;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy register info
	if (copy_from_user(&info, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("register type = %d, session_id = %d, cmd= %d", info.type, info.session_id, info.cmd);

	if (info.type == IPC_MSG_TYPE_METHOD)
	{
		ret = ipc_method_register(info.session_id, info.cmd);
		if (ret < 0)
		{
			IPC_LOG_WARNING("session %d register %d method fail", info.session_id, info.cmd);
			return -1;
		}
	}
	else if (info.type == IPC_MSG_TYPE_SIGNAL)
	{
		ret = ipc_signal_subscribe(info.session_id, info.cmd);
		if (ret < 0)
		{
			IPC_LOG_WARNING("session %d subscribe %d signal fail", info.session_id, info.cmd);
			return -1;
		}
	}
	else
	{	
		IPC_LOG_WARNING("session %d register type %d fail", info.session_id, info.type);
	}

	return 0;
}

//async call function
static int32_t ipc_ioctl_send_async_msg(struct file *filp, struct async_msg __user *p)
{
#if 0
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct ipc_drv_msg* ipc_drv_msg = NULL;
	struct async_msg async_msg;
	struct ipc_session *session = NULL;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&async_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d", async_msg.session_id);
	IPC_LOG_INFO("pid = %d", async_msg.pid);
	IPC_LOG_INFO("send data= %d", async_msg.send_msg.data);
    IPC_LOG_INFO("cmd= %d", async_msg.send_msg.cmd);
    IPC_LOG_INFO("type= %d", async_msg.send_msg.type);

	session = get_session_by_id(async_msg.session_id);
	if (session == NULL)
	{
		IPC_LOG_ERR("session is null!");
		return -1;
	}
	// session->pid_num = async_msg.pid;

	ret = ipc_async_send_msg(&async_msg);
	if (ret != 0)
	{
		IPC_LOG_ERR("send msg failed!");
		return -1;
	}

#endif
	return 0;
}


static int32_t ipc_ioctl_session_create(struct file *filp, struct _session_init *init_msg)
{
	struct _session_init init;
	int32_t session_id = -1;

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&init, init_msg, sizeof(*init_msg)))
	{
		IPC_LOG_ERR( "copy_to_user fail!\n");
		return -EFAULT;
	}
	IPC_LOG_INFO("init src = %d, dst = %d", init.src, init.dst);
	session_id = ipc_init(init.dst, init.src, NULL);
	IPC_LOG_INFO("session id %d", session_id);

	if (session_id < 0)
	{
		IPC_LOG_WARNING("ipc_init(%d) failed, ret = %d",init.dst, session_id);
	}
	return session_id;
}

static int32_t ipc_ioctl_session_destroy(struct file *filp, uint64_t __user *session_id)
{
	int32_t ret = 0;
	uint64_t id;

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&id, session_id, sizeof(*session_id)))
	{
		IPC_LOG_ERR( "copy_to_user fail!\n");
		return -EFAULT;
	}

	ret = ipc_close(id);
	if (ret < 0)
	{
		IPC_LOG_WARNING("close session %d fail!", id);
	}

	return ret;
}

static int32_t ipc_ioctl_send_msg_func(struct file *filp, struct send_or_recv_msg __user *p)
{
	int32_t ret;
	struct send_or_recv_msg _msg;

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	//copy msg
	if (copy_from_user(&_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

	IPC_LOG_INFO("session_id = %d, data= %x, cmd= %d, type= %d ", 
				_msg.session_id, _msg.msg.data, _msg.msg.cmd, _msg.msg.type);

	if (!ipc_session_valid(_msg.session_id))
	{
		IPC_LOG_WARNING("session is invalid!");
		return -1;
	}
	ipc_msg send_msg;
	send_msg.type = _msg.msg.type;
	send_msg.cmd = _msg.msg.cmd;
	send_msg.data = _msg.msg.data;
	send_msg.token = _msg.msg.token;
#ifdef MSG_SIZE_EXTENSION
	send_msg.long_data = _msg.msg.long_data;
#endif
	bool need_reply = _msg.reply_flag;
	IPC_LOG_INFO("send msg need_reply_flag is %d", need_reply);

	ret = ipc_send(_msg.session_id, &send_msg, _msg.timeout, need_reply);
	if (ret < 0)
	{
		IPC_LOG_ERR("ipc_send session %d failed, ret = %d", _msg.session_id ,ret);
		return ret;
	}
	_msg.msg.token = send_msg.token;
	if(copy_to_user(p, &_msg, sizeof(*p)) == 0)
	{
		
		return 0;
	}
	return 0;
}

static int32_t ipc_ioctl_recv_msg_func(struct file *filp, struct send_or_recv_msg __user *p)
{
	int32_t ret;
	struct send_or_recv_msg _msg;

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	//copy msg
	if (copy_from_user(&_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d, timeout = %d", _msg.session_id, _msg.timeout);

	if (!ipc_session_valid(_msg.session_id))
	{
		IPC_LOG_INFO("session is invalid!");
		return -1;
	}
	ipc_msg recv_msg;
	recv_msg.type = _msg.msg.type;
	ret = ipc_recv(_msg.session_id, &recv_msg, _msg.timeout);
	if (ret < 0)
	{
		IPC_LOG_INFO("ipc recv session %d fail, ret = %d", _msg.session_id, ret);
		return ret;
	}

	_msg.msg.type = recv_msg.type;
	_msg.msg.cmd = recv_msg.cmd;
	_msg.msg.data = recv_msg.data;
	_msg.msg.token = recv_msg.token;
#ifdef MSG_SIZE_EXTENSION
	_msg.msg.long_data = recv_msg.long_data;
#endif
	
	IPC_LOG_INFO("recv data= %x, cmd= %d, type= %d", _msg.msg.data, _msg.msg.cmd, _msg.msg.type);

	if(copy_to_user(p, &_msg, sizeof(*p)) == 0)
	{
		
		return 0;
	}

	return -1;
}

static int32_t ipc_ioctl_get_payload_addr(struct file *filp, struct session_payload_info __user *p)
{
	int32_t ret;
	struct session_payload_info info;
	struct ipc_session *session;
	uint64_t uaddr;

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	//copy msg
	if (copy_from_user(&info, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d", info.session_id);
	session = get_session_by_id(info.session_id);
	if (IS_ERR_OR_NULL(session))
	{
		IPC_LOG_WARNING("get payload session %d is NULL", info.session_id);
		return -1;
	}

	uaddr = vm_mmap(filp, 0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED,  0x8ff00000);
	if (!uaddr || IS_ERR_VALUE(uaddr)) {
		bstipc->pool->ops->free(g_ipc_memblock);
		IPC_DEV_ERR(bstipc->dev, "vm_mmap fail! ret: %ld\n", uaddr);
		return -EFAULT;
	}

	uint64_t tmp;
	tmp = (uint64_t)&(g_ipc_all_cores_register_addr_uaddr->addr[session->src].payload);
	info.send_addr = uaddr + tmp;
	tmp = (uint64_t)&(g_ipc_all_cores_register_addr_uaddr->addr[session->dest].payload);
	info.recv_addr = uaddr + tmp;

	IPC_LOG_INFO("send addr is %lx, recv addr is %lx", info.send_addr, info.recv_addr);
	
	if(copy_to_user(p, &info, sizeof(*p)) == 0)
	{
		
		return 0;
	}
	return 0;
}

//deprecated
static int32_t ipc_ioctl_driver_ready(struct file *filp, uint32_t __user *driver_id)
{
	int32_t ret = 0;
	uint32_t id;


	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&id, driver_id, sizeof(*driver_id)))
	{
		IPC_LOG_ERR( "copy_to_user fail!\n");
		return -EFAULT;
	}
	//-1: id invalid; -2: driver not ready; 0: driver ready
	ret = ipc_node_valid(id);

	return ret;
}

static int32_t ipc_ioctl_get_info(struct file *filp, uint32_t __user *core_id)
{
	struct bstipc* bstipc = NULL;
	uint32_t coid;
	struct ipc_fill_register_msg save_msg;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

    IPC_LOG_INFO("core_id = %u", coid);

	// coding print info
	int32_t cnt = save_cnt[coid];
	int32_t i;
	IPC_INFO_PRINT("       %-10s  %-6s %-3s %-2s wakeup type", "data", "token", "cmd", "ack");
	for (i=0; i<cnt; i++) {
		if (i >= IPC_INFO_MAX_NUM)
			break;
		save_msg = *((struct ipc_fill_register_msg *)&(g_ipc_all_cores_register_addr->data_saved[coid][i%IPC_INFO_MAX_NUM]));
		IPC_INFO_PRINT("%-5d  %#010x  %-6d %-3d %-2d  %-2d     %-2d", 
						i, save_msg.long_param, save_msg.short_param, save_msg.cmd, save_msg.ack, save_msg.wakeup, save_msg.type);
	}

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t ipc_ioctl_cfg_info_enable(struct file *filp, uint32_t __user *core_id)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	uint32_t coid;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

    IPC_LOG_INFO("core_id = %u", coid);
#ifdef MSG_DUMP
	save_flag[coid] = true;
#endif
	IPC_LOG_DEBUG("%s", "exit");
	return 0;

}

static int32_t ipc_ioctl_cfg_info_disable(struct file *filp, uint32_t __user *core_id)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	uint32_t coid;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

    IPC_LOG_INFO("core_id = %u", coid);
#ifdef MSG_DUMP
	save_flag[coid] = false;
	int32_t cnt = save_cnt[coid];
	int32_t i;
	for (i=0; i<cnt; i++) {
		if (i >= IPC_INFO_MAX_NUM)
			break;
		g_ipc_all_cores_register_addr->data_saved[coid][i % IPC_INFO_MAX_NUM] = 0;
	}
	save_cnt[coid] = 0;
#endif
	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t ipc_ioctl_get_sys_info(struct file *filp, void *arg)
{
	int i;
	IPC_INFO_PRINT("                                    |                 %-29s|                   %-30s", "send_error", "recv_error");
	IPC_INFO_PRINT("%-11s%-4s%-4s%-9s%-9s%-13s%-16s%-7s%-11s%-13s%-16s%-8s%-12s", 
					"session_id", "src", "dst", "send_msg", "recv_msg",
					"ipc_no_ready", "session_invalid", "No_ACK", "queue_full",
					"ipc_no_ready", "session_invalid", "timeout", "queue_empty");
	
	struct diag_info te = diagnose_info[0];
	// IPC_INFO_PRINT("%-11d%-4d%-4d%-9d%-9d%-13d%-16d%-7d%-11d%-13d%-16d%-8d%-12d",
	// 		 te.session_id, te.src, te.dst, te.num_of_send_msg, te.num_of_recv_msg,
	// 		 te.send_err.ipc_no_ready, te.send_err.session_invalid, te.send_err.no_ACK, te.send_err.queue_full,
	// 		 te.recv_err.ipc_no_ready, te.recv_err.session_invalid, te.recv_err.timeout, te.recv_err.queue_empty);
	for (i=1; i<SESSION_NUM; i++) {
		if (diagnose_info[i].session_id > 0 && diagnose_info[i].session_id < SESSION_NUM) {
			te = diagnose_info[i];
				IPC_INFO_PRINT("%-11d%-4d%-4d%-9d%-9d%-13d%-16d%-7d%-11d%-13d%-16d%-8d%-12d",
						te.session_id, te.src, te.dst, te.num_of_send_msg, te.num_of_recv_msg,
						te.send_err.ipc_no_ready, te.send_err.session_invalid, te.send_err.no_ACK, te.send_err.queue_full,
						te.recv_err.ipc_no_ready, te.recv_err.session_invalid, te.recv_err.timeout, te.recv_err.queue_empty);
		}
	}
	return 0;
}

static long ipc_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
{
	int32_t ret = 0;

	IPC_LOG_INFO("ipc_ioctl, cmd: %d", cmd);
	switch(cmd){
		case IPC_IO_MEM_ALLOC:
			//current is deprecated
			ret = ipc_ioctl_alloc(filp, (struct ipc_buffer __user *)arg);
			break;
		case IPC_IO_MEM_FREE:
			//current is deprecated
			ret = ipc_ioctl_free(filp, (struct ipc_buffer __user *)arg);
			break;
		case IPC_IO_SES_CREATE:
			ret = ipc_ioctl_session_create(filp, (struct _session_init *)arg);
			break;
		case IPC_IO_SES_DESTROY:
			ret = ipc_ioctl_session_destroy(filp, (uint64_t __user *)arg);
			break;
		case IPC_IO_REGISTER_INFO:
			ret = ipc_ioctl_register_info(filp, (struct msg_subscribe *)arg);
			break;
		case IPC_IO_MSG_SEND:
			ret = ipc_ioctl_send_msg_func(filp, (struct send_or_recv_msg *)arg);
			break;
		case IPC_IO_MSG_RECV:
			ret = ipc_ioctl_recv_msg_func(filp, (struct send_or_recv_msg *)arg);
			break;
		case IPC_IO_GET_PAYLOAD_ADDR:
			ret = ipc_ioctl_get_payload_addr(filp, (struct session_payload_info __user *)arg);
			break;
		case IPC_IO_GET_INFO:
			ret = ipc_ioctl_get_info(filp, (uint32_t __user *)arg);
			break;
		case IPC_IO_CFG_INFO_ENABLE:
			ret = ipc_ioctl_cfg_info_enable(filp, (uint32_t __user *)arg);
			break;
		case IPC_IO_CFG_INFO_DISABLE:
			ret = ipc_ioctl_cfg_info_disable(filp, (uint32_t __user *)arg);
			break; 
		case IPC_IO_GET_SYS_INFO:
			ret = ipc_ioctl_get_sys_info(filp, (void *)arg);
			break;
		default:
			ret = -EINVAL;
			break;
	}

	IPC_LOG_INFO("ipc_ioctl exit, ret: %d", ret);
	return ret;
}

//deprecated
static int32_t ipc_mmap(struct file *filp, struct vm_area_struct *vma)
{

	int32_t ret = 0;
#if 0
	IPC_LOG_INFO("%s, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx", "enter",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      vma->vm_end - vma->vm_start,
			      pgprot_writecombine(vma->vm_page_prot)); //pgprot_writecombine():MT_NORMAL_NC,no_cache

	IPC_LOG_INFO("%s, ret: %d", "exit", ret);
#endif
	return ret;
}


static int32_t ipc_open(struct inode *inode, struct file *filp)
{
	int32_t ret = 0;

	struct bstipc* bstipc = platform_get_drvdata(g_ipc_main_pdev);
	bstipc->private_data = filp;

	return 0;
}

static int32_t ipc_drv_close(struct inode *inode, struct file *filp)
{
	int32_t ret = 0;
	/*ret = ipc_session_destroy_by_pid(current->pid);
	if (ret < 0)
	{
		IPC_LOG_WARNING("ipc_drv_close free session by pid %d fail", current->pid);
		return -1;
	}*/

	return ret;
}

//deprecated
static int32_t ipc_fasync (int32_t fd, struct file *file, int32_t on)
{
	//create session for each register service and return session id
#if 0
	IPC_LOG_DEBUG("enter!");
	struct ipc_session *session;
	IPC_LOG_INFO("pid = %d", current->pid);
	session = ipc_session_by_pid(current->pid);
	if (!session)
	{
		return -EIO;
	}
    fasync_helper(fd, file, on, &(session->ipc_fasync));
#endif
	return 0;
}

//deprecated
static ssize_t ipc_read(struct file *filp,  char __user *buf, size_t count, loff_t * ppos)
{
#if 0
	struct ipc_drv_msg *pmsg;
	pmsg = read_msg_by_pid(current->pid);

	if (pmsg)
	{
		if(copy_to_user(buf,pmsg,sizeof(*pmsg)))
		{
			devm_kfree(&g_ipc_main_pdev->dev, pmsg);
			pmsg = NULL;
			return 0;
		}
	}
#endif

	return -EIO;
}

static const struct file_operations ipc_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ipc_ioctl,
	.read = ipc_read,
	.mmap = ipc_mmap,
	.open = ipc_open,
	.release = ipc_drv_close,
	.fasync= ipc_fasync,

};


static const struct miscdevice ipc_misc_base = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &ipc_fops,
};


static int32_t ipc_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	uint32_t id = 0;
	char dev_name[sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE];
	struct bstipc *bstipc = NULL;
	int32_t ioresource_mem_idx = 0;

	g_ipc_main_pdev = pdev;

	memset(dev_name, 0, sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE);

	//create bstipc device
	bstipc = devm_kzalloc(&pdev->dev, sizeof(struct bstipc), GFP_KERNEL);
	if (!bstipc) {
		IPC_LOG_ERR("no enough memory!\n");
		return -ENOMEM;
	}

	//init bstipc device
	bstipc->dev = &pdev->dev;
	bstipc->status = IPC_INIT;
	platform_set_drvdata(pdev, bstipc);

	//init ipc share mempool
	ret = dma_set_coherent_mask(bstipc->dev, DMA_BIT_MASK(36));
	if (ret < 0) {
		IPC_LOG_ERR("dma_set_coherent_mask fail, ret %d\n", ret);
	}
	ret = ipc_mempool_init(bstipc);
	IPC_LOG_INFO("ipc mempool init result: %d\n", ret);
	if (ret < 0) {
		IPC_LOG_ERR("ipc_mempool_init fail, ret %d\n", ret);
		return ret;
	}

	//init ipc miscdev
	sprintf(dev_name, "bstipc%ud", bstipc->id);

	IPC_LOG_INFO("bstipc struct ptr: %px", bstipc);
	IPC_LOG_INFO("dev_name: %s", dev_name);

	memcpy(&bstipc->miscdev, &ipc_misc_base, sizeof(struct miscdevice));
	bstipc->miscdev.name = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);
	bstipc->miscdev.nodename = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);

	//register miscdev
	ret = misc_register(&bstipc->miscdev);
	if (ret < 0) {
		goto err_destory_pool;
	}

	IPC_LOG_INFO("device[%px] %s is registed.\n", bstipc->dev, dev_name);

	return ret;

err_destory_pool:
	ipc_destroy_cma_mempool(bstipc->pool);
	return ret;
}

static int32_t ipc_remove(struct platform_device *pdev)
{
	int32_t ret;
	struct bstipc *bstipc = platform_get_drvdata(pdev);

	misc_deregister(&bstipc->miscdev);
	ret = ipc_session_destroy_by_pid(current->pid);

	if (ret < 0)
	{
		IPC_LOG_ERR("ipc_session_destroy fail, ret %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id ipc_of_match[] = {
	{
		.compatible = "bst,ipc",
	},
};

static struct platform_driver ipc_driver = {
	.probe   = ipc_probe,
	.remove  = ipc_remove,
	.driver  = {
		.name = IPC_DRIVER_NAME,
		.of_match_table = of_match_ptr(ipc_of_match),
	},
};

static int32_t __init ipc_driver_init(void)
{
	return platform_driver_register(&ipc_driver);
}

device_initcall_sync(ipc_driver_init);



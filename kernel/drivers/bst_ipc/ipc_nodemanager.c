#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include "user_head.h"
#include "ipc_session.h"
#include "ipc_common.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_nodemanager.h"

#define IPC_DRIVER_NAME "ipc_nodemanager"

/********************* extern global variables *******************/
extern struct platform_device * g_ipc_platform_dev;

/********************* local variables ***************************/
enum ipc_core_e ipc_channel[IPC_CORE_MAX] = {
	IPC_CORE_ARM1, IPC_CORE_ARM0, IPC_CORE_ARM0, IPC_CORE_ARM0,
	IPC_CORE_ARM0, IPC_CORE_ARM0, IPC_CORE_ARM0, IPC_CORE_ARM0,
	IPC_CORE_ARM5, IPC_CORE_ARM5, IPC_CORE_ARM3, IPC_CORE_ARM6,
	IPC_CORE_ARM6, IPC_CORE_ARM6, IPC_CORE_ARM7, IPC_CORE_ARM1,
	IPC_CORE_ARM2, IPC_CORE_ARM4
};

/********************* global variables ***************************/
struct ipc_client_info *client_map[IPC_CORE_MAX] = { 0 };

/********************* function declaration ***************************/

struct device* ipc_get_dev_by_coreid(enum ipc_core_e core)
{
	if(client_map[core] == NULL)
	{
		IPC_LOG_WARNING("ipc_get_dev_by_coreid NULL");
		return NULL;
	}
	return client_map[core]->dev;
}

uint64_t get_tx_reg_by_intr(int32_t intr)
{
	if (client_map[intr] != 0)
		return client_map[intr]->tx_reg;
	else
		return 0;
}

int32_t ipc_node_msgparse(enum ipc_core_e core, struct ipc_client_info **cl_info)
{
	IPC_LOG_INFO(" ipc_node_msgparse, core = %d", core);
	//check driver is ready
	if (core >= ARRAY_SIZE(client_map)) {
		*cl_info = NULL;
		IPC_LOG_WARNING(" dst is invalid!");
		return -1;
	}
	if (client_map[core] == 0) {
		*cl_info = NULL;
		IPC_LOG_WARNING(" client is not available!");
		return -1;
	}

	*cl_info = (struct ipc_client_info *)client_map[core];
	IPC_LOG_INFO("get clinet info!");

	return 0;
}

int32_t ipc_node_destroy(enum ipc_core_e core_id)
{
	IPC_LOG_INFO("ipc_node_destroy, core_id = %d", core_id);

	if (core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_WARNING(" coreid %d is invalid, destroy fail", core_id);
		return -1;
	}
	client_map[core_id] = 0;

	return 0;
}

int32_t ipc_node_valid(enum ipc_core_e core_id)
{
	IPC_LOG_INFO("core_id = %d", core_id);

	if (core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_ERR(" source id is invalid!");
		return -1;
	}

	IPC_LOG_INFO("num_online_cpus = %d", num_online_cpus());
	if (core_id >= IPC_CORE_ARM0 && core_id <= IPC_CORE_ARM7) // dst is ARM
	{
		if (num_online_cpus() <= core_id - IPC_CORE_ARM0) // 无效的核心
		{
			IPC_LOG_ERR("num_online_cpus() = %d, but core_id = %d",
				    num_online_cpus(), core_id);
			return -ENODEV;
		} else {
			return 0;
		}
	}

	if (client_map[core_id] == 0) {
		IPC_LOG_ERR("core is not ready!");
		return -CLIENT_STATUS_NOTREADY;
	}

	int32_t cnt = 100;
	struct ipc_client_info *cl_info = (void *)client_map[core_id];
	while (cl_info->status != CLIENT_STATUS_READY && cnt--) {
		// schedule_timeout_interruptible(10);
		msleep(10); // FIXME
	}

	if (cnt <= 0) {
		IPC_LOG_ERR("client_map[%d] is not OK, status is: %d", core_id,
			    cl_info->status);
		return cl_info->status;
	}

	IPC_LOG_INFO("driver is ready!");
	return cl_info->status;
}

int32_t ipc_node_get_node_of_coreid(enum ipc_core_e dest_core_id, struct ipc_client_info **cl_info)
{
	struct ipc_client_info *client_info = NULL;
	struct ipc_mbox *ipc_mbox = NULL;
	int32_t ret = -1;

	IPC_LOG_INFO("ipc_node_get_node_of_coreid, dst= %d", dest_core_id);
	if (dest_core_id < 0 || dest_core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_ERR("core id %d is NOT avaliable", dest_core_id);
		return -1;
	}

	//get client from map
	client_info = client_map[dest_core_id];
	if (client_info == NULL) {
		IPC_LOG_INFO("need new client for dst %d",dest_core_id);
		client_info = devm_kzalloc(&g_ipc_platform_dev->dev, sizeof(*client_info), GFP_KERNEL);
		if (!client_info) 
		{
			IPC_LOG_ERR("no enough memory!");
			return -ENOMEM;
		}

		ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);
		if (unlikely(IS_ERR_OR_NULL(ipc_mbox))) {
			ret = PTR_ERR_OR_ZERO(ipc_mbox);
			IPC_LOG_ERR("platform_get_drvdata(g_ipc_platform_dev), ret %d", ret);
			return ret;
		}
		client_info->core_id = dest_core_id;		
		client_info->sem_reg =
			ipc_mbox->sem_base + SEM_MST_ID_A_CPU +
			SEM_BANK0_OFFSET_BASE + SEM0_OFFSET_IN_BANK +
			SEM_IN_BANK_STEP * (dest_core_id - IPC_CORE_ARM0);

		client_info->tx_reg = IRQ_WRITE_ADDR(ipc_mbox->event_base, ipc_channel[dest_core_id], dest_core_id);
		IPC_LOG_INFO("client_info->tx_reg = 0x%px", __virt_to_phys(client_info->tx_reg));

		client_info->channel_status = CLIENT_STATUS_READY;
		init_completion(&client_info->tx_complete);
		client_info->status = CLIENT_STATUS_READY;
		client_map[dest_core_id] = client_info;
	}

	if (client_info != NULL)
	{
		IPC_LOG_INFO("return dst %d client", dest_core_id);
		*cl_info = client_info;
		return 0;
	}

	return -1;
}



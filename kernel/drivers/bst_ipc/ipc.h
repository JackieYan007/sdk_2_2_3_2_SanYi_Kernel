/*
 * IPC: Linux device driver for Black Sesame Technologies Inter Proccessor Communication
 *
 */

#ifndef IPC_H
#define IPC_H

#include <linux/miscdevice.h>
#include <linux/mailbox_client.h>
#include <linux/firmware.h>
#include <linux/spinlock.h>

#include "ipc_common.h"

typedef enum ipc_status{
	IPC_INIT = 0,
	IPC_IDLE,
	IPC_RUNNING,
	IPC_STOP,
	IPC_DESTROY,
	IPC_STATUS_MAX = INT_MAX
} IPC_STATUS;


struct bstipc {
	struct device *dev;
	struct miscdevice miscdev;
	struct ipc_mempool *pool;
	spinlock_t pool_lock;
	IPC_STATUS status;
	uint32_t id;
	void __iomem *base;
	u64 ipc_offset;
	void* private_data;

};

#endif

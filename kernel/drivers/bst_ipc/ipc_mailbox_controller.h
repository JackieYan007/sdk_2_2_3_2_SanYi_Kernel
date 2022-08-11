#ifndef IPC_MAILBOX_CONTROLLER_H
#define IPC_MAILBOX_CONTROLLER_H

#include "ipc_common.h"
#include "ipc_msg_manager.h"
#include "ipc_nodemanager.h"

typedef enum rx_mode {
	IRQ_MODE = 0,
	POLL_MODE
} RX_MODE;

struct ipc_mbox {
	struct device *dev;
	void __iomem *event_base;
	void __iomem *sem_base;
	struct ipc_mempool *pool;
#ifdef ON_FPGA
	void __iomem *fpga_reset;
	void __iomem *fpga_status;
#endif
};

//share buffer format definition
struct ipc_aligned_msg
{
	struct ipc_fill_register_msg msg;
#ifdef MSG_SIZE_EXTENSION
	uint64_t payload[6];
#else
	uint64_t payload[7];
#endif
};

//ipc shared buffer
struct ipc_all_cores_register_addr  // place in one page : 4096Byte
{
    struct ipc_aligned_msg addr[IPC_CORE_MAX];  // 64B*18 = 1152B
    uint64_t data_saved[IPC_CORE_MAX][4096];    // 8B * 18 * 4096 = 576 KB   ????
};

int32_t ipc_send_data(struct ipc_client_info *client_info, enum ipc_core_e src, void *data);

#endif

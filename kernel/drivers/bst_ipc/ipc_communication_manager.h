#ifndef IPC_COMMUNICATION_H
#define IPC_COMMUNICATION_H

#include <linux/ipc_interface.h>
#include <linux/kfifo.h>

#include "ipc_common.h"


//global interface definition
int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest, int32_t session_id, void *buf, uint32_t len);
int32_t ipc_drv_recv(enum ipc_core_e src, enum ipc_core_e dest, void* buf, uint32_t len);

int32_t ipc_communication_create(void);
int32_t ipc_communication_close(void);

#endif

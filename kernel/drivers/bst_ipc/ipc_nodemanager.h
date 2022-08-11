#ifndef IPC_NODEMANAGER_H
#define IPC_NODEMANAGER_H

#include "ipc_common.h"
#include "user_head.h"

enum ipc_channel_status_e{
    CHANNEL_READY,
    CHANNEL_SENDING,
    CHANNEL_SEND_SUCCESS,
    CHANNEL_SEND_FAIL,
};

enum ipc_client_status_e{
    CLIENT_STATUS_OFFLINE,
    CLIENT_STATUS_NOTREADY,
    CLIENT_STATUS_READY,
    CLIENT_STATUS_ONLINE,
};

/**
 * struct ipc_client_info: mailbox client information
 *
 * @recv_msg_callback: When this client received a message,
 *                     this function will be called.
 *
 */
struct ipc_client_info
{
    struct device* dev;
    enum ipc_core_e core_id;
    enum ipc_client_status_e status;
    enum ipc_channel_status_e channel_status;
    struct completion tx_complete;
    void __iomem * sem_reg;
    void __iomem* tx_reg;
};

struct device* ipc_get_dev_by_coreid(enum ipc_core_e core);
int32_t ipc_node_create(struct ipc_client_info *cl_info);
int32_t ipc_node_destroy(enum ipc_core_e core_id);
int32_t ipc_node_msgparse(enum ipc_core_e core, struct ipc_client_info **cl_info);
int32_t ipc_node_valid(enum ipc_core_e core_id);
int32_t ipc_node_create_session(enum ipc_core_e dest_core_id,enum ipc_core_e src_core_id);
int32_t ipc_node_get_node_of_coreid(enum ipc_core_e core_id, struct ipc_client_info **cl_info);
struct ipc_session* ipc_node_get_session_by_coreid(struct ipc_client_info* cl_info,enum ipc_core_e core_id);
uint64_t get_tx_reg_by_intr(int32_t intr);

#endif

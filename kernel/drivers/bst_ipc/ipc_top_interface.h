#ifndef IPC_TOP_INTERFACE
#define IPC_TOP_INTERFACE

#include <linux/ipc_interface.h>

/** callback function register definition
 * 
 */
typedef int (*recv_process_callback)(ipc_msg *recv_msg, ipc_msg_type type);

/**
 * receive message process register.
 * @session_id: the session used to let ipc know which session is interested in this method subscription. 
 *              get from ipc_init.
 * @cmd: the cmd session want to subscribe when ipc receive this method, ipc would dispatch message 
 *              to session message queue.
 * @return 0 for success, negative value for errors.
 */
int32_t ipc_recv_process_register(int32_t session_id,  recv_process_callback recv_process);


int32_t ipc_sync_send_msg(int32_t session_id, ipc_msg *msg, ipc_msg *recv_msg, int32_t timeout);

#endif
#include <linux/ipc_interface.h>

#include "ipc_top_interface.h"
#include "ipc_common.h"


#define IPC_DRIVER_NAME "ipc_top_interface"

//this API is defined as client/server model
int32_t ipc_sync_send_msg(int32_t session_id, ipc_msg *msg, ipc_msg *recv_msg, int32_t timeout)
{
    int32_t ret;
    
    //msg send
    ret = ipc_send(session_id, msg, 100);
    if (ret < 0)
    {
        IPC_LOG_ERR("send message failed!");
        return -1;
    }

    recv_msg.token = send_msg.token;
    IPC_LOG_INFO("send_msg.token = %d", send_msg.token);

    ret = ipc_recv_reply(session_id, recv_msg, timeout);
    if (ret < 0)
    {
        IPC_LOG_ERR("recv message failed!");
        return -1;
    }

    IPC_LOG_INFO("type= %d", recv_msg.type);
	IPC_LOG_INFO("recv data= %d", recv_msg.data);
    IPC_LOG_INFO("cmd= %d", recv_msg.cmd);

    return 0;
}
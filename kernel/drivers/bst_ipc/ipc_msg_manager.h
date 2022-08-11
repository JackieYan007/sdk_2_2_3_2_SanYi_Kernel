#ifndef IPC_MSG_MANAGER_H
#define IPC_MSG_MANAGER_H

#include <linux/list.h>
#include <linux/kfifo.h>
#include <linux/types.h>
#include <linux/ipc_interface.h>

#include "ipc_communication_manager.h"

/**
 * struct ipc_drv_msg - struct msg description send in ipc service
 * this struct would describe message in detail and transferred in whole ipc service
 * @session_id:		    message session id, every thread need create a session and fill this parameter in message type automatically.
 * @msg: message content
 * @dst: message send destination
 * @src: message send resource
 * @node: use for sent meassage hash map
 */
struct ipc_drv_msg{
	int32_t session_id;
	uint32_t src;
	uint32_t dst;
	struct hlist_node node; //use for sent msg hash map 
	ipc_msg msg;
};

//this message format is used for controller layer
struct ipc_fill_register_msg
{
    uint32_t long_param:32;
    uint32_t short_param:16;
    uint32_t cmd:8;
    uint32_t ack:5;
    uint32_t wakeup:1;
    uint32_t type:2;
#ifdef MSG_SIZE_EXTENSION
    uint64_t long_data:64;
#endif
};

int32_t send_msg_init(void);
int32_t send_msg_destroy(void);
int32_t send_msg_in(struct ipc_drv_msg* msg);
int32_t send_msg_out(uint32_t token, struct ipc_drv_msg** msg);
uint32_t ipc_msg_get_an_available_token(void);
int32_t show_all_msgs(void);
#endif

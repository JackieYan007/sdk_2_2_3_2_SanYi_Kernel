#ifndef __USER_HEAD_H_
#define __USER_HEAD_H_

//this is header file between driver and libipc 
#include <linux/ipc_interface.h>

//macro definition
#define MAX_SESSION_COUNTS 64
#define MAX_CMD_COUNTS 256

//IO definition
#define IPC_IO_MEM 'M'
#define IPC_IO_MEM_ALLOC        	            _IO(IPC_IO_MEM, 1)      //current is deprecated
#define IPC_IO_MEM_FREE	                        _IO(IPC_IO_MEM, 2)      //current is deprecated
#define IPC_IO_REGISTER_INFO	                _IO(IPC_IO_MEM, 3)     
#define IPC_IO_SES_CREATE                       _IO(IPC_IO_MEM, 4)
#define IPC_IO_SES_DESTROY                      _IO(IPC_IO_MEM, 5)
#define IPC_IO_DRI_READY                        _IO(IPC_IO_MEM, 6)      //current is deprecated
#define IPC_IO_MSG_SEND_ASYNC                   _IO(IPC_IO_MEM, 7)      //current is deprecated
#define IPC_IO_MSG_SEND                         _IO(IPC_IO_MEM, 8)
#define IPC_IO_MSG_RECV                         _IO(IPC_IO_MEM, 9)
#define IPC_IO_GET_INFO                         _IO(IPC_IO_MEM, 10)
#define IPC_IO_CFG_INFO_ENABLE                  _IO(IPC_IO_MEM, 11)
#define IPC_IO_CFG_INFO_DISABLE                 _IO(IPC_IO_MEM, 12)
#define IPC_IO_GET_SYS_INFO                     _IO(IPC_IO_MEM, 13)
#define IPC_IO_GET_PAYLOAD_ADDR                 _IO(IPC_IO_MEM, 14)


/**
 * this struct is used for user for to create session
 *
 * @src:    the source core id to communicate with,
 *          that's the source core id of session.
 * @dst:    the target core id of session
 */
struct _session_init {
    uint8_t src;
    uint8_t dst;
    uint8_t session_id;
};

typedef struct 
{
    ipc_msg_type type;
    uint8_t cmd;
    uint16_t token;
    uint32_t data;
    uint64_t userdata;
    #ifdef MSG_SIZE_EXTENSION
    uint64_t long_data;
    #endif
}user_msg;

// send or recv msg struct
struct send_or_recv_msg {
    user_msg msg;
    uint8_t session_id;
    int32_t timeout;
    bool reply_flag;
    bool payload_flag;
    uint8_t payload_size;
};


//definition of IPC package message.
/*struct ipc_msg_package{
    ipc_msg_type type;
    uint8_t cmd;
    uint8_t payload_size;
    uint16_t token;
    uint32_t data;
    uint64_t payload[7];
};

// send or recv msg struct
struct send_or_recv_package_msg {
    struct ipc_msg_package msg;
    int32_t session_id;
    int32_t timeout;
    bool reply_flag;
};

//sync package msg struct
struct sync_msg_package {
    struct ipc_msg_package send_msg;
    struct ipc_msg_package recv_msg;
    int32_t session_id;
    int32_t timeout;
};

//async package msg struct
struct async_msg_package {
    struct ipc_msg_package send_msg;
    int32_t session_id;
    pid_t pid;
};*/

//subscribe signal struct
struct msg_subscribe {
    ipc_msg_type type;
    uint8_t session_id;
    uint32_t cmd;
};

//get payload info
struct session_payload_info {
    uint8_t session_id;
    uint64_t send_addr;
    uint64_t recv_addr; 
};

#endif

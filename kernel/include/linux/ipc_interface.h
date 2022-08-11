/*
* © Copyright Black Sesame Technologies (Shanghai)Ltd. Co., 2020. All rights reserved.
*
* This file contains proprietary information that is the sole intellectual property of
* Black Sesame Technologies (Shanghai)Ltd. Co.
*
* No part of this material or its documentation may be reproduced, distributed,
* transmitted, displayed or published in any manner without the written permission
* of Black Sesame Technologies (Shanghai)Ltd. Co.
*/

#ifndef BST_IPC_INTERFACE_H
#define BST_IPC_INTERFACE_H

//differ from dsp to linux
#include <linux/types.h>

//err number definition
#define IPC_INIT_ERR -1
#define IPC_INIT_ERR_INVALID_PARAM -2
#define IPC_INIT_ERR_LINK -3
#define IPC_INIT_ERR_SESSION -4

#define IPC_SEND_ERR -1
#define IPC_SEND_ERR_INVALID_PARAM -2
#define IPC_SEND_ERR_LINK -3
#define IPC_SEND_ERR_TIMEOUT -4

#define IPC_RECV_ERR -1
#define IPC_RECV_ERR_INVALID_PARAM -2
#define IPC_RECV_ERR_LINK -3
#define IPC_RECV_ERR_TIMEOUT -4
#define IPC_RECV_ERR_GET_MSG_FAIL -5

#define IPC_CLOSE_ERR -1

#define IPC_SUBSCRIBE_ERR -1
#define IPC_SUBSCRIBE_INVALID_PARAM -2

#define IPC_RECV_PROCESS_REGISTER_ERR -1
#define IPC_RECV_PROCESS_REGISTER_FAIL -2

#define IPC_METHOD_REGISTER_ERR -1
#define IPC_METHOD_REGISTER_INVALID_PARAM -2
#define IPC_METHOD_REGISTER_REPETITION -3

/**
 * enum ipc_core_e : all core's ids
 */
enum ipc_core_e{                /* 5 bits */
    IPC_CORE_ARM0 = 0,
    IPC_CORE_ARM1 = 1,
    IPC_CORE_ARM2 = 2,
    IPC_CORE_ARM3 = 3,
    IPC_CORE_ARM4 = 4,
    IPC_CORE_ARM5 = 5,
    IPC_CORE_ARM6 = 6,
    IPC_CORE_ARM7 = 7,
    IPC_CORE_R5_0 = 8,
    IPC_CORE_R5_1 = 9,
    IPC_CORE_DSP_0 = 10,
    IPC_CORE_DSP_1 = 11,
    IPC_CORE_DSP_2 = 12,
    IPC_CORE_DSP_3 = 13,
    IPC_CORE_ISP = 14,
    IPC_CORE_VSP = 15,
    IPC_CORE_DSP = 16,
    IPC_CORE_SECURE = 17,
    IPC_CORE_MAX,
};

//enum for IPS message type.
typedef enum {
    IPC_MSG_TYPE_INVALID = 0,
    IPC_MSG_TYPE_METHOD,
    IPC_MSG_TYPE_REPLY,
    IPC_MSG_TYPE_SIGNAL,
}ipc_msg_type;

//basic ipc message definition
typedef struct {
    ipc_msg_type type;
    uint8_t cmd;
    uint16_t token;
    uint32_t data;
    #ifdef MSG_SIZE_EXTENSION
    uint64_t long_data;
    #endif
    int64_t timestamp;
}ipc_msg;

//async msg definition
//deprecated
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
}async_ipc_msg;


/**
 * Initialize IPC environment, including data link checking, online notification,
 * session creation, etc.
 * session contains two endpoints, one is source core id,
 * another is destination core id.
 *
 * @dest_core_id: the target core id to communicate with,
 *                  that's the destination core id of session.
 * @src_core_id: the source core id of session.
 * @arg: extra parameter for init, it can be NULL in firmware, this parameter is deprecated.
 * @return session id if success, negative on error.
 */
int32_t ipc_init(enum ipc_core_e dest_core_id, enum ipc_core_e src_core_id, void *arg);

/**
 * Close IPC communication.
 * Offline notification will be sent, and the session will be destoried.
 *
 * @session_id: The session to close, got from ipc_init.
 * @return 0 for success, negative value for errors.
 */
int32_t ipc_close(int32_t session_id);

/**
 * Send IPC message.
 * @session_id: the session used to send message. get from ipc_init.
 * @msg: the message to be sent.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *           to wait for. other value is meaningless
 * @reply_flag: true means this method need reply, 
 *           while false means this method don not need to reply.
 *           this parameter is bool type, and this api is just for compatibility, we will deprecate this API later 
 *           and use exact definition.
 * @return 0 for success, negative value for errors.
 */
int32_t ipc_send(int32_t session_id, ipc_msg* msg, int32_t timeout, ...);

/**
 * Send reply of method message.
 * @session_id: the session used to send message. get from ipc_init.
 * @msg: the message to be sent.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *           to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 */
static inline int32_t ipc_send_reply(int32_t session_id, ipc_msg* msg, int32_t timeout)
{
    msg->type = IPC_MSG_TYPE_REPLY;
    return ipc_send(session_id, msg, timeout);
}

/**
 * Send method message.
 * @session_id: the session used to send message. get from ipc_init.
 * @msg: the message to be sent.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *           to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 * @notice: this function must be called after every previous method message has received it's reply message.
 */
static inline int32_t ipc_send_method(int32_t session_id, ipc_msg* msg, int32_t timeout)
{
    msg->type = IPC_MSG_TYPE_METHOD;
    return ipc_send(session_id, msg, timeout);
}

/**
 * Send signal message.
 * @session_id: the session used to send message. get from ipc_init.
 * @msg: the message to be sent.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *           to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 */
static inline int32_t ipc_send_signal(int32_t session_id, ipc_msg* msg, int32_t timeout)
{
    msg->type = IPC_MSG_TYPE_SIGNAL;
    return ipc_send(session_id, msg, timeout);
}

/**
 * Send IPC async message.
 * @session_id: the session used to send message. get from ipc_init.
 * @msg: the message to be sent.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *           to wait for. other value is meaningless\
 * @reply_flag: true means this method need store in sent message list to wait its reply message
 * @return 0 for success, negative value for errors.
 */
//deprecated
int32_t ipc_async_send(int32_t session_id, async_ipc_msg* msg, bool reply_flag, int32_t timeout);

/**
 * Receive IPC message.
 * @session_id: the session used to receive message. get from ipc_init.
 * @msg: the buffer to receive message.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *          to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 * @note if msg->type is IPC_MSG_TYPE_REPLY, then the msg->cmd and msg->token must the same
 *         with the corresponding method call.
 */
int32_t ipc_recv(int32_t session_id,  ipc_msg* msg, int32_t timeout);

/**
 * Receive method message.
 * @param session_id, the session used to receive message. get from ipc_init.
 * @param msg, the buffer to receive message.
 * @param timeout, -1 means wait forever, 0 means no wait, positive value is the millisenconds
 * to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 */
//deprecated
static inline int32_t ipc_recv_method(int32_t session_id,  ipc_msg* msg, int32_t timeout)
{
    msg->type = IPC_MSG_TYPE_METHOD;
    return ipc_recv(session_id, msg, timeout);
}

/**
 * Receive signal message.
 *
 * @session_id: the session used to receive message. Got from ipc_init.
 * @msg: the buffer to receive message.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *          to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 */
//deprecated
static inline int32_t ipc_recv_signal(int32_t session_id,  ipc_msg* msg, int32_t timeout)
{
    msg->type = IPC_MSG_TYPE_SIGNAL;
    return ipc_recv(session_id, msg, timeout);
}
/**
 * Receive reply message.
 *
 * @session_id: the session used to receive message. Got from ipc_init.
 * @msg: the buffer to receive message.
 * @timeout: -1 means wait forever, 0 means no wait, positive value is the millisenconds
 *          to wait for. other value is meaningless
 * @return 0 for success, negative value for errors.
 */
//deprecated
int32_t ipc_recv_reply(int32_t session_id,  ipc_msg* msg, int32_t timeout);

/**
 * signal subscribe.
 * @session_id: the session used to let ipc know which session is interested in this signal subscription. 
 *              get from ipc_init.
 * @cmd: the cmd session want to subscribe when ipc receive this signal, ipc would dispatch message 
 *              to session message queue.
 * @return 0 for success, negative value for errors.
 */
int32_t ipc_signal_subscribe(int32_t session_id,  uint32_t cmd);

/**
 * method register.
 * @session_id: the session used to let ipc know which session is interested in this method subscription. 
 *              get from ipc_init.
 * @cmd: the cmd session want to subscribe when ipc receive this method, ipc would dispatch message 
 *              to session message queue.
 * @return 0 for success, negative value for errors.
 */
int32_t ipc_method_register(int32_t session_id,  uint32_t cmd);

#endif
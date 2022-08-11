/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last Editor: Shichen Lin(shichen.lin@bst.ai)
 * Previous Editor: Yuchao Wang
 *
 * @file 	bstn_msg_manager.h
 * @brief 	This is the header file of the session part in BSTN driver. It
 *			contains the structure definitions of the message manager and
 *			exchange node. It also has declarations of message handling
 *			functions as well as the initialization and cleanup functions.
 */

#ifndef BSTN_MSG_MANAGER_H
#define BSTN_MSG_MANAGER_H

#define BSTN_MAX_DEV_NUM                    1
#define BSTN_EXCHANGE_NODE_NUM              4
//1s for timeout
#define BSTN_RSP_TIMEOUT_MS                 (BSTN_EXCHANGE_NODE_NUM * 1000 * 8) // exchange timeout in 32s
#define BSTN_RSP_TIMEOUT_JIFFIES            (BSTN_RSP_TIMEOUT_MS * HZ / 1000) 

struct bstn_req_msg {
    uint16_t nid;
    struct bsnn_request req;
    uint32_t target_net;
};

struct bstn_rsp_msg {
    uint16_t nid;
    struct bsnn_response rsp;
};

struct bstn_exchange_node {
	uint16_t nid;
	Q_NEW_LINK(bstn_exchange_node, link);
	struct bstn_req_msg *req_buf;
	struct bstn_rsp_msg *rsp_buf;
	struct completion complete;
};

struct bstn_msg_manager {
#ifdef IPC
    int32_t ipc_session_id;
#else
    struct bstn_channel_manager channel_manager;
#endif

	struct bstn_exchange_node *flist; //free exchange node list
	struct mutex flist_lock;

	struct bstn_memblock *req_bufs; //an one-time allocated large buffer for all request buffers
	struct bstn_exchange_node exchange_nodes[BSTN_EXCHANGE_NODE_NUM];

	struct task_struct *msg_receiver_task;
};

int bstn_msg_manager_init(struct bstn_device *pbstn);
void bstn_msg_manager_exit(struct bstn_device *pbstn);
int bstn_msg_exchange(struct bstn_device *pbstn, struct bsnn_msg_exchange *msg);

#endif


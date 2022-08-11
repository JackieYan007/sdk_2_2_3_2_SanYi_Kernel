/*!
 * bst_lwnn: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author  Shichen Lin(shichen.lin@bst.ai)
 *
 * @file    bst_lwnn_msg_manager.h
 * @brief   This is the header file of the message manager part in bst_lwnn driver.
 *          It contains the structure definitions of the message manager and
 *          declarations of message handling functions as well as initialization
 *          and exit functions of the message manager.
 */

#ifndef BST_LWNN_MSG_MANAGER_H
#define BST_LWNN_MSG_MANAGER_H

#define BST_LWNN_MAX_WORKLOAD       10
#define BST_LWNN_RSP_TIMEOUT_MS     1000

struct bst_lwnn_xchg {
    enum {
        XCHG_STATUS_SUCCESS = 0,
        XCHG_STATUS_FAILURE,
        XCHG_STATUS_REAPER
    } result;
    struct bst_lwnn_msg_xchg *xchg;
    struct completion complete;
    struct list_head link; //work list link
};

struct bst_lwnn_dsp_msg_ctl {
    struct bst_lwnn *pbst_lwnn;
    int dsp;
    enum {
        BST_LWNN_MSG_OFFLINE = 0,
        BST_LWNN_MSG_ONLINE,
        BST_LWNN_MSG_STOP,
    } state;
    int ipc_session_id;
    int workload;
    struct completion work_sem;
    spinlock_t wl_lock;
    struct list_head work_list;
    /* 
     * we create kernel threads instead of using work queues in consideration of
     * blocking caused by DSP failure
     */
    struct task_struct *worker;
};

struct bst_lwnn_msg_manager {
    spinlock_t req_lock;

    spinlock_t worker_lock;
    struct bst_lwnn_dsp_msg_ctl dsps[BST_LWNN_MAX_DSP_NUM];
    struct bst_lwnn_memblock *req_bufs;
};

int bst_lwnn_msg_send(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t data);
int bst_lwnn_msg_recv(struct bst_lwnn *pbst_lwnn, int dsp, uint32_t *data, int timeout);
int bst_lwnn_msg_xchg(struct bst_lwnn *pbst_lwnn, struct bst_lwnn_msg_xchg *msg_xchg);
int bst_lwnn_msg_manager_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_msg_manager_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_msg_manager_exit(struct bst_lwnn *pbst_lwnn);

#endif
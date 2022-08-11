/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin(shichen.lin@bst.ai)
 *
 * @file    bst_cv_msg_manager.h
 * @brief   This is the header file of the session part in bst_cv driver. It
 *          contains the structure definitions of the message manager and
 *          declarations of message handling functions as well as initialization
 *          and exit functions of the message manager.
 */

#ifndef BST_CV_MSG_MANAGER_H
#define BST_CV_MSG_MANAGER_H

#define MAX_WORKLOAD 10

struct bst_cv_req {
    uint32_t cookie;
    enum {
        XRP_STATUS_SUCCESS = 0,
        XRP_STATUS_FAILURE,
    } result;
    struct xrp_ioctl_queue queue;
    union in_data in;
    union out_data out;
    union buffer_data buf;
    struct bst_cv_memblock *block;
    struct completion complete;
    struct hlist_node ht_node; //hash table node
    struct list_head link; //work list link
};

struct bst_cv_dsp_msg_ctl {
    struct bst_cv *pbst_cv;
    int dsp;
    enum {
        BST_CV_MSG_OFFLINE = 0,
        BST_CV_MSG_ONLINE,
        BST_CV_MSG_STOP,
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

struct bst_cv_msg_manager {
    spinlock_t req_lock;
    uint32_t next_cookie;
    int wait_req_num;
    DECLARE_HASHTABLE(req_ht, 6);
    struct list_head zombie_list;
    spinlock_t zl_lock;
    struct completion zombie_sem;
    struct task_struct *cleaner;

    spinlock_t worker_lock;
    struct bst_cv_dsp_msg_ctl dsps[BST_CV_DSP_NUM];
};

int bare_cv_msg_send(struct bst_cv *pbst_cv, int dsp, uint32_t data);
int bare_cv_msg_recv(struct bst_cv *pbst_cv, int dsp, uint32_t *data, int timeout);
int bare_cv_req_dispatch(struct bst_cv *pbst_cv, struct xrp_ioctl_queue *queue);
int bare_cv_rsp_wait(struct bst_cv *pbst_cv, struct xrp_ioctl_wait *wait);

#endif
/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin(shichen.lin@bst.ai)
 *
 * @file    bst_cv_msg_manager.c
 * @brief   This is the source file of the message manager part of bst_cv driver.
 *          It contains function definitions of message handling as well as
 *          initialization and exit of the message manager.
 */

#include "bst_cv.h"
//for debugging purposes
#include <linux/fs.h>
#include <linux/time.h>

/*
 * @func    bare_cv_msg_send
 * @brief   This function sends a message with the specified 4-byte data to the
 *          target DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          queue - the pointer to the command queue metadata
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_msg_send(struct bst_cv *pbst_cv, int dsp, uint32_t data)
{
    ipc_msg msg;

    msg.data = data;
    msg.type = IPC_MSG_TYPE_SIGNAL;
    return ipc_send(pbst_cv->msg_manager.dsps[dsp].ipc_session_id, &msg, 0);
}

/*
 * @func    bare_cv_msg_recv
 * @brief   This function receives a response from the target DSP with the
 *          specified timeout.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          dsp - the target DSP number
 *          data - the pointer to the received data
 *          timeout - the timeout in ms(-1 means waiting forever)
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_msg_recv(struct bst_cv *pbst_cv, int dsp, uint32_t *data, int timeout)
{
    int ret;
    ipc_msg msg;

    ret = ipc_recv(pbst_cv->msg_manager.dsps[dsp].ipc_session_id, &msg, timeout);
    *data = msg.data;
    return ret;
}

/*
 * @func    bst_cv_worker
 * @brief   This is the worker thread function which handles the request sending
 *          and response receiving for one DSP
 * @params  args - the pointer to the message controller of the target DSP
 * @return  0
 */
#ifdef BST_CV_DEBUG
static int cmd_count;
#endif
static int bst_cv_worker(void *args)
{
    struct bst_cv_dsp_msg_ctl *cur = args;
    struct bst_cv *pbst_cv = cur->pbst_cv;
    int dsp = cur->dsp;
    struct sched_param param;
    struct xrp_dsp_cmd *xrp_dsp_comm_base = pbst_cv->fw_manager.dsps[dsp].assigned_mem->kern_addr;
    struct bst_cv_req *req;
    uint32_t data;
    unsigned long flags;

#ifdef BST_CV_DEBUG
    struct timeval start, end;
    //debug memory dump
    char fname[25];
    struct file *filp;
    loff_t pos;
#endif

    param.sched_priority = MAX_RT_PRIO - 1;
    sched_setscheduler(current, SCHED_FIFO, &param);
    do {
        //wait for work
        wait_for_completion(&pbst_cv->msg_manager.dsps[dsp].work_sem);
        if (pbst_cv->msg_manager.dsps[dsp].state == BST_CV_MSG_STOP) { 
            goto worker_exit;
        }
        //take a request from the work list
        spin_lock_irqsave(&pbst_cv->msg_manager.dsps[dsp].wl_lock, flags);
        req = container_of(pbst_cv->msg_manager.dsps[dsp].work_list.next,
            struct bst_cv_req, link);
        list_del(pbst_cv->msg_manager.dsps[dsp].work_list.next);
        spin_unlock_irqrestore(&pbst_cv->msg_manager.dsps[dsp].wl_lock, flags);
        BST_CV_TRACE_PRINTK("prepare to send req %px with cookie %d to DSP %d", req, req->queue.cookie, dsp);

        //set up the dsp cmd for the request
        xrp_dsp_comm_base->flags = XRP_DSP_CMD_FLAG_REQUEST_VALID;
        xrp_dsp_comm_base->in = req->in;
        xrp_dsp_comm_base->out = req->out;
        xrp_dsp_comm_base->buf = req->buf;
        xrp_dsp_comm_base->in_data_size = req->queue.in_data_size;
        xrp_dsp_comm_base->out_data_size = req->queue.out_data_size;
        xrp_dsp_comm_base->buffer_size = req->queue.buffer_size;

#ifdef BST_CV_DEBUG
        snprintf(fname, 25, "/bst_cv/cmd%d", cmd_count);
        filp = filp_open(fname, O_RDWR | O_CREAT, 0777);
        if (IS_ERR(filp)) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "cannot open the file %s, ret %ld", fname, (long)filp);
        }
        else {
            pos = 0;
            kernel_write(filp, (void *)xrp_dsp_comm_base, 0x900000, &pos);
            filp_close(filp, NULL);
        }

        do_gettimeofday(&start);
#endif

        //send request and wait for response
        if (bare_cv_msg_send(pbst_cv, dsp, 0) < 0) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fail to send the request!");
            req->result = XRP_STATUS_FAILURE;
        }
        else {
wait_for_response:
            if (bare_cv_msg_recv(pbst_cv, dsp, &data, -1) < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fail to receive the response!");
                req->result = XRP_STATUS_FAILURE;
            }
            else if (xrp_dsp_comm_base->flags != XRP_DSP_CMD_FLAG_RESPONSE_VALID) { // XRP_DSP_CMD_FLAG_RESPONSE_VALID) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid response with flags 0x%x!", xrp_dsp_comm_base->flags);
                req->result = XRP_STATUS_FAILURE;
                goto wait_for_response;
            }
            else if (pbst_cv->msg_manager.dsps[dsp].state == BST_CV_MSG_STOP) {
                goto worker_exit;
            }
#ifdef BST_CV_DEBUG
            else {
                do_gettimeofday(&end);
                BST_CV_STAGE_PRINTK("cmd%d roughly took %ld us", cmd_count,
                    (end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec);
                req->result = XRP_STATUS_SUCCESS;
            }
#endif
        }

        //copy out data field if inline
        if (req->queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE) {
            req->out = xrp_dsp_comm_base->out;
        }
        //update the current workload
        spin_lock_irqsave(&pbst_cv->msg_manager.worker_lock, flags);
        pbst_cv->msg_manager.dsps[dsp].workload--;
        spin_unlock_irqrestore(&pbst_cv->msg_manager.worker_lock, flags);
        complete(&req->complete);

#ifdef BST_CV_DEBUG
        cmd_count++;
#endif
    } while (1);

worker_exit:
    while (!kthread_should_stop()) {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule();
    } 
    BST_CV_STAGE_PRINTK("The worker thread of DSP %d exits", dsp);
    return 0;
}

/*
 * @func    bare_cv_cleaner
 * @brief   This is the cleaner thread function which handles the rrecycling of
 *          request metadata structures and their data buffers.
 * @params  args - the pointer to the bst_cv device
 * @return  0
 */
int bare_cv_cleaner(void *args) {
    struct bst_cv *pbst_cv = args;
    struct bst_cv_req *req;
    unsigned long flags;

    do {
        wait_for_completion(&pbst_cv->msg_manager.zombie_sem);
        if (pbst_cv->state == BST_CV_OFFLINE) {
            while(!list_empty(&pbst_cv->msg_manager.zombie_list)) {
                req = container_of(pbst_cv->msg_manager.zombie_list.next,
                    struct bst_cv_req, link);
                list_del(pbst_cv->msg_manager.zombie_list.next);
                pbst_cv->mem_manager.ops->free(req->block);
                devm_kfree(&pbst_cv->pdev->dev, req);
                BST_CV_TRACE_PRINTK("final_cleanup: req %px", req);
            }
            goto cleaner_exit;
        }
        else {
            spin_lock_irqsave(&pbst_cv->msg_manager.zl_lock, flags);
            req = container_of(pbst_cv->msg_manager.zombie_list.next,
                struct bst_cv_req, link);
            list_del(pbst_cv->msg_manager.zombie_list.next);
            spin_unlock_irqrestore(&pbst_cv->msg_manager.zl_lock, flags);
            pbst_cv->mem_manager.ops->free(req->block);
            devm_kfree(&pbst_cv->pdev->dev, req);
            BST_CV_TRACE_PRINTK("normal_cleanup: req %px", req);
        }
    } while (1);

cleaner_exit:
    while (!kthread_should_stop()) {
        schedule();
    } 
    BST_CV_STAGE_PRINTK("The cleaner thread exits");
    return 0;
}

/*
 * @func    bare_cv_req_dispatch
 * @brief   This function dispatches a request to one DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          queue - the pointer to the command queue metadata
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_req_dispatch(struct bst_cv *pbst_cv, struct xrp_ioctl_queue *queue)
{  
    int i;
    int ret;
    struct bst_cv_req *req;
    unsigned long flags;
    int min = MAX_WORKLOAD, target = -1;
    uint32_t in_data_alloc_size, out_data_alloc_size, buffer_alloc_size;
    void *in_data_copy_addr = NULL, *buffer_copy_addr = NULL;
    dsp_ptr dsp_addr;

    req = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*req), GFP_KERNEL);
    if (req == NULL) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "run out of kernel memory!");
        return -ENOMEM;
    }

    //initialize the request
    req->queue = *queue;
    init_completion(&req->complete);

    //justify if extra memory needs to be allocated
    if (req->queue.in_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE) {
        in_data_alloc_size = 0;
        in_data_copy_addr = &req->in;
    }
    else {
        in_data_alloc_size = req->queue.in_data_size;
    }
    if (req->queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE) {
        out_data_alloc_size = 0;
    }
    else {
        out_data_alloc_size = req->queue.out_data_size;
    }
    if (req->queue.buffer_size/sizeof(struct xrp_dsp_buffer) <= XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
        buffer_alloc_size = 0;
        buffer_copy_addr = &req->buf;
    }
    else {
        buffer_alloc_size = req->queue.buffer_size;
    }

    //allocate needed extra memory
    if (in_data_alloc_size + out_data_alloc_size + buffer_alloc_size > 0) {
        req->block = pbst_cv->mem_manager.ops->alloc(pbst_cv, in_data_alloc_size
            + out_data_alloc_size + buffer_alloc_size, 0);
        if (req->block == NULL) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "run out of reserved memory!");
            ret = -ENOMEM;
            goto fail_after_req_alloc;
        }
        BST_CV_TRACE_PRINTK("req data buffer: %px", req->block);

        //work out addresses for data to be copied into the allocated memory
        dsp_addr = kern_to_bus(pbst_cv, req->block->kern_addr);
        if (in_data_alloc_size > 0) {
            in_data_copy_addr = req->block->kern_addr;
            req->in.in_data_addr = dsp_addr;
        }
        if (out_data_alloc_size > 0) {
            req->out.out_data_addr = dsp_addr + in_data_alloc_size;
        }
        if (buffer_alloc_size > 0) {
            buffer_copy_addr = req->block->kern_addr + in_data_alloc_size + out_data_alloc_size;
            req->buf.buffer_addr = dsp_addr + in_data_alloc_size + out_data_alloc_size;
        }
    }

    //copy data
    ret = copy_from_user(in_data_copy_addr, (void *)req->queue.in_data_addr, req->queue.in_data_size);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fail to copy in_data!");
        goto fail_after_buffer_alloc;
    }
    ret = copy_from_user(buffer_copy_addr, (void *)req->queue.buffer_addr, req->queue.buffer_size);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fail to copy buffer!");
        goto fail_after_buffer_alloc;
    }

    //add the request into the hash table
    spin_lock_irqsave(&pbst_cv->msg_manager.req_lock, flags);
    req->queue.cookie = pbst_cv->msg_manager.next_cookie;
    pbst_cv->msg_manager.next_cookie++;
    hash_add(pbst_cv->msg_manager.req_ht, &req->ht_node, req->queue.cookie);
    spin_unlock_irqrestore(&pbst_cv->msg_manager.req_lock, flags);

    queue->cookie = req->queue.cookie;
    BST_CV_TRACE_PRINTK("req for cookie %d: %px", queue->cookie, req);

    //assign the request work to the DSP with the least workload
    spin_lock_irqsave(&pbst_cv->msg_manager.worker_lock, flags);
    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i] && pbst_cv->msg_manager.dsps[i].workload < MAX_WORKLOAD
            && pbst_cv->msg_manager.dsps[i].workload < min) {
            min = pbst_cv->msg_manager.dsps[i].workload;
            target = i;
        }
    }
    if (target >= 0) {
        unsigned long tmp_flags;

        pbst_cv->msg_manager.dsps[target].workload++;
        //add the request work into the work list of the target DSP
        spin_lock_irqsave(&pbst_cv->msg_manager.dsps[target].wl_lock, tmp_flags);
        list_add_tail(&req->link, &pbst_cv->msg_manager.dsps[target].work_list);
        spin_unlock_irqrestore(&pbst_cv->msg_manager.dsps[target].wl_lock, tmp_flags);
        spin_unlock_irqrestore(&pbst_cv->msg_manager.worker_lock, flags);
        complete(&pbst_cv->msg_manager.dsps[target].work_sem);
        BST_CV_TRACE_PRINTK("add the req to the work list of DSP %d", target);
        ret = 0;
        goto req_dispatch_end;
    }
    //if all DSPs are at full load
    else {
        spin_unlock_irqrestore(&pbst_cv->msg_manager.worker_lock, flags);
        spin_lock_irqsave(&pbst_cv->msg_manager.req_lock, flags);
        hash_del(&req->ht_node);
        spin_unlock_irqrestore(&pbst_cv->msg_manager.req_lock, flags);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "too many queues!");
        ret = -EBUSY;
    }

fail_after_buffer_alloc:
    pbst_cv->mem_manager.ops->free(req->block);
fail_after_req_alloc:
    devm_kfree(&pbst_cv->pdev->dev, req);
req_dispatch_end:
    return ret;
}

/*
 * @func    bare_cv_rsp_wait
 * @brief   This function waits for the response of the request corresponding to
 *          the input cookie
 * @params  pbst_cv - the pointer to the bst_cv device
 *          wait - the pointer to the wait metadata(cookie)
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_rsp_wait(struct bst_cv *pbst_cv, struct xrp_ioctl_wait *wait)
{
    struct hlist_node *tmp;
    struct bst_cv_req *req;
    unsigned long flags;

    //search through the hash table for the request
    spin_lock_irqsave(&pbst_cv->msg_manager.req_lock, flags);
    hash_for_each_possible_safe(pbst_cv->msg_manager.req_ht, req, tmp, ht_node, wait->cookie) {
        if (req->queue.cookie == wait->cookie) {
            int ret = 0;
            
            BST_CV_TRACE_PRINTK("wait for req %px with cookie %d", req, wait->cookie);
            spin_unlock_irqrestore(&pbst_cv->msg_manager.req_lock, flags);
            wait_for_completion(&req->complete);
            //copy the out data if success
            if (pbst_cv->state == BST_CV_ONLINE) {
                if (req->result == XRP_STATUS_SUCCESS) {
                    void *out_data_copy_addr;

                    if (req->queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE) {
                        out_data_copy_addr = &req->out;
                    }
                    else {
                        out_data_copy_addr = bus_to_kern(pbst_cv, req->out.out_data_addr);
                    }
                    ret = copy_to_user((void *)req->queue.out_data_addr, out_data_copy_addr,
                        req->queue.out_data_size);
                    if (ret < 0) {
                        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fail to copy out_data!");
                    }
                }
                else {
                    ret = -EFAULT;
                }
                spin_lock_irqsave(&pbst_cv->msg_manager.req_lock, flags);
                hash_del(&req->ht_node);
                spin_unlock_irqrestore(&pbst_cv->msg_manager.req_lock, flags);
                spin_lock_irqsave(&pbst_cv->msg_manager.zl_lock, flags);
                list_add_tail(&req->link, &pbst_cv->msg_manager.zombie_list);
                spin_unlock_irqrestore(&pbst_cv->msg_manager.zl_lock, flags);
                complete(&pbst_cv->msg_manager.zombie_sem);
            }
            return ret;
        }
    }
    spin_unlock_irqrestore(&pbst_cv->msg_manager.req_lock, flags);

    //target request not found
    BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no such queue or some other thread has been waiting!");
    return -EINVAL;
}

/*
 * @func    bare_cv_msg_manager_exit
 * @brief   This is the exit function of the message manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bare_cv_msg_manager_exit(struct bst_cv *pbst_cv)
{
    int i;
    struct bst_cv_req *req;
    struct hlist_node *tmp;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            pbst_cv->msg_manager.dsps[i].state = BST_CV_MSG_STOP;
            ipc_close(pbst_cv->msg_manager.dsps[i].ipc_session_id);
            //stop the worker thread
            complete(&pbst_cv->msg_manager.dsps[i].work_sem);
            kthread_stop(pbst_cv->msg_manager.dsps[i].worker);
        }
    }
    complete(&pbst_cv->msg_manager.zombie_sem);
    kthread_stop(pbst_cv->msg_manager.cleaner);
    //delete remaining requests which are not waited by some threads
    hash_for_each_safe(pbst_cv->msg_manager.req_ht, i, tmp, req, ht_node) {
        hash_del(&req->ht_node);
        pbst_cv->mem_manager.ops->free(req->block);
        devm_kfree(&pbst_cv->pdev->dev, req);
        BST_CV_TRACE_PRINTK("req %px", req);
    }
    return;
}
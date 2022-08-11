#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/timekeeping.h>

#include "ipc_communication_manager.h"
#include "ipc_mempool.h"
#include "ipc_nodemanager.h"
#include "ipc_session.h"

#include "ipc_mailbox_controller.h"
#include "ipc_msg_manager.h"
#include "user_head.h"
#include "ipc.h"
#include "ipc_common.h"

// macro
#define IPC_DRIVER_NAME "ipc_communication_mgr"
#define IPC_CHANNEL_MAX 8
#define Rx_Tx_WORK_QUEUE_FIFO_SIZE 40960

/********************* extern global variables *******************/
extern enum ipc_core_e ipc_channel[IPC_CORE_MAX];
extern struct diag_info diagnose_info[SESSION_NUM];
extern struct platform_device * g_ipc_platform_dev; //read only variable
extern struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr;

#ifdef MSG_DUMP
bool save_flag[IPC_CORE_MAX] = { false, false, false, false, false, false, false, false, false,
								 false, false, false, false, false, false, false, false, false,};
uint32_t save_cnt[IPC_CORE_MAX] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif

/********************* function declaration ***************************/
//use for ipc_sys
int32_t core_addr_init(void);
int32_t core_addr_exit(void);
static void msg_dispatch(enum ipc_core_e src, enum ipc_core_e dest, struct ipc_fill_register_msg *buf, u32 len, ktime_t timestamp);

/********************* local variables ***************************/
struct work_data {
    struct ipc_fill_register_msg data;  
    int32_t data_len;
    enum ipc_core_e src;
    enum ipc_core_e dest;
    ktime_t timestamp;
};

struct send_work_data {
    struct work_struct work;
    struct ipc_fill_register_msg data;
    int32_t session_id;
    int32_t data_len;
    enum ipc_core_e src;
    enum ipc_core_e dest;
};

//work queue variable definition
static struct workqueue_struct *ipc_send_wq[IPC_CHANNEL_MAX]; 
static struct workqueue_struct *ipc_recv_wq; 
static struct work_struct ipc_recv_work;    
//work queue msg fifo
spinlock_t in_lock;
spinlock_t out_lock;
//receive buffer definition
static struct kfifo pfifo_out;

/********************* global variables ***************************/
uint64_t signal_map[SUBSCRIPTION_MAP_MAX][MSG_CMD_MAX];
uint16_t register_list[REGISTER_MAP_MAX][MSG_CMD_MAX];

//send queue function
static void send_func(struct work_struct *work) 
{
    struct send_work_data *send_msg = (struct send_work_data *)work;
    struct ipc_client_info *client = NULL;
    int ret = 0;
#ifdef MSG_DUMP
    uint32_t row, col;
#endif

    if (unlikely(IS_ERR_OR_NULL(send_msg)))
    {
        IPC_LOG_ERR("send func msg is null or error");
        return;
    }

    IPC_LOG_INFO("send data.src = %d, dst = %d", send_msg->src, send_msg->dest);
    //node parse only depend on dst variable
    ret = ipc_node_msgparse(send_msg->dest, &client);
    if (ret < 0)
    {
        IPC_LOG_WARNING("send failed, Can not find dst %d client!", send_msg->dest);
        kfree(send_msg);
        return;
    }

    // NOTE:in current ipc version, src can not be defined by user, it is only defined in ipc driver
#ifdef IPC_SRC_DEFINITION_SUPPORT
    ret =  ipc_send_data(client, send_msg->src, &send_msg->data);
#else
    ret =  ipc_send_data(client, ipc_channel[send_msg->dest], &send_msg->data);
#endif
    if (!ret)
    {
        client->channel_status = CHANNEL_READY;
        //get session and completion
        struct ipc_session *session = get_session_by_id(send_msg->session_id);
        if (IS_ERR_OR_NULL(session))
        {
            IPC_LOG_WARNING("session %d is NULL", send_msg->session_id);
            kfree(send_msg);
            return;
        }

        //send completion to interface layer to return sending result to user.
        set_session_status(send_msg->session_id, SESSION_WAIT_SEND);
        complete(&session->tx_complete);

#ifdef MSG_DUMP
        if (save_flag[send_msg->dest] == true) {
            row = send_msg->dest;
            col = save_cnt[send_msg->dest] % IPC_INFO_MAX_NUM;
            g_ipc_all_cores_register_addr->data_saved[row][col] = (*(uint64_t *)(&(send_msg->data)));
            ++save_cnt[send_msg->dest];
        }
#endif

    }

    //free send message buffer
    kfree(send_msg);
}

//receive queue function
static void recv_func(struct work_struct *work)
{
    struct work_data work_data;
    //in receive function, we just need to take an empty from recv msg fifo
    while (!kfifo_is_empty(&pfifo_out))
    {
        kfifo_out(&pfifo_out, &work_data, sizeof(work_data));
        msg_dispatch(work_data.src, work_data.dest, &work_data.data, work_data.data_len, work_data.timestamp);
    }
}

static void msg_dispatch(enum ipc_core_e src, enum ipc_core_e dest, struct ipc_fill_register_msg *buf, u32 len, ktime_t timestamp)
{
	struct ipc_fill_register_msg *fill_msg = (struct ipc_fill_register_msg*)buf;
    struct ipc_drv_msg *ipc_drv_msg = NULL;
    struct ipc_session* ipc_session = NULL;
    int ret;

    if (unlikely(IS_ERR_OR_NULL(fill_msg)))
    {
        IPC_LOG_WARNING("fill_msg is null or error");
        return;
    }

#ifdef MSG_DUMP
    uint32_t row, col;
    if (save_flag[dest] == true) 
    {
        row = dest;
        col = save_cnt[dest] % IPC_INFO_MAX_NUM;
        g_ipc_all_cores_register_addr->data_saved[row][col] = (*(uint64_t *)(fill_msg));
        ++save_cnt[dest];
    }
#endif
    
    IPC_LOG_INFO("recv msg type = %d, src = %d, dst = %d, short_param = %d, long_param = %x", fill_msg->type, src, dest,
                fill_msg->short_param, fill_msg->long_param);
#ifdef MSG_SIZE_EXTENSION
    IPC_LOG_INFO("recv msg long_data = %x", fill_msg->long_data);
#endif

    /*
    recv message dispatcher algorithm: cmd + type + msg resource core
    1. method -> use cmd to seek session in the method subscription list;
    2. signal -> use cmd to seek session lists in the signal subscription list; 
        if this cmd do not have any subscription, we will push it to the first session created on the channel.(for compatibility considerations)
    3. reply -> use token to seek session in the sent_msg_queue;
    */

    // method dispatcher
    if (fill_msg->type == IPC_MSG_TYPE_METHOD)
    {
#ifdef MSG_SIZE_EXTENSION
        struct ipc_drv_msg ipc_drv_msg = {
            .msg.type = fill_msg->type,
            .msg.cmd = fill_msg->cmd,
            .msg.token = fill_msg->short_param,
            .msg.data = fill_msg->lon g_param,
            .msg.long_data = fill_msg->long_data,
            .msg.timestamp = timestamp,
        };
#else
        struct ipc_drv_msg ipc_drv_msg = {
            .msg.type = fill_msg->type,
            .msg.cmd = fill_msg->cmd,
            .msg.token = fill_msg->short_param,
            .msg.data = fill_msg->long_param,
            .msg.timestamp = timestamp,
        };
#endif 
        if (src >= IPC_CORE_R5_0)
        {
            IPC_LOG_WARNING("method dispatch core %d invalid", src);
            goto discard_msg;
        }

        //search register map to find session id
        int8_t session_id = register_list[src][fill_msg->cmd];
        IPC_LOG_INFO("register map session %d ", session_id);
        ipc_session = get_session_by_id(session_id);
        if (IS_ERR_OR_NULL(ipc_session))
        {
            IPC_LOG_WARNING("method dispatch session %d is invalid", session_id);
            goto discard_msg;
        }
        else
        {
            IPC_LOG_INFO("method push in session %d", ipc_session->id);
            ret = ipc_session_msg_in(ipc_session->id, ipc_drv_msg);
            if (ret < 0)
            {
                IPC_LOG_WARNING("method push in session %d  failed", ipc_session->id);
                goto discard_msg;
            }
            complete(&ipc_session->rx_complete);
            return;
        }
    }
    // signal dispatcher
    else if (fill_msg->type == IPC_MSG_TYPE_SIGNAL)
    {
#ifdef MSG_SIZE_EXTENSION
        struct ipc_drv_msg ipc_drv_msg = {
            .msg.type = fill_msg->type,
            .msg.cmd = fill_msg->cmd,
            .msg.token = fill_msg->short_param,
            .msg.data = fill_msg->long_param,
            .msg.long_data = fill_msg->long_data,
            .msg.timestamp = timestamp,
        };
#else
        struct ipc_drv_msg ipc_drv_msg = {
            .msg.type = fill_msg->type,
            .msg.cmd = fill_msg->cmd,
            .msg.token = fill_msg->short_param,
            .msg.data = fill_msg->long_param,
            .msg.timestamp = timestamp,
        };
#endif

        uint8_t subscribe_id[64];
        uint8_t valid_count = get_all_1bit(signal_map[src][fill_msg->cmd], subscribe_id);
        //signal subscription list check, if do not have session id, we will push message in first session
        if (valid_count == 0)
        {
            ipc_session = find_session_by_coreid(src);
            if (IS_ERR_OR_NULL(ipc_session))
            {
                IPC_LOG_WARNING("signal dispatch src %d do not have any session", src);
                goto discard_msg;
            }
            ret = ipc_session_msg_in(ipc_session->id, ipc_drv_msg);
            if (ret < 0)
            {
                IPC_LOG_WARNING("signal push in session %d  failed", ipc_session->id);
                goto discard_msg;
            }
            complete(&ipc_session->rx_complete);
            return;
        } 
        //find session ids and push message in them.
        else 
        {
            uint8_t session_id, count;
            for (count = 0; count < valid_count; count++)
            {
                session_id = subscribe_id[count];
                IPC_LOG_INFO("session_id %d push in signal", session_id);

                ipc_session = get_session_by_id(session_id);
                if (IS_ERR_OR_NULL(ipc_session))
                {
                    IPC_LOG_WARNING("signal dispatch session %d is NULL", session_id);
                    goto discard_msg;
                }
                else
                {
                    IPC_LOG_INFO("signal push in session %d", ipc_session->id);
                    ret = ipc_session_msg_in(ipc_session->id, ipc_drv_msg);
                    if (ret < 0)
                    {
                        IPC_LOG_WARNING("signal push in session %d failed", ipc_session->id);
                        goto discard_msg;
                    }
                    complete(&ipc_session->rx_complete);
                }
            }
            return;
        }
    }
    // reply message dispatcher
    else if(fill_msg->type == IPC_MSG_TYPE_REPLY)
    {
        struct ipc_drv_msg *ipc_drv_msg;

        ret = send_msg_out(fill_msg->short_param, &ipc_drv_msg);
        if (ret < 0)
        {
            IPC_LOG_WARNING("token %d find sent msg failed", fill_msg->short_param);
            goto discard_msg;
        }
        if(IS_ERR_OR_NULL(ipc_drv_msg))
        {
            IPC_LOG_WARNING("sender %d sent a message with invalid token %d", src, fill_msg->short_param);
            goto discard_msg;   
        }

        ipc_session = get_session_by_id(ipc_drv_msg->session_id);
        if (IS_ERR_OR_NULL(ipc_session))
        {
            IPC_LOG_WARNING("reply session %d is invalid!", ipc_drv_msg->session_id);
            goto discard_msg;
        }
        
        //update receive message
        ipc_drv_msg->msg.type = fill_msg->type;
        ipc_drv_msg->msg.token = fill_msg->short_param;
        ipc_drv_msg->msg.cmd = fill_msg->cmd;
        ipc_drv_msg->msg.data = fill_msg->long_param;
        ipc_drv_msg->msg.timestamp = timestamp;
#ifdef MSG_SIZE_EXTENSION
        ipc_drv_msg->long_data = fill_msg->long_data;
#endif

        //free alloced msg in send func 
        devm_kfree(&g_ipc_platform_dev->dev ,ipc_drv_msg);
        set_session_status(ipc_session->id, SESSION_RECEIVED);
        ret = ipc_session_msg_in(ipc_session->id, *ipc_drv_msg);
        if (ret < 0)
        {
            IPC_LOG_WARNING("reply push in session %d failed", ipc_session->id);
            goto discard_msg;
        }

        //use signal means all receive message from resource core
        complete(&ipc_session->rx_complete);
        return;
    }
    // invalid type
    else 
    {
        IPC_LOG_ERR("type is invalid , type = %d", fill_msg->type);
        return;
    }

discard_msg:
    IPC_LOG_WARNING("discard msg type = %d, src = %d, dst = %d, short_param = %d, long_param = %x", fill_msg->type, src, dest,
        fill_msg->short_param, fill_msg->long_param);
    return;
}

//communication layer init function
int32_t ipc_communication_create()
{
    int8_t ret = 0;
    uint8_t cnt = 0;

    //create multi-sending thread queue
    for(cnt = 0; cnt < IPC_CHANNEL_MAX; cnt++)
    { 
        ipc_send_wq[cnt] = create_singlethread_workqueue("ipc_send_wq"); 
        if (!ipc_send_wq[cnt])
        {
            IPC_LOG_ERR("ERROR: ipc_send_wq %d == NULL", cnt);
            return -1;
        }
    }

    //create signal recv thread queue
    ipc_recv_wq = create_singlethread_workqueue("ipc_recv_wq"); 
    if (!ipc_recv_wq)
    {
        IPC_LOG_ERR("ERROR: ipc_recv_wq ==NULL");
        return -1;
    }

    //init thread one time and use shared fifo to manage msg in and out
    spin_lock_init(&in_lock);
    spin_lock_init(&out_lock);

    ret = kfifo_alloc(&pfifo_out, sizeof(struct work_data) * Rx_Tx_WORK_QUEUE_FIFO_SIZE, GFP_KERNEL);

    //init workqueue
    INIT_WORK(&ipc_recv_work, recv_func);

    //TODO:core_addr_init need remove to other layer
    core_addr_init();

    //map init
    memset(signal_map, 0, sizeof(signal_map));
    memset(register_list, 0, sizeof(register_list));
    return ret;
}

int32_t ipc_communication_close()
{
    int i;
    for (i = 0; i < IPC_CHANNEL_MAX; i++)
    {
        flush_workqueue(ipc_send_wq[i]);   
        destroy_workqueue(ipc_send_wq[i]); 
    }

    flush_workqueue(ipc_recv_wq);   
    destroy_workqueue(ipc_recv_wq); 

    //release fifo
    if (&pfifo_out) 
    {
        kfifo_free(&pfifo_out);
    }
    core_addr_exit();
    return 0;
}

//ipc communication layer api
int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest, int32_t session_id, void *buf, uint32_t len)
{
    bool queue_status;
    struct send_work_data *send_msg;

    //search send thread queue
#ifdef IPC_SRC_DEFINITION_SUPPORT
    int32_t channel_id = src;
#else
    int32_t channel_id = ipc_channel[dest];
#endif
    IPC_LOG_INFO("send thread queue id = %d", channel_id);
    if (unlikely(channel_id > IPC_CHANNEL_MAX))
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("channel id is invalid");
        return IPC_SEND_ERR_INVALID_PARAM;
    }

    //alloc sending work buffer
    send_msg = kmalloc(sizeof(struct send_work_data), GFP_KERNEL);
    send_msg->data = *(struct ipc_fill_register_msg*)buf;
    send_msg->data_len = len;
    send_msg->src = src;
    send_msg->dest = dest;
    send_msg->session_id = session_id;

    INIT_WORK(&send_msg->work, send_func);
    queue_status = queue_work(ipc_send_wq[channel_id], &send_msg->work);
    if (!queue_status) {
        diagnose_info[session_id].send_err.queue_full += 1;
    }
    
    return queue_status ? 0 : -1;
}

int32_t ipc_drv_recv(enum ipc_core_e src, enum ipc_core_e dest, void *buf, uint32_t len)
{
    struct work_data work_data;
    int32_t ret;
    uint64_t flag;

    if (ipc_recv_wq == NULL)
        return 0;

    work_data.data = *(struct ipc_fill_register_msg*)buf;
    work_data.data_len = len;
    work_data.src = src;
    work_data.dest = dest;
    work_data.timestamp = ktime_get();

    //fifo in work_data
    spin_lock_irqsave(&out_lock, flag);
    if (kfifo_avail(&pfifo_out))
    {
        IPC_LOG_INFO("msg from %d kfifo in", src);
        ret = kfifo_in(&pfifo_out, &work_data, sizeof(work_data));   
        ret = queue_work(ipc_recv_wq, &ipc_recv_work);  
    }
    else
    {
        IPC_LOG_WARNING("fifo in is overrun");
        spin_unlock_irqrestore(&out_lock, flag);
        return -1;
    }

    spin_unlock_irqrestore(&out_lock, flag);
    return 0;
}


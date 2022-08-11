#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/ipc_interface.h>
#include "ipc_common.h"
#include "ipc_msg_manager.h"
#include "ipc_nodemanager.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_session.h"


#define IPC_DRIVER_NAME "ipc_session"
#define EACH_SESSION_RECV_MSG_FIFO_SIZE 2048

/********************* extern global variables *******************/
extern struct platform_device * g_ipc_platform_dev;
extern struct diag_info diagnose_info[SESSION_NUM];

/********************* local variables ***************************/
static DECLARE_RWSEM(list_lock);
static struct ipc_session *valid_session_list;
static uint64_t queue_buff[EACH_SESSION_RECV_MSG_FIFO_SIZE];
static atomic_t atomic_session_id = ATOMIC_INIT(0);
static struct ipc_session *session_map[SESSION_NUM];     // index of this array is session id
static uint64_t bitmap = 1; // session_map occupation status
static DEFINE_SPINLOCK(bitmap_lock);

int32_t request_a_session_id(void)    // bitmap algrithem
{
    int32_t ret;
    uint64_t flags;

    //session start from 1
    spin_lock_irqsave(&bitmap_lock, flags);
    ret = find_bit_zero(bitmap, 64, 0);
    if(ret >= 0)
        bitmap |= (1 << ret);
    spin_unlock_irqrestore(&bitmap_lock, flags);
    return ret;
}

IPC_SESSION_STATUS get_session_status(int32_t session_id)
{
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return SESSION_STATE_NULL; 
    }
    struct ipc_session* session = session_map[session_id];
    return session->status;
}

int32_t set_session_status(int32_t session_id, IPC_SESSION_STATUS status)
{
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return -1; 
    }
    struct ipc_session* session = session_map[session_id];
    session->status = status;
    return 0;

}

//register session
int32_t ipc_register_session(enum ipc_core_e src, enum ipc_core_e dst)
{
    struct ipc_session *session;
    struct kfifo recv_msg_fifo;
    int32_t valid_session_id;
    struct ipc_client_info *client_info = NULL;
    int ret;

    IPC_LOG_INFO("ipc_register_session, src = %d, dst = %d", src, dst);
    valid_session_id = request_a_session_id();
    IPC_LOG_INFO("valid_session_id = %d", valid_session_id);
    if(valid_session_id < 0 || valid_session_id >= SESSION_NUM)
    {
        IPC_LOG_ERR("All session ids are consumed, please close one first");
        return -1;
    }

    //get client via dst
    ret = ipc_node_get_node_of_coreid(dst, &client_info);
    if (ret < 0) {
		IPC_LOG_ERR("ipc get client by dst(%d) failed", dst);
		return IPC_INIT_ERR_INVALID_PARAM;
	}

    //alloc session mem
    session = devm_kzalloc(&g_ipc_platform_dev->dev, sizeof(*session), GFP_KERNEL);
    if (!session) {
	    IPC_LOG_ERR("session devm_kzalloc failed");
	    return -ENOMEM;
	}

   //alloc msg fifo mem
    ret = kfifo_alloc(&recv_msg_fifo, sizeof(struct ipc_drv_msg) * EACH_SESSION_RECV_MSG_FIFO_SIZE, GFP_KERNEL);
    session->id = valid_session_id;
    session->status = SESSION_INIT;
    session->recv_msg_fifo = recv_msg_fifo;
    session->src = src;
    session->dest = dst;
    session->waiting_reply_msg_token = -1;
    IPC_LOG_INFO("current->pid = %d", current->pid);
    session->pid_num = current->pid;
    session->cl_info = client_info;
    session->recv_callback = 0;

    init_completion(&session->rx_reply_complete);
    init_completion(&session->rx_signal_complete);
    init_completion(&session->rx_method_complete);
    init_completion(&session->tx_complete);
    init_completion(&session->rx_complete);
    
    session_map[session->id] = session;
    return valid_session_id;
}

bool ipc_session_valid(int32_t id)
{
    if(id >= SESSION_NUM || id <= 0)
    {
        return false;
    }
    
    if (session_map[id] == NULL)
        return false;
    else
        return true;
}

//this function can only support to find the first session of designated destination core
struct ipc_session *find_session_by_coreid(enum ipc_core_e dest)
{
    int32_t cnt;

    IPC_LOG_INFO("enter!, dest = %d", dest);
    for(cnt=0; cnt<SESSION_NUM; cnt++)
    {
        if (session_map[cnt] != NULL)
        {    
            if(session_map[cnt]->dest == dest)
            {
                return session_map[cnt];
            }
        }
    }

    return NULL;
}


struct ipc_session * get_session_by_id(int32_t id)
{
    IPC_LOG_INFO(" enter!, id = %d", id);
    if (id >= SESSION_NUM || id < 0)
    {
        IPC_LOG_ERR("session_id %d is invalid", id);
        return NULL;
    }

    return session_map[id];
}

struct ipc_session * ipc_session_by_pid(pid_t pid)
{
    int32_t cnt;

    IPC_LOG_INFO("enter!, pid = %d", pid);
    for(cnt = 0; cnt < SESSION_NUM; cnt++)
    {
        if (session_map[cnt] != NULL)
        {    
            if(session_map[cnt]->pid_num == pid)
            {
                return session_map[cnt];
            }
        }
    }

    return NULL;
}

int32_t ipc_session_msg_in(int32_t session_id, struct ipc_drv_msg msg)
{
    int32_t ret;
    IPC_LOG_INFO(" sessino msg in!, session_id = %d", session_id);
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return -1; 
    }

    struct ipc_session* session = session_map[session_id];
    if (session->status == SESSION_DESTROY)
    {
        IPC_LOG_WARNING("session is destroy, msg in fail");
        return -1; 
    }

    if (!kfifo_is_full(&session->recv_msg_fifo))
    {
        // IPC_LOG_INFO("kfifo in %d", kfifo_avail(&session->recv_msg_fifo));
        ret = kfifo_in(&session->recv_msg_fifo, &msg, sizeof(struct ipc_drv_msg));
    }
    else
    {
        IPC_LOG_WARNING("fifo is overrun");  
        return -1;
    }
    return 0;
}

int32_t ipc_session_msg_out(int32_t session_id, struct ipc_drv_msg* recv_msg)
{
    IPC_LOG_INFO(" sessino msg out!, session_id = %d", session_id);
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return -1; 
    }

    struct ipc_session* session = session_map[session_id];
    if (session->status == SESSION_DESTROY)
    {
        IPC_LOG_INFO("session %d is destroy, out msg fail", session_id);
        return -1;
    }

    kfifo_out(&session->recv_msg_fifo, recv_msg, sizeof(struct ipc_drv_msg));
    if (sizeof(recv_msg) == 0) {
        diagnose_info[session->id].recv_err.queue_empty += 1;
        IPC_LOG_WARNING("get msg out from session %d fail", session->id);
        return -1;
    }

    return 0;
}

int32_t ipc_session_destroy_by_id(int32_t session_id)
{
    IPC_LOG_INFO(" destroy session id %d", session_id);
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_WARNING("session id %d is invalid", session_id);
        return -1; 
    }

    struct ipc_session *p_sess;
    uint64_t flags;

    p_sess = session_map[session_id];

    //session status update
    p_sess->status = SESSION_DESTROY;

    //clear bitmap
    spin_lock_irqsave(&bitmap_lock, flags);
    bitmap &= ~(1ull<<session_id);
    spin_unlock_irqrestore(&bitmap_lock, flags);

    //clear completion to release wait
    complete(&p_sess->tx_complete);
    complete(&p_sess->rx_complete);
    complete(&p_sess->rx_signal_complete);
    complete(&p_sess->rx_method_complete);
    complete(&p_sess->rx_reply_complete);
    

    if (&p_sess->recv_msg_fifo)
    {
        kfifo_free(&p_sess->recv_msg_fifo);
    }

    devm_kfree(&g_ipc_platform_dev->dev,p_sess);

    session_map[session_id] = NULL;
    IPC_LOG_INFO("free session success!");

    return 0;
}

int32_t ipc_session_destroy_by_pid(pid_t id)
{
    IPC_LOG_INFO(" destroy session by pid %d", id);
    struct ipc_session *ipc_session = ipc_session_by_pid(id);
    if (ipc_session == NULL)
    {
        IPC_LOG_WARNING("get session fail by pid %d", id);
        return -1;
    }

    int ret = ipc_session_destroy_by_id(ipc_session->id);
    if (ret < 0)
    {
        IPC_LOG_WARNING("free session %d fail", ipc_session->id);
        return -1;
    }
    return 0;
}

uint32_t ipc_get_dst_by_session_id(int32_t session_id)
{
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return -1; 
    }
    struct ipc_session* session = session_map[session_id];
    return session->dest;
}

//this API only for userspace
uint32_t ipc_register_recv_notify_in_session(int32_t session_id, uint64_t cbfunc)
{
    if (!ipc_session_valid(session_id))
    {
        IPC_LOG_INFO("session id %d is invalid", session_id);
        return -1; 
    }
    struct ipc_session* session = session_map[session_id];
    session->recv_callback = (uint64_t) cbfunc;
    return 0;
}

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/hashtable.h>

#include "ipc_common.h"
#include "ipc_msg_manager.h"

#define IPC_DRIVER_NAME "ipc_msg_manager"
#define MSG_BIT_NUM			10
#define MSG_TOKEN_MAX	65534

/********************* extern global variables *******************/



/********************* local variables ***************************/
static DECLARE_RWSEM(sent_msg_hash_rwsem);
static atomic_t token_count = ATOMIC_INIT(1);
static DECLARE_HASHTABLE(msg_hash, MSG_BIT_NUM);
static DEFINE_SPINLOCK(token_lock);


int32_t send_msg_init(void)
{
    hash_init(msg_hash);
    return 0;
}

int32_t send_msg_destroy(void)
{

    // struct msg_node *p, *q;
    // if(!sent_msg_list)
    // {
    //     return -1;
    // }
    // down_write(&sent_msg_hash_rwsem);
    // list_for_each_entry_safe(p, q, &(sent_msg_list->list), list);
    // {
    //     list_del(&(p->list));
    //     devm_kfree(&g_ipc_platform_dev->dev, p);
    // }

    // list_del(&(sent_msg_list->list));
    // devm_kfree(&g_ipc_platform_dev->dev, sent_msg_list);
    // up_write(&sent_msg_hash_rwsem);

    return 0;
}

//sent message store in hash map
//TODO: max sent msg restriction???
int32_t send_msg_in(struct ipc_drv_msg* msg)
{
    if (!msg)
        return -1;

    IPC_LOG_INFO("send_msg_in, msg token = %d", msg->msg.token);
    down_write(&sent_msg_hash_rwsem);
    hash_add(msg_hash, &msg->node, msg->msg.token);
    up_write(&sent_msg_hash_rwsem);
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif 
    return 0;
}

int32_t send_msg_out(uint32_t token, struct ipc_drv_msg** msg)
{
    IPC_LOG_INFO("send_msg_out, token = %d", token);
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif 
    struct ipc_drv_msg* obj = NULL;

    if (msg && token > 0)
    {
        down_write(&sent_msg_hash_rwsem);
        // TODO hash_for_each_possible_safe
        hash_for_each_possible(msg_hash, obj, node, token) 
        {
            if(obj->msg.token == token) {
                *msg = obj;
                hash_del(&obj->node);
                up_write(&sent_msg_hash_rwsem);
                IPC_LOG_INFO("send_msg_out msg success");
                return 0;
            }
        }
        up_write(&sent_msg_hash_rwsem);
    }
    if (msg)
    {
        *msg = NULL;
    }
    IPC_LOG_INFO("send_msg_out fail");
    return -1;
}

int32_t send_msg_get(uint32_t token, struct ipc_drv_msg** msg)
{
    IPC_LOG_INFO("send_msg_get, token = %d", token);
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif 
    struct ipc_drv_msg* obj = NULL;

    if (msg && token > 0)
    {
        down_write(&sent_msg_hash_rwsem);
        // TODO hash_for_each_possible_safe
        hash_for_each_possible(msg_hash, obj, node, token) 
        {
            if(obj->msg.token == token) {
                *msg = obj;
                up_write(&sent_msg_hash_rwsem);
                IPC_LOG_INFO("send_msg_get msg success");
                return 0;
            }
        }
        up_write(&sent_msg_hash_rwsem);
    }
    if (msg)
    {
        *msg = NULL;
    }
    IPC_LOG_INFO("send_msg_get fail");
    return -1;
}

int32_t show_all_msgs(void)
{
    int32_t i=0;
    down_read(&sent_msg_hash_rwsem);
    for (i = 0; i < HASH_SIZE(msg_hash); ++i)
    {
        if (!hlist_empty(&msg_hash[i]))
        {
            IPC_LOG_INFO("bucket[%d]=> ", i);

            struct ipc_drv_msg* obj = NULL;
            hlist_for_each_entry(obj, &msg_hash[i], node) {
                IPC_LOG_INFO("token : %d, ", obj->msg.token);
            }
        }
        IPC_LOG_INFO("-----------------bucket %d end--------------------", i);
    }
    up_read(&sent_msg_hash_rwsem);
    return 0;
}

uint32_t ipc_msg_get_an_available_token(void)
{
    uint64_t flags;
    int32_t token;
    spin_lock_irqsave(&token_lock, flags);
    token = atomic_read(&token_count);
	atomic_inc(&token_count);
	if (atomic_read(&token_count) > MSG_TOKEN_MAX) {
		atomic_set(&token_count, 1);
	}
    spin_unlock_irqrestore(&token_lock, flags);
    return token;
}
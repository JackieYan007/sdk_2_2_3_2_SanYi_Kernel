#ifndef IPC_MAILBOX_CLIENT_H
#define IPC_MAILBOX_CLIENT_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>
#include <linux/miscdevice.h>
#include <linux/mailbox_client.h>
#include <linux/firmware.h>
#include <linux/types.h>

#include "ipc_common.h"
#include "ipc_nodemanager.h"
#include "user_head.h"

struct ipc_session;

/**
 * recv_t: receive function callback type
 *
 * @src: This core which send this message
 * @buf: Message data
 * @len: Message length
 */
typedef int32_t (*recv_t)(enum ipc_core_e src, enum ipc_core_e dest, void* buf, uint32_t len); //

enum ipc_channel_status_e{
    CHANNEL_READY,
    CHANNEL_SENDING,
    CHANNEL_SEND_SUCCESS,
    CHANNEL_SEND_FAIL,
};

enum ipc_client_status_e{
    CLIENT_STATUS_OFFLINE,
    CLIENT_STATUS_NOTREADY,
    CLIENT_STATUS_READY,
    CLIENT_STATUS_ONLINE,
};

/**
 * struct ipc_client_info: mailbox client information
 *
 * @recv_msg_callback: When this client received a message,
 *                     this function will be called.
 *
 */
struct ipc_client_info
{
    struct device* dev;
    enum ipc_core_e core_id;
    enum ipc_client_status_e status;
    enum ipc_channel_status_e channel_status;
    struct completion tx_complete;
    void __iomem * sem_reg;
    void __iomem* tx_reg;
};

#endif

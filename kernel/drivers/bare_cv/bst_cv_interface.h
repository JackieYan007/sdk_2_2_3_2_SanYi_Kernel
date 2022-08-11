#ifndef BST_CV_USER_H
#define BST_CV_USER_H

#include <linux/types.h>

#define XRP_IOCTL_MAGIC 'r'
#define XRP_IOCTL_ALLOC     _IO(XRP_IOCTL_MAGIC, 1)
#define XRP_IOCTL_FREE      _IO(XRP_IOCTL_MAGIC, 2)
#define XRP_IOCTL_QUEUE     _IO(XRP_IOCTL_MAGIC, 3)
#define XRP_IOCTL_WAIT      _IO(XRP_IOCTL_MAGIC, 4)
#define XRP_IOCTL_SYNC      _IO(XRP_IOCTL_MAGIC, 5)
#define XRP_IOCTL_INV_CACHE _IO(XRP_IOCTL_MAGIC, 6)

/******************************************************************************/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

/* user types */
typedef uint32_t dsp_ptr;

struct xrp_ioctl_alloc {
    __u32 size;
    __u32 align;
    __u32 addr;
    __u64 ptr;
    __u64 kaddr;
};

struct xrp_ioctl_queue {
    __u32 cookie;
    __u32 in_data_size;
    __u32 out_data_size;
    __u32 buffer_size;
    __u64 in_data_addr;
    __u64 out_data_addr;
    __u64 buffer_addr;
};

struct xrp_ioctl_wait {
    __u32 cookie;
};

/* dsp types */
#define XRP_DSP_CMD_INLINE_DATA_SIZE 16
#define XRP_DSP_CMD_INLINE_BUFFER_COUNT 1

struct xrp_dsp_buffer {
    /*
     * When submitted to DSP: types of access allowed
     * When returned to host: actual access performed
     */
    __u32 flags;
    __u32 size;
    __u32 addr;
};

union in_data {
    __u32 in_data_addr;
    __u8 in_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
};

union out_data {
    __u32 out_data_addr;
    __u8 out_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
};

union buffer_data {
    __u32 buffer_addr;
    struct xrp_dsp_buffer buffer_data[XRP_DSP_CMD_INLINE_BUFFER_COUNT];
    __u8 buffer_alignment[XRP_DSP_CMD_INLINE_DATA_SIZE];
};

struct xrp_dsp_cmd {
    __u32 flags;
    __u32 in_data_size;
    __u32 out_data_size;
    __u32 buffer_size;
    union in_data in;
    union out_data out;
    union buffer_data buf;
};

enum XRP_DSP_CMD_FLAG {
    XRP_DSP_CMD_FLAG_REQUEST_VALID = 0x00000001,
    XRP_DSP_CMD_FLAG_RESPONSE_VALID = 0x00000003,
};

enum XRP_DSP_SYNC {
    XRP_DSP_SYNC_IDLE = 0,
    XRP_DSP_SYNC_START = 0x100,
    XRP_DSP_SYNC_DSP_READY = 0x200,
    XRP_DSP_SYNC_HOST_TO_DSP = 0x1,
    XRP_DSP_SYNC_DSP_TO_HOST = 0x3,
};

enum XRP_DSP_SYNC_IRQ_MODE {
    XRP_DSP_SYNC_IRQ_MODE_NONE = 0x0,
    XRP_DSP_SYNC_IRQ_MODE_LEVEL = 0x1,
    XRP_DSP_SYNC_IRQ_MODE_EDGE = 0x2,
    XRP_DSP_SYNC_IRQ_MODE_BST_IPC = 0x3,
};

enum XRP_DSP_DEBUG_LEVEL {
    XRP_DSP_DBEUG_LEVEL_NO_TRACE = 0x0,
    XRP_DSP_DBEUG_LEVEL_1_TRACE = 0x1,
    XRP_DSP_DBEUG_LEVEL_2_TRACE = 0x2,
    XRP_DSP_DBEUG_LEVEL_MAX_TRACE = 0x3,
};

struct xrp_dsp_sync {
    __u32 sync;    //XRP base
    __u32 device_mmio_base;
    __u32 host_irq_mode;
    __u32 host_irq_offset;
    __u32 host_irq_bit;
    __u32 device_irq_mode;
    __u32 device_irq_offset;
    __u32 device_irq_bit;
    __u32 device_irq;
    
    // new added for bst openvx xrp(IPC)
    __u32 host_ipc_irq_clear;
    __u32 device_ipc_irq_clear;
    __u32 host_irq;
    __u32 host_ipc_irq_trig;
    __u32 device_ipc_irq_trig;
    __u32 debug_buffer_base;
    __u32 debug_buffer_length;
    __u32 debug_level;
    __u32 fw_version;
    __u32 ipc_host_to_dsp_addr;
    __u32 ipc_dsp_to_host_addr;
    __u32 device_ipc_irq_enable;
    __u32 device_ipc_irq_enable_mask;
};

#pragma pack(pop)   /* restore original alignment from stack */
/******************************************************************************/

#endif
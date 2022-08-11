/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bstn_user.h
 * @brief   This is the header file which contains the constants, type
 *          definitions which are shared with BSNN libarry.
 */

#ifndef BSTN_USER_H
#define BSTN_USER_H

/******************************************************************************/
#define BSTN_DRIVER_HDRNAME         "/dev/bstn" //without index

#include <linux/types.h>

typedef uint32_t dsp_ptr;

/******************************************************************************/
//user-space use struct or definition
#define BSTN_IOCTL_BUF              'B'
#define BSTN_IOCTL_BUF_ALLOC        _IO(BSTN_IOCTL_BUF, 1)
#define BSTN_IOCTL_BUF_FREE         _IO(BSTN_IOCTL_BUF, 2)
#define BSTN_IOCTL_BUF_SYNC         _IO(BSTN_IOCTL_BUF, 3)

#define BSTN_IOCTL_DMA_BUF          'D'
#define BSTN_IOCTL_DMA_BUF_IMPORT   _IO(BSTN_IOCTL_DMA_BUF, 1)
#define BSTN_IOCTL_DMA_BUF_RETURN   _IO(BSTN_IOCTL_DMA_BUF, 2)
#define BSTN_IOCTL_DMA_BUF_EXPORT   _IO(BSTN_IOCTL_DMA_BUF, 3)

#define BSTN_IOCTL_MSG              'M'
#define BSTN_IOCTL_MSG_SEND         _IO(BSTN_IOCTL_MSG, 1)

#define BSTN_IOCTL_VER              'V'
#define BSTN_IOCTL_VER_GET          _IO(BSTN_IOCTL_VER, 1)

#define BSTN_IOCTL_PERF             'P'
#define BSTN_IOCTL_PERF_GET         _IO(BSTN_IOCTL_PERF, 1)

#define BSTN_IOCTL_ASIC_TYPE        'A'
#define BSTN_IOCTL_ASIC_TYPE_GET    _IO(BSTN_IOCTL_ASIC_TYPE, 1)

#define RT_CMD_INIT                 0
#define RT_CMD_NET_RUN              1
#define RT_CMD_DAG_RUN              2

#define RT_STATUS_SUCCESS           0
#define RT_STATUS_FAILURE           1

/******************************************************************************/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */


/******************************************************************************/
//BSTN_IOCTL_MEM_ALLOC, BSTN_IOCTL_MEM_FREE
struct bsnn_buffer {
    uint32_t size;
    uint32_t align;
    void * uaddr;           //user address
    dsp_ptr baddr;          //bus address
    void *handle;
};

struct bstn_dma_buf {
       int fd;                                 // the dma-buf file descriptor
       dsp_ptr bus_addr;               // the dma-buf bus address
       uint32_t size;                  // the dma-buf size
};

/******************************************************************************/
//BSTN_IOCTL_MSG_SEND
struct bsnn_request {
    uint32_t opcode;        //opcode, BSTN_CMD_OPCODE
    dsp_ptr pdata;
};

struct bsnn_response {
    uint32_t status;        //message return result, BSTN_CMD_STATUS
};

struct bsnn_msg_exchange {
    struct bsnn_request req;
    struct bsnn_response rsp;
    uint32_t target_net;
};

struct bstn_ver_info {
    uint8_t driver_ver_major;
    uint8_t driver_ver_minor;
    uint8_t driver_ver_patch;
    uint8_t driver_release_month;
    uint8_t driver_release_date;
    uint16_t driver_release_year;
    uint8_t fw_ver_major;
    uint8_t fw_ver_minor;
    uint8_t fw_ver_patch;
    uint8_t fw_release_month;
    uint8_t fw_release_date;
    uint16_t fw_release_year;
};

struct bstn_perf_info {
    uint32_t dag_cnt_top; 	// DAG cycle count of NET TOP
    uint32_t dag_cnt_lite;	// DAG cycle count of NET LITE
};

#pragma pack(pop)   /* restore original alignment from stack */
//user-land use struct or definition end


#endif

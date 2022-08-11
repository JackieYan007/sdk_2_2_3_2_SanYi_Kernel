/*!
 * BST_LWNN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_lwnn_uapi.h
 * @brief   This is the header file which contains the constant and type
 *          definitions of API between the LWNN kernel driver and the userland
 *          library. lwnn_shared_interface.h must be included before inclusion
 *          of this header file.
 */

#ifndef BST_LWNN_UAPI_H
#define BST_LWNN_UAPI_H

#define BST_LWNN_DRIVER_DEVICE         "/dev/bst_lwnn" //without index

//user-space use struct or definition
#define BST_LWNN_IOCTL_BUF              'B'
#define BST_LWNN_IOCTL_BUF_ALLOC        _IO(BST_LWNN_IOCTL_BUF, 1)
#define BST_LWNN_IOCTL_BUF_FREE         _IO(BST_LWNN_IOCTL_BUF, 2)
#define BST_LWNN_IOCTL_BUF_SYNC         _IO(BST_LWNN_IOCTL_BUF, 3)

#define BST_LWNN_IOCTL_DMA_BUF          'D'
#define BST_LWNN_IOCTL_DMA_BUF_IMPORT   _IO(BST_LWNN_IOCTL_DMA_BUF, 1)
#define BST_LWNN_IOCTL_DMA_BUF_RETURN   _IO(BST_LWNN_IOCTL_DMA_BUF, 2)

#define BST_LWNN_IOCTL_MSG              'M'
#define BST_LWNN_IOCTL_MSG_XCHG         _IO(BST_LWNN_IOCTL_MSG, 1)

#define BST_LWNN_IOCTL_INFO             'I'
#define BST_LWNN_IOCTL_VER_INFO_GET     _IO(BST_LWNN_IOCTL_INFO, 1)
#define BST_LWNN_IOCTL_DSP_INFO_GET     _IO(BST_LWNN_IOCTL_INFO, 2)

/******************************************************************************/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

/******************************************************************************/
//BST_LWNN_IOCTL_BUF
struct bst_lwnn_user_buffer {
    unsigned int size;          //!< the buffer size
    unsigned int align;         //!< the buffer alignment
    void * user_addr;           //!< the buffer user address
    dsp_ptr bus_addr;           //!< the buffer bus address
};

/******************************************************************************/
//BST_LWNN_IOCTL_DMA_BUf
struct bst_lwnn_dma_buf {
    int fd;                     //!< the dma-buf file descriptor
    dsp_ptr bus_addr;           //!< the dma-buf bus address
};

/******************************************************************************/
//BST_LWNN_IOCTL_MSG
struct bst_lwnn_msg_xchg {
    struct bst_lwnn_req req;    //!< the command request body
    struct bst_lwnn_rsp rsp;    //!< the command response body
    int32_t target_dsp;         //!< the target DSP of the command
};

/******************************************************************************/
//BST_LWNN_IOCTL_INFO
struct _bst_lwnn_ver_info {
    uint8_t ver_major;
    uint8_t ver_minor;
    uint8_t ver_patch;
    uint8_t release_month;
    uint8_t release_date;
    uint16_t release_year;
};

struct bst_lwnn_ver_info {
    struct _bst_lwnn_ver_info drv;
    struct _bst_lwnn_ver_info fw;
};

struct bst_lwnn_dsp_info {
    uint32_t dsp_num;
    uint8_t dsp_online[BST_LWNN_MAX_DSP_NUM];
    uint8_t dsp_indices[BST_LWNN_MAX_DSP_NUM];
};

#pragma pack(pop)   /* restore original alignment from stack */
//user-land use struct or definition end

#endif
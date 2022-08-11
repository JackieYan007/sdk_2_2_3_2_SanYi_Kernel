/*
 * BST_LWNN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bst_lwnn_user.h
 * @brief   This is the header file which contains the constants, type
 *          definitions which are shared with BSNN libarry.
 */

#ifndef BST_LWNN_USER_H
#define BST_LWNN_USER_H

/******************************************************************************/
#define BST_LWNN_DRIVER_HDRNAME         "/dev/bst_lwnn" //without index

#include <linux/types.h>

typedef uint32_t dsp_ptr;

/******************************************************************************/
//user-space use struct or definition
#define BST_LWNN_IOCTL_BUF              'B'
#define BST_LWNN_IOCTL_BUF_ALLOC        _IO(BST_LWNN_IOCTL_BUF, 1)
#define BST_LWNN_IOCTL_BUF_FREE         _IO(BST_LWNN_IOCTL_BUF, 2)
#define BST_LWNN_IOCTL_BUF_SYNC         _IO(BST_LWNN_IOCTL_BUF, 3)

#define BST_LWNN_IOCTL_MSG              'M'
#define BST_LWNN_IOCTL_MSG_XCHG         _IO(BST_LWNN_IOCTL_MSG, 1)

// #define BST_LWNN_IOCTL_VER              'V'
// #define BST_LWNN_IOCTL_VER_GET          _IO(BST_LWNN_IOCTL_VER, 1)

#define RT_CMD_INIT                 0
#define RT_CMD_RUN_MODEL            1

#define RT_STATUS_SUCCESS           0
#define RT_STATUS_FAILURE           1

/******************************************************************************/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

/******************************************************************************/
//BST_LWNN_IOCTL_MEM_ALLOC, BST_LWNN_IOCTL_MEM_FREE
struct bst_lwnn_user_buffer {
    unsigned int size;
    unsigned int align;
    void * uaddr;           //user address
    dsp_ptr baddr;          //bus address
};

/******************************************************************************/
//BST_LWNN_IOCTL_MSG_SEND
struct bst_lwnn_req {
    uint32_t opcode;        //opcode, BST_LWNN_CMD_OPCODE
    dsp_ptr pdata;
};

struct bst_lwnn_rsp {
    uint32_t status;        //message return result, BST_LWNN_CMD_STATUS
};

struct bst_lwnn_msg_xchg {
    struct bst_lwnn_req req;
    struct bst_lwnn_rsp rsp;
    int32_t target_dsp;
};

struct bst_lwnn_ver_info {
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

#pragma pack(pop)   /* restore original alignment from stack */
//user-land use struct or definition end


#endif

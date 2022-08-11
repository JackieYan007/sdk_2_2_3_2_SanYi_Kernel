/*!
 *  This file contains proprietary information that is the sole intellectual
 *  property of Black Sesame Technologies, Inc. and its affiliates. No portions
 *  of this material may be reproduced in any form without the written
 *  permission of:
 * 
 *  Black Sesame Technologies, Inc. and its affiliates
 *  2255 Martin Ave. Suite D
 *  Santa Clara, CA  95050
 *  Copyright \@2018: all right reserved.
 *
 *  Notice:
 *
 *  You are running an EVALUATION distribution of the neural network tools
 *  provided by Black Sesame Technologies, Inc. under NDA. This copy may NOT be
 *  used in production or distributed to any third party. For distribution or
 *  production, further Software License Agreement is required.
 *
 * @file
 *  lwnn_shared_interface.h
 *
 * @brief
 *  This file contains constant and type definitions of LWNN shared API across
 *  the userland library, the kernel driver and the runtime firmware. Remember
 *  to include the header of uint32_t type before including this header.
 *
 * @author
 *  Shichen Lin
 */

#ifndef LWNN_SHARED_INTERFACE_H
#define LWNN_SHARED_INTERFACE_H

#ifndef BST_LWNN_MAX_DSP_NUM
#   define BST_LWNN_MAX_DSP_NUM    4
#elif BST_LWNN_MAX_DSP_NUM != 4
#   error BST_LWNN_MAX_DSP_NUM should be 4!
#endif

typedef uint32_t dsp_ptr;

// define shared constants instead of using enum to avoid field size uncertainty
#define RT_CMD_INIT                 0
#define RT_CMD_RUN_MODEL            1

#define RT_CMD_STATUS_SUCCESS       0
#define RT_CMD_STATUS_FAILURE       1

/******************************************************************************/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

struct bst_lwnn_req {
    uint32_t opcode;            //!< the command opcode, BST_RT_CMD_*
    dsp_ptr pdata;              //!< the bus address pointer to the command data
};

struct bst_lwnn_rsp {
    uint32_t status;            //!< the command return result, BST_RT_CMD_STATUS_*
};

#pragma pack(pop)   /* restore original alignment from stack */

#endif
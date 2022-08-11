/*******************************************************************************
 * Copyright(c) Black Sesame Technologies Inc., All right reserved.
 *
 * No part of this file may be distributed, duplicated or transmitted in any
 *form or by any means, without prior consent of Black Sesame Technologies Inc.
 *
 * This file is  property. It contains BST's trade secret, proprietary
 * and confidential information.
 *
 *********************************************************************************/


#ifndef __BST_ENCODER_IOCTL__
#define __BST_ENCODER_IOCTL__


/*resolution parm*/
struct encoder_parm {
    uint32_t width;               //the width of input and output data  
    uint32_t height;              //the height of input and output data
    uint32_t framerate;           //framerate
    uint32_t ratectrol_enable;  
    uint32_t ref_num;             //ref frame num
    void *sps_pps_addr;           //the addr for stores sps_pps
};



/*selfcheck mode parm*/
struct selfcheck_parm {
    void *yuv_addr;               //the addr of input yuv data in mem
    int yuv_size;                 //the size of input yuv data 
    void *y_addr;                 //the addr of input y 1/16 data in mem
    int  y_size;                  //the size of input y 1/16 data
    int framerate;                //framerate 
};



//V4L2_CID_MPEG_VIDEO_BITRATE         for bitrate set
//V4L2_CID_MPEG_VIDEO_GOP_SIZE        for gop set
//V4L2_CID_MPEG_VIDEO_H264_MAX_QP     for qp set
#define ENCODER_CID_RES_CONFIG              _IOW('E', V4L2_CID_PRIVATE_BASE + 1, struct encoder_parm)
#define ENCODER_CID_SELFCHECK_START         _IOW('E', V4L2_CID_PRIVATE_BASE + 2, struct selfcheck_parm)
#define ENCODER_CID_SELFCHECK_STOP          _IOW('E', V4L2_CID_PRIVATE_BASE + 3, struct selfcheck_parm)
#define ENCODER_CID_FRAMERATE_CFG           _IOW('E', V4L2_CID_PRIVATE_BASE + 4, struct encoder_parm)
#define ENCODER_CID_RC_ALG_CFG              _IOW('E', V4L2_CID_PRIVATE_BASE + 5, struct encoder_parm)
#define ENCODER_CID_SPS_PPS_RESEND          _IOW('E', V4L2_CID_PRIVATE_BASE + 6, struct encoder_parm)
#define ENCODER_CID_REF_CFG                 _IOW('E', V4L2_CID_PRIVATE_BASE + 7, struct encoder_parm)



#endif

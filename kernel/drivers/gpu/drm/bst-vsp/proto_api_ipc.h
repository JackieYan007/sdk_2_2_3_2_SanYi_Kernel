/*********************************************************************************
*This file contains proprietary information that is the sole intellectual
*property of Black Sesame Technologies, Inc. and its affiliates.  No portions
*of this material may be reproduced in any form without the written permission
*of:
*
*Black Sesame Technologies, Inc. and its affiliates
*2255 Martin Ave. Suite D
*Santa Clara, CA  95050
*Copyright @2016: all right reserved.
*/
/**********************************************************************************
* @file         proto_api_ipc.h
* Version:      @PACKAGE_VERSION@
* @brief        This is a brief description.
*/

#ifndef _SONE_PROTO_API_IPC_H
#define _SONE_PROTO_API_IPC_H

#define ID_CMD_ISP_CORE 0xF0
#define ID_CMD_VSP_CORE 0xF1
#define ID_MSG_ISP_CORE 0xF2
#define ID_MSG_VSP_CORE 0xF3

#if 0
enum ipc_type_e
{
  IPC_TYPE_METHOD,
  IPC_TYPE_INVOKE,
  IPC_TYPE_SIGNAL,
  IPC_TYPE_REPLY,
};
#endif

#endif

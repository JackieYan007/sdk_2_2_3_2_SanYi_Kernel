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


#ifndef __BST_ENCODER_IPC__
#define __BST_ENCODER_IPC__

#include <linux/string.h>
#include <linux/coreip/bst_vsp_msg.h>


void vsp_encode_cmd_start(void);
void vsp_encode_cmd_close(void);
void vsp_encode_cmd_frame_stop(uint32_t channel_id);
void vsp_encode_cmd_frame_start(uint32_t paddr[], uint32_t channel_id, uint32_t buff_id);
void vsp_encode_cmd_bitrate_cfg(uint32_t *param, uint32_t channel_id);
void vsp_encode_cmd_gop_cfg(uint32_t *param, uint32_t channel_id);
void vsp_encode_cmd_qp_cfg(uint32_t *param, uint32_t channel_id);

void vsp_encode_cmd_res_cfg(uint32_t width, uint32_t height, uint32_t channel_id);
void vsp_encode_cmd_framerate_cfg(uint32_t framerate, uint32_t channel_id);
void vsp_encode_cmd_sps_pps_resend(uint32_t paddr, uint32_t channel_id);
void vsp_encode_cmd_rc_alg_cfg(uint32_t rc_alg_enable, uint32_t channel_id);
void vsp_encode_cmd_ref_cfg(uint32_t ref_num, uint32_t channel_id);

void vsp_encode_cmd_selfcheck_start(uint32_t paddr[]);
void vsp_encode_cmd_selfcheck_stop(uint32_t paddr[]);
void encode_ipc_msg_recv(struct media_command *param);

#endif

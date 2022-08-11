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

#ifndef __BST_GMWARP_MSG_H__
#define __BST_GMWARP_MSG_H__

#include <linux/string.h>
#include <linux/coreip/proto_api_common.h>
#include "bst_gmwarp_drv.h"

void vsp_gmwarp_cmd_init_config(uint32_t *index, dma_addr_t *paddr);
void vsp_gmwarp_cmd_table_config(uint32_t *index, dma_addr_t *paddr);
void vsp_gmwarp_cmd_scaler_config(uint32_t *index, dma_addr_t *paddr);
void vsp_gmwarp_cmd_pitch_config(uint32_t *index, uint32_t paddr[]);
void vsp_gmwarp_cmd_crop_config(uint32_t *index, dma_addr_t *paddr);
void vsp_gmwarp_cmd_frame_start_plus(uint32_t paddr[], uint32_t channel_id,
				     uint32_t buff_id);

void vsp_gmwarp_cmd_selfcheck_start(uint32_t *index, uint32_t paddr[]);
void vsp_gmwarp_cmd_selfcheck_stop(uint32_t *index);

void gmwarp_ipc_msg_recv(struct media_command *param);
void vsp_gmwarp_cmd_frame_start(uint32_t paddr[], int plane_num,
				uint32_t channel_id, uint32_t buff_id);
void gmwarp_free_buf_handler(struct vb2_buffer *vbs, struct vb2_buffer *vbd,
			     uint32_t channel_id, uint32_t buff_id,
			     struct gmwarp_ctx *ctx);

#endif

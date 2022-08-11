/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ti_deser_hub for BST Cameras Driver
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef _BST_TIDESER_HUB_H_

#define _BST_TI_DESER_HUB_H_

#include "../../platform/bst-a1000/cam_entity.h"
#include "../../platform/bst-a1000/csi.h"
#include "common_deser_hub.h"
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>

#define MAX_CAMERAS_PER_TI954 2
#define MAX_CAMERAS_PER_TI960 4

#define TI_DESER_DEVICE_ID 0x00
#define TI_DESER_GENERAL_CONFIGURE 0X02
#define TI_DESER_RX_PORT_CTL 0X0c
#define TI_DESER_GPIO_INPUT_CTL 0X0f
#define TI_DESER_CSI_PLL_CTL 0x1f
#define TI_DESER_FWD_CTL1 0x20
#define TI_DESER_FWD_CTL2 0x21
#define TI_DESER_CSI_CTL1 0x33
#define TI_DESER_CSI_CTL2 0x34
#define TI_DESER_PORT_SELECT 0X4C
#define TI_DESER_RX_PORT_STS 0X4D
#define TI_DESER_BCC_CONFIG 0X58
#define TI_DESER_SER_ID 0x5B
#define TI_DESER_SER_ALIAS_ID 0X5c
#define TI_DESER_REMOTE_SLAVE 0X5d
#define TI_SLAVE_ALAIS_0_REG 0x65
#define TI_DESER_PORT_CONFIG 0X6d
#define TI_DESER_BC_GPIO_CTL0 0X6e
#define TI_DESER_PROT_CFG2 0X7C
#define TI_DESER_SCL_HIGH_TIME 0X0a
#define TI_DESER_SCL_LOW_TIME 0X0b

#define TI_SER_SCL_HIGH_TIME 0x0b
#define TI_SER_SCL_LOW_TIME 0x0c
#define TI_SER_GPIO_INPUT_CTL 0x0e

#define TI_DESER_SLAVE_ID5_REG 0x62 // for camera connect status, bit4~7 stand for port0~3
#define TI_DESER_SLAVE_ID6_REG 0x63 // for SOC A mipi status
#define TI_DESER_SLAVE_ID7_REG 0x64 // for SOC B mipi status
#define TI_DESER_CAMERA_CONNECT_REG TI_DESER_SLAVE_ID5_REG
//#define TI_DESER_SLAVE_ALIAS_ID6_REG 0x6b // for SOC A deser status
//#define TI_DESER_SLAVE_ALIAS_ID7_REG 0x6c // for SOC B deser status

#define CAMERA_STATUS_UN 0
#define CAMERA_STATUS_OK 1
#define CAMERA_STATUS_NG 2

#define SOC_A_MIPI_DONE (0xa << 1)
#define SOC_B_MIPI_DONE (0xb << 1)
//#define SOC_A_DESER_DONE (0x1a << 1)
//#define SOC_B_DESER_DONE (0x1b << 1)

#define CONFIG_IIC_MASTER 0x3e
#define CONFIG_OPEN_ALL 0x17 //0xbf
#define TI_DESER_GPIO_OPEN_ALL 0X7f
//#define TI_DESER_CSI_PLL_CTL_DEFAULT 0x02
#define TI_DESER_CSI_PLL_CTL_DEFAULT 0x00
#define TI_DESER_FWD_ALL_DEFAULT 0x00
#define TI_DESER_FWD_MODE_SELECT 0x03
#define TI_DESER_CSI_4LANES_CONTINUS_DEFAULT 0x03
#define TI_DESER_CSI_CALIBRATION_DEFAULT 0x49
#define TI_DESER_PORT_0_SELECT 0x01
#define TI_DESER_PORT_1_SELECT 0x12
#define TI_DESER_BCC_DEFAULT 0x18 // bc always on and crc enable
#define TI_DESER_BCC_FREQ_10MBPS 0x2 // 10Mbps
#define TI_DESER_BCC_FREQ_50MBPS 0x6 // 50Mbps
#define TI_DESER_PORT_CSI_VC_MAP 0x72
#define TI_DESER_PORT_CONFIG_DEFAULT 0x7c
#define TI_DESER_BC_GPIO_CTL0_DEFAULT 0x88
#define TI_DESER_PROT_CFG2_DEFAULT 0xc0
#define TI_DESER_FRAME_VALID_MIN_DEFAULT 0x10
#define TI_DESER_SER_ID_REG 0x5b

#define CSI_VC_MAP_DEFAULT_VALUE 0xe4

#define TI_DESER_STREAM_ENABLE_MASK 0x01
#define TI_DESER_STREAM_SUB_PORT_MASK 0xf0
#define TI_DESER_SUB_PORT_MAX 12

// freq, 0x0: 2.5Mbps, 0x2: 10Mbps, 0x6 50Mbps
int ti_serdes_bc_frequency_select(struct deser_hub_dev *deser_hub, int camera_index, u8 freq);

int ti_ser_gpio_input_ctrl_set(struct camera_dev *camera,
				uint8_t gpio_port, uint8_t input_enable);

int ti_ser_gpio_output_ctrl_set(struct camera_dev *camera,
				uint8_t gpio_port, uint8_t output_enable);

int ti_deser_hub_set_internal_frame_sync(
		struct deser_hub_dev *ti_deser_hub,
		int trigger_gpio,
		int fps);

int ti_deser_hub_set_external_frame_sync(struct deser_hub_dev *ti_deser_hub, int external_freq, int target_freq, int fsync_in, int fsync_out,
		int camera_trigger_gpio, int deser_trigger_gpio);

int ti_deser_hub_set_fwd_sync(struct deser_hub_dev *ti_deser_hub);

int ti_deser_hub_set_frame_valid_min(struct deser_hub_dev *ti_deser_hub, uint8_t value);

int read_camera_connect_status(struct deser_hub_dev *ti_deser_hub, int index);

int write_camera_connect_status(struct deser_hub_dev *ti_deser_hub, int index, int status);

int serdes_read(struct deser_hub_dev *ti_deser_hub, uint8_t reg);

static inline u8 ti_deser_lanes_num_to_bits(u8 lanes_num)
{
	return (4 - lanes_num);
}

#endif // _BST_TI_DESER_HUB_H_

/*
* DUMP_CANFD driver for BST DUMP_CANFD
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2021: all right reserved. 
*/

/*
* ChangeLog:
* Jul 2021: v1: create by fei.jing@bst.ai
*
*/

#ifndef CANFD_DUMP_H
#define CANFD_DUMP_H

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

#define BST_CANFD_0_BASE_ADDR	0x20016000
#define BST_CANFD_1_BASE_ADDR	0x20017000

#define CAN_GLBCTRL_REG       0x0000
#define CAN_TMCTRL0_REG       0x0004
#define CAN_TMCTRL1_REG       0x0008
#define CAN_ID_REG            0x000c
#define CAN_ID_MASK_REG       0x0010
#define CAN_SEND_ID_REG       0x0014

#define CAN_TX_DATA0_REG      0x0018
#define CAN_TX_DATA1_REG      0x001c
#define CAN_TX_DATA2_REG      0x0020
#define CAN_TX_DATA3_REG      0x0024
#define CAN_TX_DATA4_REG      0x0028
#define CAN_TX_DATA5_REG      0x002c
#define CAN_TX_DATA6_REG      0x0030
#define CAN_TX_DATA7_REG      0x0034
#define CAN_TX_DATA8_REG      0x0038 
#define CAN_TX_DATA9_REG      0x003c
#define CAN_TX_DATA10_REG     0x0040
#define CAN_TX_DATA11_REG     0x0044
#define CAN_TX_DATA12_REG     0x0048
#define CAN_TX_DATA13_REG     0x004c
#define CAN_TX_DATA14_REG     0x0050
#define CAN_TX_DATA15_REG     0x0054

#define CAN_RX_FIFO_DATA_REG  0x0058

#define CAN_RX_DATA0_REG      0x0058
#define CAN_RX_DATA1_REG      0x005c
#define CAN_RX_DATA2_REG      0x0060
#define CAN_RX_DATA3_REG      0x0064
#define CAN_RX_DATA4_REG      0x0068
#define CAN_RX_DATA5_REG      0x006c
#define CAN_RX_DATA6_REG      0x0070
#define CAN_RX_DATA7_REG      0x0074
#define CAN_RX_DATA8_REG      0x0078
#define CAN_RX_DATA9_REG      0x007c
#define CAN_RX_DATA10_REG     0x0080
#define CAN_RX_DATA11_REG     0x0084
#define CAN_RX_DATA12_REG     0x0088
#define CAN_RX_DATA13_REG     0x008c
#define CAN_RX_DATA14_REG     0x0090
#define CAN_RX_DATA15_REG     0x0094

#define CAN_TXERR_CNT_REG     0x0098
#define CAN_RXERR_CNT_REG     0x009c
#define CAN_REC_CTRLBIT_REG   0x00a0
#define CAN_REC_ID_REG        0x00a4
#define CAN_OVERWRITE_JUDGE_REG 0x00a8
#define CAN_IRQ_TYPE_REG      0x00ac
#define CAN_ERR_TYPE_REG      0x00b0
#define CAN_REC_TYPE_REG      0x00b4
#define CAN_STATUS_MASK_REG   0x00b8
#define CAN_ARB_LOST_CAPTURE_REG 0x00bc
#define CAN_STATUS_REG        0x00c0
#define CAN_PARITY_RESIDUAL_CTRL_REG 0x00c4
#define CAN_GLBCTRL1_REG      0x00c8
#define CAN_DMA_CTRL_REG      0x00d0
#endif

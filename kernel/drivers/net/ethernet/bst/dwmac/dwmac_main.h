/*
* DUMP_GMAC driver for BST DUMP_GMAC
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

#ifndef BST_DWMAC_MAIN_H
#define BST_DWMAC_MAIN_H

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

//GMAC0  GMAC1  base_addr
#define BST_GMAC_0_BASE_ADDR 0x30000000
#define BST_GMAC_1_BASE_ADDR 0x30100000

//GMAC_DMA_CH0  register  addr  offset
#define GMAC_DMA_CH0_CONTROL_REG                           0x1100

#define GMAC_DMA_CH0_TX_CONTROL_REG                        0x1104
#define GMAC_DMA_CH0_RX_CONTROL_REG                        0x1108

#define GMAC_DMA_CH0_TXDESC_LIST_HADDERSS_REG              0x1110
#define GMAC_DMA_CH0_TXDESC_LIST_ADDERSS_REG               0x1114
#define GMAC_DMA_CH0_RXDESC_LIST_HADDERSS_REG              0x1118
#define GMAC_DMA_CH0_RXDESC_LIST_ADDERSS_REG               0x111C

#define GMAC_DMA_CH0_TXDESC_TAIL_POINTERT_REG              0x1120
#define GMAC_DMA_CH0_RXDESC_TAIL_POINTERT_REG              0x1128

#define GMAC_DMA_CH0_TXDESC_RING_LENGTH_REG                0x112C
#define GMAC_DMA_CH0_RXDESC_RING_LENGTH_REG                0x1130

#define GMAC_DMA_CH0_INTERRUPT_ENABLE_REG                  0x1134

#define GMAC_DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER_REG       0x1138
#define GMAC_DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS_REG      0x113C

#define GMAC_DMA_CH0_CURRENT_APP_TXDESC_REG                0x1144
#define GMAC_DMA_CH0_CURRENT_APP_RXDESC_REG                0x114C

#define GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_H_REG            0x1150
#define GMAC_DMA_CH0_CURRENT_APP_TXBUFFER_REG              0x1154

#define GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_H_REG            0x1158
#define GMAC_DMA_CH0_CURRENT_APP_RXBUFFER_REG              0x115C

#define GMAC_DMA_CH0_STATUS_REG                            0x1160
#define GMAC_DMA_CH0_MISS_FRAME_CNT_REG                    0x1164

#define GMAC_DMA_CH0_RXP_ACCEPT_CNT_REG                    0x1168
#define GMAC_DMA_CH0_RX_ERI_CNT_REG                        0x116C 

#endif

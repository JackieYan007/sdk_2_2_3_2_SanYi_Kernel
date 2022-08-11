/*
* clock driver for BST A1000
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

#ifndef _DT_BINDINGS_BST_A1000_RESETS_H_
#define _DT_BINDINGS_BST_A1000_RESETS_H_

/* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0 */
#define RST_TOP_SW_RST0_START       (0)
#define RST_CODEC_SW                (0)
#define RST_CPU_SW                  (1)
#define RST_LPDDR0_SW               (2)
#define RST_LPDDR1_SW               (3)
#define RST_TOP_SW_RST0_END         (3)

/* TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST1 */
#define RST_TOP_SW_RST1_START       (4)
#define RST_USB3_SW                 (4)
#define RST_USB2_SW                 (5)
#define RST_ISP_SW                  (6)
#define RST_NET_SW                  (7)
#define RST_CV_SW                   (8)
#define RST_VSP_SW                  (9)
#define RST_PCIE_SW                 (10)
#define RST_GPU_SW                  (11)
#define RST_GDMA_SW                 (12)
#define RST_MIPI0_SW                (13)
#define RST_MIPI1_SW                (14)
#define RST_MIPI2_SW                (15)
#define RST_MIPI3_SW                (16)
#define RST_GMAC0_SW                (17)
#define RST_GMAC1_SW                (18)
#define RST_SDEMMC0_SW              (19)
#define RST_SDEMMC1_SW              (20)
#define RST_TOP_SW_RST1_END         (20)

/* SEC_SAFE_SYS_CTRL_R_SEC_SAFE_RESET_CTRL */
#define RST_SEC_SAFE_START          (21)
#define RST_QSPI1_SW                (21)
#define RST_QSPI0_SW                (22)
#define RST_SEC_SAFE_END            (22)


/* LB_LSP0_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
#define RST_LSP0_TOP_START          (23)
#define RST_LSP0_UART1_SW           (23)
#define RST_I2C0_SW                 (24)
#define RST_I2C1_SW                 (25)
#define RST_I2C2_SW                 (26)
#define RST_I2SM_SW                 (27)
#define RST_LSP0_GPIO_SW            (28)
#define RST_LSP0_UART0_SW           (29)
#define RST_SPI0_SW                 (30)
#define RST_WDT0_SW                 (31)
#define RST_WDT1_SW                 (32)
#define RST_LSP0_CAN_FD0_SW         (33)
#define RST_LSP0_CAN_FD1_SW         (34)
#define RST_LSP0_TIMER1_SW          (35)
#define RST_LSP0_TIMER0_SW          (36)
#define RST_LSP0_TOP_END            (36)

/* LB_LSP1_TOP_R_LSP0_LOCAL_SF_RST_CTRL_REG */
#define RST_LSP1_TOP_START          (37)
#define RST_LSP1_UART1_SW           (37)
#define RST_I2C5_SW                 (38)
#define RST_LSP1_GPIO_SW            (39)
#define RST_I2SS_SW                 (40)
#define RST_LSP1_UART0_SW           (41)
#define RST_LSP1_CAN_FD_SW          (42)
#define RST_SPI1_SW                 (43)
#define RST_I2C3_SW                 (44)
#define RST_I2C4_SW                 (45)
#define RST_WDT2_SW                 (46)
#define RST_WDT3_SW                 (47)
#define RST_LSP1_TIMER0_SW          (48)
#define RST_LSP1_TIMER1_SW          (49)
#define RST_LSP1_TOP_END            (49)

#endif /* _DT_BINDINGS_BST_A1000_RESETS_H_ */

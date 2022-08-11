/*
* DUMP_QSPI driver for BST DUMP_QSPI
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

#ifndef QSPI_DUMP_H
#define QSPI_DUMP_H

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

#define BST_QSPI_0_BASE_ADDR    0X00

/* Register offsets */
#define DW_QSPI_CTRL0			0x00
#define DW_QSPI_CTRL1			0x04
#define DW_QSPI_SSIENR			0x08
#define DW_QSPI_MWCR			0x0c
#define DW_QSPI_SER			    0x10
#define DW_QSPI_BAUDR			0x14
#define DW_QSPI_TXFLTR			0x18
#define DW_QSPI_RXFLTR			0x1c
#define DW_QSPI_TXFLR			0x20
#define DW_QSPI_RXFLR			0x24
#define DW_QSPI_SR			    0x28
#define DW_QSPI_IMR			    0x2c
#define DW_QSPI_ISR			    0x30
#define DW_QSPI_RISR			0x34
#define DW_QSPI_TXOICR			0x38
#define DW_QSPI_RXOICR			0x3c
#define DW_QSPI_RXUICR			0x40
#define DW_QSPI_MSTICR			0x44
#define DW_QSPI_ICR			    0x48
#define DW_QSPI_DMACR			0x4c
#define DW_QSPI_DMATDLR			0x50
#define DW_QSPI_DMARDLR			0x54
#define DW_QSPI_IDR			    0x58
#define DW_QSPI_VERSION			0x5c
#define DW_QSPI_DR			    0x60

#endif

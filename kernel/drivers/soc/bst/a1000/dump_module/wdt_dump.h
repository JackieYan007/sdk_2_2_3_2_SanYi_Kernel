/*
* DUMP_WDT driver for BST DUMP_WDT
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

#ifndef  BST_WDT_DUMP_H
#define  BST_WDT_DUMP_H

#define WDT_CNT  4

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

#define WDT_LSP_BASE(_n)				(0x2001A000 + (_n) * 0x1000) 

#define WDT_CR            0x0
#define WDT_TORR          0x4
#define WDT_CCVR          0x8
#define WDT_CRR           0xC      //A restart also clears the WDT interrupt.
#define WDT_STAT          0x10     //Interrupt Status Register   bit0: 0:inactive  1:active
#define WDT_EOI           0x14     //clear the interrupt without restarting the watchdog counter.
#define WDT_PROT_LEVEL    0x1C     //no
#define WDT_COMP_PARAM_5  0XE4
#define WDT_COMP_PARAM_4  0XE8
#define WDT_COMP_PARAM_3  0XEC
#define WDT_COMP_PARAM_2  0XF0
#define WDT_COMP_PARAM_1  0XF4
#define WDT_COMP_VERSI    0XF8
#define WDT_COMP_TYPE     0XFC
#endif
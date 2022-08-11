/*
* DUMP_GIC driver for BST DUMP_GIC
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

#ifndef IRQ_GIC_DUMP_H
#define IRQ_GIC_DUMP_H

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

#define GIC_BASE_ADDR              (0x32000000)
#define	GIC_DISTRIBUTOR_ADDR       (0x32001000)
#define	GIC_CPU_INTERCACE_ADDR     (0x32002000)

#define	GICD_CTLR                  (0x000)					
#define	GICD_TYPER                 (0x004)
#define	GICD_IIDR                  (0x008)

#define	GICC_CTLR                  (0x0000)
#define	GICC_PMR                   (0x0004)
#define	GICC_BPR                   (0x0008)
#define	GICC_IAR                   (0x000C)
#define	GICC_EOIR                  (0x0010)
#define	GICC_RPR                   (0x0014)
#define	GICC_HPPIR                 (0x0018)
#define	GICC_ABPR                  (0x001C)
#define	GICC_IIDR                  (0x0020)
#endif

/*
* DUMP_REGISTER driver for BST DUMP_REGISTER
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

/*
* ChangeLog:
* Jul 2021: v1: create by fei.jing@bst.ai
*
*/

#ifndef BST_DUMP_REG_H
#define BST_DUMP_REG_H

#include <stdbool.h>

int canfd_dump_register(void);
int qspi_dump_register(void);
void gmac_dump_register(void);
int gic_dump_register(void);
int pwm_dump_register(void);
int wdt_dump_register(void);
int usb2_device_dump_register(void);
int usb3_host_dump_register(void);
int usb3_device_dump_register(void);
#endif

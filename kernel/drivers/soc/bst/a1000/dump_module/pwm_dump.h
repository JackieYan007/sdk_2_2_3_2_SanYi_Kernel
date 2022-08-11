/*
* DUMP_PWM driver for BST DUMP_PWM
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

#ifndef PWM_DUMP_H
#define PWM_DUMP_H

#define SIZE_256B  0X100
#define SIZE_4K    0X1000
#define SIZE_8K    0X2000

#define TIMER_CNT  2

#define TIMER_BASE(_n)           (0x20012000 + (_n) * 0x1000)

#define TIMERNLOADCOUNT(_n)      (0X00 + (_n) * 0X14)
#define TIMERNCURRENTVALUE(_n)   (0X04 + (_n) * 0X14)
#define TIMERNCTROLREG(_n)       (0X08 + (_n) * 0X14)

#define TIMERNEOI(_n)            (0X0C + (_n) * 0X14)
#define TIMERNINTSTATUS(_n)      (0X10 + (_n) * 0X14)

#define TIMERNLOADCOUNT2(_n)     (0XB0 + (_n) * 0X04)
#define TIMER_N_PROT_LEVEL(_n)   (0XD0 + (_n) * 0X04)
#endif

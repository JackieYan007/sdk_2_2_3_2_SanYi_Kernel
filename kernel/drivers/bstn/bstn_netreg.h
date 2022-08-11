/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bstn_netreg.h
 * @brief   This file is the header file of the NET register control part of
 *          BSTN driver. It contains declarations of message handling functions
 *          which are in register level.
 */

#ifndef BSTN_NETREG_H
#define BSTN_NETREG_H

#define BST_NET_CORE_GLOBAL_REG         0x50000000
#define BST_NET_LITE_OFFSET             0x10000
#define BST_NET_SIGNATURE_CFG_0         0x1C
#define BST_NET_PERF_CNT_0              0x30 // cycle count for whole DAG

#define CHANNEL_TX_OFFSET       (0x0)
#define CHANNEL_RX_OFFSET       (0x10)

int bstn_netmsg_send(struct bstn_channel *channel, dsp_ptr ptrmsg, unsigned int timeout);
dsp_ptr bstn_netmsg_receive(struct bstn_channel *channel, unsigned int timeout);

#endif


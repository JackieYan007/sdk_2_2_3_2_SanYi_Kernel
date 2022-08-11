/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file 	bstn_channel_manager.h
 * @brief 	This file is the header file of the channel manager part of BSTN
 *          driver. It contains the channel related structure definition and
 *          the declarations of the initialization and cleanup functions.
 */

#ifndef BSTN_CHANNEL_H
#define BSTN_CHANNEL_H

#define BSTN_CHANNEL_REG_OFFSET     0x30
#define BSTN_CHANNEL_NUM            1
//20ms
#define BSTN_ACK_TIMEOUT_JIFFIES    20

struct bstn_channel {
    struct bstn_device *pbstn;
    struct bstn_channel_manager *channel_manager;
	void __iomem * channel_base;
	struct task_struct *msg_receiver_task;
    struct mutex mutex; //sender mutex
};

struct bstn_channel_manager {
    struct bstn_channel channels[BSTN_CHANNEL_NUM];
    unsigned char buf_start;
    unsigned char buf_end;
    dsp_ptr rsp_circular_buf[BSTN_EXCHANGE_NODE_NUM]; //response circular buffer
    spinlock_t buf_lock; //buffer lock
    struct completion complete;
};

int bstn_channel_send(struct bstn_device *pbstn, dsp_ptr pmsg, int channelid);
dsp_ptr bstn_channels_receive(struct bstn_device *pbstn);
int bstn_channel_manager_init(struct bstn_device *pbstn);
void bstn_channel_manager_exit(struct bstn_device *pbstn);

#endif
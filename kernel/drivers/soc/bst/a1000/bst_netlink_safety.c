// SPDX-License-Identifier: GPL-2.0
/*
 * BST netlink driver
 *
 * Copyright (C) 2021 Black Sesame Technologies, Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <net/sock.h>
#include <linux/netlink.h>

#define NETLINK_TEST     30
#define MSG_LEN          125
#define USER_PORT        100

struct sock *safety_nlsk = NULL;
extern struct net init_net;
struct safety_msgdata
{
	u32 fault_code;          /* error code */
	u32 strategy_code;       /* strategy code 1：ignore 2：restart 3：shutdown */
};

int send_safety_usrmsg(u32 fault_code, u32 strategy_code)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	struct safety_msgdata msg;
	int len, ret;

	len = sizeof(struct safety_msgdata);
	memset(&msg, 0, len);
	msg.fault_code = fault_code;
	msg.strategy_code = strategy_code;

	/* creat sk_buff */
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		printk(KERN_ERR "netlink alloc failure\n");
		return -1;
	}

	/* set netlink message header */
	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_TEST, len, 0);
	if (nlh == NULL) {
		printk(KERN_ERR "nlmsg_put failaure \n");
		nlmsg_free(nl_skb);
		return -1;
	}

	/* copy data and send  */
	memcpy(nlmsg_data(nlh), &msg, len);
	ret = netlink_unicast(safety_nlsk, nl_skb, USER_PORT, MSG_DONTWAIT);

	return ret;
}
EXPORT_SYMBOL_GPL(send_safety_usrmsg);

static void netlink_rcv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	char *umsg = NULL;

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		umsg = NLMSG_DATA(nlh);
		if (umsg) {
			pr_debug("kernel recv from user: %s\n", umsg);
			send_safety_usrmsg(0xa0090000, 0x345678);
		}
	}
}

struct netlink_kernel_cfg cfg = { 
	.input  = netlink_rcv_msg, /* set recv callback */
};  

int safety_netlink_init(void)
{
	struct safety_msgdata kmsg;
	memset(&kmsg, 0, sizeof(struct safety_msgdata));
	/* create netlink socket */
	safety_nlsk = (struct sock *)netlink_kernel_create(&init_net, NETLINK_TEST, &cfg);
	if (NULL == safety_nlsk) {   
		printk(KERN_ERR "safety netlink_kernel_create error !\n");
		return -1; 
	}  
	pr_debug(KERN_INFO "%s, %d\n", __func__, __LINE__);

	return 0;
}

void safety_netlink_exit(void)
{
	if (safety_nlsk) {
		netlink_kernel_release(safety_nlsk);
		safety_nlsk = NULL;
	}   
	pr_debug(KERN_INFO "%s, %d\n", __func__, __LINE__);
}

module_init(safety_netlink_init);
module_exit(safety_netlink_exit);

MODULE_AUTHOR("changjun.liao");
MODULE_DESCRIPTION("BST Safety Netlink Driver");
MODULE_LICENSE("GPL v2");

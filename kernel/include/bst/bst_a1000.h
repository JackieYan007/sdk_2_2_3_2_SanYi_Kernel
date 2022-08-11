/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __BST_A1000_H
#define __BST_A1000_H

#include <linux/types.h>
#include <linux/err.h>

/* bst netlink safety */
#ifdef CONFIG_BST_KERNEL_NETLINK
extern int send_safety_usrmsg(u32 fault_code, u32 strategy_code);
#else
static int send_safety_usrmsg(u32 fault_code, u32 strategy_code)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_BST_ERROR_NOTIFIER
extern int call_error_notifiers(unsigned long val, void *v);
#else
static inline void call_error_notifiers(unsigned long val, void *v)
{
}
#endif

#endif


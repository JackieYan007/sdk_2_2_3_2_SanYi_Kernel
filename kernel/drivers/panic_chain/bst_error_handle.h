#ifndef __BST_ERROR_HANDLE_H
#define __BST_ERROR_HANDLE_H

#include <linux/netlink.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/string.h>
#include <asm/ptrace.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <asm/traps.h>
#include <asm/stacktrace.h>
#include <linux/uaccess.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <asm/esr.h>

enum NOTIFIER_CHAIN_STA
{
    EVENT_START,
    EVENT_TEST,
    EVENT_DIE,
    EVENT_WARN,
    EVENT_PANIC,
    EVENT_REBOOT,
    EVENT_BUG,
    EVENT_HUNG_TASK,
    EVENT_SOFT_LOCKUP,
    EVENT_USER_FAULTS,
    EVENT_INFO_UNAVA,
    EVENT_COUNT,
};

extern char *pErrorData;

#define eos(s)      ((s)+strlen(s))
#define res(s)      (ERROR_DATA_SIZE-strlen(s))
#define bst_print(fmt, ...) \
	snprintf(eos(pErrorData),res(pErrorData),fmt, ##__VA_ARGS__)

#define NR_ERROR            (EVENT_COUNT)
#define USER_PORT           100
#define ERROR_DATA_SIZE     5000
#define NETLINK_ERROR       21

typedef struct {
	struct pt_regs regs;
	unsigned int esr;
	char comm[17];
	int pid;
	const char *str;
}user_fault_s;

void bst_warn_handle(void *data);
void bst_test_handle(void *data);
void bst_panic_handle(void *data);
void bst_die_handle(void *data);
void bst_user_faults_handle(void *data);
#endif // !__BST_ERROR_HANDLE_H

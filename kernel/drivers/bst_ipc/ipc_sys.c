/*
 * Sample kobject implementation
 *
 * Copyright (C) 2004-2007 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2007 Novell Inc.
 *
 * Released under the GPL version 2 only.
 *
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>

#include "ipc_common.h"
#include "ipc_communication_manager.h"
#include "ipc_mailbox_controller.h"
#include "ipc_interface.h"

#define IPC_DRIVER_NAME "ipc-sys"

/********************* extern global variables *******************/
extern uint64_t g_ipc_all_cores_register_addr_uaddr;
extern struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr;
extern void* get_kaddr_from_phy(phys_addr_t phy);
extern struct diag_info diagnose_info[SESSION_NUM];

/********************* local variables ***************************/
static uint64_t phy_addr_val;
static int32_t s_log_level = 1; // default 100,big enough to enable all logs

static int32_t info;

/*
 * This module shows how to create a simple subdirectory in sysfs called
 * /sys/kernel/kobject-example  In that directory, 3 files are created:
 * "foo", "baz", and "bar".  If an integer is written to these files, it can be
 * later read out of it.
 */

static int32_t foo;
static int32_t baz;
static int32_t bar;

int32_t sysfs_get_log_level(void)
{
	return s_log_level;
}

/**
 * The "info" file where a static variable is read from and written to.
 */

static ssize_t info_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", info);
}

static ssize_t info_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int32_t ret;

	ret = kstrtoint(buf, 10, &info);
	if (ret < 0) {
		IPC_LOG_ERR("kstrtoint %s to int failed", buf);
		return ret;
	}

	int i;
	IPC_INFO_PRINT("                                    |                 %-29s|                   %-30s", "send_error", "recv_error");
	IPC_INFO_PRINT("%-11s%-4s%-4s%-9s%-9s%-13s%-16s%-7s%-11s%-13s%-16s%-8s%-12s", 
					"session_id", "src", "dst", "send_msg", "recv_msg",
					"ipc_no_ready", "session_invalid", "No_ACK", "queue_full",
					"ipc_no_ready", "session_invalid", "timeout", "queue_empty");
	
	struct diag_info te = diagnose_info[0];
	// IPC_INFO_PRINT("%-11d%-4d%-4d%-9d%-9d%-13d%-16d%-7d%-11d%-13d%-16d%-8d%-12d",
	// 		 te.session_id, te.src, te.dst, te.num_of_send_msg, te.num_of_recv_msg,
	// 		 te.send_err.ipc_no_ready, te.send_err.session_invalid, te.send_err.no_ACK, te.send_err.queue_full,
	// 		 te.recv_err.ipc_no_ready, te.recv_err.session_invalid, te.recv_err.timeout, te.recv_err.queue_empty);
	for (i=1; i<SESSION_NUM; i++) {
		if (diagnose_info[i].session_id > 0 && diagnose_info[i].session_id < SESSION_NUM) {
			te = diagnose_info[i];
				IPC_INFO_PRINT("%-11d%-4d%-4d%-9d%-9d%-13d%-16d%-7d%-11d%-13d%-16d%-8d%-12d",
						te.session_id, te.src, te.dst, te.num_of_send_msg, te.num_of_recv_msg,
						te.send_err.ipc_no_ready, te.send_err.session_invalid, te.send_err.no_ACK, te.send_err.queue_full,
						te.recv_err.ipc_no_ready, te.recv_err.session_invalid, te.recv_err.timeout, te.recv_err.queue_empty);
		}
	}

	return count;
}

static struct kobj_attribute info_attribute =
	__ATTR(info, 0664, info_show, info_store);


/*
 * The "foo" file where a static variable is read from and written to.
 */
static ssize_t foo_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", foo);
}

static ssize_t foo_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int32_t ret;

	ret = kstrtoint(buf, 10, &foo);
	if (ret < 0)
		return ret;

	return count;
}

/* Sysfs attributes cannot be world-writable. */
static struct kobj_attribute foo_attribute =
	__ATTR(foo, 0664, foo_show, foo_store);

/*
 * More complex function where we determine which variable is being accessed by
 * looking at the attribute for the "baz" and "bar" files.
 */
static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int32_t var;

	if (strcmp(attr->attr.name, "baz") == 0)
		var = baz;
	else
		var = bar;
	return sprintf(buf, "%d\n", var);
}

static ssize_t b_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int32_t var, ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (strcmp(attr->attr.name, "baz") == 0)
		baz = var;
	else
		bar = var;
	return count;
}

static ssize_t all_cores_register_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	return sprintf(buf, "0x%x\n", g_ipc_all_cores_register_addr_uaddr);
}

static ssize_t log_level_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	return sprintf(buf, "%d\n", s_log_level);
}

static ssize_t log_level_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
    int32_t ret = 0;
    ret = kstrtoint(buf, 10, &s_log_level);
    if(ret<0)
    {
        IPC_LOG_ERR("kstrtoll %s to int failed", buf);
        return ret;
    }

	return count;
}

static ssize_t all_cores_register_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
    int32_t ret;
    ret = kstrtoll(buf, 16, &phy_addr_val);
    if(ret<0)
    {
        IPC_LOG_ERR("kstrtoll %s to long long failed", buf);
        return ret;
    }

    char* kaddr = get_kaddr_from_phy(phy_addr_val);
    if(NULL == kaddr)
    {
        IPC_LOG_ERR("get_kaddr_from_phy failed 0x%px", phy_addr_val);
        return -ENOMEM;
    }
    int32_t i;
    #define LINE_SIZE 8
	for(i = 0; i < sizeof(*g_ipc_all_cores_register_addr)/LINE_SIZE; i++) {
		printk("0x%px: %02x%02x%02x%02x%02x%02x%02x%02x    %c%c%c%c%c%c%c%c\n",
			phy_addr_val + i * LINE_SIZE,
			*(kaddr + i * LINE_SIZE + 0),*(kaddr + i * LINE_SIZE + 1),
			*(kaddr + i * LINE_SIZE + 2),*(kaddr + i * LINE_SIZE + 3),
			*(kaddr + i * LINE_SIZE + 4),*(kaddr + i * LINE_SIZE + 5),
			*(kaddr + i * LINE_SIZE + 6),*(kaddr + i * LINE_SIZE + 7),
			*(kaddr + i * LINE_SIZE + 0),*(kaddr + i * LINE_SIZE + 1),
			*(kaddr + i * LINE_SIZE + 2),*(kaddr + i * LINE_SIZE + 3),
			*(kaddr + i * LINE_SIZE + 4),*(kaddr + i * LINE_SIZE + 5),
			*(kaddr + i * LINE_SIZE + 6),*(kaddr + i * LINE_SIZE + 7)
		);
	}

	return count;
}

static struct kobj_attribute baz_attribute =
	__ATTR(baz, 0664, b_show, b_store);
static struct kobj_attribute bar_attribute =
	__ATTR(bar, 0664, b_show, b_store);

static struct kobj_attribute g_all_core_register_addr_attribute =
	__ATTR(g_ipc_all_cores_register_addr_uaddr,
        0664, all_cores_register_show, all_cores_register_store);

static struct kobj_attribute s_log_level_attribute =
	__ATTR(s_log_level, 0664, log_level_show, log_level_store);
/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
	&info_attribute.attr,
	&foo_attribute.attr,
	&baz_attribute.attr,
	&bar_attribute.attr,
    &g_all_core_register_addr_attribute.attr,
    &s_log_level_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *example_kobj;

int32_t core_addr_init(void)
{
	int32_t retval;

	/*
	 * Create a simple kobject with the name of "kobject_example",
	 * located under /sys/kernel/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
	//example_kobj = kobject_create_and_add("kobject_example", kernel_kobj);
    example_kobj = kobject_create_and_add("bst-ipc", kernel_kobj);
	if (!example_kobj)
    {
        IPC_LOG_ERR("kobject_create_and_add failed");
		return -ENOMEM;
    }

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(example_kobj, &attr_group);
	if (retval)
    {
        IPC_LOG_ERR("sysfs_create_group failed, error code = %d", retval);
		kobject_put(example_kobj);
    }

	return retval;
}

void core_addr_exit(void)
{
	kobject_put(example_kobj);
}


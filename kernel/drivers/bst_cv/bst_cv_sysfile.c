/*!
 * BST_CV: Linux device driver for Blck Sesame Technologies Neural Network IP
 * @author Shichen Lin (shichen.lin@bst.ai)
 *
 * @file   bst_cv_misc.c
 * @brief  This file is the source code file of sysfs file interface of BST_CV
 *         driver. It contains function definitions of ioctl callbacks and
 *         initialization of the sysfs files.
 */

#include "bst_cv.h"

int bst_cv_print_level = BST_CV_LOG_PRINT;
int bst_cv_firmware_reset = 0;
int bst_cv_firmware_log_buffer = 0;
int bst_cv_firmware_log_length = 0;
int bst_cv_firmware_log_level = 0;

static ssize_t bst_cv_print_level_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    switch (bst_cv_print_level) {
        case BST_CV_DEBUG_PRINT:
            return sprintf(buf, "Current BST_CV Print Level: BST_CV_DEBUG_PRINT(2)\nSet 0 to BST_CV_NO_PIRNT or 1 to BST_CV_LOG_PRINT\n");
        case BST_CV_LOG_PRINT:
            return sprintf(buf, "Current BST_CV Print Level: BST_CV_LOG_PRINT(1)\nSet 0 to BST_CV_NO_PIRNT or 2 to BST_CV_DEBUG_PRINT\n");
        case BST_CV_NO_PRINT:
            return sprintf(buf, "Current BST_CV Print Level: BST_CV_NO_PRINT(0)\nSet 1 to BST_CV_LOG_PIRNT or 2 to BST_CV_DEBUG_PRINT\n");
        default: {
            struct bst_cv *pbst_cv;

            pbst_cv = container_of((void *)object, struct bst_cv, kobj);
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fatal error! invalid print level");
            return 0;
        }
    }
}

static ssize_t bst_cv_print_level_attr_store(struct kobject *object,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int new_print_level;
    int ret;

    ret = kstrtoint(buf, 0, &new_print_level);
    if (ret < 0 || new_print_level < 0 || new_print_level > BST_CV_DEBUG_PRINT) {
        struct bst_cv *pbst_cv;

        pbst_cv = container_of((void *)object, struct bst_cv, kobj);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "tried to set an invalid print level");
    }
    else {
        bst_cv_print_level = new_print_level;
    }
    return count;
}

static ssize_t bst_cv_firmware_log_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "0x%08x 0x%08x %d\n", bst_cv_firmware_log_buffer, bst_cv_firmware_log_length, bst_cv_firmware_log_level);
}

static ssize_t bst_cv_firmware_log_attr_store(struct kobject *object,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int log_buf, log_len, log_level;

    if (sscanf(buf, "%x %x %u", &log_buf, &log_len, &log_level) == 3) {
        bst_cv_firmware_log_buffer = log_buf;
        bst_cv_firmware_log_length = log_len;
        bst_cv_firmware_log_level = log_level;
    } else {
        struct bst_cv *pbst_cv;
        pbst_cv = container_of((void *)object, struct bst_cv, kobj);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid firmware log buffer");
    }
    return count;
}

static ssize_t bst_cv_firmware_reset_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", bst_cv_firmware_reset);
}

static ssize_t bst_cv_firmware_reset_attr_store(struct kobject *object,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int new_reset;
    int ret;

    ret = kstrtoint(buf, 0, &new_reset);
	/* bit0: DSP0, bit1: DSP1, bit2: DSP2, bit3: DSP3 */
    if (ret < 0 || new_reset < 0 || new_reset > 0x0f) {
        struct bst_cv *pbst_cv;

        pbst_cv = container_of((void *)object, struct bst_cv, kobj);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "tried to set an invalid reset value");
    }
    else {
        bst_cv_firmware_reset = new_reset;
    }
    return count;
}

static struct kobj_attribute bst_cv_print_level_attr = __ATTR(bst_cv_print_level,
    0664, bst_cv_print_level_attr_show, bst_cv_print_level_attr_store);

static struct kobj_attribute bst_cv_firmware_log_attr = __ATTR(bst_cv_firmware_log,
    0664, bst_cv_firmware_log_attr_show, bst_cv_firmware_log_attr_store);

static struct kobj_attribute bst_cv_firmware_reset_attr = __ATTR(bst_cv_firmware_reset,
    0664, bst_cv_firmware_reset_attr_show, bst_cv_firmware_reset_attr_store);

static struct attribute *bst_cv_kobj_attrs[] = {
	&bst_cv_print_level_attr.attr,
	&bst_cv_firmware_log_attr.attr,
	&bst_cv_firmware_reset_attr.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group bst_cv_kobj_attr_group = {
	.attrs = bst_cv_kobj_attrs,
};

static void dynamic_kobj_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
	kfree(kobj);
}

static struct kobj_type dynamic_kobj_ktype = {
	.release	= dynamic_kobj_release,
	.sysfs_ops	= &kobj_sysfs_ops,
};

int bst_cv_sysfile_init(struct bst_cv *pbst_cv)
{
    int ret;

    ret = kobject_init_and_add(&pbst_cv->kobj, &dynamic_kobj_ktype, kernel_kobj, BST_CV_DRIVER_NAME);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to create bst_cv kobject");
        return ret;
    }

    ret = sysfs_create_group(&pbst_cv->kobj, &bst_cv_kobj_attr_group);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to create bst_cv kobject attribute group");
        kobject_put(&pbst_cv->kobj);
    }
    return ret;
}

void bst_cv_sysfile_exit(struct bst_cv *pbst_cv)
{
    kobject_put(&pbst_cv->kobj);
    return;
}
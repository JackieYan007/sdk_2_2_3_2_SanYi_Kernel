/*!
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file   bst_cv_sysfile.c
 * @brief  This file implements the sysfs for BareCV
 */

#include "bst_cv.h"

int bare_cv_print_level = BST_CV_DEBUG_PRINT;
char cores_status[] = "0000";
char stall_cores[] = "0000";

extern void _release_rt_fw(struct bst_cv *pbst_cv, int dsp);


static ssize_t bst_cv_print_level_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    switch (bare_cv_print_level) {
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
        bare_cv_print_level = new_print_level;
    }
    return count;
}

// enable cores
static ssize_t cores_status_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    struct bst_cv *pbst_cv;
    pbst_cv = container_of((void *)object, struct bst_cv, kobj);
    int core;
    for(core=0;core<BST_CV_DSP_NUM;core++)
    {
        cores_status[core] = pbst_cv->dsp_online[core] ? '1' : '0';
    }
    return sprintf(buf, "%s\n", cores_status);
}

int bst_cv_boot_firmware_one_core(struct bst_cv *pbst_cv, int core_id);
static ssize_t cores_status_attr_store(struct kobject *object,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct bst_cv *pbst_cv;
    pbst_cv = container_of((void *)object, struct bst_cv, kobj);
    if(strlen(buf) < BST_CV_DSP_NUM)
    {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "Format must be like 0110");  
        return count;
    }

    int core;
    for(core=0; core<BST_CV_DSP_NUM; core++)
    {
        if(buf[core] == '1' )
        {
            if(pbst_cv->dsp_online[core] == 1)
            {
                // stall
                _release_rt_fw(pbst_cv, core);
            }

            // ioremap
            if (IS_ERR_OR_NULL(pbst_cv->fw_manager.dsps[core].fwmem_base)) 
            {
                pbst_cv->fw_manager.dsps[core].fwmem_base = ioremap(
                            pbst_cv->fw_manager.dsps[core].fwmem_phys_addr, 
                            pbst_cv->fw_manager.dsps[core].fwmem_size);
                BST_CV_STAGE_PRINTK("ioremap pbst_cv->fw_manager.dsps[%d].fwmem_phys_addr = 0x%x",
                    core, pbst_cv->fw_manager.dsps[core].fwmem_phys_addr);
            }
            
            // load new fw
            int ret = bst_cv_boot_firmware_one_core(pbst_cv, core);
            if(ret)
            {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_boot_firmware_one_core failed");
            }
        }
    }
    return count;
}


// stall cores
static ssize_t stall_cores_attr_show(struct kobject *object,
    struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", stall_cores);
}

static ssize_t stall_cores_attr_store(struct kobject *object,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct bst_cv *pbst_cv;
    pbst_cv = container_of((void *)object, struct bst_cv, kobj);
    if(strlen(buf) < BST_CV_DSP_NUM)
    {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fomat must be like 0110");  
        return count;
    }

    int core;
    for(core=0; core<BST_CV_DSP_NUM; core++)
    {
        if(buf[core] == '1' && pbst_cv->dsp_online[core] == 1)
        {
            // iounmap
            if (!IS_ERR_OR_NULL(pbst_cv->fw_manager.dsps[core].fwmem_base)) 
            {
                iounmap( pbst_cv->fw_manager.dsps[core].fwmem_base);
                pbst_cv->fw_manager.dsps[core].fwmem_base = 0;
            }

            // stall
            _release_rt_fw(pbst_cv, core);
        }
    }

    strcpy(stall_cores, buf);
    return count;
}

static struct kobj_attribute bst_cv_print_level_attr = __ATTR(bare_cv_print_level,
    0664, bst_cv_print_level_attr_show, bst_cv_print_level_attr_store);
    
static struct kobj_attribute cores_status_attr = __ATTR(cores_status,
    0664, cores_status_attr_show, cores_status_attr_store);

static struct kobj_attribute stall_cores_attr = __ATTR(stall_cores,
    0664, stall_cores_attr_show, stall_cores_attr_store);

static struct attribute *bst_cv_kobj_attrs[] = {
	&bst_cv_print_level_attr.attr,
	&cores_status_attr.attr,
	&stall_cores_attr.attr,
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

int bare_cv_sysfile_init(struct bst_cv *pbst_cv)
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

void bare_cv_sysfile_exit(struct bst_cv *pbst_cv)
{
    kobject_put(&pbst_cv->kobj);
    return;
}
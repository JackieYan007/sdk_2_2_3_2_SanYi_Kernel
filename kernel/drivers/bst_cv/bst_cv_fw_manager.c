/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_cv_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the bst_cv driver. It contains function definitions of
 *          initializaiton, cleanup and exit of the firmware manager and the
 *          runtime firmware setup.
 */

#include "bst_cv.h"

#define SYS_CTRL_R_CHIP_VERSION (0x3300007c)
#define SYS_CTRL_R_CORENOC_PARITY_ENABLE (0x33000084)
#define CV_PARITY_EN			 (0x2)

// the hard coded constants for firmware initialization
static const uint32_t ipc_dsp_irq_clear_addrs[BST_CV_DSP_NUM] = {0x33102a28, 0x33102b2c, 0x33102c30, 0x33102d34};
static const uint32_t ipc_dsp_irq_trigger_addr = 0x33102a0c;
static const uint32_t ipc_dsp_ireq_enable_addrs[BST_CV_DSP_NUM] = {0x33102aa8, 0x33102bac, 0x33102cb0, 0x33102db4};
static const uint32_t ipc_host_irq_clear_addr = 0x3310230c;

static const uint32_t ipc_device_irq_indices[BST_CV_DSP_NUM] = {4, 4, 4, 4};

static const uint32_t ipc_host_to_dsp_addrs[BST_CV_DSP_NUM] = {0x8ff000c0, 0x8ff00100, 0x8ff00140, 0x8ff00180};
static const uint32_t ipc_dsp_to_host_addrs[BST_CV_DSP_NUM] = {0x8ff00280, 0x8ff002c0, 0x8ff00300, 0x8ff00340};

static uint32_t bst_cv_chip_verson;

/*******************************************************************************
 * bst_cv FIRMWARE
 ******************************************************************************/
#ifdef BST_CV_DEBUG
/*
 * @func    _dump_firmware
 * @brief   This is a debug function to dump the firmware buffer.
 * @params  fw_buf - the pointer to the firmware buffer
 *          size - the dumped size
 * @return  void
 */
static void _dump_firmware(char *fw_buf, int size)
{
    int i;
    for(i = 0; i < size/BST_CV_FIRMWARE_DUMP_LINE_SIZE; i++) {
        BST_CV_TRACE_PRINTK("%02x %02x %02x %02x %02x %02x %02x %02x",
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 1),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 2),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 3),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 4),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 5),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 6),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 7));
    }
    return;
}
#endif

/*
 * @func    _load_firmware
 * @brief   This function loads the firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int _load_firmware(struct bst_cv *pbst_cv, int i, struct firmware *fw)
{
    if (pbst_cv->fw_manager.dsps[i].fwmem_size < fw->size) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "firmware too large for DSP %d", i);
        return -EINVAL;
    }

    BST_CV_TRACE_PRINTK("firmware mem base: 0x%px, size: %lld", pbst_cv->fw_manager.dsps[i].fwmem_base,
        pbst_cv->fw_manager.dsps[i].fwmem_size);
    BST_CV_TRACE_PRINTK("firmware data: 0x%px, size: %ld", fw->data, fw->size);

    memcpy_toio(pbst_cv->fw_manager.dsps[i].fwmem_base, fw->data, fw->size);

#ifdef BST_CV_DEBUG
    _dump_firmware(pbst_cv->fw_manager.dsps[i].fwmem_base, BST_CV_FIRMWARE_DUMP_SIZE);
#endif

    return 0;
}

/*
 * @func    _boot_firmware
 * @brief   This fucntion resets the firmware to boot it up.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
static void _boot_firmware(struct bst_cv *pbst_cv, int i)
{
    uint32_t reg;
    void __iomem *chip_version = NULL;
    void __iomem *crm_base = NULL;
    void __iomem *corenoc_parity_enable = NULL;
    uint32_t cv_parity_en;

    chip_version = devm_ioremap(&pbst_cv->pdev->dev, SYS_CTRL_R_CHIP_VERSION, 0x4);
    if (chip_version == NULL) {
        dev_err(&pbst_cv->pdev->dev,
            "IOREMAP SYS_CTRL_R_CHIP_VERSION 0x%8X FAILED\n", SYS_CTRL_R_CHIP_VERSION);
        return;
    }
    bst_cv_chip_verson = readl_relaxed(chip_version);
    devm_iounmap(&pbst_cv->pdev->dev, chip_version);

    crm_base = ioremap(0x33002000, 0x200); // SYS_CRM base address
    // set up CRM register for CV block, or else OS hangs after vector reset
    reg = readl_relaxed(crm_base + 0x4); // TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
    reg |= (1UL << 7); // cv block soft reset
    writel_relaxed(reg, crm_base + 0x4);
    
    // set CV freq to 800MHz
    reg = readl_relaxed(crm_base + 0x15c); // CLKMUX_SEL2
    // CLK_800_CV_CORE_CLK_SEL, bit[3:2] = 01: 800MHz
    reg &= ~(1UL << 3);
    reg |= (1UL << 2);
    writel_relaxed(reg, crm_base + 0x15c);

    corenoc_parity_enable =
        devm_ioremap(&pbst_cv->pdev->dev, SYS_CTRL_R_CORENOC_PARITY_ENABLE, 0x4);
    if (corenoc_parity_enable == NULL) {
        dev_err(&pbst_cv->pdev->dev,
            "IOREMAP SYS_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
            SYS_CTRL_R_CORENOC_PARITY_ENABLE);
        return;
    }
    cv_parity_en = readl_relaxed(corenoc_parity_enable) & CV_PARITY_EN;
    devm_iounmap(&pbst_cv->pdev->dev, corenoc_parity_enable);

    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    if (cv_parity_en) {
        reg |= 0x3; // cv block ecc and parity enable
        writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    }

    //setup reset vector
    BST_CV_STAGE_PRINTK("set vector: 0x%x, addr: 0x%px",
        addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);
    writel_relaxed(addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);

    //start dsp
    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    reg |= (1 << (BST_CV_SOFT_RESET_BIT + i));
    reg |= (1 << (BST_CV_CLK_EN_BIT + i));
    BST_CV_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    return;
}

/*
 * @func    bst_cv_boot_firmware
 * @brief   This function loads the runtime firmware and boots it up.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_boot_firmware(struct bst_cv *pbst_cv)
{
    int ret;
    int i;
    struct firmware *fw;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            ret = request_firmware((const struct firmware **)&fw,
                pbst_cv->fw_manager.dsps[i].name, &pbst_cv->pdev->dev);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to request firmware for DSP %d: %d", i, ret);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            BST_CV_STAGE_PRINTK("bst_cv firmware requested for DSP %d", i);

            ret = _load_firmware(pbst_cv, i, fw);
            release_firmware(fw);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to load firmware for DSP %d: %d", i, ret);
                pbst_cv->dsp_online[i] = 0;
                
                continue;
            }
            BST_CV_STAGE_PRINTK("bst_cv firmware loaded for DSP %d", i);

            _boot_firmware(pbst_cv, i);
            pbst_cv->fw_manager.dsps[i].boot = 1;
            BST_CV_STAGE_PRINTK("bst_cv firmware booted for DSP %d", i);
        }
    }

    return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_init
 * @brief   This function initializes the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_manager_init(struct bst_cv *pbst_cv)
{
    int i;
    int ret;
    struct resource *res;
    uint32_t assigned_mem_size;
    struct device_node *bst_cv_node, *dsp_node;

    bst_cv_node = pbst_cv->pdev->dev.of_node;
    //map registers
    res = platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, 0);
    if (!res) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to retrieve cv register");
        ret = -ENODEV;
        return ret;
    }
    pbst_cv->fw_manager.lb_cv_reg_base = devm_ioremap(&pbst_cv->pdev->dev,
        res->start, resource_size(res));
    if (IS_ERR(pbst_cv->fw_manager.lb_cv_reg_base)) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to remap cv register");
        return PTR_ERR(pbst_cv->fw_manager.lb_cv_reg_base);
    }

    //get the ipc register address
    ret = device_property_read_u32(&pbst_cv->pdev->dev, "ipc-register-addr",
        &pbst_cv->fw_manager.ipc_register_addr);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no ipc-register-addr property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid ipc-register-addr property");
        return ret;
    }

    //get assigned memory sizes
    ret = device_property_read_u32(&pbst_cv->pdev->dev, "assigned-mem-size",
        &assigned_mem_size);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no assigned-mem-size property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid assigned-mem-size");
        return ret;
    }

    //assign memory for firmware handshake
    pbst_cv->fw_manager.assigned_mem = pbst_cv->mem_manager.ops->alloc(pbst_cv,
        assigned_mem_size, 0);
    if (pbst_cv->fw_manager.assigned_mem == NULL) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "could not allocate assigned-mem");
        ret = -ENOMEM;
        return ret;
    }

    //get the dsp number
    ret = device_property_read_u32(&pbst_cv->pdev->dev, "dsp-num",
        &pbst_cv->dsp_num);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no dsp-num property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid dsp-num property");
        return ret;
    }

    if (pbst_cv->dsp_num != of_get_child_count(bst_cv_node)) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "inconsistent DSP number in device tree");
        return -EFAULT;
    }

    i = 0;
    for_each_child_of_node(bst_cv_node, dsp_node) {
        //get the CV DSP index
        ret = of_property_read_u32(dsp_node, "index", &pbst_cv->dsp_indices[i]);
        if (ret == -EINVAL || ret == -ENODATA) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no index property");
           goto cur_iter_failure;
        } else if (ret < 0) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid index property");
            goto cur_iter_failure;
        }
        BST_CV_TRACE_PRINTK("CV DSP index of DSP %d: %d", i, pbst_cv->dsp_indices[i]);

        //get the DSP initialization address
        ret = of_property_read_u32(dsp_node, "rt-init-addr",
            &pbst_cv->fw_manager.dsps[i].rt_init_addr);
        if (ret == -EINVAL || ret == -ENODATA) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no rt-init-addr property");
           goto cur_iter_failure;
        } else if (ret < 0) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid rt-init-addr property");
            goto cur_iter_failure;
        }
        BST_CV_TRACE_PRINTK("rt-init-addr of DSP %d: 0x%x", i, pbst_cv->fw_manager.dsps[i].rt_init_addr);

        //get the firmware name
        ret = of_property_read_string(dsp_node, "firmware",
            (const char **)&pbst_cv->fw_manager.dsps[i].name);
        if (ret == -EINVAL || ret == -ENODATA){
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no firmware property");
            goto cur_iter_failure;
        }
        else if (ret < 0) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid firmware property");
            goto cur_iter_failure;
        }
        BST_CV_TRACE_PRINTK("firmware of DSP %d: %s", i, pbst_cv->fw_manager.dsps[i].name);

        //get the firmware name
        ret = of_property_read_u32(dsp_node, "ipc-src-core",
            &pbst_cv->fw_manager.dsps[i].ipc_src_core);
        if (ret == -EINVAL || ret == -ENODATA){
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no ipc-src-core property");
            goto cur_iter_failure;
        }
        else if (ret < 0) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid ipc-src-core property");
            goto cur_iter_failure;
        }

        //map firmware memory
        res = platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, i + 1);
        if (!res) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "could not get firmware resource for DSP %d", i);
            goto cur_iter_failure;
        }
        BST_CV_TRACE_PRINTK("firmware mem of DSP %d start: 0x%llx, end: 0x%llx",
            i, res->start, res->end);

        pbst_cv->fw_manager.dsps[i].fwmem_base = devm_ioremap(&pbst_cv->pdev->dev,
            res->start, resource_size(res));
        if (IS_ERR(pbst_cv->fw_manager.dsps[i].fwmem_base)) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to remap firmware memory: %ld",
                PTR_ERR(pbst_cv->fw_manager.dsps[i].fwmem_base));
            goto cur_iter_failure;
        }
        pbst_cv->fw_manager.dsps[i].fwmem_size = resource_size(res);
        pbst_cv->fw_manager.dsps[i].fwmem_phys_addr = res->start;
        BST_CV_TRACE_PRINTK("firmware mem base for DSP %d: 0x%px, size: %llx", i,
            pbst_cv->fw_manager.dsps[i].fwmem_base,
            pbst_cv->fw_manager.dsps[i].fwmem_size);

        //check rt_init_addr
        if (pbst_cv->fw_manager.dsps[i].rt_init_addr < phys_to_bus(pbst_cv, res->start) ||
            pbst_cv->fw_manager.dsps[i].rt_init_addr > phys_to_bus(pbst_cv, res->end)) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "rt_init_addr of DSP %d is outside of firmware memory", i);
            goto cur_iter_failure;
        }

        //assign memory for firmware
        pbst_cv->fw_manager.dsps[i].sync_virt_base =
            pbst_cv->fw_manager.assigned_mem->kern_addr + BST_CV_HANDSHAKE_BUF_SIZE * i;
        pbst_cv->fw_manager.dsps[i].sync_phys_base =
            pbst_cv->fw_manager.assigned_mem->phys_addr + BST_CV_HANDSHAKE_BUF_SIZE * i;
        BST_CV_TRACE_PRINTK("DSP %d handshake buffer: 0x%px, 0x%llx, size: 0x%x",
            i,
            pbst_cv->fw_manager.dsps[i].sync_virt_base,
            pbst_cv->fw_manager.dsps[i].sync_phys_base,
            BST_CV_HANDSHAKE_BUF_SIZE
        );

        pbst_cv->dsp_online[i] = 1;
        pbst_cv->fw_manager.dsps[i].init = 1;
        goto loop_continue;

cur_iter_failure:
        pbst_cv->dsp_online[i] = 0;
        pbst_cv->fw_manager.dsps[i].init = 0;
loop_continue:
        i++;
    }
    return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    _dump_xrp_dsp_sync
 * @brief   This function dumps the xrp_dsp_sync structure used in the handshake
 *          with the firmware
 * @params  pbst_cv - the pointer to the bst_cv device
 *          xrp_dsp_sync_base - the base address of the xrp_dsp_sync structure
 * @return  void
 */
static inline void _dump_xrp_dsp_sync(struct bst_cv *pbst_cv, struct xrp_dsp_sync *xrp_dsp_sync_base){
    int j;
    
    for(j=0; j<(sizeof(*xrp_dsp_sync_base)+15)/16; j++) {
        BST_CV_TRACE_PRINTK("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x",
        kern_to_bus(pbst_cv, xrp_dsp_sync_base) + j * 16,
        *((uint32_t *)xrp_dsp_sync_base + j * 4),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 1),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 2),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 3));
    }
    return;
}

/*
 * @func    bst_cv_fw_rt_setup
 * @brief   This function initializes the runtime firmware. Because it relies on
 *          messaging and the message manager can only be initialized after
 *          getting some required firmware information in bst_cv_fw_manager_init,
 *          this part is separated from the firmware manager initialization.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_rt_setup(struct bst_cv *pbst_cv)
{
    int i, count;
    int ret = 0;
    struct xrp_dsp_sync *xrp_dsp_sync_base;
    uint32_t data;

    //boot firmware
    ret = bst_cv_boot_firmware(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_boot_firmware failed for all DSPs");
        return ret;
    }
    BST_CV_STAGE_PRINTK("bst_cv_boot_firmware OK");

    //init bst_cv message manager
    ret = bst_cv_msg_manager_init(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_msg_manager_init all failed");
        //bst_cv_msg_manager_cleanup(pbst_cv);
    }
    BST_CV_STAGE_PRINTK("bst_cv_msg_manager_init OK");

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            //sync setup(really stupid and unnecessary step from firmware side)
            xrp_dsp_sync_base = pbst_cv->fw_manager.dsps[i].sync_virt_base;
            xrp_dsp_sync_base->device_mmio_base = 0;
            xrp_dsp_sync_base->host_irq_mode = XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
            xrp_dsp_sync_base->host_irq_offset = 0;
            xrp_dsp_sync_base->host_irq_bit = IPC_CORE_DSP_0 + i; 
            xrp_dsp_sync_base->device_irq_mode = XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
            xrp_dsp_sync_base->device_irq_offset = 0;
            xrp_dsp_sync_base->device_irq_bit = IPC_CORE_ARM3;
            xrp_dsp_sync_base->device_irq = ipc_device_irq_indices[i];
            xrp_dsp_sync_base->host_ipc_irq_clear = ipc_host_irq_clear_addr;
            xrp_dsp_sync_base->host_irq = 0x91;
            xrp_dsp_sync_base->device_ipc_irq_clear = ipc_dsp_irq_clear_addrs[i];
            xrp_dsp_sync_base->host_ipc_irq_trig = ipc_dsp_irq_trigger_addr + i * 0x100;
            xrp_dsp_sync_base->device_ipc_irq_trig = 0x33102328 + i * 4;
            xrp_dsp_sync_base->debug_buffer_base = bst_cv_firmware_log_buffer + i * (bst_cv_firmware_log_length / BST_CV_DSP_NUM);
            xrp_dsp_sync_base->debug_buffer_length = (bst_cv_firmware_log_length / BST_CV_DSP_NUM);
            xrp_dsp_sync_base->debug_level = bst_cv_firmware_log_level;
            xrp_dsp_sync_base->ipc_host_to_dsp_addr = ipc_host_to_dsp_addrs[i];
            xrp_dsp_sync_base->ipc_dsp_to_host_addr = ipc_dsp_to_host_addrs[i];
            xrp_dsp_sync_base->device_ipc_irq_enable = ipc_dsp_ireq_enable_addrs[i];
            xrp_dsp_sync_base->device_ipc_irq_enable_mask = (1 << 30) | (1 << IPC_CORE_ARM3); 

            if (0 == bst_cv_chip_verson) {
                xrp_dsp_sync_base->device_ipc_irq_enable &= ~0x11000000UL;
                xrp_dsp_sync_base->device_ipc_irq_clear &= ~0x11000000UL;
                xrp_dsp_sync_base->host_ipc_irq_trig &= ~0x11000000UL;
            }

            BST_CV_TRACE_PRINTK("DSP %d log buffer: base %#x, length %#x, level %d",
                i,
                xrp_dsp_sync_base->debug_buffer_base,
                xrp_dsp_sync_base->debug_buffer_length,
                xrp_dsp_sync_base->debug_level);

#ifdef BST_CV_DEBUG
            _dump_xrp_dsp_sync(pbst_cv, xrp_dsp_sync_base);
#endif
            //start handshaking
            xrp_dsp_sync_base->sync = XRP_DSP_SYNC_START;
            count = 0;
            while (xrp_dsp_sync_base->sync != XRP_DSP_SYNC_DSP_READY
                && count < BST_CV_HANDSHAKE_RETRY_NUM) {
                msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
                count++;
            }
            if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to wait for handshake ready from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            xrp_dsp_sync_base->sync = XRP_DSP_SYNC_HOST_TO_DSP;
            count = 0;
            while (xrp_dsp_sync_base->sync != XRP_DSP_SYNC_DSP_TO_HOST
                 && count < BST_CV_HANDSHAKE_RETRY_NUM) {
                msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
                count++;
            }
            if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to wait for handshake sync from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            if (bst_cv_msg_send(pbst_cv, i, 0) < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to send interrupt handshake to DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            if (bst_cv_msg_recv(pbst_cv, i, &data, BST_CV_HANDSHAKE_TIMEOUT) < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "ipc timeout, failed to receive interrupt handshake from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
        }
    }

    bst_cv_fw_rt_cleanup(pbst_cv);
    return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_cleanup
 * @brief   This function cleans up the resources allocated in firmware
 *          manager initialization for subsequential failure of DSPs or even
 *          the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_cleanup(struct bst_cv *pbst_cv) {
    int i;
    int online_bit = 0;
    for (i=0; i<BST_CV_DSP_NUM; i++) {
        online_bit |= pbst_cv->dsp_online[i] << i;
    }
    if (online_bit == 0) {
        pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.assigned_mem);
    }
    return;
}

/*
 * @func    _release_rt_firmware
 * @brief   This function resets the specified DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          dsp - the CV DSP number
 * @return  void
 */
static inline void _release_rt_fw(struct bst_cv *pbst_cv, int dsp)
{
    uint32_t reg;

    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    //stall the DSP first
    //reg |= (1 << (BST_CV_RUNSTALL_BIT + dsp));
    //writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    //then reset the DSP
    reg &= ~(1 << (BST_CV_SOFT_RESET_BIT + dsp));
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    return;
}

/*
 * @func    bst_cv_fw_rt_cleanup
 * @brief   This function resets the DSPs for subsequential failure of DSPs or
 *          even the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_cleanup(struct bst_cv *pbst_cv) {
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (!pbst_cv->dsp_online[i] && pbst_cv->fw_manager.dsps[i].boot) {
            _release_rt_fw(pbst_cv, i);
        }
    }
    return;
}

/*
 * @func    bst_cv_fw_manager_exit
 * @brief   This is the exit function of the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_exit(struct bst_cv *pbst_cv)
{
    int i;
    int online_bit = 0;
    for (i=0; i<BST_CV_DSP_NUM; i++) {
        online_bit |= pbst_cv->dsp_online[i] << i;
    }
    if (online_bit == 0) {
        pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.assigned_mem);
    }
    return;
}

/*
 * @func    bst_cv_fw_rt_exit
 * @brief   This is the exit function of the runtime firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_exit(struct bst_cv *pbst_cv)
{
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            _release_rt_fw(pbst_cv, i);
        }
    }
    return;
}

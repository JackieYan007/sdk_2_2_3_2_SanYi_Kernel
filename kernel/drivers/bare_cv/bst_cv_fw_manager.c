/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file    bst_cv_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the bst_cv driver. It contains function definitions of
 *          initializaiton, cleanup and exit of the firmware manager and the
 *          runtime firmware setup.
 */

#include "bst_cv.h"
#include "fw_file.h"

// the hard coded constants for firmware initialization
static const uint32_t ipc_dsp_irq_clear_addrs[BST_CV_DSP_NUM] = {0x22102a28};
static const uint32_t ipc_dsp_irq_trigger_addr = 0x22102a0c;
static const uint32_t ipc_dsp_ireq_enable_addrs[BST_CV_DSP_NUM] = {0x22102aa8};
static const uint32_t ipc_host_irq_clear_addr = 0x3310230c;

static const uint32_t ipc_device_irq_indices[BST_CV_DSP_NUM] = {4};

static const uint32_t ipc_host_to_dsp_addrs[BST_CV_DSP_NUM] = {0x8ff000c0};
static const uint32_t ipc_dsp_to_host_addrs[BST_CV_DSP_NUM] = {0x8ff00280};
static const uint32_t CV_CRM_RESET = 0x33002180;

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
void _release_rt_fw(struct bst_cv *pbst_cv, int dsp);

/*
 * @func    _load_firmware
 * @brief   This function loads the firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int _load_firmware(struct bst_cv *pbst_cv, int i, struct firmware *fw)
{
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    if (pbst_cv->fw_manager.dsps[i].fwmem_size < fw->size) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "firmware too large for DSP %d", i);
        return -EINVAL;
    }

    BST_CV_TRACE_PRINTK("firmware mem base: 0x%px, size: %lld", pbst_cv->fw_manager.dsps[i].fwmem_base,
        pbst_cv->fw_manager.dsps[i].fwmem_size);
    BST_CV_TRACE_PRINTK("firmware data: 0x%px, size: %ld", fw->data, fw->size);

    // _release_rt_fw(pbst_cv, i);
    memcpy_toio(pbst_cv->fw_manager.dsps[i].fwmem_base, fw->data, fw->size);

#ifdef BST_CV_DEBUG
    _dump_firmware(pbst_cv->fw_manager.dsps[i].fwmem_base, BST_CV_FIRMWARE_DUMP_SIZE);
#endif
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

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
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    uint32_t reg;
    void __iomem *crm_base = NULL;
    void __iomem *ddr0_ctrl = NULL;
    void __iomem *ddr1_ctrl = NULL;
    uint32_t ecc_en;

    crm_base = ioremap(0x33002000, 0x200); // SYS_CRM base address
    // set up CRM register for CV block, or else OS hangs after vector reset
    // reg = readl_relaxed(crm_base + 0x4); // TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
    // reg |= (1UL << 7); // cv block soft reset
    // writel_relaxed(reg, crm_base + 0x4);	

    // set CV freq to 800MHz
    reg = readl_relaxed(crm_base + 0x15c); // CLKMUX_SEL2
    // CLK_800_CV_CORE_CLK_SEL, bit[3:2] = 01: 800MHz
    reg &= ~(1UL << 3);
    reg |= (1UL << 2);
    writel_relaxed(reg, crm_base + 0x15c);

    //lb_cv_top_sw_clk reset
    // crm_base = ioremap(CV_CRM_RESET, 0x4);
    reg = readl_relaxed(crm_base + 0x180);
    reg |= (1UL << 12);
    writel_relaxed(reg, crm_base + 0x180);


    //parity
    ddr0_ctrl = ioremap(DDRC0_CTRL, 0x400);
    ddr1_ctrl = ioremap(DDRC1_CTRL, 0x400);
    ecc_en = (readl_relaxed(ddr0_ctrl + OCPARCFG0) | readl_relaxed(ddr1_ctrl + OCPARCFG0)) & OC_PARITY_EN;

    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    if (ecc_en && ((reg & 0x3) != 0x3)) {
        reg |= 0x3; // cv block ecc and parity enable
        writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    }

    writel_relaxed(0x100, pbst_cv->fw_manager.lb_cv_reg_base + 0x40);


    //setup reset vector
    BST_CV_STAGE_PRINTK("set vector: 0x%x, addr: 0x%px",
        addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);
    writel_relaxed(addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);
    
    
    //start dsp
    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    BST_CV_TRACE_PRINTK("reg = %x\n",reg);
    reg |= ((1 << (BST_CV_CLK_EN_BIT + i))  |  (1 << (BST_CV_SOFT_RESET_BIT + i)));
    BST_CV_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    BST_CV_TRACE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    pbst_cv->dsp_online[i] = 1;
    pbst_cv->state = BST_CV_ONLINE;

    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
}

int bst_cv_boot_firmware_one_core(struct bst_cv *pbst_cv, int core_id)
{
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int ret;
    int i = core_id;
    struct firmware *fw;

    ret = request_firmware((const struct firmware **)&fw,
        pbst_cv->fw_manager.dsps[i].name, &pbst_cv->pdev->dev);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to request firmware %s for DSP %d: %d", 
            pbst_cv->fw_manager.dsps[i].name ,i, ret);
        pbst_cv->dsp_online[i] = 0;
        return -1;
    }

    // BST_CV_STAGE_PRINTK("bst_cv firmware requested for DSP %d", i);
    // BST_CV_TRACE_PRINTK("bst_cv firmware requested for DSP %d", i);
    // msleep(1000);

    ret = _load_firmware(pbst_cv, i, fw);
    release_firmware(fw);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to load firmware for DSP %d: %d", i, ret);
        pbst_cv->dsp_online[i] = 0;
        
        return -1;
    }

    _boot_firmware(pbst_cv, i);
    pbst_cv->fw_manager.dsps[i].boot = 1;
    BST_CV_STAGE_PRINTK("bst_cv firmware booted for DSP %d", i);
    BST_CV_TRACE_PRINTK("bst_cv firmware booted for DSP %d", i);
    // BST_CV_TRACE_PRINTK("bst_cv firmware booted for DSP %d", i);

    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

    return 0;
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
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int ret;
    int i;
    struct firmware *fw;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            ret = request_firmware((const struct firmware **)&fw,
                pbst_cv->fw_manager.dsps[i].name, &pbst_cv->pdev->dev);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to request firmware %s for DSP %d: %d", 
                    pbst_cv->fw_manager.dsps[i].name ,i, ret);
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
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

    return bare_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bare_cv_fw_manager_init
 * @brief   This function initializes the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_fw_manager_init(struct bst_cv *pbst_cv)
{   
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int i;
    int ret;
    struct resource *res;
    char *fw_names[BST_CV_DSP_NUM];
    u64 reg[2];

    void *crm_reset = (uint32_t *)devm_ioremap(&pbst_cv->pdev->dev, CV_CRM_RESET, 4);
    
	if (crm_reset == NULL) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "IOREMAP reg_ctrl10x%lx FAILED\n", crm_reset);
		return -1;
	} else {
        uint32_t crm_reset_reg_val = readl_relaxed(crm_reset);
        BST_CV_TRACE_PRINTK("crm_reset_reg_val = readl_relaxed(crm_reset)  :  %x\n",crm_reset_reg_val);
                
        if((crm_reset_reg_val & BIT(12)) == 0)
        {
            crm_reset_reg_val |= BIT(12);
        }
        writel_relaxed(crm_reset_reg_val, crm_reset);
	}

    //map registers
    ret = device_property_read_u64_array(&pbst_cv->pdev->dev, "reg", reg, 2);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no reg property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid reg property");
        return ret;
    }
    pbst_cv->fw_manager.lb_cv_reg_base = devm_ioremap(&pbst_cv->pdev->dev,
        reg[0], reg[1]);
    if (IS_ERR(pbst_cv->fw_manager.lb_cv_reg_base)) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to map cv regs");
        return PTR_ERR(pbst_cv->fw_manager.lb_cv_reg_base);
    }

    //parity
    uint32_t cv_parity_ctrl = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + CV_INTERNAL_PTY);




    //get firmware names
    ret = device_property_read_string_array(&pbst_cv->pdev->dev, "firmware", 
        (const char **)fw_names, BST_CV_DSP_NUM);
    if (ret == -EINVAL || ret == -ENODATA){
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no firmware property found");
        return ret;
    }
    else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid firmware property");
        return ret;
    }

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        //map firmware memory
        res = platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, i + 1);
        if (!res) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "could not get firmware resource for DSP %d", i);
            pbst_cv->dsp_online[i] = 0;
            pbst_cv->fw_manager.dsps[i].init = 0;
            continue;
        }
        BST_CV_TRACE_PRINTK("bst_cv firmware mem of DSP %d start: 0x%llx, end: 0x%llx",
            i, res->start, res->end);

        pbst_cv->fw_manager.dsps[i].fwmem_size = res->end - res->start + 1;
        pbst_cv->fw_manager.dsps[i].fwmem_phys_addr = res->start;
        pbst_cv->fw_manager.dsps[i].name = fw_names[i];
        pbst_cv->fw_manager.dsps[i].fwmem_base = 0;
        
        if(0)
        {
            char fw_path[32] = {0};

            sprintf(fw_path, "/lib/firmware/bstcv%d.rbf", i);
            struct file* filp = file_open(fw_path, O_RDWR, 0755);
            if(filp)
            {
                file_close(filp) ;

                BST_CV_TRACE_PRINTK("%s exists", fw_path);
                pbst_cv->fw_manager.dsps[i].fwmem_base = devm_ioremap_resource(&pbst_cv->pdev->dev,
                    res);
                if (IS_ERR(pbst_cv->fw_manager.dsps[i].fwmem_base)) {
                    BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to remap firmware mem: %d",
                        ret);
                    pbst_cv->dsp_online[i] = 0;
                    pbst_cv->fw_manager.dsps[i].init = 0;
                    continue;
                }
            }   
        }
        BST_CV_STAGE_PRINTK("firmware of dsp %d: %s", i, pbst_cv->fw_manager.dsps[i].name);

        pbst_cv->dsp_online[i] = 0;
        pbst_cv->fw_manager.dsps[i].init = 1;
    }
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

    return 0;
}

/*
 * @func    bare_cv_fw_rt_setup
 * @brief   This function initializes the runtime firmware. Because it relies on
 *          messaging and the message manager can only be initialized after
 *          getting some required firmware information in bare_cv_fw_manager_init,
 *          this part is separated from the firmware manager initialization.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_fw_rt_setup(struct bst_cv *pbst_cv)
{   
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int ret = 0;

    //boot firmware
    ret = bst_cv_boot_firmware(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_boot_firmware failed for all DSPs");
        return ret;
    }
    BST_CV_STAGE_PRINTK("bst_cv_boot_firmware OK");
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

    return 0;
}


/*
 * @func    _release_rt_firmware
 * @brief   This function resets the specified DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          dsp - the CV DSP number
 * @return  void
 */
void _release_rt_fw(struct bst_cv *pbst_cv, int dsp)
{
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);
    
    uint32_t reg;

    //stall the DSP first
    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    reg |= BIT(BST_CV_RUNSTALL_BIT + dsp);
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);

    //then reset the DSP
    uint32_t reset_val = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    reset_val = LB_CV_REG_R_CV_SYS_CTRL_DEFAULT;

    BST_CV_STAGE_PRINTK("----------------------- stall value reset_val = 0x%x\n", reset_val);
    writel_relaxed(reset_val, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);

    pbst_cv->dsp_online[dsp] = 0;
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
}


/*
 * @func    bare_cv_fw_manager_exit
 * @brief   This is the exit function of the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bare_cv_fw_manager_exit(struct bst_cv *pbst_cv)
{
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            // pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.dsps[i].assigned_mem);
        }
    }
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);

    return;
}

/*
 * @func    bare_cv_fw_manager_exit
 * @brief   This is the exit function of the runtime firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bare_cv_fw_rt_exit(struct bst_cv *pbst_cv)
{
    // BST_CV_STAGE_PRINTK("in                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    // msleep(200);

    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            _release_rt_fw(pbst_cv, i);
        }
    }
    // BST_CV_STAGE_PRINTK("out                         FUNCTION:%s, LINE:%d\n", __func__, __LINE__);
    
    return;
}
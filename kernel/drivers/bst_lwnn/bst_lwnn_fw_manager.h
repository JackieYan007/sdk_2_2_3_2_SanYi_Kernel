/*!
 * bst_lwnn: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author  Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_lwnn_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          bst_lwnn driver. It contains related constants and structure
 *          definitions and function declarations.
 */

#ifndef BST_LWNN_FW_MANAGER_H
#define BST_LWNN_FW_MANAGER_H

#define BST_LWNN_HANDSHAKE_TIMEOUT                  100
#define BST_LWNN_HANDSHAKE_SLEEP_INTERVAL           10
#define BST_LWNN_HANDSHAKE_RETRY_NUM                (BST_LWNN_HANDSHAKE_TIMEOUT / BST_LWNN_HANDSHAKE_SLEEP_INTERVAL)

#define BST_LWNN_REG_WIDTH                          4

#define BST_LWNN_FIRMWARE_DUMP_SIZE                 64
#define BST_LWNN_FIRMWARE_DUMP_LINE_SIZE            8

#define LB_CV_REG_R_CV_SYS_CTRL_OFFSET              0x0
#define BST_LWNN_SOFT_RESET_BIT                     28
#define BST_LWNN_DSP0_CLK_EN                        24
#define BST_LWNN_RUNSTALL_BIT                       20
#define LB_CV_REG_R_CV_SYS_CTRL_DEFAULT             0x0f000000

#define LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET     0x4

#define DDRC0_CTRL                      0x38001000
#define DDRC1_CTRL                      0x3C001000
#define OCPARCFG0                       0x330
#define OC_PARITY_EN                    0x1

struct bst_lwnn_dsp_fw_ctl {
    //flags used for cleanup
    uint8_t init;
    uint8_t boot;
    //firmware fileanme
    char *name;
    //memory reserved for firmware codes
    void __iomem *fwmem_base;
    resource_size_t fwmem_size;
    phys_addr_t fwmem_phys_addr;
    dsp_ptr rt_init_addr;
    //memory assigned to firmware
    struct bst_lwnn_memblock *assigned_mem;
    //ipc source ARM core
    uint32_t ipc_src_core;
};

struct bst_lwnn_fw_manager {
    //the cv registers
    void __iomem *lb_cv_reg_base;
    dsp_ptr ipc_register_addr;
    struct bst_lwnn_dsp_fw_ctl dsps[BST_LWNN_MAX_DSP_NUM];
    struct _bst_lwnn_ver_info ver_info;
};

int bst_lwnn_fw_manager_init(struct bst_lwnn *pbst_lwnn);
int bst_lwnn_fw_rt_setup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_manager_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_rt_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_manager_exit(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_rt_exit(struct bst_lwnn *pbst_lwnn);

#endif

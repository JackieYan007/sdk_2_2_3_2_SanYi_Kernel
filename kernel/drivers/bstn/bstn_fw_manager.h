/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bstn_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          BSTN driver. It contains related constants and structure definitions
 *          and function declarations.
 */

#ifndef BSTN_FIRMWARE_H
#define BSTN_FIRMWARE_H

#define BSTN_FW_BOOT_UP_TIME_MS                     100
#define BSTN_FIRMWARE_DUMP_SIZE                     64
#define BSTN_FIRMWARE_DUMP_LINE_SIZE                8

/* Bit fields */
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

// reg definition
#define TOP_CRM_REG_BASE_ADDR			0x33002000
#define CLKMUX_SEL2				0x15c
typedef union {
	uint32_t all;
	struct {
		uint32_t pll_gmac_125m_clk_sel:2;
		uint32_t clk_800_cv_core_clk_sel:2;
		uint32_t clk_200_sdemmc1_clk_sel:1; // bit4
		uint32_t clk_200_sdemmc0_clk_sel:1;
		uint32_t clk_200_net_noc_clk_sel:2; // bit6:7
		uint32_t clk_800_net_clk_sel:2; // bit[9:8]
		uint32_t clk_800_dspcore_clk_sel:2;
		uint32_t clk_400_sysbus_common_clk_sel:2; // bit[13:12]
		uint32_t clk_800_sysbus_common_clk_sel:2;
		uint32_t clk_400_sysnoc_work_clk_sel:2; // bit[17:16]
		uint32_t clk_400_cpunoc_work_clk_sel:2;
		uint32_t clk_800_cpunoc_work_clk_sel:2; // bit[21:20]
		uint32_t clk_400_corenoc_work_clk_sel:2;
		uint32_t clk_800_corenoc_work_clk_sel:2; // bit[25:24]
		uint32_t clk_100_sysbus_apb_clk_sel:2;
		uint32_t clk_200_sysbus_apb_clk_sel:2; // bit[29:28]
		uint32_t clk_400_sysbus_apb_clk_sel:2;
	}b;
} clkmux_sel2_t;

// TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0 regsiter
#define TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0_OFFSET  0x4
#define LB_NET_SW_RST_N                             0x8

#define TOP_CRM_REG_R_CLKMUX_SEL1_OFFSET            0xa0

#define CLK_800_DSPCORE_SEL_BIT1                    20
#define CLK_800_DSPCORE_SEL_BIT2                    21
#define CLK_800_NET_SEL_BIT1                        18
#define CLK_800_NET_SEL_BIT2                        19

#define TOP_CRM_REG_R_CLKGATE_EN1_OFFSET            0xac

#define LB_NET_DSPCORE_CLK_I_GATE_EN_BIT            26
#define LB_NET_CLK_I_GATE_EN_BIT                    25
            
// BSTN Hardware Register offset & bit
#define BSTN_SYS_CTRL				0x0
typedef union {
	uint32_t all;
	struct {
		uint32_t rsvd_0:19; // bit[0:18]
		uint32_t soft_rst_dsp_debug_n:1; // bit19
		uint32_t net_debug_bus_sel:2; // bit[21:20]
		uint32_t net_parity_chk_en:1;
		uint32_t dsp_parity_chk_en:1;
		uint32_t dsp_axi_ecc_en:1;
		uint32_t rsvd_25:2; // bit[26:25]
		uint32_t dsp_runstall:1;
		uint32_t core_clk_en:1; // bit28
		uint32_t dsp_clk_en:1;
		uint32_t soft_rst_core_n:1;
		uint32_t soft_rst_dsp_n:1;
	}b;
} bstn_sys_ctrl_t;

#define BSTN_CORE0_CTRL				0x60
typedef union {
	uint32_t all;
	struct {
		uint32_t core0_greg_clk_en:1;
		uint32_t core0_gemm_clk_en:1;
		uint32_t core0_edp_clk_en:1;
		uint32_t core0_hctl_clk_en:1;
		uint32_t core0_dctl_clk_en:1; // bit4
		uint32_t core0_dbuf_clk_en:1;
		uint32_t core0_slice3_clk_en:1;
		uint32_t core0_slice2_clk_en:1;
		uint32_t core0_slice1_clk_en:1; // bit8
		uint32_t core0_slice0_clk_en:1;
		uint32_t core0_conv_clk_en:1;
		uint32_t core0_btmem_clk_en:1;
		uint32_t core0_ahb_clk_en:1; // bit12
		uint32_t core0_clk_en:1;
		uint32_t rsvd_14:2; // bit[15:14]
		uint32_t soft_rst_core0_greg_n:1; // bit16
		uint32_t soft_rst_core0_gemm_n:1;
		uint32_t soft_rst_core0_edp_n:1;
		uint32_t soft_rst_core0_hctl_n:1;
		uint32_t soft_rst_core0_dctl_n:1; // bit20
		uint32_t soft_rst_core0_dbuf_n:1;
		uint32_t soft_rst_core0_slice3_n:1;
		uint32_t soft_rst_core0_slice2_n:1;
		uint32_t soft_rst_core0_slice1_n:1; // bit24
		uint32_t soft_rst_core0_slice0_n:1;
		uint32_t soft_rst_core0_conv_n:1;
		uint32_t soft_rst_core0_btmem_n:1;
		uint32_t soft_rst_core0_ahb_n:1; // bit28
		uint32_t soft_rst_core0_n:1;
		uint32_t rsvd_30:2;
	}b;
} bstn_core0_ctrl_t;

#define BSTN_CORE1_CTRL				0x64
typedef union {
	uint32_t all;
	struct {
		uint32_t core1_greg_clk_en:1;
		uint32_t core1_gemm_clk_en:1;
		uint32_t core1_edp_clk_en:1;
		uint32_t core1_hctl_clk_en:1;
		uint32_t core1_dctl_clk_en:1; // bit4
		uint32_t core1_dbuf_clk_en:1;
		uint32_t core1_slice3_clk_en:1;
		uint32_t core1_slice2_clk_en:1;
		uint32_t core1_slice1_clk_en:1; // bit8
		uint32_t core1_slice0_clk_en:1;
		uint32_t core1_conv_clk_en:1;
		uint32_t core1_btmem_clk_en:1;
		uint32_t core1_ahb_clk_en:1; // bit12
		uint32_t core1_clk_en:1;
		uint32_t rsvd_14:2; // bit[15:14]
		uint32_t soft_rst_core1_greg_n:1; // bit16
		uint32_t soft_rst_core1_gemm_n:1;
		uint32_t soft_rst_core1_edp_n:1;
		uint32_t soft_rst_core1_hctl_n:1;
		uint32_t soft_rst_core1_dctl_n:1; // bit20
		uint32_t soft_rst_core1_dbuf_n:1;
		uint32_t soft_rst_core1_slice3_n:1;
		uint32_t soft_rst_core1_slice2_n:1;
		uint32_t soft_rst_core1_slice1_n:1; // bit24
		uint32_t soft_rst_core1_slice0_n:1;
		uint32_t soft_rst_core1_conv_n:1;
		uint32_t soft_rst_core1_btmem_n:1;
		uint32_t soft_rst_core1_ahb_n:1; // bit28
		uint32_t soft_rst_core1_n:1;
		uint32_t rsvd_30:2;
	}b;
} bstn_core1_ctrl_t;

#define DDRC0_CTRL                      0x38001000
#define DDRC1_CTRL                      0x3C001000
#define OCPARCFG0                       0x330
#define OC_PARITY_EN                    BIT0

// BSTN_SYS_CTRL_STATUS register bit
#define BSTN_DSP_SOFT_RESET_BIT                     31 //active-low
#define BSTN_NET_SOFT_RESET_BIT                     30 //active-low
#define BSTN_DSP_CLK_EN_BIT                         29
#define BSTN_CORE_CLK_EN_BIT                        28
#define BSTN_DSP_RUNSTALL_BIT                       27

#define BSTN_SYS_STATUS_OFFSET                      0x4
#define BSTN_DSP_ALT_RESET_VEC_OFFSET               0x8

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

// the initialization message body for runtime firmware
struct bstn_rt_setup_info {
    dsp_ptr assigned_mem;
    uint32_t assigned_mem_size;
    dsp_ptr rsp_addr;
};
// the initialization response body for runtime firmware
struct bstn_rt_setup_rsp {
  uint32_t release_date;
  uint8_t ver_major;
  uint8_t ver_minor;
  uint8_t ver_patch;
};

#pragma pack(pop)   /* restore original alignment from stack */

struct bstn_fw_manager {
    char *name;
    //the net registers
    void __iomem *net_sreg_base;
    //memory reserved for firmware codes
    void __iomem *fwmem_base;
    resource_size_t fwmem_size;
    phys_addr_t fwmem_phys_addr;
    struct bstn_memblock *assigned_mem; //memory assigned to firmware
    unsigned char fw_setup;
    unsigned char ver_major;
    unsigned char ver_minor;
    unsigned char ver_patch;
    unsigned char release_date;
    unsigned char release_month;
    unsigned short release_year;
};

int bstn_fw_manager_init(struct bstn_device *pbstn);
void bstn_fw_manager_exit(struct bstn_device *pbstn);
void bstn_fw_rt_exit(struct bstn_device *pbstn);
int bstn_firmware_load(struct bstn_device *pbstn);
void bstn_firmware_boot(struct bstn_device *pbstn);

#endif

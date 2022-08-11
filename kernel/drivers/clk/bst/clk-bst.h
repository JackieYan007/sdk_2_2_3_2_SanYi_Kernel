/*
* clock driver for BST A1000
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

#ifndef	__BST_CLK_H
#define	__BST_CLK_H

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/spinlock.h>

extern void __iomem *a1000_clk_base_addr[4];

/* pll config register base addr */
#define PLL_CPU_BASE        (a1000_clk_base_addr[0] + 0x08)
#define PLL_DSU_BASE        (a1000_clk_base_addr[0] + 0x28)
#define PLL_SYSBUS_BASE     (a1000_clk_base_addr[0] + 0x38)
#define PLL_DISP_BASE       (a1000_clk_base_addr[0] + 0x48)
#define PLL_COREIP_BASE     (a1000_clk_base_addr[0] + 0x68)
#define PLL_HSP_LSP_BASE    (a1000_clk_base_addr[0] + 0x78)
#define PLL_GMAC_BASE       (a1000_clk_base_addr[0] + 0x8C)

#define PLL_SAFE_MAIN       (a1000_clk_base_addr[1] + 0x10)
#define PLL_SAFE_LSP        (a1000_clk_base_addr[1] + 0x50)

/* mux/divider/gate register define */
#define TOP_CRM_REG_R_CLKMUX_SEL0                   (a1000_clk_base_addr[0] + 0x9C)
#define TOP_CRM_REG_R_CLKMUX_SEL1                   (a1000_clk_base_addr[0] + 0xA0)
#define TOP_CRM_REG_R_PLL_CLKMUX_SEL                (a1000_clk_base_addr[0] + 0xA4)
#define TOP_CRM_REG_R_CLKGATE_EN0                   (a1000_clk_base_addr[0] + 0xA8)
#define TOP_CRM_REG_R_CLKGATE_EN1                   (a1000_clk_base_addr[0] + 0xAC)
#define SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL        (a1000_clk_base_addr[1] + 0x00)
#define SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN         (a1000_clk_base_addr[1] + 0x04)
#define LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG             (a1000_clk_base_addr[2] + 0x04)
#define LB_LSP0_TOP_R_LSP0_DIV_CTRL_REG             (a1000_clk_base_addr[2] + 0x08)
#define LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG            (a1000_clk_base_addr[2] + 0x0C)
#define LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG             (a1000_clk_base_addr[3] + 0x04)
#define LB_LSP1_TOP_R_LSP1_DIV_CTRL_REG             (a1000_clk_base_addr[3] + 0x08)
#define LB_LSP1_TOP_R_LSP1_DIV_CTRL1_REG            (a1000_clk_base_addr[3] + 0x0C)

/* pll config register offset */
#define PLL_REG_CONFIG0     (0x0)
#define PLL_REG_CONFIG1     (0x4)
#define PLL_REG_CONFIG2     (0x8)
#define PLL_REG_STATUS      (0xC)

/* pll cpu/dsu/coreip/display/sysbus/hsp_lsp config item */
#define PLL_PLLEN           (5)
#define PLL_LOCK            (0)
#define PLL_DSMEN           (13) // 0 -> DSM is powered down (integer mode) 1 -> DSM is active (fractional mode)
#define PLL_FBDIV           (16)
#define PLL_FBDIV_MASK      (0xFFF)
#define PLL_PSTDIV1         (9)
#define PLL_PSTDIV2         (6)
#define PLL_PSTDIV1_MIN     (1)
#define PLL_PSTDIV2_MIN     (1)
#define PLL_PSTDIV1_MAX     (7)
#define PLL_PSTDIV2_MAX     (7)
#define PLL_PSTDIV_MASK     (0x7)
#define PLL_FOUT4PHASEEN    (12)
#define PLL_FRAC            (0)
#define PLL_FRAC_MASK       (0xFFFFFF)
#define PLL_FRAC_SHIFT      (24)

/* pll gmac config item */
#define PLL_GMAC_LOCK       (1)

/* pll main/lsp config item */
#define PLL_SEC_PLLEN           (0)
#define PLL_SEC_LOCK            (0)
#define PLL_SEC_DSMEN           (2)
#define PLL_SEC_FBDIV           (4)
#define PLL_SEC_FBDIV_MASK      (0xFFF)
#define PLL_SEC_PSTDIV1         (24)
#define PLL_SEC_PSTDIV2         (28)
#define PLL_SEC_PSTDIV1_MIN     (1)
#define PLL_SEC_PSTDIV2_MIN     (1)
#define PLL_SEC_PSTDIV1_MAX     (7)
#define PLL_SEC_PSTDIV2_MAX     (7)
#define PLL_SEC_PSTDIV_MASK     (0x7)
#define PLL_SEC_FOUT4PHASEEN    (31)
#define PLL_SEC_FRAC            (0)
#define PLL_SEC_FRAC_MASK       (0xFFFFFF)
#define PLL_SEC_FRAC_SHIFT      (24)

/* pll FOUT region */
#define PLL_FOUT_MAX_RATE       (3200000000) //12M
#define PLL_FOUT_MIN_RATE       (16000000) //5M

/* clock name macros */
#define MAIN_CLK                                "main_clk" //osc
#define PLL_CPU                                 "pll_cpu" //pll cpu
#define CLK_CPU_1400_SEL                        "clk_cpu_1400_sel"
#define CLK_CPU_1400_SEL_FIX_FACTOR_2_1         "clk_cpu_1400_sel_fix_factor_2_1"
#define CLK_1400_CPUCORE_SEL                    "clk_1400_cpucore_sel"
#define LB_CPU_CLK_GATE_EN                      "lb_cpu_clk_gate_en"
#define PLL_DSU                                 "pll_dsu" //pll dsu
#define CLK_DSU_1300_SEL                        "clk_dsu_1300_sel"
#define CLK_DSU_SEL                             "clk_dsu_sel"
#define LB_CPU_DSU_CLK_GATE_EN                  "lb_cpu_dsu_clk_gate_en"
#define PLL_HSP_LSP                             "pll_hsp_lsp" //pll hsp lsp
#define CLK_2000_SEL                            "clk_2000_sel"
#define CLK_2000_SEL_FIX_FACTOR_2_1             "clk_2000_sel_fix_factor_2_1"
#define CLK_2000_SEL_FIX_FACTOR_8_1             "clk_2000_sel_fix_factor_8_1"
#define CLK_2000_SEL_FIX_FACTOR_16_1            "clk_2000_sel_fix_factor_16_1"
#define CLK_2000_SEL_FIX_FACTOR_10_1            "clk_2000_sel_fix_factor_10_1"
#define CLK_2000_SEL_FIX_FACTOR_10_1_4_1        "clk_2000_sel_fix_factor_10_1_4_1"
#define PLL_HSP_LSP_PTD                         "pll_hsp_lsp_ptd" //pll hsp lsp postdiv
#define CLK_125_PTP_SEL                         "clk_125_ptp_sel"
#define PLL_GMAC_PTD                            "pll_gmac_ptd" //pll gmac
#define PLL_GMAC_REF_25M_125M_SEL               "pll_gmac_ref_25m_125m_sel"
#define PLL_GMAC_REF_CLK_SEL                    "pll_gmac_ref_clk_sel"
#define PLL_GMAC_PTD_FIX_FACTOR_4_1             "pll_gmac_fix_factor_4_1"
#define LB_GMAC0_WCLK_GATE_EN                   "lb_gmac0_wclk_gate_en"
#define LB_GMAC1_WCLK_GATE_EN                   "lb_gmac1_wclk_gate_en"
#define LB_GMAC0_AXIM_ACLK_GATE_EN              "lb_gmac0_axim_aclk_gate_en"
#define LB_GMAC1_AXIM_ACLK_GATE_EN              "lb_gmac1_axim_aclk_gate_en"
#define LB_GMAC0_APB_S_PCLK_GATE_EN             "lb_gmac0_apb_s_pclk_gate_en"
#define LB_GMAC1_APB_S_PCLK_GATE_EN             "lb_gmac1_apb_s_pclk_gate_en"
#define LB_GMAC0_PTP_CLK_GATE_EN                "lb_gmac0_ptp_clk_gate_en"
#define LB_GMAC1_PTP_CLK_GATE_EN                "lb_gmac1_ptp_clk_gate_en"
#define RGMII0_RXCLK                            "rgmii0_rxclk"
#define RGMII1_RXCLK                            "rgmii1_rxclk"
#define PTP_CLK                                 "ptp_clk"
#define PLL_DISP_PTD                            "pll_display_ptd" //pll display postdiv
#define CLK_DISPLAY_1650_SEL                    "clk_display_1650_sel"
#define CLK_DISPLAY_1650_SEL_FIX_FACTOR_10_1    "clk_display_1650_sel_fix_factor_10_1"
#define PLL_SYSBUS                              "pll_sysbus" //pll sysbus
#define PLL_SYSBUS_PTD                          "pll_sysbus_ptd" //pll sysbus postdiv
#define CLK_800_SYSBUS_SEL                      "clk_800_sysbus_sel"
#define CLK_200_SYSBUS_APB_SEL                  "clk_200_sysbus_apb_sel"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1       "clk_800_sysbus_sel_fix_factor_2_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1       "clk_800_sysbus_sel_fix_factor_4_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1       "clk_800_sysbus_sel_fix_factor_8_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1      "clk_800_sysbus_sel_fix_factor_16_1"
#define CLK_800_SYSBUS_AXI_SEL                  "clk_800_sysbus_axi_sel"
#define CLK_100_SYSBUS_APB_SEL                  "clk_100_sysbus_apb_sel"
#define LB_LPDDR0_S0_ACLK_GATE_EN               "lb_lpddr0_s0_aclk_gate_en"
#define LB_LPDDR0_S1_ACLK_GATE_EN               "lb_lpddr0_s1_aclk_gate_en"
#define LB_LPDDR0_S2_ACLK_GATE_EN               "lb_lpddr0_s2_aclk_gate_en"
#define LB_LPDDR0_S3_ACLK_GATE_EN               "lb_lpddr0_s3_aclk_gate_en"
#define LB_LPDDR1_S0_ACLK_GATE_EN               "lb_lpddr1_s0_aclk_gate_en"
#define LB_LPDDR1_S1_ACLK_GATE_EN               "lb_lpddr1_s1_aclk_gate_en"
#define LB_LPDDR1_S2_ACLK_GATE_EN               "lb_lpddr1_s2_aclk_gate_en"
#define LB_LPDDR1_S3_ACLK_GATE_EN               "lb_lpddr1_s3_aclk_gate_en"
#define LB_LPDDR0_PCLK_GATE_EN                  "lb_lpddr0_pclk_gate_en"
#define LB_LPDDR1_PCLK_GATE_EN                  "lb_lpddr1_pclk_gate_en"
#define CLK_2400_SYSBUS_SEL                     "clk_2400_sysbus_sel"
#define CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1      "clk_2400_sysbus_sel_fix_factor_4_1"
#define CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1_2_1  "clk_2400_sysbus_sel_fix_factor_4_1_2_1"
#define CLK_800_CORENOC_WORK_SEL                "clk_800_corenoc_work_sel"
#define CLK_800_CPUNOC_WORK_SEL                 "clk_800_cpunoc_work_sel"
#define CLK_400_SYSNOC_WORK_SEL                 "clk_400_sysnoc_work_sel"
#define CLK_800_GDMA_AXI_SEL                    "clk_800_gdma_axi_sel"
#define CLK_400_SYSBUS_AXI_SEL                  "clk_400_sysbus_axi_sel"
#define LB_GDMA_AXI_ACLK_GATE_EN                "lb_gdma_axi_aclk_gate_en"
#define CLK_100_SYSBUS_APB_SEL                  "clk_100_sysbus_apb_sel"
#define LB_PCIE_AXI_ACLK_GATE_EN                "lb_pcie_axi_aclk_gate_en"
#define LB_PCIE_AUX_CLK_GATE_EN                 "lb_pcie_aux_clk_gate_en"
#define LB_PCIE_APB_PCLK_GATE_EN                "lb_pcie_apb_pclk_gate_en"
#define CLK_200_SYSBUS_AHB_SEL                  "clk_200_sysbus_ahb_sel"
#define LB_USB2_AHB_HCLK_GATE_EN                "lb_usb2_ahb_hclk_gate_en"
#define LB_USB2_REF_ALT_CLK_GATE_EN             "lb_usb2_ref_ali_clk_gate_en"
#define LB_USB2_APB_PCLK_GATE_EN                "lb_usb2_apb_pclk_gate_en"
#define LB_USB3_REF_ALT_CLK_GATE_EN             "lb_usb3_ref_alt_clk_gate_en"
#define LB_USB3_AXI_ACLK_GATE_EN                "lb_usb3_axi_aclk_gate_en"
#define LB_USB3_APB_PCLK_GATE_EN                "lb_usb3_apb_pclk_gate_en"
#define CLK_200_SDEMMC0_SEL                     "clk_200_sdemmc0_sel"
#define CLK_200_SDEMMC1_SEL                     "clk_200_sdemmc1_sel"
#define LB_SDEMMC0_W_BCLK_GATE_EN               "lb_sdemmc0_w_bclk_gate_en"
#define LB_SDEMMC0_HCLK_GATE_EN                 "lb_sdemmc0_hclk_gate_en"
#define LB_SDEMMC1_W_BCLK_GATE_EN               "lb_sdemmc1_w_bclk_gate_en"
#define LB_SDEMMC1_HCLK_GATE_EN                 "lb_sdemmc1_hclk_gate_en"
#define CLK_400_GDMA_CORE_SEL                   "clk_400_gdma_core_sel"
#define LB_GDMA_CORE_CLK_GATE_EN                "lb_gdma_core_clk_gate_en"
#define LB_GDMA_AHB_CLK_GATE_EN                 "lb_gdma_ahb_clk_gate_en"
#define PLL_MAIN_PTD                            "pll_main_ptd" //pll main postdiv
#define PLL_MAIN_PTD_FIX_FACTOR_2_1             "pll_main_ptd_fix_factor_2_1"
#define PLL_MAIN_PTD_FIX_FACTOR_4_1             "pll_main_ptd_fix_factor_4_1"
#define PLL_MAIN_PTD_FIX_FACTOR_8_1             "pll_main_ptd_fix_factor_8_1"
#define SEC_SAFE_CLK_SEL0                       "sec_safe_clk_sel0"
#define SEC_SAFE_CLK_SEL1                       "sec_safe_clk_sel1"
#define SEC_SAFE_CLK_SEL2                       "sec_safe_clk_sel2"
#define SEC_SAFE_CLK_SEL3                       "sec_safe_clk_sel3"
#define SEC_SAFE_CLK_SEL4                       "sec_safe_clk_sel4"
#define SEC_SAFE_CLK_SEL3_FIX_FACTOR_2_1        "sec_safe_clk_sel3_fix_factor_2_1"
#define CLK_QSPI_REF_DIV                        "clk_qspi_ref_div"
#define SEC_SAFE_CLK_SEL_9_8                    "sec_safe_clk_sel_9_8"
#define QSPI0_HCLK_EN                           "qspi0_hclk_en"
#define QSPI0_WCLK_EN                           "qspi0_wclk_en"
#define QSPI1_HCLK_EN                           "qspi1_hclk_en"
#define QSPI1_WCLK_EN                           "qspi1_wclk_en"
#define GIC_WCLK_EN                             "gic_wclk_en"
#define LSP0_PCLK_EN                            "lsp0_pclk_en"
#define LSP0_WCLK_EN                            "lsp0_wclk_en"
#define LSP0_I2SM_CLK_EN                        "lsp0_i2sm_clk_en"
#define UART_WCLK_EN                            "uart_wclk_en"
#define LB_SEC_SAFE_CANFD_200M_CLK_GATE_EN      "lb_sec_safe_canfd_200m_clk_gate_en"
#define CAN_FD0_WCLK_EN                         "can_fd0_wclk_en"
#define CAN_FD1_WCLK_EN                         "can_fd1_wclk_en"
#define SEC_SAFE_CLK_SEL_7                      "sec_safe_clk_sel_7"
#define CAN_FD0_WCLK_GATE_EN                    "can_fd0_wclk_gate_en"
#define CAN_FD0_PCLK_GATE_EN                    "can_fd0_pclk_gate_en"
#define CAN_FD1_WCLK_GATE_EN                    "can_fd1_wclk_gate_en"
#define CAN_FD1_PCLK_GATE_EN                    "can_fd1_pclk_gate_en"
#define LSP1_PCLK_EN                            "lsp1_pclk_en"
#define LSP1_WCLK_EN                            "lsp1_wclk_en"
#define PLL_LSP                                 "pll_lsp" //pll lsp
#define PLL_LSP_PTD                             "pll_lsp_ptd" //pll lsp postdiv
#define PLL_LSP_PTD_FIX_FACTOR                  "pll_lsp_ptd_fix_factor"
#define PLL_COREIP_PTD                          "pll_coreip_ptd" //pll coreip postdiv
#define PLL_COREIP                              "pll_coreip"
#define PLL_COREIP_PTD_FIX_FACTOR               "pll_coreip_ptd_fix_factor"
#define CLK_800_COREIP_SEL                      "clk_800_coreip_sel"
#define CLK_800_COREIP_SEL_FIX_FACTOR_2_1       "clk_800_coreip_sel_fix_factor_2_1"
#define CLK_800_COREIP_SEL_FIX_FACTOR_4_1       "clk_800_coreip_sel_fix_factor_4_1"
#define MAIN_CLK_FIX_FACTOR_20_1                "main_clk_fix_factor_20_1"
#define CLK_800_CV_CORE_SEL                     "clk_800_cv_core_sel"
#define LB_CV_CORE_CLK_GATE_EN                  "lb_cv_core_clk_gate_en"
#define LB_CV_AXIM0_CLK_GATE_EN                 "lb_cv_axim0_clk_gate_en"
#define LB_CV_AXIM1_CLK_GATE_EN                 "lb_cv_axim1_clk_gate_en"
#define LB_CV_AXIS_CLK_GATE_EN                  "lb_cv_axis_clk_gate_en"
#define CLK_800_NET_SEL                         "clk_800_net_sel"
#define CLK_800_DSPCORE_SEL                     "clk_800_dspcore_sel"
#define LB_NET_CLK_I_GATE_EN                    "lb_net_clk_i_gate_en"
#define LB_NET_DSPCORE_CLK_I_GATE_EN            "lb_net_dspcore_clk_i_gate_en"
#define LB_ISP_SCLK_GATE_EN                     "lb_isp_sclk_gate_en"
#define LB_ISP_AHB_HCLK_GATE_EN                 "lb_isp_ahb_hclk_gate_en"
#define LB_MIPI0_APB_CFG_CLK_GATE_EN            "lb_mipi0_apb_cfg_clk_gate_en"
#define LB_MIPI1_APB_CFG_CLK_GATE_EN            "lb_mipi1_apb_cfg_clk_gate_en"
#define LB_MIPI2_APB_CFG_CLK_GATE_EN            "lb_mipi2_apb_cfg_clk_gate_en"
#define LB_VSP_DISP_CLK_GATE_EN                 "lb_vsp_disp_clk_gate_en"
#define LB_VSP_CLK_GATE_EN                      "lb_vsp_clk_gate_en"
#define LB_VSP_AHB_HCLK_GATE_EN                 "lb_vsp_ahb_hclk_gate_en"
#define CLK_800_GPU_SEL                         "clk_800_gpu_sel"
#define LB_GPU_AXI_ACLK_RSTSYNC_I_GATE_EN       "lb_gpu_axi_aclk_rstsync_i_gate_en"
#define LB_MATRIX_IPC_APB_S_PCLK_GATE_EN        "lb_matrix_ipc_apb_s_pclk_gate_en"
#define LB_MATRIX_TEMPSENSOR_1M25_CLK_GATE_EN   "lb_matrix_tempsensor_1m25_clk_gate_en"
#define LB_TEMP_25M_CLK_GATE_EN                 "lb_temp_25m_clk_gate_en"
#define LB_GPU_APB_S_PCLK_EN                    "lb_gpu_apb_s_pclk_en"
#define LSP0_UART0_DIVIDER                      "lsp0_uart0_divider"
#define LSP0_UART1_DIVIDER                      "lsp0_uart1_divider"
#define LSP0_UART0_WCLK_GATE_EN                 "lsp0_uart0_wclk_gate_en"
#define LSP0_UART1_WCLK_GATE_EN                 "lsp0_uart1_wclk_gate_en"
#define LSP0_UART0_PCLK_GATE_EN                 "lsp0_uart0_pclk_gate_en"
#define LSP0_UART1_PCLK_GATE_EN                 "lsp0_uart1_pclk_gate_en"
#define LSP1_UART0_DIVIDER                      "lsp1_uart0_divider"
#define LSP1_UART1_DIVIDER                      "lsp1_uart1_divider"
#define LSP1_UART0_WCLK_GATE_EN                 "lsp1_uart0_wclk_gate_en"
#define LSP1_UART1_WCLK_GATE_EN                 "lsp1_uart1_wclk_gate_en"
#define LSP1_UART0_PCLK_GATE_EN                 "lsp1_uart0_pclk_gate_en"
#define LSP1_UART1_PCLK_GATE_EN                 "lsp1_uart1_pclk_gate_en"
#define LSP0_I2C_WCLK_FIX_FACTOR_2_1            "lsp0_i2c_wclk_fix_factor_2_1"
#define LSP0_I2C_MUX                            "lsp0_i2c_mux"
#define LSP0_I2C0_WCLK_GATE_EN                  "lsp0_i2c0_wclk_gate_en"
#define LSP0_I2C1_WCLK_GATE_EN                  "lsp0_i2c1_wclk_gate_en"
#define LSP0_I2C2_WCLK_GATE_EN                  "lsp0_i2c2_wclk_gate_en"
#define LSP0_I2C0_PCLK_GATE_EN                  "lsp0_i2c0_pclk_gate_en"
#define LSP0_I2C1_PCLK_GATE_EN                  "lsp0_i2c1_pclk_gate_en"
#define LSP0_I2C2_PCLK_GATE_EN                  "lsp0_i2c2_pclk_gate_en"
#define LSP1_I2C_WCLK_FIX_FACTOR_2_1            "lsp1_i2c_wclk_fix_factor_2_1"
#define LSP1_I2C_MUX                            "lsp1_i2c_mux"
#define LSP1_I2C3_WCLK_GATE_EN                  "lsp1_i2c3_wclk_gate_en"
#define LSP1_I2C4_WCLK_GATE_EN                  "lsp1_i2c4_wclk_gate_en"
#define LSP1_I2C5_WCLK_GATE_EN                  "lsp1_i2c5_wclk_gate_en"
#define LSP1_I2C3_PCLK_GATE_EN                  "lsp1_i2c3_pclk_gate_en"
#define LSP1_I2C4_PCLK_GATE_EN                  "lsp1_i2c4_pclk_gate_en"
#define LSP1_I2C5_PCLK_GATE_EN                  "lsp1_i2c5_pclk_gate_en"
#define LSP0_TIMER_DIVIDER                      "lsp0_timer_divider"
#define LSP0_TIMER0_WCLK_GATE_EN                "lsp0_timer0_wclk_gate_en"
#define LSP0_TIMER1_WCLK_GATE_EN                "lsp0_timer1_wclk_gate_en"
#define LSP0_TIMER0_PCLK_GATE_EN                "lsp0_timer0_pclk_gate_en"
#define LSP0_TIMER1_PCLK_GATE_EN                "lsp0_timer1_pclk_gate_en"
#define LSP1_TIMER_DIVIDER                      "lsp1_timer_divider"
#define LSP1_TIMER2_WCLK_GATE_EN                "lsp1_timer2_wclk_gate_en"
#define LSP1_TIMER3_WCLK_GATE_EN                "lsp1_timer3_wclk_gate_en"
#define LSP1_TIMER2_PCLK_GATE_EN                "lsp1_timer2_pclk_gate_en"
#define LSP1_TIMER3_PCLK_GATE_EN                "lsp1_timer3_pclk_gate_en"
#define LSP0_SPI0_DIVIDER                       "lsp0_spi0_divider"
#define LSP0_SPI0_WCLK_GATE_EN                  "lsp0_spi0_wclk_gate_en"
#define LSP0_SPI0_PCLK_GATE_EN                  "lsp0_spi0_pclk_gate_en"
#define LSP1_SPI1_DIVIDER                       "lsp1_spi1_divider"
#define LSP1_SPI1_WCLK_GATE_EN                  "lsp1_spi1_wclk_gate_en"
#define LSP1_SPI1_PCLK_GATE_EN                  "lsp1_spi1_pclk_gate_en"
#define LSP0_WDT0_DIVIDER                       "lsp0_wdt0_divider"
#define LSP0_WDT1_DIVIDER                       "lsp0_wdt1_divider"
#define LSP0_WDT0_WCLK_GATE_EN                  "lsp0_wdt0_wclk_gate_en"
#define LSP0_WDT1_WCLK_GATE_EN                  "lsp0_wdt1_wclk_gate_en"
#define LSP0_WDT0_PCLK_GATE_EN                  "lsp0_wdt0_pclk_gate_en"
#define LSP0_WDT1_PCLK_GATE_EN                  "lsp0_wdt1_pclk_gate_en"
#define LSP1_WDT2_DIVIDER                       "lsp1_wdt2_divider"
#define LSP1_WDT3_DIVIDER                       "lsp1_wdt3_divider"
#define LSP1_WDT2_WCLK_GATE_EN                  "lsp1_wdt2_wclk_gate_en"
#define LSP1_WDT3_WCLK_GATE_EN                  "lsp1_wdt3_wclk_gate_en"
#define LSP1_WDT2_PCLK_GATE_EN                  "lsp1_wdt2_pclk_gate_en"
#define LSP1_WDT3_PCLK_GATE_EN                  "lsp1_wdt3_pclk_gate_en"
#define LSP0_I2SM_DIVIDER                       "lsp0_i2sm_divider"
#define LSP0_I2SM_SCK_GATE_EN                   "lsp0_i2sm_sck_gate_en"
#define LSP0_I2SM_PCLK_GATE_EN                  "lsp0_i2sm_pclk_gate_en"
#define LSP1_I2SS_SCK_GATE_EN                   "lsp1_i2ss_sck_gate_en"
#define LSP1_I2SS_PCLK_GATE_EN                  "lsp1_i2ss_pclk_gate_en"
#define LSP0_GPIO_PCLK_GATE_EN                  "lsp0_gpio_pclk_gate_en"
#define LSP1_GPIO_PCLK_GATE_EN                  "lsp1_gpio_pclk_gate_en"

/* clock resource id enum */
enum CLK_RES_ID {
    RES_IDX_TOP_CRM = 0,
    RES_IDX_SEC_SAFE_SYS_CTRL,
    RES_IDX_LB_LSP0_TOP,
    RES_IDX_LB_LSP1_TOP
};

/**
 * struct bst_pll_mux: record the muxs after pll, when change PLL set should first
 *                     change mux output to osc and after PLL stable change back to pll output
 * @mux_addr:		mux register addr
 * @mux_oft:	    mux start bit offset in mux_addr
 * @mux_width:	    mux bits length
 * @mux_osc_val:    the value to use osc output
 * @mux_pll_val:    the value to use pll output
 */
typedef struct _bst_pll_mux_ {
	void __iomem *mux_addr;
	u8 mux_oft;
    u8 mux_width;
    u8 mux_osc_val;
    u8 mux_pll_val;
}bst_pll_mux;

/**
 * struct bst_pll
 * @hw:		        Handle between common and hardware-specific interfaces
 * @base_addr:	    pll register base addr
 * @pll_spinlock:   spinlock to protect race condition
 * @bst_pll_mux:	record the all muxs connect to pll directly
 * @pll_mux_cnt:    total mux number
 */
struct bst_pll {
	struct clk_hw hw;
	void __iomem *base_addr;
	spinlock_t *pll_spinlock;
    bst_pll_mux *pll_mux;
    u32 pll_mux_cnt;
};
#define to_bst_pll(_hw)	container_of(_hw, struct bst_pll, hw)


#endif	/* __BST_CLK_H */

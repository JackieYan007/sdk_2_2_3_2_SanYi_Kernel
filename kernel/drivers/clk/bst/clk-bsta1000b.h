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

extern void __iomem *a1000b_clk_base_addr[4];

/* pll config register base addr */
#define PLL_CPU_BASE        (a1000b_clk_base_addr[0] + 0x08)
#define PLL_DSU_BASE        (a1000b_clk_base_addr[0] + 0x28)
#define PLL_SYSBUS_BASE     (a1000b_clk_base_addr[0] + 0x38)
#define PLL_DISP_BASE       (a1000b_clk_base_addr[0] + 0x48)
#define PLL_COREIP_BASE     (a1000b_clk_base_addr[0] + 0x68)
#define PLL_HSP_LSP_BASE    (a1000b_clk_base_addr[0] + 0x78)
#define PLL_GMAC_BASE       (a1000b_clk_base_addr[0] + 0x8C)

#define PLL_SAFE_MAIN       (a1000b_clk_base_addr[1] + 0x10)
#define PLL_SAFE_LSP        (a1000b_clk_base_addr[1] + 0x50)

/* mux/divider/gate register define */
#define TOP_CRM_REG_R_CLKMUX_SEL0                   (a1000b_clk_base_addr[0] + 0x154)
#define TOP_CRM_REG_R_CLKMUX_SEL1                   (a1000b_clk_base_addr[0] + 0x158)
#define TOP_CRM_REG_R_CLKMUX_SEL2                   (a1000b_clk_base_addr[0] + 0x15C)
#define TOP_CRM_REG_R_PLL_CLKMUX_SEL                (a1000b_clk_base_addr[0] + 0x160)
#define TOP_CRM_REG_R_CLKGATE_EN0                   (a1000b_clk_base_addr[0] + 0x164)
#define TOP_CRM_REG_R_CLKGATE_EN1                   (a1000b_clk_base_addr[0] + 0x168)
#define TOP_CRM_REG_R_CLKGATE_EN2                   (a1000b_clk_base_addr[0] + 0x16C)
#define SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_SEL        (a1000b_clk_base_addr[1] + 0x00)
#define SEC_SAFE_SYS_CTRL_R_SEC_SAFE_CLK_EN         (a1000b_clk_base_addr[1] + 0x04)
#define LB_LSP0_TOP_R_LSP0_GLB_CTRL_REG             (a1000b_clk_base_addr[2] + 0x04)
#define LB_LSP0_TOP_R_LSP0_DIV_CTRL_REG             (a1000b_clk_base_addr[2] + 0x08)
#define LB_LSP0_TOP_R_LSP0_DIV_CTRL1_REG            (a1000b_clk_base_addr[2] + 0x0C)
#define LB_LSP0_TOP_R_LSP0_DIV_CTRL2_REG            (a1000b_clk_base_addr[2] + 0x10)
#define LB_LSP1_TOP_R_LSP1_GLB_CTRL_REG             (a1000b_clk_base_addr[3] + 0x04)
#define LB_LSP1_TOP_R_LSP1_DIV_CTRL_REG             (a1000b_clk_base_addr[3] + 0x08)
#define LB_LSP1_TOP_R_LSP1_DIV_CTRL1_REG            (a1000b_clk_base_addr[3] + 0x0C)

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
#define PLL_FOUTPOSTDIVEN    (28)
#define PLL_FRAC            (0)
#define PLL_FRAC_MASK       (0xFFFFFF)
#define PLL_FRAC_SHIFT      (24)

/* pll FOUT region */
#define PLL_FOUT_MAX_RATE       (3200000000) //12M
#define PLL_FOUT_MIN_RATE       (16000000) //5M

/* clock name macros */
#define MAIN_CLK                                "main_clk" //osc
#define PLL_CPU                                 "pll_cpu" //pll cpu
#define PLL_CPU_PTD                             "pll_cpu_ptd" //pll cpu ptd
#define CLK_CPU_1400_SEL                        "clk_cpu_1400_sel"
#define CLK_CPU_1400_SEL_FIX_FACTOR_2_1         "clk_cpu_1400_sel_fix_factor_2_1"
#define CLK_CPU_2800_SEL                        "clk_cpu_2800_sel"
#define CLK_CPU_2800_SEL_FIX_FACTOR_3_1         "clk_cpu_2800_sel_fix_factor_3_1"
#define CLK_DSU_1200_SEL_FIX_FACTOR_2_1         "clk_dsu_1200_sel_fix_factor_2_1"
#define CLK_DSU_1200_SEL_FIX_FACTOR_4_1         "clk_dsu_1200_sel_fix_factor_4_1"
#define CLK_1400_CPUCORE_SEL                    "clk_1400_cpucore_sel"
#define CLK_1000_GPU_SEL                        "clk_1000_gpu_sel"
#define LB_GPU_GWCLK_EN                         "lb_gpu_gwclk_en"
#define PLL_DSU                                 "pll_dsu" //pll dsu
#define CLK_DSU_1200_SEL                        "clk_dsu_1200_sel"
#define CLK_DSU_SEL                             "clk_dsu_sel"
#define CLK_CPU_ACE_SEL                         "clk_cpu_ace_sel"
#define CLK_CPU_PPI_SEL                         "clk_cpu_ppi_sel"
#define LB_CPU_DSU_CLK_GATE_EN                  "lb_cpu_dsu_clk_gate_en"
#define PLL_HSP_LSP                             "pll_hsp_lsp" //pll hsp lsp
#define CLK_2000_SEL                            "clk_2000_sel"
#define CLK_2000_SEL_FIX_FACTOR_2_1             "clk_2000_sel_fix_factor_2_1"
#define CLK_2000_SEL_FIX_FACTOR_4_1             "clk_2000_sel_fix_factor_4_1"
#define CLK_2000_SEL_FIX_FACTOR_8_1             "clk_2000_sel_fix_factor_8_1"
#define CLK_2000_SEL_FIX_FACTOR_16_1            "clk_2000_sel_fix_factor_16_1"
#define CLK_2000_SEL_FIX_FACTOR_10_1            "clk_2000_sel_fix_factor_10_1"
#define PLL_HSP_LSP_PTD                         "pll_hsp_lsp_ptd" //pll hsp lsp postdiv
#define CLK_125_PTP_SEL                         "clk_125_ptp_sel"
#define CLK_GMAC_AXI_SEL                        "clk_gmac_axi_sel"
#define PLL_GMAC_REFCLK_SEL                     "pll_gmac_refclk_sel"
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
#define CLK_600_VSP_CLK_SEL                     "clk_600_vsp_clk_sel"
#define CLK_CORENOC_VSP_NIC3X1_400_SEL          "clk_corenoc_vsp_nic3x1_400_sel"
#define PLL_SYSBUS                              "pll_sysbus" //pll sysbus
#define PLL_SYSBUS_PTD                          "pll_sysbus_ptd" //pll sysbus postdiv
#define CLK_800_SYSBUS_SEL                      "clk_800_sysbus_sel"
#define CLK_800_SYSBUS_COMMON_SEL               "clk_800_sysbus_common_sel"
#define CLK_200_SYSBUS_APB_SEL                  "clk_200_sysbus_apb_sel"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_2_1       "clk_800_sysbus_sel_fix_factor_2_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_4_1       "clk_800_sysbus_sel_fix_factor_4_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_8_1       "clk_800_sysbus_sel_fix_factor_8_1"
#define CLK_800_SYSBUS_SEL_FIX_FACTOR_16_1      "clk_800_sysbus_sel_fix_factor_16_1"
#define CLK_800_DDR_S0_AXI_SEL                  "clk_800_ddr_s0_axi_sel"
#define CLK_800_DDR_S1_AXI_SEL                  "clk_800_ddr_s1_axi_sel"
#define CLK_800_DDR_S2_AXI_SEL                  "clk_800_ddr_s2_axi_sel"
#define CLK_800_DDR_S3_AXI_SEL                  "clk_800_ddr_s3_axi_sel"
#define CLK_100_SYSBUS_APB_SEL                  "clk_100_sysbus_apb_sel"
#define LB_LPDDR0_S0_AXI_GACLK_GATE_EN          "lb_lpddr0_s0_axi_gaclk_gate_en"
#define LB_LPDDR0_S1_AXI_GACLK_GATE_EN          "lb_lpddr0_s1_axi_gaclk_gate_en"
#define LB_LPDDR0_S2_AXI_GACLK_GATE_EN          "lb_lpddr0_s2_axi_gaclk_gate_en"
#define LB_LPDDR0_S3_AXI_GACLK_GATE_EN          "lb_lpddr0_s3_axi_gaclk_gate_en"
#define LB_LPDDR1_S0_AXI_GACLK_GATE_EN          "lb_lpddr1_s0_axi_gaclk_gate_en"
#define LB_LPDDR1_S1_AXI_GACLK_GATE_EN          "lb_lpddr1_s1_axi_gaclk_gate_en"
#define LB_LPDDR1_S2_AXI_GACLK_GATE_EN          "lb_lpddr1_s2_axi_gaclk_gate_en"
#define LB_LPDDR1_S3_AXI_GACLK_GATE_EN          "lb_lpddr1_s3_axi_gaclk_gate_en"
#define LB_LPDDR0_GPCLK_GATE_EN                 "lb_lpddr0_gpclk_gate_en"
#define LB_LPDDR1_GPCLK_GATE_EN                 "lb_lpddr1_gpclk_gate_en"
#define CLK_2400_SYSBUS_SEL                     "clk_2400_sysbus_sel"
#define CLK_2400_SYSBUS_SEL_FIX_FACTOR_4_1      "clk_2400_sysbus_sel_fix_factor_4_1"
#define CLK_2400_SYSBUS_SEL_FIX_FACTOR_8_1      "clk_2400_sysbus_sel_fix_factor_8_1"
#define CLK_2400_SYSBUS_SEL_FIX_FACTOR_48_1     "clk_2400_sysbus_sel_fix_factor_48_1"
#define CLK_400_GDMA_AXI_SEL                    "clk_400_gdma_axi_sel"
#define CLK_400_SYSBUS_AXI_SEL                  "clk_400_sysbus_axi_sel"
#define CLK_400_ISP_SCLK_SEL                    "clk_400_isp_sclk_sel"
#define LB_GDMA_AXI_GACLK_GATE_EN               "lb_gdma_axi_gaclk_gate_en"
#define LB_MATRIX_IPC_APB_S_PCLK_GATE_EN        "lb_matrix_ipc_apb_s_pclk_gate_en"
#define CLK_PCIE_AXI_SEL                        "clk_pcie_axi_sel"
#define LB_PCIE_REF_ALT_CLK_P_GATE_EN           "lb_pcie_ref_alt_clk_p_gate_en"
#define LB_PCIE_AXI_ACLK_GATE_EN                "lb_pcie_axi_aclk_gate_en"
#define LB_PCIE_AUX_CLK_GATE_EN                 "lb_pcie_aux_clk_gate_en"
#define LB_PCIE_APB_PCLK_GATE_EN                "lb_pcie_apb_pclk_gate_en"
#define CLK_200_SYSBUS_AHB_SEL                  "clk_200_sysbus_ahb_sel"
#define LB_USB2_AHB_HCLK_GATE_EN                "lb_usb2_ahb_hclk_gate_en"
#define LB_USB2_REF_ALT_CLK_GATE_EN             "lb_usb2_ref_ali_clk_gate_en"
#define LB_USB2_APB_PCLK_GATE_EN                "lb_usb2_apb_pclk_gate_en"
#define LB_USB3_SUSPEND_CLK_EN                  "lb_usb3_suspend_clk_en"
#define LB_USB3_REF_ALT_CLK_GATE_EN             "lb_usb3_ref_alt_clk_gate_en"
#define CLK_USB3_AXI_SEL                        "clk_usb3_axi_sel"
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
#define LB_GDMA_AHB_GHCLK_GATE_EN               "lb_gdma_ahb_ghclk_gate_en"
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
#define CLK_MAIN_800M_FIX_FACTOR_4_1            "clk_main_800m_fix_factor_4_1"
#define SEC_SAFE_CLK_SEL_9_8                    "sec_safe_clk_sel_9_8"
#define SEC_SAFE_CLK_SEL_12                     "sec_safe_clk_sel_12"
#define SEC_SAFE_CLK_SEL_11                     "sec_safe_clk_sel_11"
#define QSPI0_GHCLK_EN                           "qspi0_ghclk_en"
#define QSPI0_GWCLK_EN                           "qspi0_gwclk_en"
#define QSPI1_GHCLK_EN                           "qspi1_ghclk_en"
#define QSPI1_GWCLK_EN                           "qspi1_gwclk_en"
#define GIC_WCLK_EN                             "gic_wclk_en"
#define LSP0_PCLK_EN                            "lsp0_pclk_en"
#define LSP0_WCLK_EN                            "lsp0_wclk_en"
#define LSP0_I2SM_CLK_EN                        "lsp0_i2sm_clk_en"
#define UART_WCLK_EN                            "uart_wclk_en"
#define LB_SEC_SAFE_CANFD_200M_CLK_GATE_EN      "lb_sec_safe_canfd_200m_clk_gate_en"
#define CAN_FD0_WCLK_EN                         "can_fd0_wclk_en"
#define CAN_FD1_WCLK_EN                         "can_fd1_wclk_en"
#define SEC_SAFE_CLK_SEL_7                      "sec_safe_clk_sel_7"
#define LSP0_CAN0_W_GCLK_GATE_EN                "lsp0_can0_w_gclk_gate_en"
#define LSP0_CAN0_P_GCLK_GATE_EN                "lsp0_can0_p_gclk_gate_en"
#define LSP0_CAN1_W_GCLK_GATE_EN                "lsp0_can1_w_gclk_gate_en"
#define LSP0_CAN1_P_GCLK_GATE_EN                "lsp0_can1_p_gclk_gate_en"
#define LSP0_CAN_GWCLK_GATE_EN                  "lsp0_can_gwclk_gate_en"
#define LSP1_CAN_W_GCLK_GATE_EN                 "lsp1_can_w_gclk_gate_en"
#define LSP1_CAN_P_GCLK_GATE_EN                 "lsp1_can_p_gclk_gate_en"
#define LSP1_CAN_GWCLK_GATE_EN                  "lsp1_can_gwclk_gate_en"
#define LSP1_PCLK_EN                            "lsp1_pclk_en"
#define LSP1_WCLK_EN                            "lsp1_wclk_en"
#define PLL_LSP                                 "pll_lsp" //pll lsp
#define PLL_LSP_PTD                             "pll_lsp_ptd" //pll lsp postdiv
#define PLL_LSP_PTD_FIX_FACTOR                  "pll_lsp_ptd_fix_factor"
#define PLL_COREIP_PTD                          "pll_coreip_ptd" //pll coreip postdiv
#define PLL_COREIP                              "pll_coreip"
#define PLL_COREIP_PTD_FIX_FACTOR               "pll_coreip_ptd_fix_factor"
#define CLK_2400_COREIP_SEL                     "clk_2400_coreip_sel"
#define CLK_2400_COREIP_SEL_FIX_FACTOR_6_1      "clk_2400_coreip_sel_fix_factor_6_1"
#define CLK_2400_COREIP_SEL_FIX_FACTOR_12_1     "clk_2400_coreip_sel_fix_factor_12_1"
#define CLK_2400_COREIP_SEL_FIX_FACTOR_3_1      "clk_2400_coreip_sel_fix_factor_3_1"
#define CLK_2400_COREIP_SEL_FIX_FACTOR_4_1      "clk_2400_coreip_sel_fix_factor_4_1"
#define MAIN_CLK_FIX_FACTOR_20_1                "main_clk_fix_factor_20_1"
#define CLK_800_CV_CORE_SEL                     "clk_800_cv_core_sel"
#define CLK_800_CODEC_AXI_SEL                   "clk_800_codec_axi_sel"
#define CLK_800_CODEC_AXI_COMMON_SEL            "clk_800_codec_axi_common_sel"
#define CLK_800_CV_AXI_SEL                      "clk_800_cv_axi_sel"
#define CLK_800_SYSBUS_COMMON_SEL               "clk_800_sysbus_common_sel"
#define CLK_400_SYSBUS_COMMON_SEL               "clk_400_sysbus_common_sel"
#define LB_CV_CORE_CLK_GATE_EN                  "lb_cv_core_clk_gate_en"
#define LB_CV_AXIM0_CLK_GATE_EN                 "lb_cv_axim0_clk_gate_en"
#define LB_CV_AXIM1_CLK_GATE_EN                 "lb_cv_axim1_clk_gate_en"
#define LB_CV_AXIS_CLK_GATE_EN                  "lb_cv_axis_clk_gate_en"
#define LB_CODEC_AXI_GACLK_GATE_EN              "lb_codec_axi_gaclk_gate_en"
#define LB_CODEC_APB_GPCLK_GATE_EN              "lb_codec_apb_gpclk_gate_en"
#define LB_MATRIX_SYSCTRL_APB_S_GPCLK_GATE_EN   "lb_matrix_sysctrl_apb_s_gpclk_gate_en"
#define CLK_800_NET_SEL                         "clk_800_net_sel"
#define CLK_800_DSPCORE_SEL                     "clk_800_dspcore_sel"
#define CLK_200_NET_NOC_SEL                     "clk_200_net_noc_sel"
#define LB_NET_AXI_GACLK_GATE_EN                "lb_net_axi_gaclk_gate_en"
#define LB_NET_DSPCORE_AXI_GACLK_GATE_EN        "lb_net_dspcore_axi_gaclk_gate_en"
#define LB_NET_NOC_AXIM_GACLK_GATE_EN           "lb_net_noc_axim_gaclk_gate_en"
#define LB_NET_NOC_AXIS_GACLK_GATE_EN           "lb_net_noc_axis_gaclk_gate_en"
#define LB_ISP_AXI_GACLK_GATE_EN                "lb_isp_axi_gaclk_gate_en"
#define LB_ISP_AHB_GHCLK_GATE_EN                "lb_isp_ahb_ghclk_gate_en"
#define LB_MIPI0_PHY_CFG_CLK_GATE_EN            "lb_mipi0_phy_cfg_clk_gate_en"
#define LB_MIPI1_PHY_CFG_CLK_GATE_EN            "lb_mipi1_phy_cfg_clk_gate_en"
#define LB_MIPI2_PHY_CFG_CLK_GATE_EN            "lb_mipi2_phy_cfg_clk_gate_en"
#define LB_MIPI3_PHY_CFG_CLK_GATE_EN            "lb_mipi3_phy_cfg_clk_gate_en"
#define LB_MIPI0_APB_CFG_GPCLK_GATE_EN          "lb_mipi0_apb_cfg_gpclk_gate_en"
#define LB_MIPI1_APB_CFG_GPCLK_GATE_EN          "lb_mipi1_apb_cfg_gpclk_gate_en"
#define LB_MIPI2_APB_CFG_GPCLK_GATE_EN          "lb_mipi2_apb_cfg_gpclk_gate_en"
#define LB_MIPI3_APB_CFG_GPCLK_GATE_EN          "lb_mipi3_apb_cfg_gpclk_gate_en"
#define LB_VSP_DISP_CLK_GATE_EN                 "lb_vsp_disp_clk_gate_en"
#define LB_VSP_AXI_GACLK_GATE_EN                "lb_vsp_axi_gaclk_gate_en"
#define LB_VSP_AHB_GHCLK_GATE_EN                "lb_vsp_ahb_ghclk_gate_en"
#define LB_MATRIX_CORENOC_VSP_NIC3X1_400_EN     "lb_matrix_corenoc_vsp_nic3x1_400_en"
#define CLK_800_GPU_SEL                         "clk_800_gpu_sel"
#define LB_GPU_APB_S_GPCLK_GATE_EN              "lb_gpu_apb_s_gpclk_gate_en"
#define CLK_GPU_CPUNOC_NIC2X1_SEL               "clk_gpu_cpunoc_nic2x1_sel"
#define LB_GPU_CPUNOC_NIC2X1_AXI_GACLK_GATE_EN  "lb_gpu_cpunoc_nic2x1_axi_gaclk_gate_en"
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
#define LSP0_I2C0_W_GCLK_GATE_EN                "lsp0_i2c0_w_gclk_gate_en"
#define LSP0_I2C1_W_GCLK_GATE_EN                "lsp0_i2c1_w_gclk_gate_en"
#define LSP0_I2C2_W_GCLK_GATE_EN                "lsp0_i2c2_w_gclk_gate_en"
#define LSP0_I2C0_P_GCLK_GATE_EN                "lsp0_i2c0_p_gclk_gate_en"
#define LSP0_I2C1_P_GCLK_GATE_EN                "lsp0_i2c1_p_gclk_gate_en"
#define LSP0_I2C2_P_GCLK_GATE_EN                "lsp0_i2c2_p_gclk_gate_en"
#define LSP1_I2C_WCLK_FIX_FACTOR_2_1            "lsp1_i2c_wclk_fix_factor_2_1"
#define LSP1_I2C_MUX                            "lsp1_i2c_mux"
#define LSP1_I2C3_W_GCLK_GATE_EN                "lsp1_i2c3_w_gclk_gate_en"
#define LSP1_I2C4_W_GCLK_GATE_EN                "lsp1_i2c4_w_gclk_gate_en"
#define LSP1_I2C5_W_GCLK_GATE_EN                "lsp1_i2c5_w_gclk_gate_en"
#define LSP1_I2C3_P_GCLK_GATE_EN                "lsp1_i2c3_p_gclk_gate_en"
#define LSP1_I2C4_P_GCLK_GATE_EN                "lsp1_i2c4_p_gclk_gate_en"
#define LSP1_I2C5_P_GCLK_GATE_EN                "lsp1_i2c5_p_gclk_gate_en"
#define LSP0_TIMER_DIVIDER                      "lsp0_timer_divider"
#define LSP0_TIMER0_WCLK_GATE_EN                "lsp0_timer0_wclk_gate_en"
#define LSP0_TIMER1_WCLK_GATE_EN                "lsp0_timer1_wclk_gate_en"
#define LSP0_TIMER0_PCLK_GATE_EN                "lsp0_timer0_pclk_gate_en"
#define LSP0_TIMER1_PCLK_GATE_EN                "lsp0_timer1_pclk_gate_en"
#define LSP1_TIMER_DIVIDER                      "lsp1_timer_divider"
#define LSP1_TIMER2_WCLK_GATE_EN                "lsp1_timer2_wclk_gate_en"
#define LSP1_TIMER3_WCLK_GATE_EN                "lsp1_timer3_wclk_gate_en"
#define LSP1_TIMER2_PCLK_GATE_EN                "lsp1_timer2_pclk_gate_en"
#define LSP1_TIMER1_PCLK_GATE_EN                "lsp1_timer1_pclk_gate_en"
#define LSP0_SPI0_DIVIDER                       "lsp0_spi0_divider"
#define LSP0_SPI0_WCLK_GATE_EN                  "lsp0_spi0_wclk_gate_en"
#define LSP0_SPI0_PCLK_GATE_EN                  "lsp0_spi0_pclk_gate_en"
#define LSP1_SPI1_DIVIDER                       "lsp1_spi1_divider"
#define LSP1_SPI1_WCLK_GATE_EN                  "lsp1_spi1_wclk_gate_en"
#define LSP1_SPI1_PCLK_GATE_EN                  "lsp1_spi1_pclk_gate_en"
#define LSP0_WDT0_DIVIDER                       "lsp0_wdt0_divider"
#define LSP0_WDT1_DIVIDER                       "lsp0_wdt1_divider"
#define LSP0_WDT0_W_GCLK_GATE_EN                "lsp0_wdt0_w_gclk_gate_en"
#define LSP0_WDT1_W_GCLK_GATE_EN                "lsp0_wdt1_w_gclk_gate_en"
#define LSP0_WDT0_P_GCLK_GATE_EN                "lsp0_wdt0_p_gclk_gate_en"
#define LSP0_WDT1_P_GCLK_GATE_EN                "lsp0_wdt1_p_gclk_gate_en"
#define LSP1_WDT2_DIVIDER                       "lsp1_wdt2_divider"
#define LSP1_WDT3_DIVIDER                       "lsp1_wdt3_divider"
#define LSP0_GPIO_DIVIDER                       "lsp0_gpio_divider"
#define LSP1_GPIO_DIVIDER                       "lsp1_gpio_divider"
#define LSP1_WDT2_W_GCLK_GATE_EN                "lsp1_wdt2_w_gclk_gate_en"
#define LSP1_WDT3_W_GCLK_GATE_EN                "lsp1_wdt3_w_gclk_gate_en"
#define LSP1_WDT2_P_GCLK_GATE_EN                "lsp1_wdt2_p_gclk_gate_en"
#define LSP1_WDT3_P_GCLK_GATE_EN                "lsp1_wdt3_p_gclk_gate_en"
#define LB_LSP0_I2SM_GCLK_GATE_EN               "lb_lsp0_i2sm_gclk_gate_en"
#define LSP0_I2SM_DIVIDER                       "lsp0_i2sm_divider"
#define I2SM_CLKOUT_GATE_EN                     "i2sm_clkout_gate_en"
#define LSP0_I2SM_P_GCLK_GATE_EN                "lsp0_i2sm_p_gclk_gate_en"
#define LSP1_I2SS_P_GCLK_GATE_EN                "lsp1_i2ss_p_gclk_gate_en"
#define LSP0_GPIO0_P_GCLK_GATE_EN               "lsp0_gpio0_p_gclk_gate_en"
#define LSP1_GPIO1_P_GCLK_GATE_EN               "lsp1_gpio1_p_gclk_gate_en"

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
    const char *pll_name;
};
#define to_bst_pll(_hw)	container_of(_hw, struct bst_pll, hw)


#endif	/* __BST_CLK_H */

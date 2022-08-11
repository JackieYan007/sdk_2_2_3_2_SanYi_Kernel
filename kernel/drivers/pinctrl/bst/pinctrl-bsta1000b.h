#ifndef __PINCTRL_BST_H__
#define __PINCTRL_BST_H__


#define TOP_MUX_GROUP(pname, mux0, mux1, mux2, mux3, bst_pin_mux_num, pmm, poff,cfg,cfgoff,ie,ieoff)        \
	{							\
		.name = #pname,			\
        .pins = pname##_pins,   \
        .npins = ARRAY_SIZE(pname##_pins),  \
		.funcs = (int[]){			\
			BST_FUNC_##mux0,			\
			BST_FUNC_##mux1,			\
			BST_FUNC_##mux2,			\
			BST_FUNC_##mux3,			\
		},	\
        .nfuncs = bst_pin_mux_num, \
		.reg_index = BST_PINC_INDEX_TOP, \
		.pmm_reg = pmm,\
		.pmm_offset = poff,\
		.io_cfg_reg= cfg,\
		.io_cfg_offset= cfgoff,\
        .io_ie_reg= ie,\
		.io_ie_offset= ieoff,\
}

#define AON_MUX_GROUP(pname, mux0, mux1, mux2, mux3, bst_pin_mux_num, pmm, poff,cfg,cfgoff,ie,ieoff)        \
    {                           \
        .name = #pname,         \
        .pins = pname##_pins,   \
        .npins = ARRAY_SIZE(pname##_pins),  \
		.funcs = (int[]){			\
			BST_FUNC_##mux0,			\
			BST_FUNC_##mux1,			\
			BST_FUNC_##mux2,			\
			BST_FUNC_##mux3,			\
		},	\
        .nfuncs = bst_pin_mux_num, \
		.reg_index = BST_PINC_INDEX_AON, \
		.pmm_reg = pmm,\
		.pmm_offset = poff,\
		.io_cfg_reg= cfg,\
		.io_cfg_offset= cfgoff,\
        .io_ie_reg= ie,\
		.io_ie_offset= ieoff,\
}

#define BST_CFG_SMT_BIT 		(1 << 6)
#define BST_CFG_PULL_MASK 		(0x3 << 4)
#define BST_CFG_PULL_UP_BITS 	(0x1 << 4)
#define BST_CFG_PULL_DOWN_BITS 	(0x2 << 4)
#define BST_CFG_PULL_NULL 		(0)

#define BST_CFG_DRIVE_STRENGTH_MASK  (0xf)

enum {
	BST_PINC_INDEX_TOP,
	BST_PINC_INDEX_AON,
	BST_PINC_INDEX_MAX,
};

struct bst_pinctrl_function {
	const char *name;
	const char * const *groups;
	unsigned ngroups;
};

struct bst_pinmux_pingroup {
	const char *name;
	const unsigned *pins;
	unsigned int npins;
	int *funcs;
	unsigned int nfuncs;
	unsigned int reg_index;
	
	u32 pmm_reg;
	u32 pmm_offset;
	
	u32 io_ie_reg;
	u32 io_ie_offset;
	
	u32 io_cfg_reg;
	u32 io_cfg_offset;
};

struct bst_pinctrl_part_data {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	const struct bst_pinmux_pingroup *groups;
	unsigned int ngroups;
	const struct bst_pinctrl_function *functions;
	unsigned int nfunctions;
};

struct bst_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrl;
	struct pinctrl_desc desc;
	raw_spinlock_t lock;
	
	const struct bst_pinctrl_part_data *part;
	void __iomem *aon_regs;
    void __iomem *top_regs;
};

enum bst_a1000_func_e {
	BST_FUNC_gpio,
    BST_FUNC_spi0,
	BST_FUNC_spi1,
	BST_FUNC_spi2,   //add
	BST_FUNC_spi3,   //add
	BST_FUNC_spi0_s, //add
	BST_FUNC_spi1_s, //add
	BST_FUNC_bist,
	BST_FUNC_boot,
	BST_FUNC_can0,
    BST_FUNC_can1,
    BST_FUNC_can2, //add
	BST_FUNC_i2c0,
	BST_FUNC_i2c1,
	BST_FUNC_i2c2,
	BST_FUNC_i2c3,
	BST_FUNC_i2c4,
	BST_FUNC_i2c5,
	BST_FUNC_i2c_sm0, //add
	BST_FUNC_i2c_sm1, //add
	BST_FUNC_i2c_sm2, //add
	BST_FUNC_i2s0,
	BST_FUNC_i2s1,
	BST_FUNC_isp,
	BST_FUNC_jtag,
	BST_FUNC_mem,
	BST_FUNC_otp,
	BST_FUNC_pcie0,
	BST_FUNC_pcie1,
	BST_FUNC_ptp,
	BST_FUNC_qspi0,  //30
	BST_FUNC_qspi1,
	BST_FUNC_rgmii0,
	BST_FUNC_rgmii1,
	BST_FUNC_err_rpt_l0,
    BST_FUNC_err_rpt_l1,
	BST_FUNC_strap,
	BST_FUNC_ts,
	BST_FUNC_uart0,
	BST_FUNC_uart1,
	BST_FUNC_uart2,
	BST_FUNC_uart3,
	BST_FUNC_uart_dbg, //add
	BST_FUNC_dsp_jtag,  //43
    BST_FUNC_sdemmc0,
    BST_FUNC_sdemmc1,
    BST_FUNC_debug,
    BST_FUNC_pwm,
    BST_FUNC_ist, //add
    BST_FUNC_vout, //add
    BST_FUNC_vin, //add
    BST_FUNC_xtal_in, //add
	BST_FUNC_null //52
};

#endif

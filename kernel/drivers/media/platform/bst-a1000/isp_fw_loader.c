// SPDX-License-Identifier: GPL-2.0

/*
 * ISP firmware loader for BST
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <linux/coreip/proto_api_common.h>

#include "isp_core.h"
#include "isp_fw_loader.h"

static const uint32_t ISP_SRAM_RESET = 0x33002004;
static const uint32_t REG_ISP_MAILBOX_IN = 0x52030090; // to store INIT physical
						       // address
static const uint32_t REG_ISP_MAILBOX_OUT = 0x52030094; // to store IPC message
							// physical address
static const uint32_t REG_CTRL1_ADDR = 0x52030000;
static const uint32_t REG_CTRL2_ADDR = 0x52030004;
static const uint32_t REG_RESET_ADDR = 0x52030004;
static const uint32_t REG_STATUS_ADDR = 0x52034280;
static const uint32_t ISP_FW_IMAGE_BASE = 0x52040000;
static const uint32_t IPC_REGISTER_ADDR = 0x8FF00000;

#define DSP_MEM_FW		      (0x98000000)
#define DSP_REG_BASE		      (0x51030000)
#define DSP_REG_OFFSET_SYS_CTRL	      (0x00)
#define DSP_REG_OFFSET_DSP0_RESET_VEC (0x04)
#define DSP_REG_OFFSET_DSP1_RESET_VEC (0x08)
#define DSP_REG_OFFSET_DSP2_RESET_VEC (0x0C)
#define DSP_REG_OFFSET_DSP3_RESET_VEC (0x10)
#define DSP_REG_OFFSET_DSP0_OUT_MAIL  (0x40)
#define DSP_REG_OFFSET_DSP1_OUT_MAIL  (0x44)
#define DSP_REG_OFFSET_DSP2_OUT_MAIL  (0x48)
#define DSP_REG_OFFSET_DSP3_OUT_MAIL  (0x4C)
#define DSP_REG_SIZE		      (DSP_REG_OFFSET_DSP3_OUT_MAIL + 4)
#define DSP1_RESET_MASK		      (0xDDDFFFFF)
#define DSP1_START_MASK		      (0x22000000)
#define DSP1_CLK_ENABLE_MASK	      (0x02000000)

#define ISP_CTRL_R_CORENOC_PARITY_ENABLE (0x33000084)
#define CV_PARITY_EN			 (0x2)
#define ISP_PARITY_EN			 (0x8)

static void __iomem *dsp_fw_base;
static void __iomem *dsp_reg_base;

static int parse_elf(const u8 *data, int size, struct a1000_isp_device *isp);

/* ISP cowork with DSP 1 */
int bst_load_dsp_fw(const char *fw_name, struct a1000_isp_device *isp)
{
	int ret;
	uint32_t reg;

	const struct firmware *fw;
	u32 val;

	void __iomem *crm_base = NULL;
	void __iomem *corenoc_parity_enable = NULL;
	uint32_t cv_parity_en;

	crm_base = ioremap(0x33002000, 0x200); // SYS_CRM base address
	// set up CRM register for CV block, or else OS hangs after vector reset
	reg = readl_relaxed(crm_base +
			    0x4); // TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
	reg |= (1UL << 7); // cv block soft reset
	writel_relaxed(reg, crm_base + 0x4);

	// set CV freq to 800MHz
	reg = readl_relaxed(crm_base + 0x15c); // CLKMUX_SEL2
	// CLK_800_CV_CORE_CLK_SEL, bit[3:2] = 01: 800MHz
	reg &= ~(1UL << 3);
	reg |= (1UL << 2);
	writel_relaxed(reg, crm_base + 0x15c);

	corenoc_parity_enable =
		devm_ioremap(isp->dev, ISP_CTRL_R_CORENOC_PARITY_ENABLE, 0x4);
	if (corenoc_parity_enable == NULL) {
		dev_err(isp->dev,
			"IOREMAP ISP_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
			ISP_CTRL_R_CORENOC_PARITY_ENABLE);
		return -EIO;
	}
	cv_parity_en = readl_relaxed(corenoc_parity_enable) & CV_PARITY_EN;
	devm_iounmap(isp->dev, corenoc_parity_enable);

	dsp_reg_base = devm_ioremap(isp->dev, DSP_REG_BASE, DSP_REG_SIZE);
	if (IS_ERR(dsp_reg_base)) {
		dev_err(isp->dev, "Failed to map DSP regs\n");
		return PTR_ERR(dsp_reg_base);
	}

	reg = readl_relaxed(dsp_reg_base + 0x50);
	if (cv_parity_en && ((reg & 0x3) != 0x3)) {
		reg |= 0x3; // cv block ecc and parity enable
		writel_relaxed(reg, dsp_reg_base + 0x50);
	}

	ret = request_firmware(&fw, fw_name, isp->dev);
	if (ret) {
		dev_err(isp->dev, "Failed to request firmware %s\n", fw_name);
		return ret;
	}
	// dev_info(isp->dev, "DSP firmware size: %ld\n", fw->size);

	dsp_fw_base = devm_ioremap(isp->dev, DSP_MEM_FW, fw->size);
	if (IS_ERR(dsp_fw_base)) {
		dev_err(isp->dev, "Failed to map DSP firmware mem\n");
		return PTR_ERR(dsp_fw_base);
	}

	/* Copy firmware to memory */
	memcpy_toio(dsp_fw_base, fw->data, fw->size);
	release_firmware(fw);

	val = readl_relaxed(dsp_reg_base + DSP_REG_OFFSET_DSP1_RESET_VEC);
	// dev_info(isp->dev, "%s: RESET_VEC: 0x%08X\n", __func__, val);
	if (val == DSP_MEM_FW) {
		dev_info(isp->dev,
			 "Firmware has been loaded, skip, RESET_VEC: 0x%08X\n",
			 val);
		return 0;
	}
	// set dsp memory
	writel_relaxed(DSP_MEM_FW,
		       (dsp_reg_base + DSP_REG_OFFSET_DSP1_RESET_VEC));

	val = readl_relaxed(dsp_reg_base + DSP_REG_OFFSET_SYS_CTRL);
	// dev_info(isp->dev, "read at first DSP sys ctrl: 0x%08X\n", val);
	val |= (1 << 29); // DSP1_RESET_MASK;
	val |= DSP1_CLK_ENABLE_MASK; // modify

	// dev_info(isp->dev, "write DSP reset ctrl: 0x%08X\n", val);

	/* Reset DSP 1 */
	writel_relaxed(val, (dsp_reg_base + DSP_REG_OFFSET_SYS_CTRL));
	val = readl_relaxed(dsp_reg_base + DSP_REG_OFFSET_SYS_CTRL);
	// dev_info(isp->dev, "read back DSP sys ctrl: 0x%08X\n", val);

	return 0;
}

void bst_start_dsp_fw(struct a1000_isp_device *isp)
{
	u32 val;

	/* Write out mailbox reg for debug */
	writel_relaxed(0xBBBBBBBB,
		       (dsp_reg_base + DSP_REG_OFFSET_DSP1_OUT_MAIL));

	val = readl_relaxed(dsp_reg_base + DSP_REG_OFFSET_SYS_CTRL);
	dev_info(isp->dev, "DSP sys ctrl: 0x%08X\n", val);
	val |= DSP1_START_MASK;
	dev_info(isp->dev, "DSP start ctrl: 0x%08X\n", val);

	/* Boot DSP 1 */
	writel_relaxed(val, (dsp_reg_base + DSP_REG_OFFSET_SYS_CTRL));
}

#define ISP_CORE_TOP_CTRL2C 0x5203002c
#define ISP_CORE_TOP_CTRL30 0x52030030

static int enable_isp_ecc_and_parity(void)
{
	uint32_t *isp_ctrl30_reg;
	uint32_t status;

	isp_ctrl30_reg = (uint32_t *)ioremap(ISP_CORE_TOP_CTRL30, 0x4);
	if (isp_ctrl30_reg == NULL) {
		pr_err("ioremap(A1000B ISP_CORE_TOP_CTRL30 _REG failed\n");
		return -1;
	}

	status = *isp_ctrl30_reg;
	status = 0xffffffff;
	*isp_ctrl30_reg = status;
	status = *isp_ctrl30_reg;

	return 0;
}

int bst_load_isp_fw(const char *fw_name, const char *slab_name,
		    struct a1000_isp_device *isp)
{
	int ret;
	const struct firmware *firmware_p;
	uint32_t *sram_reset;

	sram_reset = (uint32_t *)ioremap(ISP_SRAM_RESET, 4);
	if (sram_reset == NULL) {
		dev_err(isp->dev, "IOREMAP reg_ctrl10x%p FAILED\n", sram_reset);
		return -1;
	}
	*sram_reset = 0xffffffff;

	ret = request_firmware(&firmware_p, fw_name, isp->dev);
	if (ret) {
		dev_info(isp->dev, "there is no firmware could be used\n");
		return -1;
	}
	ret = parse_elf(firmware_p->data, firmware_p->size, isp);
	ret = request_firmware(&firmware_p, slab_name, isp->dev);
	if (ret) {
		dev_info(isp->dev, "there is no slab could be used\n");
		return -1;
	}
	memcpy(isp->slab_vaddr, firmware_p->data, firmware_p->size);

	return 0;
}

int isp_internal_trigger(int taget_freq, int fysnc_in, int fsync_out)
{
	int scr_period_value;
	uint32_t scr_en_value;
	uint32_t scr_period;
	uint32_t pulse_width_value;
	uint32_t pulse_int_value;
	uint32_t fsync_sel_en_value;
	void __iomem *scr_en = NULL;
	void __iomem *scr_period_en = NULL;
	void __iomem *pulse_width = NULL;
	void __iomem *pulse_int = NULL;
	void __iomem *fsync_sel = NULL;
	//set scr_en 0X100
	scr_en = ioremap(OUT_SRC_EN, 0x4); //modify
	if (scr_en == NULL) {
		pr_err(
			"IOREMAP scr_en 0x5203015c FAILED\n");
		return -EIO;
	}

	scr_en_value = readl_relaxed(scr_en);
	writel_relaxed(scr_en_value | 0x03, scr_en);
	iounmap(scr_en);
	//set pulse_low  int_sel
	pulse_int = ioremap(INT_PULSE_LOW, 0x4); //modify
	if (pulse_int == NULL) {
		pr_err(
			"IOREMAP pulse_int 0x5203015c FAILED\n");
		return -EIO;
	}

	pulse_int_value = readl_relaxed(pulse_int);
	//set scr_period
	if (taget_freq > 20 && taget_freq <= 30) {
		scr_period = 10000000000 / taget_freq / 25;
		scr_period_en = ioremap(SRC2_PERIIOD, 0x4); //modify
		if (scr_period_en == NULL) {
			pr_err(
				"IOREMAP scr_period_en 0x5203015c FAILED\n");
			return -EIO;
		}
		scr_period_value = readl_relaxed(scr_period_en);
		writel_relaxed(scr_period_value | 0xcb7355, scr_period_en);
		iounmap(scr_period_en);
		//set int sel
		writel_relaxed(pulse_int_value | (1 << (fsync_out * 3 + 8)), pulse_int);
	} else {
		scr_period = 10000000000 / taget_freq / 25;
		scr_period_en = ioremap(SRC1_PERIIOD, 0x4); //modify
		if (scr_period_en == NULL) {
			pr_err(
				"IOREMAP scr_period_en 0x5203015c FAILED\n");
			return -EIO;
		}
		scr_period_value = readl_relaxed(scr_period_en);
		writel_relaxed(scr_period, scr_period_en);
		iounmap(scr_period_en);
		//set int_sel import
		pulse_int_value = readl_relaxed(pulse_int);
		writel_relaxed(pulse_int_value & ~(1<<fsync_out), pulse_int);
	}
	iounmap(pulse_int);
	//set pulse_width
	pulse_width = ioremap(PULSE_WIDTH + 4 * fsync_out, 0x4); //modify
	if (pulse_width == NULL) {
		pr_err(
			"IOREMAP pulse_width 0x5203015c FAILED\n");
		return -EIO;
	}
	pulse_width_value = readl_relaxed(pulse_width);
	writel_relaxed(pulse_width_value | 0x3d0900, pulse_width);
	iounmap(pulse_width);
	//set fsync_sel fsync_en low enable output
	fsync_sel = ioremap(FSYNC_SEL, 0x4); //modify
	if (fsync_sel == NULL) {
		pr_err(
			"IOREMAP fsync_sel_en 0x5203015c FAILED\n");
		return -EIO;
	}
	fsync_sel_en_value = readl_relaxed(fsync_sel);
	writel_relaxed(fsync_sel_en_value | 0x0, fsync_sel);
	iounmap(fsync_sel);
	return 0;
}

int isp_external_trigger(int external_freq, int target_freq, int fsync_in, int fsync_out)
{
	uint32_t out_src_en_value;
	uint32_t out_scr_period_en_value;
	uint32_t decrease_period_en_value;
	uint32_t fsync_sel_en_value;
	uint32_t out_scr_period;
	uint32_t decrease_period;
	void __iomem *out_src_en = NULL;
	void __iomem *pulse_width_en = NULL;
	void __iomem *out_scr_period_en = NULL;
	void __iomem *decrease_period_en = NULL;
	void __iomem *pulse_int_en = NULL;
	void __iomem *fsync_sel_en = NULL;

	//set out_src_en
	out_src_en = ioremap(OUT_SRC_EN, 0x4);
	if (out_src_en == NULL) {
		pr_err(
			"IOREMAP out_src_en 0x5203015c FAILED\n");
		return -EIO;
	}
	//set out_scr_en
	out_src_en_value = readl_relaxed(out_src_en);
	writel_relaxed(out_src_en_value | (1 << (4+fsync_out)), out_src_en);
	//set pluse_outside_sel which fsync to receive
	out_src_en_value = readl_relaxed(out_src_en);
	writel_relaxed(out_src_en_value | (fsync_in << (16+3*fsync_out)), out_src_en);
	//inc_outside
	if (external_freq < target_freq) {
		out_scr_period = 10000000000 / target_freq / 25;
		out_src_en_value = readl_relaxed(out_src_en);
		//set inc_outside
		writel_relaxed(out_src_en_value | (1 << (8+fsync_out)), out_src_en);
		out_scr_period_en = ioremap(OUT_SRC_PERIOD+4*fsync_out, 0x4); //modify
		if (out_scr_period_en == NULL) {
			pr_err(
				"IOREMAP out_scr_period_en 0x5203015c FAILED\n");
			return -EIO;
		}
		out_scr_period_en_value = readl_relaxed(out_scr_period_en);
		writel_relaxed(out_scr_period, out_scr_period_en);
		iounmap(out_scr_period_en);
	} else if (external_freq > target_freq) {
		decrease_period = external_freq / target_freq;
		out_src_en_value = readl_relaxed(out_src_en);
		writel_relaxed(out_src_en_value & ~(1 << (fsync_out + 8)), out_src_en);
		if (fsync_out >= 0 && fsync_out <= 1) {
			decrease_period_en = ioremap(DECREASE1_PERIOD, 0x4);
			if (decrease_period_en == NULL) {
				pr_err(
					"IOREMAP decrease_period_en 0x5203015c FAILED\n");
				return -EIO;
			}
			decrease_period_en_value = readl_relaxed(decrease_period_en);
			writel_relaxed(decrease_period_en_value | decrease_period << 16 * fsync_out, decrease_period_en);
			iounmap(decrease_period_en);
		} else if (fsync_out > 1 && fsync_out <= 3) {
			decrease_period_en = ioremap(DECREASE2_PERIOD, 0x4);
			if (decrease_period_en == NULL) {
				pr_err(
					"IOREMAP decrease_period_en 0x5203015c FAILED\n");
				return -EIO;
			}
			decrease_period_en_value = readl_relaxed(decrease_period_en); //fail
			writel_relaxed(decrease_period_en_value | decrease_period << 16 * fsync_out, decrease_period_en);
			iounmap(decrease_period_en);
		}
	}
	iounmap(out_src_en);
	//set pulse_with
	pulse_width_en = ioremap(PULSE_WIDTH + 4 * fsync_out, 0x4);
	if (pulse_width_en == NULL) {
		pr_err(
			"IOREMAP pulse_width_en 0x5203015c FAILED\n");
		return -EIO;
	}
	writel_relaxed(0x3d0900, pulse_width_en);
	iounmap(pulse_width_en);
	//set pulse_LOW
	pulse_int_en = ioremap(INT_PULSE_LOW, 0x4); //modify
	if (pulse_int_en == NULL) {
		pr_err(
			"IOREMAP pulse_int_en 0x5203015c FAILED\n");
		return -EIO;
	}
	writel_relaxed(0x11a00, pulse_int_en);
	iounmap(pulse_int_en);
	//set fsync_sel
	fsync_sel_en = ioremap(FSYNC_SEL, 0x4); //modify
	if (fsync_sel_en == NULL) {
		pr_err(
			"IOREMAP fsync_sel_en 0x5203015c FAILED\n");
		return -EIO;
	}
	fsync_sel_en_value = readl_relaxed(fsync_sel_en);
	writel_relaxed(1 << fsync_in, fsync_sel_en);
	iounmap(fsync_sel_en);
	return 0;
}
void bst_start_isp_fw(struct a1000_isp_device *isp)
{
	uint32_t *reg_ctrl1;
	uint32_t *reg_reset;
	uint32_t *reg_vaddr;

	uint64_t reg_ctrl1_addr;
	uint64_t reg_reset_addr;

	void __iomem *corenoc_parity_enable = NULL;
	uint32_t isp_parity_en;

	reg_ctrl1_addr = REG_CTRL1_ADDR;
	reg_reset_addr = REG_RESET_ADDR;

	corenoc_parity_enable =
		devm_ioremap(isp->dev, ISP_CTRL_R_CORENOC_PARITY_ENABLE, 0x4);
	if (corenoc_parity_enable == NULL) {
		dev_err(isp->dev,
			"IOREMAP ISP_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
			ISP_CTRL_R_CORENOC_PARITY_ENABLE);
		return;
	}
	isp_parity_en = readl_relaxed(corenoc_parity_enable) & ISP_PARITY_EN;
	devm_iounmap(isp->dev, corenoc_parity_enable);

	if (isp_parity_en)
		enable_isp_ecc_and_parity();

	/* write INIT physical addr */
	reg_vaddr = (uint32_t *)ioremap(REG_ISP_MAILBOX_IN, 4);
	if (reg_vaddr == NULL) {
		dev_err(isp->dev, "IOREMAP REG_ISP_MAILBOX_IN 0x%x FAILED\n",
			REG_ISP_MAILBOX_IN);
		return;
	}
	*reg_vaddr = (isp->init_paddr) & LOW_32_BIT_MASK;

	/* write IPC Message Start Addr */
	reg_vaddr = (uint32_t *)ioremap(REG_ISP_MAILBOX_OUT, 4);
	if (reg_vaddr == NULL) {
		dev_err(isp->dev, "IOREMAP REG_ISP_MAILBOX_OUT 0x%x FAILED\n",
			REG_ISP_MAILBOX_OUT);
		return;
	}
	*reg_vaddr = IPC_REGISTER_ADDR;

	/* write ctrl1_reg */
	reg_ctrl1 = (uint32_t *)ioremap(reg_ctrl1_addr, 4);
	if (reg_ctrl1 == NULL) {
		dev_err(isp->dev, "IOREMAP reg_ctrl10x%llx FAILED\n",
			reg_ctrl1_addr);
		return;
	}
	*reg_ctrl1 = 0xffffffff;

	/* write reset_reg */
	reg_reset = (uint32_t *)ioremap(reg_reset_addr, 4);
	if (reg_reset == NULL)
		dev_err(isp->dev, "IOREMAP reg_reset 0x%llx FAILED\n",
			reg_reset_addr);
	else
		*reg_reset = 0xffffffff;
}

int bst_boot_isp_fw(const char *fw_name, const char *slab_name,
		    struct a1000_isp_device *isp)
{
	int ret;

	ret = bst_load_isp_fw(fw_name, slab_name, isp);
	if (ret)
		return ret;

	bst_start_isp_fw(isp);
	return 0;
}

static int parse_elf32(const u8 *data, int size, struct a1000_isp_device *isp)
{
	int i;
	u8 *dst_ptr;
	u8 *src_ptr;
	u64 phy_addr;

	Elf32_Ehdr *ehdr = (Elf32_Ehdr *)data;
	Elf32_Shdr *shdr = (Elf32_Shdr *)(data + ehdr->e_shoff);
	Elf32_Shdr *shstr =
		(Elf32_Shdr *)((char *)shdr +
			       ehdr->e_shstrndx * ehdr->e_shentsize);
	char *shstr_tbl = data + shstr->sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++) {
		Elf32_Shdr *sptr =
			(Elf32_Shdr *)((char *)shdr + i * ehdr->e_shentsize);

		// copy .text section to specified addr
		if (!strncmp((shstr_tbl + sptr->sh_name), TEXT_SECTION_NAME,
			     sizeof(TEXT_SECTION_NAME))) {
			src_ptr = data + sptr->sh_offset;
			phy_addr = ISP_FW_IMAGE_BASE;

			memcpy(&isp->fw_pack_version, src_ptr + ISP_PACK_OFFSET,
			       4);
			memcpy(&isp->fw_svn_version, src_ptr + ISP_SVN_OFFSET,
			       4);
			memcpy(&isp->fw_build_date,
			       src_ptr + ISP_BUILD_DATE_OFFSET, 4);
			dev_dbg(isp->dev,
				"fw pack version is 0x%04x,svn_version is 0x%x,build date is 0x%x\r\n",
				isp->fw_pack_version, isp->fw_svn_version,
				isp->fw_build_date);
			dst_ptr = (u8 *)ioremap(phy_addr, sptr->sh_size);
			if (dst_ptr == NULL) {
				dev_err(isp->dev, "IOREMAP 0x%llx FAILED\n",
					phy_addr);
			} else {
				memcpy(dst_ptr, src_ptr, sptr->sh_size);
			}
		}
	}
	return 0;
}

static int parse_elf(const u8 *data, int size, struct a1000_isp_device *isp)
{
	char *header;

	header = data;

	if (!((header[EI_MAG0] == 0x7f) && (header[EI_MAG1] == 'E') &&
	      (header[EI_MAG2] == 'L') && (header[EI_MAG3] == 'F'))) {
		dev_info(isp->dev, "not elf file\n");
		return -1;
	}

	if (header[EI_CLASS] == ELFCLASS32) {
		parse_elf32(data, size, isp);
	} else {
		dev_err(isp->dev, "ERROR: wrong elf class = %d\n",
			header[EI_CLASS]);
		return -2;
	}

	return 0;
}

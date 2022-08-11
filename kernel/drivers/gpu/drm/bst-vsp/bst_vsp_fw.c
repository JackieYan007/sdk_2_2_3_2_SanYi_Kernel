// SPDX-License-Identifier: GPL-2.0

/*
 * VSP Firmware definitions for BST
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

#include <linux/delay.h>
#include <linux/firmware.h>

#include <linux/coreip/bst_vsp_msg.h>
#include <linux/coreip/proto_api_common.h>
#include <linux/ipc_interface.h>

#include "bst_ipc_drv.h"

extern struct vsp_device *pvsp;

#define CORE_CODE_BIN "vsp/core_code.exe.text.bin"
#define CORE_MD_BIN   "vsp/core_md.exe.text.bin"
#define CORE_ME_BIN   "vsp/core_me.exe.text.bin"
#define VSP_SLAB_BIN  "vsp/vsp_slab.bin"
#define VSP_PARITY_EN       0x4
#define VSP_PARITY_CHECK_EN 0xffff
#define SYS_PARITY_REG      0x33000084
#define VSP_CTRL24          0x53090024


int vsp_fw_mem_init(struct vsp_device *pvsp)
{
	int ret = 0;
	uint32_t init_phy_base_low;
	uint32_t init_phy_base_high;
	uint64_t init_phy_base;
	uint64_t cmdp_phy_base;
	uint64_t slab_phy_base;
	uint64_t slab_end_phy_addr;
	uint64_t part_start_addr;
	int total_alloc_size = 0;
	int initp_size;
	int cmdp_size;
	// int slabp_size;
	uint32_t traceMask = 0xffffffff;
	uint8_t *pMemBase;
	dma_addr_t base_dma_addr;
	tSoneInit *pInit;
	tSoneCmdp *pCmdp;
	// void *pFbuf;
	void *pSlab;
	uint8_t *pMsgPayloadBase;

	// phy addr
	init_phy_base = pvsp->intp_phy_addr;
	init_phy_base_low = (init_phy_base & 0xFFFFFFFF);
	init_phy_base_high = (init_phy_base & 0xFFFFFFFF00000000) >> 32;
	part_start_addr = init_phy_base;

	initp_size = (sizeof(tSoneInit) + SONE_PART_ALIGN_MASK) &
		     (~SONE_PART_ALIGN_MASK);
	pr_info("init start = 0x%llx, initp_size = 0x%lx, align size = 0x%x\n",
		init_phy_base, sizeof(tSoneInit), initp_size);

	part_start_addr += initp_size;
	cmdp_phy_base = part_start_addr;
	pvsp->parti.cmdp_pyaddr = cmdp_phy_base;
	cmdp_size = (sizeof(tSoneCmdp) + SONE_PART_ALIGN_MASK) &
		    (~SONE_PART_ALIGN_MASK);
	pr_info("cmdp start = 0x%llx, cmdp_size = 0x%lx, align size = 0x%x\n",
		cmdp_phy_base, sizeof(tSoneCmdp), cmdp_size);

	part_start_addr += cmdp_size;
	// for buf, 1MB alignment
	part_start_addr = (part_start_addr + SONE_BUF_ADDR_ALIGN_MASK) &
			  (~SONE_BUF_ADDR_ALIGN_MASK);
	slab_phy_base = part_start_addr;
	slab_end_phy_addr = slab_phy_base + SONE_MEDIA_SLAB_BUFSIZE;
	pr_info("slab start = 0x%llx, end = 0x%llx, slab_size = 0x%x\n",
		slab_phy_base, slab_end_phy_addr, SONE_MEDIA_SLAB_BUFSIZE);

	total_alloc_size = (pvsp->firmware_c0.size + pvsp->firmware_c1.size +
			    pvsp->firmware_c2.size);
	total_alloc_size += (slab_end_phy_addr - init_phy_base);
	total_alloc_size = (total_alloc_size + SONE_BUF_ADDR_ALIGN_MASK) &
			   (~SONE_BUF_ADDR_ALIGN_MASK);
	pr_info("total_alloc_size = 0x%x\n", total_alloc_size);
	pMemBase = dma_alloc_coherent(pvsp->dev, total_alloc_size,
				      &base_dma_addr, GFP_KERNEL);
	if (pMemBase == NULL) {
		pr_info("dma_alloc_coherent partition_total_size = 0x%x failed\n",
			total_alloc_size);
		return -1;
	}
	if ((unsigned long)base_dma_addr !=
	    (unsigned long)pvsp->firmware_c0.phy_addr) {
		dma_free_coherent(pvsp->dev, total_alloc_size, pMemBase,
				  base_dma_addr);
		pr_err("dma_alloc_coherent dismatch, base_dma_addr = 0x%lx, phy_addr = 0x%lx\n",
		       (unsigned long)base_dma_addr,
		       (unsigned long)pvsp->firmware_c0.phy_addr);
		return -2;
	}

	// setup firmware image vaddr
	pvsp->firmware_c0.base = (void *)pMemBase;
	pvsp->firmware_c1.base = (uint8_t *)pMemBase + pvsp->firmware_c0.size;
	pvsp->firmware_c2.base =
		(uint8_t *)(pvsp->firmware_c1.base) + pvsp->firmware_c1.size;
	pr_info("c0.base  = %p, c1.base  = %p, c2.base  = %p\n",
		pvsp->firmware_c0.base, pvsp->firmware_c1.base,
		pvsp->firmware_c2.base);

	// setup memeory partition vaddr
	pvsp->vreg.base =
		(uint8_t *)(pvsp->firmware_c2.base) + pvsp->firmware_c2.size;
	pInit = (tSoneInit *)(pvsp->vreg.base);
	pCmdp = (tSoneCmdp *)((uint8_t *)(pvsp->vreg.base) +
			      (cmdp_phy_base - init_phy_base));
	pSlab = ((uint8_t *)(pvsp->vreg.base) +
		 (slab_phy_base - init_phy_base));

	// pr_info("vreg_base = 0x%lx, pInit = 0x%lx, pCmdp = 0x%lx, pSlab =
	// 0x%lx\n",	pvsp->vreg.base, pInit, pCmdp, pSlab);

	pvsp->parti.init_vaddr = pInit;
	pvsp->parti.cmdp_vaddr = pCmdp;
	pvsp->parti.slab_vaddr = pSlab;

	setup_init_parti(pInit, init_phy_base_low, init_phy_base_high, "A1000",
			 "Li41", traceMask, "VSP RISCV", 'V');
	setup_cmdp_parti(pInit, pCmdp, cmdp_phy_base);
	setup_slab_parti(pInit, pSlab, slab_phy_base);

	pMsgPayloadBase = (uint8_t *)pCmdp->ch[DRV_CH_INDEX].cqueue.payload_cmd;
	pMsgPayloadBase = (uint8_t *)(((uint64_t)pMsgPayloadBase +
				       VSP_PAYLOAD_ALIGN_MASK) &
				      (~VSP_PAYLOAD_ALIGN_MASK));

	pvsp->vout_payload_vaddr = pMsgPayloadBase;
	pvsp->gwarp_payload_vaddr =
		pvsp->vout_payload_vaddr + VOUT_PAYLOAD_TOTAL_SIZE;
	pvsp->encode_payload_vaddr =
		pvsp->gwarp_payload_vaddr + GWARP_PAYLOAD_TOTAL_SIZE;

	pvsp->vout_payload_paddr = cmdp_phy_base +
				   (uint64_t)(pvsp->vout_payload_vaddr) -
				   (uint64_t)(pCmdp);
	pvsp->gwarp_payload_paddr = cmdp_phy_base +
				    (uint64_t)(pvsp->gwarp_payload_vaddr) -
				    (uint64_t)(pCmdp);
	pvsp->encode_payload_paddr = cmdp_phy_base +
				     (uint64_t)(pvsp->encode_payload_vaddr) -
				     (uint64_t)(pCmdp);

	return ret;
}

// p_vaddr, stores virtual address
// p_phy_addr, stores physical address
int get_encode_payload_addr(int channel_id, uint64_t *p_vaddr,
			    phys_addr_t *p_phy_addr)
{
	if ((channel_id < 0) || (channel_id > 8))
		return -1;

	if ((p_vaddr == NULL) || (p_phy_addr == NULL))
		return -1;

	*p_vaddr = (uint64_t)(pvsp->encode_payload_vaddr) +
		   (channel_id * ENCODE_VIDEO_PAYLOAD_SIZE);

	*p_phy_addr = pvsp->encode_payload_paddr +
		      (channel_id * ENCODE_VIDEO_PAYLOAD_SIZE);

	return 0;
}

void vsp_reg_write(void *reg, uint32_t mask, uint32_t val)
{
	uint32_t status;

	status = readl_relaxed(reg);
	status = ((status & mask) | val);
	writel_relaxed(status, reg);
}

int vsp_dev_init(struct vsp_device *pvsp)
{
	uint32_t status;
	void *reg0, *reg2;

	reg0 = ioremap(CTRL_REG0, 0x4);
	//	reg1 = ioremap(CTRL_REG1, 0x8);
	reg2 = ioremap(CTRL_REG2, 0x20);

	status = 0xffffffff;
	writel_relaxed(status, reg0);

	// writel_relaxed(0x0, pvsp->vreg.regctl);

	status = 0x0;
	writel_relaxed(status, pvsp->vreg.regctl); // reset regctl
	mdelay(5); // wait 5ms

	writel_relaxed(pvsp->firmware_c2.phy_addr, pvsp->vreg.regfw);
	mdelay(1);
	status = readl_relaxed(pvsp->vreg.regfw);
	if (status != pvsp->firmware_c2.phy_addr)
		writel_relaxed(pvsp->firmware_c2.phy_addr, pvsp->vreg.regfw);
	mdelay(1);

	writel_relaxed(pvsp->firmware_c1.phy_addr, pvsp->vreg.regfw + 4);
	mdelay(1);
	status = readl_relaxed(pvsp->vreg.regfw + 4);
	if (status != pvsp->firmware_c1.phy_addr)
		writel_relaxed(pvsp->firmware_c1.phy_addr,
			       pvsp->vreg.regfw + 4);
	mdelay(1);

	writel_relaxed(pvsp->firmware_c0.phy_addr, pvsp->vreg.regfw + 8);
	mdelay(1);
	status = readl_relaxed(pvsp->vreg.regfw + 8);
	if (status != pvsp->firmware_c0.phy_addr)
		writel_relaxed(pvsp->firmware_c0.phy_addr,
			       pvsp->vreg.regfw + 8);

	vsp_reg_write(reg2, REG2_MASK, 0x60606);
	vsp_reg_write(reg2, REG2_MASK, 0x60606);
	vsp_reg_write(reg2 + 0x4, REG2_MASK1, 0x6060606);
	vsp_reg_write(reg2 + 0x8, REG2_MASK1, 0x6060606);
	vsp_reg_write(reg2 + 0xc, REG2_MASK1, 0x6060606);
	vsp_reg_write(reg2 + 0x10, REG2_MASK1, 0x6060606);
	vsp_reg_write(reg2 + 0x14, REG2_MASK1, 0x6060606);
	vsp_reg_write(reg2 + 0x18, REG2_MASK1, 0x606060f);
	vsp_reg_write(reg2 + 0x1c, REG2_MASK2, 0x60f0000);

	return 0;
}

void boot_firmware(struct vsp_device *pvsp)
{
	uint32_t status;
	uint32_t *reg24;
	int vsp_parity_en;
	void __iomem *corenoc_parity_enable = NULL;

	corenoc_parity_enable = ioremap(SYS_PARITY_REG, 0x4);
	if (corenoc_parity_enable == NULL) {
		pr_err(
			"IOREMAP CORENOC_PARITY_ENABLE 0x33000084 FAILED\n");
		return ;
	}
	vsp_parity_en = readl_relaxed(corenoc_parity_enable) & VSP_PARITY_EN;
	iounmap(corenoc_parity_enable);
	if (vsp_parity_en) {
		// reg24, for parity
		reg24 = (uint32_t *)ioremap(VSP_CTRL24, 0x4);
		writel_relaxed(VSP_PARITY_CHECK_EN, reg24);
		status = readl_relaxed(reg24);
		iounmap(vsp_parity_en);
	}

	status = 0xffffffff;
	writel_relaxed(status, pvsp->vreg.regctl);
	status = readl_relaxed(pvsp->vreg.regctl);
}

void vsp_release_fimware(struct vsp_device *pvsp)
{
	release_firmware(pvsp->firmware_c0.fw);
	release_firmware(pvsp->firmware_c1.fw);
	release_firmware(pvsp->firmware_c2.fw);
}

int vsp_boot_firmware(struct vsp_device *pvsp)
{
	int ret;
	const struct firmware *fw;
	// char fw_name[64];

	ret = request_firmware(&pvsp->firmware_c0.fw, CORE_CODE_BIN, pvsp->dev);
	if (ret == 0) {
		fw = pvsp->firmware_c0.fw;
		memcpy(pvsp->firmware_c0.base, fw->data, fw->size);
	}

	ret = request_firmware(&pvsp->firmware_c1.fw, CORE_MD_BIN, pvsp->dev);
	if (ret == 0) {
		fw = pvsp->firmware_c1.fw;
		memcpy(pvsp->firmware_c1.base, fw->data, fw->size);
	}

	ret = request_firmware(&pvsp->firmware_c2.fw, CORE_ME_BIN, pvsp->dev);
	if (ret == 0) {
		fw = pvsp->firmware_c2.fw;
		memcpy(pvsp->firmware_c2.base, fw->data, fw->size);
	}

	// snprintf(fw_name, sizeof(fw_name), "vsp/%s", pvsp->slab_name);
	ret = request_firmware(&fw, VSP_SLAB_BIN, pvsp->dev);
	if (ret == 0)
		memcpy(pvsp->parti.slab_vaddr, fw->data, fw->size);

	boot_firmware(pvsp);

	return ret;
}

int vsp_runtime_fw_setup(struct vsp_device *pvsp)
{
	int ret = 0;

	pr_info("%s enter", __func__);

	ret = vsp_boot_firmware(pvsp);
	if (ret < 0) {
		pr_info("vsp boot fw fail, ret %d", ret);
		goto error_release_firmware;
	}

	pr_info("%s exit", __func__);
	return 0;

error_release_firmware:
	vsp_release_fimware(pvsp);
	return ret;
}

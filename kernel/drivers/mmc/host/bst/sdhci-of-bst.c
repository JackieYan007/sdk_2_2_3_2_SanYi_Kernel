// SPDX-License-Identifier: GPL-2.0
/*
* mmc driver for BST DesignWare Cores Mobile Storage Host Controller
* This file contains proprietary information that is the sole intellectual
* property of Black Sesame Technologies, Inc. and its affiliates.
* No portions of this material may be reproduced in any
* form without the written permission of:
* Black Sesame Technologies, Inc. and its affiliates
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050
* Copyright @2016: all right reserved.
*/

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/i2c.h>

#include "../sdhci-pltfm.h"
#include "bst-sdhci.h"


struct dwcmshc_priv {
	struct clk	*bus_clk;
};
#if 1//add by wangfei
#define REG_SDMMC_SOFTRST_SEL        0x33002004
#define REG_SD_EMMC_SEL       0x33000064  //bit0 for sd0/emmc0  bit1 for sd1/emmc1  (0:sd  1:emmc)
#define BST_SDMMC_VER_ID        0x3138302A
#define SDHCI_VENDOR_PTR_R	0xE8
#define SYS_CTRL_SDEMMC_DIV_CTRL     0x3300003C
#define SYS_CTRL_SDEMMC_CTRL_EN_CLR        0x33000068
#define TOP_IO_CFG_REG_R_IO_CFG_41 0x33001154
#define TOP_IO_CFG_REG_R_IO_CFG_42 0x33001158

#define MBIU_CTRL  0x510
#define BURST_INCR16_EN  BIT(3)
#define BURST_INCR8_EN  BIT(2)
#define BURST_INCR4_EN  BIT(1)
#define BURST_EN (BURST_INCR16_EN|BURST_INCR8_EN|BURST_INCR4_EN)
/* Synopsys vendor specific registers */
#define reg_offset_addr_vendor		(sdhci_readw(host, SDHCI_VENDOR_PTR_R))
#define SDHC_MHSC_VER_ID_R				(reg_offset_addr_vendor)
#define SDHC_MHSC_VER_TPYE_R				(reg_offset_addr_vendor+0X4)
#define SDHC_MHSC_CTRL_R					(reg_offset_addr_vendor+0X8)
#define SDHC_MBIU_CTRL_R					(reg_offset_addr_vendor+0X10)
#define SDHC_EMMC_CTRL_R					(reg_offset_addr_vendor+0X2C)
#define SDHC_BOOT_CTRL_R					(reg_offset_addr_vendor+0X2E)
#define SDHC_GP_IN_R						(reg_offset_addr_vendor+0X30)
#define SDHC_GP_OUT_R						(reg_offset_addr_vendor+0X34)
#define SDHC_AT_CTRL_R					(reg_offset_addr_vendor+0X40)
#define SDHC_AT_STAT_R					(reg_offset_addr_vendor+0X44)

#define SDHC_SW_TUNE_EN		0x00000010

/* MMCM DRP */
#define SDHC_MMCM_DIV_REG	0x1020
#define DIV_REG_100_MHZ		0x1145
#define DIV_REG_200_MHZ		0x1083
#define SDHC_MMCM_CLKFBOUT	0x1024
#define CLKFBOUT_100_MHZ	0x0000
#define CLKFBOUT_200_MHZ	0x0080
#define SDHC_CCLK_MMCM_RST	0x00000001
#define DRIVER_NAME "sdhci_bst"

/* I2C frame. */
#define BST_ADDRESS_BASE 0x08U      /* I2C device base address */
#define BST_COMM_FRAME_SIZE 0x03U   /* Length of the communication frame */
#define BST_FRAME_SIZE 0x04U        /* Length of the complete I2C frame */
#define BST_READ_FRAME_LENGTH 0x01U /* Length of the data frame for I2C read command. */
#define BST_RX_SIZE 0x02U           /* Length of the received I2C data frame */

/* CRC polynomial. */
#define BST_CRC_TBL_SIZE 256U       /* Size of CRC table. */
#define BST_CRC_POLYNOM 0x1DU       /* CRC polynom. */
#define BST_CRC_INIT 0xFFU          /* CRC initial value. */


#define SDHCI_DUMP_BST(f, x...) \
	pr_err("%s: " DRIVER_NAME ": " f, mmc_hostname(host->mmc), ## x)
#define SD_3_3V 0
#define SD_1_8V 1


struct i2c_inital_t {
    u8 reg_addr;
    u8 data;
};

static const uint8_t BST_CRC_TABLE[BST_CRC_TBL_SIZE] = {
    0x00U, 0x1DU, 0x3AU, 0x27U, 0x74U, 0x69U, 0x4EU, 0x53U, 0xE8U, 0xF5U, 0xD2U, 0xCFU, 0x9CU,
    0x81U, 0xA6U, 0xBBU, 0xCDU, 0xD0U, 0xF7U, 0xEAU, 0xB9U, 0xA4U, 0x83U, 0x9EU, 0x25U, 0x38U,
    0x1FU, 0x02U, 0x51U, 0x4CU, 0x6BU, 0x76U, 0x87U, 0x9AU, 0xBDU, 0xA0U, 0xF3U, 0xEEU, 0xC9U,
    0xD4U, 0x6FU, 0x72U, 0x55U, 0x48U, 0x1BU, 0x06U, 0x21U, 0x3CU, 0x4AU, 0x57U, 0x70U, 0x6DU,
    0x3EU, 0x23U, 0x04U, 0x19U, 0xA2U, 0xBFU, 0x98U, 0x85U, 0xD6U, 0xCBU, 0xECU, 0xF1U, 0x13U,
    0x0EU, 0x29U, 0x34U, 0x67U, 0x7AU, 0x5DU, 0x40U, 0xFBU, 0xE6U, 0xC1U, 0xDCU, 0x8FU, 0x92U,
    0xB5U, 0xA8U, 0xDEU, 0xC3U, 0xE4U, 0xF9U, 0xAAU, 0xB7U, 0x90U, 0x8DU, 0x36U, 0x2BU, 0x0CU,
    0x11U, 0x42U, 0x5FU, 0x78U, 0x65U, 0x94U, 0x89U, 0xAEU, 0xB3U, 0xE0U, 0xFDU, 0xDAU, 0xC7U,
    0x7CU, 0x61U, 0x46U, 0x5BU, 0x08U, 0x15U, 0x32U, 0x2FU, 0x59U, 0x44U, 0x63U, 0x7EU, 0x2DU,
    0x30U, 0x17U, 0x0AU, 0xB1U, 0xACU, 0x8BU, 0x96U, 0xC5U, 0xD8U, 0xFFU, 0xE2U, 0x26U, 0x3BU,
    0x1CU, 0x01U, 0x52U, 0x4FU, 0x68U, 0x75U, 0xCEU, 0xD3U, 0xF4U, 0xE9U, 0xBAU, 0xA7U, 0x80U,
    0x9DU, 0xEBU, 0xF6U, 0xD1U, 0xCCU, 0x9FU, 0x82U, 0xA5U, 0xB8U, 0x03U, 0x1EU, 0x39U, 0x24U,
    0x77U, 0x6AU, 0x4DU, 0x50U, 0xA1U, 0xBCU, 0x9BU, 0x86U, 0xD5U, 0xC8U, 0xEFU, 0xF2U, 0x49U,
    0x54U, 0x73U, 0x6EU, 0x3DU, 0x20U, 0x07U, 0x1AU, 0x6CU, 0x71U, 0x56U, 0x4BU, 0x18U, 0x05U,
    0x22U, 0x3FU, 0x84U, 0x99U, 0xBEU, 0xA3U, 0xF0U, 0xEDU, 0xCAU, 0xD7U, 0x35U, 0x28U, 0x0FU,
    0x12U, 0x41U, 0x5CU, 0x7BU, 0x66U, 0xDDU, 0xC0U, 0xE7U, 0xFAU, 0xA9U, 0xB4U, 0x93U, 0x8EU,
    0xF8U, 0xE5U, 0xC2U, 0xDFU, 0x8CU, 0x91U, 0xB6U, 0xABU, 0x10U, 0x0DU, 0x2AU, 0x37U, 0x64U,
    0x79U, 0x5EU, 0x43U, 0xB2U, 0xAFU, 0x88U, 0x95U, 0xC6U, 0xDBU, 0xFCU, 0xE1U, 0x5AU, 0x47U,
    0x60U, 0x7DU, 0x2EU, 0x33U, 0x14U, 0x09U, 0x7FU, 0x62U, 0x45U, 0x58U, 0x0BU, 0x16U, 0x31U,
    0x2CU, 0x97U, 0x8AU, 0xADU, 0xB0U, 0xE3U, 0xFEU, 0xD9U, 0xC4U
};

struct i2c_inital_t i2c_enable_value[3] = {

    {0x8D, 0x01},
    {0x93, 0x01},
    {0x99, 0x01},
};

struct i2c_inital_t i2c_inital_value[6] = {

    {0x8E, 0x02},
    {0x8E, 0x07},
    {0x94, 0x02},
    {0x94, 0x07},    
    {0x9A, 0x02},
    {0x9A, 0x07},
};
void sdhci_bst_print_vendor(struct sdhci_host *host)
{
	SDHCI_DUMP_BST("============ SDHCI VENDOR REGISTER DUMP ===========\n");

	SDHCI_DUMP_BST("VER_ID:  0x%08x | VER_TPYE:  0x%08x\n",
		   sdhci_readl(host, SDHC_MHSC_VER_ID_R),
		   sdhci_readl(host, SDHC_MHSC_VER_TPYE_R));
	SDHCI_DUMP_BST("MHSC_CTRL:  0x%08x |MBIU_CTRL:  0x%08x\n",
		   sdhci_readw(host, SDHC_MHSC_CTRL_R),
		   sdhci_readw(host, SDHC_MBIU_CTRL_R));
	SDHCI_DUMP_BST("EMMC_CTRL:  0x%08x | BOOT_CTRL: 0x%08x\n",
		   sdhci_readl(host, SDHC_EMMC_CTRL_R),
		   sdhci_readw(host, SDHC_BOOT_CTRL_R));
	SDHCI_DUMP_BST("GP_IN:   0x%08x | GP_OUT: 0x%08x\n",
		   sdhci_readl(host, SDHC_GP_IN_R),
		   sdhci_readb(host, SDHC_GP_OUT_R));
	SDHCI_DUMP_BST("AT_CTRL:     0x%08x | AT_STAT:  0x%08x\n",
		   sdhci_readb(host, SDHC_AT_CTRL_R),
		   sdhci_readb(host, SDHC_AT_STAT_R));

}
EXPORT_SYMBOL_GPL(sdhci_bst_print_vendor);

static u32 bst_read_phys_bst(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}


static void bst_write_phys_bst(u32 phys_addr, u32 value)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}
}
static unsigned int bst_get_max_clock(struct sdhci_host *host)
{
	return host->mmc->f_max;
}

static unsigned int bst_get_min_clock(struct sdhci_host *host)
{
	return host->mmc->f_min;
}

void sdhci_enable_bst_clk(struct sdhci_host *host, unsigned int clk)
{
    #define default_max_freq 200000ul
    unsigned int div;
    unsigned int tmp;

    if (clk == 0)
    {
	div = clk;
    } else if(clk>default_max_freq)
    {
	div = clk/1000;
        div = default_max_freq/div;
    } else if(clk<1500)
    {
        div = clk;
    } else
    {
	div = default_max_freq*100;
	div = div/clk;
        div /= 100;
    }

    clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk &= ~SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

    if (div != 0)
    {
        tmp = bst_read_phys_bst(SYS_CTRL_SDEMMC_DIV_CTRL);
        if (host->mmc->caps2 & MMC_CAP2_NO_SD)
	{
            bst_write_phys_bst(SYS_CTRL_SDEMMC_DIV_CTRL, (tmp&(~(0x3ff<<10)))|((div&0x3ff)<<10));           
            bst_write_phys_bst(SYS_CTRL_SDEMMC_CTRL_EN_CLR, 0xF0);
        } else if(host->mmc->caps2 &MMC_CAP2_NO_MMC)
	{
            bst_write_phys_bst(SYS_CTRL_SDEMMC_DIV_CTRL, (tmp&(~(0x3ff)))|(div&0x3ff));
            bst_write_phys_bst(SYS_CTRL_SDEMMC_CTRL_EN_CLR, 0xF00);
	}
    }
    clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
    clk |= SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

}
static u16 bst_sdhci_calc_clk(struct sdhci_host *host, unsigned int clock,
		   unsigned int *actual_clock)
{
	int div = 0; /* Initialized for compiler warning */
	int real_div = div, clk_mul = 1;
	u16 clk = 0;
	bool switch_base_clk = false;

	if (host->version >= SDHCI_SPEC_300) {
		if (host->clk_mul) {
			for (div = 1; div <= 1024; div++) {
				if ((host->max_clk * host->clk_mul / div)
					<= clock)
					break;
			}
			if ((host->max_clk * host->clk_mul / div) <= clock) {
				/*
				 * Set Programmable Clock Mode in the Clock
				 * Control register.
				 */

				clk = SDHCI_PROG_CLOCK_MODE;
				real_div = div;
				clk_mul = host->clk_mul;
				div--;
			} else {
				/*
				 * Divisor can be too small to reach clock
				 * speed requirement. Then use the base clock.
				 */
				switch_base_clk = true;
			}
		}

		if (!host->clk_mul || switch_base_clk) {
			/* Version 3.00 divisors must be a multiple of 2. */
			if (host->max_clk <= clock)
				div = 1;
			else {
				for (div = 2; div < SDHCI_MAX_DIV_SPEC_300;
				     div += 2) {
					if ((host->max_clk / div) <= clock)
						break;
				}
			}
			real_div = div;
			div >>= 1;
			if ((host->quirks2 & SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN)
				&& !div && host->max_clk <= 25000000)
				div = 1;
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((host->max_clk / div) <= clock)
				break;
		}
		real_div = div;

	}

clock_set:
	if (real_div)
		*actual_clock = (host->max_clk * clk_mul) / real_div;
    clk = real_div;
	return clk;
}

void sdhci_set_bst_clock(struct sdhci_host *host, unsigned int clock)
{
    struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	unsigned int clk;

	host->mmc->actual_clock = 0;

	if (clock == 0)
		return;
    if (host->quirks2 & SDHCI_QUIRK2_CLK_FROM_DTS)
    {
        if (pltfm_host->clock > clock)
	{
	    clk = bst_sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
        } else
	{
            host->mmc->actual_clock = pltfm_host->clock;
	    clk = host->mmc->actual_clock/1000;
	}
    } else
    {
	clk = bst_sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
    }

	sdhci_enable_bst_clk(host, clk);
}



static uint8_t BST_CalcCRC(const uint8_t *data, uint8_t dataLen)
{
    uint8_t crc;      /* Result. */
    uint8_t tableIdx; /* Index to the CRC table. */
    uint8_t dataIdx;  /* Index to the data array (memory). */

    /* Set CRC seed value. */
    crc = BST_CRC_INIT;

    for (dataIdx = dataLen - 1; dataIdx > 0; dataIdx--)
    {
	tableIdx = crc ^ data[dataIdx];
	crc = BST_CRC_TABLE[tableIdx];
    }
    return crc;
}


static int sdhci_bst_i2c_read_bytes(u8 addr, u8 cmd, u8 *data, u8 data_len)
{

	struct i2c_msg msgs[2];
	int ret;
	u8 *buffer;
    struct i2c_adapter *adapter;

	buffer = kzalloc(data_len, GFP_KERNEL);
	if (!buffer)
		return AE_NO_MEMORY;

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;

	msgs[1].addr = addr;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = data_len;
	msgs[1].buf = buffer;
    adapter = i2c_get_adapter(5);
    if (!adapter)
    {
	kfree(buffer);
	pr_err("%s: i2c_get_adapter failed\n", __func__);
	return -ENODEV;
    }

	ret = i2c_transfer(adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		/* Getting a NACK is unfortunately normal with some DSTDs */
		if (ret == -EREMOTEIO)
			pr_err("%s:  i2c read %d bytes from client@%#x starting at reg %#x failed, error: %d\n",
				__func__, data_len, addr, cmd, ret);
		else
			pr_err("%s:  i2c read %d bytes from client@%#x starting at reg %#x failed, error: %d\n",
				__func__, data_len, addr, cmd, ret);
	/* 2 transfers must have completed successfully */
	} else if (ret == 2) {
		memcpy(data, buffer, data_len);
		ret = 0;
	} else {
		ret = -EIO;
	}

	kfree(buffer);
	return ret;
}

static int sdhci_bst_i2c_write_bytes(u8 addr, u8 cmd, u8 *data, u8 data_len)
{

	struct i2c_msg msgs[1];
	u8 *buffer;
	uint8_t txFrameCrc[BST_FRAME_SIZE] = {0};
    struct i2c_adapter *adapter;
    u8 crcval;
    int i;
	int ret = AE_OK;

	buffer = kzalloc(data_len*2 + 1, GFP_KERNEL);
	if (!buffer)
		return AE_NO_MEMORY;

    txFrameCrc[3] = (uint8_t)(addr << 1U); /* R/W bit is 0 for write */

     /* Sets address of the register (first two bits are ignored). */
    txFrameCrc[2] = cmd;

     /* Sets data. */
    txFrameCrc[1] = data[0];

	/* Add CRC. */
    crcval = BST_CalcCRC(txFrameCrc, BST_FRAME_SIZE);

	buffer[0] = cmd;
    for (i = 0; i < data_len; i++)
    {
	memcpy(buffer + 2*i+1, data, 1);
        memcpy(buffer + 2*i + 2, &crcval, 1);
    }

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = data_len*2 + 1;
	msgs[0].buf = buffer;
    adapter = i2c_get_adapter(5);
    if (!adapter)
    {
	kfree(buffer);
	pr_err("%s: i2c_get_adapter failed\n", __func__);
	return -ENODEV;
    }
	ret = i2c_transfer(adapter, msgs, ARRAY_SIZE(msgs));

	kfree(buffer);

	if (ret < 0) {
		pr_err("%s: i2c write failed: %d\n", __func__, ret);
		return ret;
	}

	/* 1 transfer must have completed successfully */
	return (ret == 1) ? 0 : -EIO;
}

static int sdhci_bst_i2c_voltage_sel(u32 voltage)
{
    u8 test_data;

    sdhci_bst_i2c_write_bytes(8, i2c_enable_value[0].reg_addr, (u8*)&i2c_enable_value[0].data, 1);    

    sdhci_bst_i2c_read_bytes(8, i2c_enable_value[0].reg_addr, &test_data, 1);
    if (test_data != i2c_enable_value[0].data)
    {
        pr_err("%s: i2c test failed readdata: %d send data:%d\n", __func__, test_data, i2c_enable_value[0].data);
	return 1;
    }

    if (voltage == SD_1_8V)
    {

        sdhci_bst_i2c_write_bytes(8, i2c_inital_value[0].reg_addr, (u8*)&i2c_inital_value[0].data, 1);            
        sdhci_bst_i2c_read_bytes(8, i2c_inital_value[0].reg_addr, &test_data, 1);
        if (test_data != i2c_inital_value[0].data)
	{
            pr_err("%s: i2c test failed readdata: %d send data:%d\n", __func__, test_data, i2c_inital_value[0].data);
	    return 1;
	}

	udelay(1200);
        bst_write_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_41, bst_read_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_41)|0x2A);
        bst_write_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_42, bst_read_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_42)|0x2A);
    } else if(voltage == SD_3_3V)
    {

        bst_write_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_41, bst_read_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_41)&(~0x2A));
        bst_write_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_42, bst_read_phys_bst(TOP_IO_CFG_REG_R_IO_CFG_42)&(~0x2A));
	udelay(1200);
        sdhci_bst_i2c_write_bytes(8, i2c_inital_value[1].reg_addr, (u8*)&i2c_inital_value[1].data, 1);
        sdhci_bst_i2c_read_bytes(8, i2c_inital_value[1].reg_addr, &test_data, 1);
        if (test_data != i2c_inital_value[1].data)
	{
            pr_err("%s: i2c test failed readdata: %d send data:%d\n", __func__, test_data, i2c_inital_value[1].data);
	    return 1;
	}

    }

    return 0;
}

static void sdhci_bst_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
    if (host->mmc->caps2 & MMC_CAP2_NO_SD)
    {
    	sdhci_writew(host, sdhci_readw(host, SDHC_EMMC_CTRL_R) & (~BIT(2)), SDHC_EMMC_CTRL_R);        
	sdhci_reset(host, mask);
	udelay(10);
	sdhci_writew(host, sdhci_readw(host, SDHC_EMMC_CTRL_R)|BIT(2), SDHC_EMMC_CTRL_R);
    } else
	sdhci_reset(host, mask);
}
static void sdhci_bst_timeout(struct sdhci_host *host, struct mmc_command *cmd)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
    sdhci_writeb(host, 0xE, SDHCI_TIMEOUT_CONTROL);
}

static void sdhci_bst_set_power(struct sdhci_host *host, unsigned char mode,
		     unsigned short vdd)
{
	if (!IS_ERR(host->mmc->supply.vmmc)) {
		struct mmc_host *mmc = host->mmc;

		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, vdd);
	}
	sdhci_set_power(host, mode, vdd);
    sdhci_writeb(host, 0xF, SDHCI_POWER_CONTROL);
    sdhci_writew(host, (sdhci_readw(host, MBIU_CTRL)&(~0xf))|BURST_EN, MBIU_CTRL);
}

static void sdhci_bst_voltage_switch(struct sdhci_host *host)
{
    struct mmc_ios *ios = &host->mmc->ios;

    if (host->mmc->caps2 & MMC_CAP2_NO_MMC)
    {
	if (sdhci_bst_i2c_voltage_sel(ios->signal_voltage))
	{
	    pr_err("sdhci_bst_voltage_switch failed\n");
        } else
	{
	    pr_info("sdhci_bst_voltage_switch ok\n");
	}
    }
}


static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock		= sdhci_set_bst_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
	.get_min_clock		= bst_get_min_clock,
	.get_max_clock		= bst_get_max_clock,
	.reset			= sdhci_bst_reset,
	.set_power = sdhci_bst_set_power,
    .set_timeout = sdhci_bst_timeout,
    .voltage_switch = sdhci_bst_voltage_switch,
};
#endif
static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_DELAY_AFTER_POWER|SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN|SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
    .quirks2 = SDHCI_QUIRK2_BROKEN_DDR50|SDHCI_QUIRK2_CLK_FROM_DTS|SDHCI_QUIRK2_BROKEN_HS200|SDHCI_QUIRK2_TUNING_WORK_AROUND,
};

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;
    char c;

    dev_err(&pdev->dev, "dwcmshc_probe\n");

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
    {
		clk_prepare_enable(priv->bus_clk);
    }
	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	sdhci_get_of_property(pdev);
    bst_write_phys_bst(0x3300005C, 0xc8c8c8c8);//timer 
    if (host->mmc->caps&(MMC_CAP_SD_HIGHSPEED|MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			  MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104 |
			  MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR))
    {
        bst_write_phys_bst(0x33000060, 0x0);//  tx 
    } else
    {
        bst_write_phys_bst(0x33000060, 0x80);//  tx 
    }
    bst_write_phys_bst(SYS_CTRL_SDEMMC_CTRL_EN_CLR, 0xFF0);
    if (sdhci_readl(host, SDHC_MHSC_VER_ID_R) != BST_SDMMC_VER_ID)
    {
	dev_err(&pdev->dev, "dwcmshc_probe wrong ver id\n");
	goto err_clk;
    }

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "bst,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.of_match_table = sdhci_dwcmshc_dt_ids,
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for BST DWC MSHC");
MODULE_AUTHOR("fei wang <fei.wang@bst.ai>");
MODULE_LICENSE("GPL v2");

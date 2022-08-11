/*******************************************************************************
  BSTGMAC Ethernet Driver -- MDIO bus implementation
  Provides Bus interface for MII registers

  Copyright (C) 2007-2009  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Carl Shaw <carl.shaw@st.com>
  Maintainer: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/slab.h>

//#include "dwxgmac2.h"
#include "bstgmac.h"

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

/* GMAC4 defines */
#define MII_GMAC4_GOC_SHIFT		2
#define MII_GMAC4_WRITE			(1 << MII_GMAC4_GOC_SHIFT)
#define MII_GMAC4_READ			(3 << MII_GMAC4_GOC_SHIFT)

/* XGMAC defines */
#define MII_XGMAC_SADDR			BIT(18)
#define MII_XGMAC_CMD_SHIFT		16
#define MII_XGMAC_WRITE			(1 << MII_XGMAC_CMD_SHIFT)
#define MII_XGMAC_READ			(3 << MII_XGMAC_CMD_SHIFT)
#define MII_XGMAC_BUSY			BIT(22)
#define MII_XGMAC_MAX_C22ADDR		3
#define MII_XGMAC_C22P_MASK		GENMASK(MII_XGMAC_MAX_C22ADDR, 0)
#define MII_ANOTHER_ADDR_FLAG   (0x8000)
#define MII_ANOTHER_ADDR        (0)
#define MII_PHYID_REG           (0x2)

/* 88e6352 defines */
#define PHY_REG_SMI_BASE        (0x0)
#define SWITCH_REG_SMI_BASE     (0x10)
#define SMI_BUSY_BIT            (15)
#define SMI_MODE_BIT            (12)
#define SMI_OP_BIT              (10)
#define SMI_DEVADDR_BIT         (5)
#define SMI_REGADDR_BIT         (0)
#define SMI_INDIRECT_COM_REG    (0)
#define SMI_INDIRECT_DATA_REG   (1)
#define SMI_CLAUSE_22_MODE      (1)
#define SMI_C22_READ_MODE       (2)
#define SMI_C22_WRITE_MODE      (1)
#define GMAC1_COM_88E6352_PORT  (5)
#define TC397_COM_88E6352_PORT  (6)
#define GMAC1_100M_IF_NUM       (3)

#define ECV3_GMAC1_COM_88E6352_PORT  (6)
#define ECV3_GMAC1_1000M_COPP_NUM	(4)
#define ECV3_GMAC1_1000M_FIBER_PORT	(ECV3_GMAC1_1000M_COPP_NUM+1)
#define ECV3_GMAC1_1000M_FIBER_NUM	(1)

#define SMI_C45_READ_MODE       (2)
#define SMI_C45_WRITE_MODE      (1)
#define SMI_C45_WRITE_ADDR      (0)
#define SMI_CLAUSE_45_MODE      (0)
#define SMI_GLOBAL2_DEVICE      (0x1c)
#define SMI_GLOBAL2_COM_REG     (0x18)
#define SMI_GLOBAL2_DATA_REG    (0x19)

static int bstgmac_mdio_real_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
    struct net_device *ndev = bus->priv;
	struct bstgmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	u32 v;
	int data;
	u32 value = MII_BUSY;

	if (!(phyreg & MII_ADDR_C45)) {
        value |= (phyaddr << priv->hw->mii.addr_shift)
            & priv->hw->mii.addr_mask;
        value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
        value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
            & priv->hw->mii.clk_csr_mask;
        if (priv->plat->has_gmac4)
            value |= MII_GMAC4_READ;
    
        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;

        writel(value, priv->ioaddr + mii_address);

        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;

        /* Read the data from the MII data register */
        data = (int)readl(priv->ioaddr + mii_data);
        data &= 0xffff;
        return data;
    } else {
        writel((phyreg & 0xffff) << 16, priv->ioaddr + mii_data);
        value |= (phyaddr << priv->hw->mii.addr_shift)
            & priv->hw->mii.addr_mask;
        value |= (phyreg & priv->hw->mii.reg_mask);
        value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
            & priv->hw->mii.clk_csr_mask;
        value |= (1 << 1) & GENMASK(1,1);//C45    
        if (priv->plat->has_gmac4)
            value |= MII_GMAC4_READ;
 
        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;

        writel(value, priv->ioaddr + mii_address);

        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;

        /* Read the data from the MII data register */
        data = (int)readl(priv->ioaddr + mii_data);
        data &= 0xffff;
        return data;
    }
}

static int bstgmac_mdio_real_write(struct mii_bus *bus, int phyaddr, int phyreg, u16 phydata)
{
    struct net_device *ndev = bus->priv;
	struct bstgmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	u32 v;
	u32 value = MII_BUSY;
   
	if (!(phyreg & MII_ADDR_C45)) {
        value |= (phyaddr << priv->hw->mii.addr_shift)
            & priv->hw->mii.addr_mask;
        value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;

        value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
            & priv->hw->mii.clk_csr_mask;
        if (priv->plat->has_gmac4)
            value |= MII_GMAC4_WRITE;
        else
            value |= MII_WRITE;

        /* Wait until any existing MII operation is complete */
        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;
        /* Set the MII address register to write */
        writel(phydata, priv->ioaddr + mii_data);
        writel(value, priv->ioaddr + mii_address);

        /* Wait until any existing MII operation is complete */
        return readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000);
    } else {
        writel((phydata &0xffff) |((phyreg & 0xffff) << 16), priv->ioaddr+mii_data);
        value |= (phyaddr << priv->hw->mii.addr_shift)
            & priv->hw->mii.addr_mask;
        value |= (phyreg & priv->hw->mii.reg_mask);

        value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
            & priv->hw->mii.clk_csr_mask;
        value |= (1 << 1) & GENMASK(1,1);//C45 
        if (priv->plat->has_gmac4)
            value |= MII_GMAC4_WRITE;
        else
            value |= MII_WRITE;

        /* Wait until any existing MII operation is complete */
        if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000))
            return -EBUSY;

        writel(value, priv->ioaddr + mii_address);

        /* Wait until any existing MII operation is complete */
        return readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
                    100, 10000);
    }
}

static int bstgmac_mdio_read_88e6352_swreg(struct mii_bus *bus, int port, int phyreg)
{
	u16 phyaddr;
    int phydata;

    phyaddr = SWITCH_REG_SMI_BASE + port;
    phydata =  bstgmac_mdio_real_read(bus, phyaddr, phyreg);

    //pr_debug("%s line %d swp %d phyaddr %d reg%d data 0x%x\n", __func__, __LINE__,  port, phyaddr, phyreg, phydata);
    return phydata;
}

static int bstgmac_mdio_write_88e6352_swreg(struct mii_bus *bus, int port, int phyreg, u16 phydata)
{
	u16 phyaddr;

    phyaddr = SWITCH_REG_SMI_BASE + port;

    //pr_debug("%s line %d swp %d phyaddr %d reg%d data 0x%x\n", __func__, __LINE__,  port, phyaddr, phyreg, phydata);

    bstgmac_mdio_real_write(bus, phyaddr, phyreg, phydata);

    return 0;
}

static int bstgmac_mdio_read_88e6352_phyreg(struct mii_bus *bus, int port, int phyreg)
{
	u16 phyaddr, value;
    u32 i = 100;
    int phydata;

    phyaddr = PHY_REG_SMI_BASE + port;
   
    value = (1 << SMI_BUSY_BIT) | (SMI_CLAUSE_22_MODE << SMI_MODE_BIT) | (SMI_C22_READ_MODE << SMI_OP_BIT) 
            | (phyaddr << SMI_DEVADDR_BIT) | (phyreg << SMI_REGADDR_BIT);
    bstgmac_mdio_real_write(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG, value);
    
    value = bstgmac_mdio_real_read(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG);
    while((value & (1 << SMI_BUSY_BIT)) && (i--)) {
        udelay(10);
        value = bstgmac_mdio_real_read(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG);
    }
    if (i == 0) {
        return -1;
    }

    phydata = bstgmac_mdio_real_read(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_DATA_REG);
    
    //pr_debug("%s line %d phyport %d phyaddr %d reg%d data 0x%x\n", __func__, __LINE__,  port, phyaddr, phyreg, phydata);
    
    return phydata;
}

static int bstgmac_mdio_write_88e6352_phyreg(struct mii_bus *bus, int port, int phyreg, u16 phydata)
{
	u16 phyaddr, value;
    u32 i = 100;

    bstgmac_mdio_real_write(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_DATA_REG, phydata);
    
    phyaddr = PHY_REG_SMI_BASE + port;
    value = (1 << SMI_BUSY_BIT) | (SMI_CLAUSE_22_MODE << SMI_MODE_BIT) | (SMI_C22_WRITE_MODE << SMI_OP_BIT) 
            | (phyaddr << SMI_DEVADDR_BIT) | (phyreg << SMI_REGADDR_BIT);
    bstgmac_mdio_real_write(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG, value);

    value = bstgmac_mdio_real_read(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG);
    while((value & (1 << SMI_BUSY_BIT)) && (i--)) {
        udelay(10);
        value = bstgmac_mdio_real_read(bus, SMI_GLOBAL2_DEVICE, SMI_GLOBAL2_COM_REG);
    }
    if (i == 0) {
        return -1;
    }
    //pr_debug("%s line %d phyport %d phyaddr %d reg%d data 0x%x\n", __func__, __LINE__,  port, phyaddr, phyreg, phydata);
    return 0;
}

int bstgmac_config_mv88e6352(struct net_device *ndev)
{
    struct bstgmac_priv *priv = netdev_priv(ndev);
    struct mii_bus *bus = priv->mii;
	int save_page, val;
    int i;

    if (priv->extend_op == BSTA1000_BOARD_ECU) {
		for (i = 0; i < GMAC1_100M_IF_NUM; i++) {
	        bstgmac_mdio_write_88e6352_phyreg(bus, i, 0, 0x3300);
	        bstgmac_mdio_write_88e6352_phyreg(bus, i, 16, 0x3b60);
	    }
		bstgmac_mdio_write_88e6352_swreg(bus, TC397_COM_88E6352_PORT, 1, 0xc03e); //link/speed1000/full duplex
    }

	if (priv->extend_op == BSTA1000_BOARD_ECV3) {
		for (i = 1; i <= ECV3_GMAC1_1000M_COPP_NUM; i++) {
	        bstgmac_mdio_write_88e6352_phyreg(bus, i, 0, 0x1340);
	        bstgmac_mdio_write_88e6352_phyreg(bus, i, 16, 0x3360);
	    }
		save_page = bstgmac_mdio_read_88e6352_phyreg(bus, ECV3_GMAC1_1000M_FIBER_PORT, 0x16);
		bstgmac_mdio_write_88e6352_phyreg(bus, ECV3_GMAC1_1000M_FIBER_PORT, 0X16, 1);
		bstgmac_mdio_write_88e6352_phyreg(bus, ECV3_GMAC1_1000M_FIBER_PORT, 0, 0x1340);
	    bstgmac_mdio_write_88e6352_phyreg(bus, ECV3_GMAC1_1000M_FIBER_PORT, 16, 0x3360);
		bstgmac_mdio_write_88e6352_phyreg(bus, ECV3_GMAC1_1000M_FIBER_PORT, 0x16, save_page);	
	}
    
    
    return 0;
}

/**
 * bstgmac_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr
 * @phyreg: MII reg
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
static int bstgmac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
    struct  net_device *ndev = bus->priv;
	struct  bstgmac_priv *priv = netdev_priv(ndev);
    int     data;
    
    if ((priv->extend_op == BSTA1000_BOARD_ECU) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
        bstgmac_mdio_write_88e6352_swreg(bus, GMAC1_COM_88E6352_PORT, phyreg, phydata);
        return 0;
    } else if ((priv->extend_op == BSTA1000_BOARD_ECV3) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
		bstgmac_mdio_write_88e6352_swreg(bus, ECV3_GMAC1_COM_88E6352_PORT, phyreg, phydata);
        return 0;
    } else {
        data = bstgmac_mdio_real_read(bus, phyaddr, MII_PHYID_REG);
        if (data == 0xffff) {
            data = bstgmac_mdio_real_write(bus, MII_ANOTHER_ADDR, phyreg, phydata);
        } else {
            data = bstgmac_mdio_real_write(bus, phyaddr, phyreg, phydata);
        }
        return data;
    }

}

/**
 * bstgmac_mdio_read
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr
 * @phyreg: MII reg
 * Description: it reads data from the MII register from within the phy device.
 * For the 7111 GMAC, we must set the bit 0 in the MII address register while
 * accessing the PHY registers.
 * Fortunately, it seems this has no drawback for the 7109 MAC.
 */
static int bstgmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
    struct net_device *ndev = bus->priv;
	struct bstgmac_priv *priv = netdev_priv(ndev); 
    int     data;
    
    if ((priv->extend_op == BSTA1000_BOARD_ECU) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
        data = bstgmac_mdio_read_88e6352_swreg(bus, GMAC1_COM_88E6352_PORT, phyreg);
        return data; 
    } else if ((priv->extend_op == BSTA1000_BOARD_ECV3) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
		data = bstgmac_mdio_read_88e6352_swreg(bus, ECV3_GMAC1_COM_88E6352_PORT, phyreg);
		return data; 
    } else {
        data = bstgmac_mdio_real_read(bus, phyaddr, MII_PHYID_REG);
        if (data == 0xffff) {
            data = bstgmac_mdio_real_read(bus, MII_ANOTHER_ADDR, phyreg);
        } else {
            data = bstgmac_mdio_real_read(bus, phyaddr, phyreg);
        }
    }

    return data;
}

/**
 * bstgmac_mdio_reset
 * @bus: points to the mii_bus structure
 * Description: reset the MII bus
 */
int bstgmac_mdio_reset(struct mii_bus *bus)
{
	struct net_device *ndev = bus->priv;
	struct bstgmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	struct stmmac_mdio_bus_data *data = priv->plat->mdio_bus_data;
    
#ifdef CONFIG_OF
	if (priv->plat->phy_node) {	
        struct device_node *np = priv->plat->phy_node;

        if (!np)
            return 0;

        if (((priv->extend_op == BSTA1000_BOARD_ECU) || (priv->extend_op == BSTA1000_BOARD_ECV3)) && (priv->plat->bus_id == BSTGMAC1_BUS_ID)) {
            data->reset_gpio = of_get_named_gpio(np, "reset-swpin", 0);
        } else {
            data->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
        }

        if (data->reset_gpio < 0)
            return 0;

        data->active_low = of_property_read_bool(np,
                    "reset-active-low");

        of_property_read_u32(np,
            "reset-assert-us", &data->delays[1]);

        of_property_read_u32(np,
            "reset-deassert-us", &data->delays[2]);
#if 0 
        gpio module has requested
        if (devm_gpio_request(priv->device, data->reset_gpio,
                        "mdio-reset"))
            return 0;	
#endif

		gpio_direction_output(data->reset_gpio,
				      data->active_low ? 1 : 0);
		msleep(DIV_ROUND_UP(500, 1000));

        gpio_set_value(data->reset_gpio, data->active_low ? 0 : 1);
		if (data->delays[1])
			msleep(DIV_ROUND_UP(data->delays[1], 1000));

		gpio_set_value(data->reset_gpio, data->active_low ? 1 : 0);
		if (data->delays[2])
			msleep(DIV_ROUND_UP(data->delays[2], 1000));
	}
#endif


	/* This is a workaround for problems with the STE101P PHY.
	 * It doesn't complete its reset until at least one clock cycle
	 * on MDC, so perform a dummy mdio read. To be updated for GMAC4
	 * if needed.
	 */
	if (!priv->plat->has_gmac4)
		writel(0, priv->ioaddr + mii_address);

	return 0;
}

/**
 * bstgmac_mdio_register
 * @ndev: net device structure
 * Description: it registers the MII bus
 */
int bstgmac_mdio_register(struct net_device *ndev)
{
	int err = 0;
	struct mii_bus *new_bus;
	struct bstgmac_priv *priv = netdev_priv(ndev);
	struct stmmac_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;
	struct device_node *mdio_node = priv->plat->mdio_node;
	struct device *dev = ndev->dev.parent;
	int addr, found, max_addr;

	if (!mdio_bus_data)
		return 0;

	new_bus = mdiobus_alloc();
	if (!new_bus)
		return -ENOMEM;

	if (mdio_bus_data->irqs)
		memcpy(new_bus->irq, mdio_bus_data->irqs, sizeof(new_bus->irq));

#ifdef CONFIG_OF
	if (priv->device->of_node)
		mdio_bus_data->reset_gpio = -1;
#endif

	new_bus->name = "bstgmac";

	new_bus->read = &bstgmac_mdio_read;
	new_bus->write = &bstgmac_mdio_write;
	max_addr = PHY_MAX_ADDR;

	new_bus->reset = &bstgmac_mdio_reset;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 new_bus->name, priv->plat->bus_id);
	new_bus->priv = ndev;
	new_bus->phy_mask = mdio_bus_data->phy_mask;
	new_bus->parent = priv->device;

	err = of_mdiobus_register(new_bus, mdio_node);
	if (err != 0) {
		dev_err(dev, "Cannot register the MDIO bus err %d\n", err);
		goto bus_register_fail;
	}

	if (priv->plat->phy_node || mdio_node)
		goto bus_register_done;

	found = 0;
	for (addr = 0; addr < max_addr; addr++) {
		struct phy_device *phydev = mdiobus_get_phy(new_bus, addr);

		if (!phydev)
			continue;

		/*
		 * If an IRQ was provided to be assigned after
		 * the bus probe, do it here.
		 */
		if (!mdio_bus_data->irqs &&
		    (mdio_bus_data->probed_phy_irq > 0)) {
			new_bus->irq[addr] = mdio_bus_data->probed_phy_irq;
			phydev->irq = mdio_bus_data->probed_phy_irq;
		}

		/*
		 * If we're going to bind the MAC to this PHY bus,
		 * and no PHY number was provided to the MAC,
		 * use the one probed here.
		 */
		if (priv->plat->phy_addr == -1)
			priv->plat->phy_addr = addr;

		phy_attached_info(phydev);
		found = 1;
	}

	if (!found && !mdio_node) {
		dev_warn(dev, "No PHY found\n");
		mdiobus_unregister(new_bus);
		mdiobus_free(new_bus);
		return -ENODEV;
	}

bus_register_done:
	priv->mii = new_bus;

	return 0;

bus_register_fail:
	mdiobus_free(new_bus);
	return err;
}

/**
 * bstgmac_mdio_unregister
 * @ndev: net device structure
 * Description: it unregisters the MII bus
 */
int bstgmac_mdio_unregister(struct net_device *ndev)
{
	struct bstgmac_priv *priv = netdev_priv(ndev);

	if (!priv->mii)
		return 0;

	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	mdiobus_free(priv->mii);
	priv->mii = NULL;

	return 0;
}

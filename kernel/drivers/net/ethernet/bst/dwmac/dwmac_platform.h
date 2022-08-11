/*******************************************************************************
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

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#ifndef __BSTGMAC_PLATFORM_H__
#define __BSTGMAC_PLATFORM_H__

#include "bstgmac.h"

struct plat_stmmacenet_data *
bstgmac_probe_config_dt(struct platform_device *pdev, const char **mac);
void bstgmac_remove_config_dt(struct platform_device *pdev,
			     struct plat_stmmacenet_data *plat);

int bstgmac_get_platform_resources(struct platform_device *pdev,
				  struct bstgmac_resources *bstgmac_res);

int bstgmac_pltfr_remove(struct platform_device *pdev);
extern const struct dev_pm_ops bstgmac_pltfr_pm_ops;

static inline void *get_bstgmac_bsp_priv(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstgmac_priv *priv = netdev_priv(ndev);

	return priv->plat->bsp_priv;
}

#endif /* __BSTGMAC_PLATFORM_H__ */

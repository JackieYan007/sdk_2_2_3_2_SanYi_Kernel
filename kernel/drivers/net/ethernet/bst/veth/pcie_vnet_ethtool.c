#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/net_tstamp.h>
#include <asm/io.h>

#include "pcievnet.h"

#define REG_SPACE_SIZE	0x1060
#define PCIEVNET_ETHTOOL_NAME	"st_gmac"
#define DRV_MODULE_VERSION	"Jan_2016"

struct pcievnet_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define PCIEVNET_STAT(m)	\
	{ #m, sizeof_field(struct pcie_vnet_extra_stats, m),	\
	offsetof(struct pcie_vnet_priv, xstats.m)}

static const struct pcievnet_stats pcievnet_gstrings_stats[] = {
	/* statu errors */
	PCIEVNET_STAT(link_not_ok),
	PCIEVNET_STAT(netif_not_ok),
	PCIEVNET_STAT(carrier_not_ok),
    PCIEVNET_STAT(intf_poll),
	
    /* tx error */
	PCIEVNET_STAT(xmit_cnt),
    PCIEVNET_STAT(txproc_poll),
	PCIEVNET_STAT(tx_clean),
    PCIEVNET_STAT(tx_alloc_skb_fail),
	PCIEVNET_STAT(tx_dma_map_fail),
    PCIEVNET_STAT(tx_busy),

	/* Receive errors */
    PCIEVNET_STAT(rxproc_poll),
    PCIEVNET_STAT(rxpicp_cnt),
    PCIEVNET_STAT(rxstack_cnt),
    PCIEVNET_STAT(rx_alloc_skb_fail),
	PCIEVNET_STAT(rx_dma_map_fail),
    PCIEVNET_STAT(rx_read_error),  
    PCIEVNET_STAT(rx_mem_poll),

};
#define PCIEVNET_STATS_LEN ARRAY_SIZE(pcievnet_gstrings_stats)

static u32 pcievnet_ethtool_getmsglevel(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void pcievnet_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	priv->msg_enable = level;

}

static int pcievnet_check_if_running(struct net_device *dev)
{
    if (!netif_running(dev))
		return -EBUSY;
       
	return 0;
}

static void pcievnet_ethtool_getdrvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, PCIEVNET_ETHTOOL_NAME, sizeof(info->driver));
    strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static void pcievnet_get_ethtool_stats(struct net_device *dev,
				 struct ethtool_stats *dummy, u64 *data)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	int i, j = 0;

	for (i = 0; i < PCIEVNET_STATS_LEN; i++) {
		char *p = (char *)priv + pcievnet_gstrings_stats[i].stat_offset;
		data[j++] = (pcievnet_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}
}

static int pcievnet_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return PCIEVNET_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static void pcievnet_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < PCIEVNET_STATS_LEN; i++) {
			memcpy(p, pcievnet_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		WARN_ON(1);
		break;
	}
}

static const struct ethtool_ops pcievnet_ethtool_ops = {
	.begin = pcievnet_check_if_running,
    .get_drvinfo = pcievnet_ethtool_getdrvinfo,
	.get_msglevel = pcievnet_ethtool_getmsglevel,
	.set_msglevel = pcievnet_ethtool_setmsglevel,
	.get_link = ethtool_op_get_link,
	.get_ethtool_stats = pcievnet_get_ethtool_stats,
    .get_sset_count	= pcievnet_get_sset_count,
	.get_strings = pcievnet_get_strings,
};

void bst_pcievnet_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &pcievnet_ethtool_ops;
}
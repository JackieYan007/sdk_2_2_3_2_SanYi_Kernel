#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#include <linux/net_tstamp.h>
#include <net/pkt_cls.h>
#include <linux/timekeeping.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/picp.h>
#include <linux/bst_boardconfig.h>
#include <linux/bst_pcie_drv.h>
#include <linux/pci.h>
#include "pcievnet.h"

#define PCIE_PCIP_TRANS 1
#define PCIE_MEMORY_RW  0

#define	DEFAULT_BUFSIZE	 1536//11264
static int buf_sz = DEFAULT_BUFSIZE;

#define TX_TIMEO	20000
static int watchdog = TX_TIMEO;
static int debug = -1;
static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
				      NETIF_MSG_LINK | NETIF_MSG_IFUP |
				      NETIF_MSG_IFDOWN | NETIF_MSG_TIMER);

#define	PCIEVNET_RX_COPYBREAK	256
#define	PCIEVNET_ALIGN(x)		__ALIGN_KERNEL(x, SMP_CACHE_BYTES)
#define PCIEVNET_COAL_TIMER(x) (jiffies + usecs_to_jiffies(x))

#define PCIEVNET_TX_THRESH	(DMA_TX_SIZE / 4)
#define PCIEVNET_RX_THRESH	(DMA_RX_SIZE / 4)

/* 0x4000
 * queue0 |----rx-info
          |----tx-info 
 */
 /* rc                             ep
 * rx 写buf/len 到0x8fd04000      rx 从0x48006000读源地址/len
 * tx 写buf/len 到0x8fd06000      tx 从0x48004000读目的地址
*/
#define PCIEVNET_REMOTE_BASE    (0x4000)
#define PCIEVNET_REMOTE_RXOFF   (0)
#define PCIEVNET_RADDR_SIZE     (DMA_RX_SIZE * sizeof(struct pcievnet_laddr_info))
#define PCIEVNET_REMOTE_TXOFF   (PCIEVNET_RADDR_SIZE)
#define PCIEVNET_REMOTE_SIZE    (2 * PCIEVNET_RADDR_SIZE)
#define RADDR_CAN_USE   (0xc & 0xf)
#define RADDR_NOT_USE   (0x3 & 0xf)

#define PCIEVENT_TRANS_TIMEOUT_MS (3000)

#define BST_PCIEVNET_UP   LOW_LEVEL
#define BST_PCIEVNET_DOWN   HIGH_LEVEL

static struct sk_buff_head pcievnet_delivery_skblist[PCIE_VNET_CORE_NUM * PCIE_VNET_RXCHAN_NUM];
static struct picp_private *trans_priv[PCIE_VNET_TXCHAN_NUM];
static int pcie_vnet_run = 0;
static int tx_clean_exec = 0;
static struct pcie_vnet_priv *pv_priv;
static struct task_handle *rx_task0 = NULL, *rx_task1 = NULL;
/**
 * bst_pcievnet_stop_all_queues - Stop all queues
 * @priv: driver private structure
 */
static void bst_pcievnet_stop_all_queues(struct pcie_vnet_priv *priv)
{
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < tx_queues_cnt; queue++)
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
}

/**
 * bst_pcievnet_start_all_queues - Start all queues
 * @priv: driver private structure
 */
static void bst_pcievnet_start_all_queues(struct pcie_vnet_priv *priv)
{
	u32 tx_queues_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < tx_queues_cnt; queue++)
		netif_tx_start_queue(netdev_get_tx_queue(priv->dev, queue));
}

static void bst_pcievnet_tx_timer_arm(struct pcie_vnet_priv *priv, u32 queue)
{
	struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];

	mod_timer(&tx_q->txtimer, PCIEVNET_COAL_TIMER(priv->tx_coal_timer));
}

/**
 * bst_pcievnet_tx_timer - mitigation sw timer for tx.
 * @data: data pointer
 * Description:
 * This is the timer handler to directly invoke the bst_pcievnet_tx_clean.
 */
static void bst_pcievnet_tx_timer(struct timer_list *t)
{
	struct pcie_vnet_tx_queue *tx_q = from_timer(tx_q, t, txtimer);
	struct pcie_vnet_priv *priv = tx_q->priv_data;
	struct pcie_vnet_channel *ch;

    priv->xstats.txproc_poll++;
	ch = &priv->tx_channel[tx_q->queue_index];
    if (!tx_clean_exec) {
        queue_work(priv->tx_wq, &ch->tx_work);
    }
}

static void bst_pcievnet_init_tx_coalesce(struct pcie_vnet_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	u32 chan;

	priv->tx_coal_frames = BST_PCIEVNET_TX_FRAMES;
	priv->tx_coal_timer = BST_PCIEVNET_COAL_TX_TIMER;

	for (chan = 0; chan < tx_channel_count; chan++) {
		struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[chan];

		timer_setup(&tx_q->txtimer, bst_pcievnet_tx_timer, 0);
	}
}

static void bst_pcievnet_display_ring(void *head, unsigned int size, bool rx)
{
	dma_addr_t *p = (dma_addr_t *)head;
	int i;

	pr_info("%s descriptor ring:\n", rx ? "RX" : "TX");

	for (i = 0; i < size; i++) {
		pr_info("%03d [0x%x]\n", i, (unsigned int)virt_to_phys(p));
		p++;
	}
}

static void bst_pcievnet_display_rx_rings(struct pcie_vnet_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	void *head_rx;
	u32 queue;

	/* Display RX rings */
	for (queue = 0; queue < rx_cnt; queue++) {
		struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];

		pr_info("\tRX Queue %u rings\n", queue);

		head_rx = (void *)rx_q->rx_skbuff_dma;

		/* Display RX ring */
		bst_pcievnet_display_ring(head_rx, DMA_RX_SIZE, true);
	}
}

static void bst_pcievnet_display_tx_rings(struct pcie_vnet_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	void *head_tx;
	u32 queue;

	/* Display TX rings */
	for (queue = 0; queue < tx_cnt; queue++) {
		struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];

		pr_info("\tTX Queue %d rings\n", queue);

		head_tx = (void *)tx_q->tx_skbuff_dma;

	    bst_pcievnet_display_ring(head_tx, DMA_TX_SIZE, false);
	}
}

static void bst_pcievnet_display_rings(struct pcie_vnet_priv *priv)
{
	/* Display RX ring */
	bst_pcievnet_display_rx_rings(priv);

	/* Display TX ring */
	bst_pcievnet_display_tx_rings(priv);
}

/**
 * bst_pcievnet_init_rx_buffers - init the RX descriptor buffer.
 * @priv: driver private structure
 * @p: descriptor pointer
 * @i: descriptor index
 * @flags: gfp flag
 * @queue: RX queue index
 * Description: this function is called to allocate a receive buffer, perform
 * the DMA mapping and init the descriptor.
 */
static int bst_pcievnet_init_rx_buffers(struct pcie_vnet_priv *priv, int i, gfp_t flags, u32 queue)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
    struct sk_buff *skb;

	skb = __netdev_alloc_skb_ip_align(priv->dev, priv->dma_buf_sz, flags);
	if (!skb) {
		netdev_err(priv->dev,
			   "%s: Rx init fails; skb is NULL\n", __func__);
		return -ENOMEM;
	}
	rx_q->rx_skbuff[i] = skb;
	rx_q->rx_skbuff_dma[i] = dma_map_single(priv->device, skb->data,
						priv->dma_buf_sz,
						DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->device, rx_q->rx_skbuff_dma[i])) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}
#if PCIE_MEMORY_RW
    struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)rx_q->laddr_info_va;

    laddr_info[i].u.s.magic = RADDR_CAN_USE;
    laddr_info[i].u.s.len = priv->dma_buf_sz;
    laddr_info[i].u.s.buf = rx_q->rx_skbuff_dma[i];
#endif	
    return 0;
}

/**
 * bst_pcievnet_free_rx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bst_pcievnet_free_rx_buffer(struct pcie_vnet_priv *priv, u32 queue, int i)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];

	if (rx_q->rx_skbuff[i]) {
		dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[i],
				 priv->dma_buf_sz, DMA_FROM_DEVICE);
		dev_kfree_skb_any(rx_q->rx_skbuff[i]);
	}
	rx_q->rx_skbuff[i] = NULL;
}

/**
 * bst_pcievnet_free_tx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bst_pcievnet_free_tx_buffer(struct pcie_vnet_priv *priv, u32 queue, int i)
{
	struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];

	if (tx_q->tx_skbuff_dma[i].buf) {
		if (tx_q->tx_skbuff_dma[i].map_as_page)
			dma_unmap_page(priv->device,
				       tx_q->tx_skbuff_dma[i].buf,
				       tx_q->tx_skbuff_dma[i].len,
				       DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device,
					 tx_q->tx_skbuff_dma[i].buf,
					 tx_q->tx_skbuff_dma[i].len,
					 DMA_TO_DEVICE);
	}

	if (tx_q->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_q->tx_skbuff[i]);
		tx_q->tx_skbuff[i] = NULL;
		tx_q->tx_skbuff_dma[i].buf = 0;
		tx_q->tx_skbuff_dma[i].map_as_page = false;
	}
}

static int bst_pcievnet_set_bfsize(int mtu, int bufsize)
{
	int ret = bufsize;

	if (mtu >= BUF_SIZE_8KiB)
        ret = BUF_SIZE_16KiB;
    else if (mtu >= BUF_SIZE_4KiB)
		ret = BUF_SIZE_8KiB;
	else if (mtu >= BUF_SIZE_2KiB)
		ret = BUF_SIZE_4KiB;
	else if (mtu > DEFAULT_BUFSIZE)
		ret = BUF_SIZE_2KiB;
	else
		ret = DEFAULT_BUFSIZE;

	return ret;
}

/**
 * init_dma_rx_desc_rings - init the RX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_rx_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	int bfsize = 0;
	int queue;
	int i;

	bfsize = bst_pcievnet_set_bfsize(dev->mtu, priv->dma_buf_sz);
	priv->dma_buf_sz = bfsize;

	/* RX INITIALIZATION */
	netif_dbg(priv, probe, priv->dev,
		  "SKB addresses:\nskb\t\tskb data\tdma data\n");

	for (queue = 0; queue < rx_count; queue++) {
		struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
        for (i = 0; i < DMA_RX_SIZE; i++) {
			ret = bst_pcievnet_init_rx_buffers(priv, i, flags,
						     queue);
			if (ret)
				goto err_init_rx_buffers;

			netif_dbg(priv, probe, priv->dev, "[%p]\t[%p]\t[%x]\n",
				  rx_q->rx_skbuff[i], rx_q->rx_skbuff[i]->data,
				  (unsigned int)rx_q->rx_skbuff_dma[i]);
		}
		rx_q->cur_rx = 0;
		rx_q->dirty_rx = (unsigned int)(i - DMA_RX_SIZE);
	}

	buf_sz = bfsize;

	return 0;

err_init_rx_buffers:
	while (queue >= 0) {
		while (--i >= 0)
			bst_pcievnet_free_rx_buffer(priv, queue, i);

		if (queue == 0)
			break;

		i = DMA_RX_SIZE;
		queue--;
	}

	return ret;
}

/**
 * init_dma_tx_desc_rings - init the TX descriptor rings
 * @dev: net device structure.
 * Description: this function initializes the DMA TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_tx_desc_rings(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	int i;

	for (queue = 0; queue < tx_queue_cnt; queue++) {
		struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];
#if PCIE_MEMORY_RW       
        struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)tx_q->laddr_info_va;
#endif
		for (i = 0; i < DMA_TX_SIZE; i++) {
			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
			tx_q->tx_skbuff_dma[i].len = 0;
			tx_q->tx_skbuff_dma[i].last_segment = false;
			tx_q->tx_skbuff[i] = NULL;
#if PCIE_MEMORY_RW
            laddr_info[i].u.s.magic = RADDR_NOT_USE;
#endif
		}

		tx_q->dirty_tx = 0;
		tx_q->cur_tx = 0;
        spin_lock_init(&tx_q->que_lock);
		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	return 0;
}

/**
 * init_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	int ret;

	ret = init_dma_rx_desc_rings(dev, flags);
	if (ret)
		return ret;

	ret = init_dma_tx_desc_rings(dev);

	if (netif_msg_hw(priv))
		bst_pcievnet_display_rings(priv);

	return ret;
}

/**
 * dma_free_rx_skbufs - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 */
static void dma_free_rx_skbufs(struct pcie_vnet_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_RX_SIZE; i++)
		bst_pcievnet_free_rx_buffer(priv, queue, i);
}

/**
 * dma_free_tx_skbufs - free TX dma buffers
 * @priv: private structure
 * @queue: TX queue index
 */
static void dma_free_tx_skbufs(struct pcie_vnet_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_TX_SIZE; i++)
		bst_pcievnet_free_tx_buffer(priv, queue, i);
}

static void bst_pcievnet_free_rxmem(struct pcie_vnet_priv *priv)
{
    int chan, busid, index, i = 0;
    struct sk_buff_head *list;
    struct sk_buff *skb; 
    dma_addr_t *buf;

    busid = priv->id;
    for (chan = 0; chan < PCIE_VNET_RXCHAN_NUM; chan++) {
        index = busid * PCIE_VNET_RXCHAN_NUM + chan;
        list = &pcievnet_delivery_skblist[index];
        while (skb_queue_len(list)) {        
            skb = skb_dequeue(list);       
            if (skb) {
                buf = (dma_addr_t *)skb->cb;
                if (*buf) {
                    dma_unmap_single(priv->device, *buf, priv->dma_buf_sz, DMA_FROM_DEVICE);
                    *buf = 0;
                    dma_wmb();
		            dev_kfree_skb_any(skb);
                }
            }
            i++;
        }
    }
}

static void free_dma_rx_mem_res(struct pcie_vnet_priv *priv)
{
    int i;
    int it_ms = 10, cnt = (PCIEVENT_TRANS_TIMEOUT_MS+it_ms)/it_ms;

    i = 0;
    while((i < cnt) && (test_bit(PCIE_VNET_RXMEM_WORK_RUN, &priv->state))) {
        msleep(it_ms);
        i++;
    }
    if (i != cnt) {
        bst_pcievnet_free_rxmem(priv);
    } else {
        printk(KERN_ERR "An exception occurred, so rxmem buf was not released\n");
    }
}

/**
 * free_dma_rx_desc_resources - free RX dma desc resources
 * @priv: private structure
 */
static void free_dma_rx_desc_resources(struct pcie_vnet_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue, i;
    int it_ms = 10, cnt = (PCIEVENT_TRANS_TIMEOUT_MS+it_ms)/it_ms;

	/* Free RX queue resources */
	for (queue = 0; queue < rx_count; queue++) {
		struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
        i = 0;
        while((i < cnt) && (rx_q->run)) {
            msleep(it_ms);
            i++;
        }
        if (i != cnt) {
            /* Release the DMA RX socket buffers */
            dma_free_rx_skbufs(priv, queue);

            kfree(rx_q->rx_skbuff_dma);
            kfree(rx_q->rx_skbuff);
        } else {
            printk(KERN_ERR "An exception occurred, so rxbuf was not released\n");
        }
	}
}

/**
 * free_dma_tx_desc_resources - free TX dma desc resources
 * @priv: private structure
 */
static void free_dma_tx_desc_resources(struct pcie_vnet_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue, i;
	int it_ms = 10, cnt = (PCIEVENT_TRANS_TIMEOUT_MS+it_ms)/it_ms;

	/* Free TX queue resources */
	for (queue = 0; queue < tx_count; queue++) {
		struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];
        i = 0;
		while((i < cnt) && (tx_q->clean_run)) {
			msleep(it_ms);
			i++;
		}
		if (i != cnt) {
			/* Release the DMA TX socket buffers */
			dma_free_tx_skbufs(priv, queue);

			kfree(tx_q->tx_skbuff_dma);
			kfree(tx_q->tx_skbuff);
		} else {
			printk(KERN_ERR "An exception occurred, so txbuf was not released\n");
		}
	}
}

/**
 * free_dma_desc_resources - free dma desc resources
 * @priv: private structure
 */
static void free_dma_desc_resources(struct pcie_vnet_priv *priv)
{
	/* Release the DMA RX socket buffers */
	free_dma_rx_desc_resources(priv);

	/* Release the DMA TX socket buffers */
	free_dma_tx_desc_resources(priv);
}

/**
 * alloc_dma_rx_desc_resources - alloc RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_rx_desc_resources(struct pcie_vnet_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* RX queues buffers and DMA */
	for (queue = 0; queue < rx_count; queue++) {
		struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->queue_index = queue;
		rx_q->priv_data = priv;

		rx_q->rx_skbuff_dma = kmalloc_array(DMA_RX_SIZE,
						    sizeof(dma_addr_t),
						    GFP_KERNEL);
		if (!rx_q->rx_skbuff_dma)
			goto err_dma;

		rx_q->rx_skbuff = kmalloc_array(DMA_RX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!rx_q->rx_skbuff)
			goto err_dma;
	}

	return 0;

err_dma:
	free_dma_rx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_tx_desc_resources - alloc TX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_tx_desc_resources(struct pcie_vnet_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* TX queues buffers and DMA */
	for (queue = 0; queue < tx_count; queue++) {
		struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];

		tx_q->queue_index = queue;
		tx_q->priv_data = priv;

		tx_q->tx_skbuff_dma = kmalloc_array(DMA_TX_SIZE,
						    sizeof(*tx_q->tx_skbuff_dma),
						    GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma)
			goto err_dma;

		tx_q->tx_skbuff = kmalloc_array(DMA_TX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!tx_q->tx_skbuff)
			goto err_dma;
	}

	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_desc_resources - alloc TX/RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_desc_resources(struct pcie_vnet_priv *priv)
{
	/* RX Allocation */
	int ret = alloc_dma_rx_desc_resources(priv);

	if (ret)
		return ret;

	ret = alloc_dma_tx_desc_resources(priv);

	return ret;
}

static void bst_pcievnet_init_rxmem(struct pcie_vnet_priv *priv, int id)
{
    int i, chan, index;
    struct sk_buff *skb; 
    dma_addr_t buf;
    int size = priv->dma_buf_sz;
    struct sk_buff_head *list;
    dma_addr_t *addr;

    if (id > PCIE_VNET_CORE_NUM) {
        return;
    }
    
    for (chan = 0; chan < PCIE_VNET_RXCHAN_NUM; chan++) {
        index = id * PCIE_VNET_RXCHAN_NUM + chan;
        list = &pcievnet_delivery_skblist[index];
        for (i = 0; i < PCIE_VNET_RXMEM_MAX; i++) {
            skb = netdev_alloc_skb_ip_align(priv->dev, size);
            if (unlikely(!skb)) {
                break;
            }
            buf = dma_map_single(priv->device, skb->data, size, DMA_FROM_DEVICE);
            if (dma_mapping_error(priv->device, buf)) {
                netdev_err(priv->dev, "Gmac map list failed\n");
                dev_kfree_skb(skb);
                break;
            }         
            addr = (dma_addr_t *)skb->cb;
            *addr = buf;                     
            dma_wmb();
            skb_queue_tail(list, skb);
        }    
    }
}

static void bst_pcievnet_up_state(struct pcie_vnet_priv *priv)
{
    bst_init_heatbeat();
    bst_set_heartbeat_infad(BST_PCIEVNET_UP);
    set_bit(PCIE_VNET_UP, &priv->state);
    clear_bit(PCIE_VNET_DOWN, &priv->state);
}

static void bst_pcievnet_down_state(struct pcie_vnet_priv *priv)
{
    bst_set_heartbeat_infad(BST_PCIEVNET_DOWN);
    set_bit(PCIE_VNET_DOWN, &priv->state);
    clear_bit(PCIE_VNET_UP, &priv->state);
}

static bool bst_pcievnet_state_is_ok(struct pcie_vnet_priv *priv)
{
    int rstate;

	if (!bst_pcie_link_is_ok()) { //pcie driver 
        return false;
    }

	if (priv->extend_op == BSTA1000B_BOARD_EVB) {
		struct pci_dev *pci_root_dev = NULL;

		pci_root_dev = pci_get_class(PCI_CLASS_STORAGE_EXPRESS, NULL);//pci_get_device(0x16C3, 0xABCD, NULL);
		if (pci_root_dev) {
			return false;
		}
	}
    rstate = bst_get_other_soc_state();
    if (test_bit(PCIE_VNET_UP, &priv->state)) { //pcie network
        if (rstate == 1) {
            return true;
        }
    }

    return false;  
}

/**
 *  bst_pcievnet_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bst_pcievnet_open(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	int ret;

	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct pcie_vnet_extra_stats));

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

    ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}

    bst_pcievnet_init_rxmem(priv, priv->id);

	bst_pcievnet_init_tx_coalesce(priv);	
	
	bst_pcievnet_start_all_queues(priv);
    
    bst_pcievnet_up_state(priv);

    queue_work(priv->wq, &priv->intf_task);
    
	return 0;

init_error:
	free_dma_desc_resources(priv);

dma_desc_error:
	return ret;
}

static void bst_pcievnet_free_res(struct pcie_vnet_priv *priv)
{
    u32 chan;

    bst_pcievnet_stop_all_queues(priv);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		del_timer_sync(&priv->tx_queue[chan].txtimer);

	/* Release and free the Rx/Tx resources */
	free_dma_desc_resources(priv);

    free_dma_rx_mem_res(priv);
}

/**
 *  bst_pcievnet_release - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int bst_pcievnet_release(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);	
	struct pcievnet_laddr_info raddr_info; 
    u32 base;
  	if ((priv->extend_op == BSTA1000_BOARD_FADA) || (priv->extend_op == BSTA1000_BOARD_FADB)) {    
		base = PCIEVNET_REMOTE_BASE;
		raddr_info.u.flag = RADDR_NOT_USE;
		bst_pcie_write_outmem(base, raddr_info.u.flag);
	}

    netif_carrier_off(dev);

    bst_pcievnet_down_state(priv);

	bst_pcievnet_free_res(priv);

	return 0;
}

static void bst_pcievnet_intf_task(struct work_struct *work)
{
	struct pcie_vnet_priv *priv = container_of(work, struct pcie_vnet_priv, intf_task);
    
    priv->xstats.intf_poll++;

    if (bst_pcievnet_state_is_ok(priv)) {
        netif_carrier_on(priv->dev);
        if (pcie_vnet_run == 0) {
            printk(KERN_INFO "pcie vnet interface up\n"); //print only once
        }
        pcie_vnet_run = 1;
    } else {
        netif_carrier_off(priv->dev);
        if (pcie_vnet_run == 1) {
            printk(KERN_INFO "pcie vnet interface down\n"); //print only once
        }
        pcie_vnet_run = 0;  
    }

    if (test_bit(PCIE_VNET_DOWN, &priv->state)) {
        return;
    }

    msleep(1000);
    queue_work(priv->wq, &priv->intf_task);
}

static void bst_pcievnet_check_ether_addr(struct pcie_vnet_priv *priv)
{
	if (!is_valid_ether_addr(priv->dev->dev_addr)) {
        eth_hw_addr_random(priv->dev);    	
		netdev_info(priv->dev, "device MAC address %pM\n",
			    priv->dev->dev_addr);
	}
}

/**
 * bst_pcievnet_rx_dirty - Get RX queue dirty
 * @priv: driver private structure
 * @queue: RX queue index
 */
static inline u32 bst_pcievnet_rx_dirty(struct pcie_vnet_priv *priv, u32 queue)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
	u32 dirty;

	if (rx_q->dirty_rx <= rx_q->cur_rx)
		dirty = rx_q->cur_rx - rx_q->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_q->dirty_rx + rx_q->cur_rx;

	return dirty;
}

/**
 * bstgmac_rx_refill - refill used skb preallocated buffers
 * @priv: driver private structure
 * @queue: RX queue index
 * Description : this is to reallocate the skb for the reception process
 * that is based on zero-copy.
 */
static inline void bst_pcievnet_rx_refill(struct pcie_vnet_priv *priv, u32 queue)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
	int dirty = bst_pcievnet_rx_dirty(priv, queue);
	unsigned int entry = rx_q->dirty_rx;
	int bfsize = priv->dma_buf_sz;
    int bus_id, cpuid;
    int index;
    struct sk_buff_head *list;
    struct sk_buff *skb;
    dma_addr_t buf;
    dma_addr_t *addr;
#if PCIE_MEMORY_RW
    struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)rx_q->laddr_info_va;
#endif
    bus_id = priv->id;
    index = bus_id * PCIE_VNET_RXCHAN_NUM + queue;
    list = &pcievnet_delivery_skblist[index];
	while (dirty-- > 0) {
		if (likely(!rx_q->rx_skbuff[entry])) {
            skb = skb_dequeue(list);       
            if (unlikely(!skb)) {         
                skb = netdev_alloc_skb_ip_align(priv->dev, bfsize);
                if (unlikely(!skb)) {
                    priv->xstats.rx_alloc_skb_fail++;
                    if (unlikely(net_ratelimit()))
                        dev_err(priv->device, "fail to alloc skb entry %d\n", entry);               
                    break;
                }
                buf = dma_map_single(priv->device, skb->data, bfsize, DMA_FROM_DEVICE);
                if (dma_mapping_error(priv->device, buf)) {
                    priv->xstats.rx_dma_map_fail++;
                    netdev_err(priv->dev, "Rx DMA map failed\n");
                    dev_kfree_skb(skb);
                    break;
                }     
            } else {
                addr = (dma_addr_t *)skb->cb;
                buf = *addr;
                *addr = 0;
                dma_wmb();
            }

            rx_q->rx_skbuff[entry] = skb;
            rx_q->rx_skbuff_dma[entry] = buf;
#if PCIE_MEMORY_RW
            laddr_info[entry].u.s.magic = RADDR_CAN_USE;
            laddr_info[entry].u.s.len = bfsize;
            laddr_info[entry].u.s.buf = buf;
#endif
		}

		dma_wmb();

		entry = PCIE_VNET_GET_ENTRY(entry, DMA_RX_SIZE);
	}
	rx_q->dirty_rx = entry;

    if (skb_queue_len(list) < PCIE_VNET_RXMEM_THRE) {
        cpuid = (bus_id == PCIE_VNET_DMA_CHAN0) ? PCIE_VNET_DMA_CHAN1 : PCIE_VNET_DMA_CHAN0;
        queue_work_on(cpuid, priv->rxmem, &priv->rxmem_work); //eth0:cpu1 eth1:cpu0
    }
}

static void print_pkt(unsigned char *buf, int len)
{
    print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1,
		       buf, len, true);
}
#if PCIE_MEMORY_RW
static int bst_pcievnet_ep_read_txinfo(struct pcie_vnet_priv *priv, int entry, struct pcievnet_trans *dest_msg)
{ 
    struct pcievnet_laddr_info raddr_info; 
    u32 info_size;
    u32 base, flag_off, ret, queue = dest_msg->queue;
   
    info_size = sizeof(struct pcievnet_laddr_info);
    base = PCIEVNET_REMOTE_BASE+PCIEVNET_REMOTE_TXOFF+queue*PCIEVNET_REMOTE_SIZE;

    flag_off = base + (entry * info_size);
    
    bst_pcie_read_outmem(flag_off, &raddr_info.u.flag);

    if (raddr_info.u.s.magic == RADDR_CAN_USE) {
        ret = ep_pcie_dma_read(queue, raddr_info.u.s.len, raddr_info.u.s.buf, dest_msg->buf);

        raddr_info.u.s.magic = RADDR_NOT_USE;
        bst_pcie_write_outmem(flag_off, raddr_info.u.flag);
        if (!ret) {
            return raddr_info.u.s.len;
        } else {
            return -1;
        }
    } else {
        return 0;
    }

    return -1;
}
#endif

static bool bst_pcievnet_can_recvdata(struct pcie_vnet_priv *priv, int entry, u32 queue)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
	struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)rx_q->laddr_info_va;
	
	if ((priv->extend_op != BSTA1000_BOARD_FADA) && (priv->extend_op != BSTA1000_BOARD_FADB)) {
		return true;
	}

	return (laddr_info[entry].u.s.magic == RADDR_CAN_USE);
}

/**
 * bst_pcievnet_rx_func - manage the receive process
 * @priv: driver private structure
 * @limit: napi bugget
 * @queue: RX queue index.
 * Description :  this the function called by the napi poll method.
 * It gets all the frames inside the ring.
 */
int bst_pcievnet_rx_func(struct pcie_vnet_priv *priv, int limit, u32 queue)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];    
	unsigned int next_entry = rx_q->cur_rx;
	unsigned int count = 0;
    uint role = 0;
    int frame_len, proc_cnt = limit;
#if PCIE_MEMORY_RW
    struct pcievnet_trans dest_msg;
    int tmp_off = next_entry;
    uint i;
    struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)rx_q->laddr_info_va;
    int temp_len[DMA_RX_SIZE] = {0};
#endif
    struct sk_buff *skb;
	
	if (!bst_pcievnet_can_recvdata(priv, 0, queue)) {
		return 0;
	}
    rx_q->run = 1;
    if (bst_pcie_get_role() == DW_PCIE_EP_TYPE) {
        role = 1;
    }
#if PCIE_MEMORY_RW   
    if (bst_pcie_get_role() == DW_PCIE_EP_TYPE) {
        role = 1;
        while(count < proc_cnt) {
            dest_msg.buf = rx_q->rx_skbuff_dma[tmp_off];
            skb = rx_q->rx_skbuff[tmp_off];
        
            if (unlikely(!dest_msg.buf) || unlikely(!skb)) {
                break;
            }
            dest_msg.len = priv->dma_buf_sz;
            dest_msg.queue = queue;
            frame_len = bst_pcievnet_ep_read_txinfo(priv, tmp_off, &dest_msg);
            if (frame_len == 0) {
                break;
            } else if (frame_len < 0) {           
                printk(KERN_ERR "%s line %d  cur_rx %d p 0x%llx bufsz %d que %d frame_len %d\n",
                    __func__, __LINE__,  tmp_off, dest_msg.buf, priv->dma_buf_sz, queue, frame_len);
                priv->xstats.rx_read_error++;            
                break;
            }
            
            tmp_off = PCIE_VNET_GET_ENTRY(tmp_off, DMA_RX_SIZE);
            temp_len[tmp_off] = frame_len;
            skb_put(skb, frame_len);
            count++;
        }
        proc_cnt = count;
    }
#endif

    count = 0;
	while (count < proc_cnt) {
		int entry;
		dma_addr_t p;
		
		entry = next_entry;	
		if (test_bit(PCIE_VNET_DOWN, &priv->state)) {
			break;
		}

		p = rx_q->rx_skbuff_dma[entry];
        skb = rx_q->rx_skbuff[entry];
        
        if (unlikely(!p) || unlikely(!skb)) {
            if (net_ratelimit())
                netdev_err(priv->dev,"%s: Inconsistent Rx chain entry %d dma[0x%llx] skb[0x%llx] \n", priv->dev->name, entry, p, (u64)skb);
            priv->dev->stats.rx_dropped++;
            break;
        }
#if PCIE_PCIP_TRANS
        priv->xstats.rxpicp_cnt++;
        frame_len = picp_read(trans_priv[queue], p, priv->dma_buf_sz, PICP_DTYP_DATA);
        if (frame_len < 0) {
            if (frame_len != PCIE_ERR_TIMEOUT) {
                priv->xstats.rx_read_error++;
            }
            break;
        }
#else
        if (!role) {//rc
            i = 300;
            while(i) {
                i--;
                dma_rmb();
                if ((laddr_info[entry].u.s.magic == RADDR_CAN_USE)) { //there is no data in buf (laddr_info[entry].u.s.magic != 0) && 
                    usleep_range(5, 10);
                    continue;
                } else {
                    frame_len = laddr_info[entry].u.s.len;
                    break;
                } 
            }
            if (i == 0) {
                break;
            }
        } else {
            frame_len = temp_len[entry];
        }
#endif
        rx_q->cur_rx = PCIE_VNET_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
		next_entry = rx_q->cur_rx;
        
        if (frame_len > priv->dma_buf_sz) {
            if (net_ratelimit())
                netdev_err(priv->dev, "len %d larger than size (%d)\n", frame_len, priv->dma_buf_sz);
            dev_kfree_skb_any(rx_q->rx_skbuff[entry]);
            rx_q->rx_skbuff[entry] = NULL;
            dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);
            priv->dev->stats.rx_length_errors++;
            continue;
        }

        prefetch(skb->data - NET_IP_ALIGN);
        rx_q->rx_skbuff[entry] = NULL;

        skb_put(skb, frame_len);
        dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);

        skb->protocol = eth_type_trans(skb, priv->dev);
        skb->ip_summed = CHECKSUM_UNNECESSARY;

        if (netif_msg_pktdata(priv)) {
            printk(KERN_EMERG ">>> frame to be recevied: buf 0x%p frame_len %d", skb->data, frame_len);
            print_pkt(skb->data, frame_len);
        }
        priv->xstats.rxstack_cnt++;
        //netif_receive_skb(skb);
        netif_rx(skb);
        priv->dev->stats.rx_packets++;
        priv->dev->stats.rx_bytes += frame_len;
		
		count++;
	}

    bst_pcievnet_rx_refill(priv, queue);

	rx_q->run = 0;
	return count;
}

static int bst_pcievnet_rx_thread(void *par)
{
	struct task_handle *rx_task = (struct task_handle *)par;
    struct pcie_vnet_priv *priv = rx_task->priv;
    int budget = DMA_RX_SIZE/2; //PCIE_VNET_RX_POLL_WEIGHT;
    int ret;
    
	while(!kthread_should_stop()) {
        /* After calling the open interface, netif_carrier_ok(priv->dev) will become 1, 
            but the interface is not really up yet */
        if (!netif_running(priv->dev)) {
            priv->xstats.netif_not_ok++; 
            msleep(1000);
            continue;
        }    
        if (!pcie_vnet_run) {
            priv->xstats.link_not_ok++;  
            msleep(1000);
            continue;
        }
        
        if (!netif_carrier_ok(priv->dev)) {
            priv->xstats.carrier_not_ok++;
            msleep(1000);
            continue;
        }
        priv->xstats.rxproc_poll++;
        ret = bst_pcievnet_rx_func(priv, budget, rx_task->id);
        if (ret) {
            usleep_range(5, 10);
        } else {
            usleep_range(50, 60);
        }		
	}

	return 0;
}

int bst_pcievnet_create_thread(struct pcie_vnet_priv *priv)
{
    rx_task0 = kmalloc(sizeof(struct task_handle), GFP_KERNEL);
    if (rx_task0 == NULL) {
        return 0;
    }

    rx_task0->priv = priv;
    rx_task0->id = 0;
    rx_task0->tsk = kthread_run(bst_pcievnet_rx_thread, rx_task0, "pcievnet_rx0");
	if (IS_ERR(rx_task0->tsk)) {
        printk(KERN_ERR "%s line %d\n", __func__, __LINE__);
        kfree(rx_task0);
		return PTR_ERR(rx_task0->tsk);
	}
    if (priv->plat->rx_queues_to_use == 2) {
        rx_task1 = kmalloc(sizeof(struct task_handle), GFP_KERNEL);
        if (rx_task1 == NULL) {
            return 0;
        }

        rx_task1->priv = priv;
        rx_task1->id = 1;
        rx_task1->tsk = kthread_run(bst_pcievnet_rx_thread, rx_task1, "pcievnet_rx1");
        if (IS_ERR(rx_task1->tsk)) {
            printk(KERN_ERR "%s line %d\n", __func__, __LINE__);
            kfree(rx_task1);
            return PTR_ERR(rx_task1->tsk);
        }   
    }
    return 0;
}

static inline u32 bst_pcievnet_tx_avail(struct pcie_vnet_priv *priv, u32 queue)
{
	struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 avail;

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx - 1;
	else
		avail = DMA_TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx - 1;

	return avail;
}

/**
 * bst_pcievnet_tx_clean - to manage the transmission completion
 * @priv: driver private structure
 * @queue: TX queue index
 * Description: it reclaims the transmit resources after transmission completes.
 */
static int bst_pcievnet_tx_clean(struct pcie_vnet_priv *priv, int budget, u32 queue)
{
	struct pcie_vnet_tx_queue *tx_q = &priv->tx_queue[queue];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry, cur_tx, count = 0;  

    uint i;
    struct pcievnet_laddr_info *laddr_info = (struct pcievnet_laddr_info *)tx_q->laddr_info_va;  
    struct pcievnet_trans src_msg;
    struct pcievnet_laddr_info laddr_ctrl;
	u32 base;

    int role = 0;

    tx_clean_exec = 1;
	priv->xstats.tx_clean++;
    tx_q->clean_run = 1;

	entry = tx_q->dirty_tx;
    cur_tx = tx_q->cur_tx;
    
    if (bst_pcie_get_role() == DW_PCIE_EP_TYPE) {
        role = 1;
    }
	if ((priv->extend_op == BSTA1000_BOARD_FADA) || (priv->extend_op == BSTA1000_BOARD_FADB)) {
		/* rc                             ep  
		* tx 写buf/len 到0x40004000     rx 从0x8fd04000读源地址/len
		*/
		laddr_ctrl.u.flag = 0;
		laddr_ctrl.u.s.magic = RADDR_CAN_USE;
		base = PCIEVNET_REMOTE_BASE+queue*PCIEVNET_REMOTE_SIZE;
		bst_pcie_write_outmem(base, laddr_ctrl.u.flag);
	}
	while ((entry != tx_q->cur_tx) && (count < budget)) {
		struct sk_buff *skb = tx_q->tx_skbuff[entry];
        dma_addr_t addr;
        int len, ret;
        if (test_bit(PCIE_VNET_DOWN, &priv->state)) {
            goto Exit;
        }
        addr = tx_q->tx_skbuff_dma[entry].buf;     
        len = tx_q->tx_skbuff_dma[entry].len;
        if ((addr == NULL) || (len == 0)) {
			pr_err("%s line %d addr 0x%x len %d\n", addr, len);
            break;
        }

#if PCIE_PCIP_TRANS                
        ret = picp_write(trans_priv[queue], addr, len, PICP_DTYP_DATA);
		if (ret > 0) {
			priv->dev->stats.tx_packets++;
        } else {
            priv->dev->stats.tx_errors++;
        }
#else
        if (!role) { //rc     
            i = 300000;
            while(i) {
                i--;
                dma_rmb();
                if (laddr_info[entry].u.s.magic == RADDR_NOT_USE) {
                    break;
                }
                usleep_range(5, 10);
                
            }
            if (i != 0) {
                priv->dev->stats.tx_packets++;
            } else {
                printk(KERN_ERR "%s line %d queue %d addr 0x%llx len %d entry %d flag 0x%llx, cur_tx %d count %d budget %d cpuid %d\n",
                __func__, __LINE__, queue, addr, len, entry, laddr_info[entry].u.flag, tx_q->cur_tx, count, budget, smp_processor_id());
                priv->dev->stats.tx_errors++;
            }
             
        } else {
            src_msg.buf = addr;
            src_msg.len = len;
            src_msg.queue = queue;
            i = 60000;
            while(i) {
                i--;
            
                ret = bst_pcievnet_ep_write_rxinfo(priv, entry, &src_msg);
                if (ret) {
                    break;
                } else {         
                    usleep_range(30, 40);
                }
            }
            if ((ret == 1) && (i != 0)) {
                priv->dev->stats.tx_packets++;
            } else {
                printk(KERN_ERR "%s line %d queue %d addr 0x%llx len %d entry %d cur_tx %d count %d budget %d cpuid %d \n",
                __func__, __LINE__, queue, addr, len, entry, tx_q->cur_tx, count, budget, smp_processor_id());
                priv->dev->stats.tx_errors++;
            }
        }
#endif
		count++;

		if (likely(addr)) {
			if (tx_q->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device,
					       tx_q->tx_skbuff_dma[entry].buf,
					       tx_q->tx_skbuff_dma[entry].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device,
						 tx_q->tx_skbuff_dma[entry].buf,
						 tx_q->tx_skbuff_dma[entry].len,
						 DMA_TO_DEVICE);
			tx_q->tx_skbuff_dma[entry].buf = 0;
			tx_q->tx_skbuff_dma[entry].len = 0;
			tx_q->tx_skbuff_dma[entry].map_as_page = false;
		}
		tx_q->tx_skbuff_dma[entry].last_segment = false;
		tx_q->tx_skbuff_dma[entry].is_jumbo = false;
        dma_wmb();

		if (likely(skb != NULL)) {
			pkts_compl++;
			bytes_compl += skb->len;
			dev_consume_skb_any(skb);
			tx_q->tx_skbuff[entry] = NULL;
		}

		entry = PCIE_VNET_GET_ENTRY(entry, DMA_TX_SIZE);
    }
	tx_q->dirty_tx = entry;
    
	netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, queue), pkts_compl, bytes_compl);
    
	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,
								queue))) &&
	    bst_pcievnet_tx_avail(priv, queue) > PCIEVNET_TX_THRESH) {

		netif_dbg(priv, tx_done, priv->dev,
			  "%s: restart transmit\n", __func__);
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, queue));
	}
Exit:
    tx_clean_exec = 0;
    tx_q->clean_run = 0;
	if ((priv->extend_op == BSTA1000_BOARD_FADA) || (priv->extend_op == BSTA1000_BOARD_FADB)) {
		laddr_ctrl.u.flag = 0;
		laddr_ctrl.u.s.magic = RADDR_NOT_USE;
		bst_pcie_write_outmem(base, laddr_ctrl.u.flag);
	}
   	return count;
}

static void bst_pcievnet_tx_thread(struct work_struct *tx_work)
{
	struct pcie_vnet_channel *ch = container_of(tx_work, struct pcie_vnet_channel, tx_work);
	struct pcie_vnet_priv *priv = ch->priv_data;
	int work_done, tx_done = 0;
	u32 chan = ch->index;
    int budget = DMA_RX_SIZE/2;//PCIE_VNET_RX_POLL_WEIGHT;

	priv->xstats.txproc_poll++;

	tx_done = bst_pcievnet_tx_clean(priv, budget, chan);

	work_done = min(tx_done, budget);
	if (work_done == budget) {
        queue_work(priv->tx_wq, &ch->tx_work);
	}
}

/**
 *  bst_pcievnet_xmit - Tx entry point of the driver
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description : this is the tx entry point of the driver.
 *  It programs the chain or the ring and supports oversized frames
 *  and SG feature.
 */
static netdev_tx_t bst_pcievnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i;
	u32 queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry;
	unsigned int first_entry;
	struct pcie_vnet_tx_queue *tx_q;
	dma_addr_t des;

    if (!pcie_vnet_run) {
		return NETDEV_TX_BUSY;
    }

    priv->xstats.xmit_cnt++;
	if (unlikely(bst_pcievnet_tx_avail(priv, queue) < nfrags + 1)) {
        priv->xstats.tx_busy++;
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

    tx_q = &priv->tx_queue[queue]; 
	entry = tx_q->cur_tx;

    first_entry = entry;
	WARN_ON(tx_q->tx_skbuff[first_entry]);

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = PCIE_VNET_GET_ENTRY(entry, DMA_TX_SIZE);
		WARN_ON(tx_q->tx_skbuff[entry]);

		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des)) {
             priv->xstats.tx_dma_map_fail++;
        	goto dma_map_err; /* should reuse desc w/o issues */
        }
		tx_q->tx_skbuff_dma[entry].buf = des;

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;

        dma_wmb();
	}
    
    /* because tx_skbuff has been updated in thread tx_clean, bytes_compl will plus an additional skb->len 
       which has not been added to dql_queue. And then trigger the BUG_ON(bytes_compl > num_queued - dql->num_completed). 
    */
    netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);
    
	if (netif_msg_tx_queued(priv)) {//   skb->len > dev->mtu
	    printk(KERN_EMERG "%s: curr=%d dirty=%d e=%d, nfrags=%d",
			   __func__, tx_q->cur_tx, tx_q->dirty_tx, entry, nfrags);

		printk(KERN_EMERG ">>> frame to be transmitted: len %d datalen %d\n", skb->len, skb->data_len);
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(bst_pcievnet_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
  		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}
    
	dev->stats.tx_bytes += skb->len;

	skb_tx_timestamp(skb);

	if (likely(!nfrags)) {
		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des)) {
		    priv->xstats.tx_dma_map_fail++;
        	goto dma_map_err;
        }
       
		tx_q->tx_skbuff_dma[first_entry].buf = des;
		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = true;
    }
    tx_q->tx_skbuff[entry] = skb;

    if (likely(!nfrags)) {
        entry =  PCIE_VNET_GET_ENTRY(first_entry, DMA_TX_SIZE);         
    } else {
        entry = PCIE_VNET_GET_ENTRY(entry, DMA_TX_SIZE);
    }

    tx_q->cur_tx = entry;
    dma_wmb();

    bst_pcievnet_tx_timer_arm(priv, queue);
	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *bst_pcievnet_fs_dir;

static void sysfs_display_ring(struct pcie_vnet_priv *priv, int queue, struct seq_file *seq, int rx)
{
	struct pcie_vnet_rx_queue *rx_q = &priv->rx_queue[queue];
    int i;
	dma_addr_t *p = (dma_addr_t *)rx_q->rx_skbuff_dma;
    struct sk_buff **q = (struct sk_buff **)rx_q->rx_skbuff;

	for (i = 0; i < DMA_RX_SIZE; i++) {
        seq_printf(seq, "%d [dma:0x%llx][skb:0x%llx]\n", i, *p, (u64)q[i]);
        p++;
		seq_printf(seq, "\n");
	}
}

static int bst_pcievnet_sysfs_ring_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct pcie_vnet_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;

	if ((dev->flags & IFF_UP) == 0)
		return 0;

	for (queue = 0; queue < rx_count; queue++) {
		seq_printf(seq, "RX Queue %d:\n", queue);
		seq_printf(seq, "Descriptor ring:\n");
		sysfs_display_ring(priv, queue, seq, 1);		
	}

	return 0;
}

static int bst_pcievnet_sysfs_ring_open(struct inode *inode, struct file *file)
{
	return single_open(file, bst_pcievnet_sysfs_ring_read, inode->i_private);
}

/* Debugfs files, should appear in /sys/kernel/debug/bst_pcievneteth/eth0 */

static const struct file_operations bst_pcievnet_rings_status_fops = {
	.owner = THIS_MODULE,
	.open = bst_pcievnet_sysfs_ring_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bst_pcievnet_sysfs_dma_cap_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct pcie_vnet_priv *priv = netdev_priv(dev);
    int i;

    i = priv->id;
    seq_printf(seq, "delivery list%d len %d\n", i, skb_queue_len(&pcievnet_delivery_skblist[i]));
    seq_printf(seq, "delivery list%d len %d\n", i+PCIE_VNET_RXCHAN_NUM, skb_queue_len(&pcievnet_delivery_skblist[i+PCIE_VNET_RXCHAN_NUM]));
	seq_printf(seq, "pcie vnet state 0x%x\n", priv->state);
	return 0;
}

static int bst_pcievnet_sysfs_dma_cap_open(struct inode *inode, struct file *file)
{
	return single_open(file, bst_pcievnet_sysfs_dma_cap_read, inode->i_private);
}

static const struct file_operations bst_pcievnet_dma_cap_fops = {
	.owner = THIS_MODULE,
	.open = bst_pcievnet_sysfs_dma_cap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bst_pcievnet_init_fs(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);

	/* Create debugfs main directory if it doesn't exist yet */
	if (!bst_pcievnet_fs_dir) {
		bst_pcievnet_fs_dir = debugfs_create_dir(BST_PCIEVNET_NAME, NULL);

		if (!bst_pcievnet_fs_dir || IS_ERR(bst_pcievnet_fs_dir)) {
			printk(KERN_ERR "ERROR %s, debugfs create directory failed\n",
			       BST_PCIEVNET_NAME);

			return -ENOMEM;
		}
	}

	/* Create per netdev entries */
	priv->dbgfs_dir = debugfs_create_dir(dev->name, bst_pcievnet_fs_dir);

	if (!priv->dbgfs_dir || IS_ERR(priv->dbgfs_dir)) {
		netdev_err(priv->dev, "ERROR failed to create debugfs directory\n");

		return -ENOMEM;
	}

	/* Entry to report DMA RX/TX rings */
	priv->dbgfs_rings_status =
		debugfs_create_file("descriptors_status", 0444,
				    priv->dbgfs_dir, dev,
				    &bst_pcievnet_rings_status_fops);

	if (!priv->dbgfs_rings_status || IS_ERR(priv->dbgfs_rings_status)) {
		netdev_err(priv->dev, "ERROR creating bst_pcievnet ring debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

	/* Entry to report the DMA HW features */
	priv->dbgfs_dma_cap = debugfs_create_file("debug", 0444,
						  priv->dbgfs_dir,
						  dev, &bst_pcievnet_dma_cap_fops);

	if (!priv->dbgfs_dma_cap || IS_ERR(priv->dbgfs_dma_cap)) {
		netdev_err(priv->dev, "ERROR creating bst_pcievnet MMC debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);

		return -ENOMEM;
	}

    return 0;
}

static void bst_pcievnet_exit_fs(struct net_device *dev)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);

	debugfs_remove_recursive(priv->dbgfs_dir);

	debugfs_remove_recursive(bst_pcievnet_fs_dir);
}
#endif /* CONFIG_DEBUG_FS */

static void bst_pcievnet_refill_rxmem(struct pcie_vnet_priv *priv, int chan)
{
    int cnt, busid, index;
    struct sk_buff *skb; 
    dma_addr_t buf;
    int size = priv->dma_buf_sz;
    dma_addr_t *addr;
    struct sk_buff_head *list;

    busid = priv->id;
    index = busid * PCIE_VNET_RXCHAN_NUM + chan;
    list = &pcievnet_delivery_skblist[index];
    cnt = 0;

    while (cnt < (PCIE_VNET_RXMEM_THRE - 1)) {    
        skb = netdev_alloc_skb_ip_align(priv->dev, size);
        if (unlikely(!skb)) {                
            break;
        }
        buf = dma_map_single(priv->device, skb->data, size, DMA_FROM_DEVICE);
        if (dma_mapping_error(priv->device, buf)) {
            netdev_err(priv->dev, "pcie vnet map list failed\n");
            dev_kfree_skb(skb);
            break;
        }
        addr = (dma_addr_t *)skb->cb;
        *addr = buf;                     
        dma_wmb();
        skb_queue_tail(list, skb);
        cnt++;
    }
}

static void bst_pcievnet_rxmem_work(struct work_struct *work)
{
    struct pcie_vnet_priv *priv = container_of(work, struct pcie_vnet_priv, rxmem_work);
    int chan, busid, index;
    struct sk_buff_head *list;
     
    priv->xstats.rx_mem_poll++;
    busid = priv->id;
    set_bit(PCIE_VNET_RXMEM_WORK_RUN, &priv->state);
    for (chan = 0; chan < PCIE_VNET_RXCHAN_NUM; chan++) {
        index = busid * PCIE_VNET_RXCHAN_NUM + chan;
        list = &pcievnet_delivery_skblist[index];
        if (skb_queue_len(list) < PCIE_VNET_RXMEM_THRE) {        
            bst_pcievnet_refill_rxmem(priv, chan);
        }
    }
    clear_bit(PCIE_VNET_RXMEM_WORK_RUN, &priv->state);
}

static void bst_pcievnet_serv_event_sch(struct pcie_vnet_priv *priv)
{
	if (!test_bit(PCIE_VNET_DOWN, &priv->state) &&
	    !test_and_set_bit(PCIE_VNET_SERVICE_SCHED, &priv->state))
		queue_work(priv->wq, &priv->service_task);
}

static void bst_pcievnet_global_err(struct pcie_vnet_priv *priv)
{
	netif_carrier_off(priv->dev);
	set_bit(PCIE_VNET_RESET_REQUESTED, &priv->state);
	bst_pcievnet_serv_event_sch(priv);
}

/**
 *  bst_pcievnet_tx_timeout
 *  @dev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable time. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void bst_pcievnet_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct pcie_vnet_priv *priv = netdev_priv(dev);

	bst_pcievnet_global_err(priv);
}

static void bst_pcievnet_reset_subtask(struct pcie_vnet_priv *priv)
{
	if (!test_and_clear_bit(PCIE_VNET_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(PCIE_VNET_DOWN, &priv->state))
		return;

	netdev_err(priv->dev, "Reset adapter.\n");
	
	rtnl_lock();
	netif_trans_update(priv->dev);
	while (test_and_set_bit(PCIE_VNET_RESETING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(PCIE_VNET_DOWN, &priv->state);
	dma_wmb();
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(PCIE_VNET_DOWN, &priv->state);
	clear_bit(PCIE_VNET_RESETING, &priv->state);
	rtnl_unlock();
}

static void bst_pcievnet_service_task(struct work_struct *work)
{
	struct pcie_vnet_priv *priv = container_of(work, struct pcie_vnet_priv,
			service_task);

	bst_pcievnet_reset_subtask(priv);
	clear_bit(PCIE_VNET_SERVICE_SCHED, &priv->state);
}

static u16 bst_pcievnet_select_queue(struct net_device *dev, struct sk_buff *skb,
			       struct net_device *sb_dev)
{
	if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6 | SKB_GSO_UDP_L4)) {
		/*
		 * There is no way to determine the number of TSO
		 * capable Queues. Let's use always the Queue 0
		 * because if TSO is supported then at least this
		 * one will be capable.
		 */
		return 0;
	}

	return netdev_pick_tx(dev, skb, NULL) % dev->real_num_tx_queues;
}

static int bst_pcievnet_set_mac_address(struct net_device *ndev, void *addr)
{
	int ret = 0;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;

	return ret;
}

static int bst_pcievnet_change_mtu(struct net_device *dev, int new_mtu)
{
	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}
    
    if (new_mtu > 9000) {
        netdev_err(dev, "cannot more than 9000\n");
        return -EINVAL;
    }
    
	dev->mtu = new_mtu;

	netdev_update_features(dev);

	return 0;
}

static const struct net_device_ops bst_pcievnet_netdev_ops = {
	.ndo_open = bst_pcievnet_open,
	.ndo_start_xmit = bst_pcievnet_xmit,
	.ndo_stop = bst_pcievnet_release,
	.ndo_change_mtu = bst_pcievnet_change_mtu,
	//.ndo_fix_features = bst_pcievnet_fix_features,
	//.ndo_set_features = bst_pcievnet_set_features,
	//.ndo_set_rx_mode = bst_pcievnet_set_rx_mode,
	.ndo_tx_timeout = bst_pcievnet_tx_timeout,
	.ndo_select_queue = bst_pcievnet_select_queue,
	.ndo_set_mac_address = bst_pcievnet_set_mac_address,
};

static void bst_pcievnet_init_trans(struct pcie_vnet_priv *priv, int queue)
{
    u32 l_base, r_base, addr, head_size;
    char name[32];
    static struct picp_cfg trans_info;

    memset(&trans_info, 0, sizeof(struct picp_cfg));

    trans_info.timeout = PCIEVENT_TRANS_TIMEOUT_MS;
    sprintf(name, "pcievnet-r%d", queue);
    strncpy(trans_info.rname, name, 32);
    sprintf(name, "pcievnet-w%d", queue);
    strncpy(trans_info.wname, name, 32);

    head_size = 4 * sizeof(struct picp_head);
    addr = queue * head_size;
    if (priv->id == 0) {
        l_base = PICP_LOCAL_HEAD_NETWORK;
        r_base = PICP_REMOTE_HEAD_NETWORK;
    } else {   
        l_base = PICP_LOCAL_HEAD_NETWORK + PCIE_VNET_TXCHAN_NUM * head_size;
        r_base = PICP_REMOTE_HEAD_NETWORK + PCIE_VNET_TXCHAN_NUM * head_size;
    }
    
    trans_info.timeout = PCIEVENT_TRANS_TIMEOUT_MS;
	trans_info.local_head = l_base + addr;
	trans_info.remote_head = r_base + addr;
	trans_info.head_physize = head_size;

    trans_priv[queue] = picp_init(&trans_info);
}

int bst_pcievnet_dvr_probe(struct platform_device *pdev, struct pcie_vnet_plat *plat_dat)
{
    struct device *device;
	struct pcie_vnet_priv *priv;
    struct net_device *ndev;
	u32 queue;
	int ret = 0;

    ndev = alloc_etherdev_mqs(sizeof(struct pcie_vnet_priv),
				  PCIE_VNET_TXCHAN_NUM,
				  PCIE_VNET_RXCHAN_NUM);                   
    if (!ndev)
		return -ENOMEM;

    device = &pdev->dev;
	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;
    bst_pcievnet_set_ethtool_ops(ndev);

    pv_priv = priv;
    priv->plat = plat_dat;
    priv->id = plat_dat->id;
    priv->extend_op = plat_dat->board_type;
    priv->tx_fifo_size = plat_dat->tx_fifo_size;
    priv->rx_fifo_size = plat_dat->rx_fifo_size;
    priv->maxmtu = plat_dat->maxmtu;
    priv->ctrl_mem_base = plat_dat->ctrl_mem_base;
    priv->ctrl_mem_size = plat_dat->ctrl_mem_size;

    dev_set_drvdata(device, priv->dev);

    dma_set_mask(priv->device, DMA_BIT_MASK(64));
    dma_set_coherent_mask(priv->device, DMA_BIT_MASK(64));

	bst_pcievnet_check_ether_addr(priv);

	/* Configure real RX and TX queues */
	netif_set_real_num_rx_queues(ndev, priv->plat->rx_queues_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_queues_to_use);

	ndev->netdev_ops = &bst_pcievnet_netdev_ops;

    ndev->watchdog_timeo = msecs_to_jiffies(watchdog);

	priv->msg_enable = netif_msg_init(debug, default_msg_level);
	/* MTU range: 46 - hw-specific max */
	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;

	ndev->max_mtu = JUMBO_LEN;

	priv->wq = create_workqueue("pcievnet_service");//create_singlethread_workqueue("bstgmac_wq");
    if (!priv->wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}
	priv->rxmem = create_workqueue("pcievnet_rxmem");//create_singlethread_workqueue("pcievnet_rxmem");
    if (!priv->rxmem) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}
    priv->tx_wq = create_workqueue("pcievnet_tx");//create_singlethread_workqueue("pcievnet_tx");
    if (!priv->tx_wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto error_wq;
	}
 
    INIT_WORK(&priv->service_task, bst_pcievnet_service_task);
    INIT_WORK(&priv->rxmem_work, bst_pcievnet_rxmem_work);
    INIT_WORK(&priv->intf_task, bst_pcievnet_intf_task);

	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if ((priv->plat->maxmtu < ndev->max_mtu) &&
	    (priv->plat->maxmtu >= ndev->min_mtu))
		ndev->max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < ndev->min_mtu)
		dev_warn(priv->device,
			 "%s: warning: maxmtu having invalid value (%d)\n",
			 __func__, priv->plat->maxmtu);

    for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
        struct pcie_vnet_channel *ch = &priv->rx_channel[queue];

		struct pcie_vnet_rx_queue *rx_q;
		u32 base;
		rx_q = &priv->rx_queue[queue];
		
		base = priv->ctrl_mem_base+PCIEVNET_REMOTE_BASE+queue*PCIEVNET_REMOTE_SIZE;

		rx_q->laddr_info_va = ioremap(base, PCIEVNET_RADDR_SIZE);

		if (rx_q->laddr_info_va == NULL) {
			printk(KERN_ERR "%s line %d rxq ioremap fail base 0x%x raddr_size 0x%x remote_size 0x%x va 0x%llx\n", 
			__func__, __LINE__, base, PCIEVNET_RADDR_SIZE, PCIEVNET_REMOTE_SIZE, rx_q->laddr_info_va);
		}
        ch->priv_data = priv;
        ch->index = queue;

        skb_queue_head_init(&pcievnet_delivery_skblist[priv->id*PCIE_VNET_RXCHAN_NUM+queue]); 
	}

    for (queue = 0; queue < priv->plat->tx_queues_to_use; queue++) {
        struct pcie_vnet_channel *ch = &priv->tx_channel[queue];
        struct pcie_vnet_tx_queue *tx_q;
        u32 base;
        tx_q = &priv->tx_queue[queue];
        base = priv->ctrl_mem_base+PCIEVNET_REMOTE_BASE+PCIEVNET_REMOTE_TXOFF+queue*PCIEVNET_REMOTE_SIZE;
        
        tx_q->laddr_info_va = ioremap(base, PCIEVNET_RADDR_SIZE);
        
        if (tx_q->laddr_info_va == NULL) {
            printk(KERN_ERR "%s line %d txq ioremap fail raddr 0x%x raddr_size 0x%x remote_size 0x%x va 0x%llx\n", 
            __func__, __LINE__, base, PCIEVNET_RADDR_SIZE, PCIEVNET_REMOTE_SIZE, tx_q->laddr_info_va);
        }

        ch->priv_data = priv;
        ch->index = queue;

		bst_pcievnet_init_trans(priv, queue);
        INIT_WORK(&ch->tx_work, bst_pcievnet_tx_thread);
	} 

    ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->device, "%s: ERROR %i registering the device\n",
			__func__, ret);
		goto error_netdev_register;
	} 

#ifdef CONFIG_DEBUG_FS
	ret = bst_pcievnet_init_fs(ndev);
	if (ret < 0)
		netdev_warn(priv->dev, "%s: failed debugFS registration\n",
			    __func__);
#endif

    bst_pcievnet_create_thread(priv);

    return 0;

error_netdev_register:
    destroy_workqueue(priv->wq);
    destroy_workqueue(priv->rxmem);
    destroy_workqueue(priv->tx_wq);
error_wq:
	free_netdev(ndev);
	return ret;
}

/**
 * bst_pcievnet_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int bst_pcievnet_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct pcie_vnet_priv *priv = netdev_priv(ndev);
	struct pcie_vnet_rx_queue *rx_q;
	struct pcie_vnet_tx_queue *tx_q;
	int queue;

	netdev_info(priv->dev, "%s: removing driver", __func__);

#ifdef CONFIG_DEBUG_FS
	bst_pcievnet_exit_fs(ndev);
#endif
	netif_carrier_off(ndev);
	unregister_netdev(ndev);

	kthread_stop(rx_task0->tsk);
	kfree(rx_task0);
	if (priv->plat->rx_queues_to_use == 2) {
		kthread_stop(rx_task1->tsk);
		kfree(rx_task1);
	}

	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
    	rx_q = &priv->rx_queue[queue];
		if (rx_q->laddr_info_va) {
    		iounmap(rx_q->laddr_info_va);
		}
	}
	for (queue = 0; queue < priv->plat->tx_queues_to_use; queue++) {
    	tx_q = &priv->tx_queue[queue];
		if (tx_q->laddr_info_va) {
    		iounmap(tx_q->laddr_info_va);
		}
		picp_exit(trans_priv[queue]);
	}
	
	destroy_workqueue(priv->wq);
    destroy_workqueue(priv->rxmem);
    destroy_workqueue(priv->tx_wq);
	mutex_destroy(&priv->lock);
	free_netdev(ndev);

	return 0;
}
EXPORT_SYMBOL_GPL(bst_pcievnet_dvr_remove);

/*
* CANFD driver for BST CANFD
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

/*
* ChangeLog:
* Jan 2020: v1: create by xing.liao@bst.ai
*
*/
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#ifndef BST_CANFD_H
#define BST_CANFD_H

#include <linux/kfifo.h>

#define BSTCANFD_DRV_NAME			"bst_canfd"

#define BST_CANFD_TX_SUCCESS		1
#define BST_CANFD_TX_FAILED			0
#define BST_CANFD_RX_SUCCESS		1
#define BST_CANFD_RX_FAILED			0

#define BST_CANFD_0_BASE_ADDR 0x20016000
#define BST_CANFD_1_BASE_ADDR 0x20016800
#define BST_CANFD_2_BASE_ADDR 0x20017000

#define SAFETY_CRM_ADDR 0x70035000

#define BST_CANFD_TIMEOUT			(1 * HZ)
#define BST_CANFD_TX_TIMEOUT		(5 * HZ)

#define BST_DEBUG             0
#define BST_REG_DEBUG         0
#define BST_TXRX_DEBUG        0
#define BST_INT_DEBUG         0
#define BST_INJECT_ERR_EN     1 /* 用于测试错误注入，默认需要关闭 */
#define BST_ARB_LOST_TEST     1 /* 用于测试仲裁丢失，默认需要关闭 */
#define BST_LOOPBACK_TEST     1 /* 用于测试还回，默认需要关闭 */

#define BST_CANFD_NAPI_WEIGHT 32	/* Rx poll quota */

#define BST_DEFAULT_BAUDRATE  0 /* 使用默认的波特率，如果需要通过iproute2工具配置需要置0 */

#define CAN_GLBCTRL_REG       0x0000
#define CAN_TMCTRL0_REG       0x0004
#define CAN_TMCTRL1_REG       0x0008
#define CAN_ID_REG            0x000c
#define CAN_ID_MASK_REG       0x0010
#define CAN_SEND_ID_REG       0x0014

#define CAN_TX_DATA0_REG      0x0018
#define CAN_TX_DATA1_REG      0x001c
#define CAN_TX_DATA2_REG      0x0020
#define CAN_TX_DATA3_REG      0x0024
#define CAN_TX_DATA4_REG      0x0028
#define CAN_TX_DATA5_REG      0x002c
#define CAN_TX_DATA6_REG      0x0030
#define CAN_TX_DATA7_REG      0x0034
#define CAN_TX_DATA8_REG      0x0038 
#define CAN_TX_DATA9_REG      0x003c
#define CAN_TX_DATA10_REG     0x0040
#define CAN_TX_DATA11_REG     0x0044
#define CAN_TX_DATA12_REG     0x0048
#define CAN_TX_DATA13_REG     0x004c
#define CAN_TX_DATA14_REG     0x0050
#define CAN_TX_DATA15_REG     0x0054

#define CAN_RX_FIFO_DATA_REG  0x0058

#define CAN_RX_DATA0_REG      0x0058
#define CAN_RX_DATA1_REG      0x005c
#define CAN_RX_DATA2_REG      0x0060
#define CAN_RX_DATA3_REG      0x0064
#define CAN_RX_DATA4_REG      0x0068
#define CAN_RX_DATA5_REG      0x006c
#define CAN_RX_DATA6_REG      0x0070
#define CAN_RX_DATA7_REG      0x0074
#define CAN_RX_DATA8_REG      0x0078
#define CAN_RX_DATA9_REG      0x007c
#define CAN_RX_DATA10_REG     0x0080
#define CAN_RX_DATA11_REG     0x0084
#define CAN_RX_DATA12_REG     0x0088
#define CAN_RX_DATA13_REG     0x008c
#define CAN_RX_DATA14_REG     0x0090
#define CAN_RX_DATA15_REG     0x0094

#define CAN_TXERR_CNT_REG     0x0098
#define CAN_RXERR_CNT_REG     0x009c
#define CAN_REC_CTRLBIT_REG   0x00a0
#define CAN_REC_ID_REG        0x00a4
#define CAN_OVERWRITE_JUDGE_REG 0x00a8
#define CAN_IRQ_TYPE_REG      0x00ac
#define CAN_ERR_TYPE_REG      0x00b0
#define CAN_REC_TYPE_REG      0x00b4
#define CAN_STATUS_MASK_REG   0x00b8
#define CAN_ARB_LOST_CAPTURE_REG 0x00bc
#define CAN_STATUS_REG        0x00c0
#define CAN_PARITY_RESIDUAL_CTRL_REG 0x00c4
#define CAN_GLBCTRL1_REG      0x00c8
#define CAN_DMA_CTRL_REG      0x00d0
#define CAN_REG_END           CAN_DMA_CTRL_REG

#define IS_REG_OFF_INVALID(x)   ((x) > CAN_REG_END)

#define IS_BIT_SET(number, n) ((number >> n) & (0x1))
#define IS_BITS_SET(number, mask) ((number) & (mask))
#define BIT_CLEAR(number, n) (number &= (~(0x1 << n)))

/* can gblctrl register bit */
#define CAN_ERR_WARNING           23
#define CANFD_CRC_DELIM_JUDGE_EN  22
#define CANFD_ISO_SELECT_EN       21
#define CAN_LOOP_BACK             20
#define CAN_ABORT_RX              19
#define CAN_TRIPLE_SAMPLE         18
#define CAN_FILTER_MODE           17
#define SELF_TEST_MODE            16
#define CAN_ERR_WARNING_SEL       15
#define OVERLOAD_REQUEST_TIME     14
#define OVERLOAD_REQUEST          13
#define LISTEN_ONLY_MODE          12
#define CAN_ABORT_TX              11
#define CAN_SEND_BRS              10
#if defined(CONFIG_ARCH_BSTA1000A)
#define CAN_NODES_TYPE_SEL         5
#else
#define CAN_BUS_ENABLE		       5
#endif
#define TX_REQUEST                 1
#define RESET_MODE                 0
#define IS_BOSCH_CANFD(x)          ((((x) & 0x200000) >> 21) == 0)   
#define IS_ISO11898_2015_CANFD(x)  ((((x) & 0x200000) >> 21) == 1)  
#define CAN_SEND_DLC(x)            (((x) & 0xf) << 6)
#define CAN_SEND_DLC_CLR(data)     (data & (~(0xf << 6)))
#define CAN_SEND_STYLE(x)          (((x) & 0x7) << 2)
#define CAN_SEND_STYLE_CLR(data)   (data & (~(0x7 << 2)))
#define TX_CAN_BASIC_REMOTE_FRAME  0
#define TX_CAN_BASIC_DATA_FRAME    1
#define TX_CANFD_BASIC_DATA_FRAME  2
#define TX_CAN_EXTENT_REMOTE_FRAME 3
#define TX_CAN_EXTENT_DATA_FRAME   4
#define TX_CANFD_EXTENT_DATA_FRAME 5

/* can tmctrl0 and tmctrl1 register bit */
#define CANFD_NCFG_NSJW(x)		(((x) & 0xf) << 11)
#define CANFD_NCFG_NTSEG2(x)	(((x) & 0x1f) << 6)
#define CANFD_NCFG_NTSEG1(x)	(((x) & 0x3f) << 0)
#define CANFD_NCFG_NBRP(x)		(((x) & 0x3ff) << 0)
#define CANFD_NCFG_ZONE0        (0x7fff)
#define CANFD_NCFG_ZONE1        (0x3ff)

#if defined(CONFIG_ARCH_BSTA1000A)
#define CANFD_DCFG_DSJW(x)		(((x) & 0x3) << 22)
#define CANFD_DCFG_DTSEG2(x)	(((x) & 0x7) << 19)
#define CANFD_DCFG_DTSEG1(x)	(((x) & 0xf) << 15)
#define CANFD_DCFG_ZONE0        (0xff8000)
#else
#define CANFD_DCFG_DSJW(x)		(((x) & 0xf) << 26)
#define CANFD_DCFG_DTSEG2(x)	(((x) & 0x1f) << 21)
#define CANFD_DCFG_DTSEG1(x)	(((x) & 0x3f) << 15)
#define CANFD_DCFG_ZONE0        (0x3fff8000)
#endif

#define CANFD_DCFG_DBRP(x)		(((x) & 0x3ff) << 10)
#define CANFD_DCFG_ZONE1        (0xffc00)

/* can filter id, filter id mask, send id register */
#define CANFD_DEFAULT_ID          0x0
#define CANFD_DEFAULT_ID_MASK     0xffffffff
#define CANFD_REFUSE_MATCH_ID     0xfff000
#define CANFD0_DEFAULT_SEND_ID    0x123456 /* 对于扩展帧0x123456，标准帧其实为0x4,bit[28:18] */
#define CANFD0_DEFAULT_FILTER_ID  0x345678 
#define CANFD1_DEFAULT_SEND_ID    0x345678 /* 对于扩展帧0x345678，标准帧其实为0x13,bit[28:18] */
#define CANFD1_DEFAULT_FILTER_ID  0x123456 

/* can rec ctrlbit register bit */
#define RX_FRAME_BRS(x)         (((x) & 0x80000000) >> 31)
#define RX_EXTENT_FRAME_RTR(x)  (((x) & 0x40000000) >> 30)
#define RX_BASIC_FRAME_RTR(x)   (((x) & 0x20000000) >> 29)
#define RX_FRAME_DLC(x)         (((x) & 0x1e000000) >> 25)
#define RX_FRAME_CRC_SEQUENCE   ((x) & 0x1000000)

/* can rec id register bit */
#define RX_FRMAE_ESI(x)         (((x) & 0x20000000) >> 29)
#define RX_FRMAE_ID(x)          ((x) & 0x1fffffff)
#define RX_FRMAE_ID_BASIC(x)    ((x) & 0x7ff) /* 发送时候把id写在send_id[28:18],读回来读receive_id[10:0] */

/* can irq type register bit */
#define NODE_WARNING_STATUS_IRQ								 15
#define NODE_ERROR_PASSIVE_IRQ								 14
#define NODE_BUS_OFF_IRQ									 13
#define CAN_ASILD_SAFETY_IRQ								 12
#define INTERNAL_WR_DATA_PARITY_ERR_IRQ                      11
#define INTERNAL_COPY_DATA_BACK_PARITY_ERR_IRQ               10
#define INTERNAL_RXFIFO_CTL_RESIDUAL_LOGIC_ERR_IRQ           9
#define INTERNAL_REG_CFG_CTL_RESIDUAL_LOGIC_ERR_IRQ          8
#define INTERNAL_RD_DATA_BACK_PARITY_ERR_IRQ                 7
#define RCV_APB_ADDR_PARITY_ERR_IRQ                          6
#define RCV_APB_WR_DATA_PARITY_ERR_IRQ                       5
#define OVERWRITE_FIFO_IRQ                                   4 /* 读清 */
#define NODE_LOST_ARBITRATION_IRQ                            3 /* 读清 */
#define RX_SUCCESS_IRQ                                       2 /* 读清 */
#define TX_SUCCESS_IRQ                                       1 /* 读清 */
#define CAN_ERR_IRQ                                          0 /* 读清 */
#define RX_AND_ERR_IRQ_MASK				     0x1D
#define ERR_LA_OW_IRQ_MASK				     0x19

/* can err type register bit */
#define CAN_FORM_ERR_FLAG(x)	   (((x) & 0x10) >> 4)
#define CAN_CRC_ERR_FLAG(x)	       (((x) & 0x8) >> 3)
#define CAN_ACK_ERR_FLAG(x)	       (((x) & 0x4) >> 2)
#define CAN_STUFF_ERR_FLAG(x)	   (((x) & 0x2) >> 1)
#define CAN_BIT_ERR_FLAG(x)		   (((x) & 0x1) >> 0)

/* can rec type register bit */
#define IS_CANFD_BASIC_DATA_FRAME(x)  ((x) == 0x20)
#define IS_CANFD_EXTENT_DATA_FRAME(x) ((x) == 0x10)
#define IS_CAN_EXTENT_DATA_FRAME(x)   ((x) == 0x8)
#define IS_CAN_EXTENT_REMOTE_FRAME(x) ((x) == 0x4)
#define IS_CAN_BASIC_DATA_FRAME(x)    ((x) == 0x2)
#define IS_CAN_BASIC_REMOTE_FRAME(x)  ((x) == 0x1)
#define RX_CAN_BASIC_REMOTE  0x1
#define RX_CAN_BASIC_DATA    0x2
#define RX_CAN_EXTENT_REMOTE 0x4
#define RX_CAN_EXTENT_DATA   0x8
#define RX_CANFD_EXTENT_DATA 0x10
#define RX_CANFD_BASIC_DATA  0x20

#define RX_CAN_DATA_FRAME_MASK 0xA
#define RX_CANFD_DATA_FRAME_MASK 0x30

/* can status mask register bit (low active, default high) */
/* Parity error or residual logic error interrupt mask */
#define INTERNAL_REG_CFG_CTL_RESIDUAL_LOGIC_ERR_MASK         22
#define RCV_APB_WR_DATA_PARITY_ERR_MASK                      21
#define RCV_APB_ADDR_PARITY_ERR_MASK                         20
#define INTERNAL_RD_DATA_BACK_PARITY_ERR_MASK                19
#define INTERNAL_WR_DATA_PARITY_ERR_MASK                     18
#define INTERNAL_RXFIFO_CTL_RESIDUAL_LOGIC_ERR_MASK          17
#define INTERNAL_RX_RD_DATA_BACK_CTL_RESIDUAL_LOGIC_ERR_MASK 16
/* Frame type flag mask */
#define CANFD_BASIC_DATA_FRAME_MASK              15
#define CANFD_EXTENT_DATA_FRAME_MASK             14
#define CAN_EXTENT_DATA_FRAME_MASK               13
#define CAN_EXTENT_REMOTE_FRAME_MASK             12
#define CAN_BASIC_DATA_FRAME_MASK                11
#define CAN_BASIC_REMOTE_FRAME_MASK              10
/* Error type flag mask */
#define FORM_ERR_MASK                            9
#define CRC_ERR_MASK                             8
#define ACK_ERR_MASK                             7
#define STUFF_BIT_ERR_MASK                       6
#define BIT_ERR_MASK                             5
/* Interrupt flag mask */
#define NODE_LOST_ARBITRATION_IRQ_MASK           4
#define TX_SUCCESS_IRQ_MASK                      3
#define RX_SUCCESS_IRQ_MASK                      2
#define CAN_ERR_IRQ_MASK                         1
#define TOTAL_INT_MASK                           0

#if defined(CONFIG_ARCH_BSTA1000A)
#define ENABLE_ALL_INT                           0x7FFFFF
#else
#define ENABLE_ALL_INT                           0x7FFFFFF
#endif
#define DISABLE_ALL_INT                          0
#define ENABLE_RX_INT							 (0x1 << 2)
#define DISABLE_RX_INT							 (~(0x1 << 2))

/* can status register bit */
#define BUS_STATUS_BUS_ON(x)       ((((x) & 0x4) >> 2) == 1)
#define BUS_STATUS_BUS_OFF(x)      ((((x) & 0x4) >> 2) == 0)
#define SERIOUS_ERR_STATUS(x)      ((((x) & 0x20) >> 5) == 1)
#define NOT_SERIOUS_ERR_STATUS(x)  ((((x) & 0x20) >> 5) == 0)
#define BUS_STATUS_PASSIVE_ERR(x)  ((((x) & 0x40) >> 6) == 1)
#define BUS_STATUS_ACTIVE_ERR(x)   ((((x) & 0x40) >> 6) == 0)
#define RCV_BUFFER_HAVE_DATA(x)    ((((x) & 0x10) >> 4) == 1)
#define TRANSMIT_BUFFER_NEED_PREPARE(x) ((((x) & 0x2) >> 1) == 1)
#define RCV_STATUS(x)              ((((x) & 0x8) >> 3) == 1) 
#define TRANSMIT_BUFFER_STATUS(x)  (((x) & 0x2)  == 0)
#define TRANSMIT_STATUS(x)         (((x) & 0x1)  == 1)
#define OVERLOAD_TIME_STATUS(x)    (((x) & 0x180) >> 7) 

/*  */
#define CAN_ERR_CNT_ERR_WARNING    ((readl(priv->base + CAN_GLBCTRL_REG) >> CAN_ERR_WARNING) & 0x1FF)
#define CAN_ERR_CNT_ERR_PASSIVE    128
#define CAN_ERR_CNT_BUS_OFF        256
#define CAN_MAX_ERR_CNT_ADDITION   8

/* canfd parity and residual handle control register bit */
#define CAN_BIT_ERR_INJECT         1
#define CAN_STUFF_ERR_INJECT       2
#define CAN_CRC_ERR_INJECT         3
#define CAN_ACK_ERR_INJECT         4
#define CAN_FORM_ERR_INJECT        5

/* canfd dma control register bit */
#define CAN_DMA_TX_CTRL_EN            1
#define CAN_DMA_RX_CTRL_EN            0

/* dma control field */
#define DMA_IS_CANFD_EXTENT_DATA_FRAME(x) (((x) >> 13) & 0x1)
#define DMA_IS_CANFD_BASIC_DATA_FRAME(x)  (((x) >> 12) & 0x1)
#define DMA_IS_CAN_EXTENT_DATA_FRAME(x)   (((x) >> 11) & 0x1)
#define DMA_IS_CAN_BASIC_DATA_FRAME(x)    (((x) >> 10) & 0x1)
#define DMA_IS_CAN_EXTENT_REMOTE_FRAME(x) (((x) >> 9) & 0x1)
#define DMA_IS_CAN_BASIC_REMOTE_FRAME(x)  (((x) >> 8) & 0x1)
#define DMA_RX_FRMAE_ESI(x)				  (((x) >> 5) & 0x1)
#define DMA_RX_FRAME_BRS(x)				  (((x) >> 4) & 0x1)
#define DMA_RX_FRAME_DLC(x)				  ((x) & 0xf)

/* Maximum number of consecutive occurrences of ack err */
#define CAN_MAX_ACK_ERR				20000
#define CAN_MAX_CONT_RESTART		3

struct bst_canfd_rx_raw_data {
	unsigned int rx_type;
	unsigned int rx_id;
	unsigned int rx_dlc;
	unsigned int rx_data[16];
};

#define BST_CANFD_RX_KFIFO_SIZE		(128 * sizeof(struct bst_canfd_rx_raw_data))

struct bst_canfd_stats {
	unsigned long	can_std_pkts;
	unsigned long	can_std_rmt_pkts;
	unsigned long	can_ext_pkts;
	unsigned long	can_ext_rmt_pkts;
	unsigned long	bosch_fd_std_pkts;
	unsigned long	bosch_fd_ext_pkts;
	unsigned long	iso_fd_std_pkts;
	unsigned long	iso_fd_ext_pkts;

	unsigned long	can_std_bytes;
	unsigned long	can_ext_bytes;
	unsigned long	bosch_fd_std_bytes;
	unsigned long	bosch_fd_ext_bytes;
	unsigned long	iso_fd_std_bytes;
	unsigned long	iso_fd_ext_bytes;
};

struct bst_canfd_errtype_cnt {
	u64	bit_err;
	u64	stuff_err;
	u64	crc_err;
	u64	form_err;
	u64	ack_err;
};

#define BST_CANFD_DMA_FRAME_MAX_LEN 18
#define BST_CANFD_RX_DMA_FIFO_SIZE	512
#define BST_CANFD_RB_NOT_FULL 0
#define BST_CANFD_RB_FULL 1

#define DMA_FALGS_CANFD_DEV		(1 << 31)
/* 队列空：   head==tail
 * 队列满：   (tail+1)% MAXN ==head */
struct bst_canfd_ring_buffer
{
    u32 head; /* 头部，出队列方向*/
    u32 tail; /* 尾部，入队列方向*/ 
    u32 size ; /* 队列总尺寸 */
    u32 *data; /* 队列空间 */
	dma_addr_t data_phy_start;
	dma_addr_t data_phy;
};
#define bst_canfd_ring_buffer_is_empty(rb) (rb.tail == rb.head)
#define bst_canfd_ring_buffer_is_full(rb) (((rb.tail + BST_CANFD_DMA_FRAME_MAX_LEN) % (rb.size)) == rb.head)

int bst_canfd_ring_buffer_init(struct bst_canfd_ring_buffer *rb, unsigned int size);
void bst_canfd_ring_buffer_free(struct bst_canfd_ring_buffer *rb);

/* BST CANFD private data structure */
struct bst_canfd_priv {
	struct can_priv		can;	/* must be the first member */
	struct napi_struct	napi;
	resource_size_t 	phy_base;
	struct net_device	*ndev;
	struct device		*dev;
	void __iomem		*base;
	
	u16                 node;      /* 0:canfd0, 1:canfd1 */
	u16                 is_no_iso; /* 0:ISO 11898-1:2015 CANFD, 1:BOSCH CANFD */
	atomic_t            tx_suc_flag; /* tx success flag */
	atomic_t            rx_suc_flag; /* tx success flag */
	u32                 irq_type_bak;
	u32 				node_id;
	u32 				node_id_mask;
	int 				transceiver_gpio;
	u64 				reset_active;
	u32					pre_ojr;
#if BST_LOOPBACK_TEST
	u32                 loopback_rxtype; /* 用于还回测试时需要的特殊处理 */
#endif
#if BST_DEBUG
	u32 				can_id;
#endif
	atomic_t			cont_ack_err;   /* The number of consecutive occurrences of ack err */
	atomic_t			cont_tx_timeout;   /* The number of consecutive sending timeouts */
	spinlock_t			tx_lock;
	spinlock_t			status_lock;
	struct clk 			*wclk;
	struct clk 			*pclk;
	struct bst_canfd_stats	tx;
	struct bst_canfd_stats	rx;
	struct bst_canfd_errtype_cnt errtype_cnt;
	struct reset_control	*rst;
	struct task_struct	*echo_thread;
	struct kfifo rx_kfifo;
	spinlock_t	rx_kfifo_lock;
	struct bst_canfd_rx_raw_data rx_raw_data_in;
	struct bst_canfd_rx_raw_data rx_raw_data_out;
	struct dentry *debug_root;
#ifdef CONFIG_BST_CANFD_MEM_TEST
	dma_addr_t dma_addr_test;
	void *kern_addr;
#endif
	dma_cap_mask_t mask;
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	struct scatterlist sg;
	struct dma_async_tx_descriptor *dma_tx;
	struct axi_dma_desc *desc;
	struct axi_dma_chan *ad_chan;
	dma_cookie_t cookie;
	struct bst_canfd_ring_buffer rb;
	u32 rb_flag;
};

#endif /* BST_CANFD_H */

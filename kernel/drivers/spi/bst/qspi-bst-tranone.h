/* SPDX-License-Identifier: GPL-2.0 */
#ifndef DW_QSPI_HEADER_H
#define DW_QSPI_HEADER_H

#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/gpio.h>

#define QSPI_REG_TOP_CFG_R	    (0x20020010)
#define BST_QSPI_FRAME           4096
#define BST_QSPI_COMPLETION_TIMEOUT		msecs_to_jiffies(2000)

/* Register offsets */
#define DW_QSPI_CTRL0			0x00
#define DW_QSPI_CTRL1			0x04
#define DW_QSPI_SSIENR			0x08
#define DW_QSPI_MWCR			0x0c
#define DW_QSPI_SER			    0x10
#define DW_QSPI_BAUDR			0x14
#define DW_QSPI_TXFLTR			0x18
#define DW_QSPI_RXFLTR			0x1c
#define DW_QSPI_TXFLR			0x20
#define DW_QSPI_RXFLR			0x24
#define DW_QSPI_SR			    0x28
#define DW_QSPI_IMR			    0x2c
#define DW_QSPI_ISR			    0x30
#define DW_QSPI_RISR			0x34
#define DW_QSPI_TXOICR			0x38
#define DW_QSPI_RXOICR			0x3c
#define DW_QSPI_RXUICR			0x40
#define DW_QSPI_MSTICR			0x44
#define DW_QSPI_ICR			    0x48
#define DW_QSPI_DMACR			0x4c
#define DW_QSPI_DMATDLR			0x50
#define DW_QSPI_DMARDLR			0x54
#define DW_QSPI_IDR			    0x58
#define DW_QSPI_VERSION			0x5c
#define DW_QSPI_DR			    0x60

#define DW_QSPI_RX_SAMPLE_DELAY			    0xf0
#define DW_QSPI_SPI_CTRLR0			        0xf4
#define DW_QSPI_DDR_DRIVER_EDGE			    0xf8
#define DW_QSPI_XIP_MODE_BITS			    0xfc



/* Bit fields in CTRLR0 */
#define QSPI_DFS_OFFSET			0

#define QSPI_FRF_OFFSET			6
#define QSPI_FRF_SPI			0x0
#define QSPI_FRF_SSP			0x1
#define QSPI_FRF_MICROWIRE		0x2
#define QSPI_FRF_RESV			0x3

#define QSPI_MODE_OFFSET			8
#define QSPI_SCPH_OFFSET			8
#define QSPI_SCPOL_OFFSET			9

#define QSPI_TMOD_OFFSET			10
#define QSPI_TMOD_MASK			(0x3 << QSPI_TMOD_OFFSET)
#define	QSPI_TMOD_TR			0x0		/* xmit & recv */
#define QSPI_TMOD_TO			0x1		/* xmit only */
#define QSPI_TMOD_RO			0x2		/* recv only */
#define QSPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define QSPI_SLVOE_OFFSET		12
#define QSPI_SRL_OFFSET			13
#define QSPI_SSTE_OFFSET		14
#define QSPI_CFS_OFFSET			16

#define QSPI_CFS_MASK			(0xf << QSPI_CFS_OFFSET)
#define	QSPI_CFS_SIZE_01_BIT	0x0		/* 01-bit Control Word */
#define	QSPI_CFS_SIZE_02_BIT	0x1		/* 02-bit Control Word */
#define	QSPI_CFS_SIZE_03_BIT	0x2		/* 03-bit Control Word */
#define	QSPI_CFS_SIZE_04_BIT	0x3		/* 04-bit Control Word */
#define	QSPI_CFS_SIZE_05_BIT	0x4		/* 05-bit Control Word */
#define	QSPI_CFS_SIZE_06_BIT	0x5		/* 06-bit Control Word */
#define	QSPI_CFS_SIZE_07_BIT	0x6		/* 07-bit Control Word */
#define	QSPI_CFS_SIZE_08_BIT	0x7		/* 08-bit Control Word */
#define	QSPI_CFS_SIZE_09_BIT	0x8		/* 09-bit Control Word */
#define	QSPI_CFS_SIZE_10_BIT	0x9		/* 10-bit Control Word */
#define	QSPI_CFS_SIZE_11_BIT	0xa		/* 11-bit Control Word */
#define	QSPI_CFS_SIZE_12_BIT	0xb		/* 12-bit Control Word */
#define	QSPI_CFS_SIZE_13_BIT	0xc		/* 13-bit Control Word */
#define	QSPI_CFS_SIZE_14_BIT	0xd		/* 14-bit Control Word */
#define	QSPI_CFS_SIZE_15_BIT	0xe		/* 15-bit Control Word */
#define	QSPI_CFS_SIZE_16_BIT	0xf		/* 16-bit Control Word */

#define QSPI_SPI_FRF_OFFSET     22
#define QSPI_SPI_FRF_MASK			(0x3 << QSPI_SPI_FRF_OFFSET)
#define	QSPI_SPI_FRF_STD			0x0		/* standard SPI format*/
#define QSPI_SPI_FRF_DUAL			0x1		/* Dual SPI format */
#define QSPI_SPI_FRF_QUAD			0x2		/* Quad SPI format */
#define QSPI_SPI_FRF_OCTAL		    0x3		/* Octal SPI format */

#define QSPI_SPI_HYPERBUS_EN_OFFSET	24      /* SPI Hyperbus Frame format enable */   

/* Bit fields in CTRLR1 */
#define QSPI_NDF_OFFSET			     0      /* Number of Data Frames */

/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				(1 << 0)
#define SR_TF_NOT_FULL		(1 << 1)
#define SR_TF_EMPT			(1 << 2)
#define SR_RF_NOT_EMPT		(1 << 3)
#define SR_RF_FULL			(1 << 4)
#define SR_TX_ERR			(1 << 5)
#define SR_DCOL				(1 << 6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define QSPI_INT_TXEI			(1 << 0)
#define QSPI_INT_TXOI			(1 << 1)
#define QSPI_INT_RXUI			(1 << 2)
#define QSPI_INT_RXOI			(1 << 3)
#define QSPI_INT_RXFI			(1 << 4)
#define QSPI_INT_MSTI			(1 << 5)
#define QSPI_INT_XRXOI			(1 << 6)


/* Bit fields in DMACR */
#define QSPI_DMA_RDMAE			(1 << 0)
#define QSPI_DMA_TDMAE			(1 << 1)

/* TX RX interrupt level threshold, max can be 256 */
#define QSPI_INT_THRESHOLD		32

/* Bit fields in SPI_CTRLR0 */
#define QSPI_TRANS_TYPE_OFFSET			0
#define QSPI_TRANS_TYPE_MASK			(0x3 << QSPI_TRANS_TYPE_OFFSET)
#define	QSPI_TRANS_TYPE_TT0			0x0		/* Instruction and Address will be send in Standard SPI mode*/
#define QSPI_TRANS_TYPE_TT1			0x1		/* Instruction will be send in Standard SPI Mode and Address will be send in the mode specified by CTRLR0.SPI_FRF */
#define QSPI_TRANS_TYPE_TT2			0x2		/* Both Instruction and Address will be send in the mode specified by SPI_FRF  */
#define QSPI_TRANS_TYPE_TT3		    0x3		/* Reserved  */

#define QSPI_ADDR_L_OFFSET			    2
#define QSPI_ADDR_L_MASK			(0xf << QSPI_ADDR_L_OFFSET)
#define	QSPI_ADDR_L_L0			0x0		/* No Address */
#define QSPI_ADDR_L_L4			0x1		/* 4 bit Address length */
#define QSPI_ADDR_L_L8			0x2		/* 8 bit Address length  */
#define QSPI_ADDR_L_L12		    0x3		/* 12 bit Address length  */
#define QSPI_ADDR_L_L16			0x4		/* 16 bit Address length */
#define QSPI_ADDR_L_L20			0x5		/* 20 bit Address length  */
#define QSPI_ADDR_L_L24		    0x6		/* 24 bit Address length  */
#define QSPI_ADDR_L_L28			0x7		/* 28 bit Address length */
#define QSPI_ADDR_L_L32			0x8		/* 32 bit Address length  */
#define QSPI_ADDR_L_L36		    0x9		/* 36 bit Address length  */
#define QSPI_ADDR_L_L40			0xa		/* 40 bit Address length */
#define QSPI_ADDR_L_L44			0xb		/* 44 bit Address length  */
#define QSPI_ADDR_L_L48		    0xc		/* 48 bit Address length  */
#define QSPI_ADDR_L_L52			0xd		/* 52 bit Address length */
#define QSPI_ADDR_L_L56			0xe		/* 56 bit Address length  */
#define QSPI_ADDR_L_L60		    0xf		/* 60 bit Address length  */

#define QSPI_XIP_MD_BIT_EN_OFFSET		7
#define QSPI_INST_L_OFFSET			    8
#define QSPI_INST_L_MASK			(0x3 << QSPI_INST_L_OFFSET)
#define	QSPI_INST_L_L0			0x0		/* No Instruction */
#define QSPI_INST_L_L4			0x1		/* 4 bit Instruction length */
#define QSPI_INST_L_L8			0x2		/* 8 bit Instruction length  */
#define QSPI_INST_L_L16		    0x3		/* 16 bit Instruction length  */

#define QSPI_WAIT_CYCLES_OFFSET			11
#define QSPI_SPI_DDR_EN_OFFSET			16
#define QSPI_INST_DDR_EN_OFFSET			17
#define QSPI_SPI_RXDS_EN_OFFSET			18
#define QSPI_XIP_DFS_HC_OFFSET		    19
#define QSPI_XIP_INST_EN_OFFSET			20
#define QSPI_SSIC_XIP_CONT_XFER_EN_OFFSET			21
#define QSPI_SPI_DM_EN_OFFSET			24
#define QSPI_SPI_RXDS_SIG_EN_OFFSET		25
#define QSPI_XIP_MBL_OFFSET			    26
#define QSPI_XIP_PREFETCH_EN_OFFSET		29
#define QSPI_CLK_STRETCH_EN_OFFSET		30

enum dw_qssi_work_mode {
	SSI_WM_NORMAL = 0,
	SSI_WM_ENHANCED ,
};

enum dw_qssi_type {
	SSI_MOTO_SPI = 0,
	SSI_TI_SSP,
	SSI_NS_MICROWIRE,
};

enum dw_qspi_spi_type {
	SPI_STANDARD = 0,
	SPI_DUAL,
	SPI_QUAD,
	SPI_OCTAL,
};

enum dw_addr_length {
    QSPI_ADDR_L0,         /* No Address*/
    QSPI_ADDR_L4,	        /* 4 bit Address length */
    QSPI_ADDR_L8,         /* 8 bit Address length  */
	QSPI_ADDR_L12,        /* 12 bit Address length  */
	QSPI_ADDR_L16,        /* 16 bit Address length */
	QSPI_ADDR_L20,        /* 20 bit Address length  */
	QSPI_ADDR_L24,        /* 24 bit Address length  */
	QSPI_ADDR_L28,        /* 28 bit Address length */
	QSPI_ADDR_L32,        /* 32 bit Address length  */
};

enum dw_inst_length {
	INST_L0 = 0,       /* No instrction*/
	INST_L4,           /* 4 bit instrction length*/
	INST_L8,           /* 8 bit instrction length*/
	INST_L16,          /* 16 bit instrction length*/
};

enum dw_trans_type {
	TT0 = 0,       /* Instruction and Address will be send in Standard SPI mode*/
	TT1,           /* Instruction will be send in Standard SPI Mode and Address will be send in the mode specified by CTRLR0.SPI_FRF */
	TT2,           /* Both Instruction and Address will be send in the mode specified by SPI_FRF  */
	TT3,          /* Reserved  */
};

struct dw_qspi;

typedef struct dw_qspi_enhanced_msg_en_s{
	void *buf;
	int istx;
	int len;
	int width_len;
}dw_qspi_enhanced_msg_en_t;
typedef struct dw_qspi_enhanced_msg_s{
	//struct dw_qspi *dws;
	dw_qspi_enhanced_msg_en_t msg_en[8];
	int msg_num;
}dw_qspi_enhanced_msg_t;

struct dw_qspi_dma_ops {
	int (*dma_init)(struct dw_qspi *dws);
	void (*dma_exit)(struct dw_qspi *dws);
	int (*dma_setup)(struct dw_qspi *dws, struct spi_transfer *xfer);
	bool (*can_dma)(struct spi_controller *master, struct spi_device *spi,
			struct spi_transfer *xfer);
	int (*dma_transfer)(struct dw_qspi *dws, struct spi_transfer *xfer);
	void (*dma_stop)(struct dw_qspi *dws);
};

struct dw_qspi {
	struct spi_controller	*master;
	enum dw_qssi_type	type;
	enum dw_qspi_spi_type	spi_type;
	
	u32 tmode;
	/* list synchronization */
	struct mutex            list_lock;
	dw_qspi_enhanced_msg_t  enhanced_msg;

	void __iomem		*regs;
	unsigned long		paddr;
	int			enh_stat;
	int			rx_ignor_len;
	int			tx_dummy_len;
	int			irq;
	int 		use_gpio_cs;
	u32			fifo_len;	/* depth of the FIFO buffer */
	u32			max_freq;	/* max bus freq supported */
	u32			work_mode;
	u32			reg_io_width;	/* DR I/O width in bytes */
	u16			bus_num;
	u16			num_cs;		/* supported slave numbers */
	u16         mode;       /* mode0-mode3 */
	int			*cs_gpios;		/* supported slave numbers */
	void (*set_cs)(struct spi_device *spi, bool enable);
	
	/* Current message transfer state info */
	struct spi_transfer	*cur_transfer;
	struct spi_message	*cur_msg;
	struct chip_data	*cur_chip;
	size_t			len;
	void			*tx;
	void			*tx_end;
	void			*rx;
	void			*rx_end;
	int			dma_mapped;
	u8			n_bytes;	/* current is a 1/2 bytes op */
	u8			start;	
	u8			done;	
	u32			dma_width;
	u32			cmd;
	u32			addr;
	u32			txmode;
	u32			has_addr;
	u32			addr_len;
	u32			data_width;
	//u8			head_buf[12];
	//u32			head_len;
	irqreturn_t		(*transfer_handler)(struct dw_qspi *dws);
	u32			current_freq;	/* frequency in hz */

	/* DMA info */
	int			dma_inited;
	struct dma_chan		*txchan;
	struct dma_chan		*rxchan;
	unsigned long		dma_chan_busy;
	dma_addr_t		dma_addr; /* phy address of the Data register */
	const struct dw_qspi_dma_ops *dma_ops;
	void			*dma_tx;
	void			*dma_rx;
	/* Bus interface info */
	void			*priv;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
};


static inline u32 dw_qspi_readl(struct dw_qspi *dws, u32 offset)
{
	return __raw_readl(dws->regs + offset);
}

static inline u16 dw_qspi_readw(struct dw_qspi *dws, u32 offset)
{
	return __raw_readw(dws->regs + offset);
}

static inline void dw_qspi_writel(struct dw_qspi *dws, u32 offset, u32 val)
{
	__raw_writel(val, dws->regs + offset);
}

static inline void dw_qspi_writew(struct dw_qspi *dws, u32 offset, u16 val)
{
	__raw_writew(val, dws->regs + offset);
}

static inline u32 dw_qspi_read_io_reg(struct dw_qspi *dws, u32 offset)
{
	switch (dws->reg_io_width) {
	case 2:
		return dw_qspi_readw(dws, offset);
	case 4:
	default:
		return dw_qspi_readl(dws, offset);
	}
}

static inline void dw_qspi_write_io_reg(struct dw_qspi *dws, u32 offset, u32 val)
{
	switch (dws->reg_io_width) {
	case 2:
		dw_qspi_writew(dws, offset, val);
		break;
	case 4:
	default:
		dw_qspi_writel(dws, offset, val);
		break;
	}
}

static inline void qspi_enable_chip(struct dw_qspi *dws, int enable)
{
	dw_qspi_writel(dws, DW_QSPI_SSIENR, (enable ? 1 : 0));
}

static inline void qspi_set_clk(struct dw_qspi *dws, u16 div)
{
	dw_qspi_writel(dws, DW_QSPI_BAUDR, div);
}

/* Disable IRQ bits */
static inline void qspi_mask_intr(struct dw_qspi *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_qspi_readl(dws, DW_QSPI_IMR) & ~mask;
	dw_qspi_writel(dws, DW_QSPI_IMR, new_mask);
}

/* Enable IRQ bits */
static inline void qspi_umask_intr(struct dw_qspi *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_qspi_readl(dws, DW_QSPI_IMR) | mask;
	dw_qspi_writel(dws, DW_QSPI_IMR, new_mask);
}

/*
 * This does disable the SPI controller, interrupts, and re-enable the
 * controller back. Transmit and receive FIFO buffers are cleared when the
 * device is disabled.
 */
static inline void qspi_reset_chip(struct dw_qspi *dws)
{
	qspi_enable_chip(dws, 0);
	qspi_mask_intr(dws, 0xff);
	dw_qspi_writel(dws, DW_QSPI_RX_SAMPLE_DELAY, 0);
	qspi_enable_chip(dws, 1);
}

static inline void qspi_shutdown_chip(struct dw_qspi *dws)
{
	qspi_enable_chip(dws, 0);
	qspi_set_clk(dws, 0);
}

/*
 * Each SPI slave device to work with dw_api controller should
 * has such a structure claiming its working mode (poll or PIO/DMA),
 * which can be save in the "controller_data" member of the
 * struct spi_device.
 */
struct dw_qspi_chip {
	u8 poll_mode;	/* 1 for controller polling mode */
	u8 type;	/* SPI/SSP/MicroWire */
	void (*cs_control)(u32 command);
};

extern void dw_qspi_set_cs(struct spi_device *spi, bool enable);
extern int dw_qspi_add_host(struct device *dev, struct dw_qspi *dws);
extern void dw_qspi_remove_host(struct dw_qspi *dws);
extern int dw_qspi_suspend_host(struct dw_qspi *dws);
extern int dw_qspi_resume_host(struct dw_qspi *dws);

/* platform related setup */
extern int dw_qspi_mid_init(struct dw_qspi *dws); /* Intel MID platforms */
#endif /* DW_QSPI_HEADER_H */

// SPDX-License-Identifier: GPL-2.0
/*
 * BST PCIE Interconnection Control Protocol system.
 *
 * Copyright (C) 2021 Black Sesame Technologies, Inc.
 */
#ifndef _PICP_H_
#define _PICP_H_

/* picp dma side */
#define PCIE_DMA_SIDE 1
#define PCIE_MEM_SIDE 0
#ifdef CONFIG_USE_RC_DMA
#define PCIE_MODE_RC PCIE_DMA_SIDE
#define PCIE_MODE_EP PCIE_MEM_SIDE
#else
#define PCIE_MODE_EP PCIE_DMA_SIDE
#define PCIE_MODE_RC PCIE_MEM_SIDE
#endif

/* read and write */
#define PICP_READ  0
#define PICP_WRITE 1

/* picp error num */
#define PCIE_ERR_TIMEOUT  -1   /* timeout */
#define PCIE_ERR_DMAERR   -2   /* DMA error */
#define PCIE_ERR_LINKERR  -3   /* link error */
#define PCIE_ERR_PARAERR  -4   /* parameter error */
#define PCIE_ERR_STOPTRF  -5   /* stop transfer */

/* picp head phys address */
#define PICP_LOCAL_HEAD_MEM     0x8fd00000
#define PICP_REMOTE_HEAD_MEM    0x8fd00000

#define PICP_HEAD_CHIPID_SIZE   128
#define PICP_HEAD_IONTEST_SIZE  256
#define PICP_HEAD_KTHREAD_SIZE  512
#define PICP_HEAD_NETWORK_SIZE  512

#define PICP_LOCAL_HEAD_CHIPID   PICP_LOCAL_HEAD_MEM
#define PICP_LOCAL_HEAD_IONTEST  (PICP_LOCAL_HEAD_CHIPID + PICP_HEAD_CHIPID_SIZE)
#define PICP_LOCAL_HEAD_KTHREAD  (PICP_LOCAL_HEAD_IONTEST + PICP_HEAD_IONTEST_SIZE)
#define PICP_LOCAL_HEAD_NETWORK  (PICP_LOCAL_HEAD_KTHREAD + PICP_HEAD_KTHREAD_SIZE)

#define PICP_REMOTE_HEAD_CHIPID   PICP_REMOTE_HEAD_MEM
#define PICP_REMOTE_HEAD_IONTEST  (PICP_REMOTE_HEAD_CHIPID + PICP_HEAD_CHIPID_SIZE)
#define PICP_REMOTE_HEAD_KTHREAD  (PICP_REMOTE_HEAD_IONTEST + PICP_HEAD_IONTEST_SIZE)
#define PICP_REMOTE_HEAD_NETWORK  (PICP_REMOTE_HEAD_KTHREAD + PICP_HEAD_KTHREAD_SIZE)

struct ion_share_info {
	int buffd;
	unsigned int buflen;
	unsigned int flag_type;
	unsigned int heap_type;
	int rwopt;
};

struct ioctl_info {
	int size;
	int rwopt;
	int thread;
	int single;
	int sniffer;
	int quit;
};

/* picp data type */
typedef enum {
	PICP_DTYP_UNDEF = 0,
	PICP_DTYP_DATA,
	PICP_DTYP_DETECT,
	PICP_DTYP_GETSDR,
	PICP_DTYP_CMD,
	PICP_DTYP_CHIPID,
	PICP_DTYP_CTRL
} picp_dtype;

/* picp ctl head */
struct picp_head {
 	u32 head;      /* head flag */
	u16 dtype;     /* data type */
 	u16 status;    /* status */
 	u32 dataid;    /* data id */
 	u32 length;    /* data length */
	u64 physaddr;  /* data dma address */
	u32 cont;
	u32 verify;    /* data verify */
	u16 itype;     /* ctl type */
};

/* local record */
struct picp_record {
	u32 rtotal;    /* read total */
	u32 rerrors;   /* read err total */
	u32 rlinkerr;  /* read link err */
	u32 rdmaerr;   /* read dma err */
	u32 rtimeout;  /* read timeout */
	u64 rbytes;

	u32 wtotal;    /* write total */
	u32 werrors;   /* write err total */
	u32 wlinkerr;  /* write link err */
	u32 wdmaerr;   /* write dma err */
	u32 wtimeout;  /* write timeout */
	u64 wbytes;

	u32 ltssm_count;
	u32 type;
	u32 state;
};

/* picp config */
struct picp_cfg {
	char rname[32];
	char wname[32];
	u32 timeout;
	u32 rwopt;
	u64 local_head;
	u64 remote_head;
	u32 head_physize;
};

struct picp_info {
	char name[32];
	u32 timeout;
	u32 channel;
	u32 currid;
	u32 previd;
	u32 linksta;   /* pcie link status */
	u32 rwside;    /* dma read/write side */
	u32 record;    /* id record */
	u32 sniffer;   /* sniffer package */
	u32 stop_flag;

	u32 hsize;
	u64 hlocal;    /* local  head phys address */
	u64 hremote;   /* remote head phys address */
	struct picp_head *hvirt;  /* local head virt address */

	u32 dsize;     /* data size */
	u64 dlocal;    /* local data phys address */
	void *dvirt;   /* local data virt address */	
	u32 dtype;     /* data type */
	
	u64 custom;
};

/* picp private date */
struct picp_private {
	u32 rwside;
	struct picp_info rinfo;
	struct picp_info winfo;
	struct picp_head *vaddr;
	struct picp_record rec;
	int (*read_data) (struct picp_info *, u64, u32, u32);
	int (*write_data) (struct picp_info *, u64, u32, u32);
};

struct pcie_chipid {
	u64 local_chipid;
	u64 remote_chipid;
	u32 timeout;
	u32 state;
};

#define PICP_IOC_MAGIC    'p'
#define PICP_IOC_QUERY    _IOR(PICP_IOC_MAGIC, 1, struct picp_record)
#define PICP_IOC_SNIFFER  _IOR(PICP_IOC_MAGIC, 2, int)
#define PICP_IOC_INIT     _IOW(PICP_IOC_MAGIC, 3, struct picp_cfg)
#define PICP_IOC_EXIT     _IOW(PICP_IOC_MAGIC, 4, struct picp_cfg)
#define PICP_IOC_RWTEST   _IOW(PICP_IOC_MAGIC, 5, struct ioctl_info)
#define PICP_IOC_SIONFD   _IOW(PICP_IOC_MAGIC, 6, struct ion_share_info)
#define PICP_IOC_CHIPID   _IOWR(PICP_IOC_MAGIC, 7, struct pcie_chipid)
#define PICP_IOC_MAXNR    8

#if (defined CONFIG_BST_PICP) || (defined CONFIG_BST_PICP_MODULE)
/**
 * picp_read() - picp read data
 */
extern int picp_read(struct picp_private *priv, u64 dma_buf, u32 len, u32 type);

/**
 * picp_write() - picp write data
 */
extern int picp_write(struct picp_private *priv, u64 dma_buf, u32 len, u32 type);

/**
 * picp_init() - alloc picp resources and init
 */
extern struct picp_private *picp_init(struct picp_cfg *cfg);

/**
 * picp_exit() - release picp resources
 */
extern void picp_exit(struct picp_private *priv);

#else

static inline int picp_read(struct picp_private *priv, u64 dma_buf, u32 len, u32 type)
{
	return -1;
}

static inline int picp_write(struct picp_private *priv, u64 dma_buf, u32 len, u32 type)
{
	return -1;
}

static inline struct picp_private *picp_init(struct picp_cfg *cfg)
{
	return NULL;
}

static inline void picp_exit(struct picp_private *priv)
{
}

static inline int picp_cdev_init(void)
{
	return -1;
}

static inline void picp_cdev_exit(void)
{
}

#endif //CONFIG_BST_PICP


#endif //_PICP_H_

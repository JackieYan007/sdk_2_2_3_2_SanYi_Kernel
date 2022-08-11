#ifndef __OFILM_IPC_ADDRESS_H
#define __OFILM_IPC_ADDRESS_H

#define MAX_BUF_NR            1024
#define MIN_BUF_NR            32
#define MAX_BUF_SIZE          8192

struct ipc_cfg {
	int tx_buf_nr;
	int tx_buf_size;

	int rx_buf_nr;
	int rx_buf_size;
};

/* IOCTL definitions */
#define OFILM_IPC_IOC_MAGIC            'o'
#define OFILM_IPC_IOC_SETCFG           _IOW(OFILM_IPC_IOC_MAGIC, 0, struct ipc_cfg)
#define OFILM_IPC_IOC_MAXNR            (1)

#endif

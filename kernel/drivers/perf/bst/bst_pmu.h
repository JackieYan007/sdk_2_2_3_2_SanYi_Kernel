#ifndef _BST_PMU_H
#define _BST_PMU_H

#define NOC_MONITOR_TIME (60)

#define CPUNOC_COUNT		(7u)
#define CORENOC_COUNT		(4u)
#define NOC_COUNT			(CPUNOC_COUNT + CORENOC_COUNT)

#define NOC_IOCTL_SET_BASE  		_IOWR('C', 0x1, NOCBASE_USER)
#define NOC_IOCTL_GET_BASE  		_IOWR('C', 0x2, NOCBASE_USER)
#define NOC_IOCTL_SET_CONF  		_IOWR('C', 0x3, NOCCONF_USER)
#define NOC_IOCTL_GET_CONF  		_IOWR('C', 0x4, NOCCONF_USER)
#define NOC_IOCTL_SET_START    		_IOWR('C', 0x5, int)
#define NOC_IOCTL_SET_STOP     		_IO('C', 0x6)
#define NOC_IOCTL_SET_WINDOWTIME 	_IOWR('C', 0x7, int)
#define NOC_IOCTL_SET_MODE 			_IOWR('C', 0x8, int)

/* NOC register definition */
#define NOC_MAIN_CTRL		(0x8u)
#define NOC_CFG_CTL			(0xCu)
#define NOC_TRACEPORT_SEL	(0x10u)
#define NOC_FILTERLUT		(0x14u)
#define NOC_STAT_PERIOD		(0x24u)
#define NOC_ROUTEID_BASE	(0x44u)
#define NOC_ROUTEID_MASK	(0x48u)
#define NOC_ADDRBASE_LOW	(0x4cu)
#define NOC_WINDOW_SIZE 	(0x54u)
#define NOC_OPCODE 			(0x60u)
#define NOC_PORT_SEL		(0x134u)
#define NOC_CNT0_SRC		(0x138u)
#define NOC_CNT0_VAL		(0x140u)
#define NOC_CNT1_SRC		(0x14Cu)
#define NOC_CNT1_VAL		(0x154u)

#define NOC_COUNTER_DEFAULT_VAL		(20)

#define NOC_STATUS_IDLE			(1u)
#define NOC_STATUS_READY		(2u)
#define NOC_STATUS_RUNNING		(3u)
#define NOC_STATUS_COMPLETION	(4u)
#define NOC_STATUS_FREE			(5u)

/* noc node name:              cpu  d0p0   d0p1   d1p0    d1p1    gdma    s2c     cv   isp    net     vsp */
int node_off_tbl[NOC_COUNT] = {0x0, 0x800, 0xc00, 0x1000, 0x1400, 0x1800, 0x1c00, 0x0, 0x800, 0x1000, 0x1800};
//for noc system
/* ----------------------noc node struct------------------------------ */
typedef struct __nocnode_conf{
	int rid_base;	//route id base
	int rid_mask;	//route id mask
	int addr_low;	//addr-base low
	int windsize;	//window size
	int opcpde;		//opcpde
	// int irsv;		
}NOCNODE_CONF, *PNOCNODE_CONF;

typedef struct __noc_node{
	int id;			//0 ~ NOC_COUNT
	int stat;		//0:invalid, 1:run, 2:completed
	NOCNODE_CONF nocnode_conf;
	void * confbase; //init from node_off_tbl[]+ioremap_nocache()
}NOC_NODE, *PNOC_NODE;

/* ----------------------noc system struct------------------------------ */
typedef struct __noc_addr{
	void __iomem *phy_addr;
	void __iomem *vir_addr;
}NOC_ADDR;

typedef struct __noc_base
{
	NOC_ADDR coresight_base;			
	NOC_ADDR cpunoc_atb_base;		
	NOC_ADDR corenoc_atb_base;		
	NOC_ADDR buf_base;
	int buf_len;	//by ioctl setting
}NOC_BASE;

typedef struct __noc_sys {
	NOC_BASE noc_base;
	void * cpunoc_basephy;		//cpunoc phy-addr
	void * corenoc_basephy; 	//corenoc phy-addr
	int window_time;
	int mode;	//0: noc monitor(default) 1: noc capture
	int status;
	NOC_NODE noc_node[NOC_COUNT];
}NOC_SYS, *PNOC_SYS;

// extern void release_sys_noc(PNOC_SYS ptrsys);

/* ----------------------noc user struct------------------------------ */
//for ioctl
typedef struct __noc_baseinfo{
	void * nocbuf;
	int buflen;
	int mode;
	int window_time;
	int mask;
	int status;
}NOCBASE_INFO, *PNOCBASE_INFO;

typedef struct __noc_baseinfo \
	NOCBASE_USER, *PNOCBASE_USER;

typedef struct __noc_confinfo{
	int id;	//for multicap, (0x1<<ID)
	NOCNODE_CONF noc_node_conf;
}NOCCONF_INFO, *PNOCCONF_INFO;

typedef struct __noc_confinfo \
	NOCCONF_USER, *PNOCCONF_USER;
/* ----------------------noc user struct------------------------------ */

struct bst_noc_cdev{
    struct cdev cdev;
    unsigned char data;
};

#endif